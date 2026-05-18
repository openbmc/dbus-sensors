/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMemoryCapacityUtilization.hpp"

#include <bits/basic_string.h>

#include <Inventory.hpp>
#include <MctpRequester.hpp>
#include <MessagePackUnpackUtils.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <NvidiaUtils.hpp>
#include <OcpMctpVdm.hpp>
#include <SerialQueue.hpp>
#include <Utils.hpp>
#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

using namespace std::literals;

NvidiaGpuMemoryCapacityUtilization::NvidiaGpuMemoryCapacityUtilization(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    uint8_t eid, std::shared_ptr<SerialQueue> longRunningQueue,
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler,
    std::shared_ptr<Inventory> inventory) :
    eid(eid), conn(conn), mctpRequester(mctpRequester),
    longRunningQueue(std::move(longRunningQueue)),
    longRunningResponseHandler(std::move(longRunningResponseHandler)),
    inventory(std::move(inventory))
{
    const sdbusplus::object_path metricObjectPath =
        sdbusplus::object_path(metricPath) / std::format("gpu_{}", deviceName) /
        "memory" / "capacity_utilization";

    metricInterface = objectServer.add_interface(
        metricObjectPath, "xyz.openbmc_project.Metric.Value");

    metricInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Percent"s);
    metricInterface->register_property("Value", 0.0);

    std::vector<Association> associations;

    const sdbusplus::object_path processorObjectPath =
        sdbusplus::object_path(inventoryPath) / deviceName;

    associations.emplace_back("measuring", "measured_by", processorObjectPath);

    metricAssociationInterface =
        objectServer.add_interface(metricObjectPath, association::interface);

    metricAssociationInterface->register_property("Associations", associations);

    if (!metricInterface->initialize())
    {
        lg2::error(
            "Failed to initialize Memory Capacity Utilization metric interface for GPU {NAME}",
            "NAME", deviceName);
    }

    if (!metricAssociationInterface->initialize())
    {
        lg2::error(
            "Failed to initialize Memory Capacity Utilization metric association interface for GPU {NAME}",
            "NAME", deviceName);
    }
}

void NvidiaGpuMemoryCapacityUtilization::update()
{
    longRunningQueue->submit([weak{weak_from_this()}](
                                 SerialQueue::ReleaseHandle handle) {
        std::shared_ptr<NvidiaGpuMemoryCapacityUtilization> self = weak.lock();
        if (!self)
        {
            return;
        }
        self->doUpdate(std::move(handle));
    });
}

void NvidiaGpuMemoryCapacityUtilization::doUpdate(
    SerialQueue::ReleaseHandle handle)
{
    if (!inventory->getMaxMemoryMiB().has_value())
    {
        // Inventory has not yet fetched MAX_MEMORY_CAPACITY; skip this cycle
        // and retry on the next polling round.
        return;
    }

    const int rc = gpu::encodeGetMemoryCapacityUtilizationRequest(0, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating GPU Memory Capacity Utilization: encode failed, rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()},
         handle = std::move(handle)](const std::error_code& ec,
                                     std::span<const uint8_t> buffer) mutable {
            std::shared_ptr<NvidiaGpuMemoryCapacityUtilization> self =
                weak.lock();
            if (!self)
            {
                return;
            }
            self->processResponse(std::move(handle), ec, buffer);
        });
}

void NvidiaGpuMemoryCapacityUtilization::processResponse(
    SerialQueue::ReleaseHandle handle, const std::error_code& ec,
    std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating GPU Memory Capacity Utilization: sending message over MCTP failed, "
            "rc={RC}, EID={EID}",
            "RC", ec.message(), "EID", eid);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    UnpackBuffer unpackBuffer(buffer);

    int rc = gpu::decodeResponseCommonHeader(
        unpackBuffer, gpu::MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                 GET_MEMORY_CAPACITY_UTILIZATION),
        cc, reasonCode);

    if (rc != 0)
    {
        lg2::error(
            "Error updating GPU Memory Capacity Utilization: decode failed, "
            "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid);
        return;
    }

    switch (cc)
    {
        case ocp::accelerator_management::CompletionCode::SUCCESS:
        {
            uint32_t reservedMemory = 0;
            uint32_t usedMemory = 0;
            rc = gpu::decodeGetMemoryCapacityUtilizationResponse(
                buffer, cc, reasonCode, reservedMemory, usedMemory);

            if (rc != 0)
            {
                lg2::error(
                    "Error updating GPU Memory Capacity Utilization: decode failed, "
                    "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                    "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
                    reasonCode, "EID", eid);
                return;
            }

            updateUtilization(reservedMemory, usedMemory);
            return;
        }

        case ocp::accelerator_management::CompletionCode::ACCEPTED:
        {
            uint8_t instanceId = 0;
            rc = ocp::accelerator_management::decodeInstanceId(buffer,
                                                               instanceId);
            if (rc != 0)
            {
                lg2::error(
                    "Error updating GPU Memory Capacity Utilization: failed to decode instance id, "
                    "rc={RC}, EID={EID}",
                    "RC", rc, "EID", eid);
                return;
            }

            rc = longRunningResponseHandler->registerResponseHandler(
                gpu::MessageType::PLATFORM_ENVIRONMENTAL,
                static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                         GET_MEMORY_CAPACITY_UTILIZATION),
                instanceId,
                [weak{weak_from_this()}, handle = std::move(handle)](
                    boost::system::error_code longRunningEc,
                    ocp::accelerator_management::CompletionCode longRunningCc,
                    uint16_t longRunningReasonCode,
                    std::span<const uint8_t> responseData) mutable {
                    std::shared_ptr<NvidiaGpuMemoryCapacityUtilization> self =
                        weak.lock();
                    if (!self)
                    {
                        return;
                    }

                    self->processLongRunningResponse(
                        longRunningEc, longRunningCc, longRunningReasonCode,
                        responseData);
                });

            if (rc != 0)
            {
                lg2::error(
                    "Error updating GPU Memory Capacity Utilization: failed to register long running response handler, "
                    "rc={RC}, EID={EID}",
                    "RC", rc, "EID", eid);
            }

            return;
        }

        default:
            lg2::error(
                "Error updating GPU Memory Capacity Utilization: received unexpected completion code, "
                "cc={CC}, reasonCode={RESC}, EID={EID}",
                "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID", eid);
            return;
    }
}

void NvidiaGpuMemoryCapacityUtilization::processLongRunningResponse(
    boost::system::error_code ec,
    ocp::accelerator_management::CompletionCode cc, uint16_t reasonCode,
    std::span<const uint8_t> responseData)
{
    if (ec)
    {
        lg2::error(
            "Error updating GPU Memory Capacity Utilization: long running response failed, "
            "rc={RC}, EID={EID}",
            "RC", ec.message(), "EID", eid);
        return;
    }

    if (cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating GPU Memory Capacity Utilization: long running response indicated failure, "
            "cc={CC}, reasonCode={RESC}, EID={EID}",
            "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID", eid);
        return;
    }

    if (responseData.size() < 8)
    {
        lg2::error(
            "Error updating GPU Memory Capacity Utilization: invalid long running response data size, "
            "size={SIZE}, EID={EID}",
            "SIZE", responseData.size(), "EID", eid);
        return;
    }

    uint32_t reservedMemory = 0;
    uint32_t usedMemory = 0;

    UnpackBuffer buffer(responseData);

    int rc = buffer.unpack(reservedMemory);
    if (rc == 0)
    {
        rc = buffer.unpack(usedMemory);
    }

    if (rc != 0)
    {
        lg2::error(
            "Error updating GPU Memory Capacity Utilization: failed to unpack long running response data, "
            "rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        return;
    }

    updateUtilization(reservedMemory, usedMemory);
}

void NvidiaGpuMemoryCapacityUtilization::updateUtilization(
    uint32_t reservedMemory, uint32_t usedMemory)
{
    const uint32_t maxMemoryMiB = inventory->getMaxMemoryMiB().value_or(0);
    if (maxMemoryMiB == 0)
    {
        return;
    }

    const uint64_t total = static_cast<uint64_t>(reservedMemory) +
                           static_cast<uint64_t>(usedMemory);
    double percent =
        static_cast<double>(total * 100) / static_cast<double>(maxMemoryMiB);

    percent = std::clamp(percent, 0.0, 100.0);

    metricInterface->set_property("Value", percent);
}
