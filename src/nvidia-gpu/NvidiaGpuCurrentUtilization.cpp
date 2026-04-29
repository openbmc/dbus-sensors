/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuCurrentUtilization.hpp"

#include <MctpRequester.hpp>
#include <MessagePackUnpackUtils.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <NvidiaUtils.hpp>
#include <OcpMctpVdm.hpp>
#include <Utils.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

using namespace std::literals;

NvidiaGpuCurrentUtilization::NvidiaGpuCurrentUtilization(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    uint8_t eid,
    const std::shared_ptr<NvidiaLongRunningResponseHandler>&
        longRunningResponseHandler) :
    eid(eid), conn(conn), mctpRequester(mctpRequester),
    longRunningResponseHandler(longRunningResponseHandler)
{
    const sdbusplus::object_path metricObjectPath =
        sdbusplus::object_path(metricPath) / std::format("gpu_{}", deviceName) /
        "processor_bandwidth";

    metricInterface = objectServer.add_interface(
        metricObjectPath, "xyz.openbmc_project.Metric.Value");

    metricInterface->register_property(
        "Unit", std::string("xyz.openbmc_project.Metric.Value.Unit.Percent"));
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
            "Failed to initialize Current Utilization metric interface for GPU {NAME}",
            "NAME", deviceName);
    }

    if (!metricAssociationInterface->initialize())
    {
        lg2::error(
            "Failed to initialize Current Utilization metric association interface for GPU {NAME}",
            "NAME", deviceName);
    }
}

void NvidiaGpuCurrentUtilization::update()
{
    const int rc = gpu::encodeGetCurrentUtilizationModeRequest(0, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating GPU Current Utilization: encode failed, rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuCurrentUtilization> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaGpuCurrentUtilization");
                return;
            }
            self->processResponse(ec, buffer);
        });
}

void NvidiaGpuCurrentUtilization::processResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating GPU Current Utilization: sending message over MCTP failed, "
            "rc={RC}, EID={EID}",
            "RC", ec.message(), "EID", eid);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    int rc = ocp::accelerator_management::decodeReasonCodeAndCC(
        buffer, cc, reasonCode);

    if (rc != 0)
    {
        lg2::error("Error updating GPU Current Utilization: decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                   "EID", eid);
        return;
    }

    switch (cc)
    {
        case ocp::accelerator_management::CompletionCode::SUCCESS:
        {
            uint32_t utilization = 0;
            uint32_t memoryUtilization = 0;
            rc = gpu::decodeGetCurrentUtilizationModeResponse(
                buffer, cc, reasonCode, utilization, memoryUtilization);

            if (rc != 0)
            {
                lg2::error(
                    "Error updating GPU Current Utilization: decode failed, "
                    "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                    "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
                    reasonCode, "EID", eid);

                return;
            }

            updateUtilization(utilization);

            return;
        }

        case ocp::accelerator_management::CompletionCode::ACCEPTED:
        {
            rc = longRunningResponseHandler->registerResponseHandler(
                gpu::MessageType::PLATFORM_ENVIRONMENTAL,
                static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                         GET_CURRENT_UTILIZATION),
                [weak{weak_from_this()}](
                    ocp::accelerator_management::CompletionCode cc,
                    uint16_t reasonCode,
                    std::span<const uint8_t> responseData) {
                    std::shared_ptr<NvidiaGpuCurrentUtilization> self =
                        weak.lock();
                    if (!self)
                    {
                        lg2::error(
                            "Invalid reference to NvidiaGpuCurrentUtilization");
                        return;
                    }

                    self->processLongRunningResponse(cc, reasonCode,
                                                     responseData);
                });

            if (rc != 0)
            {
                lg2::error(
                    "Error updating GPU Current Utilization: failed to register long running response handler, "
                    "rc={RC}, EID={EID}",
                    "RC", rc, "EID", eid);
            }

            return;
        }

        default:
            lg2::error(
                "Error updating GPU Current Utilization: received unexpected completion code, "
                "cc={CC}, reasonCode={RESC}, EID={EID}",
                "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID", eid);
            return;
    }
}

void NvidiaGpuCurrentUtilization::processLongRunningResponse(
    ocp::accelerator_management::CompletionCode cc, uint16_t reasonCode,
    std::span<const uint8_t> responseData)
{
    if (cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating GPU Current Utilization: long running response indicated failure, "
            "cc={CC}, reasonCode={RESC}, EID={EID}",
            "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID", eid);
        return;
    }

    if (responseData.size() < 8)
    {
        lg2::error(
            "Error updating GPU Current Utilization: invalid long running response data size, "
            "size={SIZE}, EID={EID}",
            "SIZE", responseData.size(), "EID", eid);
        return;
    }

    uint32_t utilization = 0;

    UnpackBuffer buffer(responseData);

    const int rc = buffer.unpack(utilization);

    if (rc != 0)
    {
        lg2::error(
            "Error updating GPU Current Utilization: failed to unpack long running response data, "
            "rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        return;
    }

    updateUtilization(utilization);
}

void NvidiaGpuCurrentUtilization::updateUtilization(uint32_t utilization)
{
    metricInterface->set_property("Value", static_cast<double>(utilization));
}
