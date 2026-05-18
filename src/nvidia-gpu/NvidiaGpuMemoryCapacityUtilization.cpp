/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMemoryCapacityUtilization.hpp"

#include <bits/basic_string.h>

#include <Inventory.hpp>
#include <MctpRequester.hpp>
#include <NvidiaGpuLongRunningCommand.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <NvidiaUtils.hpp>
#include <OcpMctpVdm.hpp>
#include <SerialQueue.hpp>
#include <Utils.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

using namespace std::literals;

NvidiaGpuMemoryCapacityUtilization::NvidiaGpuMemoryCapacityUtilization(
    mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    uint8_t eid, std::shared_ptr<SerialQueue> longRunningQueue,
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler,
    std::shared_ptr<Inventory> inventory) : inventory(std::move(inventory))
{
    const sdbusplus::object_path metricObjectPath =
        sdbusplus::object_path(metricPath) / std::format("gpu_{}", deviceName) /
        "memory_capacity_utilization";

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

    cmd = std::make_shared<NvidiaGpuLongRunningCommand>(
        eid, mctpRequester, std::move(longRunningQueue),
        std::move(longRunningResponseHandler),
        NvidiaGpuLongRunningCommand::Config{
            .metricName = "GPU Memory Capacity Utilization",
            .messageType = gpu::MessageType::PLATFORM_ENVIRONMENTAL,
            .commandId = gpu::PlatformEnvironmentalCommands::
                GET_MEMORY_CAPACITY_UTILIZATION,
            .requestSize = gpu::getMemoryCapacityUtilizationRequestSize,
            .encodeRequest = std::bind_front(
                &gpu::encodeGetMemoryCapacityUtilizationRequest, uint8_t{0}),
            .onImmediateSuccess = std::bind_front(
                &NvidiaGpuMemoryCapacityUtilization::onImmediateSuccess,
                metricInterface, this->inventory, eid),
            .onLongRunningPayload = std::bind_front(
                &NvidiaGpuMemoryCapacityUtilization::onLongRunningPayload,
                metricInterface, this->inventory, eid),
        });
}

void NvidiaGpuMemoryCapacityUtilization::update()
{
    if (!inventory->getMaxMemoryMiB().has_value())
    {
        // The maximum memory capacity is the denominator for the utilization
        // percentage. Skip this cycle until Inventory has fetched it; retry on
        // the next polling round.
        return;
    }

    cmd->update();
}

void NvidiaGpuMemoryCapacityUtilization::onImmediateSuccess(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& metricInterface,
    const std::shared_ptr<Inventory>& inventory, uint8_t eid,
    std::span<const uint8_t> buffer)
{
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t reservedMemory = 0;
    uint32_t usedMemory = 0;

    const int rc = gpu::decodeGetMemoryCapacityUtilizationResponse(
        buffer, cc, reasonCode, reservedMemory, usedMemory);

    if (rc != 0)
    {
        lg2::error(
            "Error updating GPU Memory Capacity Utilization: decode failed, "
            "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid);
        return;
    }

    applyUtilization(metricInterface, inventory, reservedMemory, usedMemory);
}

void NvidiaGpuMemoryCapacityUtilization::onLongRunningPayload(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& metricInterface,
    const std::shared_ptr<Inventory>& inventory, uint8_t eid,
    std::span<const uint8_t> payload)
{
    uint32_t reservedMemory = 0;
    uint32_t usedMemory = 0;

    const int rc = gpu::decodeGetMemoryCapacityUtilizationResponse(
        payload, reservedMemory, usedMemory);

    if (rc != 0)
    {
        lg2::error(
            "Error updating GPU Memory Capacity Utilization: failed to decode "
            "long running response data, rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        return;
    }

    applyUtilization(metricInterface, inventory, reservedMemory, usedMemory);
}

void NvidiaGpuMemoryCapacityUtilization::applyUtilization(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& metricInterface,
    const std::shared_ptr<Inventory>& inventory, uint32_t reservedMemory,
    uint32_t usedMemory)
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
