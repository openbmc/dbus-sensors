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
    // Capacity utilization is a single measurement exposed at two Redfish
    // endpoints: the Processor MemorySummary and the DRAM MemoryMetrics. Each
    // endpoint follows the "measured_by" association from its own inventory
    // object, so publish a separate Metric.Value object per inventory object,
    // each carrying a single association. Both objects keep the
    // "memory_capacity_utilization" leaf name that bmcweb filters on.
    const sdbusplus::object_path processorInventoryPath =
        sdbusplus::object_path(inventoryPath) / deviceName;
    const sdbusplus::object_path dramInventoryPath(
        processorInventoryPath.str + dramInventorySuffix);

    // Metric associated with the GPU processor inventory object.
    const sdbusplus::object_path processorMetricObjectPath =
        sdbusplus::object_path(metricPath) / std::format("gpu_{}", deviceName) /
        "memory_capacity_utilization";

    processorMetricInterface = objectServer.add_interface(
        processorMetricObjectPath, "xyz.openbmc_project.Metric.Value");
    processorMetricInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Percent"s);
    processorMetricInterface->register_property("Value", 0.0);

    std::vector<Association> processorAssociations;
    processorAssociations.emplace_back("measuring", "measured_by",
                                       processorInventoryPath);

    processorMetricAssociationInterface = objectServer.add_interface(
        processorMetricObjectPath, association::interface);
    processorMetricAssociationInterface->register_property(
        "Associations", processorAssociations);

    // Metric associated with the GPU DRAM inventory object.
    const sdbusplus::object_path dramMetricObjectPath =
        sdbusplus::object_path(metricPath) /
        std::format("gpu_{}{}", deviceName, dramInventorySuffix) /
        "memory_capacity_utilization";

    dramMetricInterface = objectServer.add_interface(
        dramMetricObjectPath, "xyz.openbmc_project.Metric.Value");
    dramMetricInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Percent"s);
    dramMetricInterface->register_property("Value", 0.0);

    std::vector<Association> dramAssociations;
    dramAssociations.emplace_back("measuring", "measured_by",
                                  dramInventoryPath);

    dramMetricAssociationInterface = objectServer.add_interface(
        dramMetricObjectPath, association::interface);
    dramMetricAssociationInterface->register_property("Associations",
                                                      dramAssociations);

    if (!processorMetricInterface->initialize())
    {
        lg2::error(
            "Error initializing processor Memory Capacity Utilization metric interface for {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
    }

    if (!processorMetricAssociationInterface->initialize())
    {
        lg2::error(
            "Error initializing processor Memory Capacity Utilization metric association interface for {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
    }

    if (!dramMetricInterface->initialize())
    {
        lg2::error(
            "Error initializing DRAM Memory Capacity Utilization metric interface for {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
    }

    if (!dramMetricAssociationInterface->initialize())
    {
        lg2::error(
            "Error initializing DRAM Memory Capacity Utilization metric association interface for {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
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
                processorMetricInterface, dramMetricInterface, this->inventory,
                eid),
            .onLongRunningPayload = std::bind_front(
                &NvidiaGpuMemoryCapacityUtilization::onLongRunningPayload,
                processorMetricInterface, dramMetricInterface, this->inventory,
                eid),
        });
}

void NvidiaGpuMemoryCapacityUtilization::update()
{
    cmd->update();
}

void NvidiaGpuMemoryCapacityUtilization::onImmediateSuccess(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        processorMetricInterface,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& dramMetricInterface,
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

    applyUtilization(processorMetricInterface, dramMetricInterface, inventory,
                     reservedMemory, usedMemory);
}

void NvidiaGpuMemoryCapacityUtilization::onLongRunningPayload(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        processorMetricInterface,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& dramMetricInterface,
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

    applyUtilization(processorMetricInterface, dramMetricInterface, inventory,
                     reservedMemory, usedMemory);
}

void NvidiaGpuMemoryCapacityUtilization::applyUtilization(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        processorMetricInterface,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& dramMetricInterface,
    const std::shared_ptr<Inventory>& inventory, uint32_t reservedMemory,
    uint32_t usedMemory)
{
    if (!inventory)
    {
        return;
    }

    const uint32_t maxMemoryMiB = inventory->getMaxMemoryMiB().value_or(0);
    if (maxMemoryMiB == 0)
    {
        return;
    }

    const uint64_t total = static_cast<uint64_t>(reservedMemory) +
                           static_cast<uint64_t>(usedMemory);

    double percent =
        static_cast<double>(total) * 100.0 / static_cast<double>(maxMemoryMiB);

    percent = std::clamp(percent, 0.0, 100.0);

    processorMetricInterface->set_property("Value", percent);
    dramMetricInterface->set_property("Value", percent);
}
