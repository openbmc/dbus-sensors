/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuUtilizationMetrics.hpp"

#include <bits/basic_string.h>

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

#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

using namespace std::literals;

NvidiaGpuUtilizationMetrics::NvidiaGpuUtilizationMetrics(
    mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    uint8_t eid, std::shared_ptr<SerialQueue> longRunningQueue,
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler)
{
    const sdbusplus::object_path metricRoot =
        sdbusplus::object_path(metricPath) / std::format("gpu_{}", deviceName);

    const sdbusplus::object_path processorMetricObjectPath =
        metricRoot / "processor_bandwidth";

    processorMetricInterface = objectServer.add_interface(
        processorMetricObjectPath, "xyz.openbmc_project.Metric.Value");

    processorMetricInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Percent"s);
    processorMetricInterface->register_property("Value", 0.0);

    const sdbusplus::object_path processorInventoryPath =
        sdbusplus::object_path(inventoryPath) / deviceName;

    std::vector<Association> processorAssociations;
    processorAssociations.emplace_back("measuring", "measured_by",
                                       processorInventoryPath);

    processorMetricAssociationInterface = objectServer.add_interface(
        processorMetricObjectPath, association::interface);
    processorMetricAssociationInterface->register_property(
        "Associations", processorAssociations);

    if (!processorMetricInterface->initialize())
    {
        lg2::error(
            "Error initializing Processor Bandwidth metric interface for GPU {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
    }

    if (!processorMetricAssociationInterface->initialize())
    {
        lg2::error(
            "Error initializing Processor Bandwidth metric association interface for GPU {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
    }

    const sdbusplus::object_path memoryMetricObjectPath =
        metricRoot / "memory_bandwidth";

    memoryMetricInterface = objectServer.add_interface(
        memoryMetricObjectPath, "xyz.openbmc_project.Metric.Value");

    memoryMetricInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Percent"s);
    memoryMetricInterface->register_property("Value", 0.0);

    const sdbusplus::object_path memoryInventoryPath(
        processorInventoryPath.str + dramInventorySuffix);

    std::vector<Association> memoryAssociations;
    memoryAssociations.emplace_back("measuring", "measured_by",
                                    memoryInventoryPath);

    memoryMetricAssociationInterface = objectServer.add_interface(
        memoryMetricObjectPath, association::interface);
    memoryMetricAssociationInterface->register_property("Associations",
                                                        memoryAssociations);

    if (!memoryMetricInterface->initialize())
    {
        lg2::error(
            "Error initializing Memory Bandwidth metric interface for GPU {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
    }

    if (!memoryMetricAssociationInterface->initialize())
    {
        lg2::error(
            "Error initializing Memory Bandwidth metric association interface for GPU {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
    }

    cmd = std::make_shared<NvidiaGpuLongRunningCommand>(
        eid, mctpRequester, std::move(longRunningQueue),
        std::move(longRunningResponseHandler),
        NvidiaGpuLongRunningCommand::Config{
            .metricName = "GPU Utilization Metrics",
            .messageType = gpu::MessageType::PLATFORM_ENVIRONMENTAL,
            .commandId =
                gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION,
            .requestSize = gpu::getCurrentUtilizationRequestSize,
            .encodeRequest = std::bind_front(
                &gpu::encodeGetCurrentUtilizationModeRequest, uint8_t{0}),
            .onImmediateSuccess = std::bind_front(
                &NvidiaGpuUtilizationMetrics::onImmediateSuccess,
                processorMetricInterface, memoryMetricInterface, eid),
            .onLongRunningPayload = std::bind_front(
                &NvidiaGpuUtilizationMetrics::onLongRunningPayload,
                processorMetricInterface, memoryMetricInterface, eid),
        });
}

void NvidiaGpuUtilizationMetrics::update()
{
    cmd->update();
}

void NvidiaGpuUtilizationMetrics::onImmediateSuccess(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        processorMetricInterface,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        memoryMetricInterface,
    uint8_t eid, std::span<const uint8_t> buffer)
{
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t utilization = 0;
    uint32_t memoryUtilization = 0;

    const int rc = gpu::decodeGetCurrentUtilizationModeResponse(
        buffer, cc, reasonCode, utilization, memoryUtilization);

    if (rc != 0)
    {
        lg2::error("Error updating GPU Utilization Metrics: decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                   "EID", eid);
        return;
    }

    applyUtilization(processorMetricInterface, utilization);
    applyUtilization(memoryMetricInterface, memoryUtilization);
}

void NvidiaGpuUtilizationMetrics::onLongRunningPayload(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        processorMetricInterface,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        memoryMetricInterface,
    uint8_t eid, std::span<const uint8_t> payload)
{
    uint32_t utilization = 0;
    uint32_t memoryUtilization = 0;

    const int rc = gpu::decodeGetCurrentUtilizationModeResponse(
        payload, utilization, memoryUtilization);

    if (rc != 0)
    {
        lg2::error("Error updating GPU Utilization Metrics: failed to decode "
                   "long running response data, rc={RC}, EID={EID}",
                   "RC", rc, "EID", eid);
        return;
    }

    applyUtilization(processorMetricInterface, utilization);
    applyUtilization(memoryMetricInterface, memoryUtilization);
}

void NvidiaGpuUtilizationMetrics::applyUtilization(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& metricInterface,
    uint32_t utilization)
{
    metricInterface->set_property("Value", static_cast<double>(utilization));
}
