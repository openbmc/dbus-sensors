/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuCurrentUtilization.hpp"

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

NvidiaGpuCurrentUtilization::NvidiaGpuCurrentUtilization(
    mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    uint8_t eid, std::shared_ptr<SerialQueue> longRunningQueue,
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler)
{
    const sdbusplus::object_path metricObjectPath =
        sdbusplus::object_path(metricPath) / std::format("gpu_{}", deviceName) /
        "processor_bandwidth";

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
            "Failed to initialize Current Utilization metric interface for GPU {NAME}",
            "NAME", deviceName);
    }

    if (!metricAssociationInterface->initialize())
    {
        lg2::error(
            "Failed to initialize Current Utilization metric association interface for GPU {NAME}",
            "NAME", deviceName);
    }

    cmd = std::make_shared<NvidiaGpuLongRunningCommand>(
        eid, mctpRequester, std::move(longRunningQueue),
        std::move(longRunningResponseHandler),
        NvidiaGpuLongRunningCommand::Config{
            .metricName = "GPU Current Utilization",
            .messageType = gpu::MessageType::PLATFORM_ENVIRONMENTAL,
            .commandId =
                gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION,
            .requestSize = gpu::getCurrentUtilizationRequestSize,
            .encodeRequest =
                [](std::span<uint8_t> buf) {
                    return gpu::encodeGetCurrentUtilizationModeRequest(0, buf);
                },
            .onImmediateSuccess = std::bind_front(
                &NvidiaGpuCurrentUtilization::onImmediateSuccess,
                metricInterface, eid),
            .onLongRunningPayload = std::bind_front(
                &NvidiaGpuCurrentUtilization::onLongRunningPayload,
                metricInterface, eid),
        });
}

void NvidiaGpuCurrentUtilization::update()
{
    cmd->update();
}

void NvidiaGpuCurrentUtilization::onImmediateSuccess(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& metricInterface,
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
        lg2::error("Error updating GPU Current Utilization: decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                   "EID", eid);
        return;
    }

    applyUtilization(metricInterface, utilization);
}

void NvidiaGpuCurrentUtilization::onLongRunningPayload(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& metricInterface,
    uint8_t eid, std::span<const uint8_t> payload)
{
    uint32_t utilization = 0;
    const int rc =
        gpu::decodeGetCurrentUtilizationModeResponse(payload, utilization);

    if (rc != 0)
    {
        lg2::error("Error updating GPU Current Utilization: failed to decode "
                   "long running response data, rc={RC}, EID={EID}",
                   "RC", rc, "EID", eid);
        return;
    }

    applyUtilization(metricInterface, utilization);
}

void NvidiaGpuCurrentUtilization::applyUtilization(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& metricInterface,
    uint32_t utilization)
{
    metricInterface->set_property("Value", static_cast<double>(utilization));
}
