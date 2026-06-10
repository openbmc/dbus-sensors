/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuViolationDuration.hpp"

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
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

using namespace std::literals;

namespace
{

constexpr double nanosecondsPerSecond = 1'000'000'000.0;

std::shared_ptr<sdbusplus::asio::dbus_interface> createMetricInterface(
    sdbusplus::asio::object_server& objectServer,
    const sdbusplus::object_path& metricObjectPath)
{
    auto metricInterface = objectServer.add_interface(
        metricObjectPath, "xyz.openbmc_project.Metric.Value");

    metricInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Seconds"s);
    metricInterface->register_property("Value", 0.0);

    return metricInterface;
}

std::shared_ptr<sdbusplus::asio::dbus_interface> createAssociationInterface(
    sdbusplus::asio::object_server& objectServer,
    const sdbusplus::object_path& metricObjectPath,
    const sdbusplus::object_path& processorObjectPath)
{
    std::vector<Association> associations;
    associations.emplace_back("measuring", "measured_by", processorObjectPath);

    auto associationInterface =
        objectServer.add_interface(metricObjectPath, association::interface);

    associationInterface->register_property("Associations", associations);

    return associationInterface;
}

void initializeInterface(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& iface,
    const std::string& deviceName, uint8_t eid, const std::string& label)
{
    if (!iface->initialize())
    {
        lg2::error("Error initializing {LABEL} for GPU {NAME}, eid={EID}",
                   "LABEL", label, "NAME", deviceName, "EID", eid);
    }
}

} // namespace

NvidiaGpuViolationDuration::NvidiaGpuViolationDuration(
    mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    uint8_t eid, std::shared_ptr<SerialQueue> longRunningQueue,
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler)
{
    const sdbusplus::object_path gpuMetricPath =
        sdbusplus::object_path(metricPath) / std::format("gpu_{}", deviceName);
    const sdbusplus::object_path powerObjectPath =
        gpuMetricPath / "power_limit_throttle_duration";
    const sdbusplus::object_path thermalObjectPath =
        gpuMetricPath / "thermal_limit_throttle_duration";

    const sdbusplus::object_path processorObjectPath =
        sdbusplus::object_path(inventoryPath) / deviceName;

    powerMetricInterface = createMetricInterface(objectServer, powerObjectPath);
    powerMetricAssociationInterface = createAssociationInterface(
        objectServer, powerObjectPath, processorObjectPath);

    thermalMetricInterface =
        createMetricInterface(objectServer, thermalObjectPath);
    thermalMetricAssociationInterface = createAssociationInterface(
        objectServer, thermalObjectPath, processorObjectPath);

    initializeInterface(powerMetricInterface, deviceName, eid,
                        "Power Limit Throttle Duration metric interface");
    initializeInterface(
        powerMetricAssociationInterface, deviceName, eid,
        "Power Limit Throttle Duration metric association interface");
    initializeInterface(thermalMetricInterface, deviceName, eid,
                        "Thermal Limit Throttle Duration metric interface");
    initializeInterface(
        thermalMetricAssociationInterface, deviceName, eid,
        "Thermal Limit Throttle Duration metric association interface");

    cmd = std::make_shared<NvidiaGpuLongRunningCommand>(
        eid, mctpRequester, std::move(longRunningQueue),
        std::move(longRunningResponseHandler),
        NvidiaGpuLongRunningCommand::Config{
            .metricName = "GPU Violation Duration",
            .messageType = gpu::MessageType::PLATFORM_ENVIRONMENTAL,
            .commandId =
                gpu::PlatformEnvironmentalCommands::GET_VIOLATION_DURATION,
            .requestSize = gpu::getViolationDurationRequestSize,
            .encodeRequest = std::bind_front(
                &gpu::encodeGetViolationDurationRequest, uint8_t{0}),
            .onImmediateSuccess = std::bind_front(
                &NvidiaGpuViolationDuration::onImmediateSuccess,
                powerMetricInterface, thermalMetricInterface, eid),
            .onLongRunningPayload = std::bind_front(
                &NvidiaGpuViolationDuration::onLongRunningPayload,
                powerMetricInterface, thermalMetricInterface, eid),
        });
}

void NvidiaGpuViolationDuration::update()
{
    cmd->update();
}

void NvidiaGpuViolationDuration::onImmediateSuccess(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        powerMetricInterface,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        thermalMetricInterface,
    uint8_t eid, std::span<const uint8_t> buffer)
{
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint64_t hwViolationDuration = 0;
    uint64_t globalSwViolationDuration = 0;
    uint64_t powerViolationDuration = 0;
    uint64_t thermalViolationDuration = 0;

    const int rc = gpu::decodeGetViolationDurationResponse(
        buffer, cc, reasonCode, hwViolationDuration, globalSwViolationDuration,
        powerViolationDuration, thermalViolationDuration);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("Error updating GPU Violation Duration: decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                   "EID", eid);
        powerMetricInterface->set_property(
            "Value", std::numeric_limits<double>::quiet_NaN());
        thermalMetricInterface->set_property(
            "Value", std::numeric_limits<double>::quiet_NaN());
        return;
    }

    applyDurations(powerMetricInterface, thermalMetricInterface,
                   powerViolationDuration, thermalViolationDuration);
}

void NvidiaGpuViolationDuration::onLongRunningPayload(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        powerMetricInterface,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        thermalMetricInterface,
    uint8_t eid, std::span<const uint8_t> payload)
{
    uint64_t hwViolationDuration = 0;
    uint64_t globalSwViolationDuration = 0;
    uint64_t powerViolationDuration = 0;
    uint64_t thermalViolationDuration = 0;

    const int rc = gpu::decodeGetViolationDurationResponse(
        payload, hwViolationDuration, globalSwViolationDuration,
        powerViolationDuration, thermalViolationDuration);

    if (rc != 0)
    {
        lg2::error("Error updating GPU Violation Duration: failed to decode "
                   "long running response data, rc={RC}, EID={EID}",
                   "RC", rc, "EID", eid);
        powerMetricInterface->set_property(
            "Value", std::numeric_limits<double>::quiet_NaN());
        thermalMetricInterface->set_property(
            "Value", std::numeric_limits<double>::quiet_NaN());
        return;
    }

    applyDurations(powerMetricInterface, thermalMetricInterface,
                   powerViolationDuration, thermalViolationDuration);
}

void NvidiaGpuViolationDuration::applyDurations(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        powerMetricInterface,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        thermalMetricInterface,
    uint64_t powerViolationNs, uint64_t thermalViolationNs)
{
    powerMetricInterface->set_property(
        "Value", static_cast<double>(powerViolationNs) / nanosecondsPerSecond);
    thermalMetricInterface->set_property(
        "Value",
        static_cast<double>(thermalViolationNs) / nanosecondsPerSecond);
}
