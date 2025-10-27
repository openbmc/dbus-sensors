/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuPowerPeakReading.hpp"

#include "MctpRequester.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <NvidiaDeviceDiscovery.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <vector>

using namespace std::literals;

NvidiaGpuPowerPeakReading::NvidiaGpuPowerPeakReading(
    mctp::MctpRequester& mctpRequester, const std::string& name, uint8_t eid,
    uint8_t sensorId, sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorId{sensorId}, mctpRequester(mctpRequester),
    objectServer(objectServer)
{
    std::string dbusPath = sensorPathPrefix + "power/"s + escapeName(name);

    telemetryReportInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Telemetry.Report");

    telemetryReportInterface->register_property("Persistency", false);
    std::vector<std::tuple<
        std::vector<std::tuple<sdbusplus::message::object_path, std::string>>,
        std::string, std::string, uint64_t>>
        readingParams{
            {{{dbusPath, ""}},
             "xyz.openbmc_project.Telemetry.Report.OperationType.Maximum",
             "",
             0}};
    telemetryReportInterface->register_property("ReadingParameters",
                                                readingParams);
    std::get<0>(readings) = 0;
    // Reading from the device is in milliwatts and unit set on the dbus
    // is watts.
    std::get<1>(readings).emplace_back("PeakReading", "", 0.0, 0);
    telemetryReportInterface->register_property("Readings", readings);
    telemetryReportInterface->register_property("Enabled", true);

    telemetryReportInterface->initialize();
}

NvidiaGpuPowerPeakReading::~NvidiaGpuPowerPeakReading()
{
    objectServer.remove_interface(telemetryReportInterface);
}

void NvidiaGpuPowerPeakReading::processResponse(const std::error_code& ec,
                                                std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating Peak Power Sensor for eid {EID} and sensor id {SID} : sending message over MCTP failed, rc={RC}",
            "EID", eid, "SID", sensorId, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t peakPower = 0;

    const int rc =
        gpu::decodeGetPowerDrawResponse(buffer, cc, reasonCode, peakPower);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Peak Power Sensor eid {EID} and sensor id {SID} : decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "SID", sensorId, "RC", rc, "CC", cc, "RESC",
            reasonCode);
        return;
    }

    // Reading from the device is in milliwatts and unit set on the dbus
    // is watts.
    std::get<2>(std::get<1>(readings)[0]) = peakPower / 1000.0;

    telemetryReportInterface->set_property("Readings", readings);
}

void NvidiaGpuPowerPeakReading::update()
{
    const int rc = gpu::encodeGetPowerDrawRequest(
        gpu::PlatformEnvironmentalCommands::GET_MAX_OBSERVED_POWER, 0, sensorId,
        averagingInterval, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating Peak Power Sensor for eid {EID} and sensor id {SID} : encode failed, rc={RC}",
            "EID", eid, "SID", sensorId, "RC", rc);
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [this](const std::error_code& ec, std::span<const uint8_t> buffer) {
            processResponse(ec, buffer);
        });
}
