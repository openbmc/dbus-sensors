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

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

using namespace std::literals;

// GPU Power Sensor Averaging Interval in seconds, 0 implies default
constexpr uint8_t gpuPowerAveragingIntervalInSec{0};

NvidiaGpuPowerPeakReading::NvidiaGpuPowerPeakReading(
    mctp::MctpRequester& mctpRequester, const std::string& name, uint8_t eid,
    uint8_t sensorId, sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorId{sensorId},
    averagingInterval{gpuPowerAveragingIntervalInSec},
    mctpRequester(mctpRequester), objectServer(objectServer)
{
    std::string dbusPath = sensorPathPrefix + "power/"s + escapeName(name);

    statisticsInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Telemetry.Report");

    std::get<0>(readings) = 0;
    // Reading from the device is in milliwatts and unit set on the dbus
    // is watts.
    std::get<1>(readings).emplace_back("PeakReading", "", 0.0, 0);

    statisticsInterface->register_property("Readings", readings);

    statisticsInterface->initialize();
}

NvidiaGpuPowerPeakReading::~NvidiaGpuPowerPeakReading()
{
    objectServer.remove_interface(statisticsInterface);
}

void NvidiaGpuPowerPeakReading::processResponse(int sendRecvMsgResult)
{
    if (sendRecvMsgResult != 0)
    {
        lg2::error(
            "Error updating Peak Power Sensor for eid {EID} and sensor id {SID} : sending message over MCTP failed, rc={RC}",
            "EID", eid, "SID", sensorId, "RC", sendRecvMsgResult);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t peakPower = 0;

    const int rc =
        gpu::decodeGetPowerDrawResponse(response, cc, reasonCode, peakPower);

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

    statisticsInterface->set_property("Readings", readings);
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
        eid, request, response,
        [this](int sendRecvMsgResult) { processResponse(sendRecvMsgResult); });
}
