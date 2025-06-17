/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaDeviceDiscovery.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace std::literals;

static constexpr double gpuTempSensorMaxReading = 127;
static constexpr double gpuTempSensorMinReading = -128;

NvidiaGpuTempSensor::NvidiaGpuTempSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, const uint8_t eid, uint8_t sensorId,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData) :
    Sensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
           "temperature", false, true, gpuTempSensorMaxReading,
           gpuTempSensorMinReading, conn),
    eid(eid), sensorId{sensorId}, mctpRequester(mctpRequester),
    objectServer(objectServer)
{
    std::string dbusPath =
        sensorPathPrefix + "temperature/"s + escapeName(name);

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(dbusPath, interface);
    }

    association = objectServer.add_interface(dbusPath, association::interface);

    setInitialProperties(sensor_paths::unitDegreesC);
}

NvidiaGpuTempSensor::~NvidiaGpuTempSensor()
{
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(association);
    objectServer.remove_interface(sensorInterface);
}

void NvidiaGpuTempSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void NvidiaGpuTempSensor::processResponse(int sendRecvMsgResult)
{
    if (sendRecvMsgResult != 0)
    {
        lg2::error(
            "Error updating Temperature Sensor for eid {EID} and sensor id {SID} : sending message over MCTP failed, rc={RC}",
            "EID", eid, "SID", sensorId, "RC", sendRecvMsgResult);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    double tempValue = 0;

    auto rc = gpu::decodeGetTemperatureReadingResponse(
        getTemperatureReadingResponse, cc, reasonCode, tempValue);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Temperature Sensor for eid {EID} and sensor id {SID} : decode failed. "
            "rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "SID", sensorId, "RC", rc, "CC", cc, "RESC",
            reasonCode);
        return;
    }

    updateValue(tempValue);
}

void NvidiaGpuTempSensor::update()
{
    auto rc = gpu::encodeGetTemperatureReadingRequest(
        0, sensorId, getTemperatureReadingRequest);

    if (rc != 0)
    {
        lg2::error(
            "Error updating Temperature Sensor for eid {EID} and sensor id {SID} : encode failed, rc={RC}",
            "EID", eid, "SID", sensorId, "RC", rc);
    }

    mctpRequester.sendRecvMsg(
        eid, getTemperatureReadingRequest, getTemperatureReadingResponse,
        [this](int sendRecvMsgResult) { processResponse(sendRecvMsgResult); });
}
