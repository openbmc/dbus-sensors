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

constexpr uint8_t gpuTempSensorId{0};
static constexpr double gpuTempSensorMaxReading = 127;
static constexpr double gpuTempSensorMinReading = -128;

NvidiaGpuTempSensor::NvidiaGpuTempSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, const uint8_t eid,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData) :
    Sensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
           "temperature", false, true, gpuTempSensorMaxReading,
           gpuTempSensorMinReading, conn),
    eid(eid), sensorId{gpuTempSensorId}, mctpRequester(mctpRequester),
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

    auto rc = gpu::encodeGetTemperatureReadingRequest(
        0, sensorId, getTemperatureReadingRequest);
    if (rc != 0)
    {
        lg2::error(
            "Error updating Temperature Sensor for eid {EID} and sensor id {SID} : encode failed, rc={RC}",
            "EID", eid, "SID", sensorId, "RC", rc);
    }
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
    mctpRequester.sendRecvMsg(
        eid, getTemperatureReadingRequest, getTemperatureReadingResponse,
        [this](int sendRecvMsgResult) { processResponse(sendRecvMsgResult); });
}

void GpuTempSensor::requestDevicePartNumber()
{
    int rc = gpu::encodeGetInventoryInformationRequest(
        0, static_cast<uint8_t>(gpu::InventoryPropertyId::DEVICE_PART_NUMBER), getDevicePartNumberRequest);
    if (rc != 0)
    {
        lg2::error("Failed to encode device part number request: rc={RC}", "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, getDevicePartNumberRequest, getDevicePartNumberResponse,
        [this](int sendRecvMsgResult) { handleDevicePartNumberResponse(sendRecvMsgResult); });
}

void GpuTempSensor::handleDevicePartNumberResponse(int sendRecvMsgResult)
{
    if (sendRecvMsgResult != 0)
    {
        lg2::error("Failed to get device part number: rc={RC}", "RC", sendRecvMsgResult);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::InventoryInfo info;

    int rc = gpu::decodeGetInventoryInformationResponse(
        getDevicePartNumberResponse, cc, reasonCode, gpu::InventoryPropertyId::DEVICE_PART_NUMBER, info);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("Failed to decode device part number: rc={RC}, cc={CC}, reason={REASON}",
                   "RC", rc, "CC", cc, "REASON", reasonCode);
        return;
    }

    std::string partNumber;
    if (std::holds_alternative<std::string>(info))
    {
        partNumber = std::get<std::string>(info);
        if (assetIface)
        {
            assetIface->set_property("PartNumber", partNumber);
        }
        else
        {
            lg2::error("Asset interface not initialized when updating PartNumber");
        }
    }
    else
    {
        lg2::error("Device part number not a string");
        return;
    }
}

