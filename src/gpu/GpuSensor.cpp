/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "UpdatableSensor.hpp"
#include "Utils.hpp"
#include "mctp/Requester.hpp"

#include <bits/basic_string.h>

#include <GpuDevice.hpp>
#include <GpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace std::literals;

constexpr uint8_t gpuTempSensorId{0};
static constexpr double gpuTempSensorMaxReading = 127;
static constexpr double gpuTempSensorMinReading = -128;

GpuTempSensor::GpuTempSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, const uint8_t eid,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData) :
    GpuSensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
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
}

GpuTempSensor::~GpuTempSensor()
{
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(association);
    objectServer.remove_interface(sensorInterface);
}

void GpuTempSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void GpuTempSensor::update()
{
    std::vector<uint8_t> reqMsg(
        sizeof(ocp::accelerator_management::BindingPciVid) +
        sizeof(gpu::GetTemperatureReadingRequest));

    auto* msg = new (reqMsg.data()) ocp::accelerator_management::Message;

    auto rc = gpu::encodeGetTemperatureReadingRequest(0, sensorId, *msg);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("Error updating Temperature Sensor: encode failed, rc={RC}",
                   "RC", static_cast<int>(rc));
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqMsg,
        [this](int sendRecvMsgResult,
               std::optional<std::vector<uint8_t>> resp) {
            if (sendRecvMsgResult != 0)
            {
                lg2::error(
                    "Error updating Temperature Sensor: sending message over MCTP failed, rc={RC}",
                    "RC", sendRecvMsgResult);
                return;
            }

            if (!resp.has_value())
            {
                lg2::error("Error updating Temperature Sensor: empty response");
                return;
            }

            auto respMsg = resp.value();

            uint8_t cc = 0;
            uint16_t reasonCode = 0;
            double tempValue = 0;

            auto rc = gpu::decodeGetTemperatureReadingResponse(
                *new (respMsg.data()) ocp::accelerator_management::Message,
                respMsg.size(), cc, reasonCode, tempValue);

            if (rc != ocp::accelerator_management::CompletionCode::SUCCESS ||
                cc != static_cast<uint8_t>(
                          ocp::accelerator_management::CompletionCode::SUCCESS))
            {
                lg2::error(
                    "Error updating Temperature Sensor: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
                    "RC", static_cast<int>(rc), "CC", cc, "RESC", reasonCode);
                return;
            }

            updateValue(tempValue);
        });
}
