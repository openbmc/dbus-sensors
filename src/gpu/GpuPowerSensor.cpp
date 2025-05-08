/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuPowerSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "UpdatableSensor.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <GpuDevice.hpp>
#include <GpuMctpVdm.hpp>
#include <MctpRequester.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace std::literals;

constexpr uint8_t gpuPowerSensorId{0};
constexpr uint8_t gpuPowerAveragingInterval{0};
static constexpr double gpuPowerSensorMaxReading =
    std::numeric_limits<uint32_t>::max();
static constexpr double gpuPowerSensorMinReading =
    std::numeric_limits<uint32_t>::min();

GpuPowerSensor::GpuPowerSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, uint8_t eid,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData) :
    GpuSensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
              "temperature", false, true, gpuPowerSensorMaxReading,
              gpuPowerSensorMinReading, conn),
    eid(eid), sensorId{gpuPowerSensorId},
    averagingInterval{gpuPowerAveragingInterval}, mctpRequester(mctpRequester),
    objectServer(objectServer)
{
    std::string dbusPath = sensorPathPrefix + "power/"s + escapeName(name);

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

GpuPowerSensor::~GpuPowerSensor()
{
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

void GpuPowerSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void GpuPowerSensor::update()
{
    std::vector<uint8_t> reqMsg(
        sizeof(ocp::accelerator_management::BindingPciVid) +
        sizeof(gpu::GetCurrentPowerDrawRequest));

    auto* msg = new (reqMsg.data()) ocp::accelerator_management::Message;

    auto rc = gpu::encodeGetCurrentPowerDrawRequest(0, sensorId,
                                                    averagingInterval, *msg);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "GpuPowerSensor::update(): encodeGetCurrentPowerDrawRequest failed, rc={RC}",
            "RC", static_cast<int>(rc));
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqMsg,
        [this](int sendRecvMsgResult, std::vector<uint8_t> respMsg) {
            if (sendRecvMsgResult != 0)
            {
                lg2::error(
                    "GpuPowerSensor::update(): MctpRequester::sendRecvMsg() failed, rc={RC}",
                    "RC", sendRecvMsgResult);
                return;
            }

            if (respMsg.empty())
            {
                lg2::error(
                    "GpuPowerSensor::update(): MctpRequester::sendRecvMsg() failed, respMsgLen=0");
                return;
            }

            uint8_t cc = 0;
            uint16_t reasonCode = 0;
            uint32_t power = 0;

            auto rc = gpu::decodeGetCurrentPowerDrawResponse(
                *new (respMsg.data()) ocp::accelerator_management::Message,
                respMsg.size(), cc, reasonCode, power);

            if (rc != ocp::accelerator_management::CompletionCode::SUCCESS ||
                cc != static_cast<uint8_t>(
                          ocp::accelerator_management::CompletionCode::SUCCESS))
            {
                lg2::error(
                    "GpuPowerSensor::update(): decodeGetCurrentPowerDrawResponse() failed, rc={RC} cc={CC} reasonCode={RESC}",
                    "RC", static_cast<int>(rc), "CC", cc, "RESC", reasonCode);
                return;
            }

            updateValue(power / 1000.0);
        });
}
