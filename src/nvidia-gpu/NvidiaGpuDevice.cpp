/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuDevice.hpp"

#include "Inventory.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaGpuSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuEnergySensor.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaGpuPowerPeakReading.hpp>
#include <NvidiaGpuPowerSensor.hpp>
#include <NvidiaGpuThresholds.hpp>
#include <NvidiaGpuVoltageSensor.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

GpuDevice::GpuDevice(const SensorConfigs& configs, const std::string& name,
                     const std::string& path,
                     const std::shared_ptr<sdbusplus::asio::connection>& conn,
                     uint8_t eid, boost::asio::io_context& io,
                     mctp::MctpRequester& mctpRequester,
                     sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{configs.pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer),
    configs(configs), name(escapeName(name)), path(path)
{
    inventory = std::make_shared<Inventory>(
        conn, objectServer, name, mctpRequester,
        gpu::DeviceIdentification::DEVICE_GPU, eid, io);
    makeSensors();
}

void GpuDevice::makeSensors()
{
    tempSensor = std::make_shared<NvidiaGpuTempSensor>(
        conn, mctpRequester, name + "_TEMP_0", path, eid, gpuTempSensorId,
        objectServer, std::vector<thresholds::Threshold>{});

    readThermalParameters(
        eid,
        std::vector<gpuThresholdId>{gpuTLimitWarnringThresholdId,
                                    gpuTLimitCriticalThresholdId,
                                    gpuTLimitHardshutDownThresholdId},
        mctpRequester,
        std::bind_front(&GpuDevice::processTLimitThresholds, this));

    dramTempSensor = std::make_shared<NvidiaGpuTempSensor>(
        conn, mctpRequester, name + "_DRAM_0_TEMP_0", path, eid,
        gpuDramTempSensorId, objectServer,
        std::vector<thresholds::Threshold>{thresholds::Threshold{
            thresholds::Level::CRITICAL, thresholds::Direction::HIGH, 95.0}});

    powerSensor = std::make_shared<NvidiaGpuPowerSensor>(
        conn, mctpRequester, name + "_Power_0", path, eid, gpuPowerSensorId,
        objectServer, std::vector<thresholds::Threshold>{});

    peakPower = std::make_shared<NvidiaGpuPowerPeakReading>(
        mctpRequester, name + "_Power_0", eid, gpuPeakPowerSensorId,
        objectServer);

    energySensor = std::make_shared<NvidiaGpuEnergySensor>(
        conn, mctpRequester, name + "_Energy_0", path, eid, gpuEnergySensorId,
        objectServer, std::vector<thresholds::Threshold>{});

    voltageSensor = std::make_shared<NvidiaGpuVoltageSensor>(
        conn, mctpRequester, name + "_Voltage_0", path, eid, gpuVoltageSensorId,
        objectServer, std::vector<thresholds::Threshold>{});

    lg2::info("Added GPU {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);

    read();
}

void GpuDevice::processTLimitThresholds(uint8_t rc,
                                        const std::vector<int32_t>& thresholds)
{
    std::vector<thresholds::Threshold> tLimitThresholds{};
    if (rc == 0)
    {
        tLimitThresholds = {
            thresholds::Threshold{thresholds::Level::WARNING,
                                  thresholds::Direction::LOW,
                                  static_cast<double>(thresholds[0])},
            thresholds::Threshold{thresholds::Level::CRITICAL,
                                  thresholds::Direction::LOW,
                                  static_cast<double>(thresholds[1])},
            thresholds::Threshold{thresholds::Level::HARDSHUTDOWN,
                                  thresholds::Direction::LOW,
                                  static_cast<double>(thresholds[2])}};
    }

    tLimitSensor = std::make_shared<NvidiaGpuTempSensor>(
        conn, mctpRequester, name + "_TEMP_1", path, eid, gpuTLimitSensorId,
        objectServer, std::move(tLimitThresholds));
}

void GpuDevice::read()
{
    tempSensor->update();
    if (tLimitSensor)
    {
        tLimitSensor->update();
    }
    dramTempSensor->update();
    powerSensor->update();
    peakPower->update();
    energySensor->update();
    voltageSensor->update();

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec)
        {
            return;
        }
        read();
    });
}
