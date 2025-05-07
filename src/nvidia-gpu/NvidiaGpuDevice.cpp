/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuDevice.hpp"

#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaGpuSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuThresholds.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
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
        mctpRequester, [this](uint8_t rc, std::vector<int32_t> thresholds) {
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
                conn, mctpRequester, name + "_TEMP_1", path, eid,
                gpuTLimitSensorId, objectServer, std::move(tLimitThresholds));
        });

    lg2::info("Added GPU {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);

    read();
}

void GpuDevice::read()
{
    tempSensor->update();
    if (tLimitSensor)
    {
        tLimitSensor->update();
    }

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec)
        {
            return;
        }
        read();
    });
}
