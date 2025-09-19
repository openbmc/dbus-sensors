/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSmaDevice.hpp"

#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaGpuSensor.hpp"
#include "asio/Thresholds.hpp"
#include "utils/Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

SmaDevice::SmaDevice(const SensorConfigs& configs, const std::string& name,
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

void SmaDevice::makeSensors()
{
    tempSensor = std::make_shared<NvidiaGpuTempSensor>(
        conn, mctpRequester, name + "_TEMP_0", path, eid, smaTempSensorId,
        objectServer, std::vector<thresholds::Threshold>{});

    lg2::info("Added MCA {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);

    read();
}

void SmaDevice::read()
{
    tempSensor->update();

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec)
        {
            return;
        }
        read();
    });
}
