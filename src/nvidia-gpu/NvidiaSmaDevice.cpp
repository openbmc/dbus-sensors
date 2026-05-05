/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSmaDevice.hpp"

#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaSmaLeakSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
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
{}

void SmaDevice::init()
{
    makeSensors();
}

void SmaDevice::makeSensors()
{
    leakSensor = std::make_shared<NvidiaSmaLeakSensor>(
        conn, mctpRequester, name + "_LEAKDETECTOR_0", path, eid,
        smaLeakSensorId, objectServer,
        std::vector<thresholds::Threshold>{
            thresholds::Threshold(thresholds::Level::CRITICAL,
                                  thresholds::Direction::HIGH, 1.815),
            thresholds::Threshold(thresholds::Level::WARNING,
                                  thresholds::Direction::LOW, 1.65),
            thresholds::Threshold(thresholds::Level::CRITICAL,
                                  thresholds::Direction::LOW, 0.165)},
        gpu::DeviceIdentification::DEVICE_SMA);

    lg2::info("Added MCA {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);

    read();
}

void SmaDevice::read()
{
    leakSensor->update();

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            std::shared_ptr<SmaDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid SmaDevice reference");
                return;
            }
            if (ec)
            {
                return;
            }
            self->read();
        });
}
