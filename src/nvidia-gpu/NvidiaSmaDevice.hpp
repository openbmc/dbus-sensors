/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuTempSensor.hpp"
#include "NvidiaSensorConfig.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

class SmaDevice : public std::enable_shared_from_this<SmaDevice>
{
  public:
    SmaDevice(const SensorConfigs& configs, const std::string& name,
              const sdbusplus::object_path& path,
              const std::shared_ptr<sdbusplus::asio::connection>& conn,
              uint8_t eid, boost::asio::io_context& io,
              mctp::MctpRequester& mctpRequester,
              sdbusplus::asio::object_server& objectServer);

    // The D-Bus path of the EntityManager configuration object the device
    // was created from.
    const sdbusplus::object_path& getPath() const
    {
        return path;
    }

    // Build the device's D-Bus objects. Polling only starts once the device
    // is taken online.
    void init();

    // Stop polling timer(s) and set reading sensors to unavailable. The
    // D-Bus objects are retained.
    void setOffline();

    // Resume polling.
    void setOnline();

  private:
    void makeSensors();

    void read();

    uint8_t eid{};

    std::chrono::milliseconds sensorPollMs;

    boost::asio::steady_timer waitTimer;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    std::shared_ptr<NvidiaGpuTempSensor> tempSensor;

    SensorConfigs configs;

    std::string name;

    sdbusplus::object_path path;
};
