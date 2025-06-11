/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Inventory.hpp"
#include "MctpRequester.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaGpuPowerSensor.hpp"
#include "NvidiaGpuSensor.hpp"

#include <NvidiaGpuEnergySensor.hpp>
#include <NvidiaGpuVoltageSensor.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

class GpuDevice
{
  public:
    GpuDevice(const SensorConfigs& configs, const std::string& name,
              const std::string& path,
              const std::shared_ptr<sdbusplus::asio::connection>& conn,
              uint8_t eid, boost::asio::io_context& io,
              mctp::MctpRequester& mctpRequester,
              sdbusplus::asio::object_server& objectServer);

    const std::string& getPath() const
    {
        return path;
    }

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
    std::shared_ptr<NvidiaGpuTempSensor> tLimitSensor;
    std::shared_ptr<NvidiaGpuPowerSensor> powerSensor;
    std::shared_ptr<NvidiaGpuEnergySensor> energySensor;
    std::shared_ptr<NvidiaGpuVoltageSensor> voltageSensor;

    SensorConfigs configs;

    std::string name;

    std::string path;

    std::unique_ptr<Inventory> inventory;
};
