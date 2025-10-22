/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Inventory.hpp"
#include "MctpRequester.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaGpuPowerSensor.hpp"
#include "NvidiaGpuSensor.hpp"
#include "NvidiaXidReporting.hpp"

#include <NvidiaGpuEnergySensor.hpp>
#include <NvidiaGpuPowerPeakReading.hpp>
#include <NvidiaGpuVoltageSensor.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class GpuDevice : public std::enable_shared_from_this<GpuDevice>
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

    void init();

  private:
    void makeSensors();

    void read();

    void processTLimitThresholds(const std::error_code& ec);

    void getTLimitThresholds();

    uint8_t eid{};

    void getNextThermalParameter();
    void readThermalParameterCallback(const std::error_code& ec,
                                      std::span<const uint8_t> buffer);

    std::chrono::milliseconds sensorPollMs;

    boost::asio::steady_timer waitTimer;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    std::shared_ptr<NvidiaGpuTempSensor> tempSensor;
    std::shared_ptr<NvidiaGpuTempSensor> tLimitSensor;
    std::shared_ptr<NvidiaGpuTempSensor> dramTempSensor;
    std::shared_ptr<NvidiaGpuPowerSensor> powerSensor;
    std::shared_ptr<NvidiaGpuPowerPeakReading> peakPower;
    std::shared_ptr<NvidiaGpuEnergySensor> energySensor;
    std::shared_ptr<NvidiaGpuVoltageSensor> voltageSensor;
    std::shared_ptr<NvidiaEventReportingConfig> eventReporting;

    std::array<uint8_t, sizeof(gpu::ReadThermalParametersRequest)>
        thermalParamReqMsg{};
    std::array<int32_t, 3> thresholds{};
    size_t current_threshold_index{};

    SensorConfigs configs;

    std::string name;

    std::string path;

    std::shared_ptr<Inventory> inventory;
};
