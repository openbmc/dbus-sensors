/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "Thresholds.hpp"
#include "sensor.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

constexpr uint8_t gpuPowerSensorId{0};

class SensorMetricReport;

struct NvidiaGpuPowerSensor :
    public Sensor,
    public std::enable_shared_from_this<NvidiaGpuPowerSensor>
{
  public:
    NvidiaGpuPowerSensor(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& name,
        const std::string& sensorConfiguration, uint8_t eid, uint8_t sensorId,
        sdbusplus::asio::object_server& objectServer,
        std::vector<thresholds::Threshold>&& thresholdData,
        SensorMetricReport& sensorMetricReport);

    ~NvidiaGpuPowerSensor() override;

    void checkThresholds() override;

    void update();

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    uint8_t eid{};

    uint8_t sensorId;

    uint8_t averagingInterval = 0;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    sdbusplus::asio::object_server& objectServer;

    std::array<uint8_t, sizeof(gpu::GetPowerDrawRequest)> request{};
};
