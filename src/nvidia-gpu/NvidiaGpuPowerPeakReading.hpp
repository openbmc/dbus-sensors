/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

constexpr uint8_t gpuPeakPowerSensorId{0};

// GPU Power Sensor Averaging Interval in seconds, 0 implies default
constexpr uint8_t gpuPowerAveragingIntervalInSec{0};

struct NvidiaGpuPowerPeakReading
{
  public:
    NvidiaGpuPowerPeakReading(mctp::MctpRequester& mctpRequester,
                              const std::string& name, uint8_t eid,
                              uint8_t sensorId,
                              sdbusplus::asio::object_server& objectServer);

    ~NvidiaGpuPowerPeakReading();

    void update();

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    uint8_t eid{};

    uint8_t sensorId;

    uint8_t averagingInterval{gpuPowerAveragingIntervalInSec};

    std::tuple<
        uint64_t,
        std::vector<std::tuple<std::string, std::string, double, uint64_t>>>
        readings;

    mctp::MctpRequester& mctpRequester;

    sdbusplus::asio::object_server& objectServer;

    std::array<uint8_t, sizeof(gpu::GetPowerDrawRequest)> request{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> telemetryReportInterface;
};
