/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
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
#include <string>
#include <vector>

constexpr uint8_t gpuPowerSensorId{0};

struct NvidiaGpuPowerSensor : public Sensor
{
  public:
    NvidiaGpuPowerSensor(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& name,
        const std::string& sensorConfiguration, uint8_t eid, uint8_t sensorId,
        sdbusplus::asio::object_server& objectServer,
        std::vector<thresholds::Threshold>&& thresholdData);

    ~NvidiaGpuPowerSensor() override;

    void checkThresholds() override;

    void update();

  private:
    void processResponse(int sendRecvMsgResult);

    uint8_t eid{};

    uint8_t sensorId;

    uint8_t averagingInterval;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    sdbusplus::asio::object_server& objectServer;

    std::array<uint8_t, sizeof(gpu::GetCurrentPowerDrawRequest)> request{};

    std::array<uint8_t, sizeof(gpu::GetCurrentPowerDrawResponse)> response{};
};
