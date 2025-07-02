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

constexpr uint8_t gpuTempSensorId{0};
constexpr uint8_t gpuTLimitSensorId{2};
constexpr uint8_t smaTempSensorId{5};

struct NvidiaGpuTempSensor :
    public Sensor,
    public std::enable_shared_from_this<NvidiaGpuTempSensor>
{
  public:
    NvidiaGpuTempSensor(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& name,
        const std::string& sensorConfiguration, uint8_t eid, uint8_t sensorId,
        sdbusplus::asio::object_server& objectServer,
        std::vector<thresholds::Threshold>&& thresholdData);

    ~NvidiaGpuTempSensor() override;

    void checkThresholds() override;

    void update();

  private:
    void processResponse(int sendRecvMsgResult);

    uint8_t eid{};

    uint8_t sensorId;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    sdbusplus::asio::object_server& objectServer;

    std::array<uint8_t, sizeof(gpu::GetTemperatureReadingRequest)>
        getTemperatureReadingRequest{};

    std::array<uint8_t, sizeof(gpu::GetTemperatureReadingResponse)>
        getTemperatureReadingResponse{};
};
