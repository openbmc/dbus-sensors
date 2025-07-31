/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
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

constexpr uint8_t gpuPeakPowerSensorId{0};

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
    void processResponse(int sendRecvMsgResult);

    uint8_t eid{};

    uint8_t sensorId;

    uint8_t averagingInterval;

    mctp::MctpRequester& mctpRequester;

    sdbusplus::asio::object_server& objectServer;

    std::array<uint8_t, sizeof(gpu::GetPowerDrawRequest)> request{};

    std::array<uint8_t, sizeof(gpu::GetPowerDrawResponse)> response{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> statisticsInterface;
};
