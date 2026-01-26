/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

/**
 * @brief NVIDIA GPU ECC Error Counts Sensor
 *
 * Queries GPU ECC error counts via NSM protocol and exposes them
 * through D-Bus Memory.MemoryECC interface.
 */
struct NvidiaGpuEccSensor :
    public std::enable_shared_from_this<NvidiaGpuEccSensor>
{
  public:
    NvidiaGpuEccSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                       mctp::MctpRequester& mctpRequester,
                       const std::string& name, uint8_t eid,
                       sdbusplus::asio::object_server& objectServer);

    ~NvidiaGpuEccSensor() = default;

    void update(); // Called periodically to query GPU

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    uint8_t eid{};
    std::string name;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    mctp::MctpRequester& mctpRequester;
    std::shared_ptr<sdbusplus::asio::dbus_interface> memoryEccInterface;
    std::array<uint8_t, sizeof(gpu::GetEccErrorCountsRequest)> requestBuffer{};
};
