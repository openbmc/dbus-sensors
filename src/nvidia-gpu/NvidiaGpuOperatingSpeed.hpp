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
#include <span>
#include <string>
#include <system_error>

struct NvidiaGpuOperatingSpeed :
    public std::enable_shared_from_this<NvidiaGpuOperatingSpeed>
{
  public:
    NvidiaGpuOperatingSpeed(std::shared_ptr<sdbusplus::asio::connection>& conn,
                            mctp::MctpRequester& mctpRequester,
                            const std::string& gpuName, uint8_t eid,
                            sdbusplus::asio::object_server& objectServer);

    ~NvidiaGpuOperatingSpeed();

    void update();

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    uint8_t eid;
    std::string gpuName;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    mctp::MctpRequester& mctpRequester;
    sdbusplus::asio::object_server& objectServer;

    std::shared_ptr<sdbusplus::asio::dbus_interface> operatingConfigInterface;

    std::array<uint8_t, sizeof(gpu::GetCurrentClockFrequencyRequest)>
        requestBuffer{};

    uint32_t operatingSpeed{0};
};
