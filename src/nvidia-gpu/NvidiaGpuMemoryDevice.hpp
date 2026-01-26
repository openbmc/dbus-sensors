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

struct NvidiaGpuMemoryDevice :
    public std::enable_shared_from_this<NvidiaGpuMemoryDevice>
{
  public:
    NvidiaGpuMemoryDevice(std::shared_ptr<sdbusplus::asio::connection>& conn,
                          mctp::MctpRequester& mctpRequester,
                          const std::string& gpuName, uint8_t eid,
                          sdbusplus::asio::object_server& objectServer);

    ~NvidiaGpuMemoryDevice();

    void update();

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    uint8_t eid;
    std::string gpuName;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    mctp::MctpRequester& mctpRequester;
    sdbusplus::asio::object_server& objectServer;

    std::shared_ptr<sdbusplus::asio::dbus_interface> sramEccInterface;

    std::array<uint8_t, sizeof(gpu::GetEccErrorCountsRequest)> requestBuffer{};

    int64_t sramCeCount{0};
    int64_t sramUeCount{0};
};
