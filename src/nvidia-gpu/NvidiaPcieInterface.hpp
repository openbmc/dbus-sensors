/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct NvidiaPcieInterface :
    public std::enable_shared_from_this<NvidiaPcieInterface>
{
  public:
    NvidiaPcieInterface(std::shared_ptr<sdbusplus::asio::connection>& conn,
                        mctp::MctpRequester& mctpRequester,
                        const std::string& name, const std::string& path,
                        const std::string& fabricPath, uint8_t eid,
                        sdbusplus::asio::object_server& objectServer);

    void update();

    static size_t decodeLinkWidth(uint32_t value);

  private:
    static constexpr size_t maxTelemetryValues = 64;

    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> response);

    static std::string mapPcieGeneration(uint32_t value);

    uint8_t eid{};

    std::string path;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::array<uint8_t, sizeof(gpu::QueryScalarGroupTelemetryV2Request)>
        request{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> pcieDeviceInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> switchInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> fabricAssociationInterface;

    std::vector<uint32_t> telemetryValues{maxTelemetryValues};
};
