/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

struct NvidiaPcieFunction :
    public std::enable_shared_from_this<NvidiaPcieFunction>
{
  public:
    NvidiaPcieFunction(std::shared_ptr<sdbusplus::asio::connection>& conn,
                       mctp::MctpRequester& mctpRequester,
                       const std::string& pcieDeviceName,
                       const std::string& path, uint8_t eid,
                       sdbusplus::asio::object_server& objectServer);

    void update();

  private:
    static constexpr size_t maxTelemetryValues = 16;

    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> response);

    uint8_t eid{};

    std::string path;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::array<uint8_t, sizeof(gpu::QueryScalarGroupTelemetryV1Request)>
        request{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> pcieFunctionInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    std::vector<uint32_t> telemetryValues{maxTelemetryValues};
};
