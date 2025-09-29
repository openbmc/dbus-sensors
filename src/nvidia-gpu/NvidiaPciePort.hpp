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
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct NvidiaPciePortInfo :
    public std::enable_shared_from_this<NvidiaPciePortInfo>
{
  public:
    NvidiaPciePortInfo(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& name,
        const std::string& pcieDeviceName, const std::string& path, uint8_t eid,
        gpu::PciePortType portType, uint8_t upstreamPortNumber,
        uint8_t portNumber, sdbusplus::asio::object_server& objectServer);

    void update();

  private:
    static constexpr size_t maxTelemetryValues = 64;

    void processResponse(const std::error_code& sendRecvMsgResult,
                         std::span<const uint8_t> response);

    static uint64_t mapPcieGenToLinkSpeedBitsPerSecond(uint32_t value);

    uint8_t eid = 0;

    gpu::PciePortType portType;

    uint8_t upstreamPortNumber;

    uint8_t portNumber;

    std::string path;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::array<uint8_t, sizeof(gpu::QueryScalarGroupTelemetryV2Request)>
        request{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> pciePortInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    std::vector<uint32_t> telemetryValues{maxTelemetryValues};
};
