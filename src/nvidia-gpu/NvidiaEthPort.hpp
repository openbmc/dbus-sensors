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

struct NvidiaEthPortMetrics :
    public std::enable_shared_from_this<NvidiaEthPortMetrics>
{
  public:
    NvidiaEthPortMetrics(std::shared_ptr<sdbusplus::asio::connection>& conn,
                         mctp::MctpRequester& mctpRequester,
                         const std::string& name, const std::string& path,
                         uint8_t eid, uint16_t portNumber,
                         sdbusplus::asio::object_server& objectServer);

    void update();

  private:
    static constexpr size_t maxTelemetryValues = 64;

    void processResponse(const std::error_code& sendRecvMsgResult,
                         std::span<const uint8_t> response);

    static double mapPcieGenToLinkSpeedGbps(uint32_t value);

    uint8_t eid;

    uint16_t portNumber;

    std::string path;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::array<uint8_t, sizeof(gpu::GetEthernetPortTelemetryCountersRequest)>
        request{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> portInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    std::array<std::shared_ptr<sdbusplus::asio::dbus_interface>,
               maxTelemetryValues>
        metricValueInterface{};
};
