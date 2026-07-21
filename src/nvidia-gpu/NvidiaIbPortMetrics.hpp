/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>

struct NvidiaIbPortMetrics :
    public std::enable_shared_from_this<NvidiaIbPortMetrics>
{
  public:
    NvidiaIbPortMetrics(mctp::MctpRequester& mctpRequester,
                        const std::string& name, const std::string& deviceName,
                        uint8_t eid, uint16_t portNumber,
                        sdbusplus::asio::object_server& objectServer);

    ~NvidiaIbPortMetrics();

    NvidiaIbPortMetrics(const NvidiaIbPortMetrics&) = delete;
    NvidiaIbPortMetrics& operator=(const NvidiaIbPortMetrics&) = delete;
    NvidiaIbPortMetrics(NvidiaIbPortMetrics&&) = delete;
    NvidiaIbPortMetrics& operator=(NvidiaIbPortMetrics&&) = delete;

    void update();

  private:
    static constexpr size_t maxTelemetryValues = 32;

    void processResponse(const std::error_code& sendRecvMsgResult,
                         std::span<const uint8_t> response);

    uint8_t eid;

    uint16_t portNumber;

    mctp::MctpRequester& mctpRequester;

    sdbusplus::asio::object_server& objectServer;

    std::array<uint8_t, gpu::getPortTelemetryCountersRequestSize> request{};

    bool requestEncoded{false};

    std::array<std::shared_ptr<sdbusplus::asio::dbus_interface>,
               maxTelemetryValues>
        metricValueInterface{};

    std::array<std::shared_ptr<sdbusplus::asio::dbus_interface>,
               maxTelemetryValues>
        metricAssociationInterfaces{};
};
