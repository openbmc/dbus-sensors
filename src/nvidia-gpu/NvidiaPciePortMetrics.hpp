/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "OcpMctpVdm.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct NvidiaMetricInfo
{
    uint8_t metricId;
    const char* metricName;
};

struct NvidiaPciePortMetrics :
    public std::enable_shared_from_this<NvidiaPciePortMetrics>
{
  public:
    NvidiaPciePortMetrics(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& name,
        const std::string& pcieDeviceName, const std::string& path, uint8_t eid,
        gpu::PciePortType portType, uint8_t upstreamPortNumber,
        uint8_t portNumber, sdbusplus::asio::object_server& objectServer,
        uint8_t scalarGroupId,
        const std::vector<NvidiaMetricInfo>& metricsInfo);

    void update();

  private:
    static constexpr size_t maxTelemetryValues = 64;

    void processResponse(const std::error_code& sendRecvMsgResult,
                         std::span<const uint8_t> response);

    static double mapPcieGenToLinkSpeedGbps(uint32_t value);

    uint8_t eid = 0;

    gpu::PciePortType portType;

    uint8_t upstreamPortNumber;

    uint8_t portNumber;

    uint8_t scalarGroupId;

    std::string path;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::array<uint8_t, sizeof(gpu::QueryScalarGroupTelemetryV2Request)>
        request{};

    std::array<uint8_t, sizeof(ocp::accelerator_management::CommonResponse) +
                            sizeof(uint32_t) * maxTelemetryValues>
        response{};

    std::array<std::shared_ptr<sdbusplus::asio::dbus_interface>,
               maxTelemetryValues>
        metricValueInterfaces;

    std::array<std::shared_ptr<sdbusplus::asio::dbus_interface>,
               maxTelemetryValues>
        metricAssociationInterfaces;

    std::vector<uint32_t> telemetryValues{maxTelemetryValues};
};

std::shared_ptr<NvidiaPciePortMetrics> makeNvidiaPciePortErrors(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& pcieDeviceName, const std::string& path, uint8_t eid,
    gpu::PciePortType portType, uint8_t upstreamPortNumber, uint8_t portNumber,
    sdbusplus::asio::object_server& objectServer);

std::shared_ptr<NvidiaPciePortMetrics> makeNvidiaPciePortCounters(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& pcieDeviceName, const std::string& path, uint8_t eid,
    gpu::PciePortType portType, uint8_t upstreamPortNumber, uint8_t portNumber,
    sdbusplus::asio::object_server& objectServer);

std::shared_ptr<NvidiaPciePortMetrics> makeNvidiaPciePortL0ToRecoveryCount(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& pcieDeviceName, const std::string& path, uint8_t eid,
    gpu::PciePortType portType, uint8_t upstreamPortNumber, uint8_t portNumber,
    sdbusplus::asio::object_server& objectServer);
