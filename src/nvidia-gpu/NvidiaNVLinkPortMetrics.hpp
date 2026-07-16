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
#include <system_error>
#include <vector>

// Describes one DMTF PortMetrics counter exposed for an NVLink port.
struct NvidiaNVLinkMetricInfo
{
    // Index of the counter in the NSM Get Port Telemetry Counter response,
    // which is also the bit position in the supportedCounters bitmap that gates
    // it.
    uint8_t counterIndex;
    // xyz.openbmc_project.metric object-path leaf, e.g. "/nvlink/rx_bytes".
    const char* metricName;
    // xyz.openbmc_project.Metric.Value.Unit.* value for this counter.
    const char* unit;
};

// Publishes xyz.openbmc_project.Metric.Value objects for a single GPU NVLink
// port and polls them via NSM 0x01 (Get Port Telemetry Counter). Each object is
// associated (measuring/measured_by) with the Connector.Port object published
// by NvidiaNVLinkPort.
struct NvidiaNVLinkPortMetrics :
    public std::enable_shared_from_this<NvidiaNVLinkPortMetrics>
{
  public:
    NvidiaNVLinkPortMetrics(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& gpuName,
        uint8_t eid, uint8_t portIndex,
        sdbusplus::asio::object_server& objectServer,
        const std::vector<NvidiaNVLinkMetricInfo>& metricsInfo);

    void update();

  private:
    void processResponse(const std::error_code& sendRecvMsgResult,
                         std::span<const uint8_t> response);

    uint8_t eid = 0;

    // 1-based port number on the wire.
    uint8_t portNumber = 0;

    std::string gpuName;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::vector<NvidiaNVLinkMetricInfo> metricsInfo;

    std::vector<uint8_t> request;

    std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>>
        metricValueInterfaces;

    std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>>
        metricAssociationInterfaces;

    std::array<uint64_t, gpu::maxPortTelemetryCounters> counters{};
};

std::shared_ptr<NvidiaNVLinkPortMetrics> makeNvidiaNVLinkPortMetrics(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    uint8_t portIndex, sdbusplus::asio::object_server& objectServer);
