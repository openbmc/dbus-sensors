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

struct NvidiaPciePortErrors :
    public std::enable_shared_from_this<NvidiaPciePortErrors>
{
  public:
    NvidiaPciePortErrors(std::shared_ptr<sdbusplus::asio::connection>& conn,
                         mctp::MctpRequester& mctpRequester,
                         const std::string& name, const std::string& path,
                         uint8_t eid, gpu::PciePortType portType,
                         uint8_t upstreamPortNumber, uint8_t portNumber,
                         sdbusplus::asio::object_server& objectServer);

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

    std::string path;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::array<uint8_t, sizeof(gpu::QueryScalarGroupTelemetryV2Request)>
        request{};

    std::array<uint8_t, sizeof(ocp::accelerator_management::CommonResponse) +
                            sizeof(uint32_t) * maxTelemetryValues>
        response{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> nonFatalErrorsInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> fatalErrorsInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> correctableErrorsInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        unsupportedRequestsInterface;

    std::vector<uint32_t> telemetryValues{maxTelemetryValues};
};

struct NvidiaPciePortCounters :
    public std::enable_shared_from_this<NvidiaPciePortCounters>
{
  public:
    NvidiaPciePortCounters(std::shared_ptr<sdbusplus::asio::connection>& conn,
                           mctp::MctpRequester& mctpRequester,
                           const std::string& name, const std::string& path,
                           uint8_t eid, gpu::PciePortType portType,
                           uint8_t upstreamPortNumber, uint8_t portNumber,
                           sdbusplus::asio::object_server& objectServer);

    void update();

  private:
    static constexpr size_t maxTelemetryValues = 64;

    void processResponse(const std::error_code& sendRecvMsgResult,
                         std::span<const uint8_t> response);

    static double mapPcieGenToLinkSpeedGbps(uint32_t value);

    uint8_t eid{};

    gpu::PciePortType portType;

    uint8_t upstreamPortNumber;

    uint8_t portNumber;

    std::string path;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::array<uint8_t, sizeof(gpu::QueryScalarGroupTelemetryV2Request)>
        request{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> replayCountInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        replayRolloverCountInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> nakSentCountInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> nakReceivedCountInterface;

    std::vector<uint32_t> telemetryValues{maxTelemetryValues};
};

struct NvidiaPciePortL0ToRecoveryCount :
    public std::enable_shared_from_this<NvidiaPciePortL0ToRecoveryCount>
{
  public:
    NvidiaPciePortL0ToRecoveryCount(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& name,
        const std::string& path, uint8_t eid, gpu::PciePortType portType,
        uint8_t upstreamPortNumber, uint8_t portNumber,
        sdbusplus::asio::object_server& objectServer);

    void update();

  private:
    static constexpr size_t maxTelemetryValues = 64;

    void processResponse(const std::error_code& sendRecvMsgResult,
                         std::span<const uint8_t> response);

    static double mapPcieGenToLinkSpeedGbps(uint32_t value);

    uint8_t eid{};

    gpu::PciePortType portType;

    uint8_t upstreamPortNumber;

    uint8_t portNumber;

    std::string path;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::array<uint8_t, sizeof(gpu::QueryScalarGroupTelemetryV2Request)>
        request{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> l0ToRecoveryCountInterface;

    std::vector<uint32_t> telemetryValues{maxTelemetryValues};
};
