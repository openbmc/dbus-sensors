/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <system_error>

// The optical power and laser bias current of each lane of the module
// plugged into one port of a network device.
class NvidiaTransceiverMetrics :
    public std::enable_shared_from_this<NvidiaTransceiverMetrics>
{
  public:
    NvidiaTransceiverMetrics(sdbusplus::asio::object_server& objectServer,
                             mctp::MctpRequester& mctpRequester,
                             const std::string& deviceName,
                             const std::string& portName,
                             const sdbusplus::object_path& transceiverPath,
                             uint8_t eid, uint16_t portNumber);

    ~NvidiaTransceiverMetrics();

    NvidiaTransceiverMetrics(const NvidiaTransceiverMetrics&) = delete;
    NvidiaTransceiverMetrics& operator=(const NvidiaTransceiverMetrics&) =
        delete;
    NvidiaTransceiverMetrics(NvidiaTransceiverMetrics&&) = delete;
    NvidiaTransceiverMetrics& operator=(NvidiaTransceiverMetrics&&) = delete;

    void update();

  private:
    using LaneInterfaces =
        std::array<std::shared_ptr<sdbusplus::asio::dbus_interface>,
                   gpu::opticalModuleLaneCount>;

    void addLaneMetrics(const sdbusplus::object_path& metricsPath,
                        const sdbusplus::object_path& transceiverPath,
                        std::string_view name, const std::string& unit,
                        LaneInterfaces& interfaces,
                        LaneInterfaces& associationInterfaces);

    void requestPage(uint32_t sequenceToken);

    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> response);

    void publish();

    sdbusplus::asio::object_server& objectServer;

    mctp::MctpRequester& mctpRequester;

    uint8_t eid;

    uint16_t portNumber;

    std::array<uint8_t, gpu::queryPortTelemetryV2RequestSize> request{};

    bool readInflight{false};

    size_t pagesRequested{0};

    // Raw readings of the read in progress and of the last one published.
    // publish() compares lanes on these, since set_property() compares with
    // == and would resend a NaN lane on every poll.
    gpu::OpticalModuleTelemetry readings;
    gpu::OpticalModuleTelemetry reported;

    ocp::accelerator_management::CompletionCode lastCompletionCode{
        ocp::accelerator_management::CompletionCode::SUCCESS};

    LaneInterfaces txPowerInterfaces;

    LaneInterfaces txPowerAssociationInterfaces;

    LaneInterfaces rxPowerInterfaces;

    LaneInterfaces rxPowerAssociationInterfaces;

    LaneInterfaces txBiasCurrentInterfaces;

    LaneInterfaces txBiasCurrentAssociationInterfaces;
};
