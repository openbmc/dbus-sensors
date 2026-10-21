/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

// Publishes a Control.Port.LinkState object for every NVLink port of a GPU and
// polls the link state each port will take after the next GPU reset via Nvidia
// MCTP VDM 0x45 (Get Port Disable Future). Writing RequestedLinkState changes
// it with 0x44 (Set Port Disable Future). The device keeps this as a single
// mask covering all of its ports, so one instance serves the whole GPU. Each
// object is associated (controlling/controlled_by) with the Connector.Port
// object of its port.
struct NvidiaNVLinkPortControl :
    public std::enable_shared_from_this<NvidiaNVLinkPortControl>
{
  public:
    NvidiaNVLinkPortControl(
        sdbusplus::asio::object_server& objectServer,
        mctp::MctpRequester& mctpRequester, boost::asio::io_context& io,
        const std::string& gpuName, uint8_t eid, uint8_t numberOfPorts);

    ~NvidiaNVLinkPortControl();

    void update();

  private:
    using PortMask = std::array<uint8_t, gpu::portDisableFutureMaskSize>;

    // setsSeen is setsSent at the time the request was sent.
    void processResponse(const std::error_code& sendRecvMsgResult,
                         std::span<const uint8_t> response, uint32_t setsSeen);

    std::string requestedLinkState(size_t portIndex) const;

    int handleRequestedLinkStateSet(
        size_t portIndex, const std::string& newState, std::string& current);

    // Coalesce the writes of one or more ports that arrive together into one
    // Set Port Disable Future: each setter arms a short debounce timer and
    // applyPendingLinkStates dispatches once the burst settles.
    void armSetTimer();

    void armSetTimerIfPending();

    void applyPendingLinkStates();

    void handleSetResponse(const std::error_code& ec,
                           std::span<const uint8_t> response);

    uint8_t eid = 0;

    sdbusplus::asio::object_server& objectServer;

    mctp::MctpRequester& mctpRequester;

    // Indexed by port index; port index i is port number i + 1 on the wire.
    std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>>
        linkStateInterfaces;

    std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>>
        associationInterfaces;

    // The mask last read from the device, empty until the device has
    // answered. RequestedLinkState is only ever derived from it.
    std::optional<PortMask> deviceMask;

    // Per port index: the state requested in the current debounce burst, true
    // for disabled. Consumed and cleared when applyPendingLinkStates
    // dispatches.
    std::vector<std::optional<bool>> pendingDisable;

    // Set from dispatching a Set until the device mask has been read back
    // after it, or the device has refused it, so a later burst is never built
    // on a mask that predates it.
    bool setInflight{false};

    // Counts the Sets dispatched. Requests to one endpoint are sent one at a
    // time, so only a Get sent after the latest Set reads a mask that reflects
    // it; a Get sent before it carries an older count.
    uint32_t setsSent{0};

    boost::asio::steady_timer setTimer;

    std::array<uint8_t, gpu::getPortDisableFutureRequestSize> request{};

    bool requestEncoded{false};

    std::array<uint8_t, gpu::setPortDisableFutureRequestSize> setRequest{};
};
