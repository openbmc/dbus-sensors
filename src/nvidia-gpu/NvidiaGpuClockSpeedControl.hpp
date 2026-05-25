/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Inventory.hpp"
#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

class NvidiaGpuClockSpeedControl :
    public std::enable_shared_from_this<NvidiaGpuClockSpeedControl>
{
  public:
    NvidiaGpuClockSpeedControl(
        sdbusplus::asio::object_server& objectServer,
        const std::string& deviceName, const std::string& inventoryPath,
        mctp::MctpRequester& mctpRequester, uint8_t eid,
        boost::asio::io_context& io,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            controlClockSpeedIface,
        std::shared_ptr<Inventory> inventory);

    ~NvidiaGpuClockSpeedControl();

    void update();

  private:
    void sendGetClockLimitRequest();
    void handleGetClockLimitResponse(const std::error_code& ec,
                                     std::span<const uint8_t> buffer);

    int handleRequestedSpeedLimitMaxHzSet(const uint64_t& newMaxHz,
                                          uint64_t& current);
    int handleRequestedSpeedLimitMinHzSet(const uint64_t& newMinHz,
                                          uint64_t& current);

    // Coalesce the RequestedSpeedLimitMinHz and RequestedSpeedLimitMaxHz
    // writes that make up a single PATCH into one SetClockLimit: each setter
    // arms a short debounce timer and applyPendingClockLimit dispatches
    // once the burst settles.
    void armSetLimitTimer();

    void applyPendingClockLimit();

    void sendSetClockLimitRequest(uint32_t limitMinMHz, uint32_t limitMaxMHz);
    void handleSetClockLimitResponse(const std::error_code& ec,
                                     std::span<const uint8_t> buffer);

    std::shared_ptr<sdbusplus::asio::dbus_interface> controlClockSpeedInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
    std::shared_ptr<Inventory> inventory;

    // presentMaxHz / presentMinHz / requestedMaxHz / requestedMinHz mirror
    // the device and are only updated from a GetClockLimit response.
    // pendingMaxHz / pendingMinHz hold the values requested in the current
    // burst, if any; they are reset once dispatched.
    uint64_t presentMaxHz{0};
    uint64_t presentMinHz{0};
    uint64_t requestedMaxHz{0};
    uint64_t requestedMinHz{0};
    std::optional<uint64_t> pendingMaxHz;
    std::optional<uint64_t> pendingMinHz;
    bool setClockLimitInflight{false};
    // True while the GetClockLimit currently in flight is the read-back issued
    // after a successful SetClockLimit (as opposed to a periodic poll). Only
    // that read-back response clears setClockLimitInflight, so the coalesce
    // gate stays closed until the device's new state has been read back.
    bool setClockLimitReadbackPending{false};

    mctp::MctpRequester& mctpRequester;
    std::string name;
    sdbusplus::asio::object_server& objectServer;
    boost::asio::steady_timer setLimitTimer;
    uint8_t eid;
    std::array<uint8_t, gpu::getClockLimitRequestSize> requestBuffer{};
    bool requestEncoded{false};
    std::array<uint8_t, gpu::setClockLimitRequestSize>
        setClockLimitRequestBuffer{};
};
