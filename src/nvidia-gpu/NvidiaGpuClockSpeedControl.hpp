/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>

class NvidiaGpuClockSpeedControl :
    public std::enable_shared_from_this<NvidiaGpuClockSpeedControl>
{
  public:
    NvidiaGpuClockSpeedControl(
        sdbusplus::asio::object_server& objectServer,
        const std::string& deviceName, const std::string& inventoryPath,
        mctp::MctpRequester& mctpRequester, uint8_t eid,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            controlClockSpeedIface);

    ~NvidiaGpuClockSpeedControl();

    void update();

    // Backs the Control.OperatingClockSpeed Reset method. Queues an NSM
    // SET_CLOCK_LIMIT (CLEAR) request and defers the D-Bus reply until the
    // device responds: the call message is handed to MctpRequester, which
    // sends the reply once the MCTP round-trip completes. Synchronous
    // failures (a reset already in flight, or an encode error) throw, so
    // sd-bus replies with the corresponding D-Bus error immediately.
    void reset(sdbusplus::message_t msg);

  private:
    void handleResponse(const std::error_code& ec,
                        std::span<const uint8_t> buffer);

    // MctpRequester callback for the Reset round-trip. Clears the in-flight
    // guard and logs a transport or NSM completion-code failure; the D-Bus
    // reply itself is sent by MctpRequester from the call message.
    void completeReset(const std::error_code& ec,
                       std::span<const uint8_t> buffer);

    std::shared_ptr<sdbusplus::asio::dbus_interface> controlClockSpeedInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    mctp::MctpRequester& mctpRequester;
    std::string name;
    sdbusplus::asio::object_server& objectServer;
    uint8_t eid;
    bool resetInFlight{false};
    std::array<uint8_t, gpu::getClockLimitRequestSize> requestBuffer{};
    std::array<uint8_t, gpu::setClockLimitRequestSize> resetRequestBuffer{};
};
