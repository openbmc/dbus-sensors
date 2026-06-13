/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/completion.hpp>
#include <sdbusplus/asio/object_server.hpp>

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
        const std::string& deviceName, mctp::MctpRequester& mctpRequester,
        uint8_t eid,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            controlClockSpeedIface);

    ~NvidiaGpuClockSpeedControl();

    void update();

    // Backs the Control.OperatingClockSpeed Reset method. Queues an NSM
    // SET_CLOCK_LIMIT (CLEAR) request and completes the deferred reply once
    // the device responds. Synchronous failures throw an sd-bus error.
    void reset(sdbusplus::asio::completion<> done);

  private:
    void handleResponse(const std::error_code& ec,
                        std::span<const uint8_t> buffer);

    void completeReset(const std::error_code& ec,
                       std::span<const uint8_t> buffer,
                       sdbusplus::asio::completion<> done);

    std::shared_ptr<sdbusplus::asio::dbus_interface> controlClockSpeedInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    mctp::MctpRequester& mctpRequester;
    std::string name;
    sdbusplus::asio::object_server& objectServer;
    uint8_t eid;
    bool resetInFlight{false};
    std::array<uint8_t, gpu::getClockLimitRequestSize> requestBuffer{};
    bool requestEncoded{false};
    std::array<uint8_t, gpu::setClockLimitRequestSize> resetRequestBuffer{};
};
