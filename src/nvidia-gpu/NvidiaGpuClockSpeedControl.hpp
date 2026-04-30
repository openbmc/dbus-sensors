/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <boost/asio/spawn.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

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

  private:
    void handleResponse(const std::error_code& ec,
                        std::span<const uint8_t> buffer);

    void reset(const boost::asio::yield_context& yield);
    std::shared_ptr<sdbusplus::asio::dbus_interface> controlClockSpeedInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    mctp::MctpRequester& mctpRequester;
    std::string name;
    sdbusplus::asio::object_server& objectServer;
    uint8_t eid;
    std::array<uint8_t, gpu::getClockLimitRequestSize> requestBuffer{};
};
