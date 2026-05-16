/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
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

    // Status codes emitted on the ResetComplete signal. 0 is success.
    // Low values are NSM reason codes from the device. High values
    // (0xFFxx) are BMC-side transport or decoder failures.
    static constexpr uint16_t resetStatusTransportError = 0xFF01;
    static constexpr uint16_t resetStatusDecodeFailure = 0xFF02;

  private:
    void handleResponse(const std::error_code& ec,
                        std::span<const uint8_t> buffer);

    void reset();
    void emitResetComplete(uint16_t status);

    std::shared_ptr<sdbusplus::asio::dbus_interface> controlClockSpeedInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    mctp::MctpRequester& mctpRequester;
    std::string name;
    sdbusplus::asio::object_server& objectServer;
    uint8_t eid;
    bool resetInFlight{false};
    std::array<uint8_t, gpu::getClockLimitRequestSize> requestBuffer{};
};
