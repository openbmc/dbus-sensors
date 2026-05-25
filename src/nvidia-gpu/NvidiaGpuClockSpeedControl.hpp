/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Inventory.hpp"
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
            controlClockSpeedIface,
        std::shared_ptr<Inventory> inventory);

    ~NvidiaGpuClockSpeedControl();

    void update();

  private:
    void handleResponse(const std::error_code& ec,
                        std::span<const uint8_t> buffer);

    int handleRequestedSpeedLimitMaxHzSet(const uint64_t& newMaxHz,
                                          uint64_t& current);
    int handleRequestedSpeedLimitMinHzSet(const uint64_t& newMinHz,
                                          uint64_t& current);

    void sendSetClockLimitRequest(bool* inflightFlag, uint32_t limitMinMHz,
                                  uint32_t limitMaxMHz);
    void handleSetClockLimitResponse(bool* inflightFlag,
                                     const std::error_code& ec,
                                     std::span<const uint8_t> buffer);

    std::shared_ptr<sdbusplus::asio::dbus_interface> controlClockSpeedInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
    std::shared_ptr<Inventory> inventory;

    uint64_t presentMaxHz{0};
    uint64_t presentMinHz{0};
    uint64_t requestedMaxHz{0};
    uint64_t requestedMinHz{0};
    bool requestedMaxHzKnown{false};
    bool requestedMinHzKnown{false};
    bool requestedMaxHzInflight{false};
    bool requestedMinHzInflight{false};

    mctp::MctpRequester& mctpRequester;
    std::string name;
    sdbusplus::asio::object_server& objectServer;
    uint8_t eid;
    std::array<uint8_t, gpu::getClockLimitRequestSize> requestBuffer{};
    std::array<uint8_t, gpu::setClockLimitRequestSize>
        setClockLimitRequestBuffer{};
};
