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
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>

class NvidiaGpuControl : public std::enable_shared_from_this<NvidiaGpuControl>
{
  public:
    NvidiaGpuControl(
        sdbusplus::asio::object_server& objectServer,
        const std::string& deviceName, const std::string& inventoryPath,
        mctp::MctpRequester& mctpRequester, uint8_t eid,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>& powerCapIface,
        std::shared_ptr<Inventory> inventory);

    ~NvidiaGpuControl();

    void update();

  private:
    void sendGetPowerLimitsRequest();

    void handleGetPowerLimitsResponse(const std::error_code& ec,
                                      std::span<const uint8_t> buffer);

    int handlePowerCapSet(const uint32_t& newCap, uint32_t& current);

    int handlePowerCapEnableSet(const bool& newEnable, bool& current);

    void sendSetPowerLimitsRequest(uint32_t milliwatts,
                                   gpu::SetPowerLimitsAction action);

    void handleSetPowerLimitsResponse(const std::error_code& ec,
                                      std::span<const uint8_t> buffer);

    std::shared_ptr<sdbusplus::asio::dbus_interface> powerCapInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
    std::shared_ptr<Inventory> inventory;

    uint32_t powerCapValue{std::numeric_limits<uint32_t>::max()};
    bool powerCapEnabled{false};
    bool setPowerLimitInflight{false};

    std::string name;
    sdbusplus::asio::object_server& objectServer;
    mctp::MctpRequester& mctpRequester;
    uint8_t eid;

    std::array<uint8_t, gpu::getPowerLimitsRequestSize>
        getPowerLimitsRequestBuffer{};
    std::array<uint8_t, gpu::setPowerLimitsRequestSize>
        setPowerLimitsRequestBuffer{};
};
