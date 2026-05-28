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
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>

class NvidiaGpuPowerControl :
    public std::enable_shared_from_this<NvidiaGpuPowerControl>
{
  public:
    NvidiaGpuPowerControl(
        sdbusplus::asio::object_server& objectServer,
        const std::string& deviceName, mctp::MctpRequester& mctpRequester,
        uint8_t eid, boost::asio::io_context& io,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>& powerCapIface,
        const std::shared_ptr<Inventory>& inventory);

    ~NvidiaGpuPowerControl();

    void update();

  private:
    void sendGetPowerLimitsRequest();

    void handleGetPowerLimitsResponse(const std::error_code& ec,
                                      std::span<const uint8_t> buffer);

    int handlePowerCapSet(const uint32_t& newCap, uint32_t& current);

    int handlePowerCapEnableSet(const bool& newEnable, bool& current);

    // Coalesce the PowerCap and PowerCapEnable writes that make up a single
    // PATCH into one SetPowerLimits: each setter arms a short debounce timer
    // and applyPendingPowerLimit dispatches once the burst settles.
    void armSetLimitTimer();

    void applyPendingPowerLimit();

    void sendSetPowerLimitsRequest(uint32_t milliwatts,
                                   gpu::SetPowerLimitsAction action);

    void handleSetPowerLimitsResponse(const std::error_code& ec,
                                      std::span<const uint8_t> buffer);

    std::shared_ptr<sdbusplus::asio::dbus_interface> powerCapInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
    std::shared_ptr<Inventory> inventory;

    // powerCapValue / powerCapEnabled mirror the device and are only updated
    // from a GetPowerLimits response. requestedPowerCapWatts and pendingEnable
    // hold the cap / enable requested in the current debounce burst; both are
    // consumed and cleared when applyPendingPowerLimit dispatches.
    uint32_t powerCapValue{std::numeric_limits<uint32_t>::max()};
    bool powerCapEnabled{false};
    std::optional<uint32_t> requestedPowerCapWatts;
    std::optional<bool> pendingEnable;
    bool setPowerLimitInflight{false};

    std::string name;
    sdbusplus::asio::object_server& objectServer;
    mctp::MctpRequester& mctpRequester;
    boost::asio::steady_timer setLimitTimer;
    uint8_t eid;

    std::array<uint8_t, gpu::getPowerLimitsRequestSize>
        getPowerLimitsRequestBuffer{};
    std::array<uint8_t, gpu::setPowerLimitsRequestSize>
        setPowerLimitsRequestBuffer{};
    bool requestEncoded{false};
};
