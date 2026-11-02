/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

class NvidiaDeviceModeControl :
    public std::enable_shared_from_this<NvidiaDeviceModeControl>
{
  public:
    NvidiaDeviceModeControl(sdbusplus::asio::object_server& objectServer,
                             const std::string& deviceName,
                             const std::string& inventoryNicPath,
                             mctp::MctpRequester& mctpRequester, uint8_t eid);

    ~NvidiaDeviceModeControl();

    void update();

  private:
    void sendGetRequest(uint32_t deviceModeIndex);
    void handleGetResponse(uint32_t deviceModeIndex,
                           const std::error_code& ec,
                           std::span<const uint8_t> buffer);

    void sendSetRequest(uint32_t deviceModeIndex,
                        std::vector<uint8_t> modeData,
                        std::string pendingDpu, bool pendingEwValid,
                        bool pendingEw);
    void handleSetResponse(std::string pendingDpu, bool pendingEwValid,
                           bool pendingEw, const std::error_code& ec,
                           std::span<const uint8_t> buffer);

    int handleDpuModeSet(const std::string& requested, std::string& current);
    int handleEwEnabledSet(const bool& requested, bool& current);

    std::shared_ptr<sdbusplus::asio::dbus_interface> deviceModeInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    // Requested = pending (writable), Current = active (read-only).
    // Members back the D-Bus getters; updated from GET responses or on
    // successful SET, with signal_property on change.
    std::string dpuRequested{"DPU"};
    std::string dpuCurrent{"DPU"};
    bool ewRequested{false};
    bool ewCurrent{false};
    bool setInflight{false};

    std::string name;
    std::string inventoryNicPath;
    sdbusplus::asio::object_server& objectServer;
    mctp::MctpRequester& mctpRequester;
    uint8_t eid;
};
