/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>

class NvidiaSwitchResetControl :
    public std::enable_shared_from_this<NvidiaSwitchResetControl>
{
  public:
    NvidiaSwitchResetControl(sdbusplus::asio::object_server& objectServer,
                             mctp::MctpRequester& mctpRequester,
                             const std::string& deviceName,
                             const std::string& inventoryPath, uint8_t eid);

    NvidiaSwitchResetControl(const NvidiaSwitchResetControl&) = delete;
    NvidiaSwitchResetControl& operator=(const NvidiaSwitchResetControl&) =
        delete;
    NvidiaSwitchResetControl(NvidiaSwitchResetControl&&) = delete;
    NvidiaSwitchResetControl& operator=(NvidiaSwitchResetControl&&) = delete;

    ~NvidiaSwitchResetControl();

    void init();

  private:
    int handlePendingResetSet(const std::string& newValue,
                              std::string& currentValue);

    void sendResetRequest();

    void handleResetResponse(const std::error_code& ec,
                             std::span<const uint8_t> response);

    sdbusplus::asio::object_server& objectServer;
    mctp::MctpRequester& mctpRequester;
    uint8_t eid{};
    std::string name;
    std::string pendingResetValue;
    std::array<uint8_t, gpu::resetNetworkDeviceRequestSize>
        resetRequestBuffer{};
    bool requestEncoded{false};
    std::shared_ptr<sdbusplus::asio::dbus_interface> resetInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
};
