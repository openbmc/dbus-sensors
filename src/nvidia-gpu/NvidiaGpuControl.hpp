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
#include <span>
#include <string>
#include <system_error>

class NvidiaGpuControl : public std::enable_shared_from_this<NvidiaGpuControl>
{
  public:
    NvidiaGpuControl(sdbusplus::asio::object_server& objectServer,
                     const std::string& deviceName,
                     const std::string& inventoryPath,
                     mctp::MctpRequester& mctpRequester, uint8_t eid);

    ~NvidiaGpuControl();

    void update();

    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        getPowerCapInterface() const
    {
        return powerCapInterface;
    }

  private:
    void sendGetPowerLimitsRequest();

    void handleGetPowerLimitsResponse(const std::error_code& ec,
                                      std::span<const uint8_t> buffer);

    std::shared_ptr<sdbusplus::asio::dbus_interface> powerCapInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    uint32_t powerCapValue{0};
    bool powerCapEnabled{false};

    std::string name;
    sdbusplus::asio::object_server& objectServer;
    mctp::MctpRequester& mctpRequester;
    uint8_t eid;

    std::array<uint8_t, sizeof(gpu::GetPowerLimitsRequest)>
        getPowerLimitsRequestBuffer{};
};
