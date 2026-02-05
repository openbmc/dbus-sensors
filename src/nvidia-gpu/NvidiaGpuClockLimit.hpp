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

class NvidiaGpuClockLimit :
    public std::enable_shared_from_this<NvidiaGpuClockLimit>
{
  public:
    NvidiaGpuClockLimit(mctp::MctpRequester& mctpRequester,
                        const std::string& name, uint8_t eid,
                        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
                            operatingConfigIface);

    void update();

  private:
    void handleResponse(const std::error_code& ec,
                        std::span<const uint8_t> buffer);

    mctp::MctpRequester& mctpRequester;
    std::string name;
    uint8_t eid;
    std::shared_ptr<sdbusplus::asio::dbus_interface> operatingConfigInterface;
    std::array<uint8_t, sizeof(gpu::GetClockLimitRequest)> requestBuffer{};
};
