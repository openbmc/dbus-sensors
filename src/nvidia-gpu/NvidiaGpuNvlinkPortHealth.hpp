/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "NvidiaEventReporting.hpp"

#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>

class NvidiaNvlinkPortHealthEventHandler :
    public std::enable_shared_from_this<NvidiaNvlinkPortHealthEventHandler>
{
  public:
    NvidiaNvlinkPortHealthEventHandler(
        const std::string& deviceName,
        const std::shared_ptr<sdbusplus::asio::connection>& conn);

    void handleNvlinkPortHealthEvent(const EventInfo& eventInfo,
                                     std::span<const uint8_t> eventData);

  private:
    void handleLoggingComplete(const boost::system::error_code& ec);

    std::string deviceName;
    std::shared_ptr<sdbusplus::asio::connection> conn;
};

// Build the message logged for an NVLink port health event. This is the
// string that surfaces as the Redfish EventLog entry Message property.
// Exposed as a free function so it can be unit tested without a D-Bus
// connection.
std::string formatNvlinkPortHealthMessage(
    std::string_view deviceName, uint8_t portNumber, uint32_t thresholdMask);
