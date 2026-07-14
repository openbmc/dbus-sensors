/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "NvidiaEventReporting.hpp"

#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <memory>
#include <span>
#include <string>

class NvidiaResetRequiredEventHandler :
    public std::enable_shared_from_this<NvidiaResetRequiredEventHandler>
{
  public:
    NvidiaResetRequiredEventHandler(
        const std::string& deviceName,
        const std::shared_ptr<sdbusplus::asio::connection>& conn);

    void handleResetRequiredEvent(const EventInfo& eventInfo,
                                  std::span<const uint8_t> eventData);

  private:
    void handleLoggingComplete(const boost::system::error_code& ec);

    std::string deviceName;
    std::shared_ptr<sdbusplus::asio::connection> conn;
};
