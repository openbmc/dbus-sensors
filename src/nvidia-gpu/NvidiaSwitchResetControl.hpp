/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <boost/asio/spawn.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <string>

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

    // Registers the Reset method and initializes the D-Bus interfaces.
    // Must be called after the object is owned by a shared_ptr so that the
    // method handler can safely capture a weak_ptr.
    void init();

  private:
    void sendResetRequest(const boost::asio::yield_context& yield);

    sdbusplus::asio::object_server& objectServer;
    mctp::MctpRequester& mctpRequester;
    uint8_t eid{};
    std::string name;
    std::shared_ptr<sdbusplus::asio::dbus_interface> resetInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
};
