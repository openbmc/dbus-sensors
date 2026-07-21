/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

struct NvidiaIbPortMetrics
{
  public:
    NvidiaIbPortMetrics(
        const std::string& name, const std::string& deviceName, uint8_t eid,
        uint16_t portNumber, sdbusplus::asio::object_server& objectServer,
        const std::vector<std::pair<uint8_t, uint64_t>>& addresses);

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> portInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface>
        networkDeviceFunctionInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface>
        networkDeviceFunctionAssociationInterface;
};
