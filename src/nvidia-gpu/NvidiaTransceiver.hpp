/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <memory>

// The module plugged into one port of a network device.
struct NvidiaTransceiver
{
  public:
    NvidiaTransceiver(sdbusplus::asio::object_server& objectServer,
                      const sdbusplus::object_path& transceiverPath,
                      const sdbusplus::object_path& portPath, uint8_t eid,
                      uint16_t portNumber);

    ~NvidiaTransceiver();

    NvidiaTransceiver(const NvidiaTransceiver&) = delete;
    NvidiaTransceiver& operator=(const NvidiaTransceiver&) = delete;
    NvidiaTransceiver(NvidiaTransceiver&&) = delete;
    NvidiaTransceiver& operator=(NvidiaTransceiver&&) = delete;

  private:
    sdbusplus::asio::object_server& objectServer;

    std::shared_ptr<sdbusplus::asio::dbus_interface> transceiverInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> itemInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
};
