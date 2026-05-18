/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpRequester.hpp>
#include <NvidiaGpuLongRunningCommand.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <SerialQueue.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>

struct NvidiaGpuEccMode
{
  public:
    NvidiaGpuEccMode(mctp::MctpRequester& mctpRequester,
                     sdbusplus::asio::object_server& objectServer,
                     const std::string& deviceName, uint8_t eid,
                     std::shared_ptr<SerialQueue> longRunningQueue,
                     std::shared_ptr<NvidiaLongRunningResponseHandler>
                         longRunningResponseHandler);

    void update();

  private:
    static void onGetImmediateSuccess(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            eccModeInterface,
        uint8_t eid, std::span<const uint8_t> fullBuffer);

    static void onGetLongRunningPayload(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            eccModeInterface,
        uint8_t eid, std::span<const uint8_t> payload);

    static void applyEccModeToDbus(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            eccModeInterface,
        bool active, bool enabled);

    std::shared_ptr<sdbusplus::asio::dbus_interface> eccModeInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        eccModeAssociationInterface;

    std::shared_ptr<NvidiaGpuLongRunningCommand> getCmd;
};
