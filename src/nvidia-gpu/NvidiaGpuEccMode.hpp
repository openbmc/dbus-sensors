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
        const std::shared_ptr<bool>& enabledValue, uint8_t eid,
        std::span<const uint8_t> fullBuffer);

    static void onGetLongRunningPayload(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            eccModeInterface,
        const std::shared_ptr<bool>& enabledValue, uint8_t eid,
        std::span<const uint8_t> payload);

    static void applyEccModeToDbus(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            eccModeInterface,
        const std::shared_ptr<bool>& enabledValue, bool active, bool enabled);

    // Enabled is member-backed (following the setter+getter pattern from
    // xyz.openbmc_project.Control.Power.Cap, 90189) so the GET pipeline can
    // refresh it via signal_property without re-triggering the writable
    // setter callback. Active is owned by sdbusplus (read-only) and refreshed
    // via set_property.
    std::shared_ptr<bool> enabledValue;

    std::shared_ptr<sdbusplus::asio::dbus_interface> eccModeInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        eccModeAssociationInterface;

    // GET refreshes Active/Enabled from hardware; SET dispatches NSM Set ECC
    // Mode for an external write. Both drive their MCTP VDM command through
    // the shared long-running command helper.
    std::shared_ptr<NvidiaGpuLongRunningCommand> getCmd;
    std::shared_ptr<NvidiaGpuLongRunningCommand> setCmd;
};
