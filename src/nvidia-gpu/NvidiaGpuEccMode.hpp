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
    void onGetImmediateSuccess(std::span<const uint8_t> fullBuffer);

    void onGetLongRunningPayload(std::span<const uint8_t> payload);

    void applyEccModeToDbus(bool active, bool enabled);

    // Enabled setter: dispatch a Set ECC Mode for the requested value. The
    // setter never writes eccModeEnabled; that value is refreshed only from a
    // GET response, so a failed SET leaves no stale optimistic value on D-Bus
    // and no failure callback is needed.
    int handleEnabledSet(const bool& newEnable, bool& current);

    void sendSetEccModeRequest(bool enable);

    mctp::MctpRequester& mctpRequester;
    uint8_t eid;
    std::shared_ptr<SerialQueue> longRunningQueue;
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler;

    // Enabled mirrors the device and is updated only from a GET response, so it
    // is the single source of truth: the getter returns it and the setter
    // dispatches a SET without touching it (pattern from
    // xyz.openbmc_project.Control.Power.Cap, 90189). Active is read-only and
    // refreshed via set_property.
    bool eccModeEnabled{false};

    std::shared_ptr<sdbusplus::asio::dbus_interface> eccModeInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        eccModeAssociationInterface;

    // GET refreshes Active/Enabled from hardware; SET dispatches NSM Set ECC
    // Mode for an external write. Both drive their MCTP VDM command through the
    // shared long-running command helper.
    std::shared_ptr<NvidiaGpuLongRunningCommand> getCmd;
    std::shared_ptr<NvidiaGpuLongRunningCommand> setCmd;
};
