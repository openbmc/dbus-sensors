/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpRequester.hpp>
#include <MessagePackUnpackUtils.hpp>
#include <NvidiaGpuLongRunningCommand.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <SerialQueue.hpp>
#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>

struct NvidiaGpuEccMode : public std::enable_shared_from_this<NvidiaGpuEccMode>
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

    void onEnabledSetRequested(bool desired);

    void doSet(SerialQueue::ReleaseHandle handle, bool desired);

    void processSetResponse(SerialQueue::ReleaseHandle handle,
                            const std::error_code& ec,
                            std::span<const uint8_t> buffer);

    void processSetLongRunningResponse(
        boost::system::error_code ec,
        ocp::accelerator_management::CompletionCode cc, uint16_t reasonCode,
        std::span<const uint8_t> responseData);

    // Active is owned by sdbusplus (registered as read-only); the GET
    // pipeline refreshes it via set_property without invoking any
    // setter callback. Enabled is member-backed (following the setter
    // +getter pattern from xyz.openbmc_project.Control.Power.Cap, 90189)
    // so the GET pipeline can refresh it via signal_property without
    // re-triggering the writable setter callback.
    std::shared_ptr<bool> enabledValue;

    std::shared_ptr<sdbusplus::asio::dbus_interface> eccModeInterface;

    uint8_t eid{};

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<SerialQueue> longRunningQueue;

    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler;

    std::shared_ptr<NvidiaGpuLongRunningCommand> getCmd;

    std::array<uint8_t, gpu::setEccModeRequestSize> setRequest{};
};
