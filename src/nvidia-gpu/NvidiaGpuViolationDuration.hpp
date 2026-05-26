/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpRequester.hpp>
#include <MessagePackUnpackUtils.hpp>
#include <NvidiaGpuLongRunningCommand.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <SerialQueue.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>

struct NvidiaGpuViolationDuration
{
  public:
    NvidiaGpuViolationDuration(mctp::MctpRequester& mctpRequester,
                               sdbusplus::asio::object_server& objectServer,
                               const std::string& deviceName, uint8_t eid,
                               std::shared_ptr<SerialQueue> longRunningQueue,
                               std::shared_ptr<NvidiaLongRunningResponseHandler>
                                   longRunningResponseHandler);

    void update();

  private:
    static void onImmediateSuccess(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            powerMetricInterface,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            thermalMetricInterface,
        uint8_t eid, std::span<const uint8_t> buffer);

    static void onLongRunningPayload(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            powerMetricInterface,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            thermalMetricInterface,
        uint8_t eid, UnpackBuffer& buf);

    static void applyDurations(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            powerMetricInterface,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            thermalMetricInterface,
        uint64_t powerViolationNs, uint64_t thermalViolationNs);

    std::shared_ptr<sdbusplus::asio::dbus_interface> powerMetricInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        powerMetricAssociationInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> thermalMetricInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        thermalMetricAssociationInterface;

    std::shared_ptr<NvidiaGpuLongRunningCommand> cmd;
};
