/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <Inventory.hpp>
#include <MctpRequester.hpp>
#include <NvidiaGpuLongRunningCommand.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <SerialQueue.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>

struct NvidiaGpuMemoryCapacityUtilization
{
  public:
    NvidiaGpuMemoryCapacityUtilization(
        mctp::MctpRequester& mctpRequester,
        sdbusplus::asio::object_server& objectServer,
        const std::string& deviceName, uint8_t eid,
        std::shared_ptr<SerialQueue> longRunningQueue,
        std::shared_ptr<NvidiaLongRunningResponseHandler>
            longRunningResponseHandler,
        std::shared_ptr<Inventory> inventory);

    void update();

  private:
    static void onImmediateSuccess(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            processorMetricInterface,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            dramMetricInterface,
        const std::shared_ptr<Inventory>& inventory, uint8_t eid,
        std::span<const uint8_t> buffer);

    static void onLongRunningPayload(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            processorMetricInterface,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            dramMetricInterface,
        const std::shared_ptr<Inventory>& inventory, uint8_t eid,
        std::span<const uint8_t> payload);

    static void applyUtilization(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            processorMetricInterface,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            dramMetricInterface,
        const std::shared_ptr<Inventory>& inventory, uint32_t reservedMemory,
        uint32_t usedMemory);

    std::shared_ptr<Inventory> inventory;

    std::shared_ptr<sdbusplus::asio::dbus_interface> processorMetricInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        processorMetricAssociationInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> dramMetricInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        dramMetricAssociationInterface;

    std::shared_ptr<NvidiaGpuLongRunningCommand> cmd;
};
