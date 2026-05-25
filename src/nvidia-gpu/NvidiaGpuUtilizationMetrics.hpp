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

struct NvidiaGpuUtilizationMetrics
{
  public:
    NvidiaGpuUtilizationMetrics(
        mctp::MctpRequester& mctpRequester,
        sdbusplus::asio::object_server& objectServer,
        const std::string& deviceName, uint8_t eid,
        std::shared_ptr<SerialQueue> longRunningQueue,
        std::shared_ptr<NvidiaLongRunningResponseHandler>
            longRunningResponseHandler);

    void update();

  private:
    static void onImmediateSuccess(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            processorMetricInterface,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            memoryMetricInterface,
        uint8_t eid, std::span<const uint8_t> buffer);

    static void onLongRunningPayload(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            processorMetricInterface,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            memoryMetricInterface,
        uint8_t eid, std::span<const uint8_t> payload);

    static void applyUtilization(
        const std::shared_ptr<sdbusplus::asio::dbus_interface>& metricInterface,
        uint32_t utilization);

    std::shared_ptr<sdbusplus::asio::dbus_interface> processorMetricInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        processorMetricAssociationInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> memoryMetricInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        memoryMetricAssociationInterface;

    std::shared_ptr<NvidiaGpuLongRunningCommand> cmd;
};
