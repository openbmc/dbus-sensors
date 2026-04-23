/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <SerialQueue.hpp>
#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>

struct NvidiaGpuCurrentUtilization :
    public std::enable_shared_from_this<NvidiaGpuCurrentUtilization>
{
  public:
    NvidiaGpuCurrentUtilization(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester,
        sdbusplus::asio::object_server& objectServer,
        const std::string& deviceName, uint8_t eid,
        std::shared_ptr<SerialQueue> longRunningQueue,
        std::shared_ptr<NvidiaLongRunningResponseHandler>
            longRunningResponseHandler);

    void update();

  private:
    void doUpdate(SerialQueue::ReleaseHandle handle);

    void processResponse(SerialQueue::ReleaseHandle handle,
                         const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    void processLongRunningResponse(
        boost::system::error_code ec,
        ocp::accelerator_management::CompletionCode cc, uint16_t reasonCode,
        std::span<const uint8_t> responseData);

    void updateUtilization(uint32_t gpuUtilization);

    uint8_t eid{};

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<SerialQueue> longRunningQueue;

    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler;

    std::shared_ptr<sdbusplus::asio::dbus_interface> metricInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> metricAssociationInterface;

    std::array<uint8_t, ocp::accelerator_management::commonRequestSize>
        request{};
};
