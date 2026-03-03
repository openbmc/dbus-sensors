/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
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
        mctp::MctpRequester& mctpRequester, const std::string& deviceName,
        uint8_t eid,
        const std::shared_ptr<NvidiaLongRunningResponseHandler>&
            longRunningResponseHandler,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>&
            operatingConfigInterface);

    void update();

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    void processLongRunningResponse(
        ocp::accelerator_management::CompletionCode cc, uint16_t reasonCode,
        std::span<const uint8_t> responseData);

    uint8_t eid{};

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler;

    std::shared_ptr<sdbusplus::asio::dbus_interface> operatingConfigInterface;

    std::array<uint8_t, sizeof(ocp::accelerator_management::CommonRequest)>
        request{};
};
