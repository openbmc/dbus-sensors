/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuResetRequired.hpp"

#include "NvidiaEventReporting.hpp"

#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <cstdint>
#include <format>
#include <map>
#include <memory>
#include <span>
#include <string>

NvidiaResetRequiredEventHandler::NvidiaResetRequiredEventHandler(
    const std::string& deviceName,
    const std::shared_ptr<sdbusplus::asio::connection>& conn) :
    deviceName(deviceName), conn(conn)
{}

void NvidiaResetRequiredEventHandler::handleLoggingComplete(
    const boost::system::error_code& ec)
{
    if (ec)
    {
        lg2::error("Failed to log ResetRequired event for device {DEV}: {EC}",
                   "DEV", deviceName, "EC", ec.message());
    }
}

void NvidiaResetRequiredEventHandler::handleResetRequiredEvent(
    const EventInfo& /* eventInfo */, std::span<const uint8_t> /* eventData */)
{
    const std::string eventMessage =
        std::format("The resource {} requires a reset. Reset the GPU or power "
                    "cycle the baseboard.",
                    deviceName);

    conn->async_method_call(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            std::shared_ptr<NvidiaResetRequiredEventHandler> self = weak.lock();
            if (!self)
            {
                lg2::error(
                    "Invalid reference to NvidiaResetRequiredEventHandler");
                return;
            }
            self->handleLoggingComplete(ec);
        },
        "xyz.openbmc_project.Logging", "/xyz/openbmc_project/logging",
        "xyz.openbmc_project.Logging.Create", "Create", eventMessage,
        "xyz.openbmc_project.Logging.Entry.Level.Critical",
        std::map<std::string, std::string>{});
}
