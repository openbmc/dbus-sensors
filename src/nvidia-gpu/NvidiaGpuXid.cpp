/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuXid.hpp"

#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <chrono>
#include <cstdint>
#include <format>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <string_view>

NvidiaXidEventHandler::NvidiaXidEventHandler(
    const std::string& deviceName,
    const std::shared_ptr<sdbusplus::asio::connection>& conn) :
    deviceName(deviceName), conn(conn)
{}

void NvidiaXidEventHandler::handleLoggingComplete(
    const boost::system::error_code& ec)
{
    if (ec)
    {
        lg2::error("Failed to log XID event for device {DEV}: {EC}", "DEV",
                   deviceName, "EC", ec.message());
    }
}

void NvidiaXidEventHandler::handleXidEvent(const EventInfo& /* eventInfo */,
                                           std::span<const uint8_t> eventData)
{
    uint8_t flags = 0;
    uint32_t eventMessageReason = 0;
    uint32_t sequenceNumber = 0;
    uint64_t timestamp = 0;
    std::string_view messageTextString;

    const int rc =
        gpu::decodeXidEvent(eventData, flags, eventMessageReason,
                            sequenceNumber, timestamp, messageTextString);
    if (rc != 0)
    {
        lg2::error("Failed to decode xid event for device {DEV}", "DEV",
                   deviceName);
        return;
    }

    // obtain a human-readable timestamp string from the event's timestamp
    const std::chrono::time_point<std::chrono::system_clock,
                                  std::chrono::nanoseconds>
        timePoint{std::chrono::nanoseconds(timestamp)};
    const std::string timestampString =
        std::format("{:%a %b %d %H:%M:%S UTC %Y}", timePoint);

    const std::string eventMessage = std::format(
        "The resource property {0} Driver Event Message has detected errors of type "
        "[{1}][{2}][{3:x}] XID {4} {5}",
        deviceName, timestampString, sequenceNumber, flags, eventMessageReason,
        messageTextString);

    conn->async_method_call(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaXidEventHandler");
                return;
            }
            self->handleLoggingComplete(ec);
        },
        "xyz.openbmc_project.Logging", "/xyz/openbmc_project/logging",
        "xyz.openbmc_project.Logging.Create", "Create", eventMessage,
        "xyz.openbmc_project.Logging.Entry.Level.Critical",
        std::map<std::string, std::string>{});
}
