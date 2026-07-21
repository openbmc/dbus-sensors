/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuNvlinkPortHealth.hpp"

#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <array>
#include <cstdint>
#include <format>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>

// Threshold bitmap bit index -> counter name. Pairing the bit explicitly keeps
// the mapping correct regardless of ordering; the bit indices must match the
// NSM NVLink port health event payload bitmap layout.
static constexpr auto thresholdNames =
    std::to_array<std::pair<uint8_t, std::string_view>>(
        {{0, "port_rcv_errors"},
         {1, "port_xmit_discard"},
         {2, "symbol_ber"},
         {3, "port_rcv_remote_physical_errors"},
         {4, "port_rcv_switch_relay_errors"},
         {5, "effective_ber"},
         {6, "estimated_effective_ber"}});

std::string formatNvlinkPortHealthMessage(
    std::string_view deviceName, uint8_t portNumber, uint32_t thresholdMask)
{
    std::string thresholds;
    for (const auto& [bit, name] : thresholdNames)
    {
        if ((thresholdMask & (static_cast<uint32_t>(1) << bit)) != 0)
        {
            if (!thresholds.empty())
            {
                thresholds += "; ";
            }
            thresholds += name;
        }
    }

    if (thresholds.empty())
    {
        // No known threshold bit (0..6) was set: the mask is zero or only has
        // reserved/future bits. Report the raw bitmap so the event stays
        // diagnosable instead of an empty error-type list.
        thresholds = std::format("{:#010x}", thresholdMask);
    }

    return std::format(
        "The resource property {0} NVLink Port {1} has detected errors of "
        "type [{2}]",
        deviceName, portNumber, thresholds);
}

NvidiaNvlinkPortHealthEventHandler::NvidiaNvlinkPortHealthEventHandler(
    const std::string& deviceName,
    const std::shared_ptr<sdbusplus::asio::connection>& conn) :
    deviceName(deviceName), conn(conn)
{}

void NvidiaNvlinkPortHealthEventHandler::handleLoggingComplete(
    const boost::system::error_code& ec)
{
    if (ec)
    {
        lg2::error(
            "Failed to log NVLink port health event for device {DEV}: {EC}",
            "DEV", deviceName, "EC", ec.message());
    }
}

void NvidiaNvlinkPortHealthEventHandler::handleNvlinkPortHealthEvent(
    const EventInfo& /* eventInfo */, std::span<const uint8_t> eventData)
{
    uint8_t portNumber = 0;
    uint32_t thresholdMask = 0;

    const int rc =
        gpu::decodeNvlinkHealthEvent(eventData, portNumber, thresholdMask);
    if (rc != 0)
    {
        lg2::error("Failed to decode NVLink port health event for device {DEV}",
                   "DEV", deviceName);
        return;
    }

    const std::string eventMessage =
        formatNvlinkPortHealthMessage(deviceName, portNumber, thresholdMask);

    conn->async_method_call(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            std::shared_ptr<NvidiaNvlinkPortHealthEventHandler> self =
                weak.lock();
            if (!self)
            {
                lg2::error(
                    "Invalid reference to NvidiaNvlinkPortHealthEventHandler");
                return;
            }
            self->handleLoggingComplete(ec);
        },
        "xyz.openbmc_project.Logging", "/xyz/openbmc_project/logging",
        "xyz.openbmc_project.Logging.Create", "Create", eventMessage,
        "xyz.openbmc_project.Logging.Entry.Level.Critical",
        std::map<std::string, std::string>{});
}
