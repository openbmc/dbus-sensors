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

namespace gpu
{

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

// Bitmask of every threshold bit that has a known name. Derived from
// thresholdNames so it stays in sync if the table changes; any bit set outside
// this mask is a reserved/future bit with no name.
static constexpr uint32_t knownThresholdMask = []() {
    uint32_t mask = 0;
    for (const auto& entry : thresholdNames)
    {
        mask |= static_cast<uint32_t>(1) << entry.first;
    }
    return mask;
}();

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

    // Append the raw bitmap whenever the mask carries bits outside the known
    // 0..6 range (reserved/future bits), or when nothing was recognised at all.
    // This keeps reserved bits diagnosable even in the mixed case (e.g. 0x84,
    // where a known bit would otherwise hide the reserved bit 7) instead of
    // silently dropping them.
    if ((thresholdMask & ~knownThresholdMask) != 0 || thresholds.empty())
    {
        if (!thresholds.empty())
        {
            thresholds += "; ";
        }
        thresholds += std::format("{:#010x}", thresholdMask);
    }

    return std::format(
        "The resource property {0} NVLink Port {1} has detected errors of "
        "type [{2}]",
        deviceName, portNumber, thresholds);
}

} // namespace gpu

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

    const std::string eventMessage = gpu::formatNvlinkPortHealthMessage(
        deviceName, portNumber, thresholdMask);

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
