/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuPowerControl.hpp"

#include "Utils.hpp"

#include <Inventory.hpp>
#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

constexpr const char* controlPowerPrefix =
    "/xyz/openbmc_project/control/power/";

constexpr uint32_t milliwattsPerWatt = 1000;
constexpr uint32_t unsetPowerLimit = std::numeric_limits<uint32_t>::max();

// Debounce window to coalesce the PowerCap and PowerCapEnable writes of a
// single PATCH (which arrive back to back) into one SetPowerLimits.
constexpr std::chrono::milliseconds setLimitDebounce{100};

NvidiaGpuPowerControl::NvidiaGpuPowerControl(
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    const std::string& inventoryPath, mctp::MctpRequester& mctpRequester,
    uint8_t eid, boost::asio::io_context& io,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& powerCapIface,
    const std::shared_ptr<Inventory>& inventory) :
    powerCapInterface(powerCapIface), inventory(inventory),
    name(escapeName(deviceName)), objectServer(objectServer),
    mctpRequester(mctpRequester), setLimitTimer(io), eid(eid)
{
    constexpr uint32_t devicePowerLimitId = 0;
    const int rc = gpu::encodeGetPowerLimitsRequest(
        0, devicePowerLimitId, getPowerLimitsRequestBuffer);
    if (rc == 0)
    {
        requestEncoded = true;
    }
    else
    {
        lg2::error(
            "Failed to encode GET_POWER_LIMITS request for eid {EID}, rc={RC}",
            "EID", eid, "RC", rc);
    }

    const std::string powerControlPath = controlPowerPrefix + name;

    associationInterface =
        objectServer.add_interface(powerControlPath, association::interface);

    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);

    associationInterface->register_property("Associations", associations);
    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association interface for GPU Control for {NAME}, eid={EID}",
            "NAME", name, "EID", eid);
    }

    powerCapInterface->register_property<uint32_t>(
        "PowerCap", std::numeric_limits<uint32_t>::max(),
        std::bind_front(&NvidiaGpuPowerControl::handlePowerCapSet, this),
        [this](uint32_t&) { return powerCapValue; });
    powerCapInterface->register_property<std::string>(
        "PowerCapEnable", powerCapEnableUnknown,
        std::bind_front(&NvidiaGpuPowerControl::handlePowerCapEnableSet, this),
        [this](std::string&) { return powerCapEnabled; });
    if (!powerCapInterface->initialize())
    {
        lg2::error(
            "Error initializing Power Cap interface for {NAME}, eid={EID}",
            "NAME", name, "EID", eid);
    }
}

NvidiaGpuPowerControl::~NvidiaGpuPowerControl()
{
    objectServer.remove_interface(associationInterface);
}

void NvidiaGpuPowerControl::update()
{
    sendGetPowerLimitsRequest();
}

void NvidiaGpuPowerControl::sendGetPowerLimitsRequest()
{
    if (!requestEncoded)
    {
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, getPowerLimitsRequestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuPowerControl> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuPowerControl reference");
                return;
            }
            self->handleGetPowerLimitsResponse(ec, buffer);
        });
}

void NvidiaGpuPowerControl::handleGetPowerLimitsResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error getting power limits for eid {EID}: {MSG}", "EID",
                   eid, "MSG", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t persistentPowerLimit = 0;
    uint32_t oneshotPowerLimit = 0;
    uint32_t enforcedLimit = 0;

    const int rc = gpu::decodeGetPowerLimitsResponse(
        buffer, cc, reasonCode, persistentPowerLimit, oneshotPowerLimit,
        enforcedLimit);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding power limits for eid {EID}: rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
            reasonCode);
        return;
    }

    // PDI defines PowerCap as the user-specified value (maxint if the
    // user has not set one). The Set path only ever writes the one-shot
    // limit, so the read path mirrors that and reports the one-shot
    // requested limit (maxint if unset), ignoring the persistent limit.
    const uint32_t newPowerCapValue =
        (oneshotPowerLimit != unsetPowerLimit)
            ? oneshotPowerLimit / milliwattsPerWatt
            : unsetPowerLimit;

    // PowerCapEnable is Enabled only when the one-shot cap is the limit
    // currently being enforced. Guard against the case where enforcedLimit
    // is the unset sentinel: a matching unset one-shot would otherwise
    // compare equal and incorrectly report the cap as enabled.
    const std::string newPowerCapEnabled =
        ((enforcedLimit != unsetPowerLimit) &&
         (oneshotPowerLimit == enforcedLimit))
            ? powerCapEnableEnabled
            : powerCapEnableDisabled;

    // Only emit PropertiesChanged when the value actually changes; this
    // handler runs on every poll.
    if (newPowerCapValue != powerCapValue)
    {
        powerCapValue = newPowerCapValue;
        powerCapInterface->signal_property("PowerCap");
    }

    if (newPowerCapEnabled != powerCapEnabled)
    {
        powerCapEnabled = newPowerCapEnabled;
        powerCapInterface->signal_property("PowerCapEnable");
    }
}

int NvidiaGpuPowerControl::handlePowerCapSet(const uint32_t& newCap,
                                             uint32_t& /*current*/)
{
    if (!inventory)
    {
        lg2::error(
            "PowerCap set rejected for eid {EID}: inventory not available",
            "EID", eid);
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    std::optional<uint32_t> min = inventory->getMinPowerCap();
    std::optional<uint32_t> max = inventory->getMaxPowerCap();
    if (!min || !max)
    {
        lg2::error("PowerCap set rejected for eid {EID}: min/max not yet known",
                   "EID", eid);
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    if (newCap < *min || newCap > *max)
    {
        lg2::error(
            "PowerCap set rejected for eid {EID}: value {VAL} out of range [{MIN}, {MAX}]",
            "EID", eid, "VAL", newCap, "MIN", *min, "MAX", *max);
        throw sdbusplus::error::xyz::openbmc_project::common::InvalidArgument();
    }

    // Record the requested cap and arm the debounce timer. The actual
    // SetPowerLimits is issued once the burst settles, so a PATCH that sets
    // both PowerCap and PowerCapEnable is coalesced into a single request.
    // Do not touch powerCapValue here: the D-Bus value is only updated once
    // the device confirms via a GetPowerLimits response.
    requestedPowerCapWatts = newCap;
    armSetLimitTimer();

    return 1;
}

int NvidiaGpuPowerControl::handlePowerCapEnableSet(const std::string& newEnable,
                                                   std::string& /*current*/)
{
    if (newEnable != powerCapEnableEnabled &&
        newEnable != powerCapEnableDisabled)
    {
        throw sdbusplus::error::xyz::openbmc_project::common::InvalidArgument();
    }
    // Record the requested enable state and arm the debounce timer. Do not
    // touch powerCapEnabled here: the D-Bus value is only updated from a
    // GetPowerLimits response.
    pendingEnable = (newEnable == powerCapEnableEnabled);
    armSetLimitTimer();

    return 1;
}

void NvidiaGpuPowerControl::armSetLimitTimer()
{
    setLimitTimer.expires_after(setLimitDebounce);
    setLimitTimer.async_wait(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                // Superseded by a later set within the debounce window.
                return;
            }
            std::shared_ptr<NvidiaGpuPowerControl> self = weak.lock();
            if (!self)
            {
                return;
            }
            self->applyPendingPowerLimit();
        });
}

void NvidiaGpuPowerControl::applyPendingPowerLimit()
{
    if (setPowerLimitInflight)
    {
        // A previous SetPowerLimits is still in flight; retry once it lands.
        armSetLimitTimer();
        return;
    }

    const bool enable =
        pendingEnable.value_or(powerCapEnabled == powerCapEnableEnabled);
    const bool enableChanged = pendingEnable.has_value();

    // Snapshot the cap to enforce before clearing: prefer the explicitly
    // requested cap, otherwise fall back to the device's current value.
    // powerCapValue is the D-Bus sentinel (unset) when the device reports
    // no cap.
    std::optional<uint32_t> capToEnforce = requestedPowerCapWatts;
    if (!capToEnforce && powerCapValue != unsetPowerLimit)
    {
        capToEnforce = powerCapValue;
    }

    // The burst is consumed; clear both pending requests so a cap set while
    // disabled is not silently re-applied on a later, unrelated enable.
    pendingEnable.reset();
    requestedPowerCapWatts.reset();

    if (enable)
    {
        if (!capToEnforce)
        {
            // Nothing to enforce. Skip rather than send the unset sentinel
            // (which would overflow when scaled to milliwatts).
            lg2::error(
                "PowerCapEnable set ignored for eid {EID}: no power cap value available",
                "EID", eid);
            return;
        }
        sendSetPowerLimitsRequest(*capToEnforce * milliwattsPerWatt,
                                  gpu::SetPowerLimitsAction::NEW_LIMIT);
    }
    else if (enableChanged)
    {
        // Explicit disable: revert to the firmware default limit.
        sendSetPowerLimitsRequest(0, gpu::SetPowerLimitsAction::DEFAULT_LIMIT);
    }
    // else: disabled and only the cap changed; nothing to enforce.
}

void NvidiaGpuPowerControl::sendSetPowerLimitsRequest(
    uint32_t milliwatts, gpu::SetPowerLimitsAction action)
{
    constexpr uint32_t devicePowerLimitId = 0;

    const int rc = gpu::encodeSetPowerLimitsRequest(
        0, devicePowerLimitId, action, gpu::SetPowerLimitsPersistence::ONE_SHOT,
        milliwatts, setPowerLimitsRequestBuffer);

    if (rc != 0)
    {
        lg2::error(
            "Error encoding SET_POWER_LIMITS request for eid {EID}, rc={RC}",
            "EID", eid, "RC", rc);
        return;
    }

    setPowerLimitInflight = true;

    mctpRequester.sendRecvMsg(
        eid, setPowerLimitsRequestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuPowerControl> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuPowerControl reference");
                return;
            }
            self->handleSetPowerLimitsResponse(ec, buffer);
        });
}

void NvidiaGpuPowerControl::handleSetPowerLimitsResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    setPowerLimitInflight = false;

    if (ec)
    {
        lg2::error("Error setting power limits for eid {EID}: {MSG}", "EID",
                   eid, "MSG", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    const int rc = gpu::decodeSetPowerLimitsResponse(buffer, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding SetPowerLimits response for eid {EID}: rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
            reasonCode);
        return;
    }

    // The set succeeded; re-read from the device so powerCapValue /
    // powerCapEnabled reflect the new state without waiting for the next
    // periodic poll.
    sendGetPowerLimitsRequest();
}
