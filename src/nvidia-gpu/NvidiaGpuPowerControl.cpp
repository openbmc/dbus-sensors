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
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

constexpr const char* controlPowerPrefix =
    "/xyz/openbmc_project/control/power/";

constexpr uint32_t milliwattsPerWatt = 1000;
constexpr uint32_t unsetPowerLimit = std::numeric_limits<uint32_t>::max();

NvidiaGpuPowerControl::NvidiaGpuPowerControl(
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    const std::string& inventoryPath, mctp::MctpRequester& mctpRequester,
    uint8_t eid,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& powerCapIface,
    std::shared_ptr<Inventory> inventory) :
    powerCapInterface(powerCapIface), inventory(std::move(inventory)),
    name(escapeName(deviceName)), objectServer(objectServer),
    mctpRequester(mctpRequester), eid(eid)
{
    const std::string powerControlPath = controlPowerPrefix + name;

    associationInterface =
        objectServer.add_interface(powerControlPath, association::interface);

    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);

    associationInterface->register_property("Associations", associations);
    associationInterface->initialize();

    powerCapInterface->register_property<uint32_t>(
        "PowerCap", std::numeric_limits<uint32_t>::max(),
        std::bind_front(&NvidiaGpuPowerControl::handlePowerCapSet, this),
        [this](uint32_t&) { return powerCapValue; });
    powerCapInterface->register_property<bool>(
        "PowerCapEnable", false,
        std::bind_front(&NvidiaGpuPowerControl::handlePowerCapEnableSet, this),
        [this](bool&) { return powerCapEnabled; });
    powerCapInterface->initialize();
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
    constexpr uint32_t devicePowerLimitId = 0;

    const int rc = gpu::encodeGetPowerLimitsRequest(
        0, devicePowerLimitId, getPowerLimitsRequestBuffer);

    if (rc != 0)
    {
        lg2::error(
            "Error encoding GET_POWER_LIMITS request for eid {EID}, rc={RC}",
            "EID", eid, "RC", rc);
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

    // PowerCapEnable is true only when the one-shot cap is the limit
    // currently being enforced. Guard against the case where enforcedLimit
    // is the unset sentinel: a matching unset one-shot would otherwise
    // compare equal and incorrectly report the cap as enabled.
    const bool newPowerCapEnabled = (enforcedLimit != unsetPowerLimit) &&
                                    (oneshotPowerLimit == enforcedLimit);

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

    if (setPowerLimitInflight)
    {
        lg2::error(
            "PowerCap set rejected for eid {EID}: SetPowerLimits in flight",
            "EID", eid);
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    // Remember the requested cap but do not touch powerCapValue here: the
    // D-Bus value is only updated once the device confirms via a
    // GetPowerLimits response. If the cap is currently being enforced, push
    // the new value to the device now.
    requestedPowerCapWatts = newCap;
    if (powerCapEnabled)
    {
        sendSetPowerLimitsRequest(newCap * milliwattsPerWatt,
                                  gpu::SetPowerLimitsAction::NEW_LIMIT);
    }

    return 1;
}

int NvidiaGpuPowerControl::handlePowerCapEnableSet(const bool& newEnable,
                                                   bool& /*current*/)
{
    if (setPowerLimitInflight)
    {
        lg2::error(
            "PowerCapEnable set rejected for eid {EID}: SetPowerLimits in flight",
            "EID", eid);
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    // Enforce the last explicitly requested cap. If the user never set one,
    // fall back to the device's current value. Do not touch powerCapEnabled
    // here: the D-Bus value is only updated from a GetPowerLimits response.
    if (newEnable)
    {
        const uint32_t capToEnforce =
            (requestedPowerCapWatts != unsetPowerLimit)
                ? requestedPowerCapWatts
                : powerCapValue;
        sendSetPowerLimitsRequest(capToEnforce * milliwattsPerWatt,
                                  gpu::SetPowerLimitsAction::NEW_LIMIT);
    }
    else
    {
        sendSetPowerLimitsRequest(0, gpu::SetPowerLimitsAction::DEFAULT_LIMIT);
    }

    return 1;
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
