/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuControl.hpp"

#include "Utils.hpp"

#include <Inventory.hpp>
#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/exception.hpp>

#include <cerrno>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

constexpr const char* controlPowerPrefix =
    "/xyz/openbmc_project/control/power/";

constexpr uint32_t milliwattsPerWatt = 1000;

NvidiaGpuControl::NvidiaGpuControl(
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
        std::bind_front(&NvidiaGpuControl::handlePowerCapSet, this));
    powerCapInterface->register_property<bool>(
        "PowerCapEnable", false,
        std::bind_front(&NvidiaGpuControl::handlePowerCapEnableSet, this));
    powerCapInterface->initialize();
}

NvidiaGpuControl::~NvidiaGpuControl()
{
    objectServer.remove_interface(associationInterface);
}

void NvidiaGpuControl::update()
{
    sendGetPowerLimitsRequest();
}

void NvidiaGpuControl::sendGetPowerLimitsRequest()
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
            std::shared_ptr<NvidiaGpuControl> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuControl reference");
                return;
            }
            self->handleGetPowerLimitsResponse(ec, buffer);
        });
}

void NvidiaGpuControl::handleGetPowerLimitsResponse(
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

    // PDI specifies PowerCap is in Watts; device reports milliwatts, so
    // convert.
    powerCapValue = enforcedLimit / milliwattsPerWatt;
    powerCapEnabled = (persistentPowerLimit == enforcedLimit) ||
                      (oneshotPowerLimit == enforcedLimit);

    internalSet = true;
    powerCapInterface->set_property("PowerCap", powerCapValue);
    powerCapInterface->set_property("PowerCapEnable", powerCapEnabled);
    internalSet = false;
}

int NvidiaGpuControl::handlePowerCapSet(const uint32_t& newCap,
                                        uint32_t& current)
{
    if (internalSet)
    {
        current = newCap;
        return 1;
    }

    auto min = inventory->getMinPowerCap();
    auto max = inventory->getMaxPowerCap();
    if (!min || !max)
    {
        lg2::error("PowerCap set rejected for eid {EID}: min/max not yet known",
                   "EID", eid);
        throw sdbusplus::exception::SdBusError(EBUSY, "min/max not yet known");
    }

    if (newCap < *min || newCap > *max)
    {
        lg2::error(
            "PowerCap set rejected for eid {EID}: value {VAL} out of range [{MIN}, {MAX}]",
            "EID", eid, "VAL", newCap, "MIN", *min, "MAX", *max);
        throw sdbusplus::exception::SdBusError(EINVAL, "value out of range");
    }

    if (setPowerLimitInflight)
    {
        lg2::error(
            "PowerCap set rejected for eid {EID}: SetPowerLimits in flight",
            "EID", eid);
        throw sdbusplus::exception::SdBusError(EBUSY,
                                               "SetPowerLimits in flight");
    }

    powerCapValue = newCap;
    if (powerCapEnabled)
    {
        sendSetPowerLimitsRequest(newCap * milliwattsPerWatt,
                                  gpu::SetPowerLimitsAction::NEW_LIMIT);
    }

    current = newCap;
    return 1;
}

int NvidiaGpuControl::handlePowerCapEnableSet(const bool& newEnable,
                                              bool& current)
{
    if (internalSet)
    {
        current = newEnable;
        return 1;
    }

    if (setPowerLimitInflight)
    {
        lg2::error(
            "PowerCapEnable set rejected for eid {EID}: SetPowerLimits in flight",
            "EID", eid);
        throw sdbusplus::exception::SdBusError(EBUSY,
                                               "SetPowerLimits in flight");
    }

    if (newEnable)
    {
        sendSetPowerLimitsRequest(powerCapValue * milliwattsPerWatt,
                                  gpu::SetPowerLimitsAction::NEW_LIMIT);
    }
    else
    {
        sendSetPowerLimitsRequest(0, gpu::SetPowerLimitsAction::DEFAULT_LIMIT);
    }

    powerCapEnabled = newEnable;
    current = newEnable;
    return 1;
}

void NvidiaGpuControl::sendSetPowerLimitsRequest(
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
            std::shared_ptr<NvidiaGpuControl> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuControl reference");
                return;
            }
            self->handleSetPowerLimitsResponse(ec, buffer);
        });
}

void NvidiaGpuControl::handleSetPowerLimitsResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    setPowerLimitInflight = false;

    if (ec)
    {
        lg2::error("Error setting power limits for eid {EID}: {MSG}", "EID",
                   eid, "MSG", ec.message());
    }
    else
    {
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        const int rc =
            gpu::decodeSetPowerLimitsResponse(buffer, cc, reasonCode);

        if (rc != 0 ||
            cc != ocp::accelerator_management::CompletionCode::SUCCESS)
        {
            lg2::error(
                "Error decoding SetPowerLimits response for eid {EID}: rc={RC}, cc={CC}, reasonCode={RESC}",
                "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
                reasonCode);
        }
    }

    internalSet = true;
    powerCapInterface->set_property("PowerCap", powerCapValue);
    powerCapInterface->set_property("PowerCapEnable", powerCapEnabled);
    internalSet = false;
}
