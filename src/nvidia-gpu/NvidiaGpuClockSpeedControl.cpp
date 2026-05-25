/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuClockSpeedControl.hpp"

#include "Inventory.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/vtable.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

static constexpr uint64_t mhzToHzFactor = 1'000'000;

NvidiaGpuClockSpeedControl::NvidiaGpuClockSpeedControl(
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    const std::string& inventoryPath, mctp::MctpRequester& mctpRequester,
    uint8_t eid,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        controlClockSpeedIface,
    std::shared_ptr<Inventory> inventory) :
    controlClockSpeedInterface(controlClockSpeedIface),
    inventory(std::move(inventory)), mctpRequester(mctpRequester),
    name(escapeName(deviceName)), objectServer(objectServer), eid(eid)
{
    const sdbusplus::object_path objPath(
        controlClockSpeedIface->get_object_path());

    associationInterface =
        objectServer.add_interface(objPath.str, association::interface);
    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);
    associationInterface->register_property("Associations", associations);
    associationInterface->initialize();

    controlClockSpeedInterface->register_property_r<uint64_t>(
        "PresentSpeedLimitMaxHz", std::numeric_limits<uint64_t>::max(),
        sdbusplus::vtable::property_::emits_change,
        [this](uint64_t&) { return presentMaxHz; });
    controlClockSpeedInterface->register_property_r<uint64_t>(
        "PresentSpeedLimitMinHz", uint64_t{0},
        sdbusplus::vtable::property_::emits_change,
        [this](uint64_t&) { return presentMinHz; });
    controlClockSpeedInterface->register_property<uint64_t>(
        "RequestedSpeedLimitMaxHz", std::numeric_limits<uint64_t>::max(),
        std::bind_front(
            &NvidiaGpuClockSpeedControl::handleRequestedSpeedLimitMaxHzSet,
            this),
        [this](uint64_t&) { return requestedMaxHz; });
    controlClockSpeedInterface->register_property<uint64_t>(
        "RequestedSpeedLimitMinHz", std::numeric_limits<uint64_t>::max(),
        std::bind_front(
            &NvidiaGpuClockSpeedControl::handleRequestedSpeedLimitMinHzSet,
            this),
        [this](uint64_t&) { return requestedMinHz; });

    controlClockSpeedInterface->initialize();
}

NvidiaGpuClockSpeedControl::~NvidiaGpuClockSpeedControl()
{
    objectServer.remove_interface(associationInterface);
}

void NvidiaGpuClockSpeedControl::update()
{
    int rc = gpu::encodeGetClockLimitRequest(0, gpu::ClockType::GRAPHICS_CLOCK,
                                             requestBuffer);
    if (rc != 0)
    {
        lg2::error("Failed to encode clock limit request for {NAME}: rc={RC}",
                   "NAME", name, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, requestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuClockSpeedControl reference");
                return;
            }
            self->handleResponse(ec, buffer);
        });
}

void NvidiaGpuClockSpeedControl::handleResponse(const std::error_code& ec,
                                                std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error reading clock limit for {NAME}: MCTP failed, rc={RC}",
                   "NAME", name, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t requestedLimitMin = 0;
    uint32_t requestedLimitMax = 0;
    uint32_t presentLimitMin = 0;
    uint32_t presentLimitMax = 0;

    int rc = gpu::decodeGetClockLimitResponse(
        buffer, cc, reasonCode, requestedLimitMin, requestedLimitMax,
        presentLimitMin, presentLimitMax);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding clock limit for {NAME}: rc={RC}, cc={CC}, reasonCode={REASON}",
            "NAME", name, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    presentMaxHz = static_cast<uint64_t>(presentLimitMax) * mhzToHzFactor;
    presentMinHz = static_cast<uint64_t>(presentLimitMin) * mhzToHzFactor;
    requestedMaxHz = static_cast<uint64_t>(requestedLimitMax) * mhzToHzFactor;
    requestedMinHz = static_cast<uint64_t>(requestedLimitMin) * mhzToHzFactor;

    controlClockSpeedInterface->signal_property("PresentSpeedLimitMaxHz");
    controlClockSpeedInterface->signal_property("PresentSpeedLimitMinHz");
    controlClockSpeedInterface->signal_property("RequestedSpeedLimitMaxHz");
    controlClockSpeedInterface->signal_property("RequestedSpeedLimitMinHz");
}

int NvidiaGpuClockSpeedControl::handleRequestedSpeedLimitMaxHzSet(
    const uint64_t& newMaxHz, uint64_t& current)
{
    if (requestedMaxHzInflight)
    {
        lg2::error(
            "RequestedSpeedLimitMaxHz set rejected for eid {EID}: SetClockLimit in flight",
            "EID", eid);
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    auto minHw = inventory->getMinSpeedInHz();
    auto maxHw = inventory->getMaxSpeedInHz();
    if (!minHw || !maxHw)
    {
        lg2::error(
            "RequestedSpeedLimitMaxHz set rejected for eid {EID}: hardware min/max not yet known",
            "EID", eid);
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    if (newMaxHz < *minHw || newMaxHz > *maxHw)
    {
        lg2::error(
            "RequestedSpeedLimitMaxHz set rejected for eid {EID}: value {VAL} out of range [{MIN}, {MAX}]",
            "EID", eid, "VAL", newMaxHz, "MIN", *minHw, "MAX", *maxHw);
        throw sdbusplus::error::xyz::openbmc_project::common::InvalidArgument();
    }

    uint64_t effectiveMinHz = requestedMinHzKnown ? requestedMinHz : *minHw;

    requestedMaxHz = newMaxHz;
    requestedMaxHzKnown = true;
    sendSetClockLimitRequest(
        &requestedMaxHzInflight,
        static_cast<uint32_t>(effectiveMinHz / mhzToHzFactor),
        static_cast<uint32_t>(newMaxHz / mhzToHzFactor));

    current = newMaxHz;
    return 1;
}

int NvidiaGpuClockSpeedControl::handleRequestedSpeedLimitMinHzSet(
    const uint64_t& newMinHz, uint64_t& current)
{
    if (requestedMinHzInflight)
    {
        lg2::error(
            "RequestedSpeedLimitMinHz set rejected for eid {EID}: SetClockLimit in flight",
            "EID", eid);
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    auto minHw = inventory->getMinSpeedInHz();
    auto maxHw = inventory->getMaxSpeedInHz();
    if (!minHw || !maxHw)
    {
        lg2::error(
            "RequestedSpeedLimitMinHz set rejected for eid {EID}: hardware min/max not yet known",
            "EID", eid);
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    if (newMinHz < *minHw || newMinHz > *maxHw)
    {
        lg2::error(
            "RequestedSpeedLimitMinHz set rejected for eid {EID}: value {VAL} out of range [{MIN}, {MAX}]",
            "EID", eid, "VAL", newMinHz, "MIN", *minHw, "MAX", *maxHw);
        throw sdbusplus::error::xyz::openbmc_project::common::InvalidArgument();
    }

    uint64_t effectiveMaxHz = requestedMaxHzKnown ? requestedMaxHz : *maxHw;

    requestedMinHz = newMinHz;
    requestedMinHzKnown = true;
    sendSetClockLimitRequest(
        &requestedMinHzInflight,
        static_cast<uint32_t>(newMinHz / mhzToHzFactor),
        static_cast<uint32_t>(effectiveMaxHz / mhzToHzFactor));

    current = newMinHz;
    return 1;
}

void NvidiaGpuClockSpeedControl::sendSetClockLimitRequest(
    bool* inflightFlag, uint32_t limitMinMHz, uint32_t limitMaxMHz)
{
    const int rc = gpu::encodeSetClockLimitRequest(
        0, gpu::ClockType::GRAPHICS_CLOCK, gpu::ClockLimitFlag::PERSISTENCE,
        limitMinMHz, limitMaxMHz, setClockLimitRequestBuffer);

    if (rc != 0)
    {
        lg2::error(
            "Error encoding SET_CLOCK_LIMIT request for eid {EID}, rc={RC}",
            "EID", eid, "RC", rc);
        return;
    }

    *inflightFlag = true;

    mctpRequester.sendRecvMsg(
        eid, setClockLimitRequestBuffer,
        [weak{weak_from_this()}, inflightFlag](
            const std::error_code& ec, std::span<const uint8_t> buffer) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuClockSpeedControl reference");
                return;
            }
            self->handleSetClockLimitResponse(inflightFlag, ec, buffer);
        });
}

void NvidiaGpuClockSpeedControl::handleSetClockLimitResponse(
    bool* inflightFlag, const std::error_code& ec,
    std::span<const uint8_t> buffer)
{
    *inflightFlag = false;

    if (ec)
    {
        lg2::error("Error setting clock limit for eid {EID}: {MSG}", "EID", eid,
                   "MSG", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    const int rc = gpu::decodeSetClockLimitResponse(buffer, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding SetClockLimit response for eid {EID}: rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
            reasonCode);
    }
}
