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
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/vtable.hpp>
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
#include <utility>
#include <vector>

static constexpr uint64_t mhzToHzFactor = 1'000'000;

// Debounce window to coalesce the RequestedSpeedLimitMin and Max writes of a
// single PATCH (which arrive back to back) into one SetClockLimit.
static constexpr std::chrono::milliseconds setLimitDebounce{100};

NvidiaGpuClockSpeedControl::NvidiaGpuClockSpeedControl(
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    const std::string& inventoryPath, mctp::MctpRequester& mctpRequester,
    uint8_t eid, boost::asio::io_context& io,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        controlClockSpeedIface,
    std::shared_ptr<Inventory> inventory) :
    controlClockSpeedInterface(controlClockSpeedIface),
    inventory(std::move(inventory)), mctpRequester(mctpRequester),
    name(escapeName(deviceName)), objectServer(objectServer), setLimitTimer(io),
    eid(eid)
{
    const int rc = gpu::encodeGetClockLimitRequest(
        0, gpu::ClockType::GRAPHICS_CLOCK, requestBuffer);
    if (rc == 0)
    {
        requestEncoded = true;
    }
    else
    {
        lg2::error("Failed to encode clock limit request for {NAME}: rc={RC}",
                   "NAME", name, "RC", rc);
    }

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
    sendGetClockLimitRequest();
}

void NvidiaGpuClockSpeedControl::sendGetClockLimitRequest()
{
    if (!requestEncoded)
    {
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
            self->handleGetClockLimitResponse(ec, buffer);
        });
}

void NvidiaGpuClockSpeedControl::handleGetClockLimitResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (setClockLimitReadbackPending)
    {
        // This response is the read-back for a completed SetClockLimit: the
        // device's new state has now been observed, so open the coalesce gate.
        // Done before the error returns below so a failed read-back cannot
        // wedge the gate shut forever. Any writes that arrived while the set
        // was in flight have re-armed setLimitTimer and dispatch on its next
        // expiry, so no manual re-dispatch is needed here.
        setClockLimitReadbackPending = false;
        setClockLimitInflight = false;
    }

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

    const uint64_t newPresentMaxHz =
        static_cast<uint64_t>(presentLimitMax) * mhzToHzFactor;
    const uint64_t newPresentMinHz =
        static_cast<uint64_t>(presentLimitMin) * mhzToHzFactor;
    const uint64_t newRequestedMaxHz =
        static_cast<uint64_t>(requestedLimitMax) * mhzToHzFactor;
    const uint64_t newRequestedMinHz =
        static_cast<uint64_t>(requestedLimitMin) * mhzToHzFactor;

    // Only emit PropertiesChanged when the value actually changes; this
    // handler runs on every poll.
    if (newPresentMaxHz != presentMaxHz)
    {
        presentMaxHz = newPresentMaxHz;
        controlClockSpeedInterface->signal_property("PresentSpeedLimitMaxHz");
    }
    if (newPresentMinHz != presentMinHz)
    {
        presentMinHz = newPresentMinHz;
        controlClockSpeedInterface->signal_property("PresentSpeedLimitMinHz");
    }
    if (newRequestedMaxHz != requestedMaxHz)
    {
        requestedMaxHz = newRequestedMaxHz;
        controlClockSpeedInterface->signal_property("RequestedSpeedLimitMaxHz");
    }
    if (newRequestedMinHz != requestedMinHz)
    {
        requestedMinHz = newRequestedMinHz;
        controlClockSpeedInterface->signal_property("RequestedSpeedLimitMinHz");
    }
}

int NvidiaGpuClockSpeedControl::handleRequestedSpeedLimitMaxHzSet(
    const uint64_t& newMaxHz, uint64_t& /*current*/)
{
    if (!inventory)
    {
        lg2::error(
            "RequestedSpeedLimitMaxHz set rejected for eid {EID}: inventory not available",
            "EID", eid);
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    std::optional<uint64_t> minHw = inventory->getMinSpeedInHz();
    std::optional<uint64_t> maxHw = inventory->getMaxSpeedInHz();
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

    // Record the requested max and arm the debounce timer. The actual
    // SetClockLimit is issued once the burst settles, so a PATCH that sets
    // both RequestedSpeedLimitMin and Max is coalesced into a single request.
    // Do not touch requestedMaxHz here: the D-Bus value is only updated once
    // the device confirms via a GetClockLimit response.
    pendingMaxHz = newMaxHz;
    armSetLimitTimer();

    return 1;
}

int NvidiaGpuClockSpeedControl::handleRequestedSpeedLimitMinHzSet(
    const uint64_t& newMinHz, uint64_t& /*current*/)
{
    if (!inventory)
    {
        lg2::error(
            "RequestedSpeedLimitMinHz set rejected for eid {EID}: inventory not available",
            "EID", eid);
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    std::optional<uint64_t> minHw = inventory->getMinSpeedInHz();
    std::optional<uint64_t> maxHw = inventory->getMaxSpeedInHz();
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

    pendingMinHz = newMinHz;
    armSetLimitTimer();

    return 1;
}

void NvidiaGpuClockSpeedControl::armSetLimitTimer()
{
    setLimitTimer.expires_after(setLimitDebounce);
    setLimitTimer.async_wait(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                // Superseded by a later set within the debounce window.
                return;
            }
            std::shared_ptr<NvidiaGpuClockSpeedControl> self = weak.lock();
            if (!self)
            {
                return;
            }
            self->applyPendingClockLimit();
        });
}

void NvidiaGpuClockSpeedControl::applyPendingClockLimit()
{
    if (setClockLimitInflight)
    {
        // A previous SetClockLimit is still in flight; retry once it lands.
        armSetLimitTimer();
        return;
    }

    const uint64_t minHz = pendingMinHz.value_or(requestedMinHz);
    const uint64_t maxHz = pendingMaxHz.value_or(requestedMaxHz);
    pendingMinHz.reset();
    pendingMaxHz.reset();

    sendSetClockLimitRequest(static_cast<uint32_t>(minHz / mhzToHzFactor),
                             static_cast<uint32_t>(maxHz / mhzToHzFactor));
}

void NvidiaGpuClockSpeedControl::sendSetClockLimitRequest(uint32_t limitMinMHz,
                                                          uint32_t limitMaxMHz)
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

    setClockLimitInflight = true;

    mctpRequester.sendRecvMsg(
        eid, setClockLimitRequestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuClockSpeedControl reference");
                return;
            }
            self->handleSetClockLimitResponse(ec, buffer);
        });
}

void NvidiaGpuClockSpeedControl::handleSetClockLimitResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error setting clock limit for eid {EID}: {MSG}", "EID", eid,
                   "MSG", ec.message());
        setClockLimitInflight = false;
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
        setClockLimitInflight = false;
        return;
    }

    // The set succeeded; re-read from the device so requestedMin/MaxHz and
    // presentMin/MaxHz reflect the new state without waiting for the next
    // periodic poll. setClockLimitInflight stays set until this read-back's
    // response lands so the coalesce gate does not reopen on stale state.
    if (!requestEncoded)
    {
        // No read-back can be sent, so no response will ever arrive to reopen
        // the coalesce gate; release it here to avoid wedging it shut.
        setClockLimitInflight = false;
        return;
    }
    setClockLimitReadbackPending = true;
    sendGetClockLimitRequest();
}
