/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaNVLinkPortControl.hpp"

#include "NvidiaGpuControlErrors.hpp"
#include "NvidiaUtils.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{

constexpr const char* linkStateInterface =
    "xyz.openbmc_project.Control.Port.LinkState";

const sdbusplus::object_path controlPortPrefix{
    "/xyz/openbmc_project/control/port"};

// Debounce window to coalesce the writes that arrive back to back, such as
// several ports being changed together, into one Set Port Disable Future.
constexpr std::chrono::milliseconds setDebounce{100};

std::string linkStateValue(const char* linkState)
{
    return std::format(
        "xyz.openbmc_project.Inventory.Connector.Port.LinkState.{}", linkState);
}

// A set bit in the mask disables the port with that port number after the
// next reset; a clear bit leaves it enabled.
bool isPortDisabled(
    const std::array<uint8_t, gpu::portDisableFutureMaskSize>& portMask,
    size_t portNumber)
{
    return ((portMask[portNumber / 8] >> (portNumber % 8)) & 1U) != 0;
}

void setPortDisabled(
    std::array<uint8_t, gpu::portDisableFutureMaskSize>& portMask,
    size_t portNumber, bool disabled)
{
    const auto bit = static_cast<uint8_t>(1U << (portNumber % 8));
    if (disabled)
    {
        portMask[portNumber / 8] |= bit;
    }
    else
    {
        portMask[portNumber / 8] &= static_cast<uint8_t>(~bit);
    }
}

} // namespace

NvidiaNVLinkPortControl::NvidiaNVLinkPortControl(
    sdbusplus::asio::object_server& objectServer,
    mctp::MctpRequester& mctpRequester, boost::asio::io_context& io,
    const std::string& gpuName, uint8_t eid, uint8_t numberOfPorts) :
    eid(eid), objectServer(objectServer), mctpRequester(mctpRequester),
    pendingDisable(numberOfPorts), setTimer(io)
{
    if (gpu::encodeGetPortDisableFutureRequest(0, request) != 0)
    {
        lg2::error(
            "Failed to encode NVLink Port disable future request, eid={EID}",
            "EID", eid);
    }
    else
    {
        requestEncoded = true;
    }

    linkStateInterfaces.reserve(numberOfPorts);
    associationInterfaces.reserve(numberOfPorts);

    for (uint8_t i = 0; i < numberOfPorts; ++i)
    {
        const std::string portName = std::format("NVLink_{}", i);
        const sdbusplus::object_path controlPath =
            controlPortPrefix / gpuName / portName;

        std::shared_ptr<sdbusplus::asio::dbus_interface> linkState =
            objectServer.add_interface(controlPath, linkStateInterface);
        linkState->register_property<std::string>(
            "RequestedLinkState", linkStateValue("Unknown"),
            std::bind_front(
                &NvidiaNVLinkPortControl::handleRequestedLinkStateSet, this, i),
            [this, i](std::string&) { return requestedLinkState(i); });

        if (!linkState->initialize())
        {
            lg2::error(
                "Error initializing NVLink Port LinkState control interface, eid={EID}, portIndex={PI}",
                "EID", eid, "PI", i);
        }

        std::vector<Association> associations;
        associations.emplace_back("controlling", "controlled_by",
                                  inventoryPrefix / gpuName / portName);

        std::shared_ptr<sdbusplus::asio::dbus_interface> association =
            objectServer.add_interface(controlPath, association::interface);
        association->register_property("Associations", associations);

        if (!association->initialize())
        {
            lg2::error(
                "Error initializing Association interface for NVLink Port LinkState control, eid={EID}, portIndex={PI}",
                "EID", eid, "PI", i);
        }

        linkStateInterfaces.emplace_back(std::move(linkState));
        associationInterfaces.emplace_back(std::move(association));
    }
}

NvidiaNVLinkPortControl::~NvidiaNVLinkPortControl()
{
    for (auto& interface : linkStateInterfaces)
    {
        objectServer.remove_interface(interface);
    }
    for (auto& interface : associationInterfaces)
    {
        objectServer.remove_interface(interface);
    }
}

void NvidiaNVLinkPortControl::update()
{
    if (!requestEncoded)
    {
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}, setsSeen{setsSent}](
            const std::error_code& ec, std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaNVLinkPortControl> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaNVLinkPortControl");
                return;
            }
            self->processResponse(ec, buffer, setsSeen);
        });
}

std::string NvidiaNVLinkPortControl::requestedLinkState(size_t portIndex) const
{
    if (!deviceMask)
    {
        return linkStateValue("Unknown");
    }

    // Port numbers start at 1, so bit 0 of the mask is never used.
    return linkStateValue(
        isPortDisabled(*deviceMask, portIndex + 1) ? "Disabled" : "Enabled");
}

void NvidiaNVLinkPortControl::processResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response,
    uint32_t setsSeen)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating NVLink Port disable future: sending message over MCTP failed, rc={RC}, EID={EID}",
            "RC", sendRecvMsgResult.message(), "EID", eid);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    PortMask portMask{};

    const int rc = gpu::decodeGetPortDisableFutureResponse(
        response, cc, reasonCode, portMask);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating NVLink Port disable future: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid);
        return;
    }

    // Only a Get sent after the latest Set reads that Set back. Writes that
    // arrived while it was in flight can be built on this mask now.
    if (setInflight && setsSeen == setsSent)
    {
        setInflight = false;
        armSetTimerIfPending();
    }

    std::vector<std::string> previous;
    previous.reserve(linkStateInterfaces.size());
    for (size_t i = 0; i < linkStateInterfaces.size(); ++i)
    {
        previous.emplace_back(requestedLinkState(i));
    }

    deviceMask = portMask;

    // Only emit PropertiesChanged when the value actually changes; this
    // handler runs on every poll.
    for (size_t i = 0; i < linkStateInterfaces.size(); ++i)
    {
        if (requestedLinkState(i) != previous[i])
        {
            linkStateInterfaces[i]->signal_property("RequestedLinkState");
        }
    }
}

int NvidiaNVLinkPortControl::handleRequestedLinkStateSet(
    size_t portIndex, const std::string& newState, std::string& /*current*/)
{
    bool disable = false;
    if (newState == linkStateValue("Disabled"))
    {
        disable = true;
    }
    else if (newState != linkStateValue("Enabled"))
    {
        // Unknown, or a string that is not a LinkState at all, cannot be
        // requested from the device.
        lg2::error(
            "RequestedLinkState set rejected for eid {EID}, portIndex={PI}: invalid value {VAL}",
            "EID", eid, "PI", portIndex, "VAL", newState);
        throw InvalidArgument();
    }

    if (!deviceMask)
    {
        // The other ports' bits come from the device mask, so a write cannot
        // be built until it has been read.
        lg2::error(
            "RequestedLinkState set rejected for eid {EID}, portIndex={PI}: port mask not yet known",
            "EID", eid, "PI", portIndex);
        throw Unavailable();
    }

    // Record the request and arm the debounce timer. Do not touch deviceMask
    // here: the D-Bus value is only updated once the device confirms it via a
    // Get Port Disable Future response.
    pendingDisable[portIndex] = disable;
    armSetTimer();

    return 1;
}

void NvidiaNVLinkPortControl::armSetTimer()
{
    setTimer.expires_after(setDebounce);
    setTimer.async_wait(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                // Superseded by a later set within the debounce window.
                return;
            }
            std::shared_ptr<NvidiaNVLinkPortControl> self = weak.lock();
            if (!self)
            {
                return;
            }
            self->applyPendingLinkStates();
        });
}

void NvidiaNVLinkPortControl::armSetTimerIfPending()
{
    if (std::ranges::any_of(pendingDisable,
                            [](const std::optional<bool>& pending) {
                                return pending.has_value();
                            }))
    {
        armSetTimer();
    }
}

void NvidiaNVLinkPortControl::applyPendingLinkStates()
{
    if (setInflight)
    {
        // A previous Set has not been read back yet. The pending writes stay
        // queued and are dispatched once it has been.
        return;
    }

    if (!deviceMask)
    {
        return;
    }

    PortMask portMask = *deviceMask;
    for (size_t i = 0; i < pendingDisable.size(); ++i)
    {
        std::optional<bool>& pending = pendingDisable[i];
        if (pending)
        {
            setPortDisabled(portMask, i + 1, *pending);
            pending.reset();
        }
    }

    if (portMask == *deviceMask)
    {
        // Every request matches what the device already has.
        return;
    }

    const int rc =
        gpu::encodeSetPortDisableFutureRequest(0, portMask, setRequest);
    if (rc != 0)
    {
        lg2::error(
            "Error encoding NVLink Port disable future set request for eid {EID}, rc={RC}",
            "EID", eid, "RC", rc);
        return;
    }

    setInflight = true;
    ++setsSent;

    mctpRequester.sendRecvMsg(
        eid, setRequest,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaNVLinkPortControl> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaNVLinkPortControl");
                return;
            }
            self->handleSetResponse(ec, buffer);
        });
}

void NvidiaNVLinkPortControl::handleSetResponse(
    const std::error_code& ec, std::span<const uint8_t> response)
{
    if (ec)
    {
        // The request may still have reached the device, so whether the mask
        // changed is unknown. Keep later writes waiting and read the mask back
        // as after a successful Set; a later poll does the same if this read
        // fails too.
        lg2::error(
            "Error setting NVLink Port disable future for eid {EID}: {MSG}",
            "EID", eid, "MSG", ec.message());
        update();
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    const int rc =
        gpu::decodeSetPortDisableFutureResponse(response, cc, reasonCode);

    if (rc != 0)
    {
        // An undecodable response says nothing about whether the device took
        // the mask; read it back as for a transport error.
        lg2::error(
            "Error decoding NVLink Port disable future set response for eid {EID}: rc={RC}",
            "EID", eid, "RC", rc);
        update();
        return;
    }

    if (cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        // The device refused the mask, so it still has the one last read.
        setInflight = false;
        lg2::error(
            "NVLink Port disable future set refused for eid {EID}: cc={CC}, reasonCode={RESC}",
            "EID", eid, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode);
        armSetTimerIfPending();
        return;
    }

    // The set succeeded; re-read from the device so RequestedLinkState
    // reflects the new mask without waiting for the next periodic poll.
    update();
}
