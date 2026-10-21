/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaNVLinkPortControl.hpp"

#include "NvidiaUtils.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
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

} // namespace

NvidiaNVLinkPortControl::NvidiaNVLinkPortControl(
    sdbusplus::asio::object_server& objectServer,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    uint8_t numberOfPorts) :
    eid(eid), objectServer(objectServer), mctpRequester(mctpRequester)
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
        linkState->register_property("RequestedLinkState",
                                     linkStateValue("Unknown"));

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
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaNVLinkPortControl> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaNVLinkPortControl");
                return;
            }
            self->processResponse(ec, buffer);
        });
}

void NvidiaNVLinkPortControl::processResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
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
    std::array<uint8_t, gpu::portDisableFutureMaskSize> portMask{};

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

    for (size_t i = 0; i < linkStateInterfaces.size(); ++i)
    {
        // Port numbers start at 1, so bit 0 of the mask is never used.
        linkStateInterfaces[i]->set_property(
            "RequestedLinkState",
            linkStateValue(
                isPortDisabled(portMask, i + 1) ? "Disabled" : "Enabled"));
    }
}
