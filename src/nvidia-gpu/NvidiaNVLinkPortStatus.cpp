/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaNVLinkPortStatus.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <system_error>

namespace
{

std::string mapPortStateToLinkStatus(uint8_t portState)
{
    const char* linkStatus = nullptr;

    switch (portState)
    {
        case 2:
            linkStatus = "LinkUp";
            break;
        case 6:
            linkStatus = "Starting";
            break;
        case 7:
        case 10:
            linkStatus = "Training";
            break;
        case 1:
        case 4:
        case 5:
        case 8:
        case 9:
            linkStatus = "LinkDown";
            break;
        case 3:
        default:
            linkStatus = "NoLink";
            break;
    }

    return std::format(
        "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.{}",
        linkStatus);
}

std::string mapPortStatusToLinkState(uint8_t portStatus)
{
    const char* linkState = nullptr;

    switch (portStatus)
    {
        case 1:
            linkState = "Disabled";
            break;
        case 2:
            linkState = "Enabled";
            break;
        default:
            linkState = "Unknown";
            break;
    }

    return std::format(
        "xyz.openbmc_project.Inventory.Connector.Port.LinkState.{}", linkState);
}

} // namespace

NvidiaNVLinkPortStatus::NvidiaNVLinkPortStatus(
    mctp::MctpRequester& mctpRequester, uint8_t eid, uint8_t portIndex,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& portInterface) :
    eid(eid), portNumber(static_cast<uint8_t>(portIndex + 1)),
    mctpRequester(mctpRequester), portInterface(portInterface)
{
    if (gpu::encodeQueryPortStatusRequest(0, portNumber, request) != 0)
    {
        lg2::error(
            "Failed to encode NVLink Port status request, eid={EID}, portNumber={PN}",
            "EID", eid, "PN", portNumber);
    }
    else
    {
        requestEncoded = true;
    }
}

void NvidiaNVLinkPortStatus::update()
{
    if (!requestEncoded)
    {
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaNVLinkPortStatus> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaNVLinkPortStatus");
                return;
            }
            self->processResponse(ec, buffer);
        });
}

void NvidiaNVLinkPortStatus::processResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating NVLink Port status: sending message over MCTP failed, rc={RC}, EID={EID}, portNumber={PN}",
            "RC", sendRecvMsgResult.message(), "EID", eid, "PN", portNumber);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint8_t portState = 0;
    uint8_t portStatus = 0;

    const int rc = gpu::decodeQueryPortStatusResponse(response, cc, reasonCode,
                                                      portState, portStatus);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating NVLink Port status: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, portNumber={PN}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid, "PN", portNumber);
        return;
    }

    portInterface->set_property("LinkStatus",
                                mapPortStateToLinkStatus(portState));
    portInterface->set_property("LinkState",
                                mapPortStatusToLinkState(portStatus));
}
