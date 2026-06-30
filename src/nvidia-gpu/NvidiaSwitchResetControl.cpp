/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSwitchResetControl.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace
{

constexpr const char* controlResetPathPrefix =
    "/xyz/openbmc_project/control/reset/";
constexpr const char* resetInterfaceName = "xyz.openbmc_project.Control.Reset";
constexpr const char* forceRestartEnumValue =
    "xyz.openbmc_project.Control.Reset.ResetTypes.ForceRestart";
constexpr const char* noneEnumValue =
    "xyz.openbmc_project.Control.Reset.ResetTypes.None";

// Single source of truth for the reset types this control supports. Both the
// SupportedResetTypes property and the PendingReset write validation are
// derived from this list.
constexpr std::array<std::string_view, 1> supportedResetTypes{
    forceRestartEnumValue};

using NotAllowed = sdbusplus::error::xyz::openbmc_project::common::NotAllowed;
using Unavailable = sdbusplus::error::xyz::openbmc_project::common::Unavailable;

} // namespace

NvidiaSwitchResetControl::NvidiaSwitchResetControl(
    sdbusplus::asio::object_server& objectServer,
    mctp::MctpRequester& mctpRequester, const std::string& deviceName,
    const std::string& inventoryPath, uint8_t eid) :
    objectServer(objectServer), mctpRequester(mctpRequester), eid(eid),
    name(escapeName(deviceName)), pendingResetValue(noneEnumValue)
{
    const int rc = gpu::encodeResetNetworkDeviceRequest(
        0, gpu::ResetNetworkDeviceMode::StartAfterResponse, resetRequestBuffer);
    if (rc == 0)
    {
        requestEncoded = true;
    }
    else
    {
        lg2::error(
            "Failed to encode ResetNetworkDevice request for EID={EID}, rc={RC}",
            "EID", eid, "RC", rc);
    }

    const std::string resetPath = controlResetPathPrefix + name;

    resetInterface = objectServer.add_interface(resetPath, resetInterfaceName);

    associationInterface =
        objectServer.add_interface(resetPath, association::interface);

    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);
    associationInterface->register_property("Associations", associations);

    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association interface for SwitchResetControl EID={EID}",
            "EID", eid);
    }

    const std::vector<std::string> supportedResetTypeList(
        supportedResetTypes.begin(), supportedResetTypes.end());
    resetInterface->register_property("SupportedResetTypes",
                                      supportedResetTypeList);

    resetInterface->register_property<std::string>(
        "PendingReset", noneEnumValue,
        std::bind_front(&NvidiaSwitchResetControl::handlePendingResetSet, this),
        [this](std::string&) { return pendingResetValue; });

    if (!resetInterface->initialize())
    {
        lg2::error("Error initializing Reset interface for EID={EID}", "EID",
                   eid);
    }
}

NvidiaSwitchResetControl::~NvidiaSwitchResetControl()
{
    if (resetInterface)
    {
        objectServer.remove_interface(resetInterface);
    }
    if (associationInterface)
    {
        objectServer.remove_interface(associationInterface);
    }
}

int NvidiaSwitchResetControl::handlePendingResetSet(
    const std::string& newValue, std::string& /*currentValue*/)
{
    if (!requestEncoded)
    {
        lg2::error(
            "PendingReset set rejected for EID={EID}: request buffer not encoded",
            "EID", eid);
        throw Unavailable();
    }

    if (pendingResetValue != noneEnumValue)
    {
        lg2::error(
            "PendingReset set rejected for EID={EID}: reset already in flight",
            "EID", eid);
        throw Unavailable();
    }

    if (std::find(supportedResetTypes.begin(), supportedResetTypes.end(),
                  newValue) == supportedResetTypes.end())
    {
        lg2::error(
            "PendingReset set rejected for EID={EID}: unsupported reset type {TYPE}",
            "EID", eid, "TYPE", newValue);
        throw NotAllowed();
    }

    pendingResetValue = newValue;
    sendResetRequest();
    return 1;
}

void NvidiaSwitchResetControl::sendResetRequest()
{
    mctpRequester.sendRecvMsg(
        eid, resetRequestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> response) {
            std::shared_ptr<NvidiaSwitchResetControl> self = weak.lock();
            if (!self)
            {
                lg2::error("NvidiaSwitchResetControl no longer exists");
                return;
            }
            self->handleResetResponse(ec, response);
        });
}

void NvidiaSwitchResetControl::handleResetResponse(
    const std::error_code& ec, std::span<const uint8_t> response)
{
    if (ec)
    {
        lg2::error("Reset MCTP send failed for EID={EID}: {MSG}", "EID", eid,
                   "MSG", ec.message());
    }
    else
    {
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        const int rc =
            gpu::decodeResetNetworkDeviceResponse(response, cc, reasonCode);

        if (rc != 0 ||
            cc != ocp::accelerator_management::CompletionCode::SUCCESS)
        {
            lg2::error(
                "Reset rejected by EID={EID}: rc={RC}, cc={CC}, reasonCode={RESC}",
                "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
                reasonCode);
        }
        else
        {
            lg2::info("Reset succeeded for EID={EID}", "EID", eid);
        }
    }

    pendingResetValue = noneEnumValue;
    resetInterface->signal_property("PendingReset");
}
