/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSwitchResetControl.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace
{

constexpr const char* controlResetPathPrefix =
    "/xyz/openbmc_project/control/reset/";
constexpr const char* resetInterfaceName = "xyz.openbmc_project.Control.Reset";
constexpr const char* forceRestartEnumValue =
    "xyz.openbmc_project.Control.Reset.ResetTypes.ForceRestart";

using InternalFailure =
    sdbusplus::error::xyz::openbmc_project::common::InternalFailure;
using NotAllowed = sdbusplus::error::xyz::openbmc_project::common::NotAllowed;
using Timeout = sdbusplus::error::xyz::openbmc_project::common::Timeout;
using Unavailable = sdbusplus::error::xyz::openbmc_project::common::Unavailable;

} // namespace

NvidiaSwitchResetControl::NvidiaSwitchResetControl(
    sdbusplus::asio::object_server& objectServer,
    mctp::MctpRequester& mctpRequester, const std::string& deviceName,
    const std::string& inventoryPath, uint8_t eid) :
    objectServer(objectServer), mctpRequester(mctpRequester), eid(eid),
    name(escapeName(deviceName))
{
    const std::string resetPath = controlResetPathPrefix + name;

    resetInterface = objectServer.add_interface(resetPath, resetInterfaceName);
    resetInterface->register_property("ResetType",
                                      std::string(forceRestartEnumValue));

    associationInterface =
        objectServer.add_interface(resetPath, association::interface);

    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);
    associationInterface->register_property("Associations", associations);

    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association Interface for SwitchResetControl EID={EID}",
            "EID", eid);
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

void NvidiaSwitchResetControl::init()
{
    std::weak_ptr<NvidiaSwitchResetControl> weak = weak_from_this();
    resetInterface->register_method(
        "Reset", [weak](const boost::asio::yield_context& yield) {
            std::shared_ptr<NvidiaSwitchResetControl> self = weak.lock();
            if (!self)
            {
                lg2::error("NvidiaSwitchResetControl no longer exists");
                throw Unavailable();
            }
            self->sendResetRequest(yield);
        });

    if (!resetInterface->initialize())
    {
        lg2::error("Error initializing Reset Interface for EID={EID}", "EID",
                   eid);
    }
}

void NvidiaSwitchResetControl::sendResetRequest(
    const boost::asio::yield_context& yield)
{
    std::array<uint8_t, gpu::resetNetworkDeviceRequestSize> reqBuf{};

    const int encodeRc = gpu::encodeResetNetworkDeviceRequest(
        0, gpu::ResetNetworkDeviceMode::StartAfterResponse, reqBuf);

    if (encodeRc != 0)
    {
        lg2::error("Reset encode failed for EID {EID}, rc={RC}", "EID", eid,
                   "RC", encodeRc);
        throw InternalFailure();
    }

    boost::system::error_code ec;
    std::vector<uint8_t> resp =
        mctpRequester.sendRecvMsg(eid, reqBuf, yield[ec]);

    if (ec)
    {
        lg2::error("Reset MCTP send failed for EID {EID}: {MSG}", "EID", eid,
                   "MSG", ec.message());
        throw Timeout();
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    const int decodeRc =
        gpu::decodeResetNetworkDeviceResponse(resp, cc, reasonCode);

    if (decodeRc != 0)
    {
        lg2::error("Reset decode failed for EID {EID}, rc={RC}", "EID", eid,
                   "RC", decodeRc);
        throw InternalFailure();
    }

    if (cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("Reset rejected by EID {EID}: cc={CC}, reasonCode={RESC}",
                   "EID", eid, "CC", static_cast<uint8_t>(cc), "RESC",
                   reasonCode);

        switch (cc)
        {
            case ocp::accelerator_management::CompletionCode::ERR_NOT_READY:
                throw Unavailable();
            case ocp::accelerator_management::CompletionCode::
                ERR_UNSUPPORTED_COMMAND_CODE:
            case ocp::accelerator_management::CompletionCode::
                ERR_UNSUPPORTED_MSG_TYPE:
                throw NotAllowed();
            default:
                throw InternalFailure();
        }
    }

    lg2::info("Reset succeeded for EID {EID}", "EID", eid);
}
