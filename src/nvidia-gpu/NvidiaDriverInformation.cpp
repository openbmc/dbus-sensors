/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaDriverInformation.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>

const std::string softwareInventoryPath =
    "/xyz/openbmc_project/software_inventory/";

NvidiaDriverInformation::NvidiaDriverInformation(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const uint8_t eid, sdbusplus::asio::object_server& objectServer) :
    eid(eid), conn(conn), mctpRequester(mctpRequester)
{
    const std::string dbusPath = softwareInventoryPath + escapeName(name);

    versionInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Software.Version");

    versionInterface->register_property<std::string>("Version", "");

    if (!versionInterface->initialize())
    {
        lg2::error(
            "Failed to initialize Version interface for Driver Information for eid {EID}",
            "EID", eid);
    }
}

void NvidiaDriverInformation::processResponse(const std::error_code& ec,
                                              std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating Driver Information for eid {EID} : sending message over MCTP failed, rc={RC}",
            "EID", eid, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::DriverState driverState{};
    std::string driverVersion;

    const int rc = gpu::decodeGetDriverInformationResponse(
        buffer, cc, reasonCode, driverState, driverVersion);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Driver Information for eid {EID} : decode failed, rc={RC}, cc={CC}, reasonCode={REASON}",
            "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    versionInterface->set_property("Version", driverVersion);
}

void NvidiaDriverInformation::update()
{
    const int rc = gpu::encodeGetDriverInformationRequest(0, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating Driver Information for eid {EID} : encode failed, rc={RC}",
            "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaDriverInformation> self = weak.lock();
            if (!self)
            {
                lg2::error("invalid reference to NvidiaDriverInformation");
                return;
            }
            self->processResponse(ec, buffer);
        });
}
