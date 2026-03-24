/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuFirmwareInformation.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <variant>
#include <vector>

const std::string softwareInventoryPath = "/xyz/openbmc_project/software/";

NvidiaGpuFirmwareInformation::NvidiaGpuFirmwareInformation(
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const sdbusplus::object_path& path, const uint8_t eid,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), mctpRequester(mctpRequester)
{
    const std::string dbusPath =
        softwareInventoryPath + escapeName(name) + "_Firmware";

    versionInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Software.Version");

    versionInterface->register_property<std::string>("Version", "");

    if (!versionInterface->initialize())
    {
        lg2::error(
            "Error initializing Version interface for Firmware Information, eid={EID}",
            "EID", eid);
    }

    std::vector<Association> associations;
    associations.emplace_back("running", "ran_on", std::string(path));

    associationInterface =
        objectServer.add_interface(dbusPath, association::interface);

    associationInterface->register_property("Associations", associations);

    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association interface for Firmware Information, eid={EID}",
            "EID", eid);
    }
}

void NvidiaGpuFirmwareInformation::processResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating Firmware Information for eid {EID} : sending message over MCTP failed, rc={RC}",
            "EID", eid, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::InventoryValue info;

    const int rc = gpu::decodeGetInventoryInformationResponse(
        buffer, cc, reasonCode, gpu::InventoryPropertyId::FIRMWARE_VERSION,
        info);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Firmware Information for eid {EID} : decode failed, rc={RC}, cc={CC}, reasonCode={REASON}",
            "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    if (!std::holds_alternative<std::string>(info))
    {
        lg2::error(
            "Firmware Version for eid {EID} expected string but got different type",
            "EID", eid);
        return;
    }

    versionInterface->set_property("Version", std::get<std::string>(info));
}

void NvidiaGpuFirmwareInformation::update()
{
    const int rc = gpu::encodeGetInventoryInformationRequest(
        0, static_cast<uint8_t>(gpu::InventoryPropertyId::FIRMWARE_VERSION),
        request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating Firmware Information for eid {EID} : encode failed, rc={RC}",
            "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuFirmwareInformation> self = weak.lock();
            if (!self)
            {
                lg2::error("invalid reference to NvidiaGpuFirmwareInformation");
                return;
            }
            self->processResponse(ec, buffer);
        });
}
