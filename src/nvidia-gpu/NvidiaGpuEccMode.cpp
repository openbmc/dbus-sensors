/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuEccMode.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuLongRunningCommand.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <NvidiaUtils.hpp>
#include <OcpMctpVdm.hpp>
#include <SerialQueue.hpp>
#include <Utils.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

NvidiaGpuEccMode::NvidiaGpuEccMode(
    mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    uint8_t eid, std::shared_ptr<SerialQueue> longRunningQueue,
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler)
{
    const std::string controlPath =
        std::string("/xyz/openbmc_project/control/processor/") + deviceName;

    eccModeInterface = objectServer.add_interface(
        controlPath, "xyz.openbmc_project.Control.Processor.EccMode");

    // Both properties are read-only in this commit. The writable setter
    // that dispatches NSM Set ECC Mode is added in a follow-up.
    eccModeInterface->register_property(
        "Active", false, sdbusplus::asio::PropertyPermission::readOnly);
    eccModeInterface->register_property(
        "Enabled", false, sdbusplus::asio::PropertyPermission::readOnly);

    if (!eccModeInterface->initialize())
    {
        lg2::error("Failed to initialize ECC mode interface for GPU {NAME}",
                   "NAME", deviceName);
    }

    // Link the control object to the processor inventory item it controls.
    // The mapper creates the reverse controlled_by edge automatically.
    const std::string inventoryPath = (inventoryPrefix / deviceName).str;

    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);

    eccModeAssociationInterface =
        objectServer.add_interface(controlPath, association::interface);
    eccModeAssociationInterface->register_property("Associations",
                                                   associations);

    if (!eccModeAssociationInterface->initialize())
    {
        lg2::error(
            "Failed to initialize ECC mode association interface for GPU {NAME}",
            "NAME", deviceName);
    }

    getCmd = std::make_shared<NvidiaGpuLongRunningCommand>(
        eid, mctpRequester, std::move(longRunningQueue),
        std::move(longRunningResponseHandler),
        NvidiaGpuLongRunningCommand::Config{
            .metricName = "GPU ECC Mode",
            .messageType = gpu::MessageType::PLATFORM_ENVIRONMENTAL,
            .commandId = gpu::PlatformEnvironmentalCommands::GET_ECC_MODE,
            .requestSize = gpu::getEccModeRequestSize,
            .encodeRequest =
                std::bind_front(&gpu::encodeGetEccModeRequest, uint8_t{0}),
            .onImmediateSuccess =
                std::bind_front(&NvidiaGpuEccMode::onGetImmediateSuccess,
                                eccModeInterface, eid),
            .onLongRunningPayload =
                std::bind_front(&NvidiaGpuEccMode::onGetLongRunningPayload,
                                eccModeInterface, eid),
        });
}

void NvidiaGpuEccMode::update()
{
    getCmd->update();
}

void NvidiaGpuEccMode::onGetImmediateSuccess(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& eccModeInterface,
    uint8_t eid, std::span<const uint8_t> fullBuffer)
{
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    bool active = false;
    bool enabled = false;
    const int rc = gpu::decodeGetEccModeResponse(fullBuffer, cc, reasonCode,
                                                 active, enabled);
    if (rc != 0)
    {
        lg2::error("Error updating GPU ECC Mode: decode failed, "
                   "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
                   "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                   "EID", eid);
        return;
    }

    applyEccModeToDbus(eccModeInterface, active, enabled);
}

void NvidiaGpuEccMode::onGetLongRunningPayload(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& eccModeInterface,
    uint8_t eid, std::span<const uint8_t> payload)
{
    bool active = false;
    bool enabled = false;
    const int rc = gpu::decodeGetEccModeResponse(payload, active, enabled);
    if (rc != 0)
    {
        lg2::error("Error updating GPU ECC Mode: "
                   "failed to unpack long running flags, rc={RC}, EID={EID}",
                   "RC", rc, "EID", eid);
        return;
    }

    applyEccModeToDbus(eccModeInterface, active, enabled);
}

void NvidiaGpuEccMode::applyEccModeToDbus(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& eccModeInterface,
    bool active, bool enabled)
{
    eccModeInterface->set_property("Active", active);
    eccModeInterface->set_property("Enabled", enabled);
}
