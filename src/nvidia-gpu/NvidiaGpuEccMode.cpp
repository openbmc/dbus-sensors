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
#include <sdbusplus/message/native_types.hpp>

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
        longRunningResponseHandler) :
    mctpRequester(mctpRequester), eid(eid),
    longRunningQueue(std::move(longRunningQueue)),
    longRunningResponseHandler(std::move(longRunningResponseHandler)),
    objectServer(objectServer)
{
    const sdbusplus::object_path controlPath =
        sdbusplus::object_path("/xyz/openbmc_project/control/processor") /
        deviceName;

    eccModeInterface = objectServer.add_interface(
        controlPath, "xyz.openbmc_project.Control.Processor.EccMode");

    eccModeInterface->register_property(
        "Active", false, sdbusplus::asio::PropertyPermission::readOnly);
    eccModeInterface->register_property(
        "Enabled", false, sdbusplus::asio::PropertyPermission::readOnly);

    if (!eccModeInterface->initialize())
    {
        lg2::error(
            "Error initializing ECC mode interface for {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
    }

    const sdbusplus::object_path inventoryPath = inventoryPrefix / deviceName;

    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);

    eccModeAssociationInterface =
        objectServer.add_interface(controlPath, association::interface);
    eccModeAssociationInterface->register_property("Associations",
                                                   associations);

    if (!eccModeAssociationInterface->initialize())
    {
        lg2::error(
            "Error initializing ECC mode association interface for {NAME}, eid={EID}",
            "NAME", deviceName, "EID", eid);
    }

    getCmd = std::make_shared<NvidiaGpuLongRunningCommand>(
        eid, this->mctpRequester, this->longRunningQueue,
        this->longRunningResponseHandler,
        NvidiaGpuLongRunningCommand::Config{
            .metricName = "GPU ECC Mode",
            .messageType = gpu::MessageType::PLATFORM_ENVIRONMENTAL,
            .commandId = gpu::PlatformEnvironmentalCommands::GET_ECC_MODE,
            .requestSize = gpu::getEccModeRequestSize,
            .encodeRequest =
                std::bind_front(&gpu::encodeGetEccModeRequest, uint8_t{0}),
            .onImmediateSuccess =
                std::bind_front(&NvidiaGpuEccMode::onGetImmediateSuccess, this),
            .onLongRunningPayload = std::bind_front(
                &NvidiaGpuEccMode::onGetLongRunningPayload, this),
        });
}

NvidiaGpuEccMode::~NvidiaGpuEccMode()
{
    objectServer.remove_interface(eccModeInterface);
    objectServer.remove_interface(eccModeAssociationInterface);
}

void NvidiaGpuEccMode::update()
{
    getCmd->update();
}

void NvidiaGpuEccMode::onGetImmediateSuccess(
    std::span<const uint8_t> fullBuffer)
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

    applyEccModeToDbus(active, enabled);
}

void NvidiaGpuEccMode::onGetLongRunningPayload(std::span<const uint8_t> payload)
{
    bool active = false;
    bool enabled = false;
    const int rc = gpu::decodeGetEccModeResponse(payload, active, enabled);
    if (rc != 0)
    {
        lg2::error("Error updating GPU ECC Mode: "
                   "failed to decode long running response, rc={RC}, EID={EID}",
                   "RC", rc, "EID", eid);
        return;
    }

    applyEccModeToDbus(active, enabled);
}

void NvidiaGpuEccMode::applyEccModeToDbus(bool active, bool enabled)
{
    eccModeInterface->set_property("Active", active);
    eccModeInterface->set_property("Enabled", enabled);
}
