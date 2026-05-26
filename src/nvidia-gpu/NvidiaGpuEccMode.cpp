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
        longRunningResponseHandler) :
    mctpRequester(mctpRequester), eid(eid),
    longRunningQueue(std::move(longRunningQueue)),
    longRunningResponseHandler(std::move(longRunningResponseHandler))
{
    const std::string controlPath =
        std::string("/xyz/openbmc_project/control/processor/") + deviceName;

    eccModeInterface = objectServer.add_interface(
        controlPath, "xyz.openbmc_project.Control.Processor.EccMode");

    // Active is read-only; the GET pipeline refreshes it via set_property.
    eccModeInterface->register_property(
        "Active", false, sdbusplus::asio::PropertyPermission::readOnly);

    // Enabled is writable and member-backed following the Control.Power.Cap
    // pattern (90189): the getter returns eccModeEnabled and the setter
    // dispatches a SET without writing it. eccModeEnabled is refreshed only
    // from a GET response, so the D-Bus value never drifts from hardware.
    eccModeInterface->register_property<bool>(
        "Enabled", false,
        std::bind_front(&NvidiaGpuEccMode::handleEnabledSet, this),
        [this](bool&) { return eccModeEnabled; });

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

void NvidiaGpuEccMode::update()
{
    getCmd->update();
}

int NvidiaGpuEccMode::handleEnabledSet(const bool& newEnable, bool& /*current*/)
{
    // Dispatch the SET for the requested value. Do not touch eccModeEnabled:
    // it mirrors the device and is refreshed only from a GET response, so a
    // failed SET cannot leave a stale optimistic value on D-Bus.
    sendSetEccModeRequest(newEnable);
    return 1;
}

void NvidiaGpuEccMode::sendSetEccModeRequest(bool enable)
{
    setCmd = std::make_shared<NvidiaGpuLongRunningCommand>(
        eid, mctpRequester, longRunningQueue, longRunningResponseHandler,
        NvidiaGpuLongRunningCommand::Config{
            .metricName = "GPU ECC Mode Set",
            .messageType = gpu::MessageType::PLATFORM_ENVIRONMENTAL,
            .commandId = gpu::PlatformEnvironmentalCommands::SET_ECC_MODE,
            .requestSize = gpu::setEccModeRequestSize,
            .encodeRequest =
                [enable](std::span<uint8_t> buf) {
                    return gpu::encodeSetEccModeRequest(0, enable, buf);
                },
            // Once the SET finishes, re-read ECC mode from hardware so Active
            // and Enabled reflect the new state without waiting for the next
            // GET poll.
            .onImmediateSuccess =
                [this](std::span<const uint8_t>) { getCmd->update(); },
            .onLongRunningPayload =
                [this](std::span<const uint8_t>) { getCmd->update(); },
        });

    setCmd->update();
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
    // Active is read-only and owned by sdbusplus; set_property updates and
    // signals it.
    eccModeInterface->set_property("Active", active);

    // Enabled is member-backed (the getter returns eccModeEnabled). Emit
    // PropertiesChanged only when it actually changes; this runs on every GET
    // poll. signal_property does not invoke the setter, so a hardware-driven
    // refresh never retriggers an NSM Set.
    if (enabled != eccModeEnabled)
    {
        eccModeEnabled = enabled;
        eccModeInterface->signal_property("Enabled");
    }
}
