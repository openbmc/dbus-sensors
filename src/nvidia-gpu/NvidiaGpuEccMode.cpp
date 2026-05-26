/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuEccMode.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuLongRunningCommand.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
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

namespace
{
constexpr const char* controlProcessorPrefix =
    "/xyz/openbmc_project/control/processor/";
constexpr const char* inventoryPrefix = "/xyz/openbmc_project/inventory/";
constexpr const char* eccModeInterfaceName =
    "xyz.openbmc_project.Control.Processor.EccMode";
} // namespace

NvidiaGpuEccMode::NvidiaGpuEccMode(
    mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    uint8_t eid, std::shared_ptr<SerialQueue> longRunningQueue,
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler) :
    enabledValue(std::make_shared<bool>(false))
{
    const std::string controlPath =
        std::string(controlProcessorPrefix) + deviceName;

    eccModeInterface =
        objectServer.add_interface(controlPath, eccModeInterfaceName);

    // Active is read-only. sdbusplus owns the storage; the GET pipeline
    // refreshes it via set_property without invoking any setter callback.
    eccModeInterface->register_property(
        "Active", false, sdbusplus::asio::PropertyPermission::readOnly);

    // Enabled is writable. An external write applies the value optimistically
    // and dispatches an NSM Set ECC Mode through setCmd. Hardware-driven
    // refreshes from the GET pipeline bypass the setter by updating
    // *enabledValue directly and calling signal_property, so they never
    // retrigger the writable setter callback.
    eccModeInterface->register_property<bool>(
        "Enabled", false,
        [this](const bool& req, bool& existing) {
            existing = req;
            *enabledValue = req;
            setCmd->update();
            return true;
        },
        [enabledPtr{enabledValue}](bool&) { return *enabledPtr; });

    if (!eccModeInterface->initialize())
    {
        lg2::error("Failed to initialize ECC mode interface for GPU {NAME}",
                   "NAME", deviceName);
    }

    // Link the control object to the processor inventory item it controls.
    // The mapper creates the reverse controlled_by edge automatically.
    const std::string inventoryPath = std::string(inventoryPrefix) + deviceName;

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
        eid, mctpRequester, longRunningQueue, longRunningResponseHandler,
        NvidiaGpuLongRunningCommand::Config{
            .metricName = "GPU ECC Mode",
            .messageType = gpu::MessageType::PLATFORM_ENVIRONMENTAL,
            .commandId = gpu::PlatformEnvironmentalCommands::GET_ECC_MODE,
            .requestSize = gpu::getEccModeRequestSize,
            .encodeRequest =
                std::bind_front(&gpu::encodeGetEccModeRequest, uint8_t{0}),
            .onImmediateSuccess =
                std::bind_front(&NvidiaGpuEccMode::onGetImmediateSuccess,
                                eccModeInterface, enabledValue, eid),
            .onLongRunningPayload =
                std::bind_front(&NvidiaGpuEccMode::onGetLongRunningPayload,
                                eccModeInterface, enabledValue, eid),
        });

    setCmd = std::make_shared<NvidiaGpuLongRunningCommand>(
        eid, mctpRequester, std::move(longRunningQueue),
        std::move(longRunningResponseHandler),
        NvidiaGpuLongRunningCommand::Config{
            .metricName = "GPU ECC Mode Set",
            .messageType = gpu::MessageType::PLATFORM_ENVIRONMENTAL,
            .commandId = gpu::PlatformEnvironmentalCommands::SET_ECC_MODE,
            .requestSize = gpu::setEccModeRequestSize,
            .encodeRequest =
                [enabledPtr{enabledValue}](std::span<uint8_t> buf) {
                    return gpu::encodeSetEccModeRequest(0, *enabledPtr, buf);
                },
            // SET carries no value to apply on success; the optimistic Enabled
            // already reflects the request.
            .onImmediateSuccess = [](std::span<const uint8_t>) {},
            .onLongRunningPayload = [](std::span<const uint8_t>) {},
            // On any SET failure the optimistic Enabled may be wrong; refresh
            // from hardware now instead of waiting for the next GET poll.
            .onError = [getCmd{getCmd}]() { getCmd->update(); },
        });
}

void NvidiaGpuEccMode::update()
{
    getCmd->update();
}

void NvidiaGpuEccMode::onGetImmediateSuccess(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& eccModeInterface,
    const std::shared_ptr<bool>& enabledValue, uint8_t eid,
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

    applyEccModeToDbus(eccModeInterface, enabledValue, active, enabled);
}

void NvidiaGpuEccMode::onGetLongRunningPayload(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& eccModeInterface,
    const std::shared_ptr<bool>& enabledValue, uint8_t eid,
    std::span<const uint8_t> payload)
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

    applyEccModeToDbus(eccModeInterface, enabledValue, active, enabled);
}

void NvidiaGpuEccMode::applyEccModeToDbus(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& eccModeInterface,
    const std::shared_ptr<bool>& enabledValue, bool active, bool enabled)
{
    // Active is owned by sdbusplus; set_property updates and signals.
    eccModeInterface->set_property("Active", active);

    // Enabled is member-backed; update the value and emit the signal manually.
    // signal_property does not invoke the registered setter, so this
    // hardware-driven refresh never retriggers an NSM Set.
    *enabledValue = enabled;
    eccModeInterface->signal_property("Enabled");
}
