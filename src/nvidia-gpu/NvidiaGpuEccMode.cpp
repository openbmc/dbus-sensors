/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuEccMode.hpp"

#include <MctpRequester.hpp>
#include <MessagePackUnpackUtils.hpp>
#include <NvidiaGpuLongRunningCommand.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <SerialQueue.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <utility>

namespace
{
constexpr const char* inventoryPrefix = "/xyz/openbmc_project/inventory/";
constexpr const char* eccModeInterfaceName =
    "xyz.openbmc_project.Control.Processor.EccMode";

// Decode the ECC mode flag byte that both the synchronous SUCCESS payload
// and the long-running event payload carry. Bit 0 is the active mode, bit
// 1 is the mode the GPU will apply on next reset.
int unpackEccFlags(UnpackBuffer& buf, bool& active, bool& enabled)
{
    uint8_t flags = 0;
    const int rc = buf.unpack(flags);
    if (rc != 0)
    {
        return rc;
    }
    active = (flags & 0x01U) != 0;
    enabled = (flags & 0x02U) != 0;
    return 0;
}
} // namespace

NvidiaGpuEccMode::NvidiaGpuEccMode(
    mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    uint8_t eid, std::shared_ptr<SerialQueue> longRunningQueue,
    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler)
{
    const std::string inventoryPath = std::string(inventoryPrefix) + deviceName;

    eccModeInterface =
        objectServer.add_interface(inventoryPath, eccModeInterfaceName);

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

    getCmd = std::make_shared<NvidiaGpuLongRunningCommand>(
        eid, mctpRequester, std::move(longRunningQueue),
        std::move(longRunningResponseHandler),
        NvidiaGpuLongRunningCommand::Config{
            .metricName = "GPU ECC Mode",
            .messageType = gpu::MessageType::PLATFORM_ENVIRONMENTAL,
            .commandId = gpu::PlatformEnvironmentalCommands::GET_ECC_MODE,
            .requestSize = gpu::getEccModeRequestSize,
            .encodeRequest =
                [](std::span<uint8_t> buf) {
                    return gpu::encodeGetEccModeRequest(0, buf);
                },
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
    uint8_t eid, UnpackBuffer& buf)
{
    uint16_t dataSize = 0;
    int rc = buf.unpack(dataSize);
    if (rc != 0)
    {
        lg2::error("Error updating GPU ECC Mode: "
                   "failed to unpack dataSize, rc={RC}, EID={EID}",
                   "RC", rc, "EID", eid);
        return;
    }

    if (dataSize != sizeof(uint8_t))
    {
        lg2::error("Error updating GPU ECC Mode: "
                   "unexpected dataSize {DS}, EID={EID}",
                   "DS", dataSize, "EID", eid);
        return;
    }

    bool active = false;
    bool enabled = false;
    rc = unpackEccFlags(buf, active, enabled);
    if (rc != 0)
    {
        lg2::error("Error updating GPU ECC Mode: "
                   "failed to unpack flags, rc={RC}, EID={EID}",
                   "RC", rc, "EID", eid);
        return;
    }

    applyEccModeToDbus(eccModeInterface, active, enabled);
}

void NvidiaGpuEccMode::onGetLongRunningPayload(
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& eccModeInterface,
    uint8_t eid, UnpackBuffer& buf)
{
    bool active = false;
    bool enabled = false;
    const int rc = unpackEccFlags(buf, active, enabled);
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
