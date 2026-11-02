/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaDeviceModeControl.hpp"

#include "NvidiaGpuControlErrors.hpp"
#include "NvidiaUtils.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <functional>
#include <string>
#include <system_error>
#include <vector>

static constexpr const char* deviceModeIfaceName =
    "com.nvidia.Control.DeviceMode";
static constexpr const char* deviceModePrefix =
    "/xyz/openbmc_project/control/network_device/";

NvidiaDeviceModeControl::NvidiaDeviceModeControl(
    sdbusplus::asio::object_server& objectServer,
    const std::string& deviceName, const std::string& inventoryNicPath,
    mctp::MctpRequester& mctpRequester, uint8_t eid) :
    name(escapeName(deviceName)), inventoryNicPath(inventoryNicPath),
    objectServer(objectServer), mctpRequester(mctpRequester), eid(eid)
{
    const std::string objPath = std::string(deviceModePrefix) + name;

    deviceModeInterface =
        objectServer.add_interface(objPath, deviceModeIfaceName);

    associationInterface =
        objectServer.add_interface(objPath, association::interface);
    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by",
                              inventoryNicPath);
    associationInterface->register_property("Associations", associations);
    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association interface for DeviceMode for {NAME}, eid={EID}",
            "NAME", name, "EID", eid);
    }

    deviceModeInterface->register_property<std::string>(
        "DPUOperationMode", "DPU",
        std::bind_front(&NvidiaDeviceModeControl::handleDpuModeSet, this),
        [this](std::string&) { return dpuRequested; });
    deviceModeInterface->register_property<std::string>(
        "DPUOperationModeCurrent", "DPU",
        sdbusplus::asio::PropertyPermission::readOnly);
    deviceModeInterface->register_property<bool>(
        "EastWestControlEnabled", false,
        std::bind_front(&NvidiaDeviceModeControl::handleEwEnabledSet, this),
        [this](bool&) { return ewRequested; });
    deviceModeInterface->register_property<bool>(
        "EastWestControlEnabledCurrent", false,
        sdbusplus::asio::PropertyPermission::readOnly);

    if (!deviceModeInterface->initialize())
    {
        lg2::error(
            "Error initializing DeviceMode interface for {NAME}, eid={EID}",
            "NAME", name, "EID", eid);
    }
}

NvidiaDeviceModeControl::~NvidiaDeviceModeControl()
{
    objectServer.remove_interface(associationInterface);
    objectServer.remove_interface(deviceModeInterface);
}

void NvidiaDeviceModeControl::update()
{
    sendGetRequest(gpu::deviceModeIndexDpuOperation);
    sendGetRequest(gpu::deviceModeIndexPcieDevice);
}

void NvidiaDeviceModeControl::sendGetRequest(uint32_t deviceModeIndex)
{
    std::array<uint8_t, gpu::getDeviceModeSettingsV2RequestSize> req{};
    if (gpu::encodeGetDeviceModeSettingsV2Request(0, deviceModeIndex, req) != 0)
    {
        lg2::error(
            "Failed to encode GET DeviceMode index {IDX} for eid {EID}", "IDX",
            deviceModeIndex, "EID", eid);
        return;
    }
    mctpRequester.sendRecvMsg(
        eid, req,
        [weak{weak_from_this()}, deviceModeIndex](
            const std::error_code& ec, std::span<const uint8_t> buffer) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaDeviceModeControl reference");
                return;
            }
            self->handleGetResponse(deviceModeIndex, ec, buffer);
        });
}

void NvidiaDeviceModeControl::handleGetResponse(
    uint32_t deviceModeIndex, const std::error_code& ec,
    std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error reading DeviceMode index {IDX} eid {EID}: {MSG}",
                   "IDX", deviceModeIndex, "EID", eid, "MSG", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    std::vector<uint8_t> cur;
    std::vector<uint8_t> pend;
    if (gpu::decodeGetDeviceModeSettingsV2Response(buffer, cc, reasonCode, cur,
                                                   pend) != 0 ||
        cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding DeviceMode index {IDX} eid {EID}: cc={CC} reason={RESC}",
            "IDX", deviceModeIndex, "EID", eid, "CC",
            static_cast<uint8_t>(cc), "RESC", reasonCode);
        return;
    }

    if (deviceModeIndex == gpu::deviceModeIndexDpuOperation)
    {
        if (!cur.empty() && cur[0] != gpu::deviceModeNoChange)
        {
            const std::string v =
                (cur[0] == gpu::dpuModeNic) ? "NIC" : "DPU";
            if (v != dpuCurrent)
            {
                dpuCurrent = v;
                deviceModeInterface->set_property("DPUOperationModeCurrent",
                                                  dpuCurrent);
            }
        }
        if (!pend.empty() && pend[0] != gpu::deviceModeNoChange)
        {
            const std::string v =
                (pend[0] == gpu::dpuModeNic) ? "NIC" : "DPU";
            if (v != dpuRequested)
            {
                dpuRequested = v;
                deviceModeInterface->signal_property("DPUOperationMode");
            }
        }
    }
    else if (deviceModeIndex == gpu::deviceModeIndexPcieDevice)
    {
        constexpr size_t ewOffset = gpu::pcieSubModeControlledEwTraffic;
        if (cur.size() > ewOffset && cur[ewOffset] != gpu::deviceModeNoChange)
        {
            const bool v = cur[ewOffset] == gpu::ewTrafficEnabled;
            if (v != ewCurrent)
            {
                ewCurrent = v;
                deviceModeInterface->set_property(
                    "EastWestControlEnabledCurrent", ewCurrent);
            }
        }
        if (pend.size() > ewOffset && pend[ewOffset] != gpu::deviceModeNoChange)
        {
            const bool v = pend[ewOffset] == gpu::ewTrafficEnabled;
            if (v != ewRequested)
            {
                ewRequested = v;
                deviceModeInterface->signal_property("EastWestControlEnabled");
            }
        }
    }
}

int NvidiaDeviceModeControl::handleDpuModeSet(const std::string& requested,
                                               std::string& /*current*/)
{
    if (requested != "DPU" && requested != "NIC")
    {
        lg2::error("DPUOperationMode set rejected eid {EID}: bad value {VAL}",
                   "EID", eid, "VAL", requested);
        throw InvalidArgument();
    }
    if (setInflight)
    {
        throw Unavailable();
    }
    const uint8_t raw =
        (requested == "NIC") ? gpu::dpuModeNic : gpu::dpuModeDpu;
    sendSetRequest(gpu::deviceModeIndexDpuOperation, {raw}, requested, false,
                   false);
    return 1;
}

int NvidiaDeviceModeControl::handleEwEnabledSet(const bool& requested,
                                                 bool& /*current*/)
{
    if (setInflight)
    {
        throw Unavailable();
    }
    std::vector<uint8_t> modeData(gpu::pcieDeviceModeDataSize,
                                  gpu::deviceModeNoChange);
    modeData[gpu::pcieSubModeControlledEwTraffic] =
        requested ? gpu::ewTrafficEnabled : gpu::ewTrafficDisabled;
    sendSetRequest(gpu::deviceModeIndexPcieDevice, std::move(modeData), {},
                   true, requested);
    return 1;
}

void NvidiaDeviceModeControl::sendSetRequest(uint32_t deviceModeIndex,
                                             std::vector<uint8_t> modeData,
                                             std::string pendingDpu,
                                             bool pendingEwValid,
                                             bool pendingEw)
{
    const size_t need = (deviceModeIndex == gpu::deviceModeIndexDpuOperation)
                            ? gpu::setDpuModeSettingsV2RequestSize
                            : gpu::setPcieModeSettingsV2RequestSize;
    std::vector<uint8_t> req(need, 0);
    if (gpu::encodeSetDeviceModeSettingsV2Request(
            0, deviceModeIndex, std::span<const uint8_t>(modeData),
            std::span<uint8_t>(req.data(), req.size())) != 0)
    {
        lg2::error("Failed to encode SET DeviceMode index {IDX} eid {EID}",
                   "IDX", deviceModeIndex, "EID", eid);
        return;
    }
    setInflight = true;
    mctpRequester.sendRecvMsg(
        eid, req,
        [weak{weak_from_this()}, pendingDpu = std::move(pendingDpu),
         pendingEwValid, pendingEw](const std::error_code& ec,
                                    std::span<const uint8_t> buffer) {
            auto self = weak.lock();
            if (!self)
            {
                return;
            }
            self->handleSetResponse(pendingDpu, pendingEwValid, pendingEw, ec,
                                    buffer);
        });
}

void NvidiaDeviceModeControl::handleSetResponse(
    std::string pendingDpu, bool pendingEwValid, bool pendingEw,
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    setInflight = false;
    if (ec)
    {
        lg2::error("Error setting DeviceMode eid {EID}: {MSG}", "EID", eid,
                   "MSG", ec.message());
        return;
    }
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    if (gpu::decodeSetDeviceModeSettingsV2Response(buffer, cc, reasonCode) !=
            0 ||
        cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding SET DeviceMode eid {EID}: cc={CC} reason={RESC}",
            "EID", eid, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode);
        return;
    }
    if (!pendingDpu.empty() && pendingDpu != dpuRequested)
    {
        dpuRequested = pendingDpu;
        deviceModeInterface->signal_property("DPUOperationMode");
    }
    if (pendingEwValid && pendingEw != ewRequested)
    {
        ewRequested = pendingEw;
        deviceModeInterface->signal_property("EastWestControlEnabled");
    }
    update();
}
