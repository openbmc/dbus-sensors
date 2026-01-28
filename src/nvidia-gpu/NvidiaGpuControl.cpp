/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuControl.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

constexpr const char* controlPowerPrefix =
    "/xyz/openbmc_project/control/power/";

NvidiaGpuControl::NvidiaGpuControl(
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    const std::string& inventoryPath, mctp::MctpRequester& mctpRequester,
    uint8_t eid) :
    name(escapeName(deviceName)), objectServer(objectServer),
    mctpRequester(mctpRequester), eid(eid)
{
    const std::string powerControlPath = controlPowerPrefix + name;

    powerCapInterface = objectServer.add_interface(
        powerControlPath, "xyz.openbmc_project.Control.Power.Cap");

    powerCapInterface->register_property("PowerCap", powerCapValue);
    powerCapInterface->register_property("PowerCapEnable", powerCapEnabled);
    powerCapInterface->register_property("MinPowerCapValue", uint32_t{0});
    powerCapInterface->register_property("MaxPowerCapValue", uint32_t{0});
    powerCapInterface->register_property(
        "DefaultPowerCap", uint32_t{0},
        sdbusplus::asio::PropertyPermission::readOnly);

    powerCapInterface->initialize();

    associationInterface =
        objectServer.add_interface(powerControlPath, association::interface);

    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);

    associationInterface->register_property("Associations", associations);
    associationInterface->initialize();
}

NvidiaGpuControl::~NvidiaGpuControl()
{
    objectServer.remove_interface(powerCapInterface);
    objectServer.remove_interface(associationInterface);
}

void NvidiaGpuControl::update()
{
    sendGetPowerLimitsRequest();
}

void NvidiaGpuControl::sendGetPowerLimitsRequest()
{
    constexpr uint32_t devicePowerLimitId = 0;

    const int rc = gpu::encodeGetPowerLimitsRequest(
        0, devicePowerLimitId, getPowerLimitsRequestBuffer);

    if (rc != 0)
    {
        lg2::error(
            "Error encoding GET_POWER_LIMITS request for eid {EID}, rc={RC}",
            "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, getPowerLimitsRequestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaGpuControl> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuControl reference");
                return;
            }
            self->handleGetPowerLimitsResponse(ec, buffer);
        });
}

void NvidiaGpuControl::handleGetPowerLimitsResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error getting power limits for eid {EID}: {MSG}", "EID",
                   eid, "MSG", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t persistentPowerLimit = 0;
    uint32_t oneshotPowerLimit = 0;
    uint32_t enforcedLimit = 0;

    const int rc = gpu::decodeGetPowerLimitsResponse(
        buffer, cc, reasonCode, persistentPowerLimit, oneshotPowerLimit,
        enforcedLimit);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding power limits for eid {EID}: rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
            reasonCode);
        return;
    }

    powerCapValue = enforcedLimit;
    powerCapEnabled = (enforcedLimit > 0 && enforcedLimit != 0xFFFFFFFF);

    powerCapInterface->set_property("PowerCap", powerCapValue);
    powerCapInterface->set_property("PowerCapEnable", powerCapEnabled);
}
