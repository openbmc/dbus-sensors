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
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

constexpr const char* controlPowerPrefix =
    "/xyz/openbmc_project/control/power/";

constexpr uint32_t milliwattsPerWatt = 1000;
constexpr uint32_t powerLimitUnlimited = std::numeric_limits<uint32_t>::max();

NvidiaGpuControl::NvidiaGpuControl(
    sdbusplus::asio::object_server& objectServer, const std::string& deviceName,
    const std::string& inventoryPath, mctp::MctpRequester& mctpRequester,
    mctp::Endpoint endpoint,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& powerCapIface) :
    powerCapInterface(powerCapIface), name(escapeName(deviceName)),
    objectServer(objectServer), mctpRequester(mctpRequester), endpoint{endpoint}
{
    const std::string powerControlPath = controlPowerPrefix + name;

    associationInterface =
        objectServer.add_interface(powerControlPath, association::interface);

    std::vector<Association> associations;
    associations.emplace_back("controlling", "controlled_by", inventoryPath);

    associationInterface->register_property("Associations", associations);
    associationInterface->initialize();
}

NvidiaGpuControl::~NvidiaGpuControl()
{
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
            "Error encoding GET_POWER_LIMITS request for eid {EID} net {NET}, rc={RC}",
            "EID", endpoint.eid, "NET", endpoint.network, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        endpoint, getPowerLimitsRequestBuffer,
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
        lg2::error("Error getting power limits for eid {EID} net {NET}: {MSG}",
                   "EID", endpoint.eid, "NET", endpoint.network, "MSG",
                   ec.message());
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
            "Error decoding power limits for eid {EID} net {NET}: rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", endpoint.eid, "NET", endpoint.network, "RC", rc, "CC",
            static_cast<uint8_t>(cc), "RESC", reasonCode);
        return;
    }

    // PDI specifies PowerCap is in Watts; device reports milliwatts, so
    // convert.
    powerCapValue = enforcedLimit / milliwattsPerWatt;
    powerCapEnabled =
        (enforcedLimit > 0 && enforcedLimit != powerLimitUnlimited);

    powerCapInterface->set_property("PowerCap", powerCapValue);
    powerCapInterface->set_property("PowerCapEnable", powerCapEnabled);
}
