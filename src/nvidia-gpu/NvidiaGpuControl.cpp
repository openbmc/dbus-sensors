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
#include <variant>
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
    std::string powerControlPath = controlPowerPrefix + name;

    powerCapInterface = objectServer.add_interface(
        powerControlPath, "xyz.openbmc_project.Control.Power.Cap");

    powerCapInterface->register_property("PowerCap", powerCapValue);
    powerCapInterface->register_property("PowerCapEnable", powerCapEnabled);
    powerCapInterface->register_property("MinPowerCapValue", minPowerCapValue);
    powerCapInterface->register_property("MaxPowerCapValue", maxPowerCapValue);
    powerCapInterface->register_property(
        "DefaultPowerCap", defaultPowerCapValue,
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

void NvidiaGpuControl::init()
{
    sendGetMinPowerLimitRequest();
}

void NvidiaGpuControl::update()
{
    sendGetPowerLimitsRequest();
}

void NvidiaGpuControl::sendGetMinPowerLimitRequest()
{
    const int rc = gpu::encodeGetInventoryInformationRequest(
        0,
        static_cast<uint8_t>(gpu::InventoryPropertyId::MIN_DEVICE_POWER_LIMIT),
        inventoryRequestBuffer);

    if (rc != 0)
    {
        lg2::error(
            "Error encoding GET_INVENTORY (MIN_POWER) request for eid {EID}, rc={RC}",
            "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, inventoryRequestBuffer,
        [this](const std::error_code& ec, std::span<const uint8_t> buffer) {
            handleGetMinPowerLimitResponse(ec, buffer);
        });
}

void NvidiaGpuControl::sendGetMaxPowerLimitRequest()
{
    const int rc = gpu::encodeGetInventoryInformationRequest(
        0,
        static_cast<uint8_t>(gpu::InventoryPropertyId::MAX_DEVICE_POWER_LIMIT),
        inventoryRequestBuffer);

    if (rc != 0)
    {
        lg2::error(
            "Error encoding GET_INVENTORY (MAX_POWER) request for eid {EID}, rc={RC}",
            "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, inventoryRequestBuffer,
        [this](const std::error_code& ec, std::span<const uint8_t> buffer) {
            handleGetMaxPowerLimitResponse(ec, buffer);
        });
}

void NvidiaGpuControl::sendGetPowerLimitsRequest()
{
    constexpr uint32_t devicePowerLimitId = 0;

    const int rc =
        gpu::encodeGetPowerLimitsRequest(0, devicePowerLimitId, requestBuffer);

    if (rc != 0)
    {
        lg2::error(
            "Error encoding GET_POWER_LIMITS request for eid {EID}, rc={RC}",
            "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, requestBuffer,
        [this](const std::error_code& ec, std::span<const uint8_t> buffer) {
            handleGetPowerLimitsResponse(ec, buffer);
        });
}

void NvidiaGpuControl::handleGetMinPowerLimitResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error getting MIN power limit for eid {EID}: {MSG}", "EID",
                   eid, "MSG", ec.message());
        sendGetMaxPowerLimitRequest();
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::InventoryValue value;

    const int rc = gpu::decodeGetInventoryInformationResponse(
        buffer, cc, reasonCode,
        gpu::InventoryPropertyId::MIN_DEVICE_POWER_LIMIT, value);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding MIN power limit for eid {EID}: rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
            reasonCode);
        sendGetMaxPowerLimitRequest();
        return;
    }

    if (std::holds_alternative<uint32_t>(value))
    {
        minPowerCapValue = std::get<uint32_t>(value);
        powerCapInterface->set_property("MinPowerCapValue", minPowerCapValue);
    }

    sendGetMaxPowerLimitRequest();
}

void NvidiaGpuControl::handleGetMaxPowerLimitResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error getting MAX power limit for eid {EID}: {MSG}", "EID",
                   eid, "MSG", ec.message());
        sendGetPowerLimitsRequest();
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::InventoryValue value;

    const int rc = gpu::decodeGetInventoryInformationResponse(
        buffer, cc, reasonCode,
        gpu::InventoryPropertyId::MAX_DEVICE_POWER_LIMIT, value);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding MAX power limit for eid {EID}: rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
            reasonCode);
        sendGetPowerLimitsRequest();
        return;
    }

    if (std::holds_alternative<uint32_t>(value))
    {
        maxPowerCapValue = std::get<uint32_t>(value);
        powerCapInterface->set_property("MaxPowerCapValue", maxPowerCapValue);
    }

    sendGetDefaultPowerLimitRequest();
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

void NvidiaGpuControl::sendGetDefaultPowerLimitRequest()
{
    const int rc = gpu::encodeGetInventoryInformationRequest(
        0,
        static_cast<uint8_t>(
            gpu::InventoryPropertyId::RATED_DEVICE_POWER_LIMIT),
        inventoryRequestBuffer);

    if (rc != 0)
    {
        lg2::error(
            "Error encoding GET_INVENTORY (RATED_POWER) request for eid {EID}, rc={RC}",
            "EID", eid, "RC", rc);
        sendGetPowerLimitsRequest();
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, inventoryRequestBuffer,
        [this](const std::error_code& ec, std::span<const uint8_t> buffer) {
            handleGetDefaultPowerLimitResponse(ec, buffer);
        });
}

void NvidiaGpuControl::handleGetDefaultPowerLimitResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error getting DEFAULT power limit for eid {EID}: {MSG}",
                   "EID", eid, "MSG", ec.message());
        sendGetPowerLimitsRequest();
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::InventoryValue value;

    const int rc = gpu::decodeGetInventoryInformationResponse(
        buffer, cc, reasonCode,
        gpu::InventoryPropertyId::RATED_DEVICE_POWER_LIMIT, value);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding DEFAULT power limit for eid {EID}: rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC",
            reasonCode);
        sendGetPowerLimitsRequest();
        return;
    }

    if (std::holds_alternative<uint32_t>(value))
    {
        defaultPowerCapValue = std::get<uint32_t>(value);
        powerCapInterface->set_property("DefaultPowerCap",
                                        defaultPowerCapValue);
    }

    sendGetPowerLimitsRequest();
}
