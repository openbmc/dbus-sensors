/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Chassis.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

static constexpr const char* uuidIfaceName = "xyz.openbmc_project.Common.UUID";
static constexpr const char* acceleratorIfaceName =
    "xyz.openbmc_project.Inventory.Item.Accelerator";
static constexpr const char* skuIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.SKU";
static constexpr const char* locationCodeIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.LocationCode";
static constexpr const char* slotIfaceName =
    "xyz.openbmc_project.Inventory.Connector.Slot";

Chassis::Chassis(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<const std::vector<ChassisCandidate>> candidates) :
    objectServer(objectServer), candidates(std::move(candidates))
{}

Chassis::~Chassis()
{
    if (uuidInterface)
    {
        objectServer.remove_interface(uuidInterface);
    }
    if (acceleratorInterface)
    {
        objectServer.remove_interface(acceleratorInterface);
    }
    if (skuInterface)
    {
        objectServer.remove_interface(skuInterface);
    }
    if (locationCodeInterface)
    {
        objectServer.remove_interface(locationCodeInterface);
    }
    if (slotInterface)
    {
        objectServer.remove_interface(slotInterface);
    }
}

void Chassis::onSerialNumber(const std::string& nsmSerial)
{
    if (uuidInterface || !candidates || nsmSerial.empty())
    {
        return;
    }

    // EM Asset.SerialNumber is hex-encoded ASCII read from the FRU EEPROM,
    // which embeds the device SerialNumber inside a larger string (board
    // revision, manufacturing date, etc.). Hex-encode the NSM SerialNumber
    // and search for it as a substring.
    std::string nsmHex;
    nsmHex.reserve(nsmSerial.size() * 2);
    for (unsigned char c : nsmSerial)
    {
        nsmHex += std::format("{:02X}", c);
    }

    for (const auto& [path, emSerial] : *candidates)
    {
        if (emSerial.find(nsmHex) == std::string::npos)
        {
            continue;
        }

        uuidInterface = objectServer.add_interface(path, uuidIfaceName);
        uuidInterface->register_property("UUID",
                                         cachedUuid.value_or(std::string{}));
        uuidInterface->initialize();

        acceleratorInterface =
            objectServer.add_interface(path, acceleratorIfaceName);
        acceleratorInterface->register_property(
            "MinPowerWatts",
            cachedMinPower.value_or(std::numeric_limits<uint32_t>::max()));
        acceleratorInterface->register_property(
            "MaxPowerWatts",
            cachedMaxPower.value_or(std::numeric_limits<uint32_t>::max()));
        acceleratorInterface->initialize();

        skuInterface = objectServer.add_interface(path, skuIfaceName);
        skuInterface->register_property("SKU",
                                        cachedSku.value_or(std::string{}));
        skuInterface->initialize();

        // Hardcoded silk-screen label derived from the chassis path
        // suffix (Nvidia_RTX_PRO_6000_Blackwell_<N> -> "GPU_SLOT_<N>")
        // until Entity-Manager publishes Decorator.LocationCode from
        // platform JSON.
        const auto underscore = path.find_last_of('_');
        const std::string locationCode =
            (underscore != std::string::npos)
                ? "GPU_SLOT_" + path.substr(underscore + 1)
                : std::string{};
        locationCodeInterface =
            objectServer.add_interface(path, locationCodeIfaceName);
        locationCodeInterface->register_property("LocationCode", locationCode);
        locationCodeInterface->initialize();

        // Marker interface; LocationType is derived from the interface
        // presence by bmcweb's location_util::getLocationType().
        slotInterface = objectServer.add_interface(path, slotIfaceName);
        slotInterface->initialize();
        return;
    }
}

void Chassis::onUuid(const std::string& uuid)
{
    cachedUuid = uuid;
    if (uuidInterface)
    {
        uuidInterface->set_property("UUID", uuid);
    }
}

void Chassis::onMinPower(uint32_t value)
{
    cachedMinPower = value;
    if (acceleratorInterface)
    {
        acceleratorInterface->set_property("MinPowerWatts", value);
    }
}

void Chassis::onMaxPower(uint32_t value)
{
    cachedMaxPower = value;
    if (acceleratorInterface)
    {
        acceleratorInterface->set_property("MaxPowerWatts", value);
    }
}

void Chassis::onSku(const std::string& sku)
{
    cachedSku = sku;
    if (skuInterface)
    {
        skuInterface->set_property("SKU", sku);
    }
}
