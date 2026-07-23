/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Chassis.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>

static constexpr const char* uuidIfaceName = "xyz.openbmc_project.Common.UUID";
static constexpr const char* powerBoundsIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.PowerBounds";
static constexpr const char* skuIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.SKU";
static constexpr const char* locationCodeIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.LocationCode";
static constexpr const char* slotIfaceName =
    "xyz.openbmc_project.Inventory.Connector.Slot";

Chassis::Chassis(sdbusplus::asio::object_server& objectServer,
                 const std::string& chassisPath) : objectServer(objectServer)
{
    uuidInterface = objectServer.add_interface(chassisPath, uuidIfaceName);
    uuidInterface->register_property("UUID", std::string{});
    if (!uuidInterface->initialize())
    {
        lg2::error("Error initializing UUID interface for chassis {PATH}",
                   "PATH", chassisPath);
    }

    powerBoundsInterface =
        objectServer.add_interface(chassisPath, powerBoundsIfaceName);
    powerBoundsInterface->register_property(
        "MinPowerWatts", std::numeric_limits<uint32_t>::max());
    powerBoundsInterface->register_property(
        "MaxPowerWatts", std::numeric_limits<uint32_t>::max());
    if (!powerBoundsInterface->initialize())
    {
        lg2::error(
            "Error initializing PowerBounds interface for chassis {PATH}",
            "PATH", chassisPath);
    }

    skuInterface = objectServer.add_interface(chassisPath, skuIfaceName);
    skuInterface->register_property("SKU", std::string{});
    if (!skuInterface->initialize())
    {
        lg2::error("Error initializing SKU interface for chassis {PATH}",
                   "PATH", chassisPath);
    }

    // Hardcoded silk-screen label derived from the board object path suffix
    // (Nvidia_RTX_PRO_6000_Blackwell_<N> -> "GPU_SLOT_<N>") until
    // Entity-Manager publishes Decorator.LocationCode from platform JSON.
    const auto underscore = chassisPath.find_last_of('_');
    const std::string locationCode =
        (underscore != std::string::npos)
            ? "GPU_SLOT_" + chassisPath.substr(underscore + 1)
            : std::string{};
    locationCodeInterface =
        objectServer.add_interface(chassisPath, locationCodeIfaceName);
    locationCodeInterface->register_property("LocationCode", locationCode);
    if (!locationCodeInterface->initialize())
    {
        lg2::error(
            "Error initializing LocationCode interface for chassis {PATH}",
            "PATH", chassisPath);
    }

    // Marker interface; bmcweb's location_util::getLocationType() derives the
    // Redfish LocationType enum ("Slot") from its presence.
    slotInterface = objectServer.add_interface(chassisPath, slotIfaceName);
    if (!slotInterface->initialize())
    {
        lg2::error("Error initializing Slot interface for chassis {PATH}",
                   "PATH", chassisPath);
    }
}

Chassis::~Chassis()
{
    objectServer.remove_interface(uuidInterface);
    objectServer.remove_interface(powerBoundsInterface);
    objectServer.remove_interface(skuInterface);
    objectServer.remove_interface(locationCodeInterface);
    objectServer.remove_interface(slotInterface);
}

void Chassis::onUuid(const std::string& uuid)
{
    uuidInterface->set_property("UUID", uuid);
}

void Chassis::onMinPowerWatts(uint32_t watts)
{
    powerBoundsInterface->set_property("MinPowerWatts", watts);
}

void Chassis::onMaxPowerWatts(uint32_t watts)
{
    powerBoundsInterface->set_property("MaxPowerWatts", watts);
}

void Chassis::onSku(const std::string& sku)
{
    skuInterface->set_property("SKU", sku);
}
