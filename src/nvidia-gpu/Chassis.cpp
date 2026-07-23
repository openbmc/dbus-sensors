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
}

Chassis::~Chassis()
{
    objectServer.remove_interface(uuidInterface);
    objectServer.remove_interface(powerBoundsInterface);
    objectServer.remove_interface(skuInterface);
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
