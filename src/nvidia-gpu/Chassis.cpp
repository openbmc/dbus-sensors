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
static constexpr const char* powerCapIfaceName =
    "xyz.openbmc_project.Control.Power.Cap";

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
    if (powerCapInterface)
    {
        objectServer.remove_interface(powerCapInterface);
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

        powerCapInterface = objectServer.add_interface(path, powerCapIfaceName);
        powerCapInterface->register_property(
            "MinPowerCapValue", cachedMinPower.value_or(uint32_t{0}));
        powerCapInterface->register_property(
            "MaxPowerCapValue",
            cachedMaxPower.value_or(std::numeric_limits<uint32_t>::max()));
        powerCapInterface->initialize();
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
    if (powerCapInterface)
    {
        powerCapInterface->set_property("MinPowerCapValue", value);
    }
}

void Chassis::onMaxPower(uint32_t value)
{
    cachedMaxPower = value;
    if (powerCapInterface)
    {
        powerCapInterface->set_property("MaxPowerCapValue", value);
    }
}
