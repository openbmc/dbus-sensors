/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// (chassis Board path, EM Asset.SerialNumber) collected from
// entity-manager. The device's NSM SerialNumber is matched against
// these to locate the chassis on which Common.UUID and
// Control.Power.Cap mirror interfaces are published.
struct ChassisCandidate
{
    std::string path;
    std::string serialNumber;
};

class Chassis
{
  public:
    Chassis(sdbusplus::asio::object_server& objectServer,
            std::shared_ptr<const std::vector<ChassisCandidate>> candidates);
    ~Chassis();

    Chassis(const Chassis&) = delete;
    Chassis& operator=(const Chassis&) = delete;
    Chassis(Chassis&&) = delete;
    Chassis& operator=(Chassis&&) = delete;

    // Match nsmSerial (hex-encoded) against EM Asset.SerialNumber of each
    // candidate; on a substring match, create Common.UUID and
    // Control.Power.Cap interfaces on the matched chassis Board path and
    // seed them with any cached UUID / Min / Max values.
    void onSerialNumber(const std::string& nsmSerial);

    // Cache the UUID; if the chassis interface already exists, also write
    // through to it.
    void onUuid(const std::string& uuid);

    // Cache MinPowerCapValue (watts); write through if attached.
    void onMinPower(uint32_t value);

    // Cache MaxPowerCapValue (watts); write through if attached.
    void onMaxPower(uint32_t value);

    // Cache the SKU string; write through if attached.
    void onSku(const std::string& sku);

  private:
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<const std::vector<ChassisCandidate>> candidates;
    std::shared_ptr<sdbusplus::asio::dbus_interface> uuidInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> powerCapInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> skuInterface;
    std::optional<std::string> cachedUuid;
    std::optional<uint32_t> cachedMinPower;
    std::optional<uint32_t> cachedMaxPower;
    std::optional<std::string> cachedSku;
};
