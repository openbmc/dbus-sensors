/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <string>

// Publishes the chassis inventory interfaces sourced from the GPU firmware over
// MCTP VDM (the device UUID and the nameplate power range) onto the
// Entity-Manager board object associated with this endpoint. The object path is
// resolved generically by MCTP discovery through the endpoint's configured_by
// association, so no device-attribute matching is done here.
class Chassis
{
  public:
    Chassis(sdbusplus::asio::object_server& objectServer,
            const std::string& chassisPath);
    ~Chassis();

    Chassis(const Chassis&) = delete;
    Chassis& operator=(const Chassis&) = delete;
    Chassis(Chassis&&) = delete;
    Chassis& operator=(Chassis&&) = delete;

    void onUuid(const std::string& uuid);
    void onMinPowerWatts(uint32_t watts);
    void onMaxPowerWatts(uint32_t watts);
    void onSku(const std::string& sku);

  private:
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::dbus_interface> uuidInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> powerBoundsInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> skuInterface;
};
