/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "DeviceInterface.hpp"
#include "MctpRequester.hpp"
#include "NvidiaGpuTempSensor.hpp"
#include "NvidiaSensorConfig.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

class SmaDevice :
    public DeviceInterface,
    public std::enable_shared_from_this<SmaDevice>
{
  public:
    SmaDevice(const SensorConfigs& configs, const std::string& name,
              const std::string& path,
              const std::shared_ptr<sdbusplus::asio::connection>& conn,
              uint8_t eid, boost::asio::io_context& io,
              mctp::MctpRequester& mctpRequester,
              sdbusplus::asio::object_server& objectServer);

    const std::string& getPath() const override
    {
        return path;
    }

    void init() override;

    void setOffline() override;

    void setOnline() override;

    void setEid(uint8_t newEid) override;

  private:
    void makeSensors();

    // Build the SMA inventory object (Inventory.Item + OperationalStatus) so
    // the device's reachability is observable on D-Bus.
    void makeInventory();

    // OperationalStatus.Functional: true while polling (online), false while
    // offline (Degraded/Recovering/Removed).
    void setFunctional(bool functional);

    void read();

    uint8_t eid{};

    std::chrono::milliseconds sensorPollMs;

    boost::asio::steady_timer waitTimer;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    std::shared_ptr<NvidiaGpuTempSensor> tempSensor;

    // TODO: temporary - reuses the generic xyz.openbmc_project.Inventory.Item
    // because there is no dedicated inventory item type for an MCTP
    // bridge/management controller yet. A proper PDI inventory item should be
    // added later and swapped in here.
    std::shared_ptr<sdbusplus::asio::dbus_interface> itemInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> operationalStatusInterface;

    SensorConfigs configs;

    std::string name;

    std::string path;

    std::string inventoryPath;
};
