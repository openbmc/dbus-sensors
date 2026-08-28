/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuTempSensor.hpp"
#include "NvidiaSensorConfig.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

class SmaDevice : public std::enable_shared_from_this<SmaDevice>
{
  public:
    SmaDevice(const SensorConfigs& configs, const std::string& name,
              const sdbusplus::object_path& path,
              const std::shared_ptr<sdbusplus::asio::connection>& conn,
              uint8_t eid, boost::asio::io_context& io,
              mctp::MctpRequester& mctpRequester,
              sdbusplus::asio::object_server& objectServer);

    // The D-Bus path of the EntityManager configuration object the device
    // was created from.
    const sdbusplus::object_path& getPath() const
    {
        return path;
    }

    // Build the device's D-Bus objects. Polling only starts once the device
    // is taken online.
    void init();

    // Stop polling timer(s) and set reading sensors to unavailable. The
    // D-Bus objects are retained.
    void setOffline();

    // Resume polling.
    void setOnline();

    void setEid(uint8_t newEid);

  private:
    void makeSensors();

    // Build the SMA inventory object (Inventory.Item + OperationalStatus) so
    // the device's reachability is observable on D-Bus.
    void makeInventory();

    // OperationalStatus.Functional: true while polling (online), false while
    // offline (Degraded/Recovering/Removed).
    void setFunctional(bool functional);

    void read();

    void updateTempSensors();

    void processTempSensorResponse(const std::error_code& ec,
                                   std::span<const uint8_t> buffer);

    uint8_t eid{};

    std::chrono::milliseconds sensorPollMs;

    boost::asio::steady_timer waitTimer;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    std::map<uint8_t, std::shared_ptr<NvidiaGpuTempSensor>> tempSensors;

    std::array<uint8_t, gpu::getTemperatureReadingRequestSize> tempRequest{};

    bool tempRequestEncoded{false};

    // Kept across polls so that a poll does not allocate.
    std::vector<gpu::TemperatureReading> tempReadings;

    // TODO: temporary - reuses the generic xyz.openbmc_project.Inventory.Item
    // because there is no dedicated inventory item type for an MCTP
    // bridge/management controller yet. A proper PDI inventory item should be
    // added later and swapped in here.
    std::shared_ptr<sdbusplus::asio::dbus_interface> itemInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> operationalStatusInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    SensorConfigs configs;

    std::string name;

    sdbusplus::object_path path;

    std::string inventoryPath;
};
