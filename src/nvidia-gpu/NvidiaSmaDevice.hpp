/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuPowerSensor.hpp"
#include "NvidiaGpuTempSensor.hpp"
#include "NvidiaGpuVoltageSensor.hpp"
#include "NvidiaSensorConfig.hpp"
#include "NvidiaSmaLeakSensor.hpp"
#include "Thresholds.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

enum class SmaSensorKind
{
    Temperature,
    Power,
    Voltage,
};

struct SmaSensorConfig
{
    SmaSensorKind kind{};
    uint8_t id{};
    std::string name;
    // GCC's -Wmissing-field-initializers flags designated initializers that
    // leave out a member without a default member initializer.
    // NOLINTNEXTLINE(readability-redundant-member-init)
    std::vector<thresholds::Threshold> thresholds{};
    // Thresholds whose value the SMA holds, read with Read Thermal
    // Parameters before the sensor is published.
    // NOLINTNEXTLINE(readability-redundant-member-init)
    std::vector<thresholds::Threshold> dynamicThresholds{};
};

// Only an SMA lists its sensors, so they are kept apart from the settings
// every device shares.
struct SmaDeviceConfigs
{
    std::vector<SmaSensorConfig> sensors;
};

constexpr std::chrono::seconds dynamicThresholdRetryDelay{5};
constexpr int maxDynamicThresholdAttempts = 3;

class SmaDevice : public std::enable_shared_from_this<SmaDevice>
{
  public:
    SmaDevice(const NvidiaMctpVdmConfig& config,
              const SmaDeviceConfigs& smaConfig,
              const std::shared_ptr<sdbusplus::asio::connection>& conn,
              uint8_t eid, boost::asio::io_context& io,
              mctp::MctpRequester& mctpRequester,
              sdbusplus::asio::object_server& objectServer,
              std::chrono::milliseconds dynamicThresholdRetryMs =
                  dynamicThresholdRetryDelay);

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

    void setOnline();

    void setEid(uint8_t newEid);

  private:
    void makeSensors();

    void readDynamicThresholds(size_t index);

    void processDynamicThresholdsResponse(size_t index,
                                          const std::error_code& ec,
                                          std::span<const uint8_t> response);

    void scheduleDynamicThresholdRetry(size_t index);

    void retryDynamicThresholds();

    void publishSensor(const SmaSensorConfig& sensor);

    void initLeakSensors();

    void processLeakSensorsResponse(const std::error_code& ec,
                                    std::span<const uint8_t> response);

    // Build the SMA inventory object (Inventory.Item + OperationalStatus) so
    // the device's reachability is observable on D-Bus.
    void makeInventory();

    // OperationalStatus.Functional: true while polling (online), false while
    // offline (Degraded/Recovering/Removed).
    void setFunctional(bool functional);

    void readPriority();

    void readRoundRobin();

    uint8_t eid{};

    bool offline{false};

    std::chrono::milliseconds sensorPollMs;

    std::chrono::milliseconds dynamicThresholdRetryMs;

    boost::asio::steady_timer dynamicThresholdRetryTimer;

    boost::asio::steady_timer waitTimerPriority;

    boost::asio::steady_timer waitTimerRoundRobin;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    sdbusplus::asio::object_server& objectServer;

    std::vector<std::shared_ptr<NvidiaGpuTempSensor>> tempSensors;

    std::vector<std::shared_ptr<NvidiaGpuPowerSensor>> powerSensors;

    std::vector<std::shared_ptr<NvidiaGpuVoltageSensor>> voltageSensors;

    std::vector<std::shared_ptr<NvidiaSmaLeakSensor>> leakSensors;

    // TODO: temporary - reuses the generic xyz.openbmc_project.Inventory.Item
    // because there is no dedicated inventory item type for an MCTP
    // bridge/management controller yet. A proper PDI inventory item should be
    // added later and swapped in here.
    std::shared_ptr<sdbusplus::asio::dbus_interface> itemInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> operationalStatusInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    SmaDeviceConfigs smaConfig;

    // How many Read Thermal Parameters requests failed in transport, by index
    // into smaConfig.sensors.
    std::unordered_map<size_t, int> dynamicThresholdAttempts;

    // Indexes into smaConfig.sensors of the sensors whose dynamic thresholds
    // are read again when dynamicThresholdRetryTimer expires.
    std::vector<size_t> dynamicThresholdRetries;

    std::string name;

    sdbusplus::object_path path;

    std::string inventoryPath;
};
