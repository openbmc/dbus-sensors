#pragma once

#include "PresenceGpio.hpp"
#include "asio/Thresholds.hpp"
#include "asio/sensor.hpp"
#include "utils/Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/random_access_file.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace redundancy
{
constexpr const char* full = "Full";
constexpr const char* degraded = "Degraded";
constexpr const char* failed = "Failed";
} // namespace redundancy

class RedundancySensor
{
  public:
    RedundancySensor(size_t count, const std::vector<std::string>& children,
                     sdbusplus::asio::object_server& objectServer,
                     const std::string& sensorConfiguration);
    ~RedundancySensor();

    void update(const std::string& name, bool failed);

  private:
    size_t count;
    std::string state = redundancy::full;
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> association;
    sdbusplus::asio::object_server& objectServer;
    boost::container::flat_map<std::string, bool> statuses;

    static void logFanRedundancyLost()
    {
        const auto* msg = "OpenBMC.0.1.FanRedundancyLost";
        lg2::error("Fan Inserted", "REDFISH_MESSAGE_ID", msg);
    }

    static void logFanRedundancyRestored()
    {
        const auto* msg = "OpenBMC.0.1.FanRedundancyRegained";
        lg2::error("Fan Removed", "REDFISH_MESSAGE_ID", msg);
    }
};

class TachSensor :
    public Sensor,
    public std::enable_shared_from_this<TachSensor>
{
  public:
    TachSensor(const std::string& path, const std::string& objectType,
               sdbusplus::asio::object_server& objectServer,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               std::shared_ptr<PresenceGpio>& presence,
               std::optional<RedundancySensor>* redundancy,
               boost::asio::io_context& io, const std::string& fanName,
               std::vector<thresholds::Threshold>&& thresholds,
               const std::string& sensorConfiguration,
               const std::pair<double, double>& limits,
               const PowerState& powerState,
               const std::optional<std::string>& led);
    ~TachSensor() override;
    void setupRead();

  private:
    // Ordering is important here; readBuf is first so that it's not destroyed
    // while async operations from other member fields might still be using it.
    std::array<char, 128> readBuf{};
    sdbusplus::asio::object_server& objServer;
    std::optional<RedundancySensor>* redundancy;
    std::shared_ptr<PresenceGpio> presence;
    std::shared_ptr<sdbusplus::asio::dbus_interface> itemIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> itemAssoc;
    boost::asio::random_access_file inputDev;
    boost::asio::steady_timer waitTimer;
    std::string path;
    std::optional<std::string> led;
    bool ledState = false;

    void handleResponse(const boost::system::error_code& err, size_t bytesRead);
    void restartRead(size_t pollTime);
    void checkThresholds() override;
};
