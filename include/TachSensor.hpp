#pragma once
#include <Thresholds.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

class PresenceSensor
{
  public:
    PresenceSensor(const std::string& pinName, bool inverted,
                   boost::asio::io_service& io, const std::string& name);
    ~PresenceSensor();

    void monitorPresence(void);
    void read(void);
    bool getValue(void);

  private:
    bool status = true;
    gpiod::line gpioLine;
    boost::asio::posix::stream_descriptor gpioFd;
    std::string name;
};

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
};

class TachSensor : public Sensor
{
  public:
    TachSensor(const std::string& path, const std::string& objectType,
               sdbusplus::asio::object_server& objectServer,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               std::unique_ptr<PresenceSensor>&& presence,
               std::optional<RedundancySensor>* redundancy,
               boost::asio::io_service& io, const std::string& fanName,
               std::vector<thresholds::Threshold>&& thresholds,
               const std::string& sensorConfiguration,
               const std::pair<size_t, size_t>& limits,
               const PowerState& powerState,
               const std::optional<std::string>& led);
    ~TachSensor() override;

  private:
    sdbusplus::asio::object_server& objServer;
    std::optional<RedundancySensor>* redundancy;
    std::unique_ptr<PresenceSensor> presence;
    std::shared_ptr<sdbusplus::asio::dbus_interface> itemIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> itemAssoc;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    std::string path;
    std::optional<std::string> led;
    bool ledState = false;
    void setupRead(void);
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
};

inline void logFanInserted(const std::string& device)
{
    auto msg = "OpenBMC.0.1.FanInserted";
    lg2::error("Fan Inserted", "REDFISH_MESSAGE_ID", msg,
               "REDFISH_MESSAGE_ARGS", device);
}

inline void logFanRemoved(const std::string& device)
{
    auto msg = "OpenBMC.0.1.FanRemoved";
    lg2::error("Fan Removed", "REDFISH_MESSAGE_ID", msg, "REDFISH_MESSAGE_ARGS",
               device);
}

inline void logFanRedundancyLost(void)
{
    auto msg = "OpenBMC.0.1.FanRedundancyLost";
    lg2::error("Fan Inserted", "REDFISH_MESSAGE_ID", msg);
}

inline void logFanRedundancyRestored(void)
{
    auto msg = "OpenBMC.0.1.FanRedundancyRegained";
    lg2::error("Fan Removed", "REDFISH_MESSAGE_ID", msg);
}
