#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <memory>
#include <optional>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>
#include <utility>
#include <vector>

class PresenceSensor
{
  public:
    PresenceSensor(const size_t index, bool inverted,
                   boost::asio::io_service& io, const std::string& name);
    ~PresenceSensor();

    void monitorPresence(void);
    void read(void);
    bool getValue(void);

  private:
    bool status = true;
    bool inverted;
    boost::asio::ip::tcp::socket inputDev;
    int fd;
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
               const std::pair<size_t, size_t>& limits);
    ~TachSensor();

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
    int errCount;
    void setupRead(void);
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
};

inline void logFanInserted(const std::string& device)
{
    phosphor::logging::log<phosphor::logging::level::ERR>(
        "Fan Inserted",
        phosphor::logging::entry("REDFISH_MESSAGE_ID",
                                 "OpenBMC.0.1.FanInserted"),
        phosphor::logging::entry("REDFISH_MESSAGE_ARGS", device_c.str()));
}

inline void logFanRemoved(const std::string& device)
{
    phosphor::logging::log<phosphor::logging::level::ERR>(
        "Fan Removed",
        phosphor::logging::entry("REDFISH_MESSAGE_ID",
                                 "OpenBMC.0.1.FanRemoved"),
        phosphor::logging::entry("REDFISH_MESSAGE_ARGS", device.c_str()));
}

inline void logFanRedundancyLost(void)
{
    phosphor::logging::log<phosphor::logging::level::ERR>(
        "Fan Inserted",
        phosphor::logging::entry("REDFISH_MESSAGE_ID",
                                 "OpenBMC.0.1.FanRedundancyLost"));
}

inline void logFanRedundancyRestored(void)
{
    phosphor::logging::log<phosphor::logging::level::ERR>(
        "Fan Removed",
        phosphor::logging::entry("REDFISH_MESSAGE_ID",
                                 "OpenBMC.0.1.FanRedundancyRegained"));
}
