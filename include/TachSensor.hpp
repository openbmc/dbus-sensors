#pragma once

#include <Thresholds.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

constexpr const char *gpioPath = "/sys/class/gpio/";
class PresenceSensor
{

  public:
    PresenceSensor(const size_t index, bool inverted,
                   boost::asio::io_service &io);
    ~PresenceSensor();

    void monitorPresence(void);
    void read(void);
    bool getValue(void);

  private:
    bool status = true;
    bool inverted;
    boost::asio::ip::tcp::socket inputDev;
    int fd;
};

class RedundancySensor
{
  public:
    RedundancySensor(size_t count, const std::vector<std::string> &children,
                     sdbusplus::asio::object_server &objectServer);
    ~RedundancySensor();

    void update(const std::string &name, bool failed);

  private:
    size_t count;
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface;
    sdbusplus::asio::object_server &objectServer;
    boost::container::flat_map<std::string, bool> statuses;
};

class TachSensor : public Sensor
{
  public:
    std::string configuration;
    TachSensor(const std::string &path,
               sdbusplus::asio::object_server &objectServer,
               std::shared_ptr<sdbusplus::asio::connection> &conn,
               std::unique_ptr<PresenceSensor> &&presence,
               const std::shared_ptr<RedundancySensor> &redundancy,
               boost::asio::io_service &io, const std::string &fanName,
               std::vector<thresholds::Threshold> &&thresholds,
               const std::string &sensorConfiguration);
    ~TachSensor();

  private:
    sdbusplus::asio::object_server &objServer;
    std::shared_ptr<RedundancySensor> redundancy;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    std::unique_ptr<PresenceSensor> presence;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    int errCount;
    double maxValue;
    double minValue;
    void setupRead(void);
    void handleResponse(const boost::system::error_code &err);
    void checkThresholds(void);
    void updateValue(const double &newValue);

    void setInitialProperties(
        std::shared_ptr<sdbusplus::asio::connection> &conn);
};
