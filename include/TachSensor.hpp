#pragma once

#include <Thresholds.hpp>
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
class TachSensor : public Sensor
{
  public:
    std::string name;
    std::string configuration;
    TachSensor(const std::string &path,
               sdbusplus::asio::object_server &objectServer,
               std::shared_ptr<sdbusplus::asio::connection> &conn,
               std::unique_ptr<PresenceSensor> &&presence,
               boost::asio::io_service &io, const std::string &fanName,
               std::vector<thresholds::Threshold> &&thresholds,
               const std::string &sensorConfiguration);
    ~TachSensor();

  private:
    std::string path;
    sdbusplus::asio::object_server &objServer;
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
