#pragma once

#include <Thresholds.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

class HwmonTempSensor : public Sensor
{
  public:
    std::string configuration;
    HwmonTempSensor(const std::string &path, const std::string &objectType,
                    sdbusplus::asio::object_server &objectServer,
                    std::shared_ptr<sdbusplus::asio::connection> &conn,
                    boost::asio::io_service &io, const std::string &fanName,
                    std::vector<thresholds::Threshold> &&thresholds,
                    const std::string &sensorConfiguration);
    ~HwmonTempSensor();

  private:
    std::string objectType;
    sdbusplus::asio::object_server &objServer;
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