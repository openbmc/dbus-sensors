#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <sdbusplus/asio/object_server.hpp>

class HwmonTempSensor : public Sensor
{
  public:
    HwmonTempSensor(const std::string& path, const std::string& objectType,
                    sdbusplus::asio::object_server& objectServer,
                    std::shared_ptr<sdbusplus::asio::connection>& conn,
                    boost::asio::io_service& io, const std::string& fanName,
                    std::vector<thresholds::Threshold>&& thresholds,
                    const std::string& sensorConfiguration);
    ~HwmonTempSensor();

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    std::string path;
    int errCount;
    void setupRead(void);
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
};
