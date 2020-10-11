#pragma once

#include <Thresholds.hpp>
#include <boost/asio/streambuf.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

#include <string>
#include <vector>

class HwmonTempSensor : public std::enable_shared_from_this<HwmonTempSensor>
{
  public:
    HwmonTempSensor(const std::string& path, const std::string& objectType,
                    sdbusplus::asio::object_server& objectServer,
                    std::shared_ptr<sdbusplus::asio::connection>& conn,
                    boost::asio::io_service& io, const std::string& fanName,
                    std::vector<thresholds::Threshold>&& thresholds,
                    const std::string& sensorConfiguration,
                    const PowerState powerState);
    ~HwmonTempSensor();
    void setupRead(void);
    Sensor sensor;

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    std::string path;
    size_t errCount;

    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void);
};
