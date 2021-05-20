#pragma once

#include <Thresholds.hpp>
#include <boost/asio/streambuf.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

#include <string>
#include <vector>

class IIOSensor : public Sensor, public std::enable_shared_from_this<IIOSensor>
{
  public:
    IIOSensor(const std::string& path, const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_service& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const float pollRate, const std::string& sensorConfiguration,
              const PowerState powerState, const std::string& sensorType);
    ~IIOSensor() override;
    void setupRead(void);

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    std::string path;
    size_t errCount;
    unsigned int sensorPollMs;

    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
};
