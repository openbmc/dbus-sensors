#pragma once

#include <Thresholds.hpp>
#include <optional>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

class ADCSensor : public Sensor
{
  public:
    ADCSensor(const std::string& path,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_service& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const double scaleFactor, PowerState readState,
              const std::string& sensorConfiguration,
              std::optional<int> bridgeGpio);
    ~ADCSensor();

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    int errCount;
    double scaleFactor;
    std::optional<int> bridgeGpio;
    PowerState readState;
    thresholds::ThresholdTimer thresholdTimer;
    void setupRead(void);
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
};
