#pragma once

#include "PwmSensor.hpp"
#include "Thresholds.hpp"
#include "sensor.hpp"

#include <sdbusplus/asio/object_server.hpp>

class PSUSensor : public Sensor
{
  public:
    PSUSensor(const std::string& path, const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_service& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const std::string& sensorConfiguration,
              std::string& sensorTypeName, unsigned int factor, double max,
              double min);
    ~PSUSensor();

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    std::string path;
    int errCount;
    unsigned int sensorFactor;
    void setupRead(void);
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;

    static constexpr unsigned int sensorPollMs = 500;
    static constexpr size_t warnAfterErrorCount = 10;
};

class PSUProperty
{
  public:
    PSUProperty(std::string name, double max, double min, unsigned int factor) :
        labelTypeName(name), maxReading(max), minReading(min),
        sensorScaleFactor(factor)
    {
    }
    ~PSUProperty() = default;

    std::string labelTypeName;
    double maxReading;
    double minReading;
    unsigned int sensorScaleFactor;
};
