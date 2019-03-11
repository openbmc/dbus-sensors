#pragma once

#include <Thresholds.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

enum class SensorType
{
    tempSensor,
    currSensor,
    powerSensor,
    voltSensor
};

class PSUSensor : public Sensor
{
  public:
    PSUSensor(const std::string& path, const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_service& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const std::string& sensorConfiguration,
              std::string sensorTypeName, unsigned int factor, double max,
              double min);
    ~PSUSensor();

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    int errCount;
    unsigned int sensorFactor;
    void setupRead(void);
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
};

class PSUProperty
{
  public:
    PSUProperty(std::string name, double max, double min, unsigned int factor) :
        sensorTypeName(name), maxReading(max), minReading(min),
        sensorScaleFactor(factor)
    {
    }
    ~PSUProperty() = default;

    std::string sensorTypeName;
    double maxReading;
    double minReading;
    unsigned int sensorScaleFactor;
};
