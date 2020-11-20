#pragma once

#include "PwmSensor.hpp"
#include "Thresholds.hpp"
#include "sensor.hpp"

#include <boost/asio/streambuf.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <string>

class PSUSensor : public Sensor, public std::enable_shared_from_this<PSUSensor>
{
  public:
    PSUSensor(const std::string& path, const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_service& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const std::string& sensorConfiguration,
              std::string& sensorTypeName, unsigned int factor, double max,
              double min, double gain, double offset, unsigned int interval,
              const std::string& label, size_t tSize);
    ~PSUSensor();
    void setupRead(void);

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    std::shared_ptr<boost::asio::streambuf> readBuf;
    std::string path;
    std::string pathRatedMin;
    std::string pathRatedMax;
    size_t errCount;
    unsigned int sensorFactor;
    double sensorGain;
    double sensorOffset;
    uint8_t minMaxReadCounter;
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
    void updateMinMaxValues(void);

    int fd;
    unsigned int sensorPollMs;
    static constexpr size_t warnAfterErrorCount = 10;
};

class PSUProperty
{
  public:
    PSUProperty(std::string name, double max, double min, unsigned int factor,
                double gain, double offset) :
        labelTypeName(name),
        maxReading(max), minReading(min), sensorScaleFactor(factor),
        sensorGainFactor(gain), sensorOffsetFactor(offset)
    {}
    ~PSUProperty() = default;

    std::string labelTypeName;
    double maxReading;
    double minReading;
    unsigned int sensorScaleFactor;
    double sensorGainFactor;
    double sensorOffsetFactor;
};
