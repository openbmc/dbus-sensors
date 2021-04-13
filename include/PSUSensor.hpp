#pragma once

#include <PwmSensor.hpp>
#include <Thresholds.hpp>
#include <boost/asio/streambuf.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

#include <memory>
#include <string>
#include <utility>

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
              double min, const std::string& label, size_t tSize);
    ~PSUSensor() override;
    void setupRead(void);

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    std::shared_ptr<boost::asio::streambuf> readBuf;
    std::string path;
    std::string pathRatedMax;
    std::string pathRatedMin;
    unsigned int sensorFactor;
    uint8_t minMaxReadCounter;
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
    void updateMinMaxValues(void);

    int fd;
    static constexpr unsigned int sensorPollMs = 1000;
    static constexpr size_t warnAfterErrorCount = 10;
};

class PSUProperty
{
  public:
    PSUProperty(std::string name, double max, double min, unsigned int factor) :
        labelTypeName(std::move(name)), maxReading(max), minReading(min),
        sensorScaleFactor(factor)
    {}
    ~PSUProperty() = default;

    std::string labelTypeName;
    double maxReading;
    double minReading;
    unsigned int sensorScaleFactor;
};
