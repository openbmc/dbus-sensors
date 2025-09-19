#pragma once

#include "asio/DeviceMgmt.hpp"
#include "asio/Thresholds.hpp"
#include "asio/sensor.hpp"
#include "utils/Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/random_access_file.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class PSUSensor : public Sensor, public std::enable_shared_from_this<PSUSensor>
{
  public:
    PSUSensor(const std::string& path, const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_context& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const std::string& sensorConfiguration,
              const PowerState& powerState, const std::string& sensorUnits,
              unsigned int factor, double max, double min, double offset,
              const std::string& label, size_t tSize, double pollRate,
              const std::shared_ptr<I2CDevice>& i2cDevice);
    ~PSUSensor() override;
    void setupRead();
    void activate(const std::string& newPath,
                  const std::shared_ptr<I2CDevice>& newI2CDevice);
    void deactivate();
    bool isActive();

    std::shared_ptr<I2CDevice> getI2CDevice() const
    {
        return i2cDevice;
    }

  private:
    // Note, this buffer is a shared_ptr because during a read, its lifetime
    // might have to outlive the PSUSensor class if the object gets destroyed
    // while in the middle of a read operation
    std::shared_ptr<std::array<char, 128>> buffer;
    std::shared_ptr<I2CDevice> i2cDevice;
    sdbusplus::asio::object_server& objServer;
    boost::asio::random_access_file inputDev;
    boost::asio::steady_timer waitTimer;
    std::string path;
    unsigned int sensorFactor;
    double sensorOffset;
    thresholds::ThresholdTimer thresholdTimer;
    void restartRead();
    void handleResponse(const boost::system::error_code& err, size_t bytesRead);
    void checkThresholds() override;
    unsigned int sensorPollMs = defaultSensorPollMs;

    static constexpr size_t warnAfterErrorCount = 10;

  public:
    static constexpr double defaultSensorPoll = 1.0;
    static constexpr unsigned int defaultSensorPollMs =
        static_cast<unsigned int>(defaultSensorPoll * 1000);
};

class PSUProperty
{
  public:
    PSUProperty(std::string name, double max, double min, unsigned int factor,
                double offset) :
        labelTypeName(std::move(name)), maxReading(max), minReading(min),
        sensorScaleFactor(factor), sensorOffset(offset)
    {}
    ~PSUProperty() = default;

    std::string labelTypeName;
    double maxReading;
    double minReading;
    unsigned int sensorScaleFactor;
    double sensorOffset;
};
