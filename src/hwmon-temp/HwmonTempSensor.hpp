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
#include <vector>

struct SensorParams
{
    double minValue;
    double maxValue;
    double offsetValue;
    double scaleValue;
    std::string units;
    std::string typeName;
};

class HwmonTempSensor :
    public Sensor,
    public std::enable_shared_from_this<HwmonTempSensor>
{
  public:
    HwmonTempSensor(const std::string& path, const std::string& objectType,
                    sdbusplus::asio::object_server& objectServer,
                    std::shared_ptr<sdbusplus::asio::connection>& conn,
                    boost::asio::io_context& io, const std::string& sensorName,
                    std::vector<thresholds::Threshold>&& thresholds,
                    const struct SensorParams& thisSensorParameters,
                    float pollRate, const std::string& sensorConfiguration,
                    PowerState powerState,
                    const std::shared_ptr<I2CDevice>& i2cDevice);
    ~HwmonTempSensor() override;
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
    // Ordering is important here; readBuf is first so that it's not destroyed
    // while async operations from other member fields might still be using it.
    std::array<char, 128> readBuf{};
    std::shared_ptr<I2CDevice> i2cDevice;
    sdbusplus::asio::object_server& objServer;
    boost::asio::random_access_file inputDev;
    boost::asio::steady_timer waitTimer;
    std::string path;
    double offsetValue;
    double scaleValue;
    unsigned int sensorPollMs;

    void handleResponse(const boost::system::error_code& err, size_t bytesRead);
    void restartRead();
    void checkThresholds() override;
};
