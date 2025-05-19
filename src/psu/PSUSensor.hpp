#pragma once

#include "DeviceMgmt.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/random_access_file.hpp>
#include <boost/asio/steady_timer.hpp>
#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <bitset>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

class BridgeGpio
{
  public:
    BridgeGpio(const std::string& name, int polarity, float setupTime) :
        name(name), polarity(polarity),
        setupTimeMs(static_cast<unsigned int>(setupTime * 1000)),
        requestFlags(polarity == gpiod::line::ACTIVE_HIGH
                         ? std::bitset<32>(0)
                         : gpiod::line_request::FLAG_ACTIVE_LOW)
    {
        line = gpiod::find_line(name);
        if (!line)
        {
            lg2::error("Failed to find GPIO '{NAME}'", "NAME", name);
            return;
        }

        if (line.is_used())
        {
            lg2::error("GPIO line '{NAME}' is already in use, skipping request",
                       "NAME", name);
            return;
        }

        try
        {
            line.request({"psusensor", gpiod::line_request::DIRECTION_OUTPUT,
                          requestFlags});
        }
        catch (const std::system_error& e)
        {
            lg2::error("Failed to request GPIO '{NAME}': {REASON}", "NAME",
                       name, "REASON", e.what());
        }
    }

    void set(int value)
    {
        if (line)
        {
            try
            {
                if (!line.is_requested())
                {
                    line.request(
                        {"psusensor", gpiod::line_request::DIRECTION_OUTPUT,
                         requestFlags});
                }

                line.set_value(value);
            }
            catch (const std::system_error& exc)
            {
                lg2::error("Failed to set GPIO '{NAME}' to {VALUE}: {EC}",
                           "NAME", name, "VALUE", value, "EC", exc.what());
            }
        }
    }

    std::string name;
    int polarity;
    unsigned int setupTimeMs;
    std::bitset<32> requestFlags;

  private:
    gpiod::line line;
};

class PSUSensor : public Sensor, public std::enable_shared_from_this<PSUSensor>
{
  public:
    PSUSensor(const std::string& path, const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_context& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const std::string& sensorConfiguration,
              const PowerState& powerState, std::string_view sensorUnits,
              double factor, double max, double min, double offset,
              const std::string& label, size_t tSize, double pollRate,
              const std::shared_ptr<I2CDevice>& i2cDevice,
              std::optional<BridgeGpio>&& bridgeGpio = std::nullopt);
    ~PSUSensor() override;
    PSUSensor(const PSUSensor&) = delete;
    PSUSensor(PSUSensor&&) = delete;
    PSUSensor& operator=(PSUSensor&&) = delete;
    PSUSensor& operator=(const PSUSensor&) = delete;
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
    // Note, this buffer must be declared before any objects that might do async
    // operations to ensure that those operations are complete before buffer is
    // destructed.
    std::array<char, 128> buffer{};
    std::shared_ptr<I2CDevice> i2cDevice;
    std::optional<BridgeGpio> bridgeGpio;
    sdbusplus::asio::object_server& objServer;
    boost::asio::random_access_file inputDev;
    boost::asio::steady_timer waitTimer;
    std::string path;
    double sensorFactor;
    double sensorOffset;
    thresholds::ThresholdTimer thresholdTimer;
    void restartRead();
    static void handleResponseStatic(const std::weak_ptr<PSUSensor>& weak,
                                     const boost::system::error_code& ec,
                                     size_t bytesRead);
    void handleResponse(const boost::system::error_code& err, size_t bytesRead);
    void checkThresholds() override;
    unsigned int sensorPollMs = defaultSensorPollMs;

    static constexpr size_t warnAfterErrorCount = 10;

  public:
    static constexpr double defaultSensorPoll = 1.0;
    static constexpr unsigned int defaultSensorPollMs =
        static_cast<unsigned int>(defaultSensorPoll * 1000);
};

struct PSUProperty
{
    std::string_view hwmonLabelName;
    std::string_view labelTypeName;
    double maxReading;
    double minReading;
    double sensorScaleFactor;
    double sensorOffset;
};
