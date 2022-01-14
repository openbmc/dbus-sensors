#pragma once

#include <PwmSensor.hpp>
#include <Thresholds.hpp>
#include <gpiod.hpp>
#include <boost/asio/streambuf.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

#include <memory>
#include <string>
#include <optional>
#include <utility>

class BridgeGpio
{
  public:
    BridgeGpio(const std::string& name, const int polarity,
               const float setupTime) :
         setupTimeMs(static_cast<unsigned int>(setupTime * 1000))
    {
        line = gpiod::find_line(name);
        if (!line)
        {
            std::cerr << "Error finding gpio: " << name << "\n";
        }
        else
        {
            try
            {
                line.request({"psusensor",
                              gpiod::line_request::DIRECTION_OUTPUT,
                              polarity == gpiod::line::ACTIVE_HIGH
                                  ? 0
                                  : gpiod::line_request::FLAG_ACTIVE_LOW});
            }
            catch (std::system_error&)
            {
                std::cerr << "Error requesting gpio: " << name << "\n";
            }
        }
    }

    void set(int value)
    {
        if (line)
        {
            try
            {
                line.set_value(value);
            }
            catch (std::system_error& exc)
            {
                std::cerr << "Error set_value: " << exc.what() << value << "\n";
            }
        }
    }

    unsigned int setupTimeMs;

  private:
    gpiod::line line;
};


class PSUSensor : public Sensor, public std::enable_shared_from_this<PSUSensor>
{
  public:
    PSUSensor(const std::string& path, const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_service& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const std::string& sensorConfiguration,
              const PowerState& powerState, const std::string& sensorUnits,
              unsigned int factor, double max, double min, double offset,
              const std::string& label, size_t tSize, double pollRate,
              std::optional<BridgeGpio>&& bridgeGpio);
    ~PSUSensor() override;
    void setupRead(void);

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    std::string path;
    unsigned int sensorFactor;
    double sensorOffset;
    thresholds::ThresholdTimer thresholdTimer;
    void restartRead();
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
    unsigned int sensorPollMs = defaultSensorPollMs;
    std::optional<BridgeGpio> bridgeGpio;

    int fd;
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
        labelTypeName(std::move(name)),
        maxReading(max), minReading(min), sensorScaleFactor(factor),
        sensorOffset(offset)
    {}
    ~PSUProperty() = default;

    std::string labelTypeName;
    double maxReading;
    double minReading;
    unsigned int sensorScaleFactor;
    double sensorOffset;
};
