#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <boost/asio/streambuf.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

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
                line.request({"adcsensor",
                              gpiod::line_request::DIRECTION_OUTPUT,
                              polarity == gpiod::line::ACTIVE_HIGH
                                  ? 0
                                  : gpiod::line_request::FLAG_ACTIVE_LOW});
            }
            catch (const std::system_error&)
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
            catch (const std::system_error& exc)
            {
                std::cerr << "Error set_value: " << exc.what() << "\n";
            }
        }
    }

    unsigned int setupTimeMs;

  private:
    gpiod::line line;
};

class ADCSensor : public Sensor, public std::enable_shared_from_this<ADCSensor>
{
  public:
    ADCSensor(const std::string& path,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_context& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              double scaleFactor, float pollRate, PowerState readState,
              const std::string& sensorConfiguration,
              std::optional<BridgeGpio>&& bridgeGpio);
    ~ADCSensor() override;
    void setupRead(void);

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::steady_timer waitTimer;
    std::shared_ptr<boost::asio::streambuf> readBuf;
    std::string path;
    double scaleFactor;
    unsigned int sensorPollMs;
    std::optional<BridgeGpio> bridgeGpio;
    thresholds::ThresholdTimer thresholdTimer;
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
};
