#pragma once

#include "asio/Thresholds.hpp"
#include "asio/sensor.hpp"
#include "utils/Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/streambuf.hpp>
#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <optional>
#include <string>
#include <system_error>
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
            lg2::error("Error finding gpio: '{NAME}'", "NAME", name);
        }
        else
        {
            try
            {
                line.request(
                    {"adcsensor", gpiod::line_request::DIRECTION_OUTPUT,
                     polarity == gpiod::line::ACTIVE_HIGH
                         ? 0
                         : gpiod::line_request::FLAG_ACTIVE_LOW});
            }
            catch (const std::system_error&)
            {
                lg2::error("Error requesting gpio: '{NAME}'", "NAME", name);
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
                lg2::error("Error set_value: '{EC}'", "EC", exc);
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
    void setupRead();

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
    void checkThresholds() override;
    void restartRead();
};
