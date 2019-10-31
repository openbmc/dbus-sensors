#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <gpiod.hpp>
#include <optional>
#include <sdbusplus/asio/object_server.hpp>

class BridgeGpio
{
  public:
    BridgeGpio(const std::string& name, const int polarity)
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
            line.set_value(value);
        }
    }

  private:
    gpiod::line line;
};

class ADCSensor : public Sensor
{
  public:
    ADCSensor(const std::string& path,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_service& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const double scaleFactor, PowerState readState,
              const std::string& sensorConfiguration,
              std::optional<BridgeGpio>&& bridgeGpio);
    ~ADCSensor();

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    std::string path;
    int errCount;
    double scaleFactor;
    std::optional<BridgeGpio> bridgeGpio;
    PowerState readState;
    thresholds::ThresholdTimer thresholdTimer;
    void setupRead(void);
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
};
