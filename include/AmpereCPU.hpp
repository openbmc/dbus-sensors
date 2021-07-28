#pragma once

#include <PwmSensor.hpp>
#include <Thresholds.hpp>
#include <boost/asio/streambuf.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

#include <memory>
#include <string>
#include <utility>

class CPUSensor : public Sensor, public std::enable_shared_from_this<CPUSensor>
{
  public:
    CPUSensor(const std::string& path, const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_service& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const std::string& sensorConfiguration,
              std::string& sensorTypeName, unsigned int factor, double max,
              double min, bool addAssociation, const std::string& label,
              size_t tSize, PowerState readState);
    ~CPUSensor() override;
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

class SoCProperty
{
  public:
    SoCProperty(std::string name, double max, double min, unsigned int factor) :
        labelTypeName(std::move(name)), maxReading(max), minReading(min),
        sensorScaleFactor(factor)
    {}
    ~SoCProperty() = default;

    std::string labelTypeName;
    double maxReading;
    double minReading;
    bool addAssociation;
    unsigned int sensorScaleFactor;
};

// this is added to socsensor.hpp to avoid having every sensor have to link
// against libgpiod, if another sensor needs it we may move it to utils
inline bool cpuIsPresent(const SensorData* sensorData)
{
    std::string gpioName = "";
    bool activeHigh = true;
    bool matchedPolarity = false;
    bool matchedPresenceGpio = false;
    static boost::container::flat_map<std::string, bool> cpuPresence;

    for (const SensorBaseConfiguration& suppConfig : *sensorData)
    {
        if (suppConfig.first.find("PresenceGpio") != std::string::npos)
        {
            auto findName = suppConfig.second.find("Name");
            if (findName != suppConfig.second.end())
            {
                matchedPresenceGpio = true;
                gpioName =
                    std::visit(VariantToStringVisitor(), findName->second);
                auto findPolarity = suppConfig.second.find("Polarity");
                if (findPolarity != suppConfig.second.end())
                {
                    matchedPolarity = true;
                    if (std::string("Low") ==
                        std::visit(VariantToStringVisitor(),
                                   findPolarity->second))
                    {
                        activeHigh = false;
                    }
                }
            }
            break;
        }
    }
    /* Set CPU present to true for soc don't have PresenceGpio setting */
    if (!matchedPresenceGpio)
    {
        std::cerr << "No PresenceGpio setting." << std::endl;
        return true;
    }

    /* Set CPU present to false when there is no Gpio name setting */
    if (gpioName.empty())
    {
        std::cerr << "No PresenceGpio Name setting." << std::endl;
        return false;
    }

    /* Set CPU present to false when there is no Polarity setting */
    if (!matchedPolarity)
    {
        std::cerr << "No PresenceGpio Polarity setting." << std::endl;
        return false;
    }

    auto line = gpiod::find_line(gpioName);
    if (!line)
    {
        std::cerr << "Error requesting gpio: " << gpioName << "\n";
        return false;
    }

    bool resp;
    try
    {
        line.request({"socsensor", gpiod::line_request::DIRECTION_INPUT,
                      activeHigh ? 0 : gpiod::line_request::FLAG_ACTIVE_LOW});
        resp = line.get_value();
    }
    catch (std::system_error&)
    {
        std::cerr << "Error reading gpio: " << gpioName << "\n";
        return false;
    }
    cpuPresence[gpioName] = resp;

    return resp;
}