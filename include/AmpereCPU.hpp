#pragma once

#include <Thresholds.hpp>
#include <boost/asio/random_access_file.hpp>
#include <gpiod.hpp>
#include <sensor.hpp>

#include <array>
#include <memory>
#include <string>
#include <utility>

class AmpereCPUSensor :
    public Sensor,
    public std::enable_shared_from_this<AmpereCPUSensor>
{
  public:
    AmpereCPUSensor(const std::string& path, const std::string& objectType,
                    sdbusplus::asio::object_server& objectServer,
                    std::shared_ptr<sdbusplus::asio::connection>& conn,
                    boost::asio::io_service& io, const std::string& sensorName,
                    std::vector<thresholds::Threshold>&& thresholds,
                    const std::string& sensorConfiguration,
                    std::string& sensorTypeName, double factor, double max,
                    double min, const std::string& label, size_t tSize,
                    PowerState readState);
    ~AmpereCPUSensor() override;
    void setupRead(void);

  private:
    static constexpr uint32_t bufferLength = 128;
    sdbusplus::asio::object_server& objServer;
    boost::asio::random_access_file inputDev;
    boost::asio::steady_timer waitTimer;
    std::array<char, bufferLength> readBuf{};
    std::string path;
    double sensorFactor;
    void restartRead();
    void handleResponse(const boost::system::error_code& err, size_t bytesRead);
    void checkThresholds(void) override;

    int fd{};
    static constexpr unsigned int sensorPollMs = 1000;
    static constexpr size_t warnAfterErrorCount = 10;
};

class AmpereCPUProperty
{
  public:
    AmpereCPUProperty(std::string name, double max, double min, double factor) :
        labelTypeName(std::move(name)), maxReading(max), minReading(min),
        sensorScaleFactor(factor)
    {}
    ~AmpereCPUProperty() = default;

    std::string labelTypeName;
    double maxReading;
    double minReading;
    double sensorScaleFactor;
};

// this is added to AmpereCPU.hpp to avoid having every sensor have to link
// against libgpiod, if another sensor needs it we may move it to utils
inline bool cpuPresence(const SensorData& sensorData)
{
    std::string gpioName;
    bool activeHigh = true;
    bool matchedPresenceGpio = false;

    for (const SensorBaseConfiguration& suppConfig : sensorData)
    {
        if (suppConfig.first.find("PresenceGpio") == std::string::npos)
        {
            continue;
        }
        auto findName = suppConfig.second.find("Name");
        if (findName == suppConfig.second.end())
        {
            std::cerr << "No name defined\n";
            return false;
        }

        gpioName = std::visit(VariantToStringVisitor(), findName->second);
        if (gpioName.empty())
        {
            std::cerr << "No PresenceGpio Name setting.\n";
            return false;
        }

        auto findPolarity = suppConfig.second.find("Polarity");
        if (findPolarity == suppConfig.second.end())
        {
            std::cerr << "No PresenceGpio Polarity setting.\n";
            return false;
        }

        if (std::string("Low") ==
            std::visit(VariantToStringVisitor(), findPolarity->second))
        {
            activeHigh = false;
        }
        matchedPresenceGpio = true;
        break;
    }
    /* Set CPU presence to true for soc don't have PresenceGpio setting */
    if (!matchedPresenceGpio)
    {
        return true;
    }

    auto line = gpiod::find_line(gpioName);
    if (!line)
    {
        std::cerr << "Error requesting gpio: " << gpioName << "\n";
        return false;
    }

    bool resp = false;
    try
    {
        line.request({"socsensor", gpiod::line_request::DIRECTION_INPUT,
                      activeHigh ? 0 : gpiod::line_request::FLAG_ACTIVE_LOW});
        resp = (line.get_value() != 0);
    }
    catch (std::system_error&)
    {
        std::cerr << "Error reading gpio: " << gpioName << "\n";
        return false;
    }

    return resp;
}