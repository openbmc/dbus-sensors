#pragma once

#include "Utils.hpp"

#include <Thresholds.hpp>
#include <boost/container/flat_map.hpp>
#include <filesystem>
#include <fstream>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

class CPUSensor : public Sensor
{
  public:
    CPUSensor(const std::string& path, const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_service& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const std::string& configuration, int cpuId, bool show,
              double dtsOffset);
    ~CPUSensor();
    static constexpr unsigned int sensorScaleFactor = 1000;
    static constexpr unsigned int sensorPollMs = 1000;
    static constexpr size_t warnAfterErrorCount = 10;
    static constexpr double maxReading = 127;
    static constexpr double minReading = -128;
    static constexpr const char* labelTcontrol = "Tcontrol";

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    std::string nameTcontrol;
    std::string path;
    double privTcontrol;
    double dtsOffset;
    bool show;
    int errCount;
    void setupRead(void);
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
};

extern boost::container::flat_map<std::string, std::unique_ptr<CPUSensor>>
    gCpuSensors;

// this is added to cpusensor.hpp to avoid having every sensor have to link
// against libgpiod, if another sensor needs it we may move it to utils
inline bool hostIsPresent(size_t gpioNum)
{
    static boost::container::flat_map<size_t, bool> cpuPresence;

    auto findIndex = cpuPresence.find(gpioNum);
    if (findIndex != cpuPresence.end())
    {
        return findIndex->second;
    }

    constexpr size_t sgpioBase = 232;

    // check if sysfs has device
    bool sysfs = std::filesystem::exists(gpioPath + std::string("gpio") +
                                         std::to_string(gpioNum));

    // todo: delete this when we remove all sysfs code
    if (sysfs)
    {
        // close it, we'll reopen it at the end
        std::ofstream unexport(gpioPath + std::string("unexport"));
        if (unexport.good())
        {
            unexport << gpioNum;
        }
        else
        {
            std::cerr << "Error cleaning up sysfs device\n";
        }
    }

    size_t chipNum = (gpioNum - sgpioBase) / 8;
    size_t index = (gpioNum - sgpioBase) % 8;
    gpiod::chip chip("gpiochip" + std::to_string(chipNum));
    auto line = chip.get_line(index);

    if (!line)
    {
        std::cerr << "Error requesting gpio\n";
        return true;
    }

    bool resp = true;
    try
    {
        line.request({"adcsensor", gpiod::line_request::DIRECTION_INPUT});
        resp = !line.get_value();
    }
    catch (std::system_error&)
    {
        std::cerr << "Error reading gpio\n";
        return true;
    }

    // todo: delete this when we remove all sysfs code
    if (sysfs)
    {
        // reopen it
        std::ofstream populate(gpioPath + std::string("export"));
        if (populate.good())
        {
            populate << gpioNum;
        }
        else
        {
            std::cerr << "Error cleaning up sysfs device\n";
        }
    }
    cpuPresence[gpioNum] = resp;
    return resp;
}
