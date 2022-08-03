#pragma once

#include <Thresholds.hpp>
#include <Utils.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/container/flat_map.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

class IntelCPUSensor :
    public Sensor,
    public std::enable_shared_from_this<IntelCPUSensor>
{
  public:
    IntelCPUSensor(const std::string& path, const std::string& objectType,
                   sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& conn,
                   boost::asio::io_service& io, const std::string& sensorName,
                   std::vector<thresholds::Threshold>&& thresholds,
                   const std::string& configuration, int cpuId, bool show,
                   double dtsOffset);
    ~IntelCPUSensor() override;
    static constexpr unsigned int sensorScaleFactor = 1000;
    static constexpr unsigned int sensorPollMs = 1000;
    static constexpr size_t warnAfterErrorCount = 10;
    static constexpr const char* labelTcontrol = "Tcontrol";
    void setupRead(void);

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::streambuf readBuf;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    std::string nameTcontrol;
    std::string path;
    double privTcontrol;
    double dtsOffset;
    bool show;
    size_t pollTime;
    bool loggedInterfaceDown = false;
    uint8_t minMaxReadCounter{0};
    int fd{};
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
    void updateMinMaxValues(void);
    void restartRead(void);
};

extern boost::container::flat_map<std::string, std::shared_ptr<IntelCPUSensor>>
    gCpuSensors;

// this is added to intelcpusensor.hpp to avoid having every sensor have to link
// against libgpiod, if another sensor needs it we may move it to utils
inline bool cpuIsPresent(const SensorBaseConfigMap& gpioConfig)
{
    static boost::container::flat_map<std::string, bool> cpuPresence;

    auto findName = gpioConfig.find("Name");
    if (findName == gpioConfig.end())
    {
        return false;
    }
    std::string gpioName =
        std::visit(VariantToStringVisitor(), findName->second);

    auto findIndex = cpuPresence.find(gpioName);
    if (findIndex != cpuPresence.end())
    {
        return findIndex->second;
    }

    bool activeHigh = true;
    auto findPolarity = gpioConfig.find("Polarity");
    if (findPolarity != gpioConfig.end())
    {
        if (std::string("Low") ==
            std::visit(VariantToStringVisitor(), findPolarity->second))
        {
            activeHigh = false;
        }
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
        line.request({"cpusensor", gpiod::line_request::DIRECTION_INPUT,
                      activeHigh ? 0 : gpiod::line_request::FLAG_ACTIVE_LOW});
        resp = (line.get_value() != 0);
    }
    catch (const std::system_error&)
    {
        std::cerr << "Error reading gpio: " << gpioName << "\n";
        return false;
    }

    cpuPresence[gpioName] = resp;

    return resp;
}
