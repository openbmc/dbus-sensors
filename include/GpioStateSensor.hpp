#pragma once

#include <boost/container/flat_map.hpp>
#include <sensor.hpp>

namespace oem
{
inline namespace twinlake_gpio
{
constexpr uint8_t netFn = 0x38;
} // namespace twinlake_gpio
} // namespace oem

static inline uint8_t lun = 0;
using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

struct ipmbGpioInfo {
uint8_t byteMask, bitMask;
};

static boost::container::flat_map<std::string, ipmbGpioInfo> twinlake_ipmbgpio_map = {{"CPU_Good",{3,1}}};

class IpmbGpioStateMonitor
{
  public:
    IpmbGpioStateMonitor(std::shared_ptr<sdbusplus::asio::connection>& conn,
             boost::asio::io_service& io, const float pollRate,
             sdbusplus::asio::object_server& objectServer, std::string& name,
             const uint8_t ipmbBusIndex, const uint8_t deviceAddress,
             const std::string& gpioTypeName, const std::string& gpioClass);
    ~IpmbGpioStateMonitor();

    bool init();
    void read();
    void setProperty(bool);
    void incrementError();

    int gpioPollMs;
    std::string gpioName;
    uint8_t ipmbBusIndex;
    uint8_t deviceAddress;
    uint8_t netfn;
    uint8_t commandAddress;
    std::vector<uint8_t> commandData = {0x15, 0xa0, 0};
    uint8_t retryCount = 0;
    const std::string gpioClass;
    bool gpioState;
    uint8_t gpioByteMask;
    uint8_t gpioBitMask;

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> gpioInterface;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    boost::asio::deadline_timer ioTimer;
    std::shared_ptr<Sensor> gpios;
};
