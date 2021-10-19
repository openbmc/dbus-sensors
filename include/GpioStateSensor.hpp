#pragma once

#include <sensor.hpp>

enum class IpmbGpioType
{
    ipmbGpio
};

namespace oem
{
namespace twinlake_gpio
{
constexpr uint8_t netFn = 0x38;
} // namespace twinlake_gpio
} // namespace oem

class IpmbGpio
{
  public:
    IpmbGpio(std::shared_ptr<sdbusplus::asio::connection>& conn,
             boost::asio::io_service& io, const float pollRate,
             sdbusplus::asio::object_server& objectServer, std::string& name,
             const uint8_t ipmbBusIndex, const uint8_t deviceAddress,
             const std::string& gpioTypeName, const std::string& gpioClass,
             const uint8_t pgoodGpioByte, const uint8_t pgoodGpioBit);
    ~IpmbGpio();

    bool init();
    bool loadDefaults();
    void read();

    int gpioPollMs;
    std::string gpioName;
    uint8_t ipmbBusIndex;
    uint8_t deviceAddress;
    uint8_t netfn;
    uint8_t commandAddress;
    std::vector<uint8_t> commandData;
    const std::string gpioClass;
    uint8_t pgoodGpioByte;
    uint8_t pgoodGpioBit;

    IpmbGpioType type;

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> gpioInterface;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    boost::asio::deadline_timer ioTimer;
    std::shared_ptr<Sensor> gpios;
};
