#pragma once

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

enum class IpmbGpioType
{
    gpio
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
             uint8_t ipmbBusIndex, uint8_t deviceAddress,
             const std::string& gpioTypeName, const std::string& gpioClass);
    ~IpmbGpio();

    void init();
    void loadDefaults();
    void read();

    int gpioPollMs;
    uint8_t ipmbBusIndex;
    uint8_t deviceAddress;
    uint8_t command;
    uint8_t netfn;
    uint8_t commandAddress;
    std::vector<uint8_t> commandData;
    const std::string gpioClass;
    uint8_t registerData;
    uint8_t registerPin;

    IpmbGpioType type;

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> gpioInterface;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    boost::asio::deadline_timer ioTimer;
};
