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

enum class ByteMask
{
    BYTE_3 = 3,
    BYTE_4 = 4,
    BYTE_5 = 5,
    BYTE_6 = 6,
    BYTE_7 = 7,
    BYTE_8 = 8
};

enum class BitMask
{
    BIT_0 = 0,
    BIT_1,
    BIT_2,
    BIT_3,
    BIT_4,
    BIT_5,
    BIT_6,
    BIT_7
};

struct IpmbGpioInfo
{
    ByteMask byte;
    BitMask bit;
};

using gpioMapType = boost::container::flat_map<std::string, IpmbGpioInfo>;
static gpioMapType twinlakeIpmbgpioMap = {
    {"CPU_Good", {ByteMask::BYTE_3, BitMask::BIT_0}},
    {"PCH_Good", {ByteMask::BYTE_3, BitMask::BIT_1}}};

static inline uint8_t ipmbHostBit = 2;
static inline uint8_t lun = 0;
using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

class IpmbGpioStateMonitor
{
  public:
    IpmbGpioStateMonitor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                         boost::asio::io_service& io, const float pollRate,
                         sdbusplus::asio::object_server& objectServer,
                         std::string& name, const uint8_t ipmbBusIndex,
                         const uint8_t deviceAddress,
                         const std::string& gpioClass,
                         const std::vector<std::string>& ipmbGpioStates);
    ~IpmbGpioStateMonitor();

    void read();
    void incrementError();
    gpioMapType getGpioMap();

    int gpioPollMs;
    std::string gpioName;
    uint8_t ipmbBusIndex;
    uint8_t deviceAddress;
    std::vector<uint8_t> commandData = {0x15, 0xa0, 0};
    uint8_t commandAddress = ipmbBusIndex << ipmbHostBit;
    uint8_t netfn = oem::twinlake_gpio::netFn;
    uint8_t retryCount = 0;
    const std::string gpioClass;
    uint8_t gpioByteMask;
    uint8_t gpioBitMask;
    std::vector<std::string> ipmbGpioStates;

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> gpioInterface;
    std::map<std::string, std::shared_ptr<sdbusplus::asio::dbus_interface>>
        gpioInterfaceMap;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    boost::asio::deadline_timer ioTimer;
};
