#pragma once

#include <boost/container/flat_map.hpp>
#include <sensor.hpp>

#include <array>

namespace oem
{
inline namespace twinlake
{
constexpr uint8_t netFn = 0x38;
} // namespace twinlake
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

constexpr std::uint8_t bit0{0x01}; // hex for 0000 0001
constexpr std::uint8_t bit1{0x02}; // hex for 0000 0010
constexpr std::uint8_t bit2{0x04}; // hex for 0000 0100
constexpr std::uint8_t bit3{0x08}; // hex for 0000 1000
constexpr std::uint8_t bit4{0x10}; // hex for 0001 0000
constexpr std::uint8_t bit5{0x20}; // hex for 0010 0000
constexpr std::uint8_t bit6{0x40}; // hex for 0100 0000
constexpr std::uint8_t bit7{0x80}; // hex for 1000 0000

struct IpmbStateInfo
{
    std::string IpmbState;
    ByteMask byte;
    std::uint8_t bit;
};

static constexpr uint8_t ipmbHostBit = 2;
static constexpr uint8_t lun = 0;
static constexpr uint8_t ipmbRspLength = 9;
using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

class IpmbPowerMonitor : public std::enable_shared_from_this<IpmbPowerMonitor>
{
  public:
    IpmbPowerMonitor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_service& io, float pollRate,
                     sdbusplus::asio::object_server& objectServer,
                     std::string& name, uint8_t ipmbBusIndex,
                     uint8_t deviceAddress, const std::string& ipmbHostClass);
    ~IpmbPowerMonitor();

    void read();
    void incrementError();
    void setIpmbPowerState(std::vector<uint8_t> ipmbResp);

    int ipmbPollMs;
    std::string hostName;
    uint8_t ipmbBusIndex;
    uint8_t deviceAddress;
    const std::vector<uint8_t> commandData = {0x15, 0xa0, 0};
    uint8_t commandAddress = ipmbBusIndex << ipmbHostBit;
    uint8_t netfn = oem::twinlake::netFn;
    uint8_t retryCount = 0;
    uint8_t retryTimes = 3;
    const std::string ipmbHostClass;
    IpmbStateInfo prop = {"CPU_Good", ByteMask::BYTE_3, bit0};

  private:
    std::map<std::string, std::shared_ptr<sdbusplus::asio::dbus_interface>>
        ipmbPowerInterfaceMap;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    boost::asio::steady_timer ioTimer;
};
