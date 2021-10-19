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

struct IpmbStateInfo
{
    std::string IpmbState;
    ByteMask byte;
    BitMask bit;
};
static std::array<IpmbStateInfo, 2> twinlakeIpmbStateProp = {
    {{"CPU_Good", ByteMask::BYTE_3, BitMask::BIT_0},
     {"PCH_Good", ByteMask::BYTE_3, BitMask::BIT_1}}};

static constexpr uint8_t ipmbHostBit = 2;
static constexpr uint8_t lun = 0;
using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

class IpmbPowerMonitor : public std::enable_shared_from_this<IpmbPowerMonitor>
{
  public:
    IpmbPowerMonitor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_service& io, float pollRate,
                     sdbusplus::asio::object_server& objectServer,
                     std::string& name, uint8_t ipmbBusIndex,
                     uint8_t deviceAddress, const std::string& ipmbHostClass,
                     const std::vector<std::string>& ipmbStateNames);
    ~IpmbPowerMonitor();

    void read();
    void incrementError();
    static IpmbStateInfo getHostCtrlProp(const std::string propName);
    void setIpmbPowerState(std::vector<uint8_t> ipmbResp);

    int ipmbPollMs;
    std::string hostName;
    uint8_t ipmbBusIndex;
    uint8_t deviceAddress;
    std::vector<uint8_t> commandData = {0x15, 0xa0, 0};
    uint8_t commandAddress = ipmbBusIndex << ipmbHostBit;
    uint8_t netfn = oem::twinlake::netFn;
    uint8_t retryCount = 0;
    const std::string ipmbHostClass;
    std::vector<std::string> ipmbStateNames;

  private:
    std::map<std::string, std::shared_ptr<sdbusplus::asio::dbus_interface>>
        ipmbPowerInterfaceMap;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    boost::asio::deadline_timer ioTimer;
};
