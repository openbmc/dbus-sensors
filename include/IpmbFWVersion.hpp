#pragma once

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

enum class IpmbVersionType
{
    twinLake
};

namespace oem
{
namespace twinlake_fw_version
{
// This is OEM specified namespace for twinlake firmware version
constexpr uint8_t netFn = 0x38;
constexpr uint8_t command = 0x0b;
} // namespace twinlake_fw_version

} // namespace oem

/* This class will read firmware versions (CPLD, ME, BIC, VR) based
on OEM specified Netfunction and Command for each platform.

OEM specified netfn and command are declared under the namespace
"twinlake_oem_version".

This can be used to verify the version of CPLD, ME, BIC, VR once
firmware update is completed.

From EM configuration file, deviceAddress, bus and pollRate of each
version can be configured.
*/

class IpmbFWVersion
{
  public:
    IpmbFWVersion(std::shared_ptr<sdbusplus::asio::connection>& conn,
                  boost::asio::io_service& io, const float pollRate,
                  sdbusplus::asio::object_server& objectServer,
                  std::string& name, uint8_t ipmbBusIndex,
                  uint8_t deviceAddress, const std::string& versionTypeName,
                  const std::string& versionClass);
    ~IpmbFWVersion();

    void init();
    void loadDefaults();
    void read();

    int versionPollMs;
    uint8_t ipmbBusIndex;
    uint8_t deviceAddress;
    uint8_t command;
    uint8_t netfn;
    uint8_t commandAddress;
    std::vector<uint8_t> commandData;
    std::string versionClass;

    IpmbVersionType type;

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> versionInterface;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    boost::asio::deadline_timer ioTimer;
    std::shared_ptr<Sensor> versions;
};
