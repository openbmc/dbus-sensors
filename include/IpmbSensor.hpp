#pragma once
#include <boost/asio/deadline_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sensor.hpp>

#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

enum class IpmbType
{
    meSensor,
    PXE1410CVR,
    IR38363VR,
    ADM1278HSC,
    mpsVR
};

enum class IpmbSubType
{
    temp,
    curr,
    power,
    volt,
    util
};

enum class ReadingFormat
{
    byte0,
    byte3,
    elevenBit,
    elevenBitShift,
    linearElevenBit
};

namespace ipmi
{
namespace sensor
{
constexpr uint8_t netFn = 0x04;
constexpr uint8_t getSensorReading = 0x2d;

static bool isValid(const std::vector<uint8_t>& data)
{
    constexpr auto readingUnavailableBit = 5;

    // Proper 'Get Sensor Reading' response has at least 4 bytes, including
    // Completion Code. Our IPMB stack strips Completion Code from payload so we
    // compare here against the rest of payload
    if (data.size() < 3)
    {
        return false;
    }

    // Per IPMI 'Get Sensor Reading' specification
    if (data[1] & (1 << readingUnavailableBit))
    {
        return false;
    }

    return true;
}

} // namespace sensor
namespace me_bridge
{
constexpr uint8_t netFn = 0x2e;
constexpr uint8_t sendRawPmbus = 0xd9;
} // namespace me_bridge

/* This is OEM specified netFn and command for twinlake firmware version */
namespace twinlake_oem_version
{
constexpr uint8_t netFn = 0x38;
constexpr uint8_t command = 0x0b;
} // namespace twinlake_oem_version

} // namespace ipmi

struct IpmbSensor : public Sensor
{
    IpmbSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
               boost::asio::io_service& io, const std::string& name,
               const std::string& sensorConfiguration,
               sdbusplus::asio::object_server& objectServer,
               std::vector<thresholds::Threshold>&& thresholds,
               uint8_t deviceAddress, uint8_t hostSMbusIndex,
               const float pollRate, std::string& sensorTypeName);
    ~IpmbSensor() override;

    void checkThresholds(void) override;
    void read(void);
    void init(void);
    std::string getSubTypeUnits(void);
    void loadDefaults(void);
    void runInitCmd(void);
    bool processReading(const std::vector<uint8_t>& data, double& resp);

    IpmbType type;
    IpmbSubType subType;
    double scaleVal;
    double offsetVal;
    uint8_t commandAddress;
    uint8_t netfn;
    uint8_t command;
    uint8_t deviceAddress;
    uint8_t errorCount;
    uint8_t hostSMbusIndex;
    std::vector<uint8_t> commandData;
    std::optional<uint8_t> initCommand;
    std::vector<uint8_t> initData;
    int sensorPollMs;

    ReadingFormat readingFormat;

  private:
    sdbusplus::asio::object_server& objectServer;
    boost::asio::deadline_timer waitTimer;
};

/* This class will read firmware versions (CPLD, ME, BIC, VR) based
on OEM specified Netfunction and Command for each platform.

OEM specified netfn and command are declared under the namespace
"twinlake_oem_version".

This can be used to verify the version of CPLD, ME, BIC, VR once
firmware update is completed.

From EM configuration file, deviceAddress, bus and pollRate of each
version can be configured.
*/

class IpmbVersion
{
  public:
    IpmbVersion(std::shared_ptr<sdbusplus::asio::connection>& conn,
                boost::asio::io_service& io, const float pollRate,
                sdbusplus::asio::object_server& objectServer, std::string& name,
                uint8_t ipmbBusIndex, uint8_t deviceAddress,
                const std::string& versionTypeName);
    ~IpmbVersion();

    void loadValues();
    void versionRead();

    int versionPollMs;
    uint8_t ipmbBusIndex;
    uint8_t deviceAddress;
    uint8_t command;
    uint8_t netfn;
    uint8_t commandAddress;
    std::vector<uint8_t> commandData;

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> versionInterface;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    boost::asio::deadline_timer ioTimer;
};
