#pragma once
#include <boost/asio/steady_timer.hpp>
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
    none,
    meSensor,
    PXE1410CVR,
    IR38363VR,
    ADM1278HSC,
    mpsVR
};

enum class IpmbSubType
{
    none,
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
    if ((data[1] & (1 << readingUnavailableBit)) != 0)
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
} // namespace ipmi

using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

struct IpmbSensor :
    public Sensor,
    public std::enable_shared_from_this<IpmbSensor>
{
    IpmbSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
               boost::asio::io_context& io, const std::string& name,
               const std::string& sensorConfiguration,
               sdbusplus::asio::object_server& objectServer,
               std::vector<thresholds::Threshold>&& thresholdData,
               uint8_t deviceAddress, uint8_t hostSMbusIndex, float pollRate,
               std::string& sensorTypeName);
    ~IpmbSensor() override;

    void checkThresholds(void) override;
    void read(void);
    void init(void);
    std::string getSubTypeUnits(void) const;
    void loadDefaults(void);
    void runInitCmd(void);
    bool processReading(const std::vector<uint8_t>& data, double& resp);
    void parseConfigValues(const SensorBaseConfigMap& entry);
    bool sensorClassType(const std::string& sensorClass);
    void sensorSubType(const std::string& sensorTypeName);

    IpmbType type = IpmbType::none;
    IpmbSubType subType = IpmbSubType::none;
    double scaleVal = 1.0;
    double offsetVal = 0.0;
    uint8_t commandAddress = 0;
    uint8_t netfn = 0;
    uint8_t command = 0;
    uint8_t deviceAddress = 0;
    uint8_t errorCount = 0;
    uint8_t hostSMbusIndex = 0;
    std::vector<uint8_t> commandData;
    std::optional<uint8_t> initCommand;
    std::vector<uint8_t> initData;
    int sensorPollMs;

    ReadingFormat readingFormat = ReadingFormat::byte0;

  private:
    void sendIpmbRequest(void);
    sdbusplus::asio::object_server& objectServer;
    boost::asio::steady_timer waitTimer;
    void ipmbRequestCompletionCb(const boost::system::error_code& ec,
                                 const IpmbMethodType& response);
};
