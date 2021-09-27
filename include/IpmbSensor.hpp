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
constexpr uint8_t manufacturerId[3] = {0x57, 0x01, 0x00};

namespace read_me
{
/**
 * Refernce:
 * Intelligent Power Node Manager External Interface Specification
 * getPmbusReadings = Get PMBUS Readings (F5h)
 *
 * bytesForTimestamp and bytesForManufacturerId are decoded from
 * response bytes for Get PMBUS Readings.
 */
constexpr uint8_t getPmbusReadings = 0xF5;
constexpr uint8_t bytesForTimestamp = 4;
constexpr uint8_t bytesForManufacturerId = 3;

constexpr size_t fixedOffset = bytesForTimestamp + bytesForManufacturerId;

void getRawData(uint8_t registerToRead, const std::vector<uint8_t>& input,
                std::vector<uint8_t>& result)
{
    if (input.size() < 3)
    {
        return;
    }

    /* Every register is two bytes*/
    size_t offset = fixedOffset + (registerToRead * 2);
    if (input.size() <= (offset + 1))
    {
        return;
    }

    result.reserve(5);

    // ID
    result.emplace_back(input[0]);
    result.emplace_back(input[1]);
    result.emplace_back(input[2]);

    // Value in registerToRead
    result.emplace_back(input[offset]);
    result.emplace_back(input[offset + 1]);
}
} // namespace read_me

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
    void parseConfigValues(const SensorBaseConfigMap& entry);
    bool sensorClassType(const std::string& sensorClass);
    void sensorSubType(const std::string& sensorTypeName);
    void setReadMethod(const SensorBaseConfigMap& sensorBaseConfig);

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
    uint8_t registerToRead = 0;
    bool isReadMe = false;
    uint8_t deviceIndex = 0;
    std::vector<uint8_t> commandData;
    std::optional<uint8_t> initCommand;
    std::vector<uint8_t> initData;
    int sensorPollMs;
    ReadingFormat readingFormat = ReadingFormat::byte0;

  private:
    sdbusplus::asio::object_server& objectServer;
    boost::asio::deadline_timer waitTimer;

    void getMeCommand();
};
