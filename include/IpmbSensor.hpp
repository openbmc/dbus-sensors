#pragma once
<<<<<<< PATCH SET (15b90d Code clean up.)
#include <Utils.hpp>
#include <boost/asio/deadline_timer.hpp>
=======
#include <boost/asio/steady_timer.hpp>
>>>>>>> BASE      (ead7e9 pwmsensor: Correct the type of MaxValue/MinValue)
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
<<<<<<< PATCH SET (15b90d Code clean up.)
constexpr uint8_t manufacturerId[3] = {0x57, 0x01, 0x00};

/*
 * From PMBus Power System Management Protocol Specification Part II
 * Command name to code mapping
 */
enum CommandName : uint8_t
{
    readTemperature = 0x8D,
    readCurrentOutput = 0x8C
};

/*
 * From Intelligent Power Node Manager : External Interface Specification Using
 * IPMI SMBUS message transaction type
 */
enum SmbusMessageType : uint8_t
{
    sendByte = 0x00,
    readByte = 0x01,
    writeByte = 0x02,
    readWord = 0x03,
    writeWord = 0x04,
    blockRead = 0x05,
    blockWrite = 0x06,
    blockWriteReadProcCall = 0x07
};

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
    /* Every register is two bytes*/
    size_t offset = fixedOffset + (registerToRead * 2);

    if (input.size() < (offset + 1))
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
=======
>>>>>>> BASE      (ead7e9 pwmsensor: Correct the type of MaxValue/MinValue)

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
               boost::asio::io_service& io, const std::string& name,
               const std::string& sensorConfiguration,
               sdbusplus::asio::object_server& objectServer,
               std::vector<thresholds::Threshold>&& thresholdData,
               uint8_t deviceAddress, uint8_t hostSMbusIndex, float pollRate,
               std::string& sensorTypeName);
    ~IpmbSensor() override;

    void init(void);
<<<<<<< PATCH SET (15b90d Code clean up.)
=======
    std::string getSubTypeUnits(void) const;
    void loadDefaults(void);
>>>>>>> BASE      (ead7e9 pwmsensor: Correct the type of MaxValue/MinValue)
    void runInitCmd(void);
<<<<<<< PATCH SET (15b90d Code clean up.)
    bool setSensorType(const std::string& sensorClass);
    void setSensorSubType(const std::string& sensorTypeName);
    void setScaleAndOffset(const SensorBaseConfigMap& sensorBaseConfig);
    void setReadMethod(const SensorBaseConfigMap& sensorBaseConfig);
=======
    bool processReading(const std::vector<uint8_t>& data, double& resp);
    void parseConfigValues(const SensorBaseConfigMap& entry);
    bool sensorClassType(const std::string& sensorClass);
    void sensorSubType(const std::string& sensorTypeName);
>>>>>>> BASE      (ead7e9 pwmsensor: Correct the type of MaxValue/MinValue)

<<<<<<< PATCH SET (15b90d Code clean up.)
  private:
    IpmbType type;
    IpmbSubType subType;
    double scaleVal = 1;
    double offsetVal = 0;
    uint8_t commandAddress;
    uint8_t netfn;
    uint8_t command;
    uint8_t deviceAddress;
    uint8_t errorCount;
    uint8_t hostSMbusIndex;
    uint8_t registerToRead = 0;
    bool isProxyRead = true;
    uint8_t deviceIndex = 0;
=======
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
>>>>>>> BASE      (ead7e9 pwmsensor: Correct the type of MaxValue/MinValue)
    std::vector<uint8_t> commandData;
    std::optional<uint8_t> initCommand;
    std::vector<uint8_t> initData;
    int sensorPollMs;
<<<<<<< PATCH SET (15b90d Code clean up.)
    ReadingFormat readingFormat;
=======

    ReadingFormat readingFormat = ReadingFormat::byte0;

  private:
    void sendIpmbRequest(void);
>>>>>>> BASE      (ead7e9 pwmsensor: Correct the type of MaxValue/MinValue)
    sdbusplus::asio::object_server& objectServer;
<<<<<<< PATCH SET (15b90d Code clean up.)
    boost::asio::deadline_timer waitTimer;
    std::function<bool(const std::vector<uint8_t>& data, double& resp)>
        readFunction;

    void read(void);
    void loadDefaults(void);
    void setReadFunction(void);
    void checkThresholds(void) override;
    std::string getSubTypeUnits(void);
    std::vector<uint8_t> getMeCommand();
    std::vector<uint8_t>
        getRawPmbusCommand(uint8_t messageType,
                           const std::vector<uint8_t>& pmbusCommand,
                           uint8_t readLength, bool isExtendedDeviceAddress,
                           bool doEnablePec, bool doReportPecErrors);
=======
    boost::asio::steady_timer waitTimer;
    void ipmbRequestCompletionCb(const boost::system::error_code& ec,
                                 const IpmbMethodType& response);
>>>>>>> BASE      (ead7e9 pwmsensor: Correct the type of MaxValue/MinValue)
};
