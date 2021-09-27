#pragma once
#include <Utils.hpp>
#include <VariantVisitors.hpp>
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

using ReadFunction =
    std::function<bool(const std::vector<uint8_t>& data, double& resp)>;

namespace ipmi
{
namespace sensor
{
constexpr uint8_t netFn = 0x04;
constexpr uint8_t getSensorReading = 0x2d;

namespace read_me
{
constexpr uint8_t getSensorReading = 0xF5;
constexpr uint8_t bytesForTimestamp = 4;
constexpr uint8_t bytesForDeviceId = 3;
constexpr uint8_t fixedOffset = bytesForTimestamp + bytesForDeviceId;

void getRawData(uint8_t registerToRead, const std::vector<uint8_t>& input,
                std::vector<uint8_t>& result)
{
    result.clear();

    /* Every register is two bytes*/
    auto offset = fixedOffset + (registerToRead * 2);

    if (input.size() < static_cast<size_t>(offset + 1))
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
    void init(void);
    void runInitCmd(void);

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
    uint8_t sensorMeAddress = 0;
    std::vector<uint8_t> commandData;
    std::optional<uint8_t> initCommand;
    std::vector<uint8_t> initData;
    int sensorPollMs;
    ReadingFormat readingFormat;

    bool setSensorType(std::string& sensorClass)
    {
        if (sensorClass == "PxeBridgeTemp")
        {
            type = IpmbType::PXE1410CVR;
        }
        else if (sensorClass == "IRBridgeTemp")
        {
            type = IpmbType::IR38363VR;
        }
        else if (sensorClass == "HSCBridge")
        {
            type = IpmbType::ADM1278HSC;
        }
        else if (sensorClass == "MpsBridgeTemp")
        {
            type = IpmbType::mpsVR;
        }
        else if (sensorClass == "METemp" || sensorClass == "MESensor")
        {
            type = IpmbType::meSensor;
        }
        else
        {
            return false;
        }

        return true;
    }

    void setSensorSubType(std::string& sensorTypeName)
    {
        if (sensorTypeName == "voltage")
        {
            subType = IpmbSubType::volt;
        }
        else if (sensorTypeName == "power")
        {
            subType = IpmbSubType::power;
        }
        else if (sensorTypeName == "current")
        {
            subType = IpmbSubType::curr;
        }
        else if (sensorTypeName == "utilization")
        {
            subType = IpmbSubType::util;
        }
    }

    void setScaleAndOffset(const SensorBaseConfigMap& sensorBaseConfig)
    {
        auto findScaleVal = sensorBaseConfig.find("ScaleValue");
        if (findScaleVal != sensorBaseConfig.end())
        {
            scaleVal =
                std::visit(VariantToDoubleVisitor(), findScaleVal->second);
        }

        auto findOffsetVal = sensorBaseConfig.find("OffsetValue");
        if (findOffsetVal != sensorBaseConfig.end())
        {
            offsetVal =
                std::visit(VariantToDoubleVisitor(), findOffsetVal->second);
        }

        auto findPowerState = sensorBaseConfig.find("PowerState");
        if (findPowerState != sensorBaseConfig.end())
        {
            std::string powerState =
                std::visit(VariantToStringVisitor(), findPowerState->second);

            setReadState(powerState, readState);
        }
    }

    void setReadMethod(const SensorBaseConfigMap& sensorBaseConfig)
    {
        /*
         * Some sensor can be read in two ways
         * 1) Using proxy: BMC read command is proxy forward by ME
         * to sensor. 2) Using 'Get PMBUS Readings': ME responds to
         * BMC with sensor data.
         *
         * By default we assume the method is 1. And if ReadMethod
         * == "IPMI" we switch to method 2.
         */
        auto readMethod = sensorBaseConfig.find("ReadMethod");
        if (readMethod != sensorBaseConfig.end())
        {
            if (std::visit(VariantToStringVisitor(), readMethod->second) ==
                "IPMI")
            {
                /*
                 * In 'Get PMBUS Readings' the response containt a
                 * set of registers from the sensor. And different
                 * values such as temperature power voltage will be
                 * mapped to different registers.
                 */
                auto registerToReadConfig = sensorBaseConfig.find("Register");
                if (registerToReadConfig != sensorBaseConfig.end())
                {
                    registerToRead = std::visit(VariantToUnsignedIntVisitor(),
                                                registerToReadConfig->second);

                    /*
                     * In 'Get PMBUS Readings' since ME is
                     * responding with the sensor data we need
                     * to use the address for sensor in ME, this
                     * is different from the actual sensor
                     * address.
                     */
                    auto sensorMeAddressConfig =
                        sensorBaseConfig.find("SensorMeAddress");
                    if (sensorMeAddressConfig != sensorBaseConfig.end())
                    {
                        sensorMeAddress =
                            std::visit(VariantToUnsignedIntVisitor(),
                                       sensorMeAddressConfig->second);
                        isProxyRead = false;
                    }
                    else
                    {
                        std::cerr << "'SensorMeAddress' not found, defaulting to "
                                     "proxy method of reading sensor\n";
                    }
                }
                else
                {
                    std::cerr << "'Register' not found, defaulting to "
                                 "proxy method of reading sensor\n";
                }
            }
            else
            {
                std::cerr << "'ReadMethod' != 'IPMI', defaulting to "
                             "proxy method of reading sensor\n";
            }
        }
    }

  private:
    sdbusplus::asio::object_server& objectServer;
    boost::asio::deadline_timer waitTimer;
    ReadFunction readFunction;

    void read(void);
    std::string getSubTypeUnits(void);
    void loadDefaults(void);
    void setReadFunction();
};
