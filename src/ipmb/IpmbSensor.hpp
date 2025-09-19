#pragma once
#include "asio/Thresholds.hpp"
#include "asio/sensor.hpp"
#include "utils/Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

constexpr const char* sensorType = "IpmbSensor";
constexpr const char* sdrInterface = "IpmbDevice";

enum class IpmbType
{
    none,
    meSensor,
    PXE1410CVR,
    IR38363VR,
    ADM1278HSC,
    mpsVR,
    SMPro
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
    nineBit,
    tenBit,
    elevenBit,
    elevenBitShift,
    linearElevenBit,
    fifteenBit
};

namespace ipmi
{
namespace sensor
{
constexpr uint8_t netFn = 0x04;
constexpr uint8_t getSensorReading = 0x2d;

static inline bool isValid(const std::vector<uint8_t>& data)
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

    void checkThresholds() override;
    void read();
    void init();
    std::string getSubTypeUnits() const;
    void loadDefaults();
    void runInitCmd();
    static bool processReading(ReadingFormat readingFormat, uint8_t command,
                               const std::vector<uint8_t>& data, double& resp,
                               size_t errCount);
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
    void sendIpmbRequest();
    sdbusplus::asio::object_server& objectServer;
    boost::asio::steady_timer waitTimer;
    void ipmbRequestCompletionCb(const boost::system::error_code& ec,
                                 const IpmbMethodType& response);
};

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<IpmbSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection);

void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<IpmbSensor>>&
        sensors);
