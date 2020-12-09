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
    mpsVR,
    SDRThresSensor
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
    linearElevenBit,
    sdrThres
};

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
} // namespace ipmi

struct IpmbSensor : public Sensor
{
    IpmbSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
               boost::asio::io_service& io, const std::string& name,
               const std::string& sensorConfiguration,
               sdbusplus::asio::object_server& objectServer,
               std::vector<thresholds::Threshold>&& thresholds,
               uint8_t deviceAddress, uint8_t hostSMbusIndex,
               std::string& sensorTypeName);
    ~IpmbSensor() override;

    void checkThresholds(void) override;
    void read(void);
    void init(void);
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

    ReadingFormat readingFormat;

  private:
    sdbusplus::asio::object_server& objectServer;
    boost::asio::deadline_timer waitTimer;
};

std::vector<std::string> Sensor_Unit{"Meters", "DegreesC", "RPMS", "Joules",
                                     "Volts",  "Amperes",  "Watts"};

class IpmbSDR
{
  public:
    std::vector<std::string> sensorReadName;
    std::vector<std::string> sensorUnit;
    std::vector<double> thresUpperCri;
    std::vector<double> thresLowerCri;
    std::vector<uint16_t> mValue;
    std::vector<uint16_t> bValue;
    std::vector<uint8_t> sensorNumber;
    std::vector<uint8_t> sensorSDRType;
    std::vector<int8_t> rExp;
    std::vector<int8_t> bExp;
    std::vector<uint8_t> negRead;
    std::vector<uint8_t> sensCap;
    std::vector<uint8_t> getSdrData;

    std::string sensorName;
    std::string strUnit;
    std::string hostName;

    uint16_t recordCount;
    uint16_t validRecordCount;
    uint8_t sdrCommandAddress;
    uint8_t curRecord;
    uint8_t devAddr;
    uint8_t command;
    uint8_t cmdAddr;
    uint8_t resrvIDLSB = 0;
    uint8_t resrvIDMSB = 0;
    uint8_t nextRecordIDLSB = 0;
    uint8_t nextRecordIDMSB = 0;

    void ipmbGetSdrInfo(
        std::shared_ptr<sdbusplus::asio::connection>& dbusConnection);
    void ipmbSdrRsrv(
        std::shared_ptr<sdbusplus::asio::connection>& dbusConnection);
    void ipmbGetSdr(
        std::shared_ptr<sdbusplus::asio::connection>& dbusConnection);
    void sdrDataProcess();
    void sdrRead(std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
                 std::unique_ptr<IpmbSensor>& sensor);
    double dataConversion(double, uint8_t);

    uint8_t maxPosReadingMargin = 127;
    uint16_t thermalConst = 256;

    uint8_t netfnStorageReq = 0x0a;
    uint8_t cmdStorageGetSdrInfo = 0x20;
    uint8_t cmdStorageRsrvSdr = 0x22;
    uint8_t cmdStorageGetSdr = 0x23;

    uint8_t sdrType01 = 1;
    uint8_t sdrType02 = 2;
    uint8_t sdrType03 = 3;

    uint8_t cntType01 = 4;
    uint8_t cntType02 = 3;
    uint8_t cntType03 = 2;
    uint8_t perLoopByte = 16;

    uint8_t sdrNxtRecLSB = 0;
    uint8_t sdrNxtRecMSB = 1;
    uint8_t sdrType = 5;
    uint8_t sdrSenNum = 9;

    uint8_t sdrAdrType01 = 56;
    uint8_t sdrAdrType02 = 38;
    uint8_t sdrAdrType03 = 21;

    uint8_t sdrLenBit = 0x1F;
    uint8_t sdrLenType01 = 53;
    uint8_t sdrLenType02 = 35;
    uint8_t sdrLenType03 = 20;

    uint8_t sdrThresAcce = 0x0C;
    uint8_t sdrSensCapab = 13;
    uint8_t sdrSensNoThres = 0;

    uint8_t sdrUnitType01 = 25;
    uint8_t sdrUpCriType01 = 43;
    uint8_t sdrLoCriType01 = 46;

    uint8_t mDataByte = 28;
    uint8_t mTolDataByte = 29;
    uint8_t bDataByte = 30;
    uint8_t bAcuDataByte = 31;
    uint8_t rbExpDataBye = 33;
    uint8_t bitShiftMsb = 6;
};
