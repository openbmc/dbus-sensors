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
    SDRType1
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
} // namespace ipmi

struct IpmbSensor : public Sensor
{
    IpmbSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
               boost::asio::io_service& io, const std::string& name,
               const std::string& sensorConfiguration,
               sdbusplus::asio::object_server& objectServer,
               std::vector<thresholds::Threshold>&& thresholds,
               uint8_t deviceAddress, uint8_t hostSMbusIndex,
               uint8_t IPMBbusIndex, const float pollRate,
               std::string& sensorTypeName);
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
    uint8_t IPMBbusIndex;
    std::vector<uint8_t> commandData;
    std::optional<uint8_t> initCommand;
    std::vector<uint8_t> initData;
    int sensorPollMs;

    ReadingFormat readingFormat;

  private:
    sdbusplus::asio::object_server& objectServer;
    boost::asio::deadline_timer waitTimer;
};

namespace SDR
{
static constexpr uint8_t maxPosReadingMargin = 127;
static constexpr uint16_t thermalConst = 256;

static constexpr uint8_t netfnStorageReq = 0x0a;
static constexpr uint8_t cmdStorageGetSdrInfo = 0x20;
static constexpr uint8_t cmdStorageRsrvSdr = 0x22;
static constexpr uint8_t cmdStorageGetSdr = 0x23;

static constexpr uint8_t sdrType01 = 1;
static constexpr uint8_t sdrType02 = 2;
static constexpr uint8_t sdrType03 = 3;

static constexpr uint8_t cntType01 = 4;
static constexpr uint8_t cntType02 = 3;
static constexpr uint8_t cntType03 = 2;
static constexpr uint8_t perLoopByte = 16;

static constexpr uint8_t sdrNxtRecLSB = 0;
static constexpr uint8_t sdrNxtRecMSB = 1;
static constexpr uint8_t sdrType = 5;
static constexpr uint8_t sdrSenNum = 9;

static constexpr uint8_t sdrAdrType01 = 56;
static constexpr uint8_t sdrAdrType02 = 38;
static constexpr uint8_t sdrAdrType03 = 21;

static constexpr uint8_t sdrLenBit = 0x1F;
static constexpr uint8_t sdrLenType01 = 53;
static constexpr uint8_t sdrLenType02 = 35;
static constexpr uint8_t sdrLenType03 = 20;

static constexpr uint8_t sdrThresAcce = 0x0C;
static constexpr uint8_t sdrSensCapab = 13;
static constexpr uint8_t sdrSensNoThres = 0;

static constexpr uint8_t sdrUnitType01 = 25;
static constexpr uint8_t sdrUpCriType01 = 43;
static constexpr uint8_t sdrLoCriType01 = 46;

static constexpr uint8_t mDataByte = 28;
static constexpr uint8_t mTolDataByte = 29;
static constexpr uint8_t bDataByte = 30;
static constexpr uint8_t bAcuDataByte = 31;
static constexpr uint8_t rbExpDataBye = 33;
static constexpr uint8_t bitShiftMsb = 6;
} // namespace SDR

struct SDRSensorInfo
{
    std::string sensorReadName;
    uint8_t sensorUnit;
    double thresUpperCri;
    double thresLowerCri;
    uint16_t mValue;
    uint16_t bValue;
    uint8_t sensorNumber;
    uint8_t sensorSDRType;
    int8_t rExp;
    int8_t bExp;
    uint8_t negRead;
    uint8_t sensCap;
};

std::vector<SDRSensorInfo> sdrSensor[4];

class IpmbSDR
{
  public:
    uint16_t validRecordCount = 0;
    uint8_t nextRecordIDLSB;
    uint8_t nextRecordIDMSB;
    uint8_t IPMBbus;
    std::string boardName;

    void findObjects(std::shared_ptr<sdbusplus::asio::connection>& conn);
    void ipmbGetSdrInfo(std::shared_ptr<sdbusplus::asio::connection>& conn,
                        uint8_t cmdAddr);
    void ipmbSdrRsrv(std::shared_ptr<sdbusplus::asio::connection>& conn,
                     uint16_t recordCount, uint8_t cmdAddr);
    void ipmbGetSdrData(std::shared_ptr<sdbusplus::asio::connection>& conn,
                        uint8_t lsb, uint8_t msb, uint16_t recordCount,
                        uint8_t cmdAddr);
    void sdrDataProcess(std::vector<uint8_t> data, uint16_t recordCount,
                        uint8_t cmdAddr);
};
