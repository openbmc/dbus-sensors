#pragma once
#include "sensor.hpp"

#include <boost/asio/deadline_timer.hpp>
#include <boost/container/flat_map.hpp>

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
    SDRThresSensor,
    SDRDiscEvtSensor,
    version
};

enum class IpmbSubType
{
    temp,
    curr,
    power,
    volt,
    util,
    version
};

enum class ReadingFormat
{
    byte0,
    byte3,
    elevenBit,
    elevenBitShift,
    sdrThres,
    sdrDiscEvt,
    version
};

std::vector<std::string> Sensor_Unit{"unspecified", "degrees C", "degrees F",
                                     "degrees K",   "Volts",     "Amps",
                                     "Watts",       "Joules",    "Coulombs"};

namespace sdr
{
void ipmbGetSdrInfo(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection);
void ipmbSdrRsrv(std::shared_ptr<sdbusplus::asio::connection>& dbusConnection);
void ipmbGetSdr(std::shared_ptr<sdbusplus::asio::connection>& dbusConnection);
void sdrDataProcess();
double dataConversion(double, uint8_t);

enum
{
    SENSOR_MB_OUTLET_TEMP = 0x01,
    SENSOR_MB_INLET_TEMP = 0x07,
    SENSOR_PCH_TEMP = 0x08,
    SENSOR_SOC_TEMP = 0x05,
    SENSOR_SOC_DIMMA_TEMP = 0xB4,
    SENSOR_SOC_DIMMB_TEMP = 0xB6,
    SENSOR_SOC_THERM_MARGIN = 0x09,
};
// List of sensors which need to do negative reading handle
const uint8_t neg_reading_sensor_support_list[] = {
    /* Temperature sensors*/
    SENSOR_MB_OUTLET_TEMP, SENSOR_MB_INLET_TEMP,  SENSOR_PCH_TEMP,
    SENSOR_SOC_TEMP,       SENSOR_SOC_DIMMA_TEMP, SENSOR_SOC_DIMMB_TEMP,
};

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

static std::vector<std::string> sensorReadName;
static std::vector<std::string> sensorUnit;
static std::vector<double> thresUpperCri;
static std::vector<double> thresLowerCri;
static std::vector<uint32_t> ipmbBus;
static std::vector<uint16_t> mValue;
static std::vector<uint16_t> bValue;
static std::vector<uint8_t> sensorNumber;
static std::vector<uint8_t> sensorSDRType;
static std::vector<int8_t> rExp;
static std::vector<int8_t> bExp;

std::vector<uint8_t> sdrCommandData;
std::vector<uint8_t> getSdrData;

std::string sensorName;
std::string strUnit;
std::string hostName;

double upperCri;
double lowerCri;
uint16_t recordCount;
uint16_t validRecordCount;
uint8_t sdrCommandAddress;
uint8_t sdrNetfn;
uint8_t sdrCommand;
uint8_t curRecord;
uint8_t dev_addr;
uint8_t cmdAddr;
uint8_t resrvIDLSB = 0;
uint8_t resrvIDMSB = 0;
uint8_t nextRecordIDLSB = 0;
uint8_t nextRecordIDMSB = 0;
} // namespace sdr

namespace ipmi
{
namespace sensor
{
constexpr uint8_t netFn = 0x04;
constexpr uint8_t getSensorReading = 0x2d;

static bool isValid(const std::vector<uint8_t>& data)
{
    constexpr auto ReadingUnavailableBit = 5;

    // Proper 'Get Sensor Reading' response has at least 4 bytes, including
    // Completion Code. Our IPMB stack strips Completion Code from payload so we
    // compare here against the rest of payload
    if (data.size() < 3)
    {
        return false;
    }

    // Per IPMI 'Get Sensor Reading' specification
    if (data[1] & (1 << ReadingUnavailableBit))
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
namespace oem
{
constexpr uint8_t netFn = 0x38;
constexpr uint8_t command = 0x0b;
} // namespace oem
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
    ~IpmbSensor();

    void checkThresholds(void) override;
    void read(void);
    void sdrRead(void);
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
    std::string versionTypeName;
    int pollTimeValue;
    uint8_t index;

    ReadingFormat readingFormat;

  private:
    sdbusplus::asio::object_server& objectServer;
    boost::asio::deadline_timer waitTimer;
};
