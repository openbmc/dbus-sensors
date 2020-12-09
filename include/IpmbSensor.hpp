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
    SDRType,
    SDRStEvtType
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
    sdrTyp,
    sdrStEvt
};

std::vector<std::string> Sensor_Unit{"unspecified", "degrees C", "degrees F",
                                     "degrees K",   "Volts",     "Amps",
                                     "Watts",       "Joules",    "Coulombs"};

namespace sdr
{
void ipmbGetSdrInfo();
void ipmbSdrRsrv();
void ipmbGetSdr();
void sdrDataProcess();

static constexpr uint8_t sdrNetfun = 0x0a;
static constexpr uint8_t sdrGetInfo = 0x20;
static constexpr uint8_t sdrRsrv = 0x22;
static constexpr uint8_t sdrGet = 0x23;

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

static constexpr uint8_t sdrEveType01 = 14;
static constexpr uint8_t sdrEveType02 = 14;
static constexpr uint8_t sdrEveType03 = 12;

static constexpr uint8_t sdrUnitType01 = 25;
static constexpr uint8_t sdrUpCriType01 = 43;
static constexpr uint8_t sdrLoCriType01 = 46;

static std::vector<std::string> sensorReadName;
static std::vector<std::string> sensorUnit;
static std::vector<uint32_t> ipmbBus;

std::vector<uint8_t> sdrCommandData;
std::vector<uint8_t> getSdrData;

static std::vector<uint8_t> sensorNumber;
static std::vector<uint8_t> sensorSDRType;
static std::vector<uint8_t> sensorSDREvent;
static std::vector<uint8_t> thresUpperCri;
static std::vector<uint8_t> thresLowerCri;

std::string sensorName;
std::string sensorTypeName;
std::string strUnit;
std::string hostName;

uint16_t recordCount;

uint8_t sdrCommandAddress;
uint8_t sdrNetfn;
uint8_t sdrCommand;
uint8_t upperCri;
uint8_t lowerCri;
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

    ReadingFormat readingFormat;

  private:
    sdbusplus::asio::object_server& objectServer;
    boost::asio::deadline_timer waitTimer;
};
