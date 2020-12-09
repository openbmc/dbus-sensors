#pragma once

#include <IpmbSensor.hpp>

namespace SDR
{
static constexpr uint8_t maxPosReadingMargin = 127;
static constexpr uint8_t negHandleValue = 24;
static constexpr uint16_t thermalConst = 256;

static constexpr uint8_t netfnStorageReq = 0x0a;
static constexpr uint8_t cmdStorageGetSdrInfo = 0x20;
static constexpr uint8_t cmdStorageRsrvSdr = 0x22;
static constexpr uint8_t cmdStorageGetSdr = 0x23;

static constexpr uint8_t sdrType01 = 1;
static constexpr uint8_t sdrType02 = 2;
static constexpr uint8_t sdrType03 = 3;

static constexpr uint8_t dataLenType01 = 72;
static constexpr uint8_t dataLenType02 = 54;
static constexpr uint8_t dataLenType03 = 36;

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
static constexpr uint8_t rbExpDataByte = 33;
static constexpr uint8_t bitShiftMsb = 6;
} // namespace SDR

struct SensorInfo
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

struct IpmbSDR : std::enable_shared_from_this<IpmbSDR>
{
  public:
    std::vector<uint8_t> getSdrData;
    uint16_t validRecordCount = 1;
    uint8_t iCnt = 0;
    uint8_t nextRecordIDLSB = 0;
    uint8_t nextRecordIDMSB = 0;

    inline static std::vector<uint8_t> sdrCommandData = {};
    inline static std::vector<SensorInfo> sensorRecord[4] = {};

    void ipmbGetSdrInfo(std::shared_ptr<sdbusplus::asio::connection>& conn,
                        uint8_t cmdAddr);

    void ipmbSdrRsrv(const std::shared_ptr<IpmbSDR>& self,
                     const std::shared_ptr<sdbusplus::asio::connection>& conn,
                     uint16_t rec, uint8_t cmd);

    void getSDRSensorData(
        const std::shared_ptr<IpmbSDR>& self,
        const std::shared_ptr<sdbusplus::asio::connection>& conn,
        uint16_t recordCount, uint8_t resrvIDLSB, uint8_t resrvIDMSB,
        uint8_t cmdAddr);

    void sdrDataProcess(std::vector<uint8_t> data, uint8_t cmdAddr,
                        uint16_t rec, uint16_t valid);
};
