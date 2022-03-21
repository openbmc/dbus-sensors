#pragma once

#include <sensor.hpp>

using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

enum class SDRType
{
    sdrType01 = 1,
    sdrType02 = 2,
    sdrType03 = 3
};

enum class SDR01Fields
{
    dataLenType = 72,
    cntType = 3,
    nameLenType = 53,
    sdrAdrType = 56,
    sdrUnitType = 25,

    sdrUpCriType = 43,
    sdrLoCriType = 46,

    maxPosReadingMargin = 127,
    negHandleValue = 24,
    thermalConst = 256
};

enum class ThresholdCheck
{
    sdrThresAccess = 0x0C,
    sdrSensCapability = 13,
    sdrSensNoThres = 0,

    mDataByte = 28,
    mTolDataByte = 29,
    bDataByte = 30,
    bAcuDataByte = 31,
    rbExpDataByte = 33,
    bitShiftMsb = 6
};

enum class SDRCmd
{
    maxLength = 72,
    sdrLenBit = 0x1F,

    perCountByte = 16,
    sdrNxtRecLSB = 0,
    sdrNxtRecMSB = 1,

    sdrType = 5,
    sdrSenNum = 9,
    sdrLinear = 27,

    twosCompVal = 128
};

namespace sdr
{

static constexpr uint8_t netfnStorageReq = 0x0a;
static constexpr uint8_t cmdStorageGetSdrInfo = 0x20;
static constexpr uint8_t cmdStorageRsrvSdr = 0x22;
static constexpr uint8_t cmdStorageGetSdr = 0x23;

} // namespace sdr

struct SensorInfo
{
    std::string sensorReadName = "";
    uint8_t sensorUnit = 0;
    double thresUpperCri = 0;
    double thresLowerCri = 0;
    uint8_t sensorNumber = 0;
    uint8_t sensCap = 0;
};

struct SensorValConversion
{
    uint16_t mValue;
    double bValue;
    double expoVal;
    uint8_t negRead;
};

class IpmbSDRDevice : public std::enable_shared_from_this<IpmbSDRDevice>
{
  public:
    std::vector<uint8_t> getSdrData;
    uint16_t validRecordCount = 1;
    uint8_t iCnt = 0;
    uint8_t nextRecordIDLSB = 0;
    uint8_t nextRecordIDMSB = 0;

    uint8_t recordCountLSB = 1;
    uint8_t recordCountMSB = 2;

    inline static std::vector<uint8_t> sdrCommandData = {};

    inline static std::map<int, std::vector<SensorInfo>> sensorRecord;

    inline static std::map<int, std::map<uint8_t, SensorValConversion>>
        sensorValRecord;

    void
        getSDRRepositoryInfo(std::shared_ptr<sdbusplus::asio::connection>& conn,
                             uint8_t cmdAddr);

    void reserveSDRRepository(
        const std::shared_ptr<IpmbSDRDevice>& self,
        const std::shared_ptr<sdbusplus::asio::connection>& conn, uint16_t rec,
        uint8_t cmd);

    void getSDRSensorData(
        const std::shared_ptr<IpmbSDRDevice>& self,
        const std::shared_ptr<sdbusplus::asio::connection>& conn,
        uint16_t recordCount, uint8_t resrvIDLSB, uint8_t resrvIDMSB,
        uint8_t cmdAddr);

    void checkSDRData(std::vector<uint8_t> data, uint8_t cmdAddr, uint16_t rec,
                      uint16_t valid);

    void checkSDRThreshold(std::vector<uint8_t> data, uint8_t busIndex,
                           std::string tempName, uint8_t unit, int neg,
                           uint8_t sensorID);

    static double sensorValidation(uint16_t mValue, double bValue,
                                   double expValue, double value);
};
