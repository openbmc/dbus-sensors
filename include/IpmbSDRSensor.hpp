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

namespace sdr
{
// IPMB Commands
static constexpr uint8_t netfnStorageReq = 0x0a;
static constexpr uint8_t cmdStorageGetSdrInfo = 0x20;
static constexpr uint8_t cmdStorageReserveSdr = 0x22;
static constexpr uint8_t cmdStorageGetSdr = 0x23;

// Get SDR Commands
static constexpr uint8_t sdrNxtRecLSB = 0;
static constexpr uint8_t sdrNxtRecMSB = 1;
static constexpr uint8_t perCountByte = 16;

// Sensor Record Bytes
static constexpr uint8_t sdrType = 5;
static constexpr uint8_t dataLengthByte = 6;
static constexpr uint8_t sdrSensorNum = 9;

} // namespace sdr

namespace sdrtype01
{
// Negative Handle Commands
static constexpr uint8_t maxPosReadingMargin = 127;
static constexpr uint8_t twosCompVal = 128;
static constexpr double thermalConst = 256;

static constexpr uint8_t sdrSensNoThres = 0;
static constexpr uint8_t sensorCapability = 13;
static constexpr uint8_t sdrNegHandle = 24;
static constexpr uint8_t sdrUnitType = 25;
static constexpr uint8_t sdrLinearByte = 27;

// SDR Type 1 Thresholds Commands
static constexpr uint8_t mDataByte = 28;
static constexpr uint8_t mTolDataByte = 29;
static constexpr uint8_t bDataByte = 30;
static constexpr uint8_t bAcuDataByte = 31;
static constexpr uint8_t rbExpDataByte = 33;
static constexpr uint8_t upperCriticalThreshold = 43;
static constexpr uint8_t lowerCriticalThreshold = 46;
static constexpr uint8_t nameLengthByte = 53;

} // namespace sdrtype01

struct SensorInfo
{
    std::string sensorReadName;
    uint8_t sensorUnit = 0;
    double thresUpperCri = 0;
    double thresLowerCri = 0;
    uint8_t sensorNumber = 0;
    uint8_t sensCap = 0;
};

struct SensorValConversion
{
    uint16_t mValue = 0;
    double bValue = 0;
    double expoVal = 0;
    uint8_t negRead = 0;
};

inline std::map<int, std::vector<SensorInfo>> sensorRecord;
inline std::map<int, std::map<uint8_t, SensorValConversion>> sensorValRecord;

class IpmbSDRDevice : public std::enable_shared_from_this<IpmbSDRDevice>
{
  public:
    IpmbSDRDevice(std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
                  uint8_t cmdAddr);

    uint8_t commandAddress = 0;
    int hostIndex = 0;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    std::vector<uint8_t> sdrData;
    uint16_t validRecordCount = 1;
    uint8_t iCnt = 0;
    uint8_t nextRecordIDLSB = 0;
    uint8_t nextRecordIDMSB = 0;

    std::vector<uint8_t> sdrCommandData = {};

    void getSDRRepositoryInfo();

    void reserveSDRRepository(uint16_t recordCount);

    void getSDRSensorData(uint16_t recordCount, uint8_t resrvIDLSB,
                          uint8_t resrvIDMSB);

    void handleSDRData(const std::vector<uint8_t>& data, uint16_t recordCount,
                       uint8_t resrvIDLSB, uint8_t resrvIDMSB);

    void checkSDRData(std::vector<uint8_t>& sdrDataBytes,
                      uint8_t dataLength) const;

    static void checkSDRType01Threshold(std::vector<uint8_t>& sdrDataBytes,
                                        int busIndex, std::string tempName);

    inline static double sensorValCalculation(uint16_t mValue, double bValue,
                                              double expValue, double value);
};
