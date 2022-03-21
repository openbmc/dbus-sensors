#pragma once

#include <sensor.hpp>

using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

struct SDR01DataFields
{
    std::array<uint8_t, 4> recordId;
    uint8_t sdrVersion;    // Byte 4
    uint8_t sdrType;       // Byte 5
    uint8_t sdrDataLength; // Byte 6
    std::array<uint8_t, 2> sensorId;
    uint8_t sdrSenNum; // Byte 9
    std::array<uint8_t, 3> entityId;
    uint8_t sdrSensCapability; // Byte 13
    uint8_t sdrSensorType;     // Byte 14
    uint8_t eventType;         // Byte 15
    std::array<uint8_t, 8> thresMask;
    uint8_t sdrNegHandle;    // Byte 24
    uint8_t sdrUnitType;     // Byte 25
    uint8_t unitModifier;    // Byte 26
    uint8_t sdrLinear;       // Byte 27
    uint8_t mDataByte;       // Byte 28
    uint8_t mTolDataByte;    // Byte 29
    uint8_t bDataByte;       // Byte 30
    uint8_t bAcuDataByte;    // Byte 31
    uint8_t sensorDirection; // Byte 32
    uint8_t rbExpDataByte;   // Byte 33
    std::array<uint8_t, 9> sensorRead;
    uint8_t sdrUpCriType; // Byte 43
    std::array<uint8_t, 2> upperThres;
    uint8_t sdrLoCriType; // Byte 46
    std::array<uint8_t, 6> lowerThres;
    uint8_t nameLenType; // Byte 53
};

enum class SDRType
{
    sdrType01 = 1,
    sdrType02 = 2,
    sdrType03 = 3
};

enum class SDR01Command
{
    dataLength = 72,
    cntType = 3,

    maxPosReadingMargin = 127,
    thermalConst = 256,
    twosCompVal = 128,

    sdrThresAccess = 0x0C,
    sdrSensNoThres = 0,

    bitShiftMsb = 6
};

enum class SDRCommand
{
    maxLength = 72,
    sdrLenBit = 0x1F,

    perCountByte = 16,
    sdrNxtRecLSB = 0,
    sdrNxtRecMSB = 1
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
    IpmbSDRDevice(std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
                  uint8_t cmdAddr);

    uint8_t commandAddress = 0;
    int hostIndex = 0;
    std::shared_ptr<sdbusplus::asio::connection> conn;

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

    void getSDRRepositoryInfo();

    void reserveSDRRepository(uint16_t recordCount);

    void getSDRSensorData(uint16_t recordCount, uint8_t resrvIDLSB,
                          uint8_t resrvIDMSB);

    void handleSDRData(const std::vector<uint8_t>& data, uint16_t recordCount,
                       uint8_t resrvIDLSB, uint8_t resrvIDMSB);

    void checkSDRData(std::vector<uint8_t> data);

    void checkSDRThreshold(SDR01DataFields* resp, int busIndex,
                           std::string tempName);

    static double sensorValCalculation(uint16_t mValue, double bValue,
                                       double expValue, double value);
};
