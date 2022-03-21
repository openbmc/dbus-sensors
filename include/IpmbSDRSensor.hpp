#pragma once

#include <sensor.hpp>

using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

#pragma pack(push, 1)
struct SDR01DataFields
{
    uint8_t reserveIdLsb;                 // Byte 0
    uint8_t reserveIdMsb;                 // Byte 1
    uint8_t recordIdLsb;                  // Byte 2
    uint8_t recordIdMsb;                  // Byte 3
    uint8_t sdrVersion;                   // Byte 4
    uint8_t sdrType;                      // Byte 5
    uint8_t sdrDataLength;                // Byte 6
    uint8_t ownerID;                      // Byte 7
    uint8_t ownerLun;                     // Byte 8
    uint8_t sdrSenNum;                    // Byte 9
    uint8_t entityID;                     // Byte 10
    uint8_t entityInstance;               // Byte 11
    uint8_t sensorInitialization;         // Byte 12
    uint8_t sdrSensCapability;            // Byte 13
    uint8_t sdrSensorType;                // Byte 14
    uint8_t eventType;                    // Byte 15
    uint8_t assertionMask;                // Byte 16
    uint8_t lowerReadMask;                // Byte 17
    uint8_t reserveId1Lsb;                // Byte 18
    uint8_t reserveId1Msb;                // Byte 19
    uint8_t deassertionMask;              // Byte 20
    uint8_t upperReadMask;                // Byte 21
    uint8_t discreteMask;                 // Byte 22
    uint8_t settableMask;                 // Byte 23
    uint8_t sdrNegHandle;                 // Byte 24
    uint8_t sdrUnitType;                  // Byte 25
    uint8_t unitModifier;                 // Byte 26
    uint8_t sdrLinear;                    // Byte 27
    uint8_t mDataByte;                    // Byte 28
    uint8_t mTolDataByte;                 // Byte 29
    uint8_t bDataByte;                    // Byte 30
    uint8_t bAcuDataByte;                 // Byte 31
    uint8_t sensorDirection;              // Byte 32
    uint8_t rbExpDataByte;                // Byte 33
    uint8_t analogFlags;                  // Byte 34
    uint8_t nominalReading;               // Byte 35
    uint8_t reserveId2Lsb;                // Byte 36
    uint8_t reserveId2Msb;                // Byte 37
    uint8_t normalMax;                    // Byte 38
    uint8_t normalMin;                    // Byte 39
    uint8_t sensorMax;                    // Byte 40
    uint8_t sensorMin;                    // Byte 41
    uint8_t upperNonrecoverableThresh;    // Byte 42
    uint8_t sdrUpCriType;                 // Byte 43
    uint8_t upperNoncriticalThresh;       // Byte 44
    uint8_t lowerNonrecoverableThreshold; // Byte 45
    uint8_t sdrLoCriType;                 // Byte 46
    uint8_t lowerNoncriticalThreshold;    // Byte 47
    uint8_t positiveThresholdHysteresis;  // Byte 48
    uint8_t negativeThresholdHysteresis;  // Byte 49
    uint8_t reserve;                      // Byte 50
    uint8_t reserved;                     // Byte 51
    uint8_t oemReserved;                  // Byte 52
    uint8_t nameLenType;                  // Byte 53
};
#pragma pack(pop)

enum class SDRType
{
    sdrType01 = 1,
    sdrType02 = 2,
    sdrType03 = 3
};

namespace sdr
{

static constexpr uint8_t netfnStorageReq = 0x0a;
static constexpr uint8_t cmdStorageGetSdrInfo = 0x20;
static constexpr uint8_t cmdStorageRsrvSdr = 0x22;
static constexpr uint8_t cmdStorageGetSdr = 0x23;

static constexpr uint8_t sdrLenBit = 0x1F;
static constexpr uint8_t perCountByte = 16;
static constexpr uint8_t sdrNxtRecLSB = 0;
static constexpr uint8_t sdrNxtRecMSB = 1;
static constexpr uint8_t dataLengthByte = 6;

} // namespace sdr

namespace sdr01_command
{

static constexpr uint8_t maxPosReadingMargin = 127;
static constexpr double thermalConst = 256;
static constexpr uint8_t twosCompVal = 128;

static constexpr uint8_t sdrThresAccess = 0x0C;
static constexpr uint8_t sdrSensNoThres = 0;

} // namespace sdr01_command

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

    void checkSDRData(std::vector<uint8_t>& data, uint8_t dataLength);

    static void checkSDRThreshold(SDR01DataFields* resp, int busIndex,
                                  std::string tempName);

    inline static double sensorValCalculation(uint16_t mValue, double bValue,
                                              double expValue, double value);
};
