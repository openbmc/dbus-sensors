#pragma once

#include <sensor.hpp>

static constexpr uint8_t ipmbLeftShift = 2;
static constexpr uint8_t lun = 0;

using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

namespace sdr
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
} // namespace sdr
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
    bool sdrVerify = true;

    inline static std::vector<uint8_t> sdrCommandData = {};
    inline static std::array<std::vector<SensorInfo>, 4> sensorRecord = {};

    inline static std::map<uint8_t, std::string> sensorUnits = {
        {1, "temperature"}, {4, "voltage"}, {5, "current"}, {6, "power"}};

    void ipmbGetSdrInfo(std::shared_ptr<sdbusplus::asio::connection>& conn,
                        uint8_t cmdAddr);

    void ipmbSdrRsrv(const std::shared_ptr<IpmbSDR>& self,
                     const std::shared_ptr<sdbusplus::asio::connection>& conn,
                     uint16_t rec, uint8_t cmd);

    void sdrDataCheck(std::vector<uint8_t> data, uint8_t cmdAddr, uint16_t rec,
                      uint16_t valid);

    void sdrThresholdCheck(std::vector<uint8_t> data, uint8_t busIndex,
                           std::string tempName, uint8_t unit, int neg,
                           uint8_t sensorType, uint8_t sensorID);

    /* This function will read all the information related to the sensor
     * such as name, threshold value, unit, device address, SDR type */
    void getSDRSensorData(
        const std::shared_ptr<IpmbSDR>& self,
        const std::shared_ptr<sdbusplus::asio::connection>& conn,
        uint16_t recordCount, uint8_t resrvIDLSB, uint8_t resrvIDMSB,
        uint8_t cmdAddr)
    {
        uint8_t loopCount = sdr::perLoopByte * iCnt;
        std::vector<uint8_t> commandData = {resrvIDLSB,      resrvIDMSB,
                                            nextRecordIDLSB, nextRecordIDMSB,
                                            loopCount,       sdr::perLoopByte};
        conn->async_method_call(
            [self, conn, recordCount, resrvIDLSB, resrvIDMSB, cmdAddr](
                boost::system::error_code ec, const IpmbMethodType& response) {
                const int status = std::get<0>(response);
                if (ec || status)
                {
                    std::cerr << "Error reading from getSensorData for host "
                              << ((cmdAddr >> ipmbLeftShift) + 1) << "\n";
                    return;
                }
                const std::vector<uint8_t> data = std::get<5>(response);
                if (data.empty())
                {
                    std::cerr << "IPMB SDR sensor data is empty for host "
                              << ((cmdAddr >> ipmbLeftShift) + 1) << "\n";
                    return;
                }
                for (size_t d : data)
                {
                    self->getSdrData.push_back(d);
                }
                if ((self->validRecordCount <= recordCount) &&
                    (self->iCnt < sdr::cntType02))
                {
                    self->iCnt += 1;
                    self->getSDRSensorData(self, conn, recordCount, resrvIDLSB,
                                           resrvIDMSB, cmdAddr);
                }
                else if ((self->validRecordCount == recordCount) &&
                         (self->iCnt == sdr::cntType02))
                {
                    self->sdrDataCheck(self->getSdrData, cmdAddr, recordCount,
                                       self->validRecordCount);
                    self->getSdrData.clear();
                    self->validRecordCount = 1;
                    self->nextRecordIDLSB = 0;
                    self->nextRecordIDMSB = 0;
                    return;
                }
                else
                {
                    self->sdrDataCheck(self->getSdrData, cmdAddr, recordCount,
                                       self->validRecordCount);
                    self->validRecordCount += 1;
                    self->iCnt = 0;
                    self->nextRecordIDLSB = self->getSdrData[sdr::sdrNxtRecLSB];
                    self->nextRecordIDMSB = self->getSdrData[sdr::sdrNxtRecMSB];
                    self->getSdrData.clear();
                    self->getSDRSensorData(self, conn, recordCount, resrvIDLSB,
                                           resrvIDMSB, cmdAddr);
                }
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", cmdAddr, sdr::netfnStorageReq, lun,
            sdr::cmdStorageGetSdr, commandData);
    }
};
