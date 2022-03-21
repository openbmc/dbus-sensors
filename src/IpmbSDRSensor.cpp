#include <IpmbSDRSensor.hpp>

const constexpr char* ipmbService = "xyz.openbmc_project.Ipmi.Channel.Ipmb";
const constexpr char* ipmbDbusPath = "/xyz/openbmc_project/Ipmi/Channel/Ipmb";
const constexpr char* ipmbInterface = "org.openbmc.Ipmb";
const constexpr char* ipmbMethod = "sendRequest";
static constexpr uint8_t ipmbLeftShift = 2;
static constexpr uint8_t lun = 0;

void validateStatus(boost::system::error_code ec, const IpmbMethodType response,
                    uint8_t cmdAddr)
{
    const int status = std::get<0>(response);
    if (ec || status)
    {
        std::cerr << "Error reading from IPMB SDR for host "
                  << (cmdAddr >> ipmbLeftShift) + 1 << "\n";
        return;
    }
}

/* This function will store the record count of the SDR sensors for each IPMB
 * bus */
void IpmbSDRDevice::getSDRRepositoryInfo(
    std::shared_ptr<sdbusplus::asio::connection>& conn, uint8_t cmdAddr)
{
    std::shared_ptr<IpmbSDRDevice> self = std::make_shared<IpmbSDRDevice>();
    uint8_t sdrInfoDataSize = 14;

    conn->async_method_call(
        [self, conn, cmdAddr, sdrInfoDataSize](boost::system::error_code ec,
                                               const IpmbMethodType& response) {
            auto status = std::bind_front(validateStatus, ec, response);
            status(cmdAddr);

            const std::vector<uint8_t> data = std::get<5>(response);

            if (data.size() < sdrInfoDataSize)
            {
                std::cerr
                    << " IPMB Get SDR Repository Info data is empty for host "
                    << (cmdAddr >> ipmbLeftShift) + 1 << "\n";
                return;
            }

            uint16_t recordCount =
                (data[self->recordCountMSB] << 8) | data[self->recordCountLSB];

            self->reserveSDRRepository(self, conn, recordCount, cmdAddr);
        },
        ipmbService, ipmbDbusPath, ipmbInterface, ipmbMethod, cmdAddr,
        sdr::netfnStorageReq, lun, sdr::cmdStorageGetSdrInfo, sdrCommandData);
}

/* This function will store the reserve ID for each IPMB bus index */
void IpmbSDRDevice::reserveSDRRepository(
    const std::shared_ptr<IpmbSDRDevice>& self,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    uint16_t recordCount, uint8_t cmdAddr)
{
    uint8_t sdrReserveDataSize = 2;

    conn->async_method_call(
        [self, conn, cmdAddr, recordCount, sdrReserveDataSize](
            boost::system::error_code ec, const IpmbMethodType& response) {
            auto status = std::bind_front(validateStatus, ec, response);
            status(cmdAddr);

            const std::vector<uint8_t> data = std::get<5>(response);

            if (data.size() < sdrReserveDataSize)
            {
                std::cerr
                    << " IPMB SDR Reserve Repository data is empty for host "
                    << (cmdAddr >> ipmbLeftShift) + 1 << "\n";
                return;
            }
            uint8_t resrvIDLSB = data[0];
            uint8_t resrvIDMSB = data[1];

            self->getSDRSensorData(self, conn, recordCount, resrvIDLSB,
                                   resrvIDMSB, cmdAddr);
        },
        ipmbService, ipmbDbusPath, ipmbInterface, ipmbMethod, cmdAddr,
        sdr::netfnStorageReq, lun, sdr::cmdStorageRsrvSdr, sdrCommandData);
}

/* This function will read all the information related to the sensor
 * such as name, threshold value, unit, device address, SDR type */
void IpmbSDRDevice::getSDRSensorData(
    const std::shared_ptr<IpmbSDRDevice>& self,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    uint16_t recordCount, uint8_t resrvIDLSB, uint8_t resrvIDMSB,
    uint8_t cmdAddr)
{
    uint8_t sdrSensorDataSize = 18;
    uint8_t loopCount = (static_cast<uint8_t>(SDRCmd::perCountByte)) * iCnt;
    std::vector<uint8_t> commandData = {
        resrvIDLSB,      resrvIDMSB,
        nextRecordIDLSB, nextRecordIDMSB,
        loopCount,       (static_cast<uint8_t>(SDRCmd::perCountByte))};

    conn->async_method_call(
        [self, conn, recordCount, resrvIDLSB, resrvIDMSB, cmdAddr,
         sdrSensorDataSize](boost::system::error_code ec,
                            const IpmbMethodType& response) {
            auto status = std::bind_front(validateStatus, ec, response);
            status(cmdAddr);

            const std::vector<uint8_t> data = std::get<5>(response);
            if (data.size() < sdrSensorDataSize)
            {
                std::cerr << "IPMB SDR sensor data is empty for host "
                          << (cmdAddr >> ipmbLeftShift) + 1 << "\n";
                return;
            }

            for (size_t d : data)
            {
                self->getSdrData.push_back(d);
            }

            if ((self->validRecordCount <= recordCount) &&
                (self->iCnt < static_cast<uint8_t>(SDR01Fields::cntType)))
            {
                self->iCnt += 1;
                self->getSDRSensorData(self, conn, recordCount, resrvIDLSB,
                                       resrvIDMSB, cmdAddr);
            }
            else if ((self->validRecordCount == recordCount) &&
                     (self->iCnt == static_cast<uint8_t>(SDR01Fields::cntType)))
            {
                self->checkSDRData(self->getSdrData, cmdAddr, recordCount,
                                   self->validRecordCount);
                self->getSdrData.clear();
                self->nextRecordIDLSB = 0;
                self->nextRecordIDMSB = 0;
                return;
            }
            else
            {
                self->checkSDRData(self->getSdrData, cmdAddr, recordCount,
                                   self->validRecordCount);
                self->validRecordCount += 1;
                self->iCnt = 0;
                self->nextRecordIDLSB = self->getSdrData[static_cast<uint8_t>(
                    SDRCmd::sdrNxtRecLSB)];
                self->nextRecordIDMSB = self->getSdrData[static_cast<uint8_t>(
                    SDRCmd::sdrNxtRecMSB)];
                self->getSdrData.clear();
                self->getSDRSensorData(self, conn, recordCount, resrvIDLSB,
                                       resrvIDMSB, cmdAddr);
            }
        },
        ipmbService, ipmbDbusPath, ipmbInterface, ipmbMethod, cmdAddr,
        sdr::netfnStorageReq, lun, sdr::cmdStorageGetSdr, commandData);
}

/* This function will convert the SDR sensor data such as sensor unit, name, ID,
 * type from decimal to readable format */
void IpmbSDRDevice::checkSDRData(std::vector<uint8_t> data, uint8_t cmdAddr,
                                 uint16_t recordCount,
                                 uint16_t validRecordCount)
{
    if (validRecordCount == recordCount)
    {
        std::cerr << "SDR sensor for host " << (cmdAddr >> ipmbLeftShift) + 1
                  << " is completed \n";
    }

    if ((data.size() != static_cast<uint8_t>(SDR01Fields::dataLenType)) &&
        (data.size() <= static_cast<uint8_t>(SDRCmd::maxLength)))
    {
        std::cerr << "SDR Type 1 data length is invalid for host "
                  << (cmdAddr >> ipmbLeftShift) + 1 << "\n";
        return;
    }

    /* sensorType represents the SDR Type (Byte 5) such as 1, 2, 3 */
    uint8_t sensorType = data[static_cast<uint8_t>(SDRCmd::sdrType)];

    if (sensorType != static_cast<uint8_t>(SDRType::sdrType01))
    {
        return;
    }

    /* sensorId represents the sensor unique number (Byte 9) */
    uint8_t sensorID = data[static_cast<uint8_t>(SDRCmd::sdrSenNum)];

    /* neg represents the sensor with max reading margin (Byte 24) */
    int neg = data[static_cast<uint8_t>(SDR01Fields::negHandleValue)];

    /* unit represents the sensor unit (Byte 25) of SDR Type 1 */
    uint8_t unit = data[static_cast<uint8_t>(SDR01Fields::sdrUnitType)];

    /* iStrAddr represents the starting byte (Byte 56) for SDR Type 1 sensor
     * name */
    int strAddr = static_cast<uint8_t>(SDR01Fields::sdrAdrType);

    /* iStrLen represents the length of the sensor name for SDR Type 1 */
    int strLen = ((data[static_cast<uint8_t>(SDR01Fields::nameLenType)]) &
                  (static_cast<int>(SDRCmd::sdrLenBit)));

    /* Below for loop will convert the bytes to string and form a sensor name */

    std::vector<uint8_t> tempData;
    copy(data.begin() + strAddr, data.begin() + strAddr + strLen,
         back_inserter(tempData));
    std::string tempName(tempData.begin(), tempData.end());

    checkSDRThreshold(data, (cmdAddr >> ipmbLeftShift) + 1, tempName, unit, neg,
                      sensorType, sensorID);
}

/* This function will convert the raw value of threshold for each sensor */
void IpmbSDRDevice::checkSDRThreshold(std::vector<uint8_t> data,
                                      uint8_t busIndex, std::string tempName,
                                      uint8_t unit, int neg, uint8_t sensorType,
                                      uint8_t sensorID)
{
    if (data.empty())
    {
        return;
    }
    struct SensorInfo temp;

    /* sdrSensCapability (Byte 13) and(&) with sdrThresAccess(0x0C) will declare
     * whether threshold is present for each sensor */

    int threshold =
        (data[static_cast<uint8_t>(ThresholdCheck::sdrSensCapability)]) &
        (static_cast<uint8_t>(ThresholdCheck::sdrThresAccess));
    temp.sensCap = threshold;

    /* mData        - 10 bits
     * mDataByte    - Byte 28 - 8 bits LSB
     * mTolDataByte - Byte 29 - 2 bits MSB [7-6]
     *
     * mTolDataByte (MSB) is right shifted by 6 and again left shifted by 8 to
     * make a 10 bit value, then OR(|) with mDataByte (LSB)
     */

    uint16_t mData =
        ((data[static_cast<uint8_t>(ThresholdCheck::mTolDataByte)] >>
          (static_cast<uint8_t>(ThresholdCheck::bitShiftMsb)))
         << 8) |
        data[static_cast<uint8_t>(ThresholdCheck::mDataByte)];

    /* bData        - 10 bits
     * bDataByte    - Byte 30 - 8 bits LSB
     * bAcuDataByte - Byte 31 - 2 bits MSB [7-6]
     *
     * bAcuDataByte (MSB) is right shifted by 6 and again left shifted by 8 to
     * make a 10 bit value, then OR(|) with bDataByte (LSB)
     */

    uint16_t bData =
        ((data[static_cast<uint8_t>(ThresholdCheck::bAcuDataByte)] >>
          (static_cast<uint8_t>(ThresholdCheck::bitShiftMsb)))
         << 8) |
        data[static_cast<uint8_t>(ThresholdCheck::bDataByte)];

    /* rbExpDataByte (Byte 33) represents the exponent value
     *  Bit [3-0] - B Exponent 2's complement signed bit.
     *  Bit [7-4] - R Exponent 2's complement signed bit.
     */

    int8_t bExpVal =
        data[static_cast<uint8_t>(ThresholdCheck::rbExpDataByte)] & 0xF;
    if (bExpVal > 7)
    {
        bExpVal = (~bExpVal + 1) & 0xF;
    }

    /* Shifting the data to right by 4, since rExpVal has 4 bits from 4 to 7 in
     * byte 33 */

    int8_t rExpVal =
        (data[static_cast<uint8_t>(ThresholdCheck::rbExpDataByte)] >> 4) & 0xF;
    if (rExpVal > 7)
    {
        rExpVal = (~rExpVal + 1) & 0xF;
        rExpVal = -rExpVal;
    }

    /* linear represents the sensor's linearization (Byte 27) */
    uint8_t linear = data[static_cast<uint8_t>(SDRCmd::sdrLinear)];

    if (linear != 0)
    {
        std::cerr << tempName << " sensor is non linear \n";
        return;
    }

    /* Sensor Threshold Reading Conversion
     *
     *  Y = ((Mx + (B * 10^K1)) * (10^K2))
     *
     *  X  - Raw value of threshold
     *  M  - mData Value
     *  B  - bData Value
     *  K1 - Signed Exponent of bExpVal
     *  K2 - Signed Exponent of rExpVal
     */

    double bDataVal = bData * pow(10, bExpVal);
    double expVal = pow(10, rExpVal);

    double thresUpCri =
        ((mData * data[static_cast<uint8_t>(SDR01Fields::sdrUpCriType)]) +
         bDataVal) *
        expVal;
    double thresLoCri =
        ((mData * data[static_cast<uint8_t>(SDR01Fields::sdrLoCriType)]) +
         bDataVal) *
        expVal;

    temp.sensorReadName = std::move(tempName);
    temp.sensorUnit = unit;

    temp.thresUpperCri = thresUpCri;
    temp.thresLowerCri = thresLoCri;
    temp.mValue = mData;
    temp.bValue = bData;
    temp.bExp = bExpVal;
    temp.rExp = rExpVal;

    temp.sensorNumber = sensorID;
    temp.sensorSDRType = sensorType;

    temp.negRead = neg;
    temp.sensCap = threshold;

    sensorRecord[busIndex].push_back(temp);
}
