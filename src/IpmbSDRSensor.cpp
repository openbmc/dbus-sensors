#include <IpmbSDRSensor.hpp>

const constexpr char* ipmbService = "xyz.openbmc_project.Ipmi.Channel.Ipmb";
const constexpr char* ipmbDbusPath = "/xyz/openbmc_project/Ipmi/Channel/Ipmb";
const constexpr char* ipmbInterface = "org.openbmc.Ipmb";
const constexpr char* ipmbMethod = "sendRequest";
static constexpr uint8_t ipmbLeftShift = 2;
static constexpr uint8_t lun = 0;

IpmbSDRDevice::IpmbSDRDevice(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    uint8_t cmdAddr) :
    commandAddress(cmdAddr),
    hostIndex((cmdAddr >> ipmbLeftShift) + 1), conn(dbusConnection)
{}

void validateStatus(boost::system::error_code ec,
                    const IpmbMethodType& response, int hostIndex)
{
    const int status = std::get<0>(response);
    if (ec || status)
    {
        std::cerr << "Error reading from IPMB SDR for host " << hostIndex
                  << "\n";
        return;
    }
}

/* This function will store the record count of the SDR sensors for each IPMB
 * bus */
void IpmbSDRDevice::getSDRRepositoryInfo()
{
    std::weak_ptr<IpmbSDRDevice> weakRef = weak_from_this();
    size_t sdrInfoDataSize = 14;

    conn->async_method_call(
        [weakRef, sdrInfoDataSize](boost::system::error_code ec,
                                   const IpmbMethodType& response) {
            auto self = weakRef.lock();

            auto status = std::bind_front(validateStatus, ec, response);
            status(self->hostIndex);

            const std::vector<uint8_t> data = std::get<5>(response);

            if (data.size() < sdrInfoDataSize)
            {
                std::cerr
                    << " IPMB Get SDR Repository Info data is empty for host "
                    << self->hostIndex << "\n";
                return;
            }

            uint16_t recordCount =
                (data[self->recordCountMSB] << 8) | data[self->recordCountLSB];

            self->reserveSDRRepository(recordCount);
        },
        ipmbService, ipmbDbusPath, ipmbInterface, ipmbMethod, commandAddress,
        sdr::netfnStorageReq, lun, sdr::cmdStorageGetSdrInfo, sdrCommandData);
}

/* This function will store the reserve ID for each IPMB bus index */
void IpmbSDRDevice::reserveSDRRepository(uint16_t recordCount)
{
    std::weak_ptr<IpmbSDRDevice> weakRef = weak_from_this();
    size_t sdrReserveDataSize = 2;

    conn->async_method_call(
        [weakRef, recordCount, sdrReserveDataSize](
            boost::system::error_code ec, const IpmbMethodType& response) {
            auto self = weakRef.lock();

            auto status = std::bind_front(validateStatus, ec, response);
            status(self->hostIndex);

            const std::vector<uint8_t> data = std::get<5>(response);

            if (data.size() < sdrReserveDataSize)
            {
                std::cerr
                    << " IPMB SDR Reserve Repository data is empty for host "
                    << self->hostIndex << "\n";
                return;
            }
            uint8_t resrvIDLSB = data[0];
            uint8_t resrvIDMSB = data[1];

            self->getSDRSensorData(recordCount, resrvIDLSB, resrvIDMSB);
        },
        ipmbService, ipmbDbusPath, ipmbInterface, ipmbMethod, commandAddress,
        sdr::netfnStorageReq, lun, sdr::cmdStorageRsrvSdr, sdrCommandData);
}

/* This function will read all the information related to the sensor
 * such as name, threshold value, unit, device address, SDR type */
void IpmbSDRDevice::getSDRSensorData(uint16_t recordCount, uint8_t resrvIDLSB,
                                     uint8_t resrvIDMSB)
{
    std::weak_ptr<IpmbSDRDevice> weakRef = weak_from_this();
    size_t sdrSensorDataSize = 18;

    uint8_t loopCount = (static_cast<uint8_t>(SDRCommand::perCountByte)) * iCnt;
    std::vector<uint8_t> commandData = {
        resrvIDLSB,      resrvIDMSB,
        nextRecordIDLSB, nextRecordIDMSB,
        loopCount,       (static_cast<uint8_t>(SDRCommand::perCountByte))};

    conn->async_method_call(
        [weakRef, recordCount, resrvIDLSB, resrvIDMSB, sdrSensorDataSize](
            boost::system::error_code ec, const IpmbMethodType& response) {
            auto self = weakRef.lock();

            auto status = std::bind_front(validateStatus, ec, response);
            status(self->hostIndex);

            const std::vector<uint8_t> data = std::get<5>(response);
            if (data.size() < sdrSensorDataSize)
            {
                std::cerr << "IPMB SDR sensor data is empty for host "
                          << self->hostIndex << "\n";
                return;
            }

            self->handleSDRData(data, recordCount, resrvIDLSB, resrvIDMSB);
        },
        ipmbService, ipmbDbusPath, ipmbInterface, ipmbMethod, commandAddress,
        sdr::netfnStorageReq, lun, sdr::cmdStorageGetSdr, commandData);
}

/* This function will handle the sensor data received by IPMB response */
void IpmbSDRDevice::handleSDRData(std::vector<uint8_t> data,
                                  uint16_t recordCount, uint8_t resrvIDLSB,
                                  uint8_t resrvIDMSB)
{
    for (size_t d : data)
    {
        getSdrData.push_back(d);
    }

    if ((validRecordCount <= recordCount) &&
        (iCnt < static_cast<uint8_t>(SDR01Command::cntType)))
    {
        iCnt += 1;
        getSDRSensorData(recordCount, resrvIDLSB, resrvIDMSB);
    }
    else if ((validRecordCount == recordCount) &&
             (iCnt == static_cast<uint8_t>(SDR01Command::cntType)))
    {
        checkSDRData(getSdrData);
        getSdrData.clear();
        nextRecordIDLSB = 0;
        nextRecordIDMSB = 0;
        return;
    }
    else
    {
        checkSDRData(getSdrData);
        validRecordCount += 1;
        iCnt = 0;
        nextRecordIDLSB =
            getSdrData[static_cast<uint8_t>(SDRCommand::sdrNxtRecLSB)];
        nextRecordIDMSB =
            getSdrData[static_cast<uint8_t>(SDRCommand::sdrNxtRecMSB)];
        getSdrData.clear();
        getSDRSensorData(recordCount, resrvIDLSB, resrvIDMSB);
    }
}

/* This function will convert the SDR sensor data such as sensor unit, name, ID,
 * type from decimal to readable format */
void IpmbSDRDevice::checkSDRData(std::vector<uint8_t> data)
{
    if ((data.size() != static_cast<int>(SDR01Command::dataLength)) ||
        (data.size() < static_cast<int>(SDRCommand::maxLength)))
    {
        std::cerr << "SDR Type 1 data length is invalid for host " << hostIndex
                  << "\n";
        return;
    }

    SDR01DataFields* resp = reinterpret_cast<SDR01DataFields*>(data.data());

    /* sensorType represents the SDR Type (Byte 5) such as 1, 2, 3 */
    uint8_t sensorType = resp->sdrType;

    if (sensorType != static_cast<uint8_t>(SDRType::sdrType01))
    {
        return;
    }

    /*  dataLen represents the data length (Byte 6) for SDR sensor */
    int dataLen = resp->sdrDataLength;

    /* iStrLen represents the length of the sensor name for SDR Type 1 */
    int strLen =
        ((resp->nameLenType) & (static_cast<int>(SDRCommand::sdrLenBit)));

    /* iStrAddr represents the starting byte (Byte 56) for SDR sensor name */
    int strAddr =
        dataLen +
        ((dataLen / (static_cast<int>(SDRCommand::perCountByte))) * 4) -
        (strLen - 1);

    /* Below for loop will convert the bytes to string and form a sensor name */

    std::vector<uint8_t> tempData;
    std::copy(data.begin() + strAddr, data.begin() + strAddr + strLen,
              back_inserter(tempData));
    std::string tempName(tempData.begin(), tempData.end());

    checkSDRThreshold(resp, hostIndex, tempName);
}

/* This function will convert the raw value of threshold for each sensor */
void IpmbSDRDevice::checkSDRThreshold(SDR01DataFields* resp, int busIndex,
                                      std::string tempName)
{
    struct SensorInfo temp;

    /* sdrSensCapability (Byte 13) and(&) with sdrThresAccess(0x0C) will declare
     * whether threshold is present for each sensor */

    int threshold = (resp->sdrSensCapability) &
                    (static_cast<uint8_t>(SDR01Command::sdrThresAccess));

    /* mData        - 10 bits
     * mDataByte    - Byte 28 - 8 bits LSB
     * mTolDataByte - Byte 29 - 2 bits MSB [7-6]
     *
     * mTolDataByte (MSB) is right shifted by 6 and again left shifted by 8 to
     * make a 10 bit value, then OR(|) with mDataByte (LSB)
     */

    uint16_t mData = ((resp->mTolDataByte >>
                       (static_cast<uint8_t>(SDR01Command::bitShiftMsb)))
                      << 8) |
                     resp->mDataByte;

    /* bData        - 10 bits
     * bDataByte    - Byte 30 - 8 bits LSB
     * bAcuDataByte - Byte 31 - 2 bits MSB [7-6]
     *
     * bAcuDataByte (MSB) is right shifted by 6 and again left shifted by 8 to
     * make a 10 bit value, then OR(|) with bDataByte (LSB)
     */

    uint16_t bData = ((resp->bAcuDataByte >>
                       (static_cast<uint8_t>(SDR01Command::bitShiftMsb)))
                      << 8) |
                     resp->bDataByte;

    /* rbExpDataByte (Byte 33) represents the exponent value
     *  Bit [3-0] - B Exponent 2's complement signed bit.
     *  Bit [7-4] - R Exponent 2's complement signed bit.
     */

    int8_t bExpVal = resp->rbExpDataByte & 0xF;
    if (bExpVal > 7)
    {
        bExpVal = (~bExpVal + 1) & 0xF;
    }

    /* Shifting the data to right by 4, since rExpVal has 4 bits from 4 to 7 in
     * byte 33 */

    int8_t rExpVal = (resp->rbExpDataByte >> 4) & 0xF;
    if (rExpVal > 7)
    {
        rExpVal = (~rExpVal + 1) & 0xF;
        rExpVal = -rExpVal;
    }

    /* linear represents the sensor's linearization (Byte 27) */
    uint8_t linear = resp->sdrLinear;

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
        sensorValCalculation(mData, bDataVal, expVal, resp->sdrUpCriType);
    double thresLoCri =
        sensorValCalculation(mData, bDataVal, expVal, resp->sdrLoCriType);

    temp.sensorReadName = std::move(tempName);
    temp.sensorUnit = resp->sdrUnitType;

    temp.thresUpperCri = thresUpCri;
    temp.thresLowerCri = thresLoCri;

    temp.sensorNumber = resp->sdrSenNum;
    temp.sensCap = threshold;

    sensorRecord[busIndex].push_back(temp);

    struct SensorValConversion val;

    val.mValue = mData;
    val.bValue = bDataVal;
    val.expoVal = expVal;
    val.negRead = resp->sdrNegHandle;

    sensorValRecord[busIndex][resp->sdrSenNum] = val;
}

/* This function will calculate the sensor's threshold value */
double IpmbSDRDevice::sensorValCalculation(uint16_t mValue, double bValue,
                                           double expValue, double value)
{
    double sensorValue = ((mValue * value) + bValue) * expValue;
    return sensorValue;
}
