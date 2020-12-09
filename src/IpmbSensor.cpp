/*
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <IpmbSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

constexpr const bool debug = false;

constexpr const char* configInterface =
    "xyz.openbmc_project.Configuration.IpmbSensor";
constexpr const char* sdrInterface =
    "xyz.openbmc_project.Configuration.SDRSensor";
static constexpr double ipmbMaxReading = 0xFF;
static constexpr double ipmbMinReading = 0;

static constexpr uint8_t meAddress = 1;
static constexpr uint8_t lun = 0;
static constexpr uint8_t hostSMbusIndexDefault = 0x03;

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

boost::container::flat_map<std::string, std::unique_ptr<IpmbSensor>> sensors;

std::unique_ptr<boost::asio::deadline_timer> initCmdTimer;

IpmbSensor::IpmbSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                       boost::asio::io_service& io,
                       const std::string& sensorName,
                       const std::string& sensorConfiguration,
                       sdbusplus::asio::object_server& objectServer,
                       std::vector<thresholds::Threshold>&& thresholdData,
                       uint8_t deviceAddress, uint8_t hostSMbusIndex,
                       std::string& sensorTypeName) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdData), sensorConfiguration,
           "xyz.openbmc_project.Configuration.ExitAirTemp", ipmbMaxReading,
           ipmbMinReading, conn, PowerState::on),
    deviceAddress(deviceAddress), hostSMbusIndex(hostSMbusIndex),
    objectServer(objectServer), waitTimer(io)
{
    std::string dbusPath = sensorPathPrefix + sensorTypeName + "/" + name;

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    association = objectServer.add_interface(dbusPath, association::interface);
}

IpmbSensor::~IpmbSensor()
{
    waitTimer.cancel();
    objectServer.remove_interface(thresholdInterfaceWarning);
    objectServer.remove_interface(thresholdInterfaceCritical);
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

void IpmbSensor::init(void)
{
    if (type == IpmbType::SDRThresSensor)
    {
        sensorInterface->register_property(
            "Unit", std::string(""),
            sdbusplus::asio::PropertyPermission::readWrite);

        readingFormat = ReadingFormat::sdrThres;
        setInitialProperties(dbusConnection);
        return;
    }

    loadDefaults();
    setInitialProperties(dbusConnection);

    if (initCommand)
    {
        runInitCmd();
    }

    read();
}

void IpmbSensor::runInitCmd()
{
    if (initCommand)
    {
        dbusConnection->async_method_call(
            [this](boost::system::error_code ec,
                   const IpmbMethodType& response) {
                const int& status = std::get<0>(response);

                if (ec || status)
                {
                    std::cerr
                        << "Error setting init command for device: " << name
                        << "\n";
                }
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", commandAddress, netfn, lun, *initCommand, initData);
    }
}

std::vector<uint8_t>
    findObjects(std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (!dbusConnection)
    {
        std::cerr << "Connection not created\n";
    }

    std::vector<uint8_t> host;
    auto method = dbusConnection->new_method_call(
        entityManagerName, "/", "org.freedesktop.DBus.ObjectManager",
        "GetManagedObjects");

    method.append();
    auto reply = dbusConnection->call(method);
    if (reply.is_method_error())
    {
        std::cerr << "Error contacting entity manager in find objects \n";
    }

    ManagedObjectType resp;
    reply.read(resp);
    for (const auto& pathPair : resp)
    {
        for (const auto& entry : pathPair.second)
        {
            if (entry.first != sdrInterface)
            {
                continue;
            }

            auto findBusType = entry.second.find("Bus");
            if (findBusType != entry.second.end())
            {
                uint8_t hostNum = std::visit(VariantToUnsignedIntVisitor(),
                                             findBusType->second);
                host.push_back(hostNum);
            }
        }
    }
    return host;
}

/* This function will store the record count of the SDR sensors in the host */
void IpmbSDR::ipmbGetSdrInfo(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    sdrCommandAddress = cmdAddr;
    std::vector<uint8_t> sdrCommandData = {};

    auto method = dbusConnection->new_method_call(
        "xyz.openbmc_project.Ipmi.Channel.Ipmb",
        "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
        "sendRequest");

    method.append(sdrCommandAddress, netfnStorageReq, lun, cmdStorageGetSdrInfo,
                  sdrCommandData);

    auto reply = dbusConnection->call(method);
    if (reply.is_method_error())
    {
        std::cerr << "Error reading from IpmbGetSdrInfo";
        return;
    }

    IpmbMethodType resp;
    reply.read(resp);

    std::vector<uint8_t> data;
    data = std::get<5>(resp);

    if (data.size() > 1)
    {
        recordCount = (((data[2] << 8) & 0xFF00) | data[1]);
    }
    else
    {
        return;
    }
}

/* This function will store the reserve ID for the sensor */
void IpmbSDR::ipmbSdrRsrv(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    sdrCommandAddress = cmdAddr;
    std::vector<uint8_t> sdrCommandData = {};

    auto method = dbusConnection->new_method_call(
        "xyz.openbmc_project.Ipmi.Channel.Ipmb",
        "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
        "sendRequest");

    method.append(sdrCommandAddress, netfnStorageReq, lun, cmdStorageRsrvSdr,
                  sdrCommandData);

    auto reply = dbusConnection->call(method);
    if (reply.is_method_error())
    {
        std::cerr << "Error reading from IpmbSdrRsrv";
    }

    IpmbMethodType resp;
    reply.read(resp);

    std::vector<uint8_t> data;
    data = std::get<5>(resp);
    resrvIDLSB = data[0];
    resrvIDMSB = data[1];
}

/* This function will read all the information related to the sensor
 * such as name, threshold value, unit, device address, SDR type */
void IpmbSDR::ipmbGetSdr(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    getSdrData.clear();
    int iLoop = cntType01;
    for (int iCnt = 0; iCnt < iLoop; iCnt++)
    {
        sdrCommandAddress = cmdAddr;
        uint8_t sdrCount = perLoopByte * iCnt;
        std::vector<uint8_t> sdrCommandData = {resrvIDLSB,      resrvIDMSB,
                                               nextRecordIDLSB, nextRecordIDMSB,
                                               sdrCount,        perLoopByte};

        auto method = dbusConnection->new_method_call(
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest");

        method.append(sdrCommandAddress, netfnStorageReq, lun, cmdStorageGetSdr,
                      sdrCommandData);

        auto reply = dbusConnection->call(method);
        if (reply.is_method_error())
        {
            std::cerr << "Error reading from IpmbGetSdr";
        }

        IpmbMethodType resp;
        reply.read(resp);

        std::vector<uint8_t> data;
        data = std::get<5>(resp);

        int dataSize = data.size();
        for (int iData = 0; iData < dataSize; iData++)
        {
            getSdrData.push_back(data.at(iData));
        }

        int iType = getSdrData[5];
        if (iType != sdrType01)
        {
            return;
        }
    }
    int neg = getSdrData[24];
    negRead.push_back(neg);
}

/* This function will convert the SDR data from decimal to readable format */
void IpmbSDR::sdrDataProcess()
{
    uint16_t mData = 0;
    uint16_t bData = 0;
    int8_t bExpVal, rExpVal;
    double thresUpCri;
    double thresLoCri;
    std::string tempName = "";
    int threshold = 0;
    int iStrAddr = 0;
    int iStrLen = 0;
    int isdrType = getSdrData[sdrType];

    nextRecordIDLSB = getSdrData[sdrNxtRecLSB];
    nextRecordIDMSB = getSdrData[sdrNxtRecMSB];

    if (isdrType == sdrType01)
    {
        threshold = (getSdrData[sdrSensCapab] & sdrThresAcce);
        sensCap.push_back(threshold);

        iStrAddr = sdrAdrType01;
        iStrLen = (getSdrData[sdrLenType01] & sdrLenBit);

        sensorUnit.push_back(Sensor_Unit[getSdrData[sdrUnitType01]]);

        mData = ((getSdrData[mTolDataByte] >> bitShiftMsb) << 8) |
                getSdrData[mDataByte];
        bData = ((getSdrData[bAcuDataByte] >> bitShiftMsb) << 8) |
                getSdrData[bDataByte];

        bExpVal = getSdrData[rbExpDataBye] & 0xF;
        if (bExpVal > 7)
        {
            bExpVal = (~bExpVal + 1) & 0xF;
            bExpVal = -bExpVal;
        }
        rExpVal = (getSdrData[rbExpDataBye] >> 4) & 0xF;
        if (rExpVal > 7)
        {
            rExpVal = (~rExpVal + 1) & 0xF;
            rExpVal = -rExpVal;
        }
        thresUpCri = ((mData * getSdrData[sdrUpCriType01]) +
                      (bData * pow(10, bExpVal))) *
                     (pow(10, rExpVal));
        thresLoCri = ((mData * getSdrData[sdrLoCriType01]) +
                      (bData * pow(10, bExpVal))) *
                     (pow(10, rExpVal));

        thresUpperCri.push_back(thresUpCri);
        thresLowerCri.push_back(thresLoCri);
        mValue.push_back(mData);
        bValue.push_back(bData);
        bExp.push_back(bExpVal);
        rExp.push_back(rExpVal);
    }

    for (int iLoop = 0; iLoop < iStrLen; iLoop++)
    {
        tempName += static_cast<char>(getSdrData[iStrAddr]);
        iStrAddr++;
    }

    sensorReadName.push_back(tempName);
    tempName.clear();

    sensorSDRType.push_back(getSdrData[sdrType]);
    sensorNumber.push_back(getSdrData[sdrSenNum]);
    validRecordCount++;
}

/* This function will convert the SDR sensor value based on
 * tolerance and accuracy */
double IpmbSDR::dataConversion(double conValue, uint8_t recCount)
{
    double dataCon;
    dataCon = ((mValue[recCount] * conValue) +
               (bValue[recCount] * pow(10, bExp[recCount]))) *
              (pow(10, rExp[recCount]));

    int twoComp = 128;
    if (dataCon > maxPosReadingMargin)
    {
        // Negative reading handle
        if (twoComp == negRead[recCount])
        {
            dataCon -= thermalConst;
        }
    }
    return dataCon;
}

void IpmbSensor::loadDefaults()
{
    if (type == IpmbType::meSensor)
    {
        commandAddress = meAddress;
        netfn = ipmi::sensor::netFn;
        command = ipmi::sensor::getSensorReading;
        commandData = {deviceAddress};
        readingFormat = ReadingFormat::byte0;
    }
    else if (type == IpmbType::PXE1410CVR)
    {
        commandAddress = meAddress;
        netfn = ipmi::me_bridge::netFn;
        command = ipmi::me_bridge::sendRawPmbus;
        initCommand = ipmi::me_bridge::sendRawPmbus;
        // pmbus read temp
        commandData = {0x57,          0x01, 0x00, 0x16, hostSMbusIndex,
                       deviceAddress, 0x00, 0x00, 0x00, 0x00,
                       0x01,          0x02, 0x8d};
        // goto page 0
        initData = {0x57,          0x01, 0x00, 0x14, hostSMbusIndex,
                    deviceAddress, 0x00, 0x00, 0x00, 0x00,
                    0x02,          0x00, 0x00, 0x00};
        readingFormat = ReadingFormat::linearElevenBit;
    }
    else if (type == IpmbType::IR38363VR)
    {
        commandAddress = meAddress;
        netfn = ipmi::me_bridge::netFn;
        command = ipmi::me_bridge::sendRawPmbus;
        // pmbus read temp
        commandData = {0x57,          0x01, 0x00, 0x16, hostSMbusIndex,
                       deviceAddress, 00,   0x00, 0x00, 0x00,
                       0x01,          0x02, 0x8D};
        readingFormat = ReadingFormat::elevenBitShift;
    }
    else if (type == IpmbType::ADM1278HSC)
    {
        commandAddress = meAddress;
        switch (subType)
        {
            case IpmbSubType::temp:
            case IpmbSubType::curr:
                uint8_t snsNum;
                if (subType == IpmbSubType::temp)
                {
                    snsNum = 0x8d;
                }
                else
                {
                    snsNum = 0x8c;
                }
                netfn = ipmi::me_bridge::netFn;
                command = ipmi::me_bridge::sendRawPmbus;
                commandData = {0x57, 0x01, 0x00, 0x86, deviceAddress,
                               0x00, 0x00, 0x01, 0x02, snsNum};
                readingFormat = ReadingFormat::elevenBit;
                break;
            case IpmbSubType::power:
            case IpmbSubType::volt:
                netfn = ipmi::sensor::netFn;
                command = ipmi::sensor::getSensorReading;
                commandData = {deviceAddress};
                readingFormat = ReadingFormat::byte0;
                break;
            default:
                throw std::runtime_error("Invalid sensor type");
        }
    }
    else if (type == IpmbType::mpsVR)
    {
        commandAddress = meAddress;
        netfn = ipmi::me_bridge::netFn;
        command = ipmi::me_bridge::sendRawPmbus;
        initCommand = ipmi::me_bridge::sendRawPmbus;
        // pmbus read temp
        commandData = {0x57,          0x01, 0x00, 0x16, hostSMbusIndex,
                       deviceAddress, 0x00, 0x00, 0x00, 0x00,
                       0x01,          0x02, 0x8d};
        // goto page 0
        initData = {0x57,          0x01, 0x00, 0x14, hostSMbusIndex,
                    deviceAddress, 0x00, 0x00, 0x00, 0x00,
                    0x02,          0x00, 0x00, 0x00};
        readingFormat = ReadingFormat::byte3;
    }
    else
    {
        throw std::runtime_error("Invalid sensor type");
    }

    if (subType == IpmbSubType::util)
    {
        // Utilization need to be scaled to percent
        maxValue = 100;
        minValue = 0;
    }
}

void IpmbSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

bool IpmbSensor::processReading(const std::vector<uint8_t>& data, double& resp)
{

    switch (readingFormat)
    {
        case (ReadingFormat::byte0):
        {
            if (command == ipmi::sensor::getSensorReading &&
                !ipmi::sensor::isValid(data))
            {
                return false;
            }
            resp = data[0];
            return true;
        }
        case (ReadingFormat::byte3):
        {
            if (data.size() < 4)
            {
                if (!errCount)
                {
                    std::cerr << "Invalid data length returned for " << name
                              << "\n";
                }
                return false;
            }
            resp = data[3];
            return true;
        }
        case (ReadingFormat::elevenBit):
        {
            if (data.size() < 5)
            {
                if (!errCount)
                {
                    std::cerr << "Invalid data length returned for " << name
                              << "\n";
                }
                return false;
            }

            int16_t value = ((data[4] << 8) | data[3]);
            resp = value;
            return true;
        }
        case (ReadingFormat::elevenBitShift):
        {
            if (data.size() < 5)
            {
                if (!errCount)
                {
                    std::cerr << "Invalid data length returned for " << name
                              << "\n";
                }
                return false;
            }

            resp = ((data[4] << 8) | data[3]) >> 3;
            return true;
        }
        case (ReadingFormat::linearElevenBit):
        {
            if (data.size() < 5)
            {
                if (!errCount)
                {
                    std::cerr << "Invalid data length returned for " << name
                              << "\n";
                }
                return false;
            }

            int16_t value = ((data[4] << 8) | data[3]);
            constexpr const size_t shift = 16 - 11; // 11bit into 16bit
            value <<= shift;
            value >>= shift;
            resp = value;
            return true;
        }
        case (ReadingFormat::sdrThres):
        {
            if (!ipmi::sensor::isValid(data))
            {
                return false;
            }
            resp = data[0];
            return true;
        }
        default:
            throw std::runtime_error("Invalid reading type");
    }
}

void IpmbSensor::read(void)
{
    static constexpr size_t pollTime = 1; // in seconds

    waitTimer.expires_from_now(boost::posix_time::seconds(pollTime));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        if (!readingStateGood())
        {
            updateValue(std::numeric_limits<double>::quiet_NaN());
            read();
            return;
        }
        dbusConnection->async_method_call(
            [this](boost::system::error_code ec,
                   const IpmbMethodType& response) {
                const int& status = std::get<0>(response);
                if (ec || status)
                {
                    incrementError();
                    read();
                    return;
                }
                const std::vector<uint8_t>& data = std::get<5>(response);
                if constexpr (debug)
                {
                    std::cout << name << ": ";
                    for (size_t d : data)
                    {
                        std::cout << d << " ";
                    }
                    std::cout << "\n";
                }
                if (data.empty())
                {
                    incrementError();
                    read();
                    return;
                }

                double value = 0;

                if (!processReading(data, value))
                {
                    incrementError();
                    read();
                    return;
                }

                // rawValue only used in debug logging
                // up to 5th byte in data are used to derive value
                size_t end = std::min(sizeof(uint64_t), data.size());
                uint64_t rawData = 0;
                for (size_t i = 0; i < end; i++)
                {
                    reinterpret_cast<uint8_t*>(&rawData)[i] = data[i];
                }
                rawValue = static_cast<double>(rawData);

                /* Adjust value as per scale and offset */
                value = (value * scaleVal) + offsetVal;
                updateValue(value);
                read();
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", commandAddress, netfn, lun, command, commandData);
    });
}

void IpmbSDR::sdrRead(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    std::unique_ptr<IpmbSensor>& sensor)
{
    uint8_t commandAddress = cmdAddr;
    uint8_t netfn = ipmi::sensor::netFn;
    uint8_t command = ipmi::sensor::getSensorReading;
    std::vector<uint8_t> commandData = {devAddr};

    sensor->sensorInterface->set_property("Unit", strUnit);

    auto method = dbusConnection->new_method_call(
        "xyz.openbmc_project.Ipmi.Channel.Ipmb",
        "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
        "sendRequest");

    method.append(commandAddress, netfn, lun, command, commandData);

    auto reply = dbusConnection->call(method);
    if (reply.is_method_error())
    {
        std::cerr << "Error reading from sdrRead";
    }

    IpmbMethodType resp;
    reply.read(resp);

    std::vector<uint8_t> data;
    data = std::get<5>(resp);

    double value = 0;

    if (!(sensor->processReading(data, value)))
    {
        std::cerr << "Reading Unknown Value from " << sensorName << "\n";
        sensor->markFunctional(false);
        return;
    }

    value = dataConversion(value, curRecord);
    /* Adjust value as per scale and offset */
    value = (value * (sensor->scaleVal)) + (sensor->offsetVal);
    sensor->updateValue(value);
}

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<IpmbSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (!dbusConnection)
    {
        std::cerr << "Connection not created\n";
        return;
    }
    dbusConnection->async_method_call(
        [&](boost::system::error_code ec, const ManagedObjectType& resp) {
            if (ec)
            {
                std::cerr << "Error contacting entity manager\n";
                return;
            }
            for (const auto& pathPair : resp)
            {
                for (const auto& entry : pathPair.second)
                {
                    if (entry.first != configInterface)
                    {
                        continue;
                    }
                    std::string name =
                        loadVariant<std::string>(entry.second, "Name");

                    std::vector<thresholds::Threshold> sensorThresholds;
                    if (!parseThresholdsFromConfig(pathPair.second,
                                                   sensorThresholds))
                    {
                        std::cerr << "error populating thresholds for " << name
                                  << "\n";
                    }
                    uint8_t deviceAddress =
                        loadVariant<uint8_t>(entry.second, "Address");

                    std::string sensorClass =
                        loadVariant<std::string>(entry.second, "Class");
                    uint8_t hostSMbusIndex = hostSMbusIndexDefault;
                    auto findSmType = entry.second.find("HostSMbusIndex");
                    if (findSmType != entry.second.end())
                    {
                        hostSMbusIndex = std::visit(
                            VariantToUnsignedIntVisitor(), findSmType->second);
                    }

                    /* Default sensor type is "temperature" */
                    std::string sensorTypeName = "temperature";
                    auto findType = entry.second.find("SensorType");
                    if (findType != entry.second.end())
                    {
                        sensorTypeName = std::visit(VariantToStringVisitor(),
                                                    findType->second);
                    }

                    auto& sensor = sensors[name];
                    sensor = std::make_unique<IpmbSensor>(
                        dbusConnection, io, name, pathPair.first, objectServer,
                        std::move(sensorThresholds), deviceAddress,
                        hostSMbusIndex, sensorTypeName);

                    /* Initialize scale and offset value */
                    sensor->scaleVal = 1;
                    sensor->offsetVal = 0;

                    auto findScaleVal = entry.second.find("ScaleValue");
                    if (findScaleVal != entry.second.end())
                    {
                        sensor->scaleVal = std::visit(VariantToDoubleVisitor(),
                                                      findScaleVal->second);
                    }

                    auto findOffsetVal = entry.second.find("OffsetValue");
                    if (findOffsetVal != entry.second.end())
                    {
                        sensor->offsetVal = std::visit(VariantToDoubleVisitor(),
                                                       findOffsetVal->second);
                    }

                    auto findPowerState = entry.second.find("PowerState");

                    if (findPowerState != entry.second.end())
                    {
                        std::string powerState = std::visit(
                            VariantToStringVisitor(), findPowerState->second);

                        setReadState(powerState, sensor->readState);
                    }

                    if (sensorClass == "PxeBridgeTemp")
                    {
                        sensor->type = IpmbType::PXE1410CVR;
                    }
                    else if (sensorClass == "IRBridgeTemp")
                    {
                        sensor->type = IpmbType::IR38363VR;
                    }
                    else if (sensorClass == "HSCBridge")
                    {
                        sensor->type = IpmbType::ADM1278HSC;
                    }
                    else if (sensorClass == "MpsBridgeTemp")
                    {
                        sensor->type = IpmbType::mpsVR;
                    }
                    else if (sensorClass == "METemp" ||
                             sensorClass == "MESensor")
                    {
                        sensor->type = IpmbType::meSensor;
                    }
                    else
                    {
                        std::cerr << "Invalid class " << sensorClass << "\n";
                        continue;
                    }

                    if (sensorTypeName == "voltage")
                    {
                        sensor->subType = IpmbSubType::volt;
                    }
                    else if (sensorTypeName == "power")
                    {
                        sensor->subType = IpmbSubType::power;
                    }
                    else if (sensorTypeName == "current")
                    {
                        sensor->subType = IpmbSubType::curr;
                    }
                    else if (sensorTypeName == "utilization")
                    {
                        sensor->subType = IpmbSubType::util;
                    }
                    else
                    {
                        sensor->subType = IpmbSubType::temp;
                    }
                    sensor->init();
                }
            }
        },
        entityManagerName, "/", "org.freedesktop.DBus.ObjectManager",
        "GetManagedObjects");
}

/* This function will create object for SDR sensor
 * and call the IpmbSensor service */
void createObj(boost::asio::io_service& io,
               sdbusplus::asio::object_server& objectServer,
               boost::container::flat_map<std::string,
                                          std::unique_ptr<IpmbSensor>>& sensors,
               std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
               IpmbSDR sdr)
{
    static ManagedObjectType managedObj;

    managedObj.clear();
    sdbusplus::message::message getManagedObjects =
        dbusConnection->new_method_call(entityManagerName, "/",
                                        "org.freedesktop.DBus.ObjectManager",
                                        "GetManagedObjects");
    bool err = false;
    try
    {
        sdbusplus::message::message reply =
            dbusConnection->call(getManagedObjects);
        reply.read(managedObj);
    }
    catch (const sdbusplus::exception::exception& e)
    {
        std::cerr << "While calling GetManagedObjects on service:"
                  << entityManagerName << " exception name:" << e.name()
                  << "and description:" << e.description() << " was thrown\n";
        err = true;
    }
    if (err)
    {
        std::cerr << "Error communicating to entity manager\n";
    }
    for (int iCount = 0; iCount < sdr.validRecordCount; iCount++)
    {
        sdr.devAddr = sdr.sensorNumber[iCount];
        sdr.sensorName = sdr.hostName + sdr.sensorReadName[iCount];
        uint8_t hostSMbusIndex = hostSMbusIndexDefault;
        std::string sensorConPath = sensorPathPrefix + sdr.sensorName;
        std::vector<thresholds::Threshold> sensorThresholds;
        std::string sdrSensorTypeName = "SDR_Sensor";

        if (sdr.sensorSDRType[iCount] == sdr.sdrType01 &&
            sdr.sensCap[iCount] != sdr.sdrSensNoThres)
        {
            sensorThresholds.emplace_back(thresholds::Level::CRITICAL,
                                          thresholds::Direction::HIGH,
                                          sdr.thresUpperCri[iCount]);
            sensorThresholds.emplace_back(thresholds::Level::CRITICAL,
                                          thresholds::Direction::LOW,
                                          sdr.thresLowerCri[iCount]);
        }
        auto& sensor = sensors[sdr.sensorName];
        sensor = std::make_unique<IpmbSensor>(
            dbusConnection, io, sdr.sensorName, sensorConPath, objectServer,
            std::move(sensorThresholds), sdr.devAddr, hostSMbusIndex,
            sdrSensorTypeName);

        /* Initialize scale and offset value */
        sensor->scaleVal = 1;
        sensor->offsetVal = 0;
        setReadState("Always", sensor->readState);
        if (sdr.sensorSDRType[iCount] == sdr.sdrType01)
        {
            sensor->type = IpmbType::SDRThresSensor;
            sdr.strUnit = sdr.sensorUnit[iCount];
            sdr.curRecord = iCount;
            sensor->init();
            sdr.sdrRead(dbusConnection, sensor);
        }
    }
}

void reinitSensors(sdbusplus::message::message& message)
{
    constexpr const size_t reinitWaitSeconds = 2;
    std::string objectName;
    boost::container::flat_map<std::string, std::variant<std::string>> values;
    message.read(objectName, values);

    auto findStatus = values.find(power::property);
    if (findStatus != values.end())
    {
        bool powerStatus = boost::ends_with(
            std::get<std::string>(findStatus->second), "Running");
        if (powerStatus)
        {
            if (!initCmdTimer)
            {
                // this should be impossible
                return;
            }
            // we seem to send this command too fast sometimes, wait before
            // sending
            initCmdTimer->expires_from_now(
                boost::posix_time::seconds(reinitWaitSeconds));

            initCmdTimer->async_wait([](const boost::system::error_code ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }

                for (const auto& sensor : sensors)
                {
                    if (sensor.second)
                    {
                        sensor.second->runInitCmd();
                    }
                }
            });
        }
    }
}

int main()
{

    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.IpmbSensor");
    sdbusplus::asio::object_server objectServer(systemBus);

    initCmdTimer = std::make_unique<boost::asio::deadline_timer>(io);

    io.post([&]() { createSensors(io, objectServer, sensors, systemBus); });

    IpmbSDR sdr;
    std::vector<uint8_t> busSize = findObjects(systemBus);
    int size = busSize.size();

    for (int iter = 0; iter < size; iter++)
    {
        sdr.validRecordCount = 0;
        sdr.cmdAddr = busSize[iter] << 2;
        sdr.hostName = std::to_string(busSize[iter] + 1) + "-";

        sdr.ipmbGetSdrInfo(systemBus);
        sdr.ipmbSdrRsrv(systemBus);

        for (int iCnt = 0; iCnt < sdr.recordCount; iCnt++)
        {
            sdr.ipmbGetSdr(systemBus);
            sdr.sdrDataProcess();
        }

        createObj(io, objectServer, sensors, systemBus, sdr);
    }

    boost::asio::deadline_timer configTimer(io);

    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message&) {
            configTimer.expires_from_now(boost::posix_time::seconds(1));
            // create a timer because normally multiple properties change
            configTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }
                createSensors(io, objectServer, sensors, systemBus);
                createObj(io, objectServer, sensors, systemBus, sdr);
                if (sensors.empty())
                {
                    std::cout << "Configuration not detected\n";
                }
            });
        };

    sdbusplus::bus::match::match configMatch(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + configInterface +
            "'",
        eventHandler);

    sdbusplus::bus::match::match powerChangeMatch(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',interface='" + std::string(properties::interface) +
            "',path='" + std::string(power::path) + "',arg0='" +
            std::string(power::interface) + "'",
        reinitSensors);

    io.run();
}
