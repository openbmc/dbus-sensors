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

#include "IpmbSensor.hpp"

#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <math.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <chrono>
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
static constexpr double ipmbMaxReading = 0xFF;
static constexpr double ipmbMinReading = 0;

static constexpr uint8_t meAddress = 1;
static constexpr uint8_t lun = 0;
static constexpr uint8_t hostSMbusIndexDefault = 0x03;

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";
static constexpr const char* statusPathPrefix =
    "/xyz/openbmc_project/software/";

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
    if (sensorTypeName == "SDR_Type_02")
    {
        std::string dbusPath = statusPathPrefix + sensorTypeName + "/" + name;

        sensorInterface = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Software.State");
    }
    else if (sensorTypeName == "SDR_Type_03")
    {
        std::string dbusPath = statusPathPrefix + sensorTypeName + "/" + name;

        sensorInterface = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Software.Events");
    }
    else
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
        association =
            objectServer.add_interface(dbusPath, association::interface);
    }
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
    loadDefaults();
    if (sdr::sensorTypeName == "SDR_Type_01")
    {
        sensorInterface->register_property(
            "Unit", std::string(""),
            sdbusplus::asio::PropertyPermission::readWrite);
        sensorInterface->register_property(
            "Thres_UpperCritical", uint8_t(0),
            sdbusplus::asio::PropertyPermission::readWrite);
        sensorInterface->register_property(
            "Thres_LowerCritical", uint8_t(0),
            sdbusplus::asio::PropertyPermission::readWrite);
        setInitialProperties(dbusConnection);
        sensorInterface->set_property("Unit", sdr::strUnit);
        sensorInterface->set_property("Thres_UpperCritical", sdr::upperCri);
        sensorInterface->set_property("Thres_LowerCritical", sdr::lowerCri);
    }
    else if (sdr::sensorTypeName == "SDR_Type_02")
    {
        sensorInterface->register_property(
            "State", uint8_t(0),
            sdbusplus::asio::PropertyPermission::readWrite);
        if (!sensorInterface->initialize())
        {
            std::cerr << "error initializing value interface\n";
        }
    }
    else if (sdr::sensorTypeName == "SDR_Type_03")
    {
        sensorInterface->register_property(
            "Events", uint8_t(0),
            sdbusplus::asio::PropertyPermission::readWrite);
        if (!sensorInterface->initialize())
        {
            std::cerr << "error initializing value interface\n";
        }
    }
    else
    {
        setInitialProperties(dbusConnection);
    }

    if (initCommand)
    {
        runInitCmd();
    }

    if ((sdr::sensorTypeName == "SDR_Type_01") ||
        (sdr::sensorTypeName == "SDR_Type_02") ||
        (sdr::sensorTypeName == "SDR_Type_03"))
    {
        sdrRead();
    }
    else
    {
        read();
    }
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

void findObjects(std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    const char* objPath;
    std::vector<std::string> paths;
    auto method = dbusConnection->new_method_call(
        mapper::busName, mapper::path, mapper::interface, "GetSubTreePaths");
    method.append(fru::path);
    method.append(0); // Depth 0 to search all
    method.append(std::vector<std::string>({fru::interface}));
    auto reply = dbusConnection->call(method);
    reply.read(paths);

    for (int iPath = 0; iPath < paths.size(); iPath++)
    {
        objPath = paths.at(iPath).c_str();
        auto method = dbusConnection->new_method_call(
            fru::busname, objPath, properties::interface, properties::get);
        method.append(fru::interface, "BUS");
        auto reply = dbusConnection->call(method);

        std::variant<uint32_t> resp;
        reply.read(resp);

        sdr::ipmbBus.push_back(std::get<uint32_t>(resp));
    }
}

void sdr::ipmbGetSdrInfo(std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    sdrCommandAddress = cmdAddr;
    sdrNetfn = netfnStorageReq;
    sdrCommand = cmdStorageGetSdrInfo;
    sdrCommandData = {};

    auto method =
        dbusConnection->new_method_call("xyz.openbmc_project.Ipmi.Channel.Ipmb",
                                   "/xyz/openbmc_project/Ipmi/Channel/Ipmb",
                                   "org.openbmc.Ipmb", "sendRequest");

    method.append(sdrCommandAddress, sdrNetfn, lun, sdrCommand, sdrCommandData);

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

    recordCount = (((data.at(2) << 8) & 0xFF00) | data.at(1));
}

void sdr::ipmbSdrRsrv(std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    sdrCommandAddress = cmdAddr;
    sdrNetfn = netfnStorageReq;
    sdrCommand = cmdStorageRsrvSdr;
    sdrCommandData = {};

    auto method =
        dbusConnection->new_method_call("xyz.openbmc_project.Ipmi.Channel.Ipmb",
                                   "/xyz/openbmc_project/Ipmi/Channel/Ipmb",
                                   "org.openbmc.Ipmb", "sendRequest");

    method.append(sdrCommandAddress, sdrNetfn, lun, sdrCommand, sdrCommandData);

    auto reply = dbusConnection->call(method);
    if (reply.is_method_error())
    {
        std::cerr << "Error reading from IpmbSdrRsrv";
    }

    IpmbMethodType resp;
    reply.read(resp);

    std::vector<uint8_t> data;
    data = std::get<5>(resp);

    resrvIDLSB = data.at(0);
    resrvIDMSB = data.at(1);
}

void sdr::ipmbGetSdr(std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    getSdrData.clear();
    int iLoop = cntType01;
    for (int iCnt = 0; iCnt < iLoop; iCnt++)
    {
        sdrCommandAddress = cmdAddr;
        sdrNetfn = netfnStorageReq;
        sdrCommand = cmdStorageGetSdr;
        uint8_t sdrCount = perLoopByte * iCnt;
        sdrCommandData = {resrvIDLSB,      resrvIDMSB, nextRecordIDLSB,
                          nextRecordIDMSB, sdrCount,   perLoopByte};

        auto method =
            dbusConnection->new_method_call("xyz.openbmc_project.Ipmi.Channel.Ipmb",
                                       "/xyz/openbmc_project/Ipmi/Channel/Ipmb",
                                       "org.openbmc.Ipmb", "sendRequest");

        method.append(sdrCommandAddress, sdrNetfn, lun, sdrCommand,
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

        for (int iData = 0; iData < data.size(); iData++)
        {
            getSdrData.push_back(data.at(iData));
        }
        int iType = getSdrData[5];
        if (iType == sdrType02)
        {
            iLoop = cntType02;
        }
        else if (iType == sdrType03)
        {
            iLoop = cntType03;
        }
    }
}

void sdr::sdrDataProcess()
{
    std::string tempName = "";
    int iStrAddr = 0;
    int iStrLen = 0;
    int isdrType = getSdrData[sdrType];

    nextRecordIDLSB = getSdrData[sdrNxtRecLSB];
    nextRecordIDMSB = getSdrData[sdrNxtRecMSB];

    sensorSDRType.push_back(getSdrData[sdrType]);
    sensorNumber.push_back(getSdrData[sdrSenNum]);

    if (isdrType == sdrType01)
    {
        iStrAddr = sdrAdrType01;
        iStrLen = (getSdrData[sdrLenType01] & sdrLenBit);

        sensorSDREvent.push_back(getSdrData[sdrEveType01]);
        sensorUnit.push_back(Sensor_Unit[getSdrData[sdrUnitType01]]);

        thresUpperCri.push_back(getSdrData[sdrUpCriType01]);
        thresLowerCri.push_back(getSdrData[sdrLoCriType01]);
    }
    else if (isdrType == sdrType02)
    {
        iStrAddr = sdrAdrType02;
        iStrLen = (getSdrData[sdrLenType02] & sdrLenBit);
        sensorSDREvent.push_back(getSdrData[sdrEveType02]);
    }
    else if (isdrType == sdrType03)
    {
        iStrAddr = sdrAdrType03;
        iStrLen = (getSdrData[sdrLenType03] & sdrLenBit);
        sensorSDREvent.push_back(getSdrData[sdrEveType03]);
    }
    for (int iLoop = 0; iLoop < iStrLen; iLoop++)
    {
        tempName = tempName + static_cast<char>(getSdrData[iStrAddr]);
        iStrAddr++;
    }
    sensorReadName.push_back(tempName);
    tempName.clear();
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
        readingFormat = ReadingFormat::elevenBit;
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
                    snsNum = 0x8d;
                else
                    snsNum = 0x8c;
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
    else if (type == IpmbType::SDRType)
    {
        commandAddress = sdr::cmdAddr;
        netfn = ipmi::sensor::netFn;
        command = ipmi::sensor::getSensorReading;
        commandData = {sdr::dev_addr};
        readingFormat = ReadingFormat::sdrTyp;
    }
    else if (type == IpmbType::SDRStEvtType)
    {
        commandAddress = sdr::cmdAddr;
        netfn = ipmi::sensor::netFn;
        command = ipmi::sensor::getSensorReading;
        commandData = {sdr::dev_addr};
        readingFormat = ReadingFormat::sdrStEvt;
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
            constexpr const size_t shift = 16 - 11; // 11bit into 16bit
            value <<= shift;
            value >>= shift;
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
        case (ReadingFormat::sdrTyp):
        {
            if (command == ipmi::sensor::getSensorReading &&
                !ipmi::sensor::isValid(data))
            {
                return false;
            }
            resp = data[0];
            return true;
        }
        case (ReadingFormat::sdrStEvt):
        {
            if (command == ipmi::sensor::getSensorReading &&
                !ipmi::sensor::isValid(data))
            {
                return false;
            }
            if (sdr::sensorTypeName == "SDR_Type_02")
            {
                sensorInterface->set_property("State", data[2]);
            }
            else if (sdr::sensorTypeName == "SDR_Type_03")
            {
                sensorInterface->set_property("Events", data[2]);
            }

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
                else
                {
                    // rawValue only used in debug logging
                    // up to 5th byte in data are used to derive value
                    size_t end = std::min(sizeof(uint64_t), data.size());
                    uint64_t rawData = 0;
                    for (size_t i = 0; i < end; i++)
                    {
                        reinterpret_cast<uint8_t*>(&rawData)[i] = data[i];
                    }
                    rawValue = static_cast<double>(rawData);
                }

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

void IpmbSensor::sdrRead(void)
{
    auto method =
        dbusConnection->new_method_call("xyz.openbmc_project.Ipmi.Channel.Ipmb",
                                   "/xyz/openbmc_project/Ipmi/Channel/Ipmb",
                                   "org.openbmc.Ipmb", "sendRequest");

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

    if (!processReading(data, value))
    {
        std::cerr << "Reading Unknown Value from " << sdr::sensorName << "\n";
    }
    if (readingFormat != ReadingFormat::sdrStEvt)
    {
        /* Adjust value as per scale and offset */
        value = (value * scaleVal) + offsetVal;
        updateValue(value);
    }
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

void createObj(boost::asio::io_service& io,
               sdbusplus::asio::object_server& objectServer,
               boost::container::flat_map<std::string,
                                          std::unique_ptr<IpmbSensor>>& sensors,
               std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
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
    for (int i = 0; i < sdr::recordCount; i++)
    {
        sdr::dev_addr = sdr::sensorNumber[i];
        sdr::sensorName = sdr::hostName + sdr::sensorReadName[i];

        if (sdr::sensorSDRType[i] == sdr::sdrType01)
        {
            sdr::sensorTypeName = "SDR_Type_01";
        }
        else if (sdr::sensorSDRType[i] == sdr::sdrType02)
        {
            sdr::sensorTypeName = "SDR_Type_02";
        }
        else if (sdr::sensorSDRType[i] == sdr::sdrType03)
        {
            sdr::sensorTypeName = "SDR_Type_03";
        }

        std::vector<thresholds::Threshold> sensorThresholds;
        uint8_t hostSMbusIndex = hostSMbusIndexDefault;

        auto& sensor = sensors[sdr::sensorName];
        sensor = std::make_unique<IpmbSensor>(
            dbusConnection, io, sdr::sensorName, sdr::sensorName, objectServer,
            std::move(sensorThresholds), sdr::dev_addr, hostSMbusIndex,
            sdr::sensorTypeName);

        /* Initialize scale and offset value */
        sensor->scaleVal = 1;
        sensor->offsetVal = 0;
        setReadState("Always", sensor->readState);
        if (sdr::sensorTypeName == "SDR_Type_01")
        {
            sensor->type = IpmbType::SDRType;
            sdr::strUnit = sdr::sensorUnit[i];
            sdr::upperCri = sdr::thresUpperCri[i];
            sdr::lowerCri = sdr::thresLowerCri[i];

            sensor->init();
        }
        else
        {
            sensor->type = IpmbType::SDRStEvtType;
            sensor->init();
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

    findObjects(systemBus);
    for (int iter = 0; iter < sdr::ipmbBus.size(); iter++)
    {
        sdr::cmdAddr = sdr::ipmbBus.at(iter) << 2;
        sdr::hostName = "Host-" + std::to_string(sdr::ipmbBus.at(iter)) + "-";

        sdr::ipmbGetSdrInfo(systemBus);
        sdr::ipmbSdrRsrv(systemBus);

        for (int iCnt = 0; iCnt < sdr::recordCount; iCnt++)
        {
            sdr::ipmbGetSdr(systemBus);
            sdr::sdrDataProcess();
        }

        createObj(io, objectServer, sensors, systemBus);
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
                createObj(io, objectServer, sensors, systemBus);
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
