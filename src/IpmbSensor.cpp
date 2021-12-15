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
static constexpr double ipmbMaxReading = 0xFF;
static constexpr double ipmbMinReading = 0;

static constexpr uint8_t meAddress = 1;
static constexpr uint8_t lun = 0;
static constexpr uint8_t hostSMbusIndexDefault = 0x03;
static constexpr float pollRateDefault = 1; // in seconds

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
                       const float pollRate, std::string& sensorTypeName) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdData), sensorConfiguration,
           "xyz.openbmc_project.Configuration.ExitAirTemp", false,
           ipmbMaxReading, ipmbMinReading, conn, PowerState::on),
    subType(IpmbSubType::temp), commandAddress(meAddress),
    deviceAddress(deviceAddress), hostSMbusIndex(hostSMbusIndex),
    sensorPollMs(static_cast<int>(pollRate * 1000)), objectServer(objectServer),
    waitTimer(io)
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

std::string IpmbSensor::getSubTypeUnits(void)
{
    switch (subType)
    {
        case IpmbSubType::temp:
            return sensor_paths::unitDegreesC;
        case IpmbSubType::curr:
            return sensor_paths::unitAmperes;
        case IpmbSubType::power:
            return sensor_paths::unitWatts;
        case IpmbSubType::volt:
            return sensor_paths::unitVolts;
        case IpmbSubType::util:
            return sensor_paths::unitPercent;
        default:
            throw std::runtime_error("Invalid sensor type");
    }
}

void IpmbSensor::init(void)
{
    loadDefaults();
    setReadFunction();
    setInitialProperties(dbusConnection, getSubTypeUnits());
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

/**
 * Refernce:
 * Intelligent Power Node Manager External Interface Specification
 */
std::vector<uint8_t> IpmbSensor::getMeCommand()
{
    std::vector<uint8_t> commandBytes;

    /*
     * Byte 1, 2, 3 = Manufacturer ID.
     */
    commandBytes.emplace_back(ipmi::sensor::manufacturerId[0]);
    commandBytes.emplace_back(ipmi::sensor::manufacturerId[1]);
    commandBytes.emplace_back(ipmi::sensor::manufacturerId[2]);

    /*
     * Byte 4 = Device Index.
     */
    commandBytes.emplace_back(deviceIndex);

    /*
     * Byte 5 = History index.
     *   bit 0 to 3 = History index. Supported value: 0Fh to retrieve
     *      current samples.
     *   bit 4 to 7 = Page number – used only for devices which support
     *      pages.
     */
    commandBytes.emplace_back(0x0F);

    /*
     * Byte 6 = First Register Offset.
     */
    commandBytes.emplace_back(0x00);

    return commandBytes;
}

/**
 * Refernce:
 * Intelligent Power Node Manager External Interface Specification
 */
std::vector<uint8_t> IpmbSensor::getRawPmbusCommand(
    uint8_t messageType, const std::vector<uint8_t>& pmbusCommand,
    uint8_t readLength, bool isExtendedDeviceAddress = true,
    bool doEnablePec = false, bool doReportPecErrors = false)
{
    std::vector<uint8_t> commandBytes;

    /*
     * Byte 1, 2, 3 = Manufacturer ID.
     */
    commandBytes.emplace_back(ipmi::sensor::manufacturerId[0]);
    commandBytes.emplace_back(ipmi::sensor::manufacturerId[1]);
    commandBytes.emplace_back(ipmi::sensor::manufacturerId[2]);

    /*
     * Byte 4
     *   bit 0 = Reserved.
     *   bit 1, 2, 3 = SMBUS message transaction type.
     *   bit 4, 5 = Device address format.
     *       0 for Standard device address
     *       1 for Extended device address
     *   bit 6 = 1 means "Do not report PEC errors in Completion Code".
     *   bit 7 = 1 means "Enable PEC".
     */
    uint8_t byte4 = 0x00;
    if (isExtendedDeviceAddress)
    {
        byte4 |= (1 << 4);
    }

    if (doReportPecErrors)
    {
        byte4 |= (1 << 6);
    }

    if (doEnablePec)
    {
        byte4 |= (1 << 7);
    }

    byte4 |= messageType << 1;

    commandBytes.emplace_back(byte4);

    if (isExtendedDeviceAddress)
    {
        /*
         * Byte 5 = Sensor Bus.
         *    00 - SMBUS
         *    01 - SMLINK0/SMLINK0B
         *    02 - SMLINK1
         *    03 - SMLINK2
         *    04 - SMLINK3
         *    05 - SMLINK4
         */
        commandBytes.emplace_back(hostSMbusIndex);

        /*
         * Byte 6 = Target PSU Address.
         *    bit 0 is Reserved.
         *    bit 1 to 7 is 7-bit SMBUS address
         */
        commandBytes.emplace_back(deviceAddress);

        /*
         * Byte 7 = MUX Address.
         */
        commandBytes.emplace_back(0x00);

        /*
         * Byte 8 = MUX channel selection.
         */
        commandBytes.emplace_back(0x00);

        /*
         * Byte 8 = MUX configuration state.
         */
        commandBytes.emplace_back(0x00);
    }
    else
    {
        /*
         * Byte 6 = Target PSU Address.
         *    bit 0 is Reserved.
         *    bit 1 to 7 is 7-bit SMBUS address
         */
        commandBytes.emplace_back(deviceAddress);

        /*
         * Byte 8 = MGPIO MUX configuration.
         */
        commandBytes.emplace_back(0x00);
    }

    /*
     * Byte 7 or 10 = Transmission Protocol parameter
     *   bit 0, 4 = Reserved.
     *   bit 5 = Transmission Protocol (0 for PMBus 1 for I2C).
     *   bit 6, 7 = Reserved.
     */
    commandBytes.emplace_back(0x00);

    /*
     * Byte 8 or 11 = Write Length.
     */
    commandBytes.emplace_back(static_cast<uint8_t>(pmbusCommand.size()));

    /*
     * Byte 9 or 12 = Read Length.
     */
    commandBytes.emplace_back(readLength);

    /*
     * Byte Byte 10 or 13 to M = PMBUS command.
     */
    for (uint8_t byte : pmbusCommand)
    {
        commandBytes.emplace_back(byte);
    }

    return commandBytes;
}

void IpmbSensor::loadDefaults()
{
    if (type == IpmbType::meSensor)
    {
        netfn = ipmi::sensor::netFn;
        command = ipmi::sensor::getSensorReading;
        commandData = {deviceAddress};
        readingFormat = ReadingFormat::byte0;
    }
    else if (type == IpmbType::PXE1410CVR)
    {
        netfn = ipmi::me_bridge::netFn;
        readingFormat = ReadingFormat::linearElevenBit;
        if (isProxyRead)
        {
            command = ipmi::me_bridge::sendRawPmbus;
            initCommand = ipmi::me_bridge::sendRawPmbus;
            commandData = getRawPmbusCommand(
                ipmi::sensor::readWord, {ipmi::sensor::readTemperature}, 0x02);
            initData =
                getRawPmbusCommand(ipmi::sensor::writeByte, {0x00, 0x00}, 0x00);
        }
        else
        {
            command = ipmi::sensor::read_me::getPmbusReadings;
            commandData = getMeCommand();
        }
    }
    else if (type == IpmbType::IR38363VR)
    {
        netfn = ipmi::me_bridge::netFn;
        command = ipmi::me_bridge::sendRawPmbus;
        commandData = getRawPmbusCommand(ipmi::sensor::readWord,
                                         {ipmi::sensor::readTemperature}, 0x02);
        readingFormat = ReadingFormat::elevenBitShift;
    }
    else if (type == IpmbType::ADM1278HSC)
    {
        switch (subType)
        {
            case IpmbSubType::temp:
            case IpmbSubType::curr:
                netfn = ipmi::me_bridge::netFn;
                command = ipmi::me_bridge::sendRawPmbus;
                if (IpmbSubType::temp == subType)
                {
                    commandData = getRawPmbusCommand(
                        ipmi::sensor::readWord, {ipmi::sensor::readTemperature},
                        0x02, false, true);
                }
                else
                {
                    commandData = getRawPmbusCommand(
                        ipmi::sensor::readWord,
                        {ipmi::sensor::readCurrentOutput}, 0x02, false, true);
                }
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
        netfn = ipmi::me_bridge::netFn;
        readingFormat = ReadingFormat::byte3;
        if (isProxyRead)
        {
            command = ipmi::me_bridge::sendRawPmbus;
            initCommand = ipmi::me_bridge::sendRawPmbus;
            commandData = getRawPmbusCommand(
                ipmi::sensor::readWord, {ipmi::sensor::readTemperature}, 0x02);
            initData =
                getRawPmbusCommand(ipmi::sensor::writeByte, {0x00, 0x00}, 0x00);
        }
        else
        {
            command = ipmi::sensor::read_me::getPmbusReadings;
            commandData = getMeCommand();
        }
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

void IpmbSensor::setReadFunction()
{
    switch (readingFormat)
    {
        case (ReadingFormat::byte0):
            readFunction = [](const std::vector<uint8_t>& data,
                              double& resp) -> bool {
                if (!ipmi::sensor::isValid(data))
                {
                    return false;
                }
                resp = data[0];
                return true;
            };
            break;
        case (ReadingFormat::byte3):
            readFunction = [](const std::vector<uint8_t>& data,
                              double& resp) -> bool {
                if (data.size() < 4)
                {
                    return false;
                }
                resp = data[3];
                return true;
            };
            break;
        case (ReadingFormat::elevenBit):
            readFunction = [](const std::vector<uint8_t>& data,
                              double& resp) -> bool {
                if (data.size() < 5)
                {
                    return false;
                }

                int16_t value = ((data[4] << 8) | data[3]);
                resp = value;
                return true;
            };
            break;
        case (ReadingFormat::elevenBitShift):
            readFunction = [](const std::vector<uint8_t>& data,
                              double& resp) -> bool {
                if (data.size() < 5)
                {
                    return false;
                }

                resp = ((data[4] << 8) | data[3]) >> 3;
                return true;
            };
            break;
        case (ReadingFormat::linearElevenBit):
            readFunction = [](const std::vector<uint8_t>& data,
                              double& resp) -> bool {
                if (data.size() < 5)
                {
                    return false;
                }

                int16_t value = ((data[4] << 8) | data[3]);
                constexpr const size_t shift = 16 - 11; // 11bit into 16bit
                value <<= shift;
                value >>= shift;
                resp = value;
                return true;
            };
            break;
        default:
            throw std::runtime_error("Invalid reading type");
    }
}

void IpmbSensor::read(void)
{
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
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

                double value = 0;
                std::vector<uint8_t> data;

                if (isProxyRead)
                {
                    data = std::get<5>(response);
                }
                else
                {
                    ipmi::sensor::read_me::getRawData(
                        registerToRead, std::get<5>(response), data);
                }

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

                if (!readFunction(data, value))
                {
                    if (!errCount)
                    {
                        std::cerr << "readFunction failed for " << name << "\n";
                    }

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

bool IpmbSensor::setSensorType(const std::string& sensorClass)
{
    if (sensorClass == "PxeBridgeTemp")
    {
        type = IpmbType::PXE1410CVR;
    }
    else if (sensorClass == "IRBridgeTemp")
    {
        type = IpmbType::IR38363VR;
    }
    else if (sensorClass == "HSCBridge")
    {
        type = IpmbType::ADM1278HSC;
    }
    else if (sensorClass == "MpsBridgeTemp")
    {
        type = IpmbType::mpsVR;
    }
    else if (sensorClass == "METemp" || sensorClass == "MESensor")
    {
        type = IpmbType::meSensor;
    }
    else
    {
        return false;
    }

    return true;
}

void IpmbSensor::setSensorSubType(const std::string& sensorTypeName)
{
    if (sensorTypeName == "voltage")
    {
        subType = IpmbSubType::volt;
    }
    else if (sensorTypeName == "power")
    {
        subType = IpmbSubType::power;
    }
    else if (sensorTypeName == "current")
    {
        subType = IpmbSubType::curr;
    }
    else if (sensorTypeName == "utilization")
    {
        subType = IpmbSubType::util;
    }
}

void IpmbSensor::setScaleAndOffset(const SensorBaseConfigMap& sensorBaseConfig)
{
    auto findScaleVal = sensorBaseConfig.find("ScaleValue");
    if (findScaleVal != sensorBaseConfig.end())
    {
        scaleVal = std::visit(VariantToDoubleVisitor(), findScaleVal->second);
    }

    auto findOffsetVal = sensorBaseConfig.find("OffsetValue");
    if (findOffsetVal != sensorBaseConfig.end())
    {
        offsetVal = std::visit(VariantToDoubleVisitor(), findOffsetVal->second);
    }

    auto findPowerState = sensorBaseConfig.find("PowerState");
    if (findPowerState != sensorBaseConfig.end())
    {
        std::string powerState =
            std::visit(VariantToStringVisitor(), findPowerState->second);

        setReadState(powerState, readState);
    }
}

void IpmbSensor::setReadMethod(const SensorBaseConfigMap& sensorBaseConfig)
{
    /*
     * Some sensor can be read in two ways
     * 1) Using proxy: BMC read command is proxy forward by ME
     * to sensor. 2) Using 'Get PMBUS Readings': ME responds to
     * BMC with sensor data.
     *
     * By default we assume the method is 1. And if ReadMethod
     * == "IPMI" we switch to method 2.
     */
    auto readMethod = sensorBaseConfig.find("ReadMethod");
    if (readMethod == sensorBaseConfig.end())
    {
        std::cerr << "'ReadMethod' not found, defaulting to "
                     "proxy method of reading sensor\n";
        return;
    }

    if (std::visit(VariantToStringVisitor(), readMethod->second) != "ReadME")
    {
        std::cerr << "'ReadMethod' != 'ReadME', defaulting to "
                     "proxy method of reading sensor\n";
        return;
    }

    /*
     * In 'Get PMBUS Readings' the response containt a
     * set of registers from the sensor. And different
     * values such as temperature power voltage will be
     * mapped to different registers.
     */
    auto registerToReadConfig = sensorBaseConfig.find("Register");
    if (registerToReadConfig == sensorBaseConfig.end())
    {
        std::cerr << "'Register' not found, defaulting to "
                     "proxy method of reading sensor\n";
        return;
    }

    registerToRead =
        std::visit(VariantToUnsignedIntVisitor(), registerToReadConfig->second);

    /*
     * In 'Get PMBUS Readings' since ME is
     * responding with the sensor data we need
     * to use the address for sensor in ME, this
     * is different from the actual sensor
     * address.
     */
    auto deviceIndexConfig = sensorBaseConfig.find("DeviceIndex");
    if (deviceIndexConfig == sensorBaseConfig.end())
    {
        std::cerr << "'DeviceIndex' not found, defaulting to "
                     "proxy method of reading sensor\n";
        return;
    }

    deviceIndex =
        std::visit(VariantToUnsignedIntVisitor(), deviceIndexConfig->second);

    /*
     * We found all parameters to use 'Get PMBUS Readings'
     * method.
     */
    isProxyRead = false;
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

                    float pollRate = pollRateDefault;
                    auto findPollRate = entry.second.find("PollRate");
                    if (findPollRate != entry.second.end())
                    {
                        pollRate = std::visit(VariantToFloatVisitor(),
                                              findPollRate->second);
                        if (pollRate <= 0.0f)
                        {
                            pollRate = pollRateDefault;
                        }
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
                        hostSMbusIndex, pollRate, sensorTypeName);

                    sensor->setReadMethod(entry.second);
                    sensor->setScaleAndOffset(entry.second);
                    sensor->setSensorType(sensorClass);
                    sensor->setSensorSubType(sensorTypeName);

                    sensor->init();
                }
            }
        },
        entityManagerName, "/", "org.freedesktop.DBus.ObjectManager",
        "GetManagedObjects");
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
            std::get<std::string>(findStatus->second), ".Running");
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

    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
