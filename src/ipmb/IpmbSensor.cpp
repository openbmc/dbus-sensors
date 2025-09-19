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

#include "IpmbSDRSensor.hpp"
#include "asio/Thresholds.hpp"
#include "asio/sensor.hpp"
#include "utils/SensorPaths.hpp"
#include "utils/Utils.hpp"
#include "utils/VariantVisitors.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

static constexpr double ipmbMaxReading = 0xFF;
static constexpr double ipmbMinReading = 0;

static constexpr uint8_t meAddress = 1;
static constexpr uint8_t lun = 0;
static constexpr uint8_t hostSMbusIndexDefault = 0x03;
static constexpr uint8_t ipmbBusIndexDefault = 0;
static constexpr float pollRateDefault = 1; // in seconds

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

IpmbSensor::IpmbSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    const std::string& sensorConfiguration,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData, uint8_t deviceAddress,
    uint8_t hostSMbusIndex, const float pollRate, std::string& sensorTypeName) :
    Sensor(escapeName(sensorName), std::move(thresholdData),
           sensorConfiguration, "IpmbSensor", false, false, ipmbMaxReading,
           ipmbMinReading, conn, PowerState::on),
    deviceAddress(deviceAddress), hostSMbusIndex(hostSMbusIndex),
    sensorPollMs(static_cast<int>(pollRate * 1000)), objectServer(objectServer),
    waitTimer(io)
{
    std::string dbusPath = sensorPathPrefix + sensorTypeName + "/" + name;

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(dbusPath, interface);
    }
    association = objectServer.add_interface(dbusPath, association::interface);
}

IpmbSensor::~IpmbSensor()
{
    waitTimer.cancel();
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

std::string IpmbSensor::getSubTypeUnits() const
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

void IpmbSensor::init()
{
    loadDefaults();
    setInitialProperties(getSubTypeUnits());
    runInitCmd();
    read();
}

static void initCmdCb(const std::weak_ptr<IpmbSensor>& weakRef,
                      const boost::system::error_code& ec,
                      const IpmbMethodType& response)
{
    std::shared_ptr<IpmbSensor> self = weakRef.lock();
    if (!self)
    {
        return;
    }
    const int& status = std::get<0>(response);
    if (ec || (status != 0))
    {
        lg2::error("Error setting init command for device: '{NAME}'", "NAME",
                   self->name);
    }
}

void IpmbSensor::runInitCmd()
{
    if (!initCommand.has_value())
    {
        return;
    }
    dbusConnection->async_method_call(
        [weakRef{weak_from_this()}](const boost::system::error_code& ec,
                                    const IpmbMethodType& response) {
            initCmdCb(weakRef, ec, response);
        },
        "xyz.openbmc_project.Ipmi.Channel.Ipmb",
        "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
        "sendRequest", commandAddress, netfn, lun, initCommand.value_or(0),
        initData);
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
        uint8_t snsNum = 0;
        switch (subType)
        {
            case IpmbSubType::temp:
            case IpmbSubType::curr:
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
    else if (type == IpmbType::SMPro)
    {
        // This is an Ampere SMPro reachable via a BMC.  For example,
        // this architecture is used on ADLINK Ampere Altra systems.
        // See the Ampere Family SoC BMC Interface Specification at
        // https://amperecomputing.com/customer-connect/products/altra-family-software---firmware
        // for details of the sensors.
        commandAddress = 0;
        netfn = 0x30;
        command = 0x31;
        commandData = {0x9e, deviceAddress};
        switch (subType)
        {
            case IpmbSubType::temp:
                readingFormat = ReadingFormat::nineBit;
                break;
            case IpmbSubType::power:
                readingFormat = ReadingFormat::tenBit;
                break;
            case IpmbSubType::curr:
            case IpmbSubType::volt:
                readingFormat = ReadingFormat::fifteenBit;
                break;
            default:
                throw std::runtime_error("Invalid sensor type");
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

void IpmbSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

bool IpmbSensor::processReading(ReadingFormat readingFormat, uint8_t command,
                                const std::vector<uint8_t>& data, double& resp,
                                size_t errCount)
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
                if (errCount == 0U)
                {
                    lg2::error("Invalid data length returned");
                }
                return false;
            }
            resp = data[3];
            return true;
        }
        case (ReadingFormat::nineBit):
        case (ReadingFormat::tenBit):
        case (ReadingFormat::fifteenBit):
        {
            if (data.size() != 2)
            {
                if (errCount == 0U)
                {
                    lg2::error("Invalid data length returned");
                }
                return false;
            }

            // From the Altra Family SoC BMC Interface Specification:
            // 0xFFFF – This sensor data is either missing or is not supported
            // by the device.
            if ((data[0] == 0xff) && (data[1] == 0xff))
            {
                return false;
            }

            if (readingFormat == ReadingFormat::nineBit)
            {
                int16_t value = data[0];
                if ((data[1] & 0x1) != 0)
                {
                    // Sign extend to 16 bits
                    value |= 0xFF00;
                }
                resp = value;
            }
            else if (readingFormat == ReadingFormat::tenBit)
            {
                uint16_t value = ((data[1] & 0x3) << 8) + data[0];
                resp = value;
            }
            else if (readingFormat == ReadingFormat::fifteenBit)
            {
                uint16_t value = ((data[1] & 0x7F) << 8) + data[0];
                // Convert mV to V
                resp = value / 1000.0;
            }

            return true;
        }
        case (ReadingFormat::elevenBit):
        {
            if (data.size() < 5)
            {
                if (errCount == 0U)
                {
                    lg2::error("Invalid data length returned");
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
                if (errCount == 0U)
                {
                    lg2::error("Invalid data length returned");
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
                if (errCount == 0U)
                {
                    lg2::error("Invalid data length returned");
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
        default:
            throw std::runtime_error("Invalid reading type");
    }
}

void IpmbSensor::ipmbRequestCompletionCb(const boost::system::error_code& ec,
                                         const IpmbMethodType& response)
{
    const int& status = std::get<0>(response);
    if (ec || (status != 0))
    {
        incrementError();
        read();
        return;
    }
    const std::vector<uint8_t>& data = std::get<5>(response);

    std::ostringstream tempStream;
    for (int d : data)
    {
        tempStream << std::setfill('0') << std::setw(2) << std::hex << d << " ";
    }
    lg2::debug("'{NAME}': '{DATA}'", "NAME", name, "DATA", tempStream.str());

    if (data.empty())
    {
        incrementError();
        read();
        return;
    }

    double value = 0;

    if (!processReading(readingFormat, command, data, value, errCount))
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
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<uint8_t*>(&rawData)[i] = data[i];
    }
    rawValue = static_cast<double>(rawData);

    /* Adjust value as per scale and offset */
    value = (value * scaleVal) + offsetVal;
    updateValue(value);
    read();
}

void IpmbSensor::read()
{
    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait(
        [weakRef{weak_from_this()}](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }
            std::shared_ptr<IpmbSensor> self = weakRef.lock();
            if (!self)
            {
                return;
            }
            self->sendIpmbRequest();
        });
}

void IpmbSensor::sendIpmbRequest()
{
    if (!readingStateGood())
    {
        updateValue(std::numeric_limits<double>::quiet_NaN());
        read();
        return;
    }
    dbusConnection->async_method_call(
        [weakRef{weak_from_this()}](boost::system::error_code ec,
                                    const IpmbMethodType& response) {
            std::shared_ptr<IpmbSensor> self = weakRef.lock();
            if (!self)
            {
                return;
            }
            self->ipmbRequestCompletionCb(ec, response);
        },
        "xyz.openbmc_project.Ipmi.Channel.Ipmb",
        "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
        "sendRequest", commandAddress, netfn, lun, command, commandData);
}

bool IpmbSensor::sensorClassType(const std::string& sensorClass)
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
    else if (sensorClass == "SMPro")
    {
        type = IpmbType::SMPro;
    }
    else
    {
        lg2::error("Invalid class '{SENSOR}'", "SENSOR", sensorClass);
        return false;
    }
    return true;
}

void IpmbSensor::sensorSubType(const std::string& sensorTypeName)
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
    else
    {
        subType = IpmbSubType::temp;
    }
}

void IpmbSensor::parseConfigValues(const SensorBaseConfigMap& entry)
{
    auto findScaleVal = entry.find("ScaleValue");
    if (findScaleVal != entry.end())
    {
        scaleVal = std::visit(VariantToDoubleVisitor(), findScaleVal->second);
    }

    auto findOffsetVal = entry.find("OffsetValue");
    if (findOffsetVal != entry.end())
    {
        offsetVal = std::visit(VariantToDoubleVisitor(), findOffsetVal->second);
    }

    readState = getPowerState(entry);
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<IpmbSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (!dbusConnection)
    {
        lg2::error("Connection not created");
        return;
    }
    dbusConnection->async_method_call(
        [&](boost::system::error_code ec, const ManagedObjectType& resp) {
            if (ec)
            {
                lg2::error("Error contacting entity manager");
                return;
            }
            for (const auto& [path, interfaces] : resp)
            {
                for (const auto& [intf, cfg] : interfaces)
                {
                    if (intf != configInterfaceName(sensorType))
                    {
                        continue;
                    }
                    std::string name = loadVariant<std::string>(cfg, "Name");

                    std::vector<thresholds::Threshold> sensorThresholds;
                    if (!parseThresholdsFromConfig(interfaces,
                                                   sensorThresholds))
                    {
                        lg2::error("error populating thresholds '{NAME}'",
                                   "NAME", name);
                    }
                    uint8_t deviceAddress =
                        loadVariant<uint8_t>(cfg, "Address");

                    std::string sensorClass =
                        loadVariant<std::string>(cfg, "Class");

                    uint8_t hostSMbusIndex = hostSMbusIndexDefault;
                    auto findSmType = cfg.find("HostSMbusIndex");
                    if (findSmType != cfg.end())
                    {
                        hostSMbusIndex = std::visit(
                            VariantToUnsignedIntVisitor(), findSmType->second);
                    }

                    float pollRate = getPollRate(cfg, pollRateDefault);

                    uint8_t ipmbBusIndex = ipmbBusIndexDefault;
                    auto findBusType = cfg.find("Bus");
                    if (findBusType != cfg.end())
                    {
                        ipmbBusIndex = std::visit(VariantToUnsignedIntVisitor(),
                                                  findBusType->second);
                        lg2::error("Ipmb Bus Index for '{NAME}' is '{INDEX}'",
                                   "NAME", name, "INDEX", ipmbBusIndex);
                    }

                    /* Default sensor type is "temperature" */
                    std::string sensorTypeName = "temperature";
                    auto findType = cfg.find("SensorType");
                    if (findType != cfg.end())
                    {
                        sensorTypeName = std::visit(VariantToStringVisitor(),
                                                    findType->second);
                    }

                    auto& sensor = sensors[name];
                    sensor = nullptr;
                    sensor = std::make_shared<IpmbSensor>(
                        dbusConnection, io, name, path, objectServer,
                        std::move(sensorThresholds), deviceAddress,
                        hostSMbusIndex, pollRate, sensorTypeName);

                    sensor->parseConfigValues(cfg);
                    if (!(sensor->sensorClassType(sensorClass)))
                    {
                        continue;
                    }
                    sensor->sensorSubType(sensorTypeName);
                    sensor->init();
                }
            }
        },
        entityManagerName, "/xyz/openbmc_project/inventory",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
}

void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<IpmbSensor>>&
        sensors)
{
    sdbusplus::message::object_path removedPath;
    std::vector<std::string> interfaces;

    message.read(removedPath, interfaces);

    // If the xyz.openbmc_project.Confguration.X interface was removed
    // for one or more sensors, delete those sensor objects.
    auto sensorIt = sensors.begin();
    while (sensorIt != sensors.end())
    {
        if ((sensorIt->second->configurationPath == removedPath) &&
            (std::find(interfaces.begin(), interfaces.end(),
                       configInterfaceName(sdrInterface)) != interfaces.end()))
        {
            sensorIt = sensors.erase(sensorIt);
        }
        else
        {
            sensorIt++;
        }
    }
}
