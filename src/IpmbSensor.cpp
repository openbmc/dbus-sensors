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

#include <IpmbSDRSensor.hpp>
#include <IpmbSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
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

constexpr const char* sensorType = "IpmbSensor";
constexpr const char* sdrInterface = "IpmbDevice";

static constexpr double ipmbMaxReading = 0xFF;
static constexpr double ipmbMinReading = 0;

static constexpr uint8_t meAddress = 1;
static constexpr uint8_t lun = 0;
static constexpr uint8_t hostSMbusIndexDefault = 0x03;
static constexpr uint8_t ipmbBusIndexDefault = 0;
static constexpr float pollRateDefault = 1; // in seconds

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

boost::container::flat_map<std::string, std::shared_ptr<IpmbSensor>> sensors;
boost::container::flat_map<uint8_t, std::shared_ptr<IpmbSDRDevice>> sdrsensor;

std::unique_ptr<boost::asio::steady_timer> initCmdTimer;
std::unique_ptr<boost::asio::steady_timer> sdrWaitTimer;

IpmbSensor::IpmbSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                       boost::asio::io_service& io,
                       const std::string& sensorName,
                       const std::string& sensorConfiguration,
                       sdbusplus::asio::object_server& objectServer,
                       std::vector<thresholds::Threshold>&& thresholdData,
                       uint8_t deviceAddress, uint8_t hostSMbusIndex,
                       uint8_t ipmbBusIndex, const float pollRate,
                       std::string& sensorTypeName) :
    Sensor(escapeName(sensorName), std::move(thresholdData),
           sensorConfiguration, "IpmbSensor", false, false, ipmbMaxReading,
           ipmbMinReading, conn, PowerState::on),
    deviceAddress(deviceAddress), hostSMbusIndex(hostSMbusIndex),
    ipmbBusIndex(ipmbBusIndex), sensorPollMs(static_cast<int>(pollRate * 1000)),
    objectServer(objectServer), waitTimer(io)
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

std::string IpmbSensor::getSubTypeUnits(void) const
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
    setInitialProperties(getSubTypeUnits());
    if (initCommand)
    {
        runInitCmd();
    }
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
        std::cerr << "Error setting init command for device: " << self->name
                  << "\n";
    }
}

void IpmbSensor::runInitCmd()
{
    if (!initCommand)
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
        "sendRequest", commandAddress, netfn, lun, *initCommand, initData);
}

void IpmbSensor::loadDefaults()
{
    if (type == IpmbType::meSensor || type == IpmbType::IpmbDevice)
    {
        if (type == IpmbType::meSensor)
        {
            commandAddress = meAddress;
        }
        else
        {
            commandAddress = ipmbBusIndex << ipmbLeftShift;
        }
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
                if (errCount == 0U)
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
                if (errCount == 0U)
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
                if (errCount == 0U)
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
                if (errCount == 0U)
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
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<uint8_t*>(&rawData)[i] = data[i];
    }
    rawValue = static_cast<double>(rawData);

    /* Adjust value as per scale and offset */
    value = (value * scaleVal) + offsetVal;
    updateValue(value);
    read();
}

void IpmbSensor::read(void)
{
    waitTimer.expires_from_now(std::chrono::milliseconds(sensorPollMs));
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
    else if (sensorClass == "IpmbDevice")
    {
        type = IpmbType::IpmbDevice;
    }
    else
    {
        std::cerr << "Invalid class " << sensorClass << "\n";
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

void configSDRSensors(const SensorInfo& sensorInfo,
                      const std::string& interface, std::string& name,
                      uint8_t& deviceAddress, uint8_t index,
                      std::vector<thresholds::Threshold>& sensorThresholds,
                      std::string& sensorTypeName)
{
    if (interface != configInterfaceName(sdrInterface))
    {
        return;
    }
    deviceAddress = sensorInfo.sensorNumber;

    name = std::to_string(index + 1) + "-" + sensorInfo.sensorReadName;

    if (sensorInfo.sensCap != (sdrtype01::sdrSensNoThres))
    {
        sensorThresholds.emplace_back(thresholds::Level::CRITICAL,
                                      thresholds::Direction::HIGH,
                                      sensorInfo.thresUpperCri);
        sensorThresholds.emplace_back(thresholds::Level::CRITICAL,
                                      thresholds::Direction::LOW,
                                      sensorInfo.thresLowerCri);
    }
    sensorTypeName = IpmbSDRDevice::sensorUnits[sensorInfo.sensorUnit];
}

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<IpmbSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    bool ipmbSDREnable)
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
        for (const auto& [path, interfaces] : resp)
        {
            for (const auto& [intf, cfg] : interfaces)
            {
                if ((!ipmbSDREnable) &&
                    (intf != configInterfaceName(sensorType)))
                {
                    continue;
                }
                if ((ipmbSDREnable) &&
                    (intf != configInterfaceName(sdrInterface)))
                {
                    continue;
                }

                std::string name = loadVariant<std::string>(cfg, "Name");

                std::vector<thresholds::Threshold> sensorThresholds;
                if (!parseThresholdsFromConfig(interfaces, sensorThresholds))
                {
                    std::cerr << "error populating thresholds for " << name
                              << "\n";
                }
                uint8_t deviceAddress = loadVariant<uint8_t>(cfg, "Address");

                std::string sensorClass =
                    loadVariant<std::string>(cfg, "Class");

                uint8_t hostSMbusIndex = hostSMbusIndexDefault;
                auto findSmType = cfg.find("HostSMbusIndex");
                if (findSmType != cfg.end())
                {
                    hostSMbusIndex = std::visit(VariantToUnsignedIntVisitor(),
                                                findSmType->second);
                }

                float pollRate = getPollRate(cfg, pollRateDefault);

                uint8_t ipmbBusIndex = ipmbBusIndexDefault;
                auto findBusType = cfg.find("Bus");
                if (findBusType != cfg.end())
                {
                    ipmbBusIndex = std::visit(VariantToUnsignedIntVisitor(),
                                              findBusType->second);
                }

                /* Default sensor type is "temperature" */
                std::string sensorTypeName = "temperature";
                auto findType = cfg.find("SensorType");
                if (findType != cfg.end())
                {
                    sensorTypeName =
                        std::visit(VariantToStringVisitor(), findType->second);
                }

                if (intf == configInterfaceName(sdrInterface))
                {
                    for (auto& sensorInfo : sensorRecord[ipmbBusIndex])
                    {
                        std::vector<thresholds::Threshold> sensorThreshold;
                        if (!parseThresholdsFromConfig(interfaces,
                                                       sensorThreshold))
                        {
                            std::cerr << "error populating thresholds for "
                                      << name << "\n";
                        }

                        configSDRSensors(sensorInfo, intf, name, deviceAddress,
                                         ipmbBusIndex, sensorThreshold,
                                         sensorTypeName);

                        auto& sensor = sensors[name];
                        sensor = std::make_shared<IpmbSensor>(
                            dbusConnection, io, name, path, objectServer,
                            std::move(sensorThreshold), deviceAddress,
                            hostSMbusIndex, ipmbBusIndex, pollRate,
                            sensorTypeName);

                        sensor->parseConfigValues(cfg);
                        if (!(sensor->sensorClassType(sensorClass)))
                        {
                            continue;
                        }
                        sensor->sensorSubType(sensorTypeName);
                        sensor->init();
                    }
                    continue;
                }
                auto& sensor = sensors[name];
                sensor = nullptr;
                sensor = std::make_shared<IpmbSensor>(
                    dbusConnection, io, name, path, objectServer,
                    std::move(sensorThresholds), deviceAddress, hostSMbusIndex,
                    ipmbBusIndex, pollRate, sensorTypeName);

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

void sdrHandler(
    sdbusplus::message_t& message,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<IpmbSensor>>&
        sensors)
{
    std::string objectName;
    SensorBaseConfigMap values;
    message.read(objectName, values);

    auto findBus = values.find("Bus");
    if (findBus == values.end())
    {
        return;
    }

    uint8_t busIndex = loadVariant<uint8_t>(values, "Bus");

    auto& sdrsen = sdrsensor[busIndex];
    sdrsen = nullptr;
    sdrsen = std::make_shared<IpmbSDRDevice>(dbusConnection, busIndex);
    sdrsen->getSDRRepositoryInfo();

    size_t waitSeconds = 15;
    sdrWaitTimer->expires_from_now(std::chrono::seconds(waitSeconds));
    sdrWaitTimer->async_wait([&io, &objectServer, &sensors, &dbusConnection](
                                 const boost::system::error_code ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }

        createSensors(io, objectServer, sensors, dbusConnection, true);
        if (sensors.empty())
        {
            std::cout << "SDR Configuration not detected\n";
        }
    });
}

void reinitSensors(sdbusplus::message_t& message)
{
    constexpr const size_t reinitWaitSeconds = 2;
    std::string objectName;
    boost::container::flat_map<std::string, std::variant<std::string>> values;
    message.read(objectName, values);

    auto findStatus = values.find(power::property);
    if (findStatus != values.end())
    {
        bool powerStatus =
            std::get<std::string>(findStatus->second).ends_with(".Running");
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
                std::chrono::seconds(reinitWaitSeconds));

            initCmdTimer->async_wait([](const boost::system::error_code ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }

                for (const auto& [name, sensor] : sensors)
                {
                    if (sensor)
                    {
                        sensor->runInitCmd();
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
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    systemBus->request_name("xyz.openbmc_project.IpmbSensor");

    initCmdTimer = std::make_unique<boost::asio::steady_timer>(io);
    sdrWaitTimer = std::make_unique<boost::asio::steady_timer>(io);

    io.post(
        [&]() { createSensors(io, objectServer, sensors, systemBus, false); });

    boost::asio::steady_timer configTimer(io);

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t&) {
        configTimer.expires_from_now(std::chrono::seconds(1));
        // create a timer because normally multiple properties change
        configTimer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }
            createSensors(io, objectServer, sensors, systemBus, false);
            if (sensors.empty())
            {
                std::cout << "Configuration not detected\n";
            }
        });
    };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(
            *systemBus, std::to_array<const char*>({sensorType}), eventHandler);

    sdbusplus::bus::match_t powerChangeMatch(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        "type='signal',interface='" + std::string(properties::interface) +
            "',path='" + std::string(power::path) + "',arg0='" +
            std::string(power::interface) + "'",
        reinitSensors);

    auto matchSignal = std::make_shared<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" +
            configInterfaceName(sdrInterface) + "'",
        [&systemBus, &io, &objectServer](sdbusplus::message_t& msg) {
        sdrHandler(msg, systemBus, io, objectServer, sensors);
        });

    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
