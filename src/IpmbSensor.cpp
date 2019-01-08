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
#include <chrono>
#include <iostream>
#include <limits>
#include <numeric>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <vector>

constexpr const bool debug = false;

constexpr const char* configInterface =
    "xyz.openbmc_project.Configuration.IpmbSensor";
static constexpr double ipmbMaxReading = 0xFF;
static constexpr double ipmbMinReading = 0;

static constexpr uint8_t meAddress = 1;
static constexpr uint8_t lun = 0;

IpmbSensor::IpmbSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                       boost::asio::io_service& io,
                       const std::string& sensorName,
                       const std::string& sensorConfiguration,
                       sdbusplus::asio::object_server& objectServer,
                       std::vector<thresholds::Threshold>&& thresholdData,
                       uint8_t deviceAddress) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           "" /* todo: remove arg from base*/, std::move(thresholdData),
           sensorConfiguration, "xyz.openbmc_project.Configuration.ExitAirTemp",
           ipmbMaxReading, ipmbMinReading),
    objectServer(objectServer), dbusConnection(conn), waitTimer(io),
    deviceAddress(deviceAddress)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    setupPowerMatch(conn);
}

IpmbSensor::~IpmbSensor()
{
    waitTimer.cancel();
    objectServer.remove_interface(thresholdInterfaceWarning);
    objectServer.remove_interface(thresholdInterfaceCritical);
    objectServer.remove_interface(sensorInterface);
}

void IpmbSensor::init(void)
{
    setInitialProperties(dbusConnection);
    loadDefaults();
    if (initCommand)
    {
        dbusConnection->async_method_call(
            [this](boost::system::error_code ec,
                   std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t,
                              std::vector<uint8_t>>
                       response) {
                int& status = std::get<0>(response);

                if (ec || status)
                {
                    std::cerr
                        << "Error setting init command for device: " << name
                        << "\n";
                }
                read();
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", commandAddress, netfn, lun, *initCommand, initData);
    }
    else
    {
        read();
    }
}

void IpmbSensor::loadDefaults()
{
    if (type == IpmbType::meSensor)
    {
        commandAddress = meAddress;
        netfn = 0x4;    // sensor
        command = 0x2d; // get sensor reading
        commandData = {deviceAddress};
    }
    else if (type == IpmbType::PXE1410CVR)
    {
        commandAddress = meAddress;
        netfn = 0x2e;       // me bridge
        command = 0xd9;     // send raw pmbus
        initCommand = 0xd9; // send raw pmbus
        commandData = {0x57, 0x01, 0x00, 0x16, 0x03, deviceAddress, 00,
                       0x00, 0x00, 0x00, 0x01, 0x02, 0x29};
        initData = {0x57, 0x01, 0x00, 0x14, 0x03, deviceAddress, 0x00,
                    0x00, 0x00, 0x00, 0x02, 0x00, 0x00,          0x60};
    }
    else if (type == IpmbType::IR38363VR)
    {
        commandAddress = meAddress;
        netfn = 0x2e;   // me bridge
        command = 0xd9; // send raw pmbus
        commandData = {0x57, 0x01, 0x00, 0x16, 0x03, deviceAddress, 00,
                       0x00, 0x00, 0x00, 0x01, 0x02, 0x8D};
    }
    else if (type == IpmbType::mpsVR)
    {
        commandAddress = meAddress;
        netfn = 0x2e;       // me bridge
        command = 0xd9;     // send raw pmbus
        initCommand = 0xd9; // send raw pmbus
        commandData = {0x57, 0x01, 0x00, 0x16, 0x3,  deviceAddress, 0x00,
                       0x00, 0x00, 0x00, 0x01, 0x02, 0x8d};
        initData = {0x57, 0x01, 0x00, 0x14, 0x03, deviceAddress, 0x00,
                    0x00, 0x00, 0x00, 0x02, 0x00, 0x00,          0x00};
    }
    else
    {
        throw std::runtime_error("Invalid sensor type");
    }
}

void IpmbSensor::checkThresholds(void)
{
    if (readState == PowerState::on && !isPowerOn())
    {
        return;
    }
    thresholds::checkThresholds(this);
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
        if (!isPowerOn() && readState == PowerState::on)
        {
            updateValue(0);
            read();
            return;
        }
        dbusConnection->async_method_call(
            [this](boost::system::error_code ec,
                   std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t,
                              std::vector<uint8_t>>
                       response) {
                int& status = std::get<0>(response);
                if (ec || status)
                {
                    std::cerr << "Error reading from device: " << name << "\n";
                    updateValue(0);
                    read();
                    return;
                }
                if (!isPowerOn() && readState == PowerState::on)
                {
                    updateValue(0);
                    read();
                    return;
                }
                std::vector<uint8_t>& data = std::get<5>(response);
                if constexpr (debug)
                {
                    std::cout << name << ": ";
                    for (size_t d : data)
                    {
                        std::cout << d << " ";
                    }
                    std::cout << "\n";
                }
                uint16_t value = 0;
                if (type == IpmbType::meSensor)
                {
                    if (data.empty())
                    {
                        std::cerr << "Invalid data from device: " << name
                                  << "\n";
                        read();
                        return;
                    }
                    value = data[0];
                }
                else if (type == IpmbType::PXE1410CVR ||
                         type == IpmbType::IR38363VR)
                {
                    if (data.size() < 4)
                    {
                        std::cerr << "Invalid data from device: " << name
                                  << "\n";
                        read();
                        return;
                    }
                    value = ((data[4] << 8) | data[3]) >> 3;
                }
                else if (type == IpmbType::mpsVR)
                {
                    if (data.size() < 4)
                    {
                        std::cerr << "Invalid data from device: " << name
                                  << "\n";
                        read();
                        return;
                    }
                    value = data[3];
                }
                else
                {
                    throw std::runtime_error("Invalid sensor type");
                }
                updateValue(value);
                read();
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", commandAddress, netfn, lun, command, commandData);
    });
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
                    auto& sensor = sensors[name];
                    sensor = std::make_unique<IpmbSensor>(
                        dbusConnection, io, name, pathPair.first, objectServer,
                        std::move(sensorThresholds), deviceAddress);

                    if (sensorClass == "PxeBridge")
                    {
                        sensor->type = IpmbType::PXE1410CVR;
                    }
                    else if (sensorClass == "IRBridge")
                    {
                        sensor->type = IpmbType::IR38363VR;
                    }
                    else if (sensorClass == "MpsBridge")
                    {
                        sensor->type = IpmbType::mpsVR;
                    }
                    else if (sensorClass == "ME")
                    {
                        sensor->type = IpmbType::meSensor;
                    }
                    else
                    {
                        std::cerr << "Invalid class " << sensorClass << "\n";
                        continue;
                    }
                    sensor->init();
                }
            }
        },
        entityManagerName, "/", "org.freedesktop.DBus.ObjectManager",
        "GetManagedObjects");
}

int main(int argc, char** argv)
{

    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.IpmbSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::unique_ptr<IpmbSensor>>
        sensors;

    io.post([&]() { createSensors(io, objectServer, sensors, systemBus); });

    boost::asio::deadline_timer configTimer(io);

    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
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

    sdbusplus::bus::match::match match(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + configInterface +
            "'",
        eventHandler);

    io.run();
}
