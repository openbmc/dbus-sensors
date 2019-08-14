/*
// Copyright (c) 2018 Intel Corporation
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

#include <systemd/sd-journal.h>

#include <ChassisIntrusionSensor.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio.hpp>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/sd_event.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/server.hpp>
#include <sdbusplus/timer.hpp>

static constexpr bool DEBUG = false;

static constexpr const char* sensorType =
    "xyz.openbmc_project.Configuration.ChassisIntrusionSensor";

static bool getIntrusionSensorConfig(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    IntrusionSensorType* pType, int* pBusId, int* pSlaveAddr, int* pGpioIndex,
    bool* pGpioInverted)
{
    // find matched configuration according to sensor type
    ManagedObjectType sensorConfigurations;
    bool useCache = false;

    if (!getSensorConfiguration(sensorType, dbusConnection,
                                sensorConfigurations, useCache))
    {
        std::cerr << "error communicating to entity manager\n";
        return false;
    }

    const SensorData* sensorData = nullptr;
    const std::pair<std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
        baseConfiguration = nullptr;

    // Get bus and addr of matched configuration
    for (const std::pair<sdbusplus::message::object_path, SensorData>& sensor :
         sensorConfigurations)
    {
        baseConfiguration = nullptr;
        sensorData = &(sensor.second);

        // match sensor type
        auto sensorBase = sensorData->find(sensorType);
        if (sensorBase == sensorData->end())
        {
            std::cerr << "error finding base configuration \n";
            continue;
        }

        baseConfiguration = &(*sensorBase);

        // judge class, "Gpio" or "I2C"
        auto findClass = baseConfiguration->second.find("Class");
        if (findClass != baseConfiguration->second.end() &&
            sdbusplus::message::variant_ns::get<std::string>(
                findClass->second) == "Gpio")
        {
            *pType = IntrusionSensorType::gpio;
        }
        else
        {
            *pType = IntrusionSensorType::pch;
        }

        // case to find GPIO info
        if (*pType == IntrusionSensorType::gpio)
        {
            auto gpioConfig =
                sensorData->find(sensorType + std::string(".GpioIntrusion"));

            if (gpioConfig == sensorData->end())
            {
                std::cerr
                    << "error finding GpioIntrusion info in configuration \n";
                continue;
            }

            auto findGpioIndex = gpioConfig->second.find("Index");
            auto findGpioPolarity = gpioConfig->second.find("Polarity");

            if (findGpioIndex == gpioConfig->second.end() ||
                findGpioPolarity == gpioConfig->second.end())
            {
                std::cerr << "error finding gpio info in configuration \n";
                continue;
            }

            try
            {
                *pGpioIndex = sdbusplus::message::variant_ns::get<uint64_t>(
                    findGpioIndex->second);
                *pGpioInverted =
                    (sdbusplus::message::variant_ns::get<std::string>(
                         findGpioPolarity->second) == "Low");
            }
            catch (const std::bad_variant_access& e)
            {
                std::cerr << "invalid value for gpio info in config. \n";
                continue;
            }

            if (DEBUG)
            {
                std::cout << "find matched GPIO index " << *pGpioIndex
                          << ", polarity inverted flag is " << *pGpioInverted
                          << "\n";
            }

            return true;
        }

        // case to find I2C info
        else if (*pType == IntrusionSensorType::pch)
        {
            auto findBus = baseConfiguration->second.find("Bus");
            auto findAddress = baseConfiguration->second.find("Address");
            if (findBus == baseConfiguration->second.end() ||
                findAddress == baseConfiguration->second.end())
            {
                std::cerr << "error finding bus or address in configuration \n";
                continue;
            }

            try
            {
                *pBusId = sdbusplus::message::variant_ns::get<uint64_t>(
                    findBus->second);
                *pSlaveAddr = sdbusplus::message::variant_ns::get<uint64_t>(
                    findAddress->second);
            }
            catch (const std::bad_variant_access& e)
            {
                std::cerr << "invalid value for bus or address in config. \n";
                continue;
            }

            if (DEBUG)
            {
                std::cout << "find matched bus " << *pBusId
                          << ", matched slave addr " << *pSlaveAddr << "\n";
            }
            return true;
        }
    }

    std::cerr << "can't find matched I2C or GPIO configuration. \n";
    *pBusId = -1;
    *pSlaveAddr = -1;
    *pGpioIndex = -1;
    return false;
}

static std::array<bool, 2> isLanConnected = {false, false};
static std::array<std::string, 2> pathSuffix = {"", ""};

static void processLanStatusChange(sdbusplus::message::message& message)
{
    std::string pathName = message.get_path();
    std::string interfaceName;
    boost::container::flat_map<std::string, BasicVariantType> properties;
    message.read(interfaceName, properties);

    auto findStateProperty = properties.find("OperationalState");
    if (findStateProperty == properties.end())
    {
        return;
    }
    std::string* pState = sdbusplus::message::variant_ns::get_if<std::string>(
        &(findStateProperty->second));
    if (pState == nullptr)
    {
        std::cerr << "invalid OperationalState \n";
        return;
    }

    bool newLanConnected = (*pState == "routable" || *pState == "carrier" ||
                            *pState == "degraded");

    // get ethNum from path. /org/freedesktop/network1/link/_32 for eth0
    int ethNum = 0;
    int pos = pathName.find("/_");
    if (pos == std::string::npos)
    {
        std::cerr << "unexpected path name " << pathName << "\n";
        return;
    }
    std::string suffixStr = pathName.substr(pos + 2);
    if (suffixStr == pathSuffix[0])
    {
        ethNum = 0;
    }
    else if (suffixStr == pathSuffix[1])
    {
        ethNum = 1;
    }
    else
    {
        std::cerr << "unexpected eth for suffixStr " << suffixStr << "\n";
        return;
    }

    if (isLanConnected[ethNum] != newLanConnected)
    {
        std::string strEthNum = "eth" + std::to_string(ethNum);
        std::string strEvent = strEthNum + " LAN leash lost";
        std::string strAssert = newLanConnected ? "de-asserted" : "asserted";
        std::string strMsg = strEthNum + " is " +
                             (newLanConnected ? "connected" : "disconnected");
        std::string strMsgId = "OpenBMC.0.1.PhysicalSecurity";
        sd_journal_send("MESSAGE=%s", strMsg.c_str(), "PRIORITY=%i", LOG_INFO,
                        "REDFISH_MESSAGE_ID=%s", strMsgId.c_str(),
                        "REDFISH_MESSAGE_ARGS=%s,%s", strEvent.c_str(),
                        strAssert.c_str(), NULL);
        isLanConnected[ethNum] = newLanConnected;
    }
}

static void
    monitorLanStatusChange(std::shared_ptr<sdbusplus::asio::connection> conn)
{
    // init pathSuffix which is ASCII of ifindex
    std::string line;
    for (int ethNum = 0; ethNum < 2; ethNum++)
    {
        auto fileName =
            "/sys/class/net/eth" + std::to_string(ethNum) + "/ifindex";
        std::ifstream sysFile(fileName);
        if (!sysFile.good())
        {
            std::cerr << "Failure reading " << fileName << "\n";
            return;
        }
        getline(sysFile, line);
        std::cout << "ethNum=" << ethNum << ",line=" << line << "\n";
        sysFile.close();
        const uint8_t ifindex = std::atoi(line.c_str());
        pathSuffix[ethNum] =
            std::to_string(ifindex + 30); // ASCII of "0" is 0x30
    }

    // init lan connected status
    for (int ethNum = 0; ethNum < 2; ethNum++)
    {
        conn->async_method_call(
            [ethNum](boost::system::error_code ec,
                     const std::variant<std::string>& property) {
                if (ec)
                {
                    return;
                }
                const std::string* pState = std::get_if<std::string>(&property);
                if (pState == nullptr)
                {
                    std::cerr << "Unable to read lan status value\n";
                    return;
                }
                isLanConnected[ethNum] =
                    (*pState == "routable" || *pState == "carrier" ||
                     *pState == "degraded");
            },
            "org.freedesktop.network1",
            "/org/freedesktop/network1/link/_" + pathSuffix[ethNum],
            "org.freedesktop.DBus.Properties", "Get",
            "org.freedesktop.network1.Link", "OperationalState");
    }

    // add match to monitor lan status change
    static auto matchLanStatusChange =
        std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*conn),
            "type='signal', member='PropertiesChanged',"
            "arg0namespace='org.freedesktop.network1.Link'",
            [](sdbusplus::message::message& msg) {
                processLanStatusChange(msg);
            });
}

int main()
{
    int busId = -1, slaveAddr = -1, gpioIndex = -1;
    bool gpioInverted = false;
    IntrusionSensorType type = IntrusionSensorType::gpio;

    // setup connection to dbus
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    auto objServer = sdbusplus::asio::object_server(systemBus);

    // setup object server, define interface
    systemBus->request_name("xyz.openbmc_project.IntrusionSensor");

    std::shared_ptr<sdbusplus::asio::dbus_interface> ifaceChassis =
        objServer.add_interface(
            "/xyz/openbmc_project/Intrusion/Chassis_Intrusion",
            "xyz.openbmc_project.Chassis.Intrusion");

    ChassisIntrusionSensor chassisIntrusionSensor(io, ifaceChassis);

    if (getIntrusionSensorConfig(systemBus, &type, &busId, &slaveAddr,
                                 &gpioIndex, &gpioInverted))
    {
        chassisIntrusionSensor.start(type, busId, slaveAddr, gpioIndex,
                                     gpioInverted);
    }

    // callback to handle configuration change
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }

            std::cout << "rescan due to configuration change \n";
            if (getIntrusionSensorConfig(systemBus, &type, &busId, &slaveAddr,
                                         &gpioIndex, &gpioInverted))
            {
                chassisIntrusionSensor.start(type, busId, slaveAddr, gpioIndex,
                                             gpioInverted);
            }
        };

    auto match = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + sensorType + "'",
        eventHandler);

    monitorLanStatusChange(systemBus);

    io.run();

    return 0;
}
