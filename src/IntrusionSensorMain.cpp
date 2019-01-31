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

#include <ChassisIntrusionSensor.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio.hpp>
#include <chrono>
#include <ctime>
#include <iostream>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/sd_event.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/server.hpp>
#include <sdbusplus/timer.hpp>

static constexpr bool DEBUG = false;

// support two types of intrusion sensor: PCH based (via I2C) or GPIO based
static constexpr std::array<const char*, 2> sensorTypes = {
    "xyz.openbmc_project.Configuration.ChassisIntrusionSensor",
    "xyz.openbmc_project.Configuration.Gpio"};
static constexpr const char* sensorName = "Chassis Intrusion Sensor";

static bool getIntrusionSensorConfig(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    IntrusionSensorType* pType, int* pBusId, int* pSlaveAddr, int* pGpioIndex,
    bool* pGpioInverted)
{
    // find matched configuration according to sensor type
    ManagedObjectType sensorConfigurations;
    bool useCache = false;

    for (const char* type : sensorTypes)
    {
        if (!getSensorConfiguration(type, dbusConnection, sensorConfigurations,
                                    useCache))
        {
            std::cerr << "error communicating to entity manager\n";
            return false;
        }
        useCache = true;
    }
    const SensorData* sensorData = nullptr;
    const char* baseType;
    const std::pair<std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
        baseConfiguration = nullptr;

    // Get bus and addr of matched configuration
    for (const std::pair<sdbusplus::message::object_path, SensorData>& sensor :
         sensorConfigurations)
    {
        baseConfiguration = nullptr;
        sensorData = &(sensor.second);
        for (const char* type : sensorTypes)
        {
            // match sensor type
            auto sensorBase = sensorData->find(type);
            if (sensorBase == sensorData->end())
            {
                continue;
            }

            // match name, especially for type of GPIO
            auto configurationName = sensorBase->second.find("Name");
            if (configurationName == sensorBase->second.end())
            {
                continue;
            }
            else
            {
                auto name = sdbusplus::message::variant_ns::get<std::string>(
                    configurationName->second);
                if (!boost::ends_with(name, sensorName))
                {
                    continue;
                }
            }

            baseConfiguration = &(*sensorBase);
            baseType = type;
            break;
        }

        if (baseConfiguration == nullptr)
        {
            continue;
        }

        if (boost::ends_with(baseType, "Gpio"))
        {
            auto findGpioIndex = baseConfiguration->second.find("Index");
            auto findGpioPolarity = baseConfiguration->second.find("Polarity");

            if (findGpioIndex == baseConfiguration->second.end() ||
                findGpioPolarity == baseConfiguration->second.end())
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

            *pType = IntrusionSensorType::gpio;
            return true;
        }
        else
        {
            auto findBus = baseConfiguration->second.find("Bus");
            auto findAddress = baseConfiguration->second.find("Address");
            if (findBus == baseConfiguration->second.end() ||
                findAddress == baseConfiguration->second.end())
            {
                std::cerr << "error finding bus or address in configuration";
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
            *pType = IntrusionSensorType::pch;
            return true;
        }
    }

    std::cerr << "can't find matched I2C or GPIO configuration. \n";
    *pBusId = -1;
    *pSlaveAddr = -1;
    *pGpioIndex = -1;
    return false;
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
        objServer.add_interface("/xyz/openbmc_project/Intrusion/Chassis",
                                "xyz.openbmc_project.Intrusion");

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

    for (const char* type : sensorTypes)
    {
        auto match = std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*systemBus),
            "type='signal',member='PropertiesChanged',path_namespace='" +
                std::string(inventoryPath) + "',arg0namespace='" + type + "'",
            eventHandler);
    }

    io.run();

    return 0;
}
