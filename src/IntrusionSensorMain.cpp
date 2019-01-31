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
                std::cerr << "error finding GpioIntrusion info in configuration \n";
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

    auto match = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + sensorType + "'",
        eventHandler);

    io.run();

    return 0;
}
