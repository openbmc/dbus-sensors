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

static constexpr const char* sensorType =
    "xyz.openbmc_project.Configuration.ChassisIntrusionSensor";

static int getIntrusionSensorConfig(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection, int* pBusId,
    int* pSlaveAddr)
{
    // find matched configuration according to sensor type
    ManagedObjectType sensorConfigurations;
    bool useCache = false;

    if (!getSensorConfiguration(sensorType, dbusConnection,
                                sensorConfigurations, useCache))
    {
        std::cerr << "error communicating to entity manager\n";
        return -1;
    }

    const SensorData* sensorData = nullptr;
    const std::pair<std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
        baseConfiguration = nullptr;

    // Get bus and addr of matched configuration
    for (const std::pair<sdbusplus::message::object_path, SensorData>& sensor :
         sensorConfigurations)
    {
        sensorData = &(sensor.second);
        auto sensorBase = sensorData->find(sensorType);
        if (sensorBase != sensorData->end())
        {
            baseConfiguration = &(*sensorBase);
        }
        else
        {
            std::cerr << "error finding base configuration \n";
            continue;
        }

        auto configurationBus = baseConfiguration->second.find("Bus");
        auto configurationAddress = baseConfiguration->second.find("Address");
        if (configurationBus == baseConfiguration->second.end() ||
            configurationAddress == baseConfiguration->second.end())
        {
            std::cerr << "error finding bus or address in configuration";
            continue;
        }
        else
        {
            *pBusId = sdbusplus::message::variant_ns::get<uint64_t>(
                configurationBus->second);
            *pSlaveAddr = sdbusplus::message::variant_ns::get<uint64_t>(
                configurationAddress->second);
            std::cout << "find matched bus " << *pBusId
                      << ", matched slave addr " << *pSlaveAddr << "\n";
            return 0;
        }
    }

    std::cerr << "can't find matched bus or address in configuration. \n";
    *pBusId = -1;
    *pSlaveAddr = -1;
    return -1;
}

int main()
{
    int busId = -1, slaveAddr = -1;

    // setup connection to dbus
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    auto objServer = sdbusplus::asio::object_server(systemBus);

    // setup object server, define interface and properties
    systemBus->request_name("xyz.openbmc_project.IntrusionSensor");

    std::shared_ptr<sdbusplus::asio::dbus_interface> iface =
        objServer.add_interface("/xyz/openbmc_project/Intrusion",
                                "xyz.openbmc_project.Intrusion");

    iface->register_property("ChassisIntrusionStatus", -1);
    iface->initialize();

    ChassisIntrusionSensor chassisIntrusionSensor(io, iface);

    getIntrusionSensorConfig(systemBus, &busId, &slaveAddr);
    chassisIntrusionSensor.start(busId, slaveAddr);

    // callback to handle configuration change
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }

            std::cout << "rescan due to configuration change \n";
            getIntrusionSensorConfig(systemBus, &busId, &slaveAddr);
            chassisIntrusionSensor.start(busId, slaveAddr);
        };

    auto match = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + sensorType + "'",
        eventHandler);

    io.run();

    return 0;
}
