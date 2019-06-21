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

#include <HSCNodePower.hpp>
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

static constexpr const char* hscType =
    "xyz.openbmc_project.Configuration.MP5920";

static bool getHSCNodePowerConfig(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    HSCType* pType, int* pBusId, int* pSlaveAddr)
{
    // find matched configuration according to sensor type
    ManagedObjectType sensorConfigurations;
    bool useCache = false;

    if (!getSensorConfiguration(hscType, dbusConnection, sensorConfigurations,
                                useCache))
    {
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
        auto sensorBase = sensorData->find(hscType);
        if (sensorBase == sensorData->end())
        {
            continue;
        }

        baseConfiguration = &(*sensorBase);

        // Judge HSC Type
        auto findType = baseConfiguration->second.find("Type");
        if (findType != baseConfiguration->second.end() &&
            std::get<std::string>(findType->second) == "MP5920")
        {
            *pType = HSCType::MP5920;
        }

        // Find I2C info
        auto findBus = baseConfiguration->second.find("Bus");
        auto findAddress = baseConfiguration->second.find("Address");
        if (findBus == baseConfiguration->second.end() ||
            findAddress == baseConfiguration->second.end())
        {
            continue;
        }

        try
        {
            *pBusId = std::get<uint64_t>(findBus->second);
            *pSlaveAddr = std::get<uint64_t>(findAddress->second);
        }
        catch (const std::bad_variant_access& e)
        {
            continue;
        }

        return true;
    }

    *pBusId = -1;
    *pSlaveAddr = -1;
    return false;
}

int main()
{
    int busId = -1, slaveAddr = -1;
    HSCType type = HSCType::MP5920;

    // setup connection to dbus
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus);

    // setup object server, define interface
    systemBus->request_name("xyz.openbmc_project.NodePower");

    HSCNodePower nodePower(io, objectServer, systemBus);

    if (getHSCNodePowerConfig(systemBus, &type, &busId, &slaveAddr))
    {
        nodePower.start(type, busId, slaveAddr);
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
            if (getHSCNodePowerConfig(systemBus, &type, &busId, &slaveAddr))
            {
                nodePower.start(type, busId, slaveAddr);
            }
        };

    auto match = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + hscType + "'",
        eventHandler);

    io.run();

    return 0;
}
