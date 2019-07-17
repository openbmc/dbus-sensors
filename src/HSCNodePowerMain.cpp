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

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string,
                               std::unique_ptr<HSCNodePowerSensor>>& sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    int busId = -1;
    int hscAddress = -1;
    std::string sensorName;
    HSCType type;

    if (!dbusConnection)
    {
        return;
    }

    // find matched configuration according to sensor type
    ManagedObjectType sensorConfigurations;
    bool useCache = false;

    if (!getSensorConfiguration(hscType, dbusConnection, sensorConfigurations,
                                useCache))
    {
        std::cerr << "error communicating to entity manager\n";
        return;
    }

    const SensorData* sensorData = nullptr;
    const std::pair<std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
        baseConfiguration = nullptr;
    const std::string* interfacePath = nullptr;
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
            type = HSCType::MP5920;
        }

        // Find I2C info
        auto findBus = baseConfiguration->second.find("Bus");
        auto findAddress = baseConfiguration->second.find("Address");
        auto findSensorName = baseConfiguration->second.find("Name");
        if (findBus == baseConfiguration->second.end() ||
            findAddress == baseConfiguration->second.end() ||
            findSensorName == baseConfiguration->second.end())
        {
            continue;
        }

        try
        {
            busId = std::get<uint64_t>(findBus->second);
            hscAddress = std::get<uint64_t>(findAddress->second);
            sensorName = std::get<std::string>(findSensorName->second);
        }
        catch (const std::bad_variant_access& e)
        {
            continue;
        }

        sensorData = &(sensor.second);
        interfacePath = &(sensor.first.str);
        break;
    }

    if (sensorData == nullptr)
    {
        std::cerr << "failed to find match   "
                  << "\n";
    }

    std::vector<thresholds::Threshold> sensorThresholds;
    if (!parseThresholdsFromConfig(*sensorData, sensorThresholds))
    {
        std::cerr << "error populating thresholds for " << sensorName << "\n";
    }

    auto& sensor = sensors[sensorName];
    sensor = nullptr;
    sensor = std::make_unique<HSCNodePowerSensor>(
        dbusConnection, io, sensorName, *interfacePath, objectServer,
        std::move(sensorThresholds), busId, hscAddress, type);

    sensor->start(type, busId, hscAddress);
}

int main()
{
    // setup connection to dbus
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::unique_ptr<HSCNodePowerSensor>>
        sensors;
    // setup object server, define interface
    systemBus->request_name("xyz.openbmc_project.NodePowerSensor");

    io.post([&]() { createSensors(io, objectServer, sensors, systemBus); });

    boost::asio::deadline_timer configTimer(io);
    // callback to handle configuration change
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message&) {
            configTimer.expires_from_now(boost::posix_time::seconds(1));
            // create a timer because normally multiple properties change
            configTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }
                // config timer error
                else if (ec)
                {
                    return;
                }
                createSensors(io, objectServer, sensors, systemBus);
                if (sensors.empty())
                {
                    std::cout << "Configuration not detected\n";
                }
            });
        };

    auto match = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + hscType + "'",
        eventHandler);

    io.run();

    return 0;
}
