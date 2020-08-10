/*
// Copyright (c) 2017 Intel Corporation
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

#include "HostSensor.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <variant>
#include <vector>

static constexpr bool DEBUG = false;

static constexpr std::array<const char*, 1> sensorTypes = {
    "xyz.openbmc_project.Configuration.HostSensor"};

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<HostSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        std::move([&io, &objectServer, &sensors, &dbusConnection,
                   sensorsChanged](
                      const ManagedObjectType& sensorConfigurations) {
            bool firstScan = sensorsChanged == nullptr;
            std::vector<std::filesystem::path> paths;
            if (!findFiles(std::filesystem::path("/sys/class/hwmon"),
                           R"(in\d+_input)", paths))
            {
                std::cerr << "No temperature sensors in system\n";
                return;
            }

            int index = 0;

            // iterate through all found host sensors, and try to match them
            // with configuration
            for (auto& path : paths)
            {
                // if (!isAdc(path.parent_path()))
                // {
                //     continue;
                // }
                std::smatch match;
                std::string pathStr = path.string();

                auto directory = path.parent_path();

                const SensorData* sensorData = nullptr;
                const std::string* interfacePath = nullptr;
                const std::pair<
                    std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
                    baseConfiguration;
                for (const std::pair<sdbusplus::message::object_path,
                                     SensorData>& sensor : sensorConfigurations)
                {
                    // clear it out each loop
                    baseConfiguration = nullptr;

                    // find base configuration
                    for (const char* type : sensorTypes)
                    {
                        auto sensorBase = sensor.second.find(type);
                        if (sensorBase != sensor.second.end())
                        {
                            baseConfiguration = &(*sensorBase);
                            break;
                        }
                    }
                    if (baseConfiguration == nullptr)
                    {
                        continue;
                    }
                    auto findIndex = baseConfiguration->second.find("Index");
                    if (findIndex == baseConfiguration->second.end())
                    {
                        std::cerr << "Base configuration missing Index"
                                  << baseConfiguration->first << "\n";
                        continue;
                    }

                    unsigned int number = std::visit(
                        VariantToUnsignedIntVisitor(), findIndex->second);

                    if (number != index)
                    {
                        continue;
                    }

                    sensorData = &(sensor.second);
                    interfacePath = &(sensor.first.str);
                    break;
                }
                if (sensorData == nullptr)
                {
                    std::cerr << "failed to find match for " << path.string()
                              << "\n";
                    continue;
                }

                if (baseConfiguration == nullptr)
                {
                    std::cerr << "error finding base configuration for"
                              << path.string() << "\n";
                    continue;
                }

                auto findSensorName = baseConfiguration->second.find("Name");
                if (findSensorName == baseConfiguration->second.end())
                {
                    std::cerr << "could not determine configuration name for "
                              << path.string() << "\n";
                    continue;
                }
                std::string sensorName =
                    std::get<std::string>(findSensorName->second);

                // on rescans, only update sensors we were signaled by
                auto findSensor = sensors.find(sensorName);
                if (!firstScan && findSensor != sensors.end())
                {
                    bool found = false;
                    for (auto it = sensorsChanged->begin();
                         it != sensorsChanged->end(); it++)
                    {
                        if (findSensor->second &&
                            boost::ends_with(*it, findSensor->second->name))
                        {
                            sensorsChanged->erase(it);
                            findSensor->second = nullptr;
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        continue;
                    }
                }
                std::vector<thresholds::Threshold> sensorThresholds;
                if (!parseThresholdsFromConfig(*sensorData, sensorThresholds))
                {
                    std::cerr << "error populating thresholds for "
                              << sensorName << "\n";
                }

                auto findPowerOn = baseConfiguration->second.find("PowerState");
                PowerState readState = PowerState::always;
                if (findPowerOn != baseConfiguration->second.end())
                {
                    std::string powerState = std::visit(
                        VariantToStringVisitor(), findPowerOn->second);
                    setReadState(powerState, readState);
                }

                auto& sensor = sensors[sensorName];
                sensor = nullptr;

                sensor = std::make_shared<HostSensor>(
                    path.string(), objectServer, dbusConnection, io, sensorName,
                    std::move(sensorThresholds), *interfacePath);
            }
        }));

    getter->getConfiguration(
        std::vector<std::string>{sensorTypes.begin(), sensorTypes.end()});
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.HostSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::shared_ptr<HostSensor>>
        sensors;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    io.post([&]() {
        createSensors(io, objectServer, sensors, systemBus, nullptr);
    });

    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            sensorsChanged->insert(message.get_path());
            // this implicitly cancels the timer
            filterTimer.expires_from_now(boost::posix_time::seconds(1));

            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    /* we were canceled*/
                    return;
                }
                else if (ec)
                {
                    std::cerr << "timer error\n";
                    return;
                }
                createSensors(io, objectServer, sensors, systemBus,
                              sensorsChanged);
            });
        };

    const char* type = "xyz.openbmc_project.Configuration.HostSensor";

    auto match = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + type + "'",
        eventHandler);
    matches.emplace_back(std::move(match));

    io.run();
}
