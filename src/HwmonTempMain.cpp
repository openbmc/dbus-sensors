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

#include "HwmonTempSensor.hpp"
#include "Utils.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_set.hpp>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

static constexpr bool DEBUG = false;

namespace fs = std::filesystem;
static constexpr std::array<const char*, 7> sensorTypes = {
    "xyz.openbmc_project.Configuration.TMP75",
    "xyz.openbmc_project.Configuration.TMP421",
    "xyz.openbmc_project.Configuration.TMP441",
    "xyz.openbmc_project.Configuration.TMP112",
    "xyz.openbmc_project.Configuration.TMP175",
    "xyz.openbmc_project.Configuration.EMC1413",
    "xyz.openbmc_project.Configuration.MAX31725"};

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<HwmonTempSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::unique_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        std::move([&io, &objectServer, &sensors, &dbusConnection,
                   &sensorsChanged](
                      const ManagedObjectType& sensorConfigurations) {
            bool firstScan = sensorsChanged == nullptr;

            std::vector<fs::path> paths;
            if (!findFiles(fs::path("/sys/class/hwmon"), R"(temp\d+_input)",
                           paths))
            {
                std::cerr << "No temperature sensors in system\n";
                return;
            }

            boost::container::flat_set<std::string> directories;

            // iterate through all found temp sensors, and try to match them
            // with configuration
            for (auto& path : paths)
            {
                std::smatch match;
                const std::string& pathStr = path.string();
                auto directory = path.parent_path();

                auto ret = directories.insert(directory.string());
                if (!ret.second)
                {
                    continue; // already searched this path
                }

                fs::path device = directory / "device";
                std::string deviceName = fs::canonical(device).stem();
                auto findHyphen = deviceName.find("-");
                if (findHyphen == std::string::npos)
                {
                    std::cerr << "found bad device " << deviceName << "\n";
                    continue;
                }
                std::string busStr = deviceName.substr(0, findHyphen);
                std::string addrStr = deviceName.substr(findHyphen + 1);

                size_t bus = 0;
                size_t addr = 0;
                try
                {
                    bus = std::stoi(busStr);
                    addr = std::stoi(addrStr, 0, 16);
                }
                catch (std::invalid_argument&)
                {
                    continue;
                }
                const SensorData* sensorData = nullptr;
                const std::string* interfacePath = nullptr;
                const char* sensorType = nullptr;
                const std::pair<
                    std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
                    baseConfiguration = nullptr;

                for (const std::pair<sdbusplus::message::object_path,
                                     SensorData>& sensor : sensorConfigurations)
                {
                    sensorData = &(sensor.second);
                    for (const char* type : sensorTypes)
                    {
                        auto sensorBase = sensorData->find(type);
                        if (sensorBase != sensorData->end())
                        {
                            baseConfiguration = &(*sensorBase);
                            sensorType = type;
                            break;
                        }
                    }
                    if (baseConfiguration == nullptr)
                    {
                        std::cerr << "error finding base configuration for "
                                  << deviceName << "\n";
                        continue;
                    }
                    auto configurationBus =
                        baseConfiguration->second.find("Bus");
                    auto configurationAddress =
                        baseConfiguration->second.find("Address");

                    if (configurationBus == baseConfiguration->second.end() ||
                        configurationAddress == baseConfiguration->second.end())
                    {
                        std::cerr
                            << "error finding bus or address in configuration";
                        continue;
                    }

                    if (std::get<uint64_t>(configurationBus->second) != bus ||
                        std::get<uint64_t>(configurationAddress->second) !=
                            addr)
                    {
                        continue;
                    }

                    interfacePath = &(sensor.first.str);
                    break;
                }
                if (interfacePath == nullptr)
                {
                    std::cerr << "failed to find match for " << deviceName
                              << "\n";
                    continue;
                }

                auto findSensorName = baseConfiguration->second.find("Name");
                if (findSensorName == baseConfiguration->second.end())
                {
                    std::cerr << "could not determine configuration name for "
                              << deviceName << "\n";
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
                        if (boost::ends_with(*it, findSensor->second->name))
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
                auto& sensor1 = sensors[sensorName];
                sensor1 = nullptr;
                sensor1 = std::make_unique<HwmonTempSensor>(
                    directory.string() + "/temp1_input", sensorType,
                    objectServer, dbusConnection, io, sensorName,
                    std::move(sensorThresholds), *interfacePath);
                auto findSecondName = baseConfiguration->second.find("Name1");
                if (findSecondName == baseConfiguration->second.end())
                {
                    continue;
                }
                sensorName = std::get<std::string>(findSecondName->second);
                auto& sensor2 = sensors[sensorName];
                sensor2 = nullptr;
                sensor2 = std::make_unique<HwmonTempSensor>(
                    directory.string() + "/temp2_input", sensorType,
                    objectServer, dbusConnection, io, sensorName,
                    std::vector<thresholds::Threshold>(), *interfacePath);
            }
        }));
    getter->getConfiguration(
        std::vector<std::string>(sensorTypes.begin(), sensorTypes.end()));
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.HwmonTempSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::unique_ptr<HwmonTempSensor>>
        sensors;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    std::unique_ptr<boost::container::flat_set<std::string>> sensorsChanged =
        std::make_unique<boost::container::flat_set<std::string>>();

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

    for (const char* type : sensorTypes)
    {
        auto match = std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*systemBus),
            "type='signal',member='PropertiesChanged',path_namespace='" +
                std::string(inventoryPath) + "',arg0namespace='" + type + "'",
            eventHandler);
        matches.emplace_back(std::move(match));
    }

    io.run();
}
