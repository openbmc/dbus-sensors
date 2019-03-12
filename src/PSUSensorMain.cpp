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

#include "filesystem.hpp"

#include <PSUSensor.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_set.hpp>
#include <fstream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

static constexpr std::array<const char*, 1> sensorTypes = {
    "xyz.openbmc_project.Configuration.pmbus"};

namespace fs = std::filesystem;

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    boost::container::flat_map<std::string, std::unique_ptr<PSUSensor>>&
        sensors,
    boost::container::flat_map<SensorType, std::unique_ptr<PSUProperty>>&
        sensorTable,
    boost::container::flat_map<std::string, std::string>& labelMatch)
{

    ManagedObjectType sensorConfigs;
    bool useCache = false;

    for (const char* type : sensorTypes)
    {
        if (!getSensorConfiguration(type, dbusConnection, sensorConfigs,
                                    useCache))
        {
            std::cerr << "error get sensor config from entity manager\n";
            return;
        }
        useCache = true;
    }

    std::vector<fs::path> pmbusPaths;
    if (!findFiles(fs::path("/sys/class/hwmon"), "name", pmbusPaths))
    {
        std::cerr << "No PSU sensors in system\n";
        return;
    }

    boost::container::flat_set<std::string> directories;
    for (const auto& pmbusPath : pmbusPaths)
    {
        const std::string pathStr = pmbusPath.string();
        auto directory = pmbusPath.parent_path();

        auto ret = directories.insert(directory.string());
        if (!ret.second)
        {
            continue; // check if path i1 already searched
        }

        auto device = fs::path(directory / "device");
        std::string deviceName = fs::canonical(device).stem();
        auto findHyphen = deviceName.find("-");
        if (findHyphen == std::string::npos)
        {
            std::cerr << "found bad device" << deviceName << "\n";
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
        catch (std::invalid_argument)
        {
            continue;
        }

        std::ifstream nameFile(pmbusPath);
        if (!nameFile.good())
        {
            std::cerr << "Failure reading " << pmbusPath << "\n";
            continue;
        }

        std::string pmbusName;
        std::getline(nameFile, pmbusName);
        nameFile.close();
        if (pmbusName != "pmbus")
        {
            continue;
        }

        const std::pair<std::string, boost::container::flat_map<
                                         std::string, BasicVariantType>>*
            baseConfig = nullptr;
        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        const char* sensorType = nullptr;

        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigs)
        {
            sensorData = &(sensor.second);
            for (const char* type : sensorTypes)
            {
                auto sensorBase = sensorData->find(type);
                if (sensorBase != sensorData->end())
                {
                    baseConfig = &(*sensorBase);
                    sensorType = type;
                    break;
                }
            }
            if (baseConfig == nullptr)
            {
                std::cerr << "error finding base configuration for "
                          << deviceName << "\n";
                continue;
            }

            auto configBus = baseConfig->second.find("Bus");
            auto configAddress = baseConfig->second.find("Address");

            if (configBus == baseConfig->second.end() ||
                configAddress == baseConfig->second.end())
            {
                std::cerr << "error finding necessary entry in configuration";
                continue;
            }

            if (std::get<uint64_t>(configBus->second) != bus ||
                std::get<uint64_t>(configAddress->second) != addr)
            {
                continue;
            }

            interfacePath = &(sensor.first.str);
            break;
        }
        if (interfacePath == nullptr)
        {
            std::cerr << "failed to find match for " << deviceName << "\n";
            continue;
        }

        auto findSensorName = baseConfig->second.find("Name");
        if (findSensorName == baseConfig->second.end())
        {
            std::cerr << "could not determine configuration name for "
                      << deviceName << "\n";
            continue;
        }

        std::vector<fs::path> powerPaths;
        if (!findFiles(fs::path(directory), R"(power\d+_input$)", powerPaths,
                       0))
        {
            std::cerr << "No power sensor in PSU\n";
            continue;
        }

        for (const auto& powerPath : powerPaths)
        {
            auto powerPathStr = powerPath.string();
            auto labelPath =
                boost::replace_all_copy(powerPathStr, "input", "label");
            std::ifstream labelFile(labelPath);
            if (!labelFile.good())
            {
                std::cerr << "Failure reading " << powerPath << "\n";
                continue;
            }
            std::string label;
            std::getline(labelFile, label);
            labelFile.close();

            auto findSensor = sensors.find(label);
            if (findSensor != sensors.end())
            {
                continue;
            }

            std::vector<thresholds::Threshold> sensorThresholds;
            std::string labelHead = label.substr(0, label.find(" "));
            parseThresholdsFromConfig(*sensorData, sensorThresholds,
                                      &labelHead);
            if (sensorThresholds.empty())
            {
                continue;
            }

            std::string labelName;
            auto findLabel = labelMatch.find(label);
            if (findLabel != labelMatch.end())
            {
                labelName = findLabel->second;
            }
            else
            {
                labelName = label;
            }
            std::string sensorName =
                std::get<std::string>(findSensorName->second) + " " + labelName;

            auto findProperty = sensorTable.find(SensorType::powerSensor);
            if (findProperty == sensorTable.end())
            {
                std::cerr << "Cannot find PSU sensorType " << sensorType
                          << "\n";
                continue;
            }

            sensors[sensorName] = std::make_unique<PSUSensor>(
                powerPathStr, sensorType, objectServer, dbusConnection, io,
                sensorName, std::move(sensorThresholds), *interfacePath,
                findProperty->second->sensorTypeName,
                findProperty->second->sensorScaleFactor,
                findProperty->second->maxReading,
                findProperty->second->minReading);
        }
    }
    return;
}

void propertyInitialize(
    boost::container::flat_map<SensorType, std::unique_ptr<PSUProperty>>&
        sensorTable,
    boost::container::flat_map<std::string, std::string>& labelMatch)
{
    sensorTable[SensorType::powerSensor] =
        std::make_unique<PSUProperty>("power/", 65535, 0, 100000);
    labelMatch["pin"] = "Input Power";
}

int main(int argc, char** argv)
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);

    systemBus->request_name("xyz.openbmc_project.PSUSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::unique_ptr<PSUSensor>> sensors;
    boost::container::flat_map<SensorType, std::unique_ptr<PSUProperty>>
        sensorTable;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    boost::container::flat_map<std::string, std::string> labelMatch;

    propertyInitialize(sensorTable, labelMatch);

    io.post([&]() {
        createSensors(io, objectServer, systemBus, sensors, sensorTable,
                      labelMatch);
    });
    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            filterTimer.expires_from_now(boost::posix_time::seconds(1));
            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return;
                }
                else if (ec)
                {
                    std::cerr << "timer error\n";
                }
                createSensors(io, objectServer, systemBus, sensors, sensorTable,
                              labelMatch);
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
