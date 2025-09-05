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

#include "ADCSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

static constexpr float pollRateDefault = 0.5;
static constexpr float gpioBridgeSetupTimeDefault = 0.02;

static constexpr auto sensorTypes{std::to_array<std::string_view>({"ADC"})};
static std::regex inputRegex(R"(in(\d+)_input)");

static boost::container::flat_map<size_t, bool> cpuPresence;

enum class UpdateType
{
    init,
    cpuPresenceChange
};

// filter out adc from any other voltage sensor
bool isAdc(const std::filesystem::path& parentPath)
{
    std::filesystem::path namePath = parentPath / "name";

    std::ifstream nameFile(namePath);
    if (!nameFile.good())
    {
        lg2::error("Failure reading '{PATH}'", "PATH", namePath.string());
        return false;
    }

    std::string name;
    std::getline(nameFile, name);

    return name == "iio_hwmon";
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<ADCSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    UpdateType updateType)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&io, &objectServer, &sensors, &dbusConnection, sensorsChanged,
         updateType](const ManagedObjectType& sensorConfigurations) {
            bool firstScan = sensorsChanged == nullptr;
            std::vector<std::filesystem::path> paths;
            if (!findFiles(std::filesystem::path("/sys/class/hwmon"),
                           R"(in\d+_input)", paths))
            {
                lg2::error("No adc sensors in system");
                return;
            }

            // iterate through all found adc sensors, and try to match them with
            // configuration
            for (auto& path : paths)
            {
                if (!isAdc(path.parent_path()))
                {
                    continue;
                }
                std::smatch match;
                std::string pathStr = path.string();

                std::regex_search(pathStr, match, inputRegex);
                std::string indexStr = *(match.begin() + 1);

                // convert to 0 based
                size_t index = std::stoul(indexStr) - 1;

                const SensorData* sensorData = nullptr;
                const std::string* interfacePath = nullptr;
                const std::pair<std::string, SensorBaseConfigMap>*
                    baseConfiguration = nullptr;
                for (const auto& [path, cfgData] : sensorConfigurations)
                {
                    // clear it out each loop
                    baseConfiguration = nullptr;

                    // find base configuration
                    for (const std::string_view type : sensorTypes)
                    {
                        auto sensorBase =
                            cfgData.find(configInterfaceName(type));
                        if (sensorBase != cfgData.end())
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
                        lg2::error(
                            "Base configuration missing Index: '{INTERFACE}'",
                            "INTERFACE", baseConfiguration->first);
                        continue;
                    }

                    unsigned int number = std::visit(
                        VariantToUnsignedIntVisitor(), findIndex->second);

                    if (number != index)
                    {
                        continue;
                    }

                    sensorData = &cfgData;
                    interfacePath = &path.str;
                    break;
                }
                if (sensorData == nullptr)
                {
                    lg2::debug("failed to find match for '{PATH}'", "PATH",
                               path.string());
                    continue;
                }

                if (baseConfiguration == nullptr)
                {
                    lg2::error("error finding base configuration for '{PATH}'",
                               "PATH", path.string());
                    continue;
                }

                auto findSensorName = baseConfiguration->second.find("Name");
                if (findSensorName == baseConfiguration->second.end())
                {
                    lg2::error(
                        "could not determine configuration name for '{PATH}'",
                        "PATH", path.string());
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
                            it->ends_with(findSensor->second->name))
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

                auto findCPU = baseConfiguration->second.find("CPURequired");
                if (findCPU != baseConfiguration->second.end())
                {
                    size_t index =
                        std::visit(VariantToIntVisitor(), findCPU->second);
                    auto presenceFind = cpuPresence.find(index);
                    if (presenceFind == cpuPresence.end())
                    {
                        continue; // no such cpu
                    }
                    if (!presenceFind->second)
                    {
                        continue; // cpu not installed
                    }
                }
                else if (updateType == UpdateType::cpuPresenceChange)
                {
                    continue;
                }

                std::vector<thresholds::Threshold> sensorThresholds;
                if (!parseThresholdsFromConfig(*sensorData, sensorThresholds))
                {
                    lg2::error("error populating thresholds for '{NAME}'",
                               "NAME", sensorName);
                }

                auto findScaleFactor =
                    baseConfiguration->second.find("ScaleFactor");
                float scaleFactor = 1.0;
                if (findScaleFactor != baseConfiguration->second.end())
                {
                    scaleFactor = std::visit(VariantToFloatVisitor(),
                                             findScaleFactor->second);
                    // scaleFactor is used in division
                    if (scaleFactor == 0.0F)
                    {
                        scaleFactor = 1.0;
                    }
                }

                float pollRate =
                    getPollRate(baseConfiguration->second, pollRateDefault);
                PowerState readState = getPowerState(baseConfiguration->second);

                auto& sensor = sensors[sensorName];
                sensor = nullptr;

                std::optional<BridgeGpio> bridgeGpio;
                for (const auto& [key, cfgMap] : *sensorData)
                {
                    if (key.find("BridgeGpio") != std::string::npos)
                    {
                        auto findName = cfgMap.find("Name");
                        if (findName != cfgMap.end())
                        {
                            std::string gpioName = std::visit(
                                VariantToStringVisitor(), findName->second);

                            int polarity = gpiod::line::ACTIVE_HIGH;
                            auto findPolarity = cfgMap.find("Polarity");
                            if (findPolarity != cfgMap.end())
                            {
                                if (std::string("Low") ==
                                    std::visit(VariantToStringVisitor(),
                                               findPolarity->second))
                                {
                                    polarity = gpiod::line::ACTIVE_LOW;
                                }
                            }

                            float setupTime = gpioBridgeSetupTimeDefault;
                            auto findSetupTime = cfgMap.find("SetupTime");
                            if (findSetupTime != cfgMap.end())
                            {
                                setupTime = std::visit(VariantToFloatVisitor(),
                                                       findSetupTime->second);
                            }

                            bridgeGpio =
                                BridgeGpio(gpioName, polarity, setupTime);
                        }

                        break;
                    }
                }

                sensor = std::make_shared<ADCSensor>(
                    path.string(), objectServer, dbusConnection, io, sensorName,
                    std::move(sensorThresholds), scaleFactor, pollRate,
                    readState, *interfacePath, std::move(bridgeGpio));
                sensor->setupRead();
            }
        });

    std::vector<std::string_view> sensorTypesView{sensorTypes.begin(),
                                                  sensorTypes.end()};
    getter->getConfiguration(sensorTypesView);
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");

    systemBus->request_name("xyz.openbmc_project.ADCSensor");
    boost::container::flat_map<std::string, std::shared_ptr<ADCSensor>> sensors;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, sensors, systemBus, nullptr,
                      UpdateType::init);
    });

    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t& message) {
            if (message.is_method_error())
            {
                lg2::error("callback method error");
                return;
            }
            sensorsChanged->insert(message.get_path());
            // this implicitly cancels the timer
            filterTimer.expires_after(std::chrono::seconds(1));

            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    /* we were canceled*/
                    return;
                }
                if (ec)
                {
                    lg2::error("timer error");
                    return;
                }
                createSensors(io, objectServer, sensors, systemBus,
                              sensorsChanged, UpdateType::init);
            });
        };

    boost::asio::steady_timer cpuFilterTimer(io);
    std::function<void(sdbusplus::message_t&)> cpuPresenceHandler =
        [&](sdbusplus::message_t& message) {
            std::string path = message.get_path();
            boost::to_lower(path);

            sdbusplus::message::object_path cpuPath(path);
            std::string cpuName = cpuPath.filename();
            if (!cpuName.starts_with("cpu"))
            {
                return; // not interested
            }
            size_t index = 0;
            try
            {
                index = std::stoi(path.substr(path.size() - 1));
            }
            catch (const std::invalid_argument&)
            {
                lg2::error("Found invalid path: '{PATH}'", "PATH", path);
                return;
            }

            std::string objectName;
            boost::container::flat_map<std::string, std::variant<bool>> values;
            message.read(objectName, values);
            auto findPresence = values.find("Present");
            if (findPresence != values.end())
            {
                cpuPresence[index] = std::get<bool>(findPresence->second);
            }

            // this implicitly cancels the timer
            cpuFilterTimer.expires_after(std::chrono::seconds(1));

            cpuFilterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    /* we were canceled*/
                    return;
                }
                if (ec)
                {
                    lg2::error("timer error");
                    return;
                }
                createSensors(io, objectServer, sensors, systemBus, nullptr,
                              UpdateType::cpuPresenceChange);
            });
        };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(*systemBus, sensorTypes, eventHandler);
    matches.emplace_back(std::make_unique<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(cpuInventoryPath) +
            "',arg0namespace='xyz.openbmc_project.Inventory.Item'",
        cpuPresenceHandler));

    setupManufacturingModeMatch(*systemBus);
    io.run();
}
