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

#include <ADCSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
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

static constexpr float scaleFactorDefault = 1.0;
static constexpr float pollRateDefault = 0.5;
static constexpr float gpioBridgeSetupTimeDefault = 0.02;
static std::regex inputRegex(R"(in(\d+)_input)");

static constexpr auto supportedDrivers{
    std::to_array<const char*>({"iio_hwmon", "ads7828"})};

static constexpr auto sensorTypes{
    std::to_array<const char*>({"xyz.openbmc_project.Configuration.ADC",
                                "xyz.openbmc_project.Configuration.I2CADC"})};

namespace fs = std::filesystem;

static boost::container::flat_map<size_t, bool> cpuPresence;

enum class UpdateType
{
    init,
    cpuPresenceChange
};

static void createSensorsCallback(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const ManagedObjectType& sensorConfigurations,
    boost::container::flat_map<std::string, std::shared_ptr<ADCSensor>>&
        sensors,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    UpdateType updateType)
{
    bool firstScan = sensorsChanged == nullptr;

    std::vector<fs::path> hwmonPaths;
    std::vector<fs::path> inputPaths;

    if (!findFiles(fs::path("/sys/class/hwmon"), "name", hwmonPaths))
    {
        std::cerr << "No hwmon sensors in system\n";
        return;
    }

    boost::container::flat_set<std::string> directories;
    for (const auto& hwmonPath : hwmonPaths)
    {
        std::ifstream nameFile(hwmonPath);
        if (!nameFile.good())
        {
            std::cerr << "Failure finding hwmon path " << hwmonPath << "\n";
            continue;
        }
        std::string driverName;
        std::getline(nameFile, driverName);
        nameFile.close();

        if (std::find(supportedDrivers.begin(), supportedDrivers.end(),
                      driverName) == supportedDrivers.end())
        {
            continue;
        }

        auto directory = hwmonPath.parent_path();

        auto ret = directories.insert(directory.string());
        if (!ret.second)
        {
            std::cerr << "Duplicate path " << directory.string() << "\n";
            continue;
        }

        fs::path device = directory / "device";
        std::string deviceName = fs::canonical(device).stem();
        auto findHyphen = deviceName.find('-');
        if (findHyphen == std::string::npos)
        {
            std::cerr << "found bad device" << deviceName << "\n";
            continue;
        }

        std::optional<size_t> bus;
        std::optional<size_t> addr;

        if (deviceName != "iio-hwmon")
        {
            std::string busStr = deviceName.substr(0, findHyphen);
            std::string addrStr = deviceName.substr(findHyphen + 1);
            try
            {
                bus = std::stoi(busStr);
                addr = std::stoi(addrStr, nullptr, 16);
            }
            catch (const std::invalid_argument&)
            {
                std::cerr << "Error parsing bus " << busStr << " addr "
                          << addrStr << "\n";
                continue;
            }
        }

        inputPaths.clear();
        if (!findFiles(directory, R"(in\d+_input)", inputPaths))
        {
            std::cerr << "No hwmon voltage sensors in the path: " << directory
                      << "\n";
            return;
        }

        for (const auto& inputPath : inputPaths)
        {
            std::smatch match;
            std::string inputPathStr = inputPath.string();
            std::regex_search(inputPathStr, match, inputRegex);
            int hwmonInputIndex = 0;

            try
            {
                hwmonInputIndex = std::stoul(std::string(match[1]));

                // Maintain backward compatibility with already existing
                // Entity-Manager configs that use indexing starting from 0 for
                // iio-hwmon sensors
                if (deviceName == "iio-hwmon")
                {
                    hwmonInputIndex -= 1;
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                continue;
            }
            const std::pair<std::string, boost::container::flat_map<
                                             std::string, BasicVariantType>>*
                baseConfig = nullptr;
            const SensorData* sensorData = nullptr;
            const std::string* interfacePath = nullptr;
            const char* sensorType = nullptr;

            for (const std::pair<sdbusplus::message::object_path, SensorData>&
                     sensor : sensorConfigurations)
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
                auto inputIndexFound = baseConfig->second.find("Index");
                if (inputIndexFound == baseConfig->second.end())
                {
                    std::cerr << "Base configuration missing Index"
                              << baseConfig->first << "\n";
                    continue;
                }

                int inputIndex =
                    std::visit(VariantToIntVisitor(), inputIndexFound->second);

                if (hwmonInputIndex != inputIndex)
                {
                    continue;
                }

                if (std::string(sensorType) ==
                    std::string("xyz.openbmc_project.Configuration.I2CADC"))
                {
                    if (!bus || !addr)
                    {
                        std::cerr
                            << "Skipping config entry because either bus or "
                               "addr not found on fs\n";
                        continue;
                    }

                    auto configBus = baseConfig->second.find("Bus");
                    auto configAddress = baseConfig->second.find("Address");

                    if (configBus == baseConfig->second.end() ||
                        configAddress == baseConfig->second.end())
                    {
                        std::cerr << "error finding necessary entry in "
                                     "configuration\n";
                        continue;
                    }

                    const uint64_t* confBus =
                        std::get_if<uint64_t>(&(configBus->second));
                    const uint64_t* confAddr =
                        std::get_if<uint64_t>(&(configAddress->second));
                    if (confBus == nullptr || confAddr == nullptr)
                    {
                        std::cerr << "Cannot get bus or address, invalid "
                                     "configuration\n";
                        continue;
                    }

                    if ((*confBus != *bus) || (*confAddr != *addr))
                    {
                        std::cerr << "Configuration skipping " << *confBus
                                  << "-" << *confAddr << " because not " << *bus
                                  << "-" << *addr << "\n";
                        continue;
                    }
                }
                interfacePath = &(sensor.first.str);
                break;
            }
            if (interfacePath == nullptr)
            {
                // To avoid this error message, add your export map entry,
                // from Entity Manager, to sensorTypes at the top of this file.
                std::cerr << "failed to find match for " << deviceName << "\n";
                continue;
            }
            auto findSensorName = baseConfig->second.find("Name");
            if (findSensorName == baseConfig->second.end())
            {
                std::cerr << "could not determine configuration name for "
                          << inputPath.string() << "\n";
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

            auto findCPU = baseConfig->second.find("CPURequired");
            if (findCPU != baseConfig->second.end())
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
                std::cerr << "error populating thresholds for " << sensorName
                          << "\n";
            }

            auto findScaleFactor = baseConfig->second.find("ScaleFactor");
            float scaleFactor = scaleFactorDefault;
            if (findScaleFactor != baseConfig->second.end())
            {
                scaleFactor = std::visit(VariantToFloatVisitor(),
                                         findScaleFactor->second);
                // scaleFactor is used in division
                if (scaleFactor == 0.0f)
                {
                    scaleFactor = scaleFactorDefault;
                }
            }

            auto findPollRate = baseConfig->second.find("PollRate");
            float pollRate = pollRateDefault;
            if (findPollRate != baseConfig->second.end())
            {
                pollRate =
                    std::visit(VariantToFloatVisitor(), findPollRate->second);
                if (pollRate <= 0.0f)
                {
                    pollRate = pollRateDefault; // polling time too short
                }
            }

            auto findPowerOn = baseConfig->second.find("PowerState");
            PowerState readState = PowerState::always;
            if (findPowerOn != baseConfig->second.end())
            {
                std::string powerState =
                    std::visit(VariantToStringVisitor(), findPowerOn->second);
                setReadState(powerState, readState);
            }

            auto& sensor = sensors[sensorName];
            sensor = nullptr;

            std::optional<BridgeGpio> bridgeGpio;
            for (const SensorBaseConfiguration& suppConfig : *sensorData)
            {
                if (suppConfig.first.find("BridgeGpio") != std::string::npos)
                {
                    auto findName = suppConfig.second.find("Name");
                    if (findName != suppConfig.second.end())
                    {
                        std::string gpioName = std::visit(
                            VariantToStringVisitor(), findName->second);

                        int polarity = gpiod::line::ACTIVE_HIGH;
                        auto findPolarity = suppConfig.second.find("Polarity");
                        if (findPolarity != suppConfig.second.end())
                        {
                            if (std::string("Low") ==
                                std::visit(VariantToStringVisitor(),
                                           findPolarity->second))
                            {
                                polarity = gpiod::line::ACTIVE_LOW;
                            }
                        }

                        float setupTime = gpioBridgeSetupTimeDefault;
                        auto findSetupTime =
                            suppConfig.second.find("SetupTime");
                        if (findSetupTime != suppConfig.second.end())
                        {
                            setupTime = std::visit(VariantToFloatVisitor(),
                                                   findSetupTime->second);
                        }

                        bridgeGpio = BridgeGpio(gpioName, polarity, setupTime);
                    }

                    break;
                }
            }
            sensor = std::make_shared<ADCSensor>(
                inputPath.string(), objectServer, dbusConnection, io,
                sensorName, std::move(sensorThresholds), scaleFactor, pollRate,
                readState, *interfacePath, std::move(bridgeGpio));
            sensor->setupRead();
        }
    }
}

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
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
            createSensorsCallback(io, objectServer, dbusConnection,
                                  sensorConfigurations, sensors, sensorsChanged,
                                  updateType);
        });

    getter->getConfiguration(
        std::vector<std::string>{sensorTypes.begin(), sensorTypes.end()});
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.ADCSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::shared_ptr<ADCSensor>> sensors;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    io.post([&]() {
        createSensors(io, objectServer, sensors, systemBus, nullptr,
                      UpdateType::init);
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
                if (ec)
                {
                    std::cerr << "timer error\n";
                    return;
                }
                createSensors(io, objectServer, sensors, systemBus,
                              sensorsChanged, UpdateType::init);
            });
        };

    std::function<void(sdbusplus::message::message&)> cpuPresenceHandler =
        [&](sdbusplus::message::message& message) {
            std::string path = message.get_path();
            boost::to_lower(path);

            if (path.rfind("cpu") == std::string::npos)
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
                std::cerr << "Found invalid path " << path << "\n";
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
            filterTimer.expires_from_now(boost::posix_time::seconds(1));

            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    /* we were canceled*/
                    return;
                }
                if (ec)
                {
                    std::cerr << "timer error\n";
                    return;
                }
                createSensors(io, objectServer, sensors, systemBus, nullptr,
                              UpdateType::cpuPresenceChange);
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
    matches.emplace_back(std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(cpuInventoryPath) +
            "',arg0namespace='xyz.openbmc_project.Inventory.Item'",
        cpuPresenceHandler));

    setupManufacturingModeMatch(*systemBus);
    io.run();
}
