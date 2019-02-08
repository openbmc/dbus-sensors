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

#include "filesystem.hpp"

#include <PwmSensor.hpp>
#include <TachSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

static constexpr bool DEBUG = false;

namespace fs = std::filesystem;

static constexpr std::array<const char*, 2> sensorTypes = {
    "xyz.openbmc_project.Configuration.AspeedFan",
    "xyz.openbmc_project.Configuration.I2CFan"};
constexpr const char* redundancyConfiguration =
    "xyz.openbmc_project.Configuration.FanRedundancy";
static std::regex inputRegex(R"(fan(\d+)_input)");

enum class FanTypes
{
    aspeed,
    i2c
};

// todo: power supply fan redundancy
std::shared_ptr<RedundancySensor> systemRedundancy = nullptr;

FanTypes getFanType(const fs::path& parentPath)
{
    fs::path linkPath = parentPath / "device";
    std::string canonical = fs::read_symlink(linkPath);
    if (boost::ends_with(canonical, "1e786000.pwm-tacho-controller"))
    {
        return FanTypes::aspeed;
    }
    // todo: will we need to support other types?
    return FanTypes::i2c;
}

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<TachSensor>>&
        tachSensors,
    boost::container::flat_map<std::string, std::unique_ptr<PwmSensor>>&
        pwmSensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::unique_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    bool firstScan = sensorsChanged == nullptr;
    // use new data the first time, then refresh
    ManagedObjectType sensorConfigurations;
    bool useCache = false;
    for (const char* type : sensorTypes)
    {
        if (!getSensorConfiguration(type, dbusConnection, sensorConfigurations,
                                    useCache))
        {
            std::cerr << "error communicating to entity manager\n";
            return;
        }
        useCache = true;
    }
    std::vector<fs::path> paths;
    if (!findFiles(fs::path("/sys/class/hwmon"), R"(fan\d+_input)", paths))
    {
        std::cerr << "No temperature sensors in system\n";
        return;
    }

    // iterate through all found fan sensors, and try to match them with
    // configuration
    for (const auto& path : paths)
    {
        std::smatch match;
        std::string pathStr = path.string();

        std::regex_search(pathStr, match, inputRegex);
        std::string indexStr = *(match.begin() + 1);

        auto directory = path.parent_path();
        FanTypes fanType = getFanType(directory);
        size_t bus = 0;
        size_t address = 0;
        if (fanType == FanTypes::i2c)
        {
            std::string link =
                fs::read_symlink(directory / "device").filename();

            size_t findDash = link.find("-");
            if (findDash == std::string::npos || link.size() <= findDash + 1)
            {
                std::cerr << "Error finding device from symlink";
            }
            bus = std::stoi(link.substr(0, findDash));
            address = std::stoi(link.substr(findDash + 1), nullptr, 16);
        }
        // convert to 0 based
        size_t index = std::stoul(indexStr) - 1;

        const char* baseType;
        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        const SensorBaseConfiguration* baseConfiguration = nullptr;
        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigurations)
        {
            // find the base of the configuration to see if indexes match
            for (const char* type : sensorTypes)
            {
                auto sensorBaseFind = sensor.second.find(type);
                if (sensorBaseFind != sensor.second.end())
                {
                    baseConfiguration = &(*sensorBaseFind);
                    interfacePath = &(sensor.first.str);
                    baseType = type;
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
                std::cerr << baseConfiguration->first << " missing index\n";
                continue;
            }
            unsigned int configIndex =
                std::visit(VariantToUnsignedIntVisitor(), findIndex->second);
            if (configIndex != index)
            {
                continue;
            }
            if (fanType == FanTypes::aspeed)
            {
                // there will be only 1 aspeed sensor object in sysfs, we found
                // the fan
                sensorData = &(sensor.second);
                break;
            }
            else if (baseType == "xyz.openbmc_project.Configuration.I2CFan")
            {
                auto findBus = baseConfiguration->second.find("Bus");
                auto findAddress = baseConfiguration->second.find("Address");
                if (findBus == baseConfiguration->second.end() ||
                    findAddress == baseConfiguration->second.end())
                {
                    std::cerr << baseConfiguration->first
                              << " missing bus or address\n";
                    continue;
                }
                unsigned int configBus =
                    std::visit(VariantToUnsignedIntVisitor(), findBus->second);
                unsigned int configAddress = std::visit(
                    VariantToUnsignedIntVisitor(), findAddress->second);

                if (configBus == bus && configAddress == configAddress)
                {
                    sensorData = &(sensor.second);
                    break;
                }
            }
        }
        if (sensorData == nullptr)
        {
            std::cerr << "failed to find match for " << path.string() << "\n";
            continue;
        }

        auto findSensorName = baseConfiguration->second.find("Name");
        if (findSensorName == baseConfiguration->second.end())
        {
            std::cerr << "could not determine configuration name for "
                      << path.string() << "\n";
            continue;
        }
        std::string sensorName = std::get<std::string>(findSensorName->second);
        // on rescans, only update sensors we were signaled by
        auto findSensor = tachSensors.find(sensorName);
        if (!firstScan && findSensor != tachSensors.end())
        {
            bool found = false;
            for (auto it = sensorsChanged->begin(); it != sensorsChanged->end();
                 it++)
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
            std::cerr << "error populating thresholds for " << sensorName
                      << "\n";
        }

        auto presenceConfig =
            sensorData->find(baseType + std::string(".Presence"));

        std::unique_ptr<PresenceSensor> presenceSensor(nullptr);

        // presence sensors are optional
        if (presenceConfig != sensorData->end())
        {
            auto findIndex = presenceConfig->second.find("Index");
            auto findPolarity = presenceConfig->second.find("Polarity");

            if (findIndex == presenceConfig->second.end() ||
                findPolarity == presenceConfig->second.end())
            {
                std::cerr << "Malformed Presence Configuration\n";
            }
            else
            {
                size_t index = std::get<uint64_t>(findIndex->second);
                bool inverted =
                    std::get<std::string>(findPolarity->second) == "Low";
                presenceSensor =
                    std::make_unique<PresenceSensor>(index, inverted, io);
            }
        }
        std::shared_ptr<RedundancySensor> redundancy;
        if (fanType == FanTypes::aspeed)
        {
            redundancy = systemRedundancy;
        }

        constexpr double defaultMaxReading = 25000;
        constexpr double defaultMinReading = 0;
        auto limits = std::make_pair(defaultMinReading, defaultMaxReading);

        findLimits(limits, baseConfiguration);
        tachSensors[sensorName] = std::make_unique<TachSensor>(
            path.string(), baseType, objectServer, dbusConnection,
            std::move(presenceSensor), redundancy, io, sensorName,
            std::move(sensorThresholds), *interfacePath, limits);
    }
    std::vector<fs::path> pwms;
    if (!findFiles(fs::path("/sys/class/hwmon"), R"(pwm\d+)", pwms))
    {
        std::cerr << "No pwm in system\n";
        return;
    }
    for (const fs::path& pwm : pwms)
    {
        if (pwmSensors.find(pwm) != pwmSensors.end())
        {
            continue;
        }
        // only add new elements
        pwmSensors.insert(std::pair<std::string, std::unique_ptr<PwmSensor>>(
            pwm.string(),
            std::make_unique<PwmSensor>(pwm.string(), objectServer)));
    }
}

void createRedundancySensor(
    const boost::container::flat_map<std::string, std::unique_ptr<TachSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    sdbusplus::asio::object_server& objectServer)
{

    conn->async_method_call(
        [&objectServer, &sensors](boost::system::error_code& ec,
                                  const ManagedObjectType managedObj) {
            if (ec)
            {
                std::cerr << "Error calling entity manager \n";
                return;
            }
            for (const auto& pathPair : managedObj)
            {
                for (const auto& interfacePair : pathPair.second)
                {
                    if (interfacePair.first == redundancyConfiguration)
                    {
                        // currently only support one
                        auto findCount =
                            interfacePair.second.find("AllowedFailures");
                        if (findCount == interfacePair.second.end())
                        {
                            std::cerr << "Malformed redundancy record \n";
                            return;
                        }
                        std::vector<std::string> sensorList;

                        for (const auto& sensor : sensors)
                        {
                            sensorList.push_back(
                                "/xyz/openbmc_project/sensors/fan_tach/" +
                                sensor.second->name);
                        }
                        systemRedundancy = std::make_unique<RedundancySensor>(
                            std::get<uint64_t>(findCount->second), sensorList,
                            objectServer);

                        return;
                    }
                }
            }
        },
        "xyz.openbmc_project.EntityManager", "/",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
}

int main(int argc, char** argv)
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.FanSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::unique_ptr<TachSensor>>
        tachSensors;
    boost::container::flat_map<std::string, std::unique_ptr<PwmSensor>>
        pwmSensors;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    std::unique_ptr<boost::container::flat_set<std::string>> sensorsChanged =
        std::make_unique<boost::container::flat_set<std::string>>();

    io.post([&]() {
        createSensors(io, objectServer, tachSensors, pwmSensors, systemBus,
                      nullptr);
        createRedundancySensor(tachSensors, systemBus, objectServer);
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
                createSensors(io, objectServer, tachSensors, pwmSensors,
                              systemBus, sensorsChanged);
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

    // redundancy sensor
    std::function<void(sdbusplus::message::message&)> redundancyHandler =
        [&tachSensors, &systemBus,
         &objectServer](sdbusplus::message::message& message) {
            createRedundancySensor(tachSensors, systemBus, objectServer);
        };
    auto match = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" +
            redundancyConfiguration + "'",
        redundancyHandler);
    matches.emplace_back(std::move(match));

    io.run();
}
