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

#include <PwmSensor.hpp>
#include <TachSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace fs = std::filesystem;

// The following two structures need to be consistent
static auto sensorTypes{std::to_array<const char*>(
    {"xyz.openbmc_project.Configuration.AspeedFan",
     "xyz.openbmc_project.Configuration.I2CFan",
     "xyz.openbmc_project.Configuration.NuvotonFan"})};

enum FanTypes
{
    aspeed = 0,
    i2c,
    nuvoton,
    max,
};

static_assert(std::tuple_size<decltype(sensorTypes)>::value == FanTypes::max,
              "sensorTypes element number is not equal to FanTypes number");

constexpr const char* redundancyConfiguration =
    "xyz.openbmc_project.Configuration.FanRedundancy";
static std::regex inputRegex(R"(fan(\d+)_input)");

// todo: power supply fan redundancy
std::optional<RedundancySensor> systemRedundancy;

FanTypes getFanType(const fs::path& parentPath)
{
    fs::path linkPath = parentPath / "device";
    std::string canonical = fs::read_symlink(linkPath);
    if (boost::ends_with(canonical, "pwm-tacho-controller") ||
        boost::ends_with(canonical, "pwm_tach:tach"))
    {
        return FanTypes::aspeed;
    }
    if (boost::ends_with(canonical, "pwm-fan-controller"))
    {
        return FanTypes::nuvoton;
    }
    // todo: will we need to support other types?
    return FanTypes::i2c;
}
void enablePwm(const fs::path& filePath)
{
    std::fstream enableFile(filePath, std::ios::in | std::ios::out);
    if (!enableFile.good())
    {
        std::cerr << "Error read/write " << filePath << "\n";
        return;
    }

    std::string regulateMode;
    std::getline(enableFile, regulateMode);
    if (regulateMode == "0")
    {
        enableFile << 1;
    }
}
bool findPwmfanPath(unsigned int configPwmfanIndex, fs::path& pwmPath)
{
    /* Search PWM since pwm-fan had separated
     * PWM from tach directory and 1 channel only*/
    std::vector<fs::path> pwmfanPaths;
    std::string pwnfanDevName("pwm-fan");

    pwnfanDevName += std::to_string(configPwmfanIndex);

    if (!findFiles(fs::path("/sys/class/hwmon"), R"(pwm\d+)", pwmfanPaths))
    {
        std::cerr << "No PWMs are found!\n";
        return false;
    }
    for (const auto& path : pwmfanPaths)
    {
        std::error_code ec;
        fs::path link = fs::read_symlink(path.parent_path() / "device", ec);

        if (ec)
        {
            std::cerr << "read_symlink() failed: " << ec.message() << " ("
                      << ec.value() << ")\n";
            continue;
        }

        if (link.filename().string() == pwnfanDevName)
        {
            pwmPath = path;
            return true;
        }
    }
    return false;
}
bool findPwmPath(const fs::path& directory, unsigned int pwm, fs::path& pwmPath)
{
    std::error_code ec;

    /* Assuming PWM file is appeared in the same directory as fanX_input */
    auto path = directory / ("pwm" + std::to_string(pwm + 1));
    bool exists = fs::exists(path, ec);

    if (ec || !exists)
    {
        /* PWM file not exist or error happened */
        if (ec)
        {
            std::cerr << "exists() failed: " << ec.message() << " ("
                      << ec.value() << ")\n";
        }
        /* try search form pwm-fanX directory */
        return findPwmfanPath(pwm, pwmPath);
    }

    pwmPath = path;
    return true;
}
void createRedundancySensor(
    const boost::container::flat_map<std::string, std::shared_ptr<TachSensor>>&
        sensors,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    sdbusplus::asio::object_server& objectServer)
{

    conn->async_method_call(
        [&objectServer, &sensors](boost::system::error_code& ec,
                                  const ManagedObjectType& managedObj) {
        if (ec)
        {
            std::cerr << "Error calling entity manager \n";
            return;
        }
        for (const auto& [path, interfaces] : managedObj)
        {
            for (const auto& [intf, cfg] : interfaces)
            {
                if (intf == redundancyConfiguration)
                {
                    // currently only support one
                    auto findCount = cfg.find("AllowedFailures");
                    if (findCount == cfg.end())
                    {
                        std::cerr << "Malformed redundancy record \n";
                        return;
                    }
                    std::vector<std::string> sensorList;

                    for (const auto& [name, sensor] : sensors)
                    {
                        sensorList.push_back(
                            "/xyz/openbmc_project/sensors/fan_tach/" +
                            sensor->name);
                    }
                    systemRedundancy.reset();
                    systemRedundancy.emplace(
                        RedundancySensor(std::get<uint64_t>(findCount->second),
                                         sensorList, objectServer, path));

                    return;
                }
            }
        }
        },
        "xyz.openbmc_project.EntityManager", "/",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
}

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<TachSensor>>&
        tachSensors,
    boost::container::flat_map<std::string, std::unique_ptr<PwmSensor>>&
        pwmSensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    size_t retries = 0)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&io, &objectServer, &tachSensors, &pwmSensors, &dbusConnection,
         sensorsChanged](const ManagedObjectType& sensorConfigurations) {
        bool firstScan = sensorsChanged == nullptr;
        std::vector<fs::path> paths;
        if (!findFiles(fs::path("/sys/class/hwmon"), R"(fan\d+_input)", paths))
        {
            std::cerr << "No fan sensors in system\n";
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

            fs::path directory = path.parent_path();
            FanTypes fanType = getFanType(directory);

            // convert to 0 based
            size_t index = std::stoul(indexStr) - 1;

            const char* baseType = nullptr;
            const SensorData* sensorData = nullptr;
            const std::string* interfacePath = nullptr;
            const SensorBaseConfiguration* baseConfiguration = nullptr;
            for (const auto& [path, cfgData] : sensorConfigurations)
            {
                // find the base of the configuration to see if indexes
                // match
                auto sensorBaseFind = cfgData.find(sensorTypes[fanType]);
                if (sensorBaseFind == cfgData.end())
                {
                    continue;
                }

                baseConfiguration = &(*sensorBaseFind);
                interfacePath = &path.str;
                baseType = sensorTypes[fanType];

                auto findIndex = baseConfiguration->second.find("Index");
                if (findIndex == baseConfiguration->second.end())
                {
                    std::cerr << baseConfiguration->first << " missing index\n";
                    continue;
                }
                unsigned int configIndex = std::visit(
                    VariantToUnsignedIntVisitor(), findIndex->second);
                if (configIndex != index)
                {
                    continue;
                }
                if (fanType == FanTypes::aspeed || fanType == FanTypes::nuvoton)
                {
                    // there will be only 1 aspeed or nuvoton sensor object
                    // in sysfs, we found the fan
                    sensorData = &cfgData;
                    break;
                }
                if (fanType == FanTypes::i2c)
                {
                    size_t bus = 0;
                    size_t address = 0;

                    std::string link =
                        fs::read_symlink(directory / "device").filename();

                    size_t findDash = link.find('-');
                    if (findDash == std::string::npos ||
                        link.size() <= findDash + 1)
                    {
                        std::cerr << "Error finding device from symlink";
                    }
                    bus = std::stoi(link.substr(0, findDash));
                    address = std::stoi(link.substr(findDash + 1), nullptr, 16);

                    auto findBus = baseConfiguration->second.find("Bus");
                    auto findAddress =
                        baseConfiguration->second.find("Address");
                    if (findBus == baseConfiguration->second.end() ||
                        findAddress == baseConfiguration->second.end())
                    {
                        std::cerr << baseConfiguration->first
                                  << " missing bus or address\n";
                        continue;
                    }
                    unsigned int configBus = std::visit(
                        VariantToUnsignedIntVisitor(), findBus->second);
                    unsigned int configAddress = std::visit(
                        VariantToUnsignedIntVisitor(), findAddress->second);

                    if (configBus == bus && configAddress == address)
                    {
                        sensorData = &cfgData;
                        break;
                    }
                }
            }
            if (sensorData == nullptr)
            {
                std::cerr << "failed to find match for " << path.string()
                          << "\n";
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
            auto findSensor = tachSensors.find(sensorName);
            if (!firstScan && findSensor != tachSensors.end())
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
                std::cerr << "error populating thresholds for " << sensorName
                          << "\n";
            }

            auto presenceConfig =
                sensorData->find(baseType + std::string(".Presence"));

            std::unique_ptr<PresenceSensor> presenceSensor(nullptr);

            // presence sensors are optional
            if (presenceConfig != sensorData->end())
            {
                auto findPolarity = presenceConfig->second.find("Polarity");
                auto findPinName = presenceConfig->second.find("PinName");

                if (findPinName == presenceConfig->second.end() ||
                    findPolarity == presenceConfig->second.end())
                {
                    std::cerr << "Malformed Presence Configuration\n";
                }
                else
                {
                    bool inverted =
                        std::get<std::string>(findPolarity->second) == "Low";
                    if (const auto* pinName =
                            std::get_if<std::string>(&findPinName->second))
                    {
                        presenceSensor = std::make_unique<PresenceSensor>(
                            *pinName, inverted, io, sensorName);
                    }
                    else
                    {
                        std::cerr << "Malformed Presence pinName for sensor "
                                  << sensorName << " \n";
                    }
                }
            }
            std::optional<RedundancySensor>* redundancy = nullptr;
            if (fanType == FanTypes::aspeed)
            {
                redundancy = &systemRedundancy;
            }

            PowerState powerState = getPowerState(baseConfiguration->second);

            constexpr double defaultMaxReading = 25000;
            constexpr double defaultMinReading = 0;
            std::pair<double, double> limits =
                std::make_pair(defaultMinReading, defaultMaxReading);

            auto connector =
                sensorData->find(baseType + std::string(".Connector"));

            std::optional<std::string> led;
            std::string pwmName;
            fs::path pwmPath;

            // The Mutable parameter is optional, defaulting to false
            bool isValueMutable = false;
            if (connector != sensorData->end())
            {
                auto findPwm = connector->second.find("Pwm");
                if (findPwm != connector->second.end())
                {
                    size_t pwm = std::visit(VariantToUnsignedIntVisitor(),
                                            findPwm->second);
                    if (!findPwmPath(directory, pwm, pwmPath))
                    {
                        std::cerr << "Connector for " << sensorName
                                  << " no pwm channel found!\n";
                        continue;
                    }

                    fs::path pwmEnableFile =
                        "pwm" + std::to_string(pwm + 1) + "_enable";
                    fs::path enablePath = pwmPath.parent_path() / pwmEnableFile;
                    enablePwm(enablePath);

                    /* use pwm name override if found in configuration else
                     * use default */
                    auto findOverride = connector->second.find("PwmName");
                    if (findOverride != connector->second.end())
                    {
                        pwmName = std::visit(VariantToStringVisitor(),
                                             findOverride->second);
                    }
                    else
                    {
                        pwmName = "Pwm_" + std::to_string(pwm + 1);
                    }

                    // Check PWM sensor mutability
                    auto findMutable = connector->second.find("Mutable");
                    if (findMutable != connector->second.end())
                    {
                        const auto* ptrMutable =
                            std::get_if<bool>(&(findMutable->second));
                        if (ptrMutable != nullptr)
                        {
                            isValueMutable = *ptrMutable;
                        }
                    }
                }
                else
                {
                    std::cerr << "Connector for " << sensorName
                              << " missing pwm!\n";
                }

                auto findLED = connector->second.find("LED");
                if (findLED != connector->second.end())
                {
                    const auto* ledName =
                        std::get_if<std::string>(&(findLED->second));
                    if (ledName == nullptr)
                    {
                        std::cerr << "Wrong format for LED of " << sensorName
                                  << "\n";
                    }
                    else
                    {
                        led = *ledName;
                    }
                }
            }

            findLimits(limits, baseConfiguration);

            auto& tachSensor = tachSensors[sensorName];
            tachSensor = nullptr;
            tachSensor = std::make_shared<TachSensor>(
                path.string(), baseType, objectServer, dbusConnection,
                std::move(presenceSensor), redundancy, io, sensorName,
                std::move(sensorThresholds), *interfacePath, limits, powerState,
                led);
            tachSensor->setupRead();

            if (!pwmPath.empty() && fs::exists(pwmPath) &&
                (pwmSensors.count(pwmPath) == 0U))
            {
                pwmSensors[pwmPath] = std::make_unique<PwmSensor>(
                    pwmName, pwmPath, dbusConnection, objectServer,
                    *interfacePath, "Fan", isValueMutable);
            }
        }

        createRedundancySensor(tachSensors, dbusConnection, objectServer);
        });
    getter->getConfiguration(
        std::vector<std::string>{sensorTypes.begin(), sensorTypes.end()},
        retries);
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.FanSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::shared_ptr<TachSensor>>
        tachSensors;
    boost::container::flat_map<std::string, std::unique_ptr<PwmSensor>>
        pwmSensors;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    io.post([&]() {
        createSensors(io, objectServer, tachSensors, pwmSensors, systemBus,
                      nullptr);
    });

    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t& message) {
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
            createSensors(io, objectServer, tachSensors, pwmSensors, systemBus,
                          sensorsChanged, 5);
        });
    };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(*systemBus, sensorTypes, eventHandler);

    // redundancy sensor
    std::function<void(sdbusplus::message_t&)> redundancyHandler =
        [&tachSensors, &systemBus, &objectServer](sdbusplus::message_t&) {
        createRedundancySensor(tachSensors, systemBus, objectServer);
    };
    auto match = std::make_unique<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" +
            redundancyConfiguration + "'",
        std::move(redundancyHandler));
    matches.emplace_back(std::move(match));

    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
