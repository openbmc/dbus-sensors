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

#include "PresenceGpio.hpp"
#include "PwmSensor.hpp"
#include "TachSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <ios>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

// The following two structures need to be consistent
static constexpr std::array<std::string_view, 4> sensorTypes{
    {"AspeedFan", "I2CFan", "NuvotonFan", "HPEFan"}};

enum FanTypes
{
    aspeed = 0,
    i2c,
    nuvoton,
    hpe,
    max,
};

static_assert(std::tuple_size<decltype(sensorTypes)>::value == FanTypes::max,
              "sensorTypes element number is not equal to FanTypes number");

constexpr const char* redundancyConfiguration =
    "xyz.openbmc_project.Configuration.FanRedundancy";
static std::regex inputRegex(R"(fan(\d+)_input)");

// todo: power supply fan redundancy
std::optional<RedundancySensor> systemRedundancy;

static const std::map<std::string, FanTypes> compatibleFanTypes = {
    {"aspeed,ast2400-pwm-tacho", FanTypes::aspeed},
    {"aspeed,ast2500-pwm-tacho", FanTypes::aspeed},
    {"aspeed,ast2600-pwm-tach", FanTypes::aspeed},
    {"nuvoton,npcm750-pwm-fan", FanTypes::nuvoton},
    {"nuvoton,npcm845-pwm-fan", FanTypes::nuvoton},
    {"hpe,gxp-fan-ctrl", FanTypes::hpe}
    // add compatible string here for new fan type
};

FanTypes getFanType(const std::filesystem::path& parentPath)
{
    std::filesystem::path linkPath = parentPath / "of_node";
    if (!std::filesystem::exists(linkPath))
    {
        return FanTypes::i2c;
    }

    std::string canonical = std::filesystem::canonical(linkPath);
    std::string compatiblePath = canonical + "/compatible";
    std::ifstream compatibleStream(compatiblePath);

    if (!compatibleStream)
    {
        lg2::error("Error opening '{PATH}'", "PATH", compatiblePath);
        return FanTypes::i2c;
    }

    std::string compatibleString;
    while (std::getline(compatibleStream, compatibleString))
    {
        compatibleString.pop_back(); // trim EOL before comparisons

        std::map<std::string, FanTypes>::const_iterator compatibleIterator =
            compatibleFanTypes.find(compatibleString);

        if (compatibleIterator != compatibleFanTypes.end())
        {
            return compatibleIterator->second;
        }
    }

    return FanTypes::i2c;
}
void enablePwm(const std::filesystem::path& filePath)
{
    std::fstream enableFile(filePath, std::ios::in | std::ios::out);
    if (!enableFile.good())
    {
        lg2::error("Error read/write '{PATH}'", "PATH", filePath);
        return;
    }

    std::string regulateMode;
    std::getline(enableFile, regulateMode);
    if (regulateMode == "0")
    {
        enableFile << 1;
    }
}
bool findPwmfanPath(unsigned int configPwmfanIndex,
                    std::filesystem::path& pwmPath)
{
    /* Search PWM since pwm-fan had separated
     * PWM from tach directory and 1 channel only*/
    std::vector<std::filesystem::path> pwmfanPaths;
    std::string pwnfanDevName("pwm-fan");

    pwnfanDevName += std::to_string(configPwmfanIndex);

    if (!findFiles(std::filesystem::path("/sys/class/hwmon"), R"(pwm\d+)",
                   pwmfanPaths))
    {
        lg2::error("No PWMs are found!");
        return false;
    }
    for (const auto& path : pwmfanPaths)
    {
        std::error_code ec;
        std::filesystem::path link =
            std::filesystem::read_symlink(path.parent_path() / "device", ec);

        if (ec)
        {
            lg2::error("read_symlink() failed: '{ERROR_MESSAGE}'",
                       "ERROR_MESSAGE", ec.message());
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
bool findPwmPath(const std::filesystem::path& directory, unsigned int pwm,
                 std::filesystem::path& pwmPath)
{
    std::error_code ec;

    /* Assuming PWM file is appeared in the same directory as fanX_input */
    auto path = directory / ("pwm" + std::to_string(pwm + 1));
    bool exists = std::filesystem::exists(path, ec);

    if (ec || !exists)
    {
        /* PWM file not exist or error happened */
        if (ec)
        {
            lg2::error("exists() failed: '{ERROR_MESSAGE}'", "ERROR_MESSAGE",
                       ec.message());
        }
        /* try search form pwm-fanX directory */
        return findPwmfanPath(pwm, pwmPath);
    }

    pwmPath = path;
    return true;
}

// The argument to this function should be the fanN_input file that we want to
// enable. The function will locate the corresponding fanN_enable file if it
// exists. Note that some drivers don't provide this file if the sensors are
// always enabled.
void enableFanInput(const std::filesystem::path& fanInputPath)
{
    std::error_code ec;
    std::string path(fanInputPath.string());
    boost::replace_last(path, "input", "enable");

    bool exists = std::filesystem::exists(path, ec);
    if (ec || !exists)
    {
        return;
    }

    std::fstream enableFile(path, std::ios::out);
    if (!enableFile.good())
    {
        return;
    }
    enableFile << 1;
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
                lg2::error("Error calling entity manager");
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
                            lg2::error("Malformed redundancy record");
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
                        systemRedundancy.emplace(RedundancySensor(
                            std::get<uint64_t>(findCount->second), sensorList,
                            objectServer, path));

                        return;
                    }
                }
            }
        },
        "xyz.openbmc_project.EntityManager", "/xyz/openbmc_project/inventory",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<TachSensor>>&
        tachSensors,
    boost::container::flat_map<std::string, std::unique_ptr<PwmSensor>>&
        pwmSensors,
    boost::container::flat_map<std::string, std::weak_ptr<PresenceGpio>>&
        presenceGpios,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    size_t retries = 0)
{
    auto getter = std::make_shared<
        GetSensorConfiguration>(dbusConnection, [&io, &objectServer,
                                                 &tachSensors, &pwmSensors,
                                                 &presenceGpios,
                                                 &dbusConnection,
                                                 sensorsChanged](
                                                    const ManagedObjectType&
                                                        sensorConfigurations) {
        bool firstScan = sensorsChanged == nullptr;
        std::vector<std::filesystem::path> paths;
        if (!findFiles(std::filesystem::path("/sys/class/hwmon"),
                       R"(fan\d+_input)", paths))
        {
            lg2::error("No fan sensors in system");
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

            std::filesystem::path directory = path.parent_path();
            FanTypes fanType = getFanType(directory);
            std::string cfgIntf = configInterfaceName(sensorTypes[fanType]);

            // convert to 0 based
            size_t index = std::stoul(indexStr) - 1;

            std::string_view baseType;
            const SensorData* sensorData = nullptr;
            const std::string* interfacePath = nullptr;
            const SensorBaseConfiguration* baseConfiguration = nullptr;
            for (const auto& [path, cfgData] : sensorConfigurations)
            {
                // find the base of the configuration to see if indexes
                // match
                auto sensorBaseFind = cfgData.find(cfgIntf);
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
                    lg2::error("'{INTERFACE}' missing index", "INTERFACE",
                               baseConfiguration->first);
                    continue;
                }
                unsigned int configIndex = std::visit(
                    VariantToUnsignedIntVisitor(), findIndex->second);
                if (configIndex != index)
                {
                    continue;
                }
                if (fanType == FanTypes::aspeed ||
                    fanType == FanTypes::nuvoton || fanType == FanTypes::hpe)
                {
                    // there will be only 1 aspeed or nuvoton or hpe sensor
                    // object in sysfs, we found the fan
                    sensorData = &cfgData;
                    break;
                }
                if (fanType == FanTypes::i2c)
                {
                    std::string deviceName =
                        std::filesystem::read_symlink(directory / "device")
                            .filename();

                    size_t bus = 0;
                    size_t addr = 0;
                    if (!getDeviceBusAddr(deviceName, bus, addr))
                    {
                        continue;
                    }

                    auto findBus = baseConfiguration->second.find("Bus");
                    auto findAddress =
                        baseConfiguration->second.find("Address");
                    if (findBus == baseConfiguration->second.end() ||
                        findAddress == baseConfiguration->second.end())
                    {
                        lg2::error("'{INTERFACE}' missing bus or address",
                                   "INTERFACE", baseConfiguration->first);
                        continue;
                    }
                    unsigned int configBus = std::visit(
                        VariantToUnsignedIntVisitor(), findBus->second);
                    unsigned int configAddress = std::visit(
                        VariantToUnsignedIntVisitor(), findAddress->second);

                    if (configBus == bus && configAddress == addr)
                    {
                        sensorData = &cfgData;
                        break;
                    }
                }
            }
            if (sensorData == nullptr)
            {
                lg2::error("failed to find match for '{PATH}'", "PATH",
                           path.string());
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
            auto findSensor = tachSensors.find(sensorName);
            if (!firstScan && findSensor != tachSensors.end())
            {
                bool found = false;
                for (auto it = sensorsChanged->begin();
                     it != sensorsChanged->end(); it++)
                {
                    if (it->ends_with(findSensor->second->name))
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
                lg2::error("error populating thresholds for '{NAME}'", "NAME",
                           sensorName);
            }

            auto presenceConfig =
                sensorData->find(cfgIntf + std::string(".Presence"));

            std::shared_ptr<PresenceGpio> presenceGpio(nullptr);

            // presence sensors are optional
            if (presenceConfig != sensorData->end())
            {
                auto findPolarity = presenceConfig->second.find("Polarity");
                auto findPinName = presenceConfig->second.find("PinName");

                if (findPinName == presenceConfig->second.end() ||
                    findPolarity == presenceConfig->second.end())
                {
                    lg2::error("Malformed Presence Configuration");
                }
                else
                {
                    bool inverted =
                        std::get<std::string>(findPolarity->second) == "Low";
                    const auto* pinName =
                        std::get_if<std::string>(&findPinName->second);

                    if (pinName != nullptr)
                    {
                        auto findPresenceGpio = presenceGpios.find(*pinName);
                        if (findPresenceGpio != presenceGpios.end())
                        {
                            auto p = findPresenceGpio->second.lock();
                            if (p)
                            {
                                presenceGpio = p;
                            }
                        }
                        if (!presenceGpio)
                        {
                            auto findMonitorType =
                                presenceConfig->second.find("MonitorType");
                            bool polling = false;
                            if (findMonitorType != presenceConfig->second.end())
                            {
                                auto mType = std::get<std::string>(
                                    findMonitorType->second);
                                if (mType == "Polling")
                                {
                                    polling = true;
                                }
                                else if (mType != "Event")
                                {
                                    lg2::error(
                                        "Unsupported GPIO MonitorType of '{TYPE}' for '{NAME}', "
                                        "supported types: Polling, Event default",
                                        "TYPE", mType, "NAME", sensorName);
                                }
                            }
                            try
                            {
                                if (polling)
                                {
                                    presenceGpio =
                                        std::make_shared<PollingPresenceGpio>(
                                            "Fan", sensorName, *pinName,
                                            inverted, io);
                                }
                                else
                                {
                                    presenceGpio =
                                        std::make_shared<EventPresenceGpio>(
                                            "Fan", sensorName, *pinName,
                                            inverted, io);
                                }
                                presenceGpios[*pinName] = presenceGpio;
                            }
                            catch (const std::system_error& e)
                            {
                                lg2::error(
                                    "Failed to create GPIO monitor object for "
                                    "'{PIN_NAME}' / '{SENSOR_NAME}': '{ERROR}'",
                                    "PIN_NAME", *pinName, "SENSOR_NAME",
                                    sensorName, "ERROR", e);
                            }
                        }
                    }
                    else
                    {
                        lg2::error(
                            "Malformed Presence pinName for sensor '{NAME}'",
                            "NAME", sensorName);
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
                sensorData->find(cfgIntf + std::string(".Connector"));

            std::optional<std::string> led;
            std::string pwmName;
            std::filesystem::path pwmPath;

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
                        lg2::error(
                            "Connector for '{NAME}' no pwm channel found!",
                            "NAME", sensorName);
                        continue;
                    }

                    std::filesystem::path pwmEnableFile =
                        "pwm" + std::to_string(pwm + 1) + "_enable";
                    std::filesystem::path enablePath =
                        pwmPath.parent_path() / pwmEnableFile;
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
                    lg2::error("Connector for '{NAME}' missing pwm!", "NAME",
                               sensorName);
                }

                auto findLED = connector->second.find("LED");
                if (findLED != connector->second.end())
                {
                    const auto* ledName =
                        std::get_if<std::string>(&(findLED->second));
                    if (ledName == nullptr)
                    {
                        lg2::error("Wrong format for LED of '{NAME}'", "NAME",
                                   sensorName);
                    }
                    else
                    {
                        led = *ledName;
                    }
                }
            }

            findLimits(limits, baseConfiguration);

            enableFanInput(path);

            auto& tachSensor = tachSensors[sensorName];
            tachSensor = nullptr;
            tachSensor = std::make_shared<TachSensor>(
                path.string(), baseType, objectServer, dbusConnection,
                presenceGpio, redundancy, io, sensorName,
                std::move(sensorThresholds), *interfacePath, limits, powerState,
                led);
            tachSensor->setupRead();

            if (!pwmPath.empty() && std::filesystem::exists(pwmPath) &&
                (pwmSensors.count(pwmPath) == 0U))
            {
                pwmSensors[pwmPath] = std::make_unique<PwmSensor>(
                    pwmName, pwmPath, dbusConnection, objectServer,
                    *interfacePath, "Fan", isValueMutable);
            }
        }

        createRedundancySensor(tachSensors, dbusConnection, objectServer);
    });
    getter->getConfiguration(sensorTypes, retries);
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);

    objectServer.add_manager("/xyz/openbmc_project/sensors");
    objectServer.add_manager("/xyz/openbmc_project/control");
    objectServer.add_manager("/xyz/openbmc_project/inventory");
    systemBus->request_name("xyz.openbmc_project.FanSensor");
    boost::container::flat_map<std::string, std::shared_ptr<TachSensor>>
        tachSensors;
    boost::container::flat_map<std::string, std::unique_ptr<PwmSensor>>
        pwmSensors;
    boost::container::flat_map<std::string, std::weak_ptr<PresenceGpio>>
        presenceGpios;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, tachSensors, pwmSensors, presenceGpios,
                      systemBus, nullptr);
    });

    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t& message) {
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
                createSensors(io, objectServer, tachSensors, pwmSensors,
                              presenceGpios, systemBus, sensorsChanged, 5);
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
