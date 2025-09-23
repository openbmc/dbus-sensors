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

#include "DeviceMgmt.hpp"
#include "HwmonTempSensor.hpp"
#include "Reactor.hpp"
#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <phosphor-logging/lg2.hpp>
#include <phosphor-logging/lg2/flags.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

static constexpr float pollRateDefault = 0.5;

static constexpr double maxValuePressure = 120000;      // Pascals
static constexpr double minValuePressure = 30000;       // Pascals

static constexpr double maxValueRelativeHumidity = 100; // PercentRH
static constexpr double minValueRelativeHumidity = 0;   // PercentRH

static constexpr double maxValueTemperature = 127;      // DegreesC
static constexpr double minValueTemperature = -128;     // DegreesC

static const I2CDeviceTypeMap sensorTypes{
    {"ADM1021", I2CDeviceType{"adm1021", true}},
    {"DPS310", I2CDeviceType{"dps310", false}},
    {"EMC1403", I2CDeviceType{"emc1403", true}},
    {"EMC1412", I2CDeviceType{"emc1412", true}},
    {"EMC1413", I2CDeviceType{"emc1413", true}},
    {"EMC1414", I2CDeviceType{"emc1414", true}},
    {"HDC1080", I2CDeviceType{"hdc1080", false}},
    {"JC42", I2CDeviceType{"jc42", true}},
    {"LM75A", I2CDeviceType{"lm75a", true}},
    {"LM95234", I2CDeviceType{"lm95234", true}},
    {"MAX31725", I2CDeviceType{"max31725", true}},
    {"MAX31730", I2CDeviceType{"max31730", true}},
    {"MAX6581", I2CDeviceType{"max6581", true}},
    {"MAX6654", I2CDeviceType{"max6654", true}},
    {"MAX6639", I2CDeviceType{"max6639", true}},
    {"MCP9600", I2CDeviceType{"mcp9600", false}},
    {"NCT6779", I2CDeviceType{"nct6779", true}},
    {"NCT7802", I2CDeviceType{"nct7802", true}},
    {"PT5161L", I2CDeviceType{"pt5161l", true}},
    {"SBTSI", I2CDeviceType{"sbtsi", true}},
    {"SI7020", I2CDeviceType{"si7020", false}},
    {"TMP100", I2CDeviceType{"tmp100", true}},
    {"TMP1075", I2CDeviceType{"tmp1075", true}},
    {"TMP112", I2CDeviceType{"tmp112", true}},
    {"TMP175", I2CDeviceType{"tmp175", true}},
    {"TMP411", I2CDeviceType{"tmp411", true}},
    {"TMP421", I2CDeviceType{"tmp421", true}},
    {"TMP432", I2CDeviceType{"tmp432", true}},
    {"TMP441", I2CDeviceType{"tmp441", true}},
    {"TMP451", I2CDeviceType{"tmp451", true}},
    {"TMP461", I2CDeviceType{"tmp461", true}},
    {"TMP464", I2CDeviceType{"tmp464", true}},
    {"TMP468", I2CDeviceType{"tmp468", true}},
    {"TMP75", I2CDeviceType{"tmp75", true}},
    {"W83773G", I2CDeviceType{"w83773g", true}},
};

static struct SensorParams getSensorParameters(
    const std::filesystem::path& path)
{
    // offset is to default to 0 and scale to 1, see lore
    // https://lore.kernel.org/linux-iio/5c79425f-6e88-36b6-cdfe-4080738d039f@metafoo.de/
    struct SensorParams tmpSensorParameters = {
        .minValue = minValueTemperature,
        .maxValue = maxValueTemperature,
        .offsetValue = 0.0,
        .scaleValue = 1.0,
        .units = sensor_paths::unitDegreesC,
        .typeName = "temperature"};

    // For IIO RAW sensors we get a raw_value, an offset, and scale
    // to compute the value = (raw_value + offset) * scale
    // with a _raw IIO device we need to get the
    // offsetValue and scaleValue from the driver
    // these are used to compute the reading in
    // units that have yet to be scaled for D-Bus.
    const std::string pathStr = path.string();
    if (pathStr.ends_with("_raw"))
    {
        std::string pathOffsetStr =
            pathStr.substr(0, pathStr.size() - 4) + "_offset";
        std::optional<double> tmpOffsetValue = readFile(pathOffsetStr, 1.0);
        // In case there is nothing to read skip this device
        // This is not an error condition see lore
        // https://lore.kernel.org/linux-iio/5c79425f-6e88-36b6-cdfe-4080738d039f@metafoo.de/
        if (tmpOffsetValue)
        {
            tmpSensorParameters.offsetValue = *tmpOffsetValue;
        }

        std::string pathScaleStr =
            pathStr.substr(0, pathStr.size() - 4) + "_scale";
        std::optional<double> tmpScaleValue = readFile(pathScaleStr, 1.0);
        // In case there is nothing to read skip this device
        // This is not an error condition see lore
        // https://lore.kernel.org/linux-iio/5c79425f-6e88-36b6-cdfe-4080738d039f@metafoo.de/
        if (tmpScaleValue)
        {
            tmpSensorParameters.scaleValue = *tmpScaleValue;
        }
    }

    // Temperatures are read in milli degrees Celsius, we need
    // degrees Celsius. Pressures are read in kilopascal, we need
    // Pascals.  On D-Bus for Open BMC we use the International
    // System of Units without prefixes. Links to the kernel
    // documentation:
    // https://www.kernel.org/doc/Documentation/hwmon/sysfs-interface
    // https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-bus-iio
    if (path.filename() == "in_pressure_input" ||
        path.filename() == "in_pressure_raw")
    {
        tmpSensorParameters.minValue = minValuePressure;
        tmpSensorParameters.maxValue = maxValuePressure;
        // Pressures are read in kilopascal, we need Pascals.
        tmpSensorParameters.scaleValue *= 1000.0;
        tmpSensorParameters.typeName = "pressure";
        tmpSensorParameters.units = sensor_paths::unitPascals;
    }
    else if (path.filename() == "in_humidityrelative_input" ||
             path.filename() == "in_humidityrelative_raw")
    {
        tmpSensorParameters.minValue = minValueRelativeHumidity;
        tmpSensorParameters.maxValue = maxValueRelativeHumidity;
        // Relative Humidity are read in milli-percent, we need percent.
        tmpSensorParameters.scaleValue *= 0.001;
        tmpSensorParameters.typeName = "humidity";
        tmpSensorParameters.units = sensor_paths::unitPercentRH;
    }
    else
    {
        // Temperatures are read in milli degrees Celsius,
        // we need degrees Celsius.
        tmpSensorParameters.scaleValue *= 0.001;
    }

    return tmpSensorParameters;
}

struct SensorConfigKey
{
    uint64_t bus;
    uint64_t addr;
    bool operator<(const SensorConfigKey& other) const
    {
        if (bus != other.bus)
        {
            return bus < other.bus;
        }
        return addr < other.addr;
    }
};

struct SensorConfig
{
    std::string sensorPath;
    SensorData sensorData;
    std::string interface;
    SensorBaseConfigMap config;
    std::vector<std::string> name;
};

using SensorConfigMap =
    boost::container::flat_map<SensorConfigKey, SensorConfig>;

static SensorConfigMap buildSensorConfigMap(
    const ManagedObjectType& sensorConfigs)
{
    SensorConfigMap configMap;
    for (const auto& [path, cfgData] : sensorConfigs)
    {
        for (const auto& [intf, cfg] : cfgData)
        {
            auto busCfg = cfg.find("Bus");
            auto addrCfg = cfg.find("Address");
            if ((busCfg == cfg.end()) || (addrCfg == cfg.end()))
            {
                continue;
            }

            if ((std::get_if<uint64_t>(&busCfg->second) == nullptr) ||
                (std::get_if<uint64_t>(&addrCfg->second) == nullptr))
            {
                lg2::error("'{PATH}' Bus or Address invalid", "PATH", path.str);
                continue;
            }

            std::vector<std::string> hwmonNames;
            auto nameCfg = cfg.find("Name");
            if (nameCfg != cfg.end())
            {
                hwmonNames.push_back(std::get<std::string>(nameCfg->second));
                size_t i = 1;
                while (true)
                {
                    auto sensorNameCfg = cfg.find("Name" + std::to_string(i));
                    if (sensorNameCfg == cfg.end())
                    {
                        break;
                    }
                    hwmonNames.push_back(
                        std::get<std::string>(sensorNameCfg->second));
                    i++;
                }
            }

            SensorConfigKey key = {std::get<uint64_t>(busCfg->second),
                                   std::get<uint64_t>(addrCfg->second)};
            SensorConfig val = {path.str, cfgData, intf, cfg, hwmonNames};

            auto [it, inserted] = configMap.emplace(key, std::move(val));
            if (!inserted)
            {
                lg2::error(
                    "'{PATH}': ignoring duplicate entry for '{BUS}', '{ADDR}'",
                    "PATH", path.str, "BUS", key.bus, "ADDR", lg2::hex,
                    key.addr);
            }
        }
    }
    return configMap;
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<HwmonTempSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    bool activateOnly)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&io, &objectServer, &sensors, &dbusConnection, sensorsChanged,
         activateOnly](const ManagedObjectType& sensorConfigurations) {
            bool firstScan = sensorsChanged == nullptr;

            SensorConfigMap configMap =
                buildSensorConfigMap(sensorConfigurations);

            auto devices =
                instantiateDevices(sensorConfigurations, sensors, sensorTypes);

            // IIO _raw devices look like this on sysfs:
            //     /sys/bus/iio/devices/iio:device0/in_temp_raw
            //     /sys/bus/iio/devices/iio:device0/in_temp_offset
            //     /sys/bus/iio/devices/iio:device0/in_temp_scale
            //
            // Other IIO devices look like this on sysfs:
            //     /sys/bus/iio/devices/iio:device1/in_temp_input
            //     /sys/bus/iio/devices/iio:device1/in_pressure_input
            std::vector<std::filesystem::path> paths;
            std::filesystem::path root("/sys/bus/iio/devices");
            findFiles(root, R"(in_temp\d*_(input|raw))", paths);
            findFiles(root, R"(in_pressure\d*_(input|raw))", paths);
            findFiles(root, R"(in_humidityrelative\d*_(input|raw))", paths);
            findFiles(std::filesystem::path("/sys/class/hwmon"),
                      R"(temp\d+_input)", paths);

            // iterate through all found temp and pressure sensors,
            // and try to match them with configuration
            for (auto& path : paths)
            {
                std::smatch match;
                const std::string pathStr = path.string();
                auto directory = path.parent_path();
                std::filesystem::path device;

                std::string deviceName;
                std::error_code ec;
                if (pathStr.starts_with("/sys/bus/iio/devices"))
                {
                    device = std::filesystem::canonical(directory, ec);
                    if (ec)
                    {
                        lg2::error("Fail to find device in '{PATH}'", "PATH",
                                   pathStr);
                        continue;
                    }
                    deviceName = device.parent_path().stem();
                }
                else
                {
                    device =
                        std::filesystem::canonical(directory / "device", ec);
                    if (ec)
                    {
                        lg2::error("Fail to find device in '{PATH}'", "PATH",
                                   pathStr);
                        continue;
                    }
                    deviceName = device.stem();
                }

                uint64_t bus = 0;
                uint64_t addr = 0;
                if (!getDeviceBusAddr(deviceName, bus, addr))
                {
                    continue;
                }

                auto thisSensorParameters = getSensorParameters(path);
                auto findSensorCfg = configMap.find({bus, addr});
                if (findSensorCfg == configMap.end())
                {
                    continue;
                }

                const std::string& interfacePath =
                    findSensorCfg->second.sensorPath;
                auto findI2CDev = devices.find(interfacePath);

                std::shared_ptr<I2CDevice> i2cDev;
                if (findI2CDev != devices.end())
                {
                    // If we're only looking to activate newly-instantiated i2c
                    // devices and this sensor's underlying device was already
                    // there before this call, there's nothing more to do here.
                    if (activateOnly && !findI2CDev->second.second)
                    {
                        continue;
                    }
                    i2cDev = findI2CDev->second.first;
                }

                const SensorData& sensorData = findSensorCfg->second.sensorData;
                std::string sensorType = findSensorCfg->second.interface;
                auto pos = sensorType.find_last_of('.');
                if (pos != std::string::npos)
                {
                    sensorType = sensorType.substr(pos + 1);
                }
                const SensorBaseConfigMap& baseConfigMap =
                    findSensorCfg->second.config;
                std::vector<std::string>& hwmonName =
                    findSensorCfg->second.name;

                // Temperature has "Name", pressure has "Name1"
                auto findSensorName = baseConfigMap.find("Name");
                int index = 1;
                if (thisSensorParameters.typeName == "pressure" ||
                    thisSensorParameters.typeName == "humidity")
                {
                    findSensorName = baseConfigMap.find("Name1");
                    index = 2;
                }

                if (findSensorName == baseConfigMap.end())
                {
                    lg2::error(
                        "could not determine configuration name for '{NAME}'",
                        "NAME", deviceName);
                    continue;
                }
                std::string sensorName =
                    std::get<std::string>(findSensorName->second);
                // on rescans, only update sensors we were signaled by
                auto findSensor = sensors.find(sensorName);
                if (!firstScan && findSensor != sensors.end())
                {
                    bool found = false;
                    auto it = sensorsChanged->begin();
                    while (it != sensorsChanged->end())
                    {
                        if (it->ends_with(findSensor->second->name))
                        {
                            it = sensorsChanged->erase(it);
                            findSensor->second = nullptr;
                            found = true;
                            break;
                        }
                        ++it;
                    }
                    if (!found)
                    {
                        continue;
                    }
                }

                std::vector<thresholds::Threshold> sensorThresholds;

                if (!parseThresholdsFromConfig(sensorData, sensorThresholds,
                                               nullptr, &index))
                {
                    lg2::error("error populating thresholds for "
                               "'{NAME}', index: '{INDEX}'",
                               "NAME", sensorName, "INDEX", index);
                }

                float pollRate = getPollRate(baseConfigMap, pollRateDefault);
                PowerState readState = getPowerState(baseConfigMap);

                auto permitSet = getPermitSet(baseConfigMap);
                auto& sensor = sensors[sensorName];
                if (!activateOnly)
                {
                    sensor = nullptr;
                }
                auto hwmonFile = getFullHwmonFilePath(directory.string(),
                                                      "temp1", permitSet);
                if (pathStr.starts_with("/sys/bus/iio/devices"))
                {
                    hwmonFile = pathStr;
                }
                if (hwmonFile)
                {
                    if (sensor != nullptr)
                    {
                        sensor->activate(*hwmonFile, i2cDev);
                    }
                    else
                    {
                        sensor = std::make_shared<HwmonTempSensor>(
                            *hwmonFile, sensorType, objectServer,
                            dbusConnection, io, sensorName,
                            std::move(sensorThresholds), thisSensorParameters,
                            pollRate, interfacePath, readState, i2cDev);
                        sensor->setupRead();
                    }
                }
                hwmonName.erase(
                    remove(hwmonName.begin(), hwmonName.end(), sensorName),
                    hwmonName.end());

                // Looking for keys like "Name1" for temp2_input,
                // "Name2" for temp3_input, etc.
                int i = 0;
                while (true)
                {
                    ++i;
                    auto findKey =
                        baseConfigMap.find("Name" + std::to_string(i));
                    if (findKey == baseConfigMap.end())
                    {
                        break;
                    }
                    std::string sensorName =
                        std::get<std::string>(findKey->second);
                    hwmonFile = getFullHwmonFilePath(
                        directory.string(), "temp" + std::to_string(i + 1),
                        permitSet);
                    if (pathStr.starts_with("/sys/bus/iio/devices"))
                    {
                        continue;
                    }
                    if (hwmonFile)
                    {
                        // To look up thresholds for these additional sensors,
                        // match on the Index property in the threshold data
                        // where the index comes from the sysfs file we're on,
                        // i.e. index = 2 for temp2_input.
                        int index = i + 1;
                        std::vector<thresholds::Threshold> thresholds;

                        if (!parseThresholdsFromConfig(sensorData, thresholds,
                                                       nullptr, &index))
                        {
                            lg2::error("error populating thresholds for "
                                       "'{NAME}', index: '{INDEX}'",
                                       "NAME", sensorName, "INDEX", index);
                        }

                        auto& sensor = sensors[sensorName];
                        if (!activateOnly)
                        {
                            sensor = nullptr;
                        }

                        if (sensor != nullptr)
                        {
                            sensor->activate(*hwmonFile, i2cDev);
                        }
                        else
                        {
                            sensor = std::make_shared<HwmonTempSensor>(
                                *hwmonFile, sensorType, objectServer,
                                dbusConnection, io, sensorName,
                                std::move(thresholds), thisSensorParameters,
                                pollRate, interfacePath, readState, i2cDev);
                            sensor->setupRead();
                        }
                    }

                    hwmonName.erase(
                        remove(hwmonName.begin(), hwmonName.end(), sensorName),
                        hwmonName.end());
                }
                if (hwmonName.empty())
                {
                    configMap.erase(findSensorCfg);
                }
            }
        });
    std::vector<std::string> types;
    types.reserve(sensorTypes.size());
    for (const auto& [type, dt] : sensorTypes)
    {
        types.push_back(type);
    }
    getter->getConfiguration(types);
}

void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<HwmonTempSensor>>&
        sensors)
{
    if (message.is_method_error())
    {
        lg2::error("interfacesRemoved callback method error");
        return;
    }

    sdbusplus::message::object_path path;
    std::vector<std::string> interfaces;

    message.read(path, interfaces);

    // If the xyz.openbmc_project.Confguration.X interface was removed
    // for one or more sensors, delete those sensor objects.
    auto sensorIt = sensors.begin();
    while (sensorIt != sensors.end())
    {
        if (sensorIt->second && (sensorIt->second->configurationPath == path) &&
            (std::find(interfaces.begin(), interfaces.end(),
                       sensorIt->second->configInterface) != interfaces.end()))
        {
            sensorIt = sensors.erase(sensorIt);
        }
        else
        {
            sensorIt++;
        }
    }
}

static void powerStateChanged(
    PowerState type, bool newState,
    boost::container::flat_map<std::string, std::shared_ptr<HwmonTempSensor>>&
        sensors,
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (newState)
    {
        createSensors(io, objectServer, sensors, dbusConnection, nullptr, true);
    }
    else
    {
        for (auto& [path, sensor] : sensors)
        {
            if (sensor != nullptr && sensor->readState == type)
            {
                sensor->deactivate();
            }
        }
    }
}

int main()
{
    boost::asio::io_context io;
    Reactor reactor("xyz.openbmc_project.HwmonTempSensor", true, io);

    reactor.objectServer.add_manager("/xyz/openbmc_project/sensors");

    reactor.requestName();

    boost::container::flat_map<std::string, std::shared_ptr<HwmonTempSensor>>
        sensors;

    auto powerCallBack = [&sensors, &reactor](PowerState type, bool state) {
        powerStateChanged(type, state, sensors, reactor.io,
                          reactor.objectServer, reactor.systemBus);
    };
    setupPowerMatchCallback(reactor.systemBus, powerCallBack);

    reactor.post([&]() {
        createSensors(reactor.io, reactor.objectServer, sensors,
                      reactor.systemBus, nullptr, false);
    });

    reactor.eventHandler = [&](sdbusplus::message_t& message) {
        if (message.is_method_error())
        {
            lg2::error("callback method error");
            return;
        }
        reactor.sensorsChanged->insert(message.get_path());
        // this implicitly cancels the timer
        reactor.filterTimer.expires_after(std::chrono::seconds(1));

        reactor.filterTimer.async_wait(
            [&](const boost::system::error_code& ec) {
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
                createSensors(reactor.io, reactor.objectServer, sensors,
                              reactor.systemBus, reactor.sensorsChanged, false);
            });
    };

    reactor.matches = setupPropertiesChangedMatches(
        *reactor.systemBus, sensorTypes, reactor.eventHandler);

    // Watch for entity-manager to remove configuration interfaces
    // so the corresponding sensors can be removed.
    auto ifaceRemovedMatch = std::make_unique<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*reactor.systemBus),
        "type='signal',member='InterfacesRemoved',arg0path='" +
            std::string(inventoryPath) + "/'",
        [&sensors](sdbusplus::message_t& msg) {
            interfaceRemoved(msg, sensors);
        });

    reactor.matches.emplace_back(std::move(ifaceRemovedMatch));

    return reactor.run();
}
