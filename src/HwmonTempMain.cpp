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

#include <HwmonTempSensor.hpp>
#include <Utils.hpp>
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
#include <regex>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

static constexpr bool debug = false;
static constexpr float pollRateDefault = 0.5;

static constexpr double maxReadingPressure = 120000; // Pascals
static constexpr double minReadingPressure = 30000;  // Pascals

static constexpr double maxReadingTemperature = 127;  // DegreesC
static constexpr double minReadingTemperature = -128; // DegreesC

namespace fs = std::filesystem;
static constexpr std::array<const char*, 18> sensorTypes = {
    "xyz.openbmc_project.Configuration.EMC1412",
    "xyz.openbmc_project.Configuration.EMC1413",
    "xyz.openbmc_project.Configuration.EMC1414",
    "xyz.openbmc_project.Configuration.MAX31725",
    "xyz.openbmc_project.Configuration.MAX31730",
    "xyz.openbmc_project.Configuration.MAX6581",
    "xyz.openbmc_project.Configuration.MAX6654",
    "xyz.openbmc_project.Configuration.SBTSI",
    "xyz.openbmc_project.Configuration.LM95234",
    "xyz.openbmc_project.Configuration.TMP112",
    "xyz.openbmc_project.Configuration.TMP175",
    "xyz.openbmc_project.Configuration.TMP421",
    "xyz.openbmc_project.Configuration.TMP441",
    "xyz.openbmc_project.Configuration.LM75A",
    "xyz.openbmc_project.Configuration.TMP75",
    "xyz.openbmc_project.Configuration.W83773G",
    "xyz.openbmc_project.Configuration.DPS310",
    "xyz.openbmc_project.Configuration.SI7020"};

#define MAX_LINE_LEN 80

static double readRawSensorCompValue(const char* path, const char* typeString)
{
    size_t index, index2;
    double readRawSensorCompResult;
    FILE* readCompStream;
    const char rawString[] = "raw";
    char sensorCompFilename[MAX_LINE_LEN];
    char sensorCompValueRead[MAX_LINE_LEN];

    if (nullptr == path)
    {
        return SNAN;
    }

    if (nullptr == typeString)
    {
        return SNAN;
    }

    if (strlen(rawString) + 1 > strlen(path))
    {
        return SNAN;
    }

    for (index = 0; index < MAX_LINE_LEN; index++)
    {
        sensorCompFilename[index] = path[index];
        if ('\0' == path[index])
        {
            break;
        }
    }

    if (MAX_LINE_LEN - 1 < index + strlen(typeString))
    {
        return SNAN;
    }

    for (index2 = 0; index2 < strlen(typeString); index2++)
    {
        sensorCompFilename[index - strlen(rawString)] = typeString[index2];
        index++;
        sensorCompFilename[index - strlen(rawString)] = '\0';
    }

    if (0 != access(sensorCompFilename, R_OK))
    {
        return SNAN;
    }

    readCompStream = fopen(sensorCompFilename, "r");

    if (nullptr == readCompStream)
    {
        return SNAN;
    }

    if (sensorCompValueRead !=
        fgets(sensorCompValueRead, MAX_LINE_LEN - 1, readCompStream))
    {
        fclose(readCompStream);
        return SNAN;
    }

    fclose(readCompStream);

    readRawSensorCompResult = strtod(sensorCompValueRead, nullptr);

    return readRawSensorCompResult;
}

static double readRawSensorOffsetValue(const char* path)
{
    const char offsetString[] = "offset";

    if (nullptr == path)
    {
        return SNAN;
    }

    return readRawSensorCompValue(path, offsetString);
}

static double readRawSensorScaleValue(const char* path)
{
    const char scaleString[] = "scale";

    if (nullptr == path)
    {
        return SNAN;
    }

    return readRawSensorCompValue(path, scaleString);
}

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<HwmonTempSensor>>&
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

            std::vector<fs::path> paths;
            fs::path root("/sys/bus/iio/devices");
            findFiles(root, R"(in_temp\d*_(input|raw))", paths);
            findFiles(root, R"(in_pressure\d*_(input|raw))", paths);
            findFiles(fs::path("/sys/class/hwmon"), R"(temp\d+_input)", paths);

            if (paths.empty())
            {
                return;
            }

            // iterate through all found temp and pressure sensors,
            // and try to match them with configuration
            for (auto& path : paths)
            {
                std::smatch match;
                const std::string& pathStr = path.string();
                auto directory = path.parent_path();
                fs::path device;

                std::string deviceName;
                if (pathStr.starts_with("/sys/bus/iio/devices"))
                {
                    device = fs::canonical(directory);
                    deviceName = device.parent_path().stem();
                }
                else
                {
                    device = directory / "device";
                    deviceName = fs::canonical(device).stem();
                }
                auto findHyphen = deviceName.find('-');
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
                    addr = std::stoi(addrStr, nullptr, 16);
                }
                catch (std::invalid_argument&)
                {
                    continue;
                }
                const SensorData* sensorData = nullptr;
                const std::string* interfacePath = nullptr;
                const char* sensorType = nullptr;
                const SensorBaseConfiguration* baseConfiguration = nullptr;
                const SensorBaseConfigMap* baseConfigMap = nullptr;
                std::string sensorTypeName = "temperature";

                // Temperatures are read in milli degrees Celsius, we need
                // degrees Celsius. Pressures are read in kilopascal, we need
                // Pascals.  On D-Bus for Open BMC we use the International
                // System of Units without prefixes. Links to the kernel
                // documentation:
                // https://www.kernel.org/doc/Documentation/hwmon/sysfs-interface
                // https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-bus-iio
                // For IIO RAW sensors we get a raw_value, an offset, and scale
                // to compute the value = (raw_value + offset) * scale

                double minReading = minReadingTemperature;
                double maxReading = maxReadingTemperature;
                double offsetValue = 0.0;
                double scaleValue = 1.0;
                std::string units = "DegreesC";

                // with a _raw IIO device we need to get the
                // offsetValue and scaleValue from the driver
                if (pathStr.ends_with("_raw"))
                {
                    double tmpOffsetValue =
                        readRawSensorOffsetValue(pathStr.c_str());
                    double tmpScaleValue =
                        readRawSensorScaleValue(pathStr.c_str());
                    if (SNAN != tmpOffsetValue)
                    {
                        offsetValue = tmpOffsetValue;
                    }
                    if (SNAN != tmpScaleValue)
                    {
                        scaleValue = tmpScaleValue;
                    }
                }

                if (path.string().find("in_pressure_") != std::string::npos)
                {
                    minReading = minReadingPressure;
                    maxReading = maxReadingPressure;
                    // Pressures are read in kilopascal, we need Pascals.
                    scaleValue *= 1000.0;
                    sensorTypeName = "pressure";
                    units = "Pascals";
                }
                else
                {
                    // Temperatures are read in milli degrees Celsius,
                    // we need degrees Celsius.
                    scaleValue *= 0.001;
                }

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
                    baseConfigMap = &baseConfiguration->second;
                    auto configurationBus = baseConfigMap->find("Bus");
                    auto configurationAddress = baseConfigMap->find("Address");

                    if (configurationBus == baseConfigMap->end() ||
                        configurationAddress == baseConfigMap->end())
                    {
                        std::cerr << "error finding bus or address in "
                                     "configuration\n";
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

                // Temperature has "Name", pressure has "Name1"
                auto findSensorName = baseConfigMap->find("Name");
                if (sensorTypeName == "pressure")
                {
                    findSensorName = baseConfigMap->find("Name1");
                }

                if (findSensorName == baseConfigMap->end())
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
                    auto it = sensorsChanged->begin();
                    while (it != sensorsChanged->end())
                    {
                        if (boost::ends_with(*it, findSensor->second->name))
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
                int index = 1;

                if (!parseThresholdsFromConfig(*sensorData, sensorThresholds,
                                               nullptr, &index))
                {
                    std::cerr << "error populating thresholds for "
                              << sensorName << " index 1\n";
                }

                auto findPollRate = baseConfiguration->second.find("PollRate");
                float pollRate = pollRateDefault;
                if (findPollRate != baseConfiguration->second.end())
                {
                    pollRate = std::visit(VariantToFloatVisitor(),
                                          findPollRate->second);
                    if (pollRate <= 0.0f)
                    {
                        pollRate = pollRateDefault; // polling time too short
                    }
                }

                auto findPowerOn = baseConfiguration->second.find("PowerState");
                PowerState readState = PowerState::always;
                if (findPowerOn != baseConfiguration->second.end())
                {
                    std::string powerState = std::visit(
                        VariantToStringVisitor(), findPowerOn->second);
                    setReadState(powerState, readState);
                }

                auto permitSet = getPermitSet(*baseConfigMap);
                auto& sensor = sensors[sensorName];
                sensor = nullptr;
                if (pathStr.starts_with("/sys/bus/iio/devices"))
                {
                    sensor = std::make_shared<HwmonTempSensor>(
                        pathStr, sensorType, objectServer, dbusConnection, io,
                        sensorName, std::move(sensorThresholds), offsetValue,
                        scaleValue, minReading, maxReading, units, pollRate,
                        *interfacePath, readState, sensorTypeName);
                    sensor->setupRead();
                }
                else
                {
                    auto hwmonFile = getFullHwmonFilePath(directory.string(),
                                                          "temp1", permitSet);
                    if (hwmonFile)
                    {
                        sensor = std::make_shared<HwmonTempSensor>(
                            *hwmonFile, sensorType, objectServer,
                            dbusConnection, io, sensorName,
                            std::move(sensorThresholds), offsetValue,
                            scaleValue, minReading, maxReading, units, pollRate,
                            *interfacePath, readState, sensorTypeName);
                        sensor->setupRead();
                    }
                    // Looking for keys like "Name1" for temp2_input,
                    // "Name2" for temp3_input, etc.
                    int i = 0;
                    while (true)
                    {
                        ++i;
                        auto findKey =
                            baseConfigMap->find("Name" + std::to_string(i));
                        if (findKey == baseConfigMap->end())
                        {
                            break;
                        }
                        std::string sensorName =
                            std::get<std::string>(findKey->second);
                        hwmonFile = getFullHwmonFilePath(
                            directory.string(), "temp" + std::to_string(i + 1),
                            permitSet);
                        if (hwmonFile)
                        {
                            // To look up thresholds for these additional
                            // sensors, match on the Index property in the
                            // threshold data where the index comes from the
                            // sysfs file we're on, i.e. index = 2 for
                            // temp2_input.
                            int index = i + 1;
                            std::vector<thresholds::Threshold> thresholds;

                            if (!parseThresholdsFromConfig(
                                    *sensorData, thresholds, nullptr, &index))
                            {
                                std::cerr << "error populating thresholds for "
                                          << sensorName << " index " << index
                                          << "\n";
                            }

                            auto& sensor = sensors[sensorName];
                            sensor = nullptr;
                            sensor = std::make_shared<HwmonTempSensor>(
                                *hwmonFile, sensorType, objectServer,
                                dbusConnection, io, sensorName,
                                std::move(thresholds), offsetValue, scaleValue,
                                minReading, maxReading, units, pollRate,
                                *interfacePath, readState, sensorTypeName);
                            sensor->setupRead();
                        }
                    }
                }
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
    boost::container::flat_map<std::string, std::shared_ptr<HwmonTempSensor>>
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
                if (ec)
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

    setupManufacturingModeMatch(*systemBus);
    io.run();
}
