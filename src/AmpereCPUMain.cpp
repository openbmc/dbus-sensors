/*
// Copyright 2022 Ampere Computing LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <AmpereCPU.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <regex>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace fs = std::filesystem;

const constexpr char* ampereDbusName = "xyz.openbmc_project.AmpereCPUSensor";
static const char* smproDevType =
    "xyz.openbmc_project.Configuration.smpro_hwmon";
static const std::string smproDrvName = "smpro_hwmon";
static boost::container::flat_map<std::string, std::shared_ptr<AmpereCPUSensor>>
    sensors;
static boost::container::flat_map<std::string, std::string> sensorTable;
static boost::container::flat_map<std::string, AmpereCPUProperty> propMatch;
static std::vector<AmpereCPUProperty> socProperties;
static std::regex i2cDevRegex(R"((\d+)-([a-fA-F0-9]+))");

static bool getDeviceInfo(const std::string& devPath, size_t* bus, size_t* addr)
{
    std::smatch match;
    std::regex_search(devPath, match, i2cDevRegex);

    if (match.empty() || (match.size() != 3))
    {
        std::cerr << "Found bad device path " << devPath << "\n";
        return false;
    }

    std::string busStr = match[1];
    std::string addrStr = match[2];

    try
    {
        *bus = (size_t)std::stoi(busStr);
        *addr = (size_t)std::stoi(addrStr, nullptr, 16);
    }
    catch (std::invalid_argument&)
    {
        std::cerr << "Error parsing bus " << busStr << " addr " << addrStr
                  << "\n";
        return false;
    }

    return true;
}

static bool findMatchConfig(
    const ManagedObjectType& sensorConfigs,
    const std::shared_ptr<std::pair<std::string, SensorBaseConfigMap>>&
        baseConfig,
    const std::shared_ptr<SensorData>& sensorData,
    const std::shared_ptr<std::string>& interfacePath,
    const std::shared_ptr<std::string>& devType, size_t bus, size_t addr)
{
    for (const std::pair<sdbusplus::message::object_path, SensorData>& sensor :
         sensorConfigs)
    {
        *sensorData = sensor.second;
        auto sensorBase = sensorData->find(smproDevType);
        if (sensorBase == sensorData->end())
        {
            std::cerr << "Error finding base configuration for dev " << bus
                      << ":" << addr << "\n";
            continue;
        }
        *baseConfig = *sensorBase;
        *devType = smproDevType;

        auto configBus = baseConfig->second.find("Bus");
        auto configAddress = baseConfig->second.find("Address");

        if (configBus == baseConfig->second.end() ||
            configAddress == baseConfig->second.end())
        {
            std::cerr << "Error finding necessary entry in configuration\n";
            continue;
        }

        const uint64_t* confBus = std::get_if<uint64_t>(&(configBus->second));
        const uint64_t* confAddr =
            std::get_if<uint64_t>(&(configAddress->second));
        if ((confBus == nullptr) || (confAddr == nullptr))
        {
            std::cerr << "Cannot get bus or address, invalid configuration\n";
            continue;
        }

        if ((*confBus != bus) || (*confAddr != addr))
        {
            continue;
        }
        *interfacePath = sensor.first.str;

        return true;
    }

    return false;
}

static bool matchAndCreateSensor(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::pair<std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>&
        baseConfig,
    const SensorData& sensorData, const std::string& interfacePath,
    const std::string& devType, const fs::path& sensorPath,
    const std::vector<std::string>& findLabels, PowerState readState)
{
    bool maxLabel = false;
    std::regex sensorNameRegEx("([A-Za-z]+)[0-9]*_");
    std::string sensorNameSubStr;
    std::string sensorNameStr = sensorPath.filename();
    std::string sensorPathStr = sensorPath.string();
    std::smatch matches;
    std::string labelHead;

    if (std::regex_search(sensorNameStr, matches, sensorNameRegEx))
    {
        // hwmon *_input filename without number:
        // in, curr, power, temp, ...
        try
        {
            sensorNameSubStr = matches[1];
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to match sensor name with RegEx" << '\n';
            return false;
        }
    }
    else
    {
        std::cerr << "Could not extract the alpha prefix from "
                  << sensorNameStr;
        return false;
    }

    std::string labelPath;

    /* find and differentiate _max and _input to replace "label" */
    size_t pos = sensorPathStr.find('_');
    if (pos == std::string::npos)
    {
        return false;
    }

    std::string sensorPathStrMax = sensorPathStr.substr(pos);
    if (sensorPathStrMax == "_max")
    {
        labelPath = boost::replace_all_copy(sensorPathStr, "max", "label");
        maxLabel = true;
    }
    else
    {
        labelPath = boost::replace_all_copy(sensorPathStr, "input", "label");
        maxLabel = false;
    }

    std::ifstream labelFile(labelPath);
    if (!labelFile.good())
    {
        // hwmon *_input filename with number:
        // temp1, temp2, temp3, ...
        labelHead = sensorNameStr.substr(0, sensorNameStr.find('_'));
    }
    else
    {
        std::string label;
        std::getline(labelFile, label);
        labelFile.close();
        auto findSensor = sensors.find(label);
        if (findSensor != sensors.end())
        {
            return false;
        }

        // hwmon corresponding *_label file contents:
        // vin1, vout1, ...
        labelHead = label.substr(0, label.find(' '));
    }

    /* append "max" for label */
    if (maxLabel)
    {
        labelHead.insert(0, "max");
    }

    if (!findLabels.empty())
    {
        /* Check if this labelHead is enabled in config file */
        if (std::find(findLabels.begin(), findLabels.end(), labelHead) ==
            findLabels.end())
        {
            return false;
        }
    }

    auto findProperty = propMatch.find(sensorNameSubStr);
    if (findProperty == propMatch.end())
    {
        return false;
    }

    // Protect the hardcoded propMatch list from changes,
    // by making a copy and modifying that instead.
    // Avoid bleedthrough of one device's customizations to
    // the next device, as each should be independently customizable.
    socProperties.push_back(findProperty->second);
    auto socProperty = socProperties.rbegin();

    // Use label head as prefix for reading from config file,
    // example if temp1: temp1_Name, temp1_Scale, temp1_Min, ...
    std::string keyName = labelHead + "_Name";
    std::string keyScale = labelHead + "_Scale";
    std::string keyMin = labelHead + "_Min";
    std::string keyMax = labelHead + "_Max";

    auto findCustomName = baseConfig.second.find(keyName);
    if (findCustomName == baseConfig.second.end())
    {
        return false; /* expected that each label have label name */
    }
    try
    {
        socProperty->labelTypeName =
            std::visit(VariantToStringVisitor(), findCustomName->second);
    }
    catch (std::invalid_argument&)
    {
        std::cerr << "Unable to parse " << keyName << "\n";
        return false;
    }
    std::string sensorName = socProperty->labelTypeName;
    if (sensorName.empty())
    {
        // Allow selective disabling of an individual sensor,
        // by customizing its name to an empty string.
        std::cerr << "Sensor disabled, empty string\n";
        return false;
    }

    bool customizedScale = false;
    auto findCustomScale = baseConfig.second.find(keyScale);
    if (findCustomScale != baseConfig.second.end())
    {
        try
        {
            socProperty->sensorScaleFactor =
                std::visit(VariantToDoubleVisitor(), findCustomScale->second);
        }
        catch (std::invalid_argument&)
        {
            std::cerr << "Unable to parse " << keyScale << "\n";
            return false;
        }

        // Avoid later division by zero
        if (socProperty->sensorScaleFactor > 0)
        {
            customizedScale = true;
        }
        else
        {
            std::cerr << "Unable to accept " << keyScale << "\n";
            return false;
        }
    }
    // Use device's scale factor
    double factor = socProperty->sensorScaleFactor;
    if (!customizedScale)
    {
        // Preserve existing usage of hardcoded labelMatch table below
        factor = std::pow(10.0, factor);

        /* Change first char of substring to uppercase */
        char firstChar = static_cast<char>(std::toupper(sensorNameSubStr[0]));
        std::string strScaleFactor =
            firstChar + sensorNameSubStr.substr(1) + "ScaleFactor";

        // Preserve existing configs by accepting earlier syntax,
        // example CurrScaleFactor, PowerScaleFactor, ...
        auto findScaleFactor = baseConfig.second.find(strScaleFactor);
        if (findScaleFactor != baseConfig.second.end())
        {
            factor =
                std::visit(VariantToDoubleVisitor(), findScaleFactor->second);
        }
    }

    auto findCustomMin = baseConfig.second.find(keyMin);
    if (findCustomMin != baseConfig.second.end())
    {
        try
        {
            socProperty->minReading =
                std::visit(VariantToDoubleVisitor(), findCustomMin->second);
        }
        catch (std::invalid_argument&)
        {
            std::cerr << "Unable to parse " << keyMin << "\n";
            return false;
        }
    }
    auto findCustomMax = baseConfig.second.find(keyMax);
    if (findCustomMax != baseConfig.second.end())
    {
        try
        {
            socProperty->maxReading =
                std::visit(VariantToDoubleVisitor(), findCustomMax->second);
        }
        catch (std::invalid_argument&)
        {
            std::cerr << "Unable to parse " << keyMax << "\n";
            return false;
        }
    }
    if (!(socProperty->minReading < socProperty->maxReading))
    {
        std::cerr << "Min must be less than Max\n";
        return false;
    }

    std::vector<thresholds::Threshold> sensorThresholds;
    if (!parseThresholdsFromConfig(sensorData, sensorThresholds, &labelHead))
    {
        std::cerr << "Error populating thresholds for " << sensorNameSubStr
                  << "\n";
    }

    auto findSensorType = sensorTable.find(sensorNameSubStr);
    if (findSensorType == sensorTable.end())
    {
        std::cerr << sensorNameSubStr << " is not a recognized sensor type\n";
        return false;
    }

    // destruct existing one first if already created
    sensors[sensorName] = nullptr;
    sensors[sensorName] = std::make_shared<AmpereCPUSensor>(
        sensorPathStr, devType, objectServer, dbusConnection, io, sensorName,
        std::move(sensorThresholds), interfacePath, findSensorType->second,
        factor, socProperty->maxReading, socProperty->minReading, labelHead,
        sensorThresholds.size(), readState);
    sensors[sensorName]->setupRead();

    return true;
}

static bool parseSensorConfig(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::pair<std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>&
        baseConfig,
    const SensorData& sensorData, const std::string& interfacePath,
    const std::string& sensorType, const fs::path& directory,
    unsigned int* numSensors)
{
    unsigned int numCreated = 0;
    std::vector<fs::path> sensorPaths;

    auto findSOCName = baseConfig.second.find("Name");
    if (findSOCName == baseConfig.second.end())
    {
        std::cerr << "Could not determine configuration name for "
                  << interfacePath << "\n";
        return false;
    }

    if (!findFiles(directory, R"(\w\d+_input$)", sensorPaths, 0))
    {
        std::cerr << "No SOC non-label sensor in SOC\n";
        return false;
    }
    /* read max value in sysfs for in, curr, power, temp, ... */
    if (!findFiles(directory, R"(\w\d+_max$)", sensorPaths, 0))
    {
        std::cout << "No max name in SOC \n";
    }

    PowerState readState = getPowerState(baseConfig.second);

    /* Find array of labels to be exposed if it is defined in config */
    std::vector<std::string> findLabels;
    auto findLabelObj = baseConfig.second.find("Labels");
    if (findLabelObj != baseConfig.second.end())
    {
        findLabels = std::get<std::vector<std::string>>(findLabelObj->second);
    }
    for (const auto& sensorPath : sensorPaths)
    {
        if (matchAndCreateSensor(io, objectServer, dbusConnection, baseConfig,
                                 sensorData, interfacePath, sensorType,
                                 sensorPath, findLabels, readState))
        {
            numCreated++;
        }
    }
    *numSensors = numCreated;

    return true;
}

static void createSensorsCallback(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const ManagedObjectType& sensorConfigs)
{
    std::vector<fs::path> busPaths;

    if (!findFiles(fs::path("/sys/class/hwmon"), "name", busPaths))
    {
        std::cerr << "No SOC sensors in system\n";
        return;
    }

    boost::container::flat_set<std::string> directories;
    for (const auto& busPath : busPaths)
    {
        boost::container::flat_map<std::string, std::vector<std::string>>
            eventPathList;
        boost::container::flat_map<
            std::string,
            boost::container::flat_map<std::string, std::vector<std::string>>>
            groupEventPathList;

        std::ifstream nameFile(busPath);
        if (!nameFile.good())
        {
            std::cerr << "Failure finding SoC sensors path " << busPath << "\n";
            continue;
        }

        std::string busName;
        std::getline(nameFile, busName);
        nameFile.close();
        if (busName != smproDrvName)
        {
            continue;
        }

        auto directory = busPath.parent_path();
        auto ret = directories.insert(directory.string());
        if (!ret.second)
        {
            std::cerr << "Duplicate path " << directory.string() << "\n";
            continue;
        }

        size_t bus = 0;
        size_t addr = 0;
        fs::path device = directory / "device";
        std::string devPath = fs::canonical(device);
        if (!getDeviceInfo(devPath, &bus, &addr))
        {
            continue;
        }

        std::shared_ptr<std::pair<std::string, SensorBaseConfigMap>>
            baseConfig =
                std::make_shared<std::pair<std::string, SensorBaseConfigMap>>();
        std::shared_ptr<SensorData> sensorData = std::make_shared<SensorData>();
        std::shared_ptr<std::string> interfacePath =
            std::make_shared<std::string>();
        std::shared_ptr<std::string> devType = std::make_shared<std::string>();

        if (!findMatchConfig(sensorConfigs, baseConfig, sensorData,
                             interfacePath, devType, bus, addr))
        {
            std::cerr << "Failed to find match for " << devPath << "\n";
            continue;
        }

        unsigned int numCreated = 0;
        parseSensorConfig(io, objectServer, dbusConnection, *baseConfig,
                          *sensorData, *interfacePath, *devType, directory,
                          &numCreated);

        std::cerr << "Device " << bus << ":" << addr << " have " << numCreated
                  << " sensors \n";
    }
}

void createSensors(boost::asio::io_service& io,
                   sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&io, &objectServer,
         &dbusConnection](const ManagedObjectType& sensorConfigs) {
        createSensorsCallback(io, objectServer, dbusConnection, sensorConfigs);
        });
    getter->getConfiguration(std::vector<std::string>{smproDrvName});
}

void propertyInitialize()
{
    sensorTable = {{"power", sensor_paths::unitWatts},
                   {"curr", sensor_paths::unitAmperes},
                   {"temp", sensor_paths::unitDegreesC},
                   {"in", sensor_paths::unitVolts}};

    propMatch = {{"power", AmpereCPUProperty("Power Property", 200, 0, 1)},
                 {"curr", AmpereCPUProperty("Curr property", 100, 0, 1)},
                 {"temp", AmpereCPUProperty("Temp property", 255, 0, 1)},
                 {"in", AmpereCPUProperty("Voltage property", 100, 0, 1)}};
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);

    systemBus->request_name(ampereDbusName);
    sdbusplus::asio::object_server objectServer(systemBus);
    propertyInitialize();

    io.post([&]() { createSensors(io, objectServer, systemBus); });
    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
        if (message.is_method_error())
        {
            std::cerr << "Callback method error\n";
            return;
        }

        filterTimer.expires_from_now(std::chrono::seconds(1));
        filterTimer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return;
            }
            if (ec)
            {
                std::cerr << "timer error\n";
            }
            createSensors(io, objectServer, systemBus);
        });
    };

    auto matchPropChanged = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + smproDevType +
            "'",
        eventHandler);

    io.run();

    return 0;
}
