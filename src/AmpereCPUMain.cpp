/*
// Copyright 2021 Ampere Computing LLC
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

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <regex>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

static constexpr bool debug = false;

static constexpr std::array<const char*, 1> sensorTypes = {
    "xyz.openbmc_project.Configuration.smpro_hwmon"};

static std::vector<std::string> busNames = {"smpro_hwmon"};

namespace fs = std::filesystem;

static boost::container::flat_map<std::string, std::shared_ptr<CPUSensor>>
    sensors;
static boost::container::flat_map<std::string, std::string> sensorTable;
static boost::container::flat_map<std::string, SoCProperty> propMatch;
static boost::container::flat_map<std::string, int> numCPUSensors;
boost::container::flat_map<std::string,
                           std::shared_ptr<sdbusplus::asio::dbus_interface>>
    inventoryIfaces;
std::vector<std::string> socInventNames;

static std::vector<SoCProperty> socProperties;
static std::regex i2cDevRegex(R"((\d+)-([a-fA-F0-9]+))");

bool getDeviceInfo(const std::string& devPath, size_t* bus, size_t* addr)
{
    std::smatch match;
    std::regex_search(devPath, match, i2cDevRegex);

    if (match.empty() || (match.size() != 3))
    {
        std::cerr << "found bad device path " << devPath << "\n";
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

bool findMatchConfig(
    const ManagedObjectType& sensorConfigs,
    std::pair<std::string,
              boost::container::flat_map<std::string, BasicVariantType>>*
        baseConfig,
    SensorData* sensorData, std::string* interfacePath, std::string* sensorType,
    size_t* thresholdConfSize, size_t bus, size_t addr,
    const std::string& deviceName, std::string* socName)
{
    for (const std::pair<sdbusplus::message::object_path, SensorData>& sensor :
         sensorConfigs)
    {
        bool matchedConfig = false;
        *sensorData = sensor.second;
        for (const char* type : sensorTypes)
        {
            auto sensorBase = sensorData->find(type);
            if (sensorBase != sensorData->end())
            {
                *baseConfig = *sensorBase;
                *sensorType = type;
                matchedConfig = true;
                break;
            }
        }
        if (!matchedConfig)
        {
            std::cerr << "error finding base configuration for " << deviceName
                      << " Type " << *sensorType << "\n";
            continue;
        }

        auto configBus = baseConfig->second.find("Bus");
        auto configAddress = baseConfig->second.find("Address");

        if (configBus == baseConfig->second.end() ||
            configAddress == baseConfig->second.end())
        {
            std::cerr << "error finding necessary entry in configuration\n";
            continue;
        }

        const uint64_t* confBus;
        const uint64_t* confAddr;
        if (!(confBus = std::get_if<uint64_t>(&(configBus->second))) ||
            !(confAddr = std::get_if<uint64_t>(&(configAddress->second))))
        {
            std::cerr << "Cannot get bus or address, invalid configuration\n";
            continue;
        }

        if ((*confBus != bus) || (*confAddr != addr))
        {
            std::cerr << "Configuration skipping " << *confBus << "-"
                      << *confAddr << " because not " << bus << "-" << addr
                      << "\n";
            continue;
        }

        auto findSOCName = baseConfig->second.find("Name");
        if (findSOCName == baseConfig->second.end())
        {
            std::cerr << "could not determine configuration name for "
                      << *interfacePath << "\n";
            return false;
        }
        std::string name =
            std::visit(VariantToStringVisitor(), findSOCName->second);
        if (std::empty(name))
        {
            std::cerr << "Cannot find soc name, invalid configuration\n";
            return false;
        }
        *socName = std::regex_replace(name, illegalDbusRegex, "_");

        std::vector<thresholds::Threshold> confThresholds;
        if (!parseThresholdsFromConfig(*sensorData, confThresholds))
        {
            std::cerr << "error populating totoal thresholds\n";
        }
        *thresholdConfSize = confThresholds.size();
        *interfacePath = sensor.first.str;

        return true;
    }

    return false;
}

static bool rescanSensors(
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    const std::string& socName)
{
    if (sensorsChanged == nullptr)
    {
        return true;
    }

    std::string socNameStr = "/" + socName;
    auto it = std::find_if(sensorsChanged->begin(), sensorsChanged->end(),
                           [socNameStr](std::string& s) {
                               return boost::ends_with(s, socNameStr);
                           });
    if (it == sensorsChanged->end())
    {
        return false;
    }
    sensorsChanged->erase(it);

    return true;
}

static bool matchSensor(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::pair<std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
        baseConfig,
    SensorData* sensorData, std::string* interfacePath,
    const std::string& sensorType, size_t thresholdConfSize,
    const fs::path& sensorPath, std::vector<std::string> socNames,
    std::vector<std::string> findLabels, PowerState readState)
{
    bool maxLabel = false;
    std::regex sensorNameRegEx("([A-Za-z]+)[0-9]*_");
    std::string sensorNameSubStr{""};
    std::string sensorNameStr = sensorPath.filename();
    std::string sensorPathStr = sensorPath.string();
    std::smatch matches;
    std::string labelHead;

    if (std::regex_search(sensorNameStr, matches, sensorNameRegEx))
    {
        // hwmon *_input filename without number:
        // in, curr, power, temp, ...
        sensorNameSubStr = matches[1];
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
    if (sensorPathStrMax.compare("_max") == 0)
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
        if constexpr (debug)
        {
            std::cerr << "Input file " << sensorPath
                      << " has no corresponding label file\n";
        }
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

    findProperty->second;

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

    bool customizedName = false;
    auto findCustomName = baseConfig->second.find(keyName);
    if (findCustomName != baseConfig->second.end())
    {
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

        // All strings are valid, including empty string
        customizedName = true;
    }

    bool customizedScale = false;
    auto findCustomScale = baseConfig->second.find(keyScale);
    if (findCustomScale != baseConfig->second.end())
    {
        try
        {
            socProperty->sensorScaleFactor = std::visit(
                VariantToUnsignedIntVisitor(), findCustomScale->second);
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

    auto findCustomMin = baseConfig->second.find(keyMin);
    if (findCustomMin != baseConfig->second.end())
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

    auto findCustomMax = baseConfig->second.find(keyMax);
    if (findCustomMax != baseConfig->second.end())
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

    // If the sensor name is being customized by config file,
    // then prefix/suffix composition becomes not necessary,
    // and in fact not wanted, because it gets in the way.
    std::string socNameFromIndex;
    if (!customizedName)
    {
        /* Find out sensor name index for this label */
        std::regex rgx("[A-Za-z]+([0-9]+)");
        size_t nameIndex{0};
        if (std::regex_search(labelHead, matches, rgx))
        {
            nameIndex = std::stoi(matches[1]);

            // Decrement to preserve alignment, because hwmon
            // human-readable filenames and labels use 1-based
            // numbering, but the "Name", "Name1", "Name2", etc. naming
            // convention (the socNames vector) uses 0-based numbering.
            if (nameIndex > 0)
            {
                --nameIndex;
            }
        }
        else
        {
            nameIndex = 0;
        }

        if (socNames.size() <= nameIndex)
        {
            std::cerr << "Could not pair " << labelHead
                      << " with a Name field\n";
            return false;
        }

        socNameFromIndex = socNames[nameIndex];
    }

    // Similarly, if sensor scaling factor is being customized,
    // then the below power-of-10 constraint becomes unnecessary,
    // as config should be able to specify an arbitrary divisor.
    unsigned int factor = socProperty->sensorScaleFactor;
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
        auto findScaleFactor = baseConfig->second.find(strScaleFactor);
        if (findScaleFactor != baseConfig->second.end())
        {
            factor = std::visit(VariantToIntVisitor(), findScaleFactor->second);
        }
    }

    std::vector<thresholds::Threshold> sensorThresholds;
    if (!parseThresholdsFromConfig(*sensorData, sensorThresholds, &labelHead))
    {
        std::cerr << "error populating thresholds for " << sensorNameSubStr
                  << "\n";
    }

    auto findSensorType = sensorTable.find(sensorNameSubStr);
    if (findSensorType == sensorTable.end())
    {
        std::cerr << sensorNameSubStr << " is not a recognized sensor type\n";
        return false;
    }

    if constexpr (debug)
    {
        std::cerr << "Sensor properties: Name \"" << socProperty->labelTypeName
                  << "\" Scale " << socProperty->sensorScaleFactor << " Min "
                  << socProperty->minReading << " Max "
                  << socProperty->maxReading << "\n";
    }

    std::string sensorName = socProperty->labelTypeName;
    if (customizedName)
    {
        if (sensorName.empty())
        {
            // Allow selective disabling of an individual sensor,
            // by customizing its name to an empty string.
            std::cerr << "Sensor disabled, empty string\n";
            return false;
        }
    }
    else
    {
        // Sensor name not customized, do prefix/suffix composition,
        // preserving default behavior by using socNameFromIndex.
        sensorName = socNameFromIndex + " " + socProperty->labelTypeName;
    }

    if constexpr (debug)
    {
        std::cerr << "Sensor name \"" << sensorName << "\" path \""
                  << sensorPathStr << "\" type \"" << sensorType << "\"\n";
    }

    // destruct existing one first if already created
    sensors[sensorName] = nullptr;
    sensors[sensorName] = std::make_shared<CPUSensor>(
        sensorPathStr, sensorType, objectServer, dbusConnection, io, sensorName,
        std::move(sensorThresholds), *interfacePath, findSensorType->second,
        factor, socProperty->maxReading, socProperty->minReading, labelHead,
        thresholdConfSize, readState);
    sensors[sensorName]->setupRead();

    return true;
}

bool addSoCInventory(sdbusplus::asio::object_server& objectServer,
                     const std::string& name, bool present)
{
    std::string socName = std::regex_replace(name, illegalDbusRegex, "_");
    if (inventoryIfaces.find(socName) == inventoryIfaces.end())
    {
        std::cerr << "Add inventory " << socName << ":" << present << "\n";
        auto iface = objectServer.add_interface(
            cpuInventoryPath + std::string("/") + socName,
            "xyz.openbmc_project.Inventory.Item");
        iface->register_property("PrettyName", socName);
        iface->register_property("Present", present);
        iface->initialize();
        inventoryIfaces[socName] = std::move(iface);
    }

    return true;
}

static bool parseSensorConfig(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::pair<std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
        baseConfig,
    SensorData* sensorData, std::string* interfacePath,
    const std::string& sensorType, size_t thresholdConfSize,
    const fs::path& directory, int* numSensors)
{
    int numCreated = 0;

    std::vector<fs::path> sensorPaths;
    if (!findFiles(directory, R"(\w\d+_input$)", sensorPaths, 0))
    {
        std::cerr << "No SOC non-label sensor in SOC\n";
        return false;
    }

    auto findSOCName = baseConfig->second.find("Name");
    if (findSOCName == baseConfig->second.end())
    {
        std::cerr << "could not determine configuration name for "
                  << *interfacePath << "\n";
        return false;
    }

    /* Check if there are more sensors in the same interface */
    std::vector<std::string> socNames;
    int idx = 0;
    do
    {
        // Individual string fields: Name, Name1, Name2, Name3, ...
        socNames.push_back(std::get<std::string>(findSOCName->second));
        findSOCName = baseConfig->second.find("Name" + std::to_string(idx++));
    } while (findSOCName != baseConfig->second.end());

    /* read max value in sysfs for in, curr, power, temp, ... */
    if (!findFiles(directory, R"(\w\d+_max$)", sensorPaths, 0))
    {
        std::cout << "No max name in SOC \n";
    }

    auto findPowerOn = baseConfig->second.find("PowerState");
    PowerState readState = PowerState::always;
    if (findPowerOn != baseConfig->second.end())
    {
        std::string powerState =
            std::visit(VariantToStringVisitor(), findPowerOn->second);
        setReadState(powerState, readState);
    }

    /* Find array of labels to be exposed if it is defined in config */
    std::vector<std::string> findLabels;
    auto findLabelObj = baseConfig->second.find("Labels");
    if (findLabelObj != baseConfig->second.end())
    {
        findLabels = std::get<std::vector<std::string>>(findLabelObj->second);
    }
    for (const auto& sensorPath : sensorPaths)
    {
        auto ret = matchSensor(io, objectServer, dbusConnection, baseConfig,
                               sensorData, interfacePath, sensorType,
                               thresholdConfSize, sensorPath, socNames,
                               findLabels, readState);
        if (!ret)
        {
            continue;
        }
        numCreated++;
    }

    *numSensors = numCreated;
    return true;
}

bool createCpuPresentDbus(sdbusplus::asio::object_server& objectServer,
                          const ManagedObjectType& sensorConfigs)
{
    const std::pair<std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
        baseConfig = nullptr;
    const SensorData* sensorData = nullptr;
    for (const std::pair<sdbusplus::message::object_path, SensorData>& sensor :
         sensorConfigs)
    {
        sensorData = &(sensor.second);
        for (const char* type : sensorTypes)
        {
            auto sensorBase = sensorData->find(type);
            if (sensorBase != sensorData->end())
            {
                baseConfig = &(*sensorBase);
                break;
            }
        }
        if (baseConfig == nullptr)
        {
            std::cerr << "Can find SoC sensor type " << std::endl;
            continue;
        }

        auto findSOCName = baseConfig->second.find("Name");
        if (findSOCName == baseConfig->second.end())
        {
            std::cerr << "could not determine configuration" << std::endl;
            continue;
        }

        std::string socName =
            std::visit(VariantToStringVisitor(), findSOCName->second);
        if (std::empty(socName))
        {
            std::cerr << "Cannot find soc name, invalid configuration\n";
            continue;
        }

        auto present = cpuIsPresent(sensorData);
        addSoCInventory(objectServer, socName, present);
    }

    return true;
}

static void createSensorsCallback(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const ManagedObjectType& sensorConfigs,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    std::vector<fs::path> busPaths;

    if (inventoryIfaces.size() == 0)
    {
        if (!createCpuPresentDbus(objectServer, sensorConfigs))
        {
            std::cerr << "Can not find SoC config " << std::endl;
            return;
        }
    }

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
            std::cerr << "Failure finding pmbus path " << busPath << "\n";
            continue;
        }

        std::string busName;
        std::getline(nameFile, busName);
        nameFile.close();
        if (std::find(busNames.begin(), busNames.end(), busName) ==
            busNames.end())
        {
            // To avoid this error message, add your driver name to
            // the busNames vector at the top of this file.
            std::cerr << "Driver name " << busName
                      << " not found in sensor whitelist\n";
            continue;
        }

        auto directory = busPath.parent_path();
        auto ret = directories.insert(directory.string());
        if (!ret.second)
        {
            std::cerr << "Duplicate path " << directory.string() << "\n";
            continue; // check if path has already been searched
        }

        size_t bus = 0;
        size_t addr = 0;
        fs::path device = directory / "device";
        std::string devPath = fs::canonical(device);
        if (!getDeviceInfo(devPath, &bus, &addr))
        {
            continue;
        }

        std::pair<std::string,
                  boost::container::flat_map<std::string, BasicVariantType>>
            baseConfig;
        SensorData sensorData;
        std::string interfacePath;
        std::string socName;
        std::string sensorType;
        size_t thresholdConfSize = 0;
        bool retVal = findMatchConfig(
            sensorConfigs, &baseConfig, &sensorData, &interfacePath,
            &sensorType, &thresholdConfSize, bus, addr, devPath, &socName);
        if (!retVal)
        {
            std::cerr << "failed to find match for " << socName << "\n";
            continue;
        }

        if ((numCPUSensors.find(socName) != numCPUSensors.end()) &&
            (numCPUSensors[socName] > 0))
        {
            if (!rescanSensors(sensorsChanged, socName))
            {
                continue;
            }
        }

        int numCreated = 0;
        parseSensorConfig(io, objectServer, dbusConnection, &baseConfig,
                          &sensorData, &interfacePath, sensorType,
                          thresholdConfSize, directory, &numCreated);
        numCPUSensors[socName] = numCreated;
        std::cerr << "Device " << socName;
        std::cerr << " created " << numCreated << " sensors \n";
    }

    return;
}

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection, [&io, &objectServer, &dbusConnection, sensorsChanged](
                            const ManagedObjectType& sensorConfigs) {
            if (sensorConfigs.size() > 0)
            {
                createSensorsCallback(io, objectServer, dbusConnection,
                                      sensorConfigs, sensorsChanged);
            }
        });
    getter->getConfiguration(
        std::vector<std::string>(sensorTypes.begin(), sensorTypes.end()));
}

void propertyInitialize(void)
{
    sensorTable = {{"power", "power/"},
                   {"curr", "current/"},
                   {"temp", "temperature/"},
                   {"in", "voltage/"}};

    propMatch = {{"power", SoCProperty("Power Property", 30000, 0, 1)},
                 {"curr", SoCProperty("Curr property", 30000, 0, 1)},
                 {"temp", SoCProperty("Temp property", 255, 0, 1)},
                 {"in", SoCProperty("Voltage property", 30000, 0, 1)}};
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);

    systemBus->request_name("xyz.openbmc_project.CPUSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    propertyInitialize();

    io.post([&]() { createSensors(io, objectServer, systemBus, nullptr); });
    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            sensorsChanged->insert(message.get_path());
            filterTimer.expires_from_now(boost::posix_time::seconds(5));
            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return;
                }
                if (ec)
                {
                    std::cerr << "timer error\n";
                }
                createSensors(io, objectServer, systemBus, sensorsChanged);
            });
        };

    std::function<void(sdbusplus::message::message&)> hostStateHandler =
        [&](sdbusplus::message::message& message) {
            std::string objectName;
            boost::container::flat_map<std::string, std::variant<std::string>>
                values;
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }

            message.read(objectName, values);
            auto findState = values.find(power::property);
            if (findState == values.end())
            {
                return;
            }

            bool on = boost::ends_with(std::get<std::string>(findState->second),
                                       ".Running");
            /* only rescan the host sensors when the host changed to Running */
            if (!on)
            {
                return;
            }

            filterTimer.expires_from_now(boost::posix_time::seconds(5));
            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return;
                }
                if (ec)
                {
                    std::cerr << "timer error\n";
                }
                createSensors(io, objectServer, systemBus, nullptr);
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
    auto match = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',interface='" + std::string(properties::interface) +
            "',path='" + std::string(power::path) + "',arg0='" +
            std::string(power::interface) + "'",
        hostStateHandler);
    matches.emplace_back(std::move(match));

    io.run();
    return 0;
}
