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

#include "DeviceMgmt.hpp"
#include "PSUEvent.hpp"
#include "PSUSensor.hpp"
#include "PwmSensor.hpp"
#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/algorithm/string/case_conv.hpp>
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
#include <sdbusplus/exception.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

static std::regex i2cDevRegex(R"((\/i2c\-\d+\/\d+-[a-fA-F0-9]{4,4})(\/|$))");

static const I2CDeviceTypeMap sensorTypes{
    {"ADC128D818", I2CDeviceType{"adc128d818", true}},
    {"ADM1266", I2CDeviceType{"adm1266", true}},
    {"ADM1272", I2CDeviceType{"adm1272", true}},
    {"ADM1275", I2CDeviceType{"adm1275", true}},
    {"ADM1278", I2CDeviceType{"adm1278", true}},
    {"ADM1293", I2CDeviceType{"adm1293", true}},
    {"ADS1015", I2CDeviceType{"ads1015", true}},
    {"ADS7830", I2CDeviceType{"ads7830", true}},
    {"AHE50DC_FAN", I2CDeviceType{"ahe50dc_fan", true}},
    {"BMR490", I2CDeviceType{"bmr490", true}},
    {"cffps", I2CDeviceType{"cffps", true}},
    {"cffps1", I2CDeviceType{"cffps", true}},
    {"cffps2", I2CDeviceType{"cffps", true}},
    {"cffps3", I2CDeviceType{"cffps", true}},
    {"CRPS185", I2CDeviceType{"crps185", true}},
    {"DPS800", I2CDeviceType{"dps800", true}},
    {"INA219", I2CDeviceType{"ina219", true}},
    {"INA226", I2CDeviceType{"ina226", true}},
    {"INA230", I2CDeviceType{"ina230", true}},
    {"INA233", I2CDeviceType{"ina233", true}},
    {"INA238", I2CDeviceType{"ina238", true}},
    {"IPSPS1", I2CDeviceType{"ipsps1", true}},
    {"IR35221", I2CDeviceType{"ir35221", true}},
    {"IR38060", I2CDeviceType{"ir38060", true}},
    {"IR38164", I2CDeviceType{"ir38164", true}},
    {"IR38263", I2CDeviceType{"ir38263", true}},
    {"ISL28022", I2CDeviceType{"isl28022", true}},
    {"ISL68137", I2CDeviceType{"isl68137", true}},
    {"ISL68220", I2CDeviceType{"isl68220", true}},
    {"ISL68223", I2CDeviceType{"isl68223", true}},
    {"ISL69225", I2CDeviceType{"isl69225", true}},
    {"ISL69243", I2CDeviceType{"isl69243", true}},
    {"ISL69260", I2CDeviceType{"isl69260", true}},
    {"LM25066", I2CDeviceType{"lm25066", true}},
    {"LM5066I", I2CDeviceType{"lm5066i", true}},
    {"LTC2945", I2CDeviceType{"ltc2945", true}},
    {"LTC4286", I2CDeviceType{"ltc4286", true}},
    {"LTC4287", I2CDeviceType{"ltc4287", true}},
    {"MAX5970", I2CDeviceType{"max5970", true}},
    {"MAX11607", I2CDeviceType{"max11607", false}},
    {"MAX11615", I2CDeviceType{"max11615", false}},
    {"MAX11617", I2CDeviceType{"max11617", false}},
    {"MAX16601", I2CDeviceType{"max16601", true}},
    {"MAX20710", I2CDeviceType{"max20710", true}},
    {"MAX20730", I2CDeviceType{"max20730", true}},
    {"MAX20734", I2CDeviceType{"max20734", true}},
    {"MAX20796", I2CDeviceType{"max20796", true}},
    {"MAX34451", I2CDeviceType{"max34451", true}},
    {"MP2856", I2CDeviceType{"mp2856", true}},
    {"MP2857", I2CDeviceType{"mp2857", true}},
    {"MP2869", I2CDeviceType{"mp2869", true}},
    {"MP2971", I2CDeviceType{"mp2971", true}},
    {"MP2973", I2CDeviceType{"mp2973", true}},
    {"MP2975", I2CDeviceType{"mp2975", true}},
    {"MP2993", I2CDeviceType{"mp2993", true}},
    {"MP5023", I2CDeviceType{"mp5023", true}},
    {"MP5990", I2CDeviceType{"mp5990", true}},
    {"MP5998", I2CDeviceType{"mp5998", true}},
    {"MP9945", I2CDeviceType{"mp9945", true}},
    {"MP29612", I2CDeviceType{"mp29612", true}},
    {"MPQ8785", I2CDeviceType{"mpq8785", true}},
    {"NCP4200", I2CDeviceType{"ncp4200", true}},
    {"PLI1209BC", I2CDeviceType{"pli1209bc", true}},
    {"pmbus", I2CDeviceType{"pmbus", true}},
    {"PXE1610", I2CDeviceType{"pxe1610", true}},
    {"SQ52206", I2CDeviceType{"sq52206", true}},
    {"RAA228000", I2CDeviceType{"raa228000", true}},
    {"RAA228004", I2CDeviceType{"raa228004", true}},
    {"RAA228006", I2CDeviceType{"raa228006", true}},
    {"RAA228228", I2CDeviceType{"raa228228", true}},
    {"RAA228620", I2CDeviceType{"raa228620", true}},
    {"RAA229001", I2CDeviceType{"raa229001", true}},
    {"RAA229004", I2CDeviceType{"raa229004", true}},
    {"RAA229126", I2CDeviceType{"raa229126", true}},
    {"RTQ6056", I2CDeviceType{"rtq6056", false}},
    {"SBRMI", I2CDeviceType{"sbrmi", true}},
    {"smpro_hwmon", I2CDeviceType{"smpro", false}},
    {"SY24655", I2CDeviceType{"sy24655", true}},
    {"TDA38640", I2CDeviceType{"tda38640", true}},
    {"TPS25990", I2CDeviceType{"tps25990", true}},
    {"TPS53679", I2CDeviceType{"tps53679", true}},
    {"TPS546D24", I2CDeviceType{"tps546d24", true}},
    {"XDP710", I2CDeviceType{"xdp710", true}},
    {"XDPE11280", I2CDeviceType{"xdpe11280", true}},
    {"XDPE12284", I2CDeviceType{"xdpe12284", true}},
    {"XDPE152C4", I2CDeviceType{"xdpe152c4", true}},
};

enum class DevTypes
{
    Unknown = 0,
    HWMON,
    IIO
};

struct DevParams
{
    unsigned int matchIndex = 0;
    std::string matchRegEx;
    std::string nameRegEx;
};

static boost::container::flat_map<std::string, std::shared_ptr<PSUSensor>>
    sensors;
static boost::container::flat_map<std::string, std::unique_ptr<PSUCombineEvent>>
    combineEvents;
static boost::container::flat_map<std::string, std::unique_ptr<PwmSensor>>
    pwmSensors;
static boost::container::flat_map<std::string, std::string> sensorTable;
static boost::container::flat_map<std::string, PSUProperty> labelMatch;
static EventPathList eventMatch;
static EventPathList limitEventMatch;

static boost::container::flat_map<size_t, bool> cpuPresence;
static boost::container::flat_map<DevTypes, DevParams> devParamMap;

// Function CheckEvent will check each attribute from eventMatch table in the
// sysfs. If the attributes exists in sysfs, then store the complete path
// of the attribute into eventPathList.
void checkEvent(const std::string& directory, const EventPathList& eventMatch,
                EventPathList& eventPathList)
{
    for (const auto& match : eventMatch)
    {
        const std::vector<std::string>& eventAttrs = match.second;
        const std::string& eventName = match.first;
        for (const auto& eventAttr : eventAttrs)
        {
            std::string eventPath = directory;
            eventPath += "/";
            eventPath += eventAttr;

            std::ifstream eventFile(eventPath);
            if (!eventFile.good())
            {
                continue;
            }

            eventPathList[eventName].push_back(eventPath);
        }
    }
}

// Check Group Events which contains more than one targets in each combine
// events.
void checkGroupEvent(const std::string& directory,
                     GroupEventPathList& groupEventPathList)
{
    EventPathList pathList;
    std::vector<std::filesystem::path> eventPaths;
    if (!findFiles(std::filesystem::path(directory), R"(fan\d+_(alarm|fault))",
                   eventPaths))
    {
        return;
    }

    for (const auto& eventPath : eventPaths)
    {
        std::string attrName = eventPath.filename();
        pathList[attrName.substr(0, attrName.find('_'))].push_back(eventPath);
    }
    groupEventPathList["FanFault"] = pathList;
}

// Function checkEventLimits will check all the psu related xxx_input attributes
// in sysfs to see if xxx_crit_alarm xxx_lcrit_alarm xxx_max_alarm
// xxx_min_alarm exist, then store the existing paths of the alarm attributes
// to eventPathList.
void checkEventLimits(const std::string& sensorPathStr,
                      const EventPathList& limitEventMatch,
                      EventPathList& eventPathList)
{
    auto attributePartPos = sensorPathStr.find_last_of('_');
    if (attributePartPos == std::string::npos)
    {
        // There is no '_' in the string, skip it
        return;
    }
    auto attributePart =
        std::string_view(sensorPathStr).substr(attributePartPos + 1);
    if (attributePart != "input")
    {
        // If the sensor is not xxx_input, skip it
        return;
    }

    auto prefixPart = sensorPathStr.substr(0, attributePartPos + 1);
    for (const auto& limitMatch : limitEventMatch)
    {
        const std::vector<std::string>& limitEventAttrs = limitMatch.second;
        const std::string& eventName = limitMatch.first;
        for (const auto& limitEventAttr : limitEventAttrs)
        {
            auto limitEventPath = prefixPart + limitEventAttr;
            std::ifstream eventFile(limitEventPath);
            if (!eventFile.good())
            {
                continue;
            }
            eventPathList[eventName].push_back(limitEventPath);
        }
    }
}

static void checkPWMSensor(
    const std::filesystem::path& sensorPath, std::string& labelHead,
    const std::string& interfacePath,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    sdbusplus::asio::object_server& objectServer, const std::string& psuName)
{
    if (!labelHead.starts_with("fan"))
    {
        return;
    }
    std::string labelHeadIndex = labelHead.substr(3);

    const std::string& sensorPathStr = sensorPath.string();
    const std::string& pwmPathStr =
        boost::replace_all_copy(sensorPathStr, "input", "target");
    std::ifstream pwmFile(pwmPathStr);
    if (!pwmFile.good())
    {
        return;
    }

    auto findPWMSensor = pwmSensors.find(psuName + labelHead);
    if (findPWMSensor != pwmSensors.end())
    {
        return;
    }

    std::string name = "Pwm_";
    name += psuName;
    name += "_Fan_";
    name += labelHeadIndex;

    std::string objPath = interfacePath;
    objPath += "_Fan_";
    objPath += labelHeadIndex;

    pwmSensors[psuName + labelHead] = std::make_unique<PwmSensor>(
        name, pwmPathStr, dbusConnection, objectServer, objPath, "PSU");
}

static void createSensorsCallback(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const ManagedObjectType& sensorConfigs,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    bool activateOnly)
{
    int numCreated = 0;
    bool firstScan = sensorsChanged == nullptr;

    auto devices = instantiateDevices(sensorConfigs, sensors, sensorTypes);

    std::vector<std::filesystem::path> pmbusPaths;
    findFiles(std::filesystem::path("/sys/bus/iio/devices"), "name",
              pmbusPaths);
    findFiles(std::filesystem::path("/sys/class/hwmon"), "name", pmbusPaths);
    if (pmbusPaths.empty())
    {
        lg2::error("No PSU sensors in system");
        return;
    }

    boost::container::flat_set<std::string> directories;
    for (const auto& pmbusPath : pmbusPaths)
    {
        EventPathList eventPathList;
        GroupEventPathList groupEventPathList;

        std::ifstream nameFile(pmbusPath);
        if (!nameFile.good())
        {
            lg2::error("Failure finding '{PATH}'", "PATH", pmbusPath);
            continue;
        }

        std::string pmbusName;
        std::getline(nameFile, pmbusName);
        nameFile.close();

        if (!sensorTypes.contains(pmbusName))
        {
            // To avoid this error message, add your driver name to
            // the pmbusNames vector at the top of this file.
            lg2::error("'{NAME}' not found in sensor whitelist", "NAME",
                       pmbusName);
            continue;
        }

        auto directory = pmbusPath.parent_path();

        auto ret = directories.insert(directory.string());
        if (!ret.second)
        {
            lg2::error("Duplicate path: '{PATH}'", "PATH", directory);
            continue; // check if path has already been searched
        }

        DevTypes devType = DevTypes::HWMON;
        std::string deviceName;
        if (directory.parent_path() == "/sys/class/hwmon")
        {
            std::string devicePath =
                std::filesystem::canonical(directory / "device");
            std::smatch match;
            // Find /i2c-<bus>/<bus>-<address> match in device path
            std::regex_search(devicePath, match, i2cDevRegex);
            if (match.empty())
            {
                lg2::error("Found bad device path: '{PATH}'", "PATH",
                           devicePath);
                continue;
            }
            // Extract <bus>-<address>
            std::string matchStr = match[1];
            deviceName = matchStr.substr(matchStr.find_last_of('/') + 1);
        }
        else
        {
            deviceName =
                std::filesystem::canonical(directory).parent_path().stem();
            devType = DevTypes::IIO;
        }

        size_t bus = 0;
        size_t addr = 0;
        if (!getDeviceBusAddr(deviceName, bus, addr))
        {
            continue;
        }

        const SensorBaseConfigMap* baseConfig = nullptr;
        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        std::string sensorType;
        size_t thresholdConfSize = 0;

        for (const auto& [path, cfgData] : sensorConfigs)
        {
            sensorData = &cfgData;
            for (const auto& [type, dt] : sensorTypes)
            {
                auto sensorBase = sensorData->find(configInterfaceName(type));
                if (sensorBase != sensorData->end())
                {
                    baseConfig = &sensorBase->second;
                    sensorType = type;
                    break;
                }
            }
            if (baseConfig == nullptr)
            {
                lg2::error("error finding base configuration for '{NAME}'",
                           "NAME", deviceName);
                continue;
            }

            auto configBus = baseConfig->find("Bus");
            auto configAddress = baseConfig->find("Address");

            if (configBus == baseConfig->end() ||
                configAddress == baseConfig->end())
            {
                lg2::error("error finding necessary entry in configuration");
                continue;
            }

            const uint64_t* confBus =
                std::get_if<uint64_t>(&(configBus->second));
            const uint64_t* confAddr =
                std::get_if<uint64_t>(&(configAddress->second));
            if (confBus == nullptr || confAddr == nullptr)
            {
                lg2::error("Cannot get bus or address, invalid configuration");
                continue;
            }

            if ((*confBus != bus) || (*confAddr != addr))
            {
                lg2::debug(
                    "Configuration skipping '{CONFBUS}'-'{CONFADDR}' because not {BUS}-{ADDR}",
                    "CONFBUS", *confBus, "CONFADDR", *confAddr, "BUS", bus,
                    "ADDR", addr);
                continue;
            }

            std::vector<thresholds::Threshold> confThresholds;
            if (!parseThresholdsFromConfig(*sensorData, confThresholds))
            {
                lg2::error("error populating total thresholds");
            }
            thresholdConfSize = confThresholds.size();

            interfacePath = &path.str;
            break;
        }
        if (interfacePath == nullptr)
        {
            // To avoid this error message, add your export map entry,
            // from Entity Manager, to sensorTypes at the top of this file.
            lg2::error("failed to find match for '{NAME}'", "NAME", deviceName);
            continue;
        }

        auto findI2CDev = devices.find(*interfacePath);

        std::shared_ptr<I2CDevice> i2cDev;
        if (findI2CDev != devices.end())
        {
            if (activateOnly && !findI2CDev->second.second)
            {
                continue;
            }
            i2cDev = findI2CDev->second.first;
        }

        auto findPSUName = baseConfig->find("Name");
        if (findPSUName == baseConfig->end())
        {
            lg2::error("could not determine configuration name for '{NAME}'",
                       "NAME", deviceName);
            continue;
        }
        const std::string* psuName =
            std::get_if<std::string>(&(findPSUName->second));
        if (psuName == nullptr)
        {
            lg2::error("Cannot find psu name, invalid configuration");
            continue;
        }

        auto findCPU = baseConfig->find("CPURequired");
        if (findCPU != baseConfig->end())
        {
            size_t index = std::visit(VariantToIntVisitor(), findCPU->second);
            auto presenceFind = cpuPresence.find(index);
            if (presenceFind == cpuPresence.end() || !presenceFind->second)
            {
                continue;
            }
        }

        // on rescans, only update sensors we were signaled by
        if (!firstScan)
        {
            std::string psuNameStr = "/" + escapeName(*psuName);
            auto it =
                std::find_if(sensorsChanged->begin(), sensorsChanged->end(),
                             [psuNameStr](std::string& s) {
                                 return s.ends_with(psuNameStr);
                             });

            if (it == sensorsChanged->end())
            {
                continue;
            }
            sensorsChanged->erase(it);
        }
        checkEvent(directory.string(), eventMatch, eventPathList);
        checkGroupEvent(directory.string(), groupEventPathList);

        PowerState readState = getPowerState(*baseConfig);

        /* Check if there are more sensors in the same interface */
        int i = 1;
        std::vector<std::string> psuNames;
        do
        {
            // Individual string fields: Name, Name1, Name2, Name3, ...
            psuNames.push_back(
                escapeName(std::get<std::string>(findPSUName->second)));
            findPSUName = baseConfig->find("Name" + std::to_string(i++));
        } while (findPSUName != baseConfig->end());

        std::vector<std::filesystem::path> sensorPaths;
        if (!findFiles(directory, devParamMap[devType].matchRegEx, sensorPaths,
                       0))
        {
            lg2::error("No PSU non-label sensor in PSU");
            continue;
        }

        /* read max value in sysfs for in, curr, power, temp, ... */
        if (!findFiles(directory, R"(\w\d+_max$)", sensorPaths, 0))
        {
            lg2::debug("No max name in PSU");
        }

        float pollRate = getPollRate(*baseConfig, PSUSensor::defaultSensorPoll);

        /* Find array of labels to be exposed if it is defined in config */
        std::vector<std::string> findLabels;
        auto findLabelObj = baseConfig->find("Labels");
        if (findLabelObj != baseConfig->end())
        {
            findLabels =
                std::get<std::vector<std::string>>(findLabelObj->second);
        }

        std::regex sensorNameRegEx(devParamMap[devType].nameRegEx);
        std::smatch matches;

        for (const auto& sensorPath : sensorPaths)
        {
            bool maxLabel = false;
            std::string labelHead;
            std::string sensorPathStr = sensorPath.string();
            std::string sensorNameStr = sensorPath.filename();
            std::string sensorNameSubStr;
            if (std::regex_search(sensorNameStr, matches, sensorNameRegEx))
            {
                // hwmon *_input filename without number:
                // in, curr, power, temp, ...
                // iio in_*_raw filename without number:
                // voltage, temp, pressure, ...
                sensorNameSubStr = matches[devParamMap[devType].matchIndex];
            }
            else
            {
                lg2::error("Could not extract the alpha prefix from '{NAME}'",
                           "NAME", sensorNameStr);
                continue;
            }

            std::string labelPath;

            if (devType == DevTypes::HWMON)
            {
                /* find and differentiate _max and _input to replace "label" */
                size_t pos = sensorPathStr.find('_');
                if (pos != std::string::npos)
                {
                    std::string sensorPathStrMax = sensorPathStr.substr(pos);
                    if (sensorPathStrMax == "_max")
                    {
                        labelPath = boost::replace_all_copy(sensorPathStr,
                                                            "max", "label");
                        maxLabel = true;
                    }
                    else
                    {
                        labelPath = boost::replace_all_copy(sensorPathStr,
                                                            "input", "label");
                        maxLabel = false;
                    }
                }
                else
                {
                    continue;
                }

                std::ifstream labelFile(labelPath);
                if (!labelFile.good())
                {
                    lg2::debug(
                        "Input file '{PATH}' has no corresponding label file",
                        "PATH", sensorPath.string());
                    // hwmon *_input filename with number:
                    // temp1, temp2, temp3, ...
                    labelHead =
                        sensorNameStr.substr(0, sensorNameStr.find('_'));
                }
                else
                {
                    std::string label;
                    std::getline(labelFile, label);
                    labelFile.close();
                    auto findSensor = sensors.find(label);
                    if (findSensor != sensors.end())
                    {
                        continue;
                    }

                    // hwmon corresponding *_label file contents:
                    // vin1, vout1, ...
                    labelHead = label.substr(0, label.find(' '));
                }

                /* append "max" for labelMatch */
                if (maxLabel)
                {
                    labelHead.insert(0, "max");
                }

                // Don't add PWM sensors if it's not in label list
                if (!findLabels.empty())
                {
                    /* Check if this labelHead is enabled in config file */
                    if (std::find(findLabels.begin(), findLabels.end(),
                                  labelHead) == findLabels.end())
                    {
                        lg2::debug("could not find {LABEL} in the Labels list",
                                   "LABEL", labelHead);
                        continue;
                    }
                }
                checkPWMSensor(sensorPath, labelHead, *interfacePath,
                               dbusConnection, objectServer, psuNames[0]);
            }
            else if (devType == DevTypes::IIO)
            {
                auto findIIOHyphen = sensorNameStr.find_last_of('_');
                labelHead = sensorNameStr.substr(0, findIIOHyphen);
            }

            lg2::debug("Sensor type: {NAME}, label: {LABEL}", "NAME",
                       sensorNameSubStr, "LABEL", labelHead);

            if (!findLabels.empty())
            {
                /* Check if this labelHead is enabled in config file */
                if (std::find(findLabels.begin(), findLabels.end(),
                              labelHead) == findLabels.end())
                {
                    lg2::debug("could not find '{LABEL}' in the Labels list",
                               "LABEL", labelHead);
                    continue;
                }
            }
            auto it = std::find_if(labelHead.begin(), labelHead.end(),
                                   static_cast<int (*)(int)>(std::isdigit));
            std::string_view labelHeadView(
                labelHead.data(), std::distance(labelHead.begin(), it));
            auto findProperty =
                labelMatch.find(static_cast<std::string>(labelHeadView));
            if (findProperty == labelMatch.end())
            {
                lg2::debug(
                    "Could not find matching default property for '{LABEL}'",
                    "LABEL", labelHead);
                continue;
            }

            // Protect the hardcoded labelMatch list from changes,
            // by making a copy and modifying that instead.
            // Avoid bleedthrough of one device's customizations to
            // the next device, as each should be independently customizable.
            PSUProperty psuProperty = findProperty->second;

            // Use label head as prefix for reading from config file,
            // example if temp1: temp1_Name, temp1_Scale, temp1_Min, ...
            std::string keyName = labelHead + "_Name";
            std::string keyScale = labelHead + "_Scale";
            std::string keyMin = labelHead + "_Min";
            std::string keyMax = labelHead + "_Max";
            std::string keyOffset = labelHead + "_Offset";
            std::string keyPowerState = labelHead + "_PowerState";

            bool customizedName = false;
            auto findCustomName = baseConfig->find(keyName);
            if (findCustomName != baseConfig->end())
            {
                try
                {
                    psuProperty.labelTypeName = std::visit(
                        VariantToStringVisitor(), findCustomName->second);
                }
                catch (const std::invalid_argument&)
                {
                    lg2::error("Unable to parse '{NAME}'", "NAME", keyName);
                    continue;
                }

                // All strings are valid, including empty string
                customizedName = true;
            }

            bool customizedScale = false;
            auto findCustomScale = baseConfig->find(keyScale);
            if (findCustomScale != baseConfig->end())
            {
                try
                {
                    psuProperty.sensorScaleFactor = std::visit(
                        VariantToUnsignedIntVisitor(), findCustomScale->second);
                }
                catch (const std::invalid_argument&)
                {
                    lg2::error("Unable to parse '{SCALE}'", "SCALE", keyScale);
                    continue;
                }

                // Avoid later division by zero
                if (psuProperty.sensorScaleFactor > 0)
                {
                    customizedScale = true;
                }
                else
                {
                    lg2::error("Unable to accept '{SCALE}'", "SCALE", keyScale);
                    continue;
                }
            }

            auto findCustomMin = baseConfig->find(keyMin);
            if (findCustomMin != baseConfig->end())
            {
                try
                {
                    psuProperty.minReading = std::visit(
                        VariantToDoubleVisitor(), findCustomMin->second);
                }
                catch (const std::invalid_argument&)
                {
                    lg2::error("Unable to parse '{MIN}'", "MIN", keyMin);
                    continue;
                }
            }

            auto findCustomMax = baseConfig->find(keyMax);
            if (findCustomMax != baseConfig->end())
            {
                try
                {
                    psuProperty.maxReading = std::visit(
                        VariantToDoubleVisitor(), findCustomMax->second);
                }
                catch (const std::invalid_argument&)
                {
                    lg2::error("Unable to parse '{MAX}'", "MAX", keyMax);
                    continue;
                }
            }

            auto findCustomOffset = baseConfig->find(keyOffset);
            if (findCustomOffset != baseConfig->end())
            {
                try
                {
                    psuProperty.sensorOffset = std::visit(
                        VariantToDoubleVisitor(), findCustomOffset->second);
                }
                catch (const std::invalid_argument&)
                {
                    lg2::error("Unable to parse '{OFFSET}'", "OFFSET",
                               keyOffset);
                    continue;
                }
            }

            // if we find label head power state set ，override the powerstate.
            auto findPowerState = baseConfig->find(keyPowerState);
            if (findPowerState != baseConfig->end())
            {
                std::string powerState = std::visit(VariantToStringVisitor(),
                                                    findPowerState->second);
                setReadState(powerState, readState);
            }
            if (!(psuProperty.minReading < psuProperty.maxReading))
            {
                lg2::error("Min must be less than Max");
                continue;
            }

            // If the sensor name is being customized by config file,
            // then prefix/suffix composition becomes not necessary,
            // and in fact not wanted, because it gets in the way.
            std::string psuNameFromIndex;
            std::string nameIndexStr = "1";
            if (!customizedName)
            {
                /* Find out sensor name index for this label */
                std::regex rgx("[A-Za-z]+([0-9]+)");
                size_t nameIndex{0};
                if (std::regex_search(labelHead, matches, rgx))
                {
                    nameIndexStr = matches[1];
                    nameIndex = std::stoi(nameIndexStr);

                    // Decrement to preserve alignment, because hwmon
                    // human-readable filenames and labels use 1-based
                    // numbering, but the "Name", "Name1", "Name2", etc. naming
                    // convention (the psuNames vector) uses 0-based numbering.
                    if (nameIndex > 0)
                    {
                        --nameIndex;
                    }
                }
                else
                {
                    nameIndex = 0;
                }

                if (psuNames.size() <= nameIndex)
                {
                    lg2::error("Could not pair '{LABEL}' with a Name field",
                               "LABEL", labelHead);
                    continue;
                }

                psuNameFromIndex = psuNames[nameIndex];

                lg2::debug("'{LABEL}' paired with '{NAME}' at index '{INDEX}'",
                           "LABEL", labelHead, "NAME", psuNameFromIndex,
                           "INDEX", nameIndex);
            }

            if (devType == DevTypes::HWMON)
            {
                checkEventLimits(sensorPathStr, limitEventMatch, eventPathList);
            }

            // Similarly, if sensor scaling factor is being customized,
            // then the below power-of-10 constraint becomes unnecessary,
            // as config should be able to specify an arbitrary divisor.
            unsigned int factor = psuProperty.sensorScaleFactor;
            if (!customizedScale)
            {
                // Preserve existing usage of hardcoded labelMatch table below
                factor = std::pow(10.0, factor);

                /* Change first char of substring to uppercase */
                char firstChar =
                    static_cast<char>(std::toupper(sensorNameSubStr[0]));
                std::string strScaleFactor =
                    firstChar + sensorNameSubStr.substr(1) + "ScaleFactor";

                // Preserve existing configs by accepting earlier syntax,
                // example CurrScaleFactor, PowerScaleFactor, ...
                auto findScaleFactor = baseConfig->find(strScaleFactor);
                if (findScaleFactor != baseConfig->end())
                {
                    factor = std::visit(VariantToIntVisitor(),
                                        findScaleFactor->second);
                }

                lg2::debug(
                    "Sensor scaling factor '{FACTOR}' string '{SCALE_FACTOR}'",
                    "FACTOR", factor, "SCALE_FACTOR", strScaleFactor);
            }

            std::vector<thresholds::Threshold> sensorThresholds;
            if (!parseThresholdsFromConfig(*sensorData, sensorThresholds,
                                           &labelHead, nullptr, &sensorPathStr))
            {
                lg2::error("error populating thresholds for '{NAME}'", "NAME",
                           sensorNameSubStr);
            }

            auto findSensorUnit = sensorTable.find(sensorNameSubStr);
            if (findSensorUnit == sensorTable.end())
            {
                lg2::error("'{NAME}' is not a recognized sensor type", "NAME",
                           sensorNameSubStr);
                continue;
            }

            lg2::debug("Sensor properties - Name: {NAME}, Scale: {SCALE}, "
                       "Min: {MIN}, Max: {MAX}, Offset: {OFFSET}",
                       "NAME", psuProperty.labelTypeName, "SCALE",
                       psuProperty.sensorScaleFactor, "MIN",
                       psuProperty.minReading, "MAX", psuProperty.maxReading,
                       "OFFSET", psuProperty.sensorOffset);

            std::string sensorName = psuProperty.labelTypeName;
            if (customizedName)
            {
                if (sensorName.empty())
                {
                    // Allow selective disabling of an individual sensor,
                    // by customizing its name to an empty string.
                    lg2::error("Sensor disabled, empty string");
                    continue;
                }
            }
            else
            {
                // Sensor name not customized, do prefix/suffix composition,
                // preserving default behavior by using psuNameFromIndex.
                sensorName = psuNameFromIndex + " " + psuProperty.labelTypeName;

                // The labelTypeName of a fan can be:
                // "Fan Speed 1", "Fan Speed 2", "Fan Speed 3" ...
                if (labelHead == "fan" + nameIndexStr)
                {
                    sensorName += nameIndexStr;
                }
            }

            lg2::debug("Sensor name: {NAME}, path: {PATH}, type: {TYPE}",
                       "NAME", sensorName, "PATH", sensorPathStr, "TYPE",
                       sensorType);
            // destruct existing one first if already created

            auto& sensor = sensors[sensorName];
            if (!activateOnly)
            {
                sensor = nullptr;
            }

            if (sensor != nullptr)
            {
                sensor->activate(sensorPathStr, i2cDev);
            }
            else
            {
                sensors[sensorName] = std::make_shared<PSUSensor>(
                    sensorPathStr, sensorType, objectServer, dbusConnection, io,
                    sensorName, std::move(sensorThresholds), *interfacePath,
                    readState, findSensorUnit->second, factor,
                    psuProperty.maxReading, psuProperty.minReading,
                    psuProperty.sensorOffset, labelHead, thresholdConfSize,
                    pollRate, i2cDev);
                sensors[sensorName]->setupRead();
                ++numCreated;
                lg2::debug("Created '{NUM}' sensors so far", "NUM", numCreated);
            }
        }

        if (devType == DevTypes::HWMON)
        {
            // OperationalStatus event
            combineEvents[*psuName + "OperationalStatus"] = nullptr;
            combineEvents[*psuName + "OperationalStatus"] =
                std::make_unique<PSUCombineEvent>(
                    objectServer, dbusConnection, io, *psuName, readState,
                    eventPathList, groupEventPathList, "OperationalStatus",
                    pollRate);
        }
    }

    lg2::debug("Created total of '{NUM}' sensors", "NUM", numCreated);
}

static void getPresentCpus(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    static const int depth = 2;
    static const int numKeys = 1;
    GetSubTreeType cpuSubTree;

    try
    {
        auto getItems = dbusConnection->new_method_call(
            mapper::busName, mapper::path, mapper::interface, mapper::subtree);
        getItems.append(cpuInventoryPath, static_cast<int32_t>(depth),
                        std::array<const char*, numKeys>{
                            "xyz.openbmc_project.Inventory.Item"});
        auto getItemsResp = dbusConnection->call(getItems);
        getItemsResp.read(cpuSubTree);
    }
    catch (sdbusplus::exception_t& e)
    {
        lg2::error("error getting inventory item subtree: '{ERR}'", "ERR", e);
        return;
    }

    for (const auto& [path, objDict] : cpuSubTree)
    {
        auto obj = sdbusplus::message::object_path(path).filename();
        boost::to_lower(obj);

        if (!obj.starts_with("cpu") || objDict.empty())
        {
            continue;
        }
        const std::string& owner = objDict.begin()->first;

        std::variant<bool> respValue;
        try
        {
            auto getPresence = dbusConnection->new_method_call(
                owner.c_str(), path.c_str(), "org.freedesktop.DBus.Properties",
                "Get");
            getPresence.append("xyz.openbmc_project.Inventory.Item", "Present");
            auto resp = dbusConnection->call(getPresence);
            resp.read(respValue);
        }
        catch (sdbusplus::exception_t& e)
        {
            lg2::error("Error in getting CPU presence: '{ERR}'", "ERR", e);
            continue;
        }

        auto* present = std::get_if<bool>(&respValue);
        if (present != nullptr && *present)
        {
            int cpuIndex = 0;
            try
            {
                cpuIndex = std::stoi(obj.substr(obj.size() - 1));
            }
            catch (const std::exception& e)
            {
                lg2::error("Error converting CPU index: '{ERR}'", "ERR", e);
                continue;
            }
            cpuPresence[cpuIndex] = *present;
        }
    }
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    bool activateOnly)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection, [&io, &objectServer, &dbusConnection, sensorsChanged,
                         activateOnly](const ManagedObjectType& sensorConfigs) {
            createSensorsCallback(io, objectServer, dbusConnection,
                                  sensorConfigs, sensorsChanged, activateOnly);
        });
    std::vector<std::string> types;
    types.reserve(sensorTypes.size());
    for (const auto& [type, dt] : sensorTypes)
    {
        types.push_back(type);
    }
    getter->getConfiguration(types);
}

void propertyInitialize()
{
    sensorTable = {{"power", sensor_paths::unitWatts},
                   {"curr", sensor_paths::unitAmperes},
                   {"temp", sensor_paths::unitDegreesC},
                   {"in", sensor_paths::unitVolts},
                   {"voltage", sensor_paths::unitVolts},
                   {"fan", sensor_paths::unitRPMs}};

    labelMatch = {
        {"pin", PSUProperty("Input Power", 3000, 0, 6, 0)},
        {"pout", PSUProperty("Output Power", 3000, 0, 6, 0)},
        {"power", PSUProperty("Output Power", 3000, 0, 6, 0)},
        {"maxpin", PSUProperty("Max Input Power", 3000, 0, 6, 0)},
        {"vin", PSUProperty("Input Voltage", 300, 0, 3, 0)},
        {"maxvin", PSUProperty("Max Input Voltage", 300, 0, 3, 0)},
        {"in_voltage", PSUProperty("Output Voltage", 255, 0, 3, 0)},
        {"voltage", PSUProperty("Output Voltage", 255, 0, 3, 0)},
        {"vout", PSUProperty("Output Voltage", 255, 0, 3, 0)},
        {"vmon", PSUProperty("Auxiliary Input Voltage", 255, 0, 3, 0)},
        {"in", PSUProperty("Output Voltage", 255, 0, 3, 0)},
        {"iin", PSUProperty("Input Current", 20, 0, 3, 0)},
        {"iout", PSUProperty("Output Current", 255, 0, 3, 0)},
        {"curr", PSUProperty("Output Current", 255, 0, 3, 0)},
        {"maxiout", PSUProperty("Max Output Current", 255, 0, 3, 0)},
        {"temp", PSUProperty("Temperature", 127, -128, 3, 0)},
        {"maxtemp", PSUProperty("Max Temperature", 127, -128, 3, 0)},
        {"fan", PSUProperty("Fan Speed ", 30000, 0, 0, 0)}};

    limitEventMatch = {{"PredictiveFailure", {"max_alarm", "min_alarm"}},
                       {"Failure", {"crit_alarm", "lcrit_alarm"}}};

    eventMatch = {{"PredictiveFailure", {"power1_alarm"}},
                  {"Failure", {"in2_alarm"}},
                  {"ACLost", {"in1_beep"}},
                  {"ConfigureError", {"in1_fault"}}};

    devParamMap = {
        {DevTypes::HWMON, {1, R"(\w\d+_input$)", "([A-Za-z]+)[0-9]*_"}},
        {DevTypes::IIO,
         {2, R"(\w+_(raw|input)$)", "^(in|out)_([A-Za-z]+)[0-9]*_"}}};
}

static void powerStateChanged(
    PowerState type, bool newState,
    boost::container::flat_map<std::string, std::shared_ptr<PSUSensor>>&
        sensors,
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (newState)
    {
        createSensors(io, objectServer, dbusConnection, nullptr, true);
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
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);

    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    objectServer.add_manager("/xyz/openbmc_project/control");
    systemBus->request_name("xyz.openbmc_project.PSUSensor");
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    propertyInitialize();

    auto powerCallBack = [&io, &objectServer,
                          &systemBus](PowerState type, bool state) {
        powerStateChanged(type, state, sensors, io, objectServer, systemBus);
    };

    setupPowerMatchCallback(systemBus, powerCallBack);

    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, systemBus, nullptr, false);
    });
    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t& message) {
            sensorsChanged->insert(message.get_path());
            filterTimer.expires_after(std::chrono::seconds(3));
            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return;
                }
                if (ec)
                {
                    lg2::error("timer error");
                }
                createSensors(io, objectServer, systemBus, sensorsChanged,
                              false);
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
                return;
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
            if (findPresence == values.end())
            {
                return;
            }
            try
            {
                cpuPresence[index] = std::get<bool>(findPresence->second);
            }
            catch (const std::bad_variant_access& err)
            {
                return;
            }

            if (!cpuPresence[index])
            {
                return;
            }
            cpuFilterTimer.expires_after(std::chrono::seconds(1));
            cpuFilterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return;
                }
                if (ec)
                {
                    lg2::error("timer error");
                    return;
                }
                createSensors(io, objectServer, systemBus, nullptr, false);
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

    getPresentCpus(systemBus);

    setupManufacturingModeMatch(*systemBus);
    io.run();
}
