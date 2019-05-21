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

#include <PSUEvent.hpp>
#include <PSUSensor.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_set.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

static constexpr std::array<const char*, 1> sensorTypes = {
    "xyz.openbmc_project.Configuration.pmbus"};

namespace fs = std::filesystem;

static boost::container::flat_map<std::string, std::unique_ptr<PSUSensor>>
    sensors;
static boost::container::flat_map<std::string, std::unique_ptr<PSUCombineEvent>>
    combineEvents;
static boost::container::flat_map<std::string, std::unique_ptr<PwmSensor>>
    pwmSensors;
static boost::container::flat_map<std::string, std::string> sensorTable;
static boost::container::flat_map<std::string, PSUProperty> labelMatch;
static boost::container::flat_map<std::string, std::string> pwmTable;
static boost::container::flat_map<std::string, std::vector<std::string>>
    eventMatch;
static boost::container::flat_map<std::string, std::vector<std::string>>
    limitEventMatch;

// Function CheckEvent will check each attribute from eventMatch table in the
// sysfs. If the attributes exists in sysfs, then store the complete path
// of the attribute into eventPathList.
void checkEvent(
    const std::string& directory,
    const boost::container::flat_map<std::string, std::vector<std::string>>&
        eventMatch,
    boost::container::flat_map<std::string, std::vector<std::string>>&
        eventPathList)
{
    for (const auto& match : eventMatch)
    {
        const std::vector<std::string>& eventAttrs = match.second;
        const std::string& eventName = match.first;
        for (const auto& eventAttr : eventAttrs)
        {
            auto eventPath = directory + "/" + eventAttr;

            std::ifstream eventFile(eventPath);
            if (!eventFile.good())
            {
                continue;
            }

            eventPathList[eventName].push_back(eventPath);
        }
    }
}

// Function checkEventLimits will check all the psu related xxx_input attributes
// in sysfs to see if xxx_crit_alarm xxx_lcrit_alarm xxx_max_alarm
// xxx_min_alarm exist, then store the existing paths of the alarm attributes
// to eventPathList.
void checkEventLimits(
    const std::string& sensorPathStr,
    const boost::container::flat_map<std::string, std::vector<std::string>>&
        limitEventMatch,
    boost::container::flat_map<std::string, std::vector<std::string>>&
        eventPathList)
{
    for (const auto& limitMatch : limitEventMatch)
    {
        const std::vector<std::string>& limitEventAttrs = limitMatch.second;
        const std::string& eventName = limitMatch.first;
        for (const auto& limitEventAttr : limitEventAttrs)
        {
            auto limitEventPath =
                boost::replace_all_copy(sensorPathStr, "input", limitEventAttr);
            std::ifstream eventFile(limitEventPath);
            if (!eventFile.good())
            {
                continue;
            }
            eventPathList[eventName].push_back(limitEventPath);
        }
    }
}

static void checkPWMSensor(const fs::path& sensorPath, std::string& labelHead,
                           const std::string& interfacePath,
                           sdbusplus::asio::object_server& objectServer,
                           std::string psuName)
{
    for (const auto& pwmName : pwmTable)
    {
        if (pwmName.first != labelHead)
        {
            continue;
        }

        const std::string& sensorPathStr = sensorPath.string();
        const std::string& pwmPathStr =
            boost::replace_all_copy(sensorPathStr, "input", "target");
        std::ifstream pwmFile(pwmPathStr);
        if (!pwmFile.good())
        {
            continue;
        }

        auto findPWMSensor = pwmSensors.find(psuName + labelHead);
        if (findPWMSensor != pwmSensors.end())
        {
            continue;
        }

        pwmSensors[psuName + labelHead] = std::make_unique<PwmSensor>(
            "Pwm_" + psuName + "_" + pwmName.second, pwmPathStr, objectServer,
            interfacePath + "/" + psuName + " " + pwmName.second);
    }
}

void createSensors(boost::asio::io_service& io,
                   sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{

    ManagedObjectType sensorConfigs;
    bool useCache = false;

    // TODO may need only modify the ones that need to be changed.
    sensors.clear();
    for (const char* type : sensorTypes)
    {
        if (!getSensorConfiguration(type, dbusConnection, sensorConfigs,
                                    useCache))
        {
            std::cerr << "error get sensor config from entity manager\n";
            return;
        }
        useCache = true;
    }

    std::vector<fs::path> pmbusPaths;
    if (!findFiles(fs::path("/sys/class/hwmon"), "name", pmbusPaths))
    {
        std::cerr << "No PSU sensors in system\n";
        return;
    }

    boost::container::flat_set<std::string> directories;
    for (const auto& pmbusPath : pmbusPaths)
    {
        boost::container::flat_map<std::string, std::vector<std::string>>
            eventPathList;

        std::ifstream nameFile(pmbusPath);
        if (!nameFile.good())
        {
            std::cerr << "Failure reading " << pmbusPath << "\n";
            continue;
        }

        std::string pmbusName;
        std::getline(nameFile, pmbusName);
        nameFile.close();
        if (pmbusName != "pmbus")
        {
            continue;
        }

        const std::string* psuName;
        auto directory = pmbusPath.parent_path();

        auto ret = directories.insert(directory.string());
        if (!ret.second)
        {
            continue; // check if path has already been searched
        }

        auto device = fs::path(directory / "device");
        std::string deviceName = fs::canonical(device).stem();
        auto findHyphen = deviceName.find("-");
        if (findHyphen == std::string::npos)
        {
            std::cerr << "found bad device" << deviceName << "\n";
            continue;
        }
        std::string busStr = deviceName.substr(0, findHyphen);
        std::string addrStr = deviceName.substr(findHyphen + 1);

        size_t bus = 0;
        size_t addr = 0;

        try
        {
            bus = std::stoi(busStr);
            addr = std::stoi(addrStr, 0, 16);
        }
        catch (std::invalid_argument)
        {
            continue;
        }

        const std::pair<std::string, boost::container::flat_map<
                                         std::string, BasicVariantType>>*
            baseConfig = nullptr;
        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        const char* sensorType = nullptr;

        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigs)
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
                std::cerr
                    << "Canot get bus or address, invalid configuration\n";
                continue;
            }

            if ((*confBus != bus) || (*confAddr != addr))
            {
                continue;
            }

            interfacePath = &(sensor.first.str);
            break;
        }
        if (interfacePath == nullptr)
        {
            std::cerr << "failed to find match for " << deviceName << "\n";
            continue;
        }

        auto findPSUName = baseConfig->second.find("Name");
        if (findPSUName == baseConfig->second.end())
        {
            std::cerr << "could not determine configuration name for "
                      << deviceName << "\n";
            continue;
        }

        if (!(psuName = std::get_if<std::string>(&(findPSUName->second))))
        {
            std::cerr << "Cannot find psu name, invalid configuration\n";
            continue;
        }
        checkEvent(directory.string(), eventMatch, eventPathList);

        std::vector<fs::path> sensorPaths;
        if (!findFiles(fs::path(directory), R"(\w\d+_input$)", sensorPaths, 0))
        {
            std::cerr << "No PSU non-label sensor in PSU\n";
            continue;
        }

        for (const auto& sensorPath : sensorPaths)
        {

            std::string labelHead;
            std::string sensorPathStr = sensorPath.string();
            std::string sensorNameStr = sensorPath.filename();
            std::string sensorNameSubStr =
                sensorNameStr.substr(0, sensorNameStr.find("_") - 1);

            std::string labelPathStr =
                boost::replace_all_copy(sensorNameStr, "input", "label");
            std::vector<fs::path> labelPaths;
            if (!findFiles(fs::path(directory), labelPathStr, labelPaths, 0))
            {
                std::cerr << "No PSU non-label sensor in PSU\n";
                continue;
            }

            if (labelPaths.empty())
            {
                labelHead = sensorNameStr.substr(0, sensorNameStr.find("_"));
            }
            else
            {
                auto labelPath =
                    boost::replace_all_copy(sensorPathStr, "input", "label");
                std::ifstream labelFile(labelPath);
                if (!labelFile.good())
                {
                    std::cerr << "Failure reading " << sensorPath << "\n";
                    continue;
                }
                std::string label;
                std::getline(labelFile, label);
                labelFile.close();

                auto findSensor = sensors.find(label);
                if (findSensor != sensors.end())
                {
                    continue;
                }

                labelHead = label.substr(0, label.find(" "));
            }

            checkPWMSensor(sensorPath, labelHead, *interfacePath, objectServer,
                           std::get<std::string>(findPSUName->second));

            std::vector<thresholds::Threshold> sensorThresholds;

            parseThresholdsFromConfig(*sensorData, sensorThresholds,
                                      &labelHead);

            auto findProperty = labelMatch.find(labelHead);
            if (findProperty == labelMatch.end())
            {
                continue;
            }

            checkEventLimits(sensorPathStr, limitEventMatch, eventPathList);

            unsigned int factor =
                std::pow(10, findProperty->second.sensorScaleFactor);
            if (sensorThresholds.empty())
            {
                if (!parseThresholdsFromAttr(sensorThresholds, sensorPathStr,
                                             factor))
                {
                    std::cerr << "error populating thresholds\n";
                }
            }

            auto findSensorType = sensorTable.find(sensorNameSubStr);
            if (findSensorType == sensorTable.end())
            {
                std::cerr << "Cannot find PSU sensorType\n";
                continue;
            }

            std::string sensorName =
                *psuName + " " + findProperty->second.labelTypeName;

            sensors[sensorName] = std::make_unique<PSUSensor>(
                sensorPathStr, sensorType, objectServer, dbusConnection, io,
                sensorName, std::move(sensorThresholds), *interfacePath,
                findSensorType->second, factor, findProperty->second.maxReading,
                findProperty->second.minReading);
        }

        // OperationalStatus event
        combineEvents[*psuName + "OperationalStatus"] =
            std::make_unique<PSUCombineEvent>(
                objectServer, io, *psuName, eventPathList, "OperationalStatus");
    }
    return;
}

void propertyInitialize(void)
{
    sensorTable = {{"power", "power/"},
                   {"curr", "current/"},
                   {"temp", "temperature/"},
                   {"in", "voltage/"},
                   {"fan", "fan_tach/"}};

    labelMatch = {{"pin", PSUProperty("Input Power", 3000, 0, 6)},
                  {"pout1", PSUProperty("Output Power", 3000, 0, 6)},
                  {"vin", PSUProperty("Input Voltage", 300, 0, 3)},
                  {"iin", PSUProperty("Input Current", 20, 0, 3)},
                  {"iout1", PSUProperty("Output Current", 255, 0, 3)},
                  {"temp1", PSUProperty("Temperature", 127, -128, 3)},
                  {"fan1", PSUProperty("Fan Speed 1", 10000, 0, 0)},
                  {"fan2", PSUProperty("Fan Speed 2", 10000, 0, 0)}};

    pwmTable = {{"fan1", "Fan_1"}, {"fan2", "Fan_2"}};

    limitEventMatch = {{"PredictiveFailure", {"max_alarm", "min_alarm"}},
                       {"Failure", {"crit_alarm", "lcrit_alarm"}}};

    eventMatch = {
        {"PredictiveFailure", {"power1_alarm"}},
        {"Failure", {"in2_alarm"}},
        {"ACLost", {"in1_alarm", "in1_lcrit_alarm"}},
        {"FanFault", {"fan1_alarm", "fan2_alarm", "fan1_fault", "fan2_fault"}}};
}

int main(int argc, char** argv)
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);

    systemBus->request_name("xyz.openbmc_project.PSUSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;

    propertyInitialize();

    io.post([&]() { createSensors(io, objectServer, systemBus); });
    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            filterTimer.expires_from_now(boost::posix_time::seconds(1));
            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return;
                }
                else if (ec)
                {
                    std::cerr << "timer error\n";
                }
                createSensors(io, objectServer, systemBus);
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
    io.run();
}
