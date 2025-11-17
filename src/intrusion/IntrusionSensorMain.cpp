/*
// Copyright (c) 2018 Intel Corporation
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

#include "ChassisIntrusionSensor.hpp"
#include "Utils.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <array>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

static constexpr const char* sensorType = "ChassisIntrusionSensor";
static constexpr const char* nicType = "NIC";
static constexpr auto nicTypes{std::to_array<const char*>({nicType})};

static const std::map<std::string, std::string> compatibleHwmonNames = {
    {"Aspeed2600_Hwmon", "intrusion0_alarm"}
    // Add compatible strings here for new hwmon intrusion detection
    // drivers that have different hwmon names but would also like to
    // use the available Hwmon class.
};

static void createSensorsFromConfig(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objServer,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    std::shared_ptr<ChassisIntrusionSensor>& pSensor)
{
    // find matched configuration according to sensor type
    ManagedObjectType sensorConfigurations;
    bool useCache = false;

    if (!getSensorConfiguration(sensorType, dbusConnection,
                                sensorConfigurations, useCache))
    {
        lg2::error("error communicating to entity manager");
        return;
    }

    const SensorData* sensorData = nullptr;
    const std::pair<std::string, SensorBaseConfigMap>* baseConfiguration =
        nullptr;

    for (const auto& [path, cfgData] : sensorConfigurations)
    {
        baseConfiguration = nullptr;
        sensorData = &cfgData;

        // match sensor type
        auto sensorBase = sensorData->find(configInterfaceName(sensorType));
        if (sensorBase == sensorData->end())
        {
            lg2::error("error finding base configuration");
            continue;
        }

        baseConfiguration = &(*sensorBase);

        // Rearm defaults to "Automatic" mode
        bool autoRearm = true;
        auto findRearm = baseConfiguration->second.find("Rearm");
        if (findRearm != baseConfiguration->second.end())
        {
            std::string rearmStr = std::get<std::string>(findRearm->second);
            if (rearmStr != "Automatic" && rearmStr != "Manual")
            {
                lg2::error("Wrong input for Rearm parameter");
                continue;
            }
            autoRearm = (rearmStr == "Automatic");
        }

        // judge class, "Gpio", "Hwmon" or "I2C"
        auto findClass = baseConfiguration->second.find("Class");
        if (findClass != baseConfiguration->second.end())
        {
            auto classString = std::get<std::string>(findClass->second);
            if (classString == "Gpio")
            {
                auto findGpioPolarity =
                    baseConfiguration->second.find("GpioPolarity");

                if (findGpioPolarity == baseConfiguration->second.end())
                {
                    lg2::error("error finding gpio polarity in configuration");
                    continue;
                }

                try
                {
                    bool gpioInverted =
                        (std::get<std::string>(findGpioPolarity->second) ==
                         "Low");
                    pSensor = std::make_shared<ChassisIntrusionGpioSensor>(
                        autoRearm, io, objServer, gpioInverted);
                    pSensor->start();
                    lg2::debug(
                        "find chassis intrusion sensor polarity inverted flag is '{GPIO_INVERTED}'",
                        "GPIO_INVERTED", gpioInverted);
                    return;
                }
                catch (const std::bad_variant_access& e)
                {
                    lg2::error("invalid value for gpio info in config.");
                    continue;
                }
                catch (const std::exception& e)
                {
                    lg2::error(
                        "error creating chassis intrusion gpio sensor: '{ERROR}'",
                        "ERROR", e);
                    continue;
                }
            }
            // If class string contains Hwmon string
            else if (classString.find("Hwmon") != std::string::npos)
            {
                std::string hwmonName;
                std::map<std::string, std::string>::const_iterator
                    compatIterator = compatibleHwmonNames.find(classString);

                if (compatIterator == compatibleHwmonNames.end())
                {
                    lg2::error("Hwmon Class string is not supported");
                    continue;
                }

                hwmonName = compatIterator->second;

                try
                {
                    pSensor = std::make_shared<ChassisIntrusionHwmonSensor>(
                        autoRearm, io, objServer, hwmonName);
                    pSensor->start();
                    return;
                }
                catch (const std::exception& e)
                {
                    lg2::error(
                        "error creating chassis intrusion hwmon sensor: '{ERROR}'",
                        "ERROR", e);
                    continue;
                }
            }
            else
            {
                auto findBus = baseConfiguration->second.find("Bus");
                auto findAddress = baseConfiguration->second.find("Address");
                if (findBus == baseConfiguration->second.end() ||
                    findAddress == baseConfiguration->second.end())
                {
                    lg2::error("error finding bus or address in configuration");
                    continue;
                }
                try
                {
                    int busId = std::get<uint64_t>(findBus->second);
                    int slaveAddr = std::get<uint64_t>(findAddress->second);
                    pSensor = std::make_shared<ChassisIntrusionPchSensor>(
                        autoRearm, io, objServer, busId, slaveAddr);
                    pSensor->start();
                    lg2::debug(
                        "find matched bus '{BUS}', matched slave addr '{ADDR}'",
                        "BUS", busId, "ADDR", slaveAddr);
                    return;
                }
                catch (const std::bad_variant_access& e)
                {
                    lg2::error("invalid value for bus or address in config.");
                    continue;
                }
                catch (const std::exception& e)
                {
                    lg2::error(
                        "error creating chassis intrusion pch sensor: '{ERROR}'",
                        "ERROR", e);
                    continue;
                }
            }
        }
    }

    lg2::error("Can't find matched I2C, GPIO or Hwmon configuration");

    // Make sure nothing runs when there's failure in configuration for the
    // sensor after rescan
    if (pSensor)
    {
        lg2::error("Reset the occupied sensor pointer");
        pSensor = nullptr;
    }
}

boost::container::flat_map<int, bool> lanStatusMap;
boost::container::flat_map<int, std::string> lanInfoMap;
boost::container::flat_map<std::string, int> pathSuffixMap;

static void getNicNameInfo(
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection, [](const ManagedObjectType& sensorConfigurations) {
            // Get NIC name and save to map
            lanInfoMap.clear();
            for (const auto& [path, cfgData] : sensorConfigurations)
            {
                const std::pair<std::string, SensorBaseConfigMap>*
                    baseConfiguration = nullptr;

                // find base configuration
                auto sensorBase = cfgData.find(configInterfaceName(nicType));
                if (sensorBase == cfgData.end())
                {
                    continue;
                }
                baseConfiguration = &(*sensorBase);

                auto findEthIndex = baseConfiguration->second.find("EthIndex");
                auto findName = baseConfiguration->second.find("Name");

                if (findEthIndex != baseConfiguration->second.end() &&
                    findName != baseConfiguration->second.end())
                {
                    const auto* pEthIndex =
                        std::get_if<uint64_t>(&findEthIndex->second);
                    const auto* pName =
                        std::get_if<std::string>(&findName->second);
                    if (pEthIndex != nullptr && pName != nullptr)
                    {
                        lanInfoMap[*pEthIndex] = *pName;
                        lg2::debug("find name of eth{ETH_INDEX} is '{NAME}'",
                                   "ETH_INDEX", *pEthIndex, "NAME", *pName);
                    }
                }
            }

            if (lanInfoMap.empty())
            {
                lg2::error("can't find matched NIC name.");
            }
        });

    getter->getConfiguration(
        std::vector<std::string>{nicTypes.begin(), nicTypes.end()});
}

static void processLanStatusChange(sdbusplus::message_t& message)
{
    const std::string& pathName = message.get_path();
    std::string interfaceName;
    SensorBaseConfigMap properties;
    message.read(interfaceName, properties);

    auto findStateProperty = properties.find("OperationalState");
    if (findStateProperty == properties.end())
    {
        return;
    }
    std::string* pState =
        std::get_if<std::string>(&(findStateProperty->second));
    if (pState == nullptr)
    {
        lg2::error("invalid OperationalState");
        return;
    }

    bool newLanConnected = (*pState == "routable" || *pState == "carrier" ||
                            *pState == "degraded");

    // get ethNum from path. /org/freedesktop/network1/link/_32 for eth0
    size_t pos = pathName.find("/_");
    if (pos == std::string::npos || pathName.length() <= pos + 2)
    {
        lg2::error("unexpected path name '{NAME}'", "NAME", pathName);
        return;
    }
    std::string suffixStr = pathName.substr(pos + 2);

    auto findEthNum = pathSuffixMap.find(suffixStr);
    if (findEthNum == pathSuffixMap.end())
    {
        lg2::error("unexpected eth for suffixStr '{SUFFIX}'", "SUFFIX",
                   suffixStr);
        return;
    }
    int ethNum = findEthNum->second;

    // get lan status from map
    auto findLanStatus = lanStatusMap.find(ethNum);
    if (findLanStatus == lanStatusMap.end())
    {
        lg2::error("unexpected eth{ETH_INDEX} is lanStatusMap", "ETH_INDEX",
                   ethNum);
        return;
    }
    bool oldLanConnected = findLanStatus->second;

    // get lan info from map
    std::string lanInfo;
    if (!lanInfoMap.empty())
    {
        auto findLanInfo = lanInfoMap.find(ethNum);
        if (findLanInfo == lanInfoMap.end())
        {
            lg2::error("unexpected eth{ETH_INDEX} is lanInfoMap", "ETH_INDEX",
                       ethNum);
        }
        else
        {
            lanInfo = "(" + findLanInfo->second + ")";
        }
    }

    lg2::debug(
        "ethNum = {ETH_INDEX}, state = {LAN_STATUS}, oldLanConnected = {OLD_LAN_CONNECTED}, "
        "newLanConnected = {NEW_LAN_CONNECTED}",
        "ETH_INDEX", ethNum, "LAN_STATUS", *pState, "OLD_LAN_CONNECTED",
        (oldLanConnected ? "true" : "false"), "NEW_LAN_CONNECTED",
        (newLanConnected ? "true" : "false"));

    if (oldLanConnected != newLanConnected)
    {
        std::string strEthNum = "eth" + std::to_string(ethNum) + lanInfo;
        const auto* strState = newLanConnected ? "connected" : "lost";
        const auto* strMsgId =
            newLanConnected ? "OpenBMC.0.1.LanRegained" : "OpenBMC.0.1.LanLost";

        lg2::info("'{ETH_INFO}' LAN leash '{LAN_STATUS}'", "ETH_INFO",
                  strEthNum, "LAN_STATUS", strState, "REDFISH_MESSAGE_ID",
                  strMsgId, "REDFISH_MESSAGE_ARGS", strEthNum);

        lanStatusMap[ethNum] = newLanConnected;
    }
}

/** @brief Initialize the lan status.
 *
 * @return true on success and false on failure
 */
static bool initializeLanStatus(
    const std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    // init lan port name from configuration
    getNicNameInfo(conn);

    // get eth info from sysfs
    std::vector<std::filesystem::path> files;
    if (!findFiles(std::filesystem::path("/sys/class/net/"),
                   R"(eth\d+/ifindex)", files))
    {
        lg2::error("No eth in system");
        return false;
    }

    // iterate through all found eth files, and save ifindex
    for (const std::filesystem::path& fileName : files)
    {
        lg2::debug("Reading '{NAME}'", "NAME", fileName);
        std::ifstream sysFile(fileName);
        if (!sysFile.good())
        {
            lg2::error("Failure reading '{NAME}'", "NAME", fileName);
            continue;
        }
        std::string line;
        getline(sysFile, line);
        const uint8_t ifindex = std::stoi(line);
        // pathSuffix is ASCII of ifindex
        const std::string& pathSuffix = std::to_string(ifindex + 30);

        // extract ethNum
        const std::string& fileStr = fileName.string();
        const int pos = fileStr.find("eth");
        const std::string& ethNumStr = fileStr.substr(pos + 3);
        int ethNum = 0;
        std::from_chars_result r = std::from_chars(
            ethNumStr.data(), ethNumStr.data() + ethNumStr.size(), ethNum);
        if (r.ec != std::errc())
        {
            lg2::error("invalid ethNum string: '{ETH_INDEX}'", "ETH_INDEX",
                       ethNumStr);
            continue;
        }

        // save pathSuffix
        pathSuffixMap[pathSuffix] = ethNum;
        lg2::debug(
            "ethNum = {ETH_INDEX}, ifindex = {LINE}, pathSuffix = {PATH}",
            "ETH_INDEX", ethNum, "LINE", line, "PATH", pathSuffix);

        // init lan connected status from networkd
        conn->async_method_call(
            [ethNum](boost::system::error_code ec,
                     const std::variant<std::string>& property) {
                lanStatusMap[ethNum] = false;
                if (ec)
                {
                    lg2::error("Error reading init status of eth{ETH_INDEX}",
                               "ETH_INDEX", ethNum);
                    return;
                }
                const std::string* pState = std::get_if<std::string>(&property);
                if (pState == nullptr)
                {
                    lg2::error("Unable to read lan status value");
                    return;
                }
                bool isLanConnected =
                    (*pState == "routable" || *pState == "carrier" ||
                     *pState == "degraded");
                lg2::debug("ethNum = {ETH_INDEX}, init LAN status = {STATUS}",
                           "ETH_INDEX", ethNum, "STATUS",
                           (isLanConnected ? "true" : "false"));
                lanStatusMap[ethNum] = isLanConnected;
            },
            "org.freedesktop.network1",
            "/org/freedesktop/network1/link/_" + pathSuffix,
            "org.freedesktop.DBus.Properties", "Get",
            "org.freedesktop.network1.Link", "OperationalState");
    }
    return true;
}

int main()
{
    std::shared_ptr<ChassisIntrusionSensor> intrusionSensor;

    // setup connection to dbus
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);

    // setup object server, define interface
    systemBus->request_name("xyz.openbmc_project.IntrusionSensor");

    sdbusplus::asio::object_server objServer(systemBus, true);

    objServer.add_manager("/xyz/openbmc_project/Chassis");

    createSensorsFromConfig(io, objServer, systemBus, intrusionSensor);

    // callback to handle configuration change
    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t&) {
            // this implicitly cancels the timer
            filterTimer.expires_after(std::chrono::seconds(1));
            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    // timer was cancelled
                    return;
                }
                lg2::info("rescan due to configuration change");
                createSensorsFromConfig(io, objServer, systemBus,
                                        intrusionSensor);
            });
        };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(
            *systemBus, std::to_array<const char*>({sensorType}), eventHandler);

    if (initializeLanStatus(systemBus))
    {
        // add match to monitor lan status change
        sdbusplus::bus::match_t lanStatusMatch(
            static_cast<sdbusplus::bus_t&>(*systemBus),
            "type='signal', member='PropertiesChanged',"
            "arg0namespace='org.freedesktop.network1.Link'",
            [](sdbusplus::message_t& msg) { processLanStatusChange(msg); });

        // add match to monitor entity manager signal about nic name config
        // change
        sdbusplus::bus::match_t lanConfigMatch(
            static_cast<sdbusplus::bus_t&>(*systemBus),
            "type='signal', member='PropertiesChanged',path_namespace='" +
                std::string(inventoryPath) + "',arg0namespace='" +
                configInterfaceName(nicType) + "'",
            [&systemBus](sdbusplus::message_t&) { getNicNameInfo(systemBus); });
    }

    io.run();

    return 0;
}
