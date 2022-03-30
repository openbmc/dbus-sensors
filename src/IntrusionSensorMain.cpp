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

#include <ChassisIntrusionSensor.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/sd_event.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/server.hpp>
#include <sdbusplus/timer.hpp>

#include <array>
#include <charconv>
#include <chrono>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

static constexpr bool debug = false;

static constexpr const char* sensorType =
    "xyz.openbmc_project.Configuration.ChassisIntrusionSensor";
static constexpr const char* nicType = "xyz.openbmc_project.Configuration.NIC";
static constexpr const char* ethernetInterfaceName =
    "xyz.openbmc_project.Network.EthInterface";
static constexpr const char* operationalStatusInterfaceName =
    "xyz.openbmc_project.State.Decorator.OperationalStatus";
static constexpr auto nicTypes{std::to_array<const char*>({nicType})};
static constexpr const char* cableStatusConfiguration =
    "xyz.openbmc_project.Configuration.CableStatus";
static std::shared_ptr<sdbusplus::asio::connection> systemBus;
static constexpr unsigned int entityManagerConfigurationPollIntervalSec = 300;

namespace fs = std::filesystem;

static bool getIntrusionSensorConfig(IntrusionSensorType* pType, int* pBusId,
                                     int* pSlaveAddr, bool* pGpioInverted)
{
    // find matched configuration according to sensor type
    ManagedObjectType sensorConfigurations;
    bool useCache = false;

    if (!getSensorConfiguration(sensorType, systemBus, sensorConfigurations,
                                useCache))
    {
        std::cerr << "error communicating to entity manager\n";
        return false;
    }

    const SensorData* sensorData = nullptr;
    const std::pair<std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
        baseConfiguration = nullptr;

    // Get bus and addr of matched configuration
    for (const std::pair<sdbusplus::message::object_path, SensorData>& sensor :
         sensorConfigurations)
    {
        baseConfiguration = nullptr;
        sensorData = &(sensor.second);

        // match sensor type
        auto sensorBase = sensorData->find(sensorType);
        if (sensorBase == sensorData->end())
        {
            std::cerr << "error finding base configuration \n";
            continue;
        }

        baseConfiguration = &(*sensorBase);

        // judge class, "Gpio" or "I2C"
        auto findClass = baseConfiguration->second.find("Class");
        if (findClass != baseConfiguration->second.end() &&
            std::get<std::string>(findClass->second) == "Gpio")
        {
            *pType = IntrusionSensorType::gpio;
        }
        else
        {
            *pType = IntrusionSensorType::pch;
        }

        // case to find GPIO info
        if (*pType == IntrusionSensorType::gpio)
        {
            auto findGpioPolarity =
                baseConfiguration->second.find("GpioPolarity");

            if (findGpioPolarity == baseConfiguration->second.end())
            {
                std::cerr << "error finding gpio polarity in configuration \n";
                continue;
            }

            try
            {
                *pGpioInverted =
                    (std::get<std::string>(findGpioPolarity->second) == "Low");
            }
            catch (const std::bad_variant_access& e)
            {
                std::cerr << "invalid value for gpio info in config. \n";
                continue;
            }

            if (debug)
            {
                std::cout << "find chassis intrusion sensor polarity inverted "
                             "flag is "
                          << *pGpioInverted << "\n";
            }

            return true;
        }

        // case to find I2C info
        if (*pType == IntrusionSensorType::pch)
        {
            auto findBus = baseConfiguration->second.find("Bus");
            auto findAddress = baseConfiguration->second.find("Address");
            if (findBus == baseConfiguration->second.end() ||
                findAddress == baseConfiguration->second.end())
            {
                std::cerr << "error finding bus or address in configuration \n";
                continue;
            }

            try
            {
                *pBusId = std::get<uint64_t>(findBus->second);
                *pSlaveAddr = std::get<uint64_t>(findAddress->second);
            }
            catch (const std::bad_variant_access& e)
            {
                std::cerr << "invalid value for bus or address in config. \n";
                continue;
            }

            if (debug)
            {
                std::cout << "find matched bus " << *pBusId
                          << ", matched slave addr " << *pSlaveAddr << "\n";
            }
            return true;
        }
    }

    std::cerr << "can't find matched I2C or GPIO configuration for intrusion "
                 "sensor. \n";
    *pBusId = -1;
    *pSlaveAddr = -1;
    return false;
}

static constexpr bool debugLanLeash = false;
boost::container::flat_map<int, bool> lanStatusMap;
boost::container::flat_map<int, std::string> lanInfoMap;
boost::container::flat_map<std::string, int> pathSuffixMap;
boost::container::flat_map<
    std::string, std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>>>
    ethInterfaces;
boost::container::flat_map<std::string,
                           std::shared_ptr<sdbusplus::asio::dbus_interface>>
    assocInterfaces;
static std::unique_ptr<sdbusplus::bus::match::match> lanStatusMatch;
static std::unique_ptr<sdbusplus::bus::match::match> lanConfigMatch;

static void getNicNameInfo()
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        systemBus, [](const ManagedObjectType& sensorConfigurations) {
            // Get NIC name and save to map
            lanInfoMap.clear();
            for (const std::pair<sdbusplus::message::object_path, SensorData>&
                     sensor : sensorConfigurations)
            {
                const std::pair<
                    std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
                    baseConfiguration = nullptr;

                // find base configuration
                auto sensorBase = sensor.second.find(nicType);
                if (sensorBase == sensor.second.end())
                {
                    continue;
                }
                baseConfiguration = &(*sensorBase);

                auto findEthIndex = baseConfiguration->second.find("EthIndex");
                auto findName = baseConfiguration->second.find("Name");

                if (findEthIndex != baseConfiguration->second.end() &&
                    findName != baseConfiguration->second.end())
                {
                    auto* pEthIndex =
                        std::get_if<uint64_t>(&findEthIndex->second);
                    auto* pName = std::get_if<std::string>(&findName->second);
                    if (pEthIndex != nullptr && pName != nullptr)
                    {
                        lanInfoMap[*pEthIndex] = *pName;
                        if (debugLanLeash)
                        {
                            std::cout << "find name of eth" << *pEthIndex
                                      << " is " << *pName << "\n";
                        }
                    }
                }
            }

            if (lanInfoMap.size() == 0)
            {
                std::cerr << "can't find matched NIC name. \n";
            }
        });

    getter->getConfiguration(
        std::vector<std::string>{nicTypes.begin(), nicTypes.end()});
}

static void processLanStatusChange(sdbusplus::message::message& message)
{
    const std::string& pathName = message.get_path();
    std::string interfaceName;
    boost::container::flat_map<std::string, BasicVariantType> properties;
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
        std::cerr << "invalid OperationalState \n";
        return;
    }

    bool newLanConnected = (*pState == "routable" || *pState == "carrier" ||
                            *pState == "degraded");

    // get ethNum from path. /org/freedesktop/network1/link/_32 for eth0
    size_t pos = pathName.find("/_");
    if (pos == std::string::npos || pathName.length() <= pos + 2)
    {
        std::cerr << "unexpected path name " << pathName << "\n";
        return;
    }
    std::string suffixStr = pathName.substr(pos + 2);

    auto findEthNum = pathSuffixMap.find(suffixStr);
    if (findEthNum == pathSuffixMap.end())
    {
        std::cerr << "unexpected eth for suffixStr " << suffixStr << "\n";
        return;
    }
    int ethNum = findEthNum->second;

    // get lan status from map
    auto findLanStatus = lanStatusMap.find(ethNum);
    if (findLanStatus == lanStatusMap.end())
    {
        std::cerr << "unexpected eth " << ethNum << " in lanStatusMap \n";
        return;
    }
    bool oldLanConnected = findLanStatus->second;

    // get lan info from map
    std::string lanInfo = "";
    if (lanInfoMap.size() > 0)
    {
        auto findLanInfo = lanInfoMap.find(ethNum);
        if (findLanInfo == lanInfoMap.end())
        {
            std::cerr << "unexpected eth " << ethNum << " in lanInfoMap \n";
        }
        else
        {
            lanInfo = "(" + findLanInfo->second + ")";
        }
    }

    if (debugLanLeash)
    {
        std::cout << "ethNum = " << ethNum << ", state = " << *pState
                  << ", oldLanConnected = "
                  << (oldLanConnected ? "true" : "false")
                  << ", newLanConnected = "
                  << (newLanConnected ? "true" : "false") << "\n";
    }

    if (oldLanConnected != newLanConnected)
    {
        std::string strEthNum = "eth" + std::to_string(ethNum) + lanInfo;
        auto strState = newLanConnected ? "connected" : "lost";
        auto strMsgId =
            newLanConnected ? "OpenBMC.0.1.LanRegained" : "OpenBMC.0.1.LanLost";

        lg2::info("{ETHDEV} LAN leash {STATE}", "ETHDEV", strEthNum, "STATE",
                  strState, "REDFISH_MESSAGE_ID", strMsgId,
                  "REDFISH_MESSAGE_ARGS", strEthNum);

        lanStatusMap[ethNum] = newLanConnected;
        std::string interfacePath =
            "/xyz/openbmc_project/LanLeash/EthInterface/" +
            std::to_string(ethNum);
        if (ethInterfaces.find(interfacePath) == ethInterfaces.end())
        {
            std::cerr << "unexpected interface path: " << interfacePath << "\n";
            return;
        }
        if (ethInterfaces[interfacePath].size() != 2)
        {
            std::cerr << "unexpected interfaces size: "
                      << ethInterfaces[interfacePath].size()
                      << ", for interface path: " << interfacePath << "\n";
            return;
        }
        const int ethernetInterfaceIndex = 0;
        const int operationalStatusInterfaceIndex = 1;
        // Update dbus interfaces
        ethInterfaces[interfacePath][ethernetInterfaceIndex]->set_property(
            "LinkUp", newLanConnected);

        ethInterfaces[interfacePath][operationalStatusInterfaceIndex]
            ->set_property("Functional", newLanConnected);
    }
}

/** @brief Initialize the lan status.
 *
 * @return true on success and false on failure
 */
static bool initializeLanStatus(sdbusplus::asio::object_server& objServer)
{
    // init lan port name from configuration
    getNicNameInfo();

    // get eth info from sysfs
    std::vector<fs::path> files;
    if (!findFiles(fs::path("/sys/class/net/"), R"(eth\d+/ifindex)", files))
    {
        std::cerr << "No eth in system\n";
        return false;
    }

    // iterate through all found eth files, and save ifindex
    for (const fs::path& fileName : files)
    {
        if (debugLanLeash)
        {
            std::cout << "Reading " << fileName << "\n";
        }
        std::ifstream sysFile(fileName);
        if (!sysFile.good())
        {
            std::cerr << "Failure reading " << fileName << "\n";
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
            std::cerr << "invalid ethNum string: " << ethNumStr << "\n";
            continue;
        }

        // save pathSuffix
        pathSuffixMap[pathSuffix] = ethNum;
        if (debugLanLeash)
        {
            std::cout << "ethNum = " << std::to_string(ethNum)
                      << ", ifindex = " << line
                      << ", pathSuffix = " << pathSuffix << "\n";
        }

        // init lan connected status from networkd
        systemBus->async_method_call(
            [ethNum, &objServer](boost::system::error_code ec,
                                 const std::variant<std::string>& property) {
                lanStatusMap[ethNum] = false;
                if (ec)
                {
                    std::cerr << "Error reading init status of eth" << ethNum
                              << "\n";
                    return;
                }
                const std::string* pState = std::get_if<std::string>(&property);
                if (pState == nullptr)
                {
                    std::cerr << "Unable to read lan status value\n";
                    return;
                }
                bool isLanConnected =
                    (*pState == "routable" || *pState == "carrier" ||
                     *pState == "degraded");
                if (debugLanLeash)
                {
                    std::cout << "ethNum = " << std::to_string(ethNum)
                              << ", init LAN status = "
                              << (isLanConnected ? "true" : "false") << "\n";
                }
                lanStatusMap[ethNum] = isLanConnected;
                std::string interfacePath =
                    "/xyz/openbmc_project/LanLeash/EthInterface/" +
                    std::to_string(ethNum);
                std::shared_ptr<sdbusplus::asio::dbus_interface>
                    ifaceEthInterface = objServer.add_interface(
                        interfacePath, ethernetInterfaceName);
                std::shared_ptr<sdbusplus::asio::dbus_interface>
                    ifaceOperationalStatus = objServer.add_interface(
                        interfacePath, operationalStatusInterfaceName);

                ethInterfaces[interfacePath].push_back(ifaceEthInterface);
                ethInterfaces[interfacePath].push_back(ifaceOperationalStatus);
                ifaceEthInterface->register_property(
                    "InterfaceName", "eth" + std::to_string(ethNum));
                ifaceEthInterface->register_property("LinkUp", isLanConnected);
                ifaceOperationalStatus->register_property("Functional",
                                                          isLanConnected);
                ifaceEthInterface->initialize();
                ifaceOperationalStatus->initialize();
            },
            "org.freedesktop.network1",
            "/org/freedesktop/network1/link/_" + pathSuffix,
            "org.freedesktop.DBus.Properties", "Get",
            "org.freedesktop.network1.Link", "OperationalState");
    }
    return true;
}

/** @brief Generate associations for CableStatus interface.
 *
 */
static void generateAssociations(sdbusplus::asio::object_server& objServer)
{
    std::cout << "Generating associations\n";
    systemBus->async_method_call(
        [&objServer](boost::system::error_code& ec,
                     const ManagedObjectType& managedObj) {
            if (ec)
            {
                std::cerr << "Error calling entity manager \n";
                return;
            }
            for (const auto& pathPair : managedObj)
            {
                for (const auto& interfacePair : pathPair.second)
                {
                    if (interfacePair.first == cableStatusConfiguration)
                    {
                        sdbusplus::message::object_path parentPath =
                            pathPair.first.parent_path();
                        if (assocInterfaces.find(parentPath.str) !=
                                assocInterfaces.end() &&
                            assocInterfaces[parentPath.str] != nullptr)
                        {
                            return;
                        }

                        auto findAssociationPath =
                            interfacePair.second.find("AssociationPath");
                        auto findConnectionType =
                            interfacePair.second.find("ConnectionType");
                        if (findAssociationPath == interfacePair.second.end())
                        {
                            std::cerr << interfacePair.first
                                      << " missing AssociationPath\n";
                            return;
                        }
                        if (findConnectionType == interfacePair.second.end())
                        {
                            std::cerr << interfacePair.first
                                      << " missing ConnectionType\n";
                            return;
                        }

                        std::string associationPath =
                            std::get<std::string>(findAssociationPath->second);
                        std::string connectionType =
                            std::get<std::string>(findConnectionType->second);
                        std::cout << "Adding association interface at path: "
                                  << parentPath.str << "\n";
                        assocInterfaces[parentPath.str] =
                            objServer.add_interface(parentPath,
                                                    association::interface);
                        assocInterfaces[parentPath.str]->register_property(
                            "Associations",
                            std::vector<Association>{
                                {"attached_cables", connectionType + "_chassis",
                                 associationPath}});
                        assocInterfaces[parentPath.str]->initialize();
                    }
                }
            }
        },
        "xyz.openbmc_project.EntityManager", "/",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
}

int main()
{
    int busId = -1;
    int slaveAddr = -1;
    bool gpioInverted = false;
    IntrusionSensorType type = IntrusionSensorType::gpio;

    // setup connection to dbus
    boost::asio::io_service io;
    systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objServer(systemBus);

    // setup object server, define interface
    systemBus->request_name("xyz.openbmc_project.IntrusionSensor");

    std::shared_ptr<sdbusplus::asio::dbus_interface> ifaceChassis =
        objServer.add_interface(
            "/xyz/openbmc_project/Intrusion/Chassis_Intrusion",
            "xyz.openbmc_project.Chassis.Intrusion");

    ChassisIntrusionSensor chassisIntrusionSensor(io, ifaceChassis);

    if (getIntrusionSensorConfig(&type, &busId, &slaveAddr, &gpioInverted))
    {
        chassisIntrusionSensor.start(type, busId, slaveAddr, gpioInverted);
    }

    // callback to handle configuration change
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }

            std::cout << "rescan due to configuration change \n";
            if (getIntrusionSensorConfig(&type, &busId, &slaveAddr,
                                         &gpioInverted))
            {
                chassisIntrusionSensor.start(type, busId, slaveAddr,
                                             gpioInverted);
            }
        };

    auto eventMatch = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + sensorType + "'",
        eventHandler);

    auto cableMatch = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        sdbusplus::bus::match::rules::propertiesChangedNamespace(
            std::string(inventoryPath), cableStatusConfiguration),
        [&objServer](sdbusplus::message::message& msg) {
            if (msg.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            generateAssociations(objServer);
        });

    if (initializeLanStatus(objServer))
    {
        // add match to monitor lan status change
        lanStatusMatch = std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*systemBus),
            "type='signal',"
            "interface='org.freedesktop.DBus.Properties',"
            "member='PropertiesChanged',"
            "arg0='org.freedesktop.network1.Link'",
            [](sdbusplus::message::message& msg) {
                processLanStatusChange(msg);
            });

        // add match to monitor entity manager signal about nic name config
        // change
        lanConfigMatch = std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*systemBus),
            "type='signal', "
            "member='PropertiesChanged',interface='org.freedesktop.DBus."
            "Properties',path_namespace='" +
                std::string(inventoryPath) + "',arg0='" + nicType + "'",
            [](sdbusplus::message::message& msg) {
                if (msg.is_method_error())
                {
                    std::cerr << "callback method error\n";
                    return;
                }
                getNicNameInfo();
            });
    }

    io.run();

    return 0;
}
