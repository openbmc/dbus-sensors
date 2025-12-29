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

#include "IntelCPUSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <peci.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <ios>
#include <iterator>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

// clang-format off
// this needs to be included last or we'll have build issues
#include <linux/peci-ioctl.h>
#if !defined(PECI_MBX_INDEX_DDR_DIMM_TEMP)
#define PECI_MBX_INDEX_DDR_DIMM_TEMP MBX_INDEX_DDR_DIMM_TEMP
#endif
// clang-format on

boost::container::flat_map<std::string, std::shared_ptr<IntelCPUSensor>>
    gCpuSensors;
boost::container::flat_map<std::string,
                           std::shared_ptr<sdbusplus::asio::dbus_interface>>
    inventoryIfaces;

enum State
{
    OFF,  // host powered down
    ON,   // host powered on
    READY // host powered on and mem test passed - fully ready
};

struct CPUConfig
{
    CPUConfig(const uint64_t& bus, const uint64_t& addr,
              const std::string& name, const State& state) :
        bus(bus), addr(addr), name(name), state(state)
    {}
    int bus;
    int addr;
    std::string name;
    State state;

    bool operator<(const CPUConfig& rhs) const
    {
        // NOLINTNEXTLINE
        return (name < rhs.name);
    }
};

static constexpr const char* peciDev = "/dev/peci-";
static constexpr const char* peciDevPath = "/sys/bus/peci/devices/";
static constexpr const char* rescanPath = "/sys/bus/peci/rescan";
static constexpr const unsigned int rankNumMax = 8;

static constexpr auto sensorTypes{std::to_array<const char*>({"XeonCPU"})};
static constexpr auto hiddenProps{std::to_array<const char*>(
    {IntelCPUSensor::labelTcontrol, "Tthrottle", "Tjmax"})};

void detectCpuAsync(
    boost::asio::steady_timer& pingTimer,
    boost::asio::steady_timer& creationTimer, boost::asio::io_context& io,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    boost::container::flat_set<CPUConfig>& cpuConfigs,
    ManagedObjectType& sensorConfigs);

std::string createSensorName(const std::string& label, const std::string& item,
                             const int& cpuId)
{
    std::string sensorName = label;
    if (item != "input")
    {
        sensorName += " " + item;
    }

    std::string cpuStr = "CPU" + std::to_string(cpuId);
    constexpr const char* subLabel = "DIMM";
    std::size_t found = label.find(subLabel);
    if (found != std::string::npos)
    {
        sensorName = cpuStr + " " + sensorName;
    }
    else
    {
        sensorName += " " + cpuStr;
    }
    // converting to Upper Camel case whole name
    bool isWordEnd = true;
    std::transform(sensorName.begin(), sensorName.end(), sensorName.begin(),
                   [&isWordEnd](int c) {
                       if (std::isspace(c) != 0)
                       {
                           isWordEnd = true;
                       }
                       else
                       {
                           if (isWordEnd)
                           {
                               isWordEnd = false;
                               return std::toupper(c);
                           }
                       }
                       return c;
                   });
    return sensorName;
}

bool createSensors(boost::asio::io_context& io,
                   sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
                   boost::container::flat_set<CPUConfig>& cpuConfigs,
                   ManagedObjectType& sensorConfigs)
{
    bool available = false;
    for (const CPUConfig& cpu : cpuConfigs)
    {
        if (cpu.state != State::OFF)
        {
            available = true;
            std::shared_ptr<sdbusplus::asio::dbus_interface>& iface =
                inventoryIfaces[cpu.name];
            if (iface != nullptr)
            {
                continue;
            }
            iface = objectServer.add_interface(
                cpuInventoryPath + std::string("/") + cpu.name,
                "xyz.openbmc_project.Inventory.Item");
            iface->register_property("PrettyName", cpu.name);
            iface->register_property("Present", true);
            iface->initialize();
        }
    }
    if (!available)
    {
        return false;
    }

    if (sensorConfigs.empty())
    {
        return false;
    }

    std::vector<std::filesystem::path> hwmonNamePaths;
    findFiles(std::filesystem::path(peciDevPath),
              R"(peci-\d+/\d+-.+/peci[-_].+/hwmon/hwmon\d+/name$)",
              hwmonNamePaths, 6);
    if (hwmonNamePaths.empty())
    {
        lg2::error("No CPU sensors in system");
        return false;
    }

    boost::container::flat_set<std::string> scannedDirectories;
    boost::container::flat_set<std::string> createdSensors;

    for (const std::filesystem::path& hwmonNamePath : hwmonNamePaths)
    {
        auto hwmonDirectory = hwmonNamePath.parent_path();

        auto ret = scannedDirectories.insert(hwmonDirectory.string());
        if (!ret.second)
        {
            continue; // already searched this path
        }

        std::filesystem::path::iterator it = hwmonNamePath.begin();
        std::advance(it, 6); // pick the 6th part for a PECI client device name
        std::string deviceName = *it;

        size_t bus = 0;
        size_t addr = 0;
        if (!getDeviceBusAddr(deviceName, bus, addr))
        {
            continue;
        }

        std::ifstream nameFile(hwmonNamePath);
        if (!nameFile.good())
        {
            lg2::error("Failure reading '{PATH}'", "PATH", hwmonNamePath);
            continue;
        }
        std::string hwmonName;
        std::getline(nameFile, hwmonName);
        nameFile.close();
        if (hwmonName.empty())
        {
            // shouldn't have an empty name file
            continue;
        }
        lg2::debug("Checking: '{PATH}': '{NAME}'", "PATH", hwmonNamePath,
                   "NAME", hwmonName);

        std::string sensorType;
        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        const SensorBaseConfiguration* baseConfiguration = nullptr;

        for (const auto& [path, cfgData] : sensorConfigs)
        {
            sensorData = &cfgData;
            for (const char* type : sensorTypes)
            {
                sensorType = type;
                auto sensorBase =
                    sensorData->find(configInterfaceName(sensorType));
                if (sensorBase != sensorData->end())
                {
                    baseConfiguration = &(*sensorBase);
                    break;
                }
            }
            if (baseConfiguration == nullptr)
            {
                lg2::error("error finding base configuration for '{NAME}'",
                           "NAME", hwmonName);
                continue;
            }
            auto configurationBus = baseConfiguration->second.find("Bus");
            auto configurationAddress =
                baseConfiguration->second.find("Address");

            if (configurationBus == baseConfiguration->second.end() ||
                configurationAddress == baseConfiguration->second.end())
            {
                lg2::error("error finding bus or address in configuration");
                continue;
            }

            if (std::get<uint64_t>(configurationBus->second) != bus ||
                std::get<uint64_t>(configurationAddress->second) != addr)
            {
                continue;
            }

            interfacePath = &path.str;
            break;
        }
        if (interfacePath == nullptr)
        {
            lg2::error("failed to find match for '{NAME}'", "NAME", hwmonName);
            continue;
        }

        auto findCpuId = baseConfiguration->second.find("CpuID");
        if (findCpuId == baseConfiguration->second.end())
        {
            lg2::error("could not determine CPU ID for '{NAME}'", "NAME",
                       hwmonName);
            continue;
        }
        int cpuId =
            std::visit(VariantToUnsignedIntVisitor(), findCpuId->second);

        auto directory = hwmonNamePath.parent_path();
        std::vector<std::filesystem::path> inputPaths;
        if (!findFiles(directory, R"((temp|power)\d+_(input|average|cap)$)",
                       inputPaths, 0))
        {
            lg2::error("No temperature sensors in system");
            continue;
        }

        // iterate through all found temp sensors
        for (const auto& inputPath : inputPaths)
        {
            auto fileParts = splitFileName(inputPath);
            if (!fileParts)
            {
                continue;
            }
            auto& [type, nr, item] = *fileParts;
            auto inputPathStr = inputPath.string();
            auto labelPath =
                boost::replace_all_copy(inputPathStr, item, "label");
            std::ifstream labelFile(labelPath);
            if (!labelFile.good())
            {
                lg2::error("Failure reading '{PATH}'", "PATH", labelPath);
                continue;
            }
            std::string label;
            std::getline(labelFile, label);
            labelFile.close();

            std::string sensorName = createSensorName(label, item, cpuId);

            auto findSensor = gCpuSensors.find(sensorName);
            if (findSensor != gCpuSensors.end())
            {
                if(label != "DTS")
                {
                   lg2::debug("Skipped: '{PATH}': '{NAME}' is already created",
                              "PATH", inputPath, "NAME", sensorName);
                   continue;
                }
            }

            // check hidden properties
            bool show = true;
            for (const char* prop : hiddenProps)
            {
                if (label == prop)
                {
                    show = false;
                    break;
                }
            }

            /*
             * Find if there is DtsCritOffset is configured in config file
             * set it if configured or else set it to 0
             */
            double dtsOffset = 0;
            if (label == "DTS")
            {
                auto findThrOffset =
                    baseConfiguration->second.find("DtsCritOffset");
                if (findThrOffset != baseConfiguration->second.end())
                {
                    dtsOffset = std::visit(VariantToDoubleVisitor(),
                                           findThrOffset->second);
                }
            }

            std::vector<thresholds::Threshold> sensorThresholds;
            std::string labelHead = label.substr(0, label.find(' '));
            parseThresholdsFromConfig(*sensorData, sensorThresholds,
                                      &labelHead);
            if (sensorThresholds.empty())
            {
                if (!parseThresholdsFromAttr(sensorThresholds, inputPathStr,
                                             IntelCPUSensor::sensorScaleFactor,
                                             dtsOffset, 0))
                {
                    lg2::error("error populating thresholds for '{NAME}'",
                               "NAME", sensorName);
                }
            }
            auto& sensorPtr = gCpuSensors[sensorName];
            // make sure destructor fires before creating a new one
            sensorPtr = nullptr;
            sensorPtr = std::make_shared<IntelCPUSensor>(
                inputPathStr, sensorType, objectServer, dbusConnection, io,
                sensorName, std::move(sensorThresholds), *interfacePath, cpuId,
                show, dtsOffset);
            sensorPtr->setupRead();
            createdSensors.insert(sensorName);
            lg2::debug("Mapped: '{PATH}' to '{NAME}'", "PATH", inputPath,
                       "NAME", sensorName);
        }
    }

    if (static_cast<unsigned int>(!createdSensors.empty()) != 0U)
    {
        if (createdSensors.size() == 1)
        {
            lg2::info("Sensor is created");
        }
        else
        {
            lg2::info("Sensors are created");
        }
    }

    return true;
}

bool exportDevice(const CPUConfig& config)
{
    std::ostringstream hex;
    hex << std::hex << config.addr;
    const std::string& addrHexStr = hex.str();
    std::string busStr = std::to_string(config.bus);

    std::string parameters = "peci-client 0x" + addrHexStr;
    std::string devPath = peciDevPath;
    std::string delDevice = devPath + "peci-" + busStr + "/delete_device";
    std::string newDevice = devPath + "peci-" + busStr + "/new_device";
    std::string newClient = devPath + busStr + "-" + addrHexStr + "/driver";

    std::filesystem::path devicePath(newDevice);
    const std::string& dir = devicePath.parent_path().string();
    for (const auto& path : std::filesystem::directory_iterator(dir))
    {
        if (!std::filesystem::is_directory(path))
        {
            continue;
        }

        const std::string& directoryName = path.path().filename();
        if (directoryName.starts_with(busStr) &&
            directoryName.ends_with(addrHexStr))
        {
            lg2::debug("'{PARAMETERS}' on bus '{BUS}' is already exported",
                       "PARAMETERS", parameters, "BUS", busStr);

            std::ofstream delDeviceFile(delDevice);
            if (!delDeviceFile.good())
            {
                lg2::error("Error opening '{DEVICE}'", "DEVICE", delDevice);
                return false;
            }
            delDeviceFile << parameters;
            delDeviceFile.close();

            break;
        }
    }

    std::ofstream deviceFile(newDevice);
    if (!deviceFile.good())
    {
        lg2::error("Error opening '{DEVICE}'", "DEVICE", newDevice);
        return false;
    }
    deviceFile << parameters;
    deviceFile.close();

    if (!std::filesystem::exists(newClient))
    {
        lg2::error("Error creating '{CLIENT}'", "CLIENT", newClient);
        return false;
    }

    lg2::info("'{PARAMETERS}' on bus '{BUS}' is exported", "PARAMETERS",
              parameters, "BUS", busStr);

    return true;
}

void detectCpu(boost::asio::steady_timer& pingTimer,
               boost::asio::steady_timer& creationTimer,
               boost::asio::io_context& io,
               sdbusplus::asio::object_server& objectServer,
               std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
               boost::container::flat_set<CPUConfig>& cpuConfigs,
               ManagedObjectType& sensorConfigs)
{
    size_t rescanDelaySeconds = 0;
    static bool keepPinging = false;
    int peciFd = -1;

    for (CPUConfig& config : cpuConfigs)
    {
        if (config.state == State::READY)
        {
            continue;
        }

        std::fstream rescan{rescanPath, std::ios::out};
        if (rescan.is_open())
        {
            std::vector<std::filesystem::path> peciPaths;
            std::ostringstream searchPath;
            searchPath << std::hex << "peci-" << config.bus << "/" << config.bus
                       << "-" << config.addr;
            findFiles(std::filesystem::path(peciDevPath + searchPath.str()),
                      R"(peci_cpu.dimmtemp.+/hwmon/hwmon\d+/name$)", peciPaths,
                      3);
            if (!peciPaths.empty())
            {
                config.state = State::READY;
                rescanDelaySeconds = 1;
            }
            else
            {
                findFiles(std::filesystem::path(peciDevPath + searchPath.str()),
                          R"(peci_cpu.cputemp.+/hwmon/hwmon\d+/name$)",
                          peciPaths, 3);
                if (!peciPaths.empty())
                {
                    config.state = State::ON;
                    rescanDelaySeconds = 3;
                }
                else
                {
                    // https://www.kernel.org/doc/html/latest/admin-guide/abi-testing.html#abi-sys-bus-peci-rescan
                    rescan << "1";
                }
            }
            if (config.state != State::READY)
            {
                keepPinging = true;
            }

            continue;
        }

        std::string peciDevPath = peciDev + std::to_string(config.bus);

        peci_SetDevName(peciDevPath.data());

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        if ((peci_Lock(&peciFd, PECI_NO_WAIT) != PECI_CC_SUCCESS) ||
            (peciFd < 0))
        {
            lg2::error("unable to open '{PATH}', '{ERRNO}'", "PATH",
                       peciDevPath, "ERRNO", std::strerror(errno));
            detectCpuAsync(pingTimer, creationTimer, io, objectServer,
                           dbusConnection, cpuConfigs, sensorConfigs);
            return;
        }

        State newState = State::OFF;

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        if (peci_Ping(config.addr) == PECI_CC_SUCCESS)
        {
            bool dimmReady = false;
            for (unsigned int rank = 0; rank < rankNumMax; rank++)
            {
                std::array<uint8_t, 8> pkgConfig{};
                uint8_t cc = 0;

                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
                if (peci_RdPkgConfig(config.addr, PECI_MBX_INDEX_DDR_DIMM_TEMP,
                                     rank, 4, pkgConfig.data(), &cc) ==
                    PECI_CC_SUCCESS)
                {
                    // Depending on CPU generation, both 0 and 0xFF can be used
                    // to indicate no DIMM presence
                    if (((pkgConfig[0] != 0xFF) && (pkgConfig[0] != 0U)) ||
                        ((pkgConfig[1] != 0xFF) && (pkgConfig[1] != 0U)))
                    {
                        dimmReady = true;
                        break;
                    }
                }
                else
                {
                    break;
                }
            }

            if (dimmReady)
            {
                newState = State::READY;
            }
            else
            {
                newState = State::ON;
            }
        }

        if (config.state != newState)
        {
            if (newState != State::OFF)
            {
                if (config.state == State::OFF)
                {
                    std::array<uint8_t, 8> pkgConfig{};
                    uint8_t cc = 0;

                    if (peci_RdPkgConfig(config.addr, PECI_MBX_INDEX_CPU_ID, 0,
                                         4, pkgConfig.data(), &cc) ==
                        PECI_CC_SUCCESS)
                    {
                        lg2::info("'{NAME}' is detected", "NAME", config.name);
                        if (!exportDevice(config))
                        {
                            newState = State::OFF;
                        }
                    }
                    else
                    {
                        newState = State::OFF;
                    }
                }

                if (newState == State::ON)
                {
                    rescanDelaySeconds = 3;
                }
                else if (newState == State::READY)
                {
                    rescanDelaySeconds = 5;
                    lg2::info("DIMM(s) on '{NAME}' is/are detected", "NAME",
                              config.name);
                }
            }

            config.state = newState;
        }

        if (config.state != State::READY)
        {
            keepPinging = true;
        }

        lg2::debug("'{NAME}', state: '{STATE}'", "NAME", config.name, "STATE",
                   config.state);
        peci_Unlock(peciFd);
    }

    if (rescanDelaySeconds != 0U)
    {
        creationTimer.expires_after(std::chrono::seconds(rescanDelaySeconds));
        creationTimer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }

            if (!createSensors(io, objectServer, dbusConnection, cpuConfigs,
                               sensorConfigs) ||
                keepPinging)
            {
                detectCpuAsync(pingTimer, creationTimer, io, objectServer,
                               dbusConnection, cpuConfigs, sensorConfigs);
            }
        });
    }
    else if (keepPinging)
    {
        detectCpuAsync(pingTimer, creationTimer, io, objectServer,
                       dbusConnection, cpuConfigs, sensorConfigs);
    }
}

void detectCpuAsync(
    boost::asio::steady_timer& pingTimer,
    boost::asio::steady_timer& creationTimer, boost::asio::io_context& io,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    boost::container::flat_set<CPUConfig>& cpuConfigs,
    ManagedObjectType& sensorConfigs)
{
    pingTimer.expires_after(std::chrono::seconds(1));
    pingTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }

        detectCpu(pingTimer, creationTimer, io, objectServer, dbusConnection,
                  cpuConfigs, sensorConfigs);
    });
}

bool getCpuConfig(const std::shared_ptr<sdbusplus::asio::connection>& systemBus,
                  boost::container::flat_set<CPUConfig>& cpuConfigs,
                  ManagedObjectType& sensorConfigs,
                  sdbusplus::asio::object_server& objectServer)
{
    bool useCache = false;
    sensorConfigs.clear();
    // use new data the first time, then refresh
    for (const char* type : sensorTypes)
    {
        if (!getSensorConfiguration(type, systemBus, sensorConfigs, useCache))
        {
            return false;
        }
        useCache = true;
    }

    // check PECI client addresses and names from CPU configuration
    // before starting ping operation
    for (const char* type : sensorTypes)
    {
        for (const auto& [path, cfgData] : sensorConfigs)
        {
            for (const auto& [intf, cfg] : cfgData)
            {
                if (intf != configInterfaceName(type))
                {
                    continue;
                }

                auto findName = cfg.find("Name");
                if (findName == cfg.end())
                {
                    continue;
                }
                std::string nameRaw =
                    std::visit(VariantToStringVisitor(), findName->second);
                std::string name =
                    std::regex_replace(nameRaw, illegalDbusRegex, "_");

                auto present = std::optional<bool>();
                // if we can't detect it via gpio, we set presence later
                for (const auto& [suppIntf, suppCfg] : cfgData)
                {
                    if (suppIntf.find("PresenceGpio") != std::string::npos)
                    {
                        present = cpuIsPresent(suppCfg);
                        break;
                    }
                }

                if (!inventoryIfaces.contains(name) && present)
                {
                    auto iface = objectServer.add_interface(
                        cpuInventoryPath + std::string("/") + name,
                        "xyz.openbmc_project.Inventory.Item");
                    iface->register_property("PrettyName", name);
                    iface->register_property("Present", *present);
                    iface->initialize();
                    inventoryIfaces[name] = std::move(iface);
                }

                auto findBus = cfg.find("Bus");
                if (findBus == cfg.end())
                {
                    lg2::error("Can't find 'Bus' setting in '{NAME}'", "NAME",
                               name);
                    continue;
                }
                uint64_t bus =
                    std::visit(VariantToUnsignedIntVisitor(), findBus->second);

                auto findAddress = cfg.find("Address");
                if (findAddress == cfg.end())
                {
                    lg2::error("Can't find 'Address' setting in '{NAME}'",
                               "NAME", name);
                    continue;
                }
                uint64_t addr = std::visit(VariantToUnsignedIntVisitor(),
                                           findAddress->second);

                lg2::debug(
                    "bus: {BUS}, addr: {ADDR}, name: {NAME}, type: {TYPE}",
                    "BUS", bus, "ADDR", addr, "NAME", name, "TYPE", type);

                cpuConfigs.emplace(bus, addr, name, State::OFF);
            }
        }
    }

    if (static_cast<unsigned int>(!cpuConfigs.empty()) != 0U)
    {
        if (cpuConfigs.size() == 1)
        {
            lg2::info("CPU config is parsed");
        }
        else
        {
            lg2::info("CPU configs are parsed");
        }
        return true;
    }

    return false;
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    boost::container::flat_set<CPUConfig> cpuConfigs;

    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    boost::asio::steady_timer pingTimer(io);
    boost::asio::steady_timer creationTimer(io);
    boost::asio::steady_timer filterTimer(io);
    ManagedObjectType sensorConfigs;

    filterTimer.expires_after(std::chrono::seconds(1));
    filterTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }

        if (getCpuConfig(systemBus, cpuConfigs, sensorConfigs, objectServer))
        {
            detectCpuAsync(pingTimer, creationTimer, io, objectServer,
                           systemBus, cpuConfigs, sensorConfigs);
        }
    });

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t& message) {
            lg2::debug("'{PATH}' is changed", "PATH", message.get_path());

            // this implicitly cancels the timer
            filterTimer.expires_after(std::chrono::seconds(1));
            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }

                if (getCpuConfig(systemBus, cpuConfigs, sensorConfigs,
                                 objectServer))
                {
                    detectCpuAsync(pingTimer, creationTimer, io, objectServer,
                                   systemBus, cpuConfigs, sensorConfigs);
                }
            });
        };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(*systemBus, sensorTypes, eventHandler);

    systemBus->request_name("xyz.openbmc_project.IntelCPUSensor");

    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
