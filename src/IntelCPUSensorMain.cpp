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
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <fcntl.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <array>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
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

static constexpr bool debug = false;

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
        bus(bus),
        addr(addr), name(name), state(state)
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
static constexpr const unsigned int rankNumMax = 8;

namespace fs = std::filesystem;

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
    sensorName += " CPU" + std::to_string(cpuId);
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

    std::vector<fs::path> hwmonNamePaths;
    if (!findFiles(fs::path(R"(/sys/bus/peci/devices/peci-0)"),
                   R"(\d+-.+/peci-.+/hwmon/hwmon\d+/name$)", hwmonNamePaths, 5))
    {
        std::cerr << "No CPU sensors in system\n";
        return true;
    }

    boost::container::flat_set<std::string> scannedDirectories;
    boost::container::flat_set<std::string> createdSensors;

    for (const fs::path& hwmonNamePath : hwmonNamePaths)
    {
        auto hwmonDirectory = hwmonNamePath.parent_path();

        auto ret = scannedDirectories.insert(hwmonDirectory.string());
        if (!ret.second)
        {
            continue; // already searched this path
        }

        fs::path::iterator it = hwmonNamePath.begin();
        std::advance(it, 6); // pick the 6th part for a PECI client device name
        std::string deviceName = *it;
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
        catch (const std::invalid_argument&)
        {
            continue;
        }

        std::ifstream nameFile(hwmonNamePath);
        if (!nameFile.good())
        {
            std::cerr << "Failure reading " << hwmonNamePath << "\n";
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
        if (debug)
        {
            std::cout << "Checking: " << hwmonNamePath << ": " << hwmonName
                      << "\n";
        }

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
                std::cerr << "error finding base configuration for" << hwmonName
                          << "\n";
                continue;
            }
            auto configurationBus = baseConfiguration->second.find("Bus");
            auto configurationAddress =
                baseConfiguration->second.find("Address");

            if (configurationBus == baseConfiguration->second.end() ||
                configurationAddress == baseConfiguration->second.end())
            {
                std::cerr << "error finding bus or address in configuration";
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
            std::cerr << "failed to find match for " << hwmonName << "\n";
            continue;
        }

        auto findCpuId = baseConfiguration->second.find("CpuID");
        if (findCpuId == baseConfiguration->second.end())
        {
            std::cerr << "could not determine CPU ID for " << hwmonName << "\n";
            continue;
        }
        int cpuId =
            std::visit(VariantToUnsignedIntVisitor(), findCpuId->second);

        auto directory = hwmonNamePath.parent_path();
        std::vector<fs::path> inputPaths;
        if (!findFiles(directory, R"((temp|power)\d+_(input|average|cap)$)",
                       inputPaths, 0))
        {
            std::cerr << "No temperature sensors in system\n";
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
                std::cerr << "Failure reading " << labelPath << "\n";
                continue;
            }
            std::string label;
            std::getline(labelFile, label);
            labelFile.close();

            std::string sensorName = createSensorName(label, item, cpuId);

            auto findSensor = gCpuSensors.find(sensorName);
            if (findSensor != gCpuSensors.end())
            {
                if (debug)
                {
                    std::cout << "Skipped: " << inputPath << ": " << sensorName
                              << " is already created\n";
                }
                continue;
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
                                             dtsOffset))
                {
                    std::cerr << "error populating thresholds for "
                              << sensorName << "\n";
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
            if (debug)
            {
                std::cout << "Mapped: " << inputPath << " to " << sensorName
                          << "\n";
            }
        }
    }

    if (static_cast<unsigned int>(!createdSensors.empty()) != 0U)
    {
        std::cout << "Sensor" << (createdSensors.size() == 1 ? " is" : "s are")
                  << " created\n";
    }

    return true;
}

void exportDevice(const CPUConfig& config)
{
    std::ostringstream hex;
    hex << std::hex << config.addr;
    const std::string& addrHexStr = hex.str();
    std::string busStr = std::to_string(config.bus);

    std::string parameters = "peci-client 0x" + addrHexStr;
    std::string device = "/sys/bus/peci/devices/peci-" + busStr + "/new_device";

    std::filesystem::path devicePath(device);
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
            if (debug)
            {
                std::cout << parameters << " on bus " << busStr
                          << " is already exported\n";
            }
            return;
        }
    }

    std::ofstream deviceFile(device);
    if (!deviceFile.good())
    {
        std::cerr << "Error writing " << device << "\n";
        return;
    }
    deviceFile << parameters;
    deviceFile.close();

    std::cout << parameters << " on bus " << busStr << " is exported\n";
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

    for (CPUConfig& config : cpuConfigs)
    {
        std::string peciDevPath = peciDev + std::to_string(config.bus);

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        auto file = open(peciDevPath.c_str(), O_RDWR | O_CLOEXEC);
        if (file < 0)
        {
            std::cerr << "unable to open " << peciDevPath << " "
                      << std::strerror(errno) << "\n";
            detectCpuAsync(pingTimer, creationTimer, io, objectServer,
                           dbusConnection, cpuConfigs, sensorConfigs);
            return;
        }

        State newState = State::OFF;
        struct peci_ping_msg msg
        {};
        msg.addr = config.addr;

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        if (ioctl(file, PECI_IOC_PING, &msg) == 0)
        {
            bool dimmReady = false;
            for (unsigned int rank = 0; rank < rankNumMax; rank++)
            {
                struct peci_rd_pkg_cfg_msg msg
                {};
                msg.addr = config.addr;
                msg.index = PECI_MBX_INDEX_DDR_DIMM_TEMP;
                msg.param = rank;
                msg.rx_len = 4;

                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
                if (ioctl(file, PECI_IOC_RD_PKG_CFG, &msg) == 0)
                {
                    if ((msg.pkg_config[0] != 0U) ||
                        (msg.pkg_config[1] != 0U) || (msg.pkg_config[2] != 0U))
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

        close(file);

        if (config.state != newState)
        {
            if (newState != State::OFF)
            {
                if (config.state == State::OFF)
                {
                    std::cout << config.name << " is detected\n";
                    exportDevice(config);
                }

                if (newState == State::ON)
                {
                    rescanDelaySeconds = 3;
                }
                else if (newState == State::READY)
                {
                    rescanDelaySeconds = 5;
                    std::cout << "DIMM(s) on " << config.name
                              << " is/are detected\n";
                }
            }

            config.state = newState;
        }

        if (config.state != State::READY)
        {
            keepPinging = true;
        }

        if (debug)
        {
            std::cout << config.name << ", state: " << config.state << "\n";
        }
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

                if (inventoryIfaces.find(name) == inventoryIfaces.end() &&
                    present)
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
                    std::cerr << "Can't find 'Bus' setting in " << name << "\n";
                    continue;
                }
                uint64_t bus =
                    std::visit(VariantToUnsignedIntVisitor(), findBus->second);

                auto findAddress = cfg.find("Address");
                if (findAddress == cfg.end())
                {
                    std::cerr << "Can't find 'Address' setting in " << name
                              << "\n";
                    continue;
                }
                uint64_t addr = std::visit(VariantToUnsignedIntVisitor(),
                                           findAddress->second);

                if (debug)
                {
                    std::cout << "bus: " << bus << "\n";
                    std::cout << "addr: " << addr << "\n";
                    std::cout << "name: " << name << "\n";
                    std::cout << "type: " << type << "\n";
                }

                cpuConfigs.emplace(bus, addr, name, State::OFF);
            }
        }
    }

    if (static_cast<unsigned int>(!cpuConfigs.empty()) != 0U)
    {
        std::cout << "CPU config" << (cpuConfigs.size() == 1 ? " is" : "s are")
                  << " parsed\n";
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
        if (message.is_method_error())
        {
            std::cerr << "callback method error\n";
            return;
        }

        if (debug)
        {
            std::cout << message.get_path() << " is changed\n";
        }

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
