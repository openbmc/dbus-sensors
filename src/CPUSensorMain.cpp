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

#include <fcntl.h>

#include <CPUSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/process/child.hpp>
#include <experimental/filesystem>
#include <fstream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

// clang-format off
// this needs to be included last or we'll have build issues
#include <linux/peci-ioctl.h>
// clang-format on

static constexpr bool DEBUG = false;

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
    {
    }
    int bus;
    int addr;
    std::string name;
    State state;

    bool operator<(const CPUConfig& rhs) const
    {
        return (name < rhs.name);
    }
};

static constexpr const char* peciDev = "/dev/peci-";
static constexpr const unsigned int rankNumMax = 8;

namespace fs = std::experimental::filesystem;
namespace variant_ns = sdbusplus::message::variant_ns;
static constexpr const char* configPrefix =
    "xyz.openbmc_project.Configuration.";
static constexpr std::array<const char*, 3> sensorTypes = {
    "SkylakeCPU", "BroadwellCPU", "HaswellCPU"};

void detectCpuAsync(
    boost::asio::deadline_timer& pingTimer,
    boost::asio::deadline_timer& creationTimer, boost::asio::io_service& io,
    sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<CPUSensor>>&
        sensors,
    boost::container::flat_set<CPUConfig>& configs,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection);

bool createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<CPUSensor>>&
        sensors,
    boost::container::flat_set<CPUConfig>& configs,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    bool available = false;
    for (CPUConfig cpu : configs)
    {
        if (cpu.state != State::OFF)
        {
            available = true;
            break;
        }
    }
    if (!available)
    {
        return false;
    }

    // use new data the first time, then refresh
    ManagedObjectType sensorConfigurations;
    bool useCache = false;
    for (const char* type : sensorTypes)
    {
        if (!getSensorConfiguration(configPrefix + std::string(type),
                                    dbusConnection, sensorConfigurations,
                                    useCache))
        {
            return false;
        }
        useCache = true;
    }

    std::vector<fs::path> hwmonNamePaths;
    if (!findFiles(fs::path(R"(/sys/bus/peci/devices)"),
                   R"(peci-\d+/\d+-.+/peci-.+/hwmon/hwmon\d+/name$)",
                   hwmonNamePaths, 1))
    {
        std::cerr << "No CPU sensors in system\n";
        return true;
    }

    boost::container::flat_set<std::string> scannedDirectories;
    boost::container::flat_set<std::string> createdSensors;

    for (fs::path& hwmonNamePath : hwmonNamePaths)
    {
        const std::string& pathStr = hwmonNamePath.string();
        auto hwmonDirectory = hwmonNamePath.parent_path();

        auto ret = scannedDirectories.insert(hwmonDirectory.string());
        if (!ret.second)
        {
            continue; // already searched this path
        }

        fs::path::iterator it = hwmonNamePath.begin();
        std::advance(it, 6); // pick the 6th part for a PECI client device name
        std::string deviceName = *it;
        auto findHyphen = deviceName.find("-");
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
            addr = std::stoi(addrStr, 0, 16);
        }
        catch (std::invalid_argument)
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
        if (!hwmonName.size())
        {
            // shouldn't have an empty name file
            continue;
        }
        if (DEBUG)
        {
            std::cout << "Checking: " << hwmonNamePath << ": " << hwmonName
                      << "\n";
        }

        std::string sensorType;
        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        const std::pair<std::string, boost::container::flat_map<
                                         std::string, BasicVariantType>>*
            baseConfiguration = nullptr;

        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigurations)
        {
            sensorData = &(sensor.second);
            for (const char* type : sensorTypes)
            {
                sensorType = configPrefix + std::string(type);
                auto sensorBase = sensorData->find(sensorType);
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

            if (sdbusplus::message::variant_ns::get<uint64_t>(
                    configurationBus->second) != bus ||
                sdbusplus::message::variant_ns::get<uint64_t>(
                    configurationAddress->second) != addr)
            {
                continue;
            }

            interfacePath = &(sensor.first.str);
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
            variant_ns::visit(VariantToUnsignedIntVisitor(), findCpuId->second);

        auto directory = hwmonNamePath.parent_path();
        std::vector<fs::path> inputPaths;
        if (!findFiles(fs::path(directory), R"(temp\d+_input$)", inputPaths, 0))
        {
            std::cerr << "No temperature sensors in system\n";
            continue;
        }

        // iterate through all found temp sensors
        for (auto& inputPath : inputPaths)
        {
            auto inputPathStr = inputPath.string();
            auto labelPath =
                boost::replace_all_copy(inputPathStr, "input", "label");
            std::ifstream labelFile(labelPath);
            if (!labelFile.good())
            {
                std::cerr << "Failure reading " << labelPath << "\n";
                continue;
            }
            std::string label;
            std::getline(labelFile, label);
            labelFile.close();
            std::string sensorName = label + " CPU" + std::to_string(cpuId);

            auto findSensor = sensors.find(sensorName);
            if (findSensor != sensors.end())
            {
                if (DEBUG)
                {
                    std::cout << "Skipped: " << inputPath << ": " << sensorName
                              << " is already created\n";
                }
                continue;
            }

            std::vector<thresholds::Threshold> sensorThresholds;
            std::string labelHead = label.substr(0, label.find(" "));
            parseThresholdsFromConfig(*sensorData, sensorThresholds,
                                      &labelHead);
            if (!sensorThresholds.size())
            {
                if (!parseThresholdsFromAttr(sensorThresholds, inputPathStr,
                                             CPUSensor::sensorScaleFactor))
                {
                    std::cerr << "error populating thresholds for "
                              << sensorName << "\n";
                }
            }
            sensors[sensorName] = std::make_unique<CPUSensor>(
                inputPathStr, sensorType, objectServer, dbusConnection, io,
                sensorName, std::move(sensorThresholds), *interfacePath);
            createdSensors.insert(sensorName);
            if (DEBUG)
            {
                std::cout << "Mapped: " << inputPath << " to " << sensorName
                          << "\n";
            }
        }
    }

    if (createdSensors.size())
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

    std::experimental::filesystem::path devicePath(device);
    const std::string& dir = devicePath.parent_path().string();
    for (const auto& path :
         std::experimental::filesystem::directory_iterator(dir))
    {
        if (!std::experimental::filesystem::is_directory(path))
        {
            continue;
        }

        const std::string& directoryName = path.path().filename();
        if (boost::starts_with(directoryName, busStr) &&
            boost::ends_with(directoryName, addrHexStr))
        {
            if (DEBUG)
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

void detectCpu(boost::asio::deadline_timer& pingTimer,
               boost::asio::deadline_timer& creationTimer,
               boost::asio::io_service& io,
               sdbusplus::asio::object_server& objectServer,
               boost::container::flat_map<std::string,
                                          std::unique_ptr<CPUSensor>>& sensors,
               boost::container::flat_set<CPUConfig>& configs,
               std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    size_t rescanDelaySeconds = 0;
    bool keepPinging = false;

    for (CPUConfig& config : configs)
    {
        std::string peciDevPath = peciDev + std::to_string(config.bus);
        auto file = open(peciDevPath.c_str(), O_RDWR | O_CLOEXEC);
        if (file < 0)
        {
            std::cerr << "unable to open " << peciDevPath << "\n";
            std::exit(EXIT_FAILURE);
        }

        State state;
        struct peci_ping_msg msg;
        msg.addr = config.addr;
        if (!ioctl(file, PECI_IOC_PING, &msg))
        {
            bool dimmReady = false;
            for (unsigned int rank = 0; rank < rankNumMax; rank++)
            {
                struct peci_rd_pkg_cfg_msg msg;
                msg.addr = config.addr;
                msg.index = MBX_INDEX_DDR_DIMM_TEMP;
                msg.param = rank;
                msg.rx_len = 4;
                if (!ioctl(file, PECI_IOC_RD_PKG_CFG, &msg))
                {
                    if (msg.pkg_config[0] || msg.pkg_config[1] ||
                        msg.pkg_config[2])
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
                state = State::READY;
            }
            else
            {
                state = State::ON;
            }
        }
        else
        {
            state = State::OFF;
        }

        close(file);

        if (config.state != state)
        {
            if (config.state == State::OFF)
            {
                std::cout << config.name << " is detected\n";
                exportDevice(config);
            }
            if (state == State::READY)
            {
                std::cout << "DIMM(s) on " << config.name
                          << " is/are detected\n";
            }
            config.state = state;
        }

        if (config.state != State::OFF)
        {
            if (config.state == State::ON)
            {
                rescanDelaySeconds = 1;
            }
            else
            {
                rescanDelaySeconds = 5;
            }
        }

        if (config.state != State::READY)
        {
            keepPinging = true;
        }

        if (DEBUG)
        {
            std::cout << config.name << ", state: " << config.state << "\n";
        }
    }

    if (rescanDelaySeconds)
    {
        creationTimer.expires_from_now(
            boost::posix_time::seconds(rescanDelaySeconds));
        creationTimer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }

            if (!createSensors(io, objectServer, sensors, configs,
                               dbusConnection))
            {
                detectCpuAsync(pingTimer, creationTimer, io, objectServer,
                               sensors, configs, dbusConnection);
            }
        });
    }

    if (keepPinging)
    {
        detectCpuAsync(pingTimer, creationTimer, io, objectServer, sensors,
                       configs, dbusConnection);
    }
}

void detectCpuAsync(
    boost::asio::deadline_timer& pingTimer,
    boost::asio::deadline_timer& creationTimer, boost::asio::io_service& io,
    sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<CPUSensor>>&
        sensors,
    boost::container::flat_set<CPUConfig>& configs,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    pingTimer.expires_from_now(boost::posix_time::seconds(1));
    pingTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }

        detectCpu(pingTimer, creationTimer, io, objectServer, sensors, configs,
                  dbusConnection);
    });
}

bool getCpuConfig(const std::shared_ptr<sdbusplus::asio::connection>& systemBus,
                  boost::container::flat_set<CPUConfig>& configs)
{
    ManagedObjectType sensorConfigurations;
    bool useCache = false;
    // use new data the first time, then refresh
    for (const char* type : sensorTypes)
    {
        if (!getSensorConfiguration(configPrefix + std::string(type), systemBus,
                                    sensorConfigurations, useCache))
        {
            return false;
        }
        useCache = true;
    }

    // check PECI client addresses and DT overlay names from CPU configuration
    // before starting ping operation
    for (const char* type : sensorTypes)
    {
        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigurations)
        {
            for (const std::pair<
                     std::string,
                     boost::container::flat_map<std::string, BasicVariantType>>&
                     config : sensor.second)
            {
                if ((configPrefix + std::string(type)) != config.first)
                {
                    continue;
                }

                auto findName = config.second.find("Name");
                if (findName == config.second.end())
                {
                    continue;
                }
                std::string nameRaw = variant_ns::visit(
                    VariantToStringVisitor(), findName->second);
                std::string name =
                    std::regex_replace(nameRaw, illegalDbusRegex, "_");

                auto findBus = config.second.find("Bus");
                if (findBus == config.second.end())
                {
                    std::cerr << "Can't find 'Bus' setting in " << name << "\n";
                    continue;
                }
                uint64_t bus = variant_ns::visit(VariantToUnsignedIntVisitor(),
                                                 findBus->second);

                auto findAddress = config.second.find("Address");
                if (findAddress == config.second.end())
                {
                    std::cerr << "Can't find 'Address' setting in " << name
                              << "\n";
                    continue;
                }
                uint64_t addr = variant_ns::visit(VariantToUnsignedIntVisitor(),
                                                  findAddress->second);

                if (DEBUG)
                {
                    std::cout << "bus: " << bus << "\n";
                    std::cout << "addr: " << addr << "\n";
                    std::cout << "name: " << name << "\n";
                    std::cout << "type: " << type << "\n";
                }

                configs.emplace(bus, addr, name, State::OFF);
            }
        }
    }

    if (configs.size())
    {
        std::cout << "CPU config" << (configs.size() == 1 ? " is" : "s are")
                  << " parsed\n";
        return true;
    }

    return false;
}

int main(int argc, char** argv)
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    boost::container::flat_set<CPUConfig> configs;

    systemBus->request_name("xyz.openbmc_project.CPUSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::unique_ptr<CPUSensor>> sensors;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    boost::asio::deadline_timer pingTimer(io);
    boost::asio::deadline_timer creationTimer(io);
    boost::asio::deadline_timer filterTimer(io);

    filterTimer.expires_from_now(boost::posix_time::seconds(1));
    filterTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }

        if (getCpuConfig(systemBus, configs))
        {
            detectCpuAsync(pingTimer, creationTimer, io, objectServer, sensors,
                           configs, systemBus);
        }
    });

    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }

            if (DEBUG)
            {
                std::cout << message.get_path() << " is changed\n";
            }

            // this implicitly cancels the timer
            filterTimer.expires_from_now(boost::posix_time::seconds(1));
            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }

                if (getCpuConfig(systemBus, configs))
                {
                    detectCpuAsync(pingTimer, creationTimer, io, objectServer,
                                   sensors, configs, systemBus);
                }
            });
        };

    for (const char* type : sensorTypes)
    {
        auto match = std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*systemBus),
            "type='signal',member='PropertiesChanged',path_namespace='" +
                std::string(inventoryPath) + "',arg0namespace='" +
                configPrefix + type + "'",
            eventHandler);
        matches.emplace_back(std::move(match));
    }

    io.run();
}
