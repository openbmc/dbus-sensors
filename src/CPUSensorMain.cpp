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
#include <linux/peci-ioctl.h>

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

static constexpr bool DEBUG = false;

enum State
{
    OFF,  // host powered down
    ON,   // host powered on
    READY // host powered on and mem test passed - fully ready
};

struct CPUConfig
{
    CPUConfig(const int& address, const std::string& overlayName,
              const State& st) :
        addr(address),
        ovName(overlayName), state(st)
    {
    }
    int addr;
    std::string ovName;
    State state;

    bool operator<(const CPUConfig& rhs) const
    {
        return (ovName < rhs.ovName);
    }
};

static constexpr const char* DT_OVERLAY = "/usr/bin/dtoverlay";
static constexpr const char* OVERLAY_DIR = "/tmp/overlays";
static constexpr const char* PECI_DEV = "/dev/peci0";
static constexpr const unsigned int RANK_NUM_MAX = 8;

namespace fs = std::experimental::filesystem;
static constexpr const char* CONFIG_PREFIX =
    "xyz.openbmc_project.Configuration.";
static constexpr std::array<const char*, 3> SENSOR_TYPES = {
    "SkylakeCPU", "BroadwellCPU", "HaswellCPU"};

const static std::regex ILLEGAL_NAME_REGEX("[^A-Za-z0-9_]");

void createSensors(
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
        return;
    }

    // use new data the first time, then refresh
    ManagedObjectType sensorConfigurations;
    bool useCache = false;
    for (const char* type : SENSOR_TYPES)
    {
        if (!getSensorConfiguration(CONFIG_PREFIX + std::string(type),
                                    dbusConnection, sensorConfigurations,
                                    useCache))
        {
            std::cerr << "error communicating to entity manager\n";
            return;
        }
        useCache = true;
    }

    std::vector<fs::path> oemNamePaths;
    if (!find_files(fs::path(R"(/sys/bus/peci/devices)"),
                    R"(peci\d+/\d+-.+/of_node/oemname1$)", oemNamePaths, 2))
    {
        std::cerr << "No CPU sensors in system\n";
        return;
    }

    for (fs::path& oemNamePath : oemNamePaths)
    {
        std::ifstream nameFile(oemNamePath);
        if (!nameFile.good())
        {
            std::cerr << "Failure reading " << oemNamePath << "\n";
            continue;
        }
        std::string oemName;
        std::getline(nameFile, oemName);
        nameFile.close();
        if (!oemName.size())
        {
            // shouldn't have an empty name file
            continue;
        }
        oemName.pop_back(); // remove trailing null
        if (DEBUG)
            std::cout << "Checking: " << oemNamePath << ": " << oemName << "\n";

        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigurations)
        {
            if (!boost::ends_with(sensor.first.str, oemName))
            {
                continue;
            }
            sensorData = &(sensor.second);
            interfacePath = &(sensor.first.str);
            break;
        }
        if (sensorData == nullptr)
        {
            std::cerr << "failed to find match for " << oemName << "\n";
            continue;
        }
        const std::pair<std::string, boost::container::flat_map<
                                         std::string, BasicVariantType>>*
            baseConfiguration = nullptr;
        std::string sensorObjectType;
        for (const char* type : SENSOR_TYPES)
        {
            sensorObjectType = CONFIG_PREFIX + std::string(type);
            auto sensorBase = sensorData->find(sensorObjectType);
            if (sensorBase != sensorData->end())
            {
                baseConfiguration = &(*sensorBase);
                break;
            }
        }

        if (baseConfiguration == nullptr)
        {
            std::cerr << "error finding base configuration for" << oemName
                      << "\n";
            continue;
        }

        auto findCpuId = baseConfiguration->second.find("CpuID");
        if (findCpuId == baseConfiguration->second.end())
        {
            std::cerr << "could not determine CPU ID for " << oemName << "\n";
            continue;
        }
        int cpuId = mapbox::util::apply_visitor(VariantToIntVisitor(),
                                                findCpuId->second);

        auto directory = oemNamePath.parent_path().parent_path();
        std::vector<fs::path> inputPaths;
        if (!find_files(fs::path(directory),
                        R"(peci-.+/hwmon/hwmon\d+/temp\d+_input$)", inputPaths,
                        0))
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
            std::vector<thresholds::Threshold> sensorThresholds;
            std::string labelHead = label.substr(0, label.find(" "));
            if (!ParseThresholdsFromConfig(*sensorData, sensorThresholds,
                                           &labelHead))
            {
                continue;
            }
            if (!sensorThresholds.size())
            {
                if (!ParseThresholdsFromAttr(sensorThresholds, inputPathStr,
                                             CPUSensor::SENSOR_SCALE_FACTOR))
                {
                    continue;
                }
            }
            sensors[sensorName] = std::make_unique<CPUSensor>(
                inputPathStr, sensorObjectType, objectServer, dbusConnection,
                io, sensorName, std::move(sensorThresholds), *interfacePath);
            if (DEBUG)
                std::cout << "Mapped: " << inputPath << " to " << sensorName
                          << "\n";
        }
    }
}

void reloadOverlay(const std::string& overlay)
{
    boost::process::child c1(DT_OVERLAY, "-d", OVERLAY_DIR, "-r", overlay);
    c1.wait();
    if (c1.exit_code())
    {
        if (DEBUG)
        {
            std::cout << "DTOverlay unload error with file " << overlay
                      << ". error: " << c1.exit_code() << "\n";
        }

        /* fall through anyway */
    }

    boost::process::child c2(DT_OVERLAY, "-d", OVERLAY_DIR, overlay);
    c2.wait();
    if (c2.exit_code())
    {
        std::cerr << "DTOverlay load error with file " << overlay
                  << ". error: " << c2.exit_code() << "\n";
        return;
    }
}

void detectCpu(boost::asio::deadline_timer& timer, boost::asio::io_service& io,
               sdbusplus::asio::object_server& objectServer,
               boost::container::flat_map<std::string,
                                          std::unique_ptr<CPUSensor>>& sensors,
               boost::container::flat_set<CPUConfig>& configs,
               std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    auto file = open(PECI_DEV, O_RDWR);
    if (file < 0)
    {
        std::cerr << "unable to open " << PECI_DEV << "\n";
        std::exit(EXIT_FAILURE);
    }

    size_t rescanDelaySeconds = 0;
    bool keepPinging = false;
    for (CPUConfig& config : configs)
    {
        State state;
        struct peci_ping_msg msg;
        msg.addr = config.addr;
        if (!ioctl(file, PECI_IOC_PING, &msg))
        {
            bool dimmReady = false;
            for (unsigned int rank = 0; rank < RANK_NUM_MAX; rank++)
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

        if (config.state != state)
        {
            if (config.state == State::OFF)
            {
                reloadOverlay(config.ovName);
            }
            if (state != State::OFF)
            {
                if (state == State::ON)
                {
                    rescanDelaySeconds = 1;
                }
                else
                {
                    rescanDelaySeconds = 5;
                }
            }
            config.state = state;
        }

        if (state != State::READY)
        {
            keepPinging = true;
        }

        if (DEBUG)
            std::cout << config.ovName << ", state: " << state << "\n";
    }

    close(file);

    if (rescanDelaySeconds)
    {
        std::this_thread::sleep_for(std::chrono::seconds(rescanDelaySeconds));
        createSensors(io, objectServer, sensors, configs, dbusConnection);
    }

    if (keepPinging)
    {
        timer.expires_from_now(boost::posix_time::seconds(1));
        timer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                /* we were canceled*/
                return;
            }
            else if (ec)
            {
                std::cerr << "timer error\n";
                return;
            }
            detectCpu(timer, io, objectServer, sensors, configs,
                      dbusConnection);
        });
    }
}

void getCpuConfig(const std::shared_ptr<sdbusplus::asio::connection>& systemBus,
                  boost::container::flat_set<CPUConfig>& configs)
{
    ManagedObjectType sensorConfigurations;
    bool useCache = false;
    // use new data the first time, then refresh
    for (const char* type : SENSOR_TYPES)
    {
        if (!getSensorConfiguration(CONFIG_PREFIX + std::string(type),
                                    systemBus, sensorConfigurations, useCache))
        {
            std::cerr
                << "getCpuConfig: error communicating to entity manager\n";
            return;
        }
        useCache = true;
    }

    // check PECI client addresses and DT overlay names from CPU configuration
    // before starting ping operation
    for (const char* type : SENSOR_TYPES)
    {
        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigurations)
        {
            for (const std::pair<
                     std::string,
                     boost::container::flat_map<std::string, BasicVariantType>>&
                     config : sensor.second)
            {
                if ((CONFIG_PREFIX + std::string(type)) != config.first)
                {
                    continue;
                }

                auto findAddress = config.second.find("Address");
                if (findAddress == config.second.end())
                {
                    continue;
                }
                std::string addrStr = mapbox::util::apply_visitor(
                    VariantToStringVisitor(), findAddress->second);
                int addr = std::stoi(addrStr, 0, 16);

                auto findName = config.second.find("Name");
                if (findName == config.second.end())
                {
                    continue;
                }
                std::string nameRaw = mapbox::util::apply_visitor(
                    VariantToStringVisitor(), findName->second);
                std::string name =
                    std::regex_replace(nameRaw, ILLEGAL_NAME_REGEX, "_");
                std::string overlayName = name + "_" + type;

                if (DEBUG)
                {
                    std::cout << "addr: " << addr << "\n";
                    std::cout << "name: " << name << "\n";
                    std::cout << "type: " << type << "\n";
                    std::cout << "overlayName: " << overlayName << "\n";
                }

                configs.emplace(addr, overlayName, State::OFF);
            }
        }
    }
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
    getCpuConfig(systemBus, configs);
    if (configs.size())
    {
        detectCpu(pingTimer, io, objectServer, sensors, configs, systemBus);
    }

    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            // this implicitly cancels the timer
            filterTimer.expires_from_now(boost::posix_time::seconds(1));

            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    /* we were canceled*/
                    return;
                }
                else if (ec)
                {
                    std::cerr << "timer error\n";
                    return;
                }

                getCpuConfig(systemBus, configs);

                if (configs.size())
                {
                    detectCpu(pingTimer, io, objectServer, sensors, configs,
                              systemBus);
                }
            });
        };

    for (const char* type : SENSOR_TYPES)
    {
        auto match = std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*systemBus),
            "type='signal',member='PropertiesChanged',path_namespace='" +
                std::string(INVENTORY_PATH) + "',arg0namespace='" +
                CONFIG_PREFIX + type + "'",
            eventHandler);
        matches.emplace_back(std::move(match));
    }

    io.run();
}
