/*
// Copyright (c) 2017 Intel Corporation
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

#include <NVMeSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>
#include <regex>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

static constexpr std::array<const char*, 1> sensorTypes = {
    "xyz.openbmc_project.Configuration.NVME1000"};
static std::regex driveRegex(R"((Pcie|M2|U2)_Slot)");
static std::regex busRegex(R"(\w[^-]*$)");

namespace fs = std::filesystem;

using MctpDeviceData = std::shared_ptr<struct NVMeContext>;
using SensorInfo = std::unique_ptr<NVMeSensor>;
std::vector<std::pair<SensorInfo, MctpDeviceData>> nvmeDeviceList;

size_t lastQueriedDeviceIndex;

void printNvmeDeviceList()
{
    for (auto it = nvmeDeviceList.begin(); it != nvmeDeviceList.end(); it++)
    {
        std::cout << "-------------------------------"
                  << "\nBus: " << it->second->bus
                  << "\nRootBus: " << it->second->rootBus
                  << "\nNvmeSlaveSocket: " << it->second->nvmeSlaveSocket
                  << "\nIn_fd: "
                  << mctp_smbus_get_in_fd(it->second->smbus.get()) << "\n";
    }
}

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<NVMeSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::unique_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    ManagedObjectType sensorConfigurations;
    bool useCache = false;

    for (const char* type : sensorTypes)
    {
        if (!getSensorConfiguration(type, dbusConnection, sensorConfigurations,
                                    useCache))
        {
            std::cerr << "Error communicating with Entity Manager";
            return;
        }
        useCache = true;
    }
    std::vector<fs::path> paths;
    if (!findFiles(fs::path("/dev/i2c-mux"), R"((Pcie|M2|U2)_Slot)", paths))
    {
        std::cerr << "No NVMe sensors in system\n";
        return;
    }

    // iterate through all found NVMe drives and match them with configuration
    for (auto& path : paths)
    {
        std::smatch match;
        std::string pathStr = path.string();
        fs::path directory = path.parent_path();
        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        const std::pair<std::string, boost::container::flat_map<
                                         std::string, BasicVariantType>>*
            baseConfiguration;

        std::string busPath = fs::read_symlink(path);

        std::regex_search(busPath, match, busRegex);
        std::string busStr = *(match.begin());
        size_t bus = std::stoul(busStr);
        int rootBus{};

        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigurations)
        {
            // clear it out each loop
            baseConfiguration = nullptr;

            // find base configuration
            for (const char* type : sensorTypes)
            {
                auto sensorBase = sensor.second.find(type);
                if (sensorBase != sensor.second.end())
                {
                    baseConfiguration = &(*sensorBase);
                    break;
                }
            }
            if (baseConfiguration == nullptr)
            {
                continue;
            }
            auto findBus = baseConfiguration->second.find("Bus");
            if (findBus == baseConfiguration->second.end())
            {
                continue;
            }

            unsigned int busNumber =
                std::visit(VariantToUnsignedIntVisitor(), findBus->second);

            if (busNumber != bus)
            {
                continue;
            }
            sensorData = &(sensor.second);
            interfacePath = &(sensor.first.str);
            break;
        }
        if (sensorData == nullptr)
        {
            std::cerr << "failed to find match for " << path.string() << "\n";
            continue;
        }

        if (baseConfiguration == nullptr)
        {
            std::cerr << "error finding base configuration for" << path.string()
                      << "\n";
            continue;
        }

        auto findSensorName = baseConfiguration->second.find("Name");
        if (findSensorName == baseConfiguration->second.end())
        {
            std::cerr << "could not determine configuration name for "
                      << path.string() << "\n";
            continue;
        }
        std::string sensorName = std::get<std::string>(findSensorName->second);

        std::vector<thresholds::Threshold> sensorThresholds;
        if (!parseThresholdsFromConfig(*sensorData, sensorThresholds))
        {
            std::cerr << "error populating thresholds for " << sensorName
                      << "\n";
        }

        std::string rootPath =
            "/sys/bus/i2c/devices/i2c-" + std::to_string(bus) + "/mux_device";

        if (fs::exists(rootPath) && fs::is_symlink(rootPath))
        {
            std::regex rootBusPattern(R"(.?\/?(\d+)-\d+)");
            std::smatch match{};

            std::string search = fs::read_symlink(rootPath).string();
            if (std::regex_search(search, match, rootBusPattern))
            {
                rootBus = stoi(match.str(1));
            }
        }

        nvmeDeviceList.emplace_back(std::make_pair<SensorInfo, MctpDeviceData>(
            std::move(std::make_unique<NVMeSensor>(
                path.string(), objectServer, dbusConnection, io, sensorName,
                std::move(sensorThresholds), *interfacePath, bus)),
            std::make_shared<struct NVMeContext>(io, bus, rootBus)));

        if (DEBUG)
        {
            printNvmeDeviceList();
        }
    }
}

void getDeviceInfoToQuery()
{
    if (!nvmeDeviceList.empty())
    {
        lastQueriedDeviceIndex++;

        if (lastQueriedDeviceIndex >= nvmeDeviceList.size())
        {
            lastQueriedDeviceIndex = 0;
        }
    }
}

void pollNVMeDevices(boost::asio::io_service& io,
                     boost::asio::deadline_timer& scanTimer,
                     boost::asio::deadline_timer& mctpResponseTimer)
{
    scanTimer.expires_from_now(boost::posix_time::seconds(3));
    scanTimer.async_wait([&io, &scanTimer, &mctpResponseTimer](
                             const boost::system::error_code& errorCode) {
        if (errorCode != boost::asio::error::operation_aborted)
        {
            if (!nvmeDeviceList.empty())
            {
                getDeviceInfoToQuery();
                NVMeSensor& sensorInfo =
                    *nvmeDeviceList[lastQueriedDeviceIndex].first;

                struct NVMeContext& nvmeDevice =
                    *nvmeDeviceList[lastQueriedDeviceIndex].second;

                if (&sensorInfo != NULL && &nvmeDevice != NULL)
                {
                    readAndProcessNVMeSensor(sensorInfo, nvmeDevice, io,
                                             mctpResponseTimer);
                }
            }
        }
        else if (errorCode)
        {
            std::cerr << "timer error\n";
        }

        pollNVMeDevices(io, scanTimer, mctpResponseTimer);
    });
}

int main(int argc, char** argv)
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.NVMeSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    boost::container::flat_map<std::string, std::unique_ptr<NVMeSensor>>
        sensors;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    std::unique_ptr<boost::container::flat_set<std::string>> sensorsChanged =
        std::make_unique<boost::container::flat_set<std::string>>();

    boost::asio::deadline_timer mctpResponseTimer(io);

    io.post([&]() {
        createSensors(io, objectServer, sensors, systemBus, nullptr);
    });

    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            sensorsChanged->insert(message.get_path());
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

                createSensors(io, objectServer, sensors, systemBus,
                              sensorsChanged);
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

    boost::asio::deadline_timer scanTimer(io);

    io.post([&]() { pollNVMeDevices(io, scanTimer, mctpResponseTimer); });

    io.run();
}
