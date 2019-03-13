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

std::vector<int> rootBuses = {2};

using MctpDeviceData = std::shared_ptr<struct nvmeContext>;
using SensorInfo = std::unique_ptr<NVMeSensor>;
using NvmeDeviceList = boost::container::flat_map<
    int, std::pair<SensorInfo, MctpDeviceData>>; // bus, <sensorinfo, mctp>

boost::container::flat_map<int, NvmeDeviceList>
    deviceRootBusMap; // root bus, map of busses containing drives

boost::container::flat_map<
    int, std::pair<struct mctp_binding_smbus*,
                   std::shared_ptr<boost::asio::ip::tcp::socket>>>
    rootBusFileDescriptors; // bus , fd

std::pair<int, int> lastQueriedDevice; // device was on this rootbus, bus

void createInboundFD(boost::asio::io_service& io)
{
    for (auto bus : rootBuses)
    {
        auto nvmeSlaveSocket =
            std::make_shared<boost::asio::ip::tcp::socket>(io);
        struct mctp_binding_smbus* smbus = mctp_smbus_init();

        int r = mctp_smbus_open_root_bus(smbus, bus);
        nvmeSlaveSocket->assign(boost::asio::ip::tcp::v4(),
                                mctp_smbus_get_in_fd(smbus));
        rootBusFileDescriptors[bus] = std::make_pair(smbus, nvmeSlaveSocket);
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
    NvmeDeviceList nvmeDeviceList;

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
        auto directory = path.parent_path();
        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        const std::pair<std::string, boost::container::flat_map<
                                         std::string, BasicVariantType>>*
            baseConfiguration;

        std::string busPath = fs::read_symlink(path);

        std::regex_search(busPath, match, busRegex);
        std::string busStr = *(match.begin());
        size_t bus = std::stoul(busStr);

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

            if (busNumber == 2)
                busNumber = 43;

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

        nvmeDeviceList[bus] = std::make_pair<SensorInfo, MctpDeviceData>(
            std::move(std::make_unique<NVMeSensor>(
                path.string(), objectServer, dbusConnection, io, sensorName,
                std::move(sensorThresholds), *interfacePath, bus)),
            std::make_shared<struct nvmeContext>(
                io, bus, rootBusFileDescriptors.find(2)->first));
    }
    deviceRootBusMap[2] = std::move(nvmeDeviceList);
}

// std::pair<NVMeSensor&, struct nvmeContext&> getDeviceInfoToQuery()
void getDeviceInfoToQuery()
{
    typename boost::container::flat_map<
        int, std::pair<SensorInfo, MctpDeviceData>>::iterator nextIt;
    typename boost::container::flat_map<int, NvmeDeviceList>::iterator
        nextRootBusIt;

    auto rootBusIt = deviceRootBusMap.find(lastQueriedDevice.first);
    if (rootBusIt != deviceRootBusMap.end())
    {
        nextRootBusIt = std::next(rootBusIt);
        if (deviceRootBusMap.find(nextRootBusIt->first) !=
            deviceRootBusMap.end())
        {
            lastQueriedDevice.first = nextRootBusIt->first;
        }
        else
        {
            lastQueriedDevice.first = deviceRootBusMap.begin()->first;
        }
    }
    else
    {

        lastQueriedDevice.first = deviceRootBusMap.begin()->first;
    }

    auto busDeviceMap = deviceRootBusMap.find(lastQueriedDevice.first);

    auto It = busDeviceMap->second.find(lastQueriedDevice.second);
    if (It != busDeviceMap->second.end())
    {
        nextIt = std::next(It);
        if (busDeviceMap->second.find(nextIt->first) !=
            busDeviceMap->second.end())
        {
            lastQueriedDevice.second = nextIt->first;
        }
        else
        {
            lastQueriedDevice.second = busDeviceMap->second.begin()->first;
        }
    }
    else
    {
        lastQueriedDevice.second = busDeviceMap->second.begin()->first;
    }
}

void pollNVMeDevices(boost::asio::io_service& io,
                     boost::asio::deadline_timer& scanTimer,
                     boost::asio::deadline_timer& tcpResponseTimer)
{
    scanTimer.expires_from_now(boost::posix_time::seconds(3));
    scanTimer.async_wait([&io, &scanTimer, &tcpResponseTimer](
                             const boost::system::error_code& errorCode) {
        if (errorCode != boost::asio::error::operation_aborted)
        {
            getDeviceInfoToQuery();

            NVMeSensor& sensorInfo =
                *deviceRootBusMap.find(lastQueriedDevice.first)
                     ->second.find(lastQueriedDevice.second)
                     ->second.first;

            struct nvmeContext& nvmeDevice =
                *deviceRootBusMap.find(lastQueriedDevice.first)
                     ->second.find(lastQueriedDevice.second)
                     ->second.second;

            boost::asio::ip::tcp::socket& nvmeSlaveSocket =
                *(rootBusFileDescriptors.find(lastQueriedDevice.first)
                      ->second.second);

            readAndProcessNVMeSensor(sensorInfo, nvmeDevice, io,
                                     nvmeSlaveSocket, tcpResponseTimer);
        }

        pollNVMeDevices(io, scanTimer, tcpResponseTimer);
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

    boost::asio::deadline_timer tcpResponseTimer(io);

    io.post([&]() {
        createInboundFD(io);
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
                createInboundFD(io);
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

    io.post([&]() {
        lastQueriedDevice =
            std::make_pair(deviceRootBusMap.begin()->first,
                           deviceRootBusMap.begin()->second.begin()->first);
        pollNVMeDevices(io, scanTimer, tcpResponseTimer);
    });

    io.run();
}
