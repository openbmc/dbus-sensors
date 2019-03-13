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

#include "NVMeDeviceList.hpp"

#include <boost/asio/deadline_timer.hpp>
#include <regex>

static std::string sensorType = "xyz.openbmc_project.Configuration.NVME1000";
static std::regex driveRegex(R"((Pcie|M2|U2)_Slot)");
static std::regex busRegex(R"(\w[^-]*$)");
static std::regex rootBusPattern(R"(.?\/?(\d+)-\d+)");

void printNvmeDeviceList()
{
    for (auto i = 0; i < nvmeDeviceList.size(); i++)
    {
        std::cout << "-------------------------------"
                  << "\nBus: " << nvmeDeviceList[i].second->bus
                  << "\nRootBus: " << nvmeDeviceList[i].second->rootBus
                  << "\nNvmeSlaveSocket: "
                  << nvmeDeviceList[i].second->nvmeSlaveSocket << "\nIn_fd: "
                  << mctp_smbus_get_in_fd(nvmeDeviceList[i].second->smbus.get())
                  << "\n";
    }
}

void createSensors(boost::asio::io_service& io,
                   sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    ManagedObjectType sensorConfigurations;
    bool useCache = false;

    if (!getSensorConfiguration(sensorType, dbusConnection,
                                sensorConfigurations, useCache))
    {
        std::cerr << "Error communicating with Entity Manager";
        return;
    }
    useCache = true;

    std::vector<std::filesystem::path> paths;
    if (!findFiles(std::filesystem::path("/dev/i2c-mux"),
                   R"((Pcie|M2|U2)_Slot)", paths))
    {
        return;
    }

    // iterate through all found NVMe drives and match them with configuration
    for (auto& path : paths)
    {
        std::smatch match;
        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        const std::pair<std::string, boost::container::flat_map<
                                         std::string, BasicVariantType>>*
            baseConfiguration;

        std::string busPath = std::filesystem::read_symlink(path);

        std::regex_search(busPath, match, busRegex);

        if (match.empty())
        {
            continue;
        }

        std::string busStr = *(match.begin());
        size_t bus = std::stoul(busStr);
        int rootBus = 0;

        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigurations)
        {
            // clear it out each loop
            baseConfiguration = nullptr;

            // find base configuration
            auto sensorBase = sensor.second.find(sensorType);
            if (sensorBase != sensor.second.end())
            {
                baseConfiguration = &(*sensorBase);
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

        std::string rootPath =
            "/sys/bus/i2c/devices/i2c-" + std::to_string(bus) + "/mux_device";

        if (std::filesystem::exists(rootPath) &&
            std::filesystem::is_symlink(rootPath))
        {
            std::smatch match{};
            std::string search =
                std::filesystem::read_symlink(rootPath).string();
            if (std::regex_search(search, match, rootBusPattern))
            {
                rootBus = stoi(match.str(1));
            }
        }

        std::vector<thresholds::Threshold> sensorThresholds;
        std::string sensorName = std::get<std::string>(findSensorName->second);

        nvmeDeviceList.emplace_back(
            std::make_pair<std::unique_ptr<NVMeSensor>,
                           std::shared_ptr<struct NVMeContext>>(
                std::move(std::make_unique<NVMeSensor>(
                    objectServer, io, dbusConnection, sensorName,
                    std::move(sensorThresholds), *interfacePath)),
                std::make_shared<struct NVMeContext>(io, bus, rootBus)));

        if (DEBUG)
        {
            printNvmeDeviceList();
        }
    }
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.NVMeSensor");
    sdbusplus::asio::object_server objectServer(systemBus);

    io.post([&io, &objectServer, &systemBus]() {
        createSensors(io, objectServer, systemBus);
    });

    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&filterTimer, &io, &objectServer,
         &systemBus](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            // this implicitly cancels the timer
            filterTimer.expires_from_now(boost::posix_time::seconds(1));

            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec)
                {
                    std::cerr << "Error: " << ec.message() << "\n";
                    return;
                }

                createSensors(io, objectServer, systemBus);
            });
        };

    std::unique_ptr<sdbusplus::bus::match::match> match =
        std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*systemBus),
            "type='signal',member='PropertiesChanged',path_namespace='" +
                std::string(inventoryPath) + "',arg0namespace='" + sensorType +
                "'",
            eventHandler);

    boost::asio::deadline_timer scanTimer(io);

    io.post([&]() { pollNVMeDevices(io, scanTimer); });

    io.run();
}
