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

#include "NVMeSensor.hpp"

#include <boost/asio/deadline_timer.hpp>
#include <regex>

static std::string sensorType = "xyz.openbmc_project.Configuration.NVME1000";
static std::regex driveRegex(R"((Pcie|M2|U2)_Slot)");
static std::regex busRegex(R"(\w[^-]*$)");
static std::regex rootBusPattern(R"(.?\/?(\d+)-\d+)");

static NVMEList nvmeDeviceList;

static constexpr bool DEBUG = false;

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

    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        std::move([&io, &objectServer, &dbusConnection](
                      const ManagedObjectType& sensorConfigurations) {
            // iterate through all found configurations
            for (const std::pair<sdbusplus::message::object_path, SensorData>&
                     sensor : sensorConfigurations)
            {

                const SensorData& sensorData = sensor.second;
                const std::string& interfacePath = sensor.first.str;
                const std::pair<
                    std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
                    baseConfiguration;

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

                auto findSensorName = baseConfiguration->second.find("Name");
                if (findSensorName == baseConfiguration->second.end())
                {
                    std::cerr << "could not determine configuration name for "
                              << interfacePath << "\n";
                    continue;
                }
                std::string sensorName =
                    std::get<std::string>(findSensorName->second);

                std::vector<thresholds::Threshold> sensorThresholds;

                if (!parseThresholdsFromConfig(sensorData, sensorThresholds))
                {
                    std::cerr << "error populating thresholds for "
                              << sensorName << "\n";
                }

                int rootBus = 0;

                std::string muxPath = "/sys/bus/i2c/devices/i2c-" +
                                      std::to_string(busNumber) + "/mux_device";

                if (!std::filesystem::is_symlink(muxPath))
                {
                    std::cerr << "Error finding root bus " << muxPath << "\n";
                }

                std::string rootName =
                    std::filesystem::read_symlink(muxPath).filename();
                size_t dash = rootName.find("-");
                if (dash == std::string::npos)
                {
                    std::cerr << "Error finding root bus for " << rootName
                              << "\n";
                    continue;
                }
                rootBus = std::stoi(rootName.substr(0, dash));

                std::shared_ptr<NVMeSensor> sensorPtr =
                    std::make_shared<NVMeSensor>(
                        objectServer, io, dbusConnection, sensorName,
                        std::move(sensorThresholds), interfacePath);

                nvmeDeviceList.emplace_back(
                    sensorPtr, std::make_shared<NVMeContext>(
                                   io, busNumber, rootBus, sensorPtr));

                if (DEBUG)
                {
                    printNvmeDeviceList();
                }
            }
        }));
    getter->getConfiguration(std::vector<std::string>{sensorType});
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

    io.post([&]() { pollNVMeDevices(io, scanTimer, nvmeDeviceList); });

    io.run();
}
