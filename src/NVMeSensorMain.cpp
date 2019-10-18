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

#include "NVMe.hpp"

#include <Utils.hpp>
#include <boost/asio/steady_timer.hpp>
#include <chrono>
#include <regex>

struct NVMeScanner
{
    NVMeScanner(boost::asio::io_service& io,
                sdbusplus::asio::object_server& objectServer,
                sdbusplus::asio::connection& conn) :
        io(io),
        objectServer(objectServer), conn(conn), scanTimer(io)
    {
    }

    std::vector<NVMeSensor> nvmeDeviceList;
    boost::asio::io_service& io;
    sdbusplus::asio::object_server& objectServer;
    sdbusplus::asio::connection& conn;
    boost::asio::steady_timer scanTimer(io);

    void printNvmeDeviceList()
    {
        for (const NVMeSensor& nvme : nvmeDeviceList)
        {
            std::cout << "-------------------------------"
                      << "\nBus: " << nvme.bus << "\nRootBus: " << nvme.rootBus
                      << "\nNvmeSlaveSocket: " << nvme.nvmeSlaveSocket
                      << "\nIn_fd: " << mctp_smbus_get_in_fd(nvme.smbus.get())
                      << "\n";
        }
    }

    void createSensors()
    {
        // if we're waiting for a scan, cancel the scanning while we rebuild the
        // list
        scanTimer.cancel();

        nvmeDeviceList.clear();

        std::string sensorType = "xyz.openbmc_project.Configuration.NVME1000";
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
        std::regex rootBusPattern(R"(.?\/?(\d+)-\d+)");
        // iterate through all found NVMe drives and match them with
        // configuration
        for (const std::filesystem::path& path : paths)
        {
            const SensorData* sensorData = nullptr;
            const std::string* interfacePath = nullptr;
            const std::pair<std::string, boost::container::flat_map<
                                             std::string, BasicVariantType>>*
                baseConfiguration;

            std::string busPath = std::filesystem::read_symlink(path);
            size_t dashIndex = busPath.find("-");
            if (dashIndex == std::string::npos)
            {
                continue;
            }
            dashIndex++;
            if (dashIndex >= busPath.size())
            {
                continue;
            }

            std::string busStr = busPath.substr(dashIndex);
            int bus = std::stoi(busStr);
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
                std::cerr << "failed to find match for " << path.string()
                          << "\n";
                continue;
            }

            if (baseConfiguration == nullptr)
            {
                std::cerr << "error finding base configuration for"
                          << path.string() << "\n";
                continue;
            }

            auto findSensorName = baseConfiguration->second.find("Name");
            if (findSensorName == baseConfiguration->second.end())
            {
                std::cerr << "could not determine configuration name for "
                          << path.string() << "\n";
                continue;
            }

            std::string rootPath = "/sys/bus/i2c/devices/i2c-" +
                                   std::to_string(bus) + "/mux_device";

            if (std::filesystem::exists(rootPath) &&
                std::filesystem::is_symlink(rootPath))
            {
                std::smatch match;
                std::string search =
                    std::filesystem::read_symlink(rootPath).string();
                if (std::regex_search(search, match, rootBusPattern))
                {
                    rootBus = stoi(match.str(1));
                }
            }

            std::string sensorName =
                std::get<std::string>(findSensorName->second);

            nvmeDeviceList.emplace_back(objectServer, io, dbusConnection,
                                        sensorName, bus, rootBus);

            if (DEBUG)
            {
                printNvmeDeviceList();
            }
        }

        // only need to start scanning if we have drives, otherwise we can ignore
        if (nvmeDeviceList.empty())){
            return;
        }
        scanTimer.expires_after(std::chrono::seconds(1));
        timer.async_wait(
            [this](const boost::system::error_code& error) { scanSensors(); });

    }


    void readAndProcessNVMeSensor()
    {
        // Arm a timeout timer in case the drive never replies
        scanTimer.expires_from_now(std::chrono::milliseconds(500));
        scanTimer.async_wait([slaveSocket{nvmeDevice.nvmeSlaveSocket}](
                                    const boost::system::error_code& ec) {
            if (ec)
            {
                return;
            }

            slaveSocket->cancel();
        });

        readResponse(nvmeDevice);

        if (DEBUG)
        {
            std::cout << "Sending message to read data from Drive on bus: "
                    << nvmeDevice.bus << " , rootBus: " << nvmeDevice.rootBus
                    << " device index:" << nvmeDevice.sindex << "\n";
        }

        struct nvme_mi_msg_request requestMsg = {};
        requestMsg.header.opcode = NVME_MI_OPCODE_HEALTH_STATUS_POLL;
        requestMsg.header.dword0 = 0;
        requestMsg.header.dword1 = 0;

        int rc = nvmeMessageTransmit(nvmeDevice.mctp, nvmeDevice.eid, &requestMsg);

        if (rc != 0)
        {
            std::cerr << "Error sending request message to NVMe device\n";
        }
    }

    void scanSensors()
    {

        scanTimer.expires_after(std::chrono::seconds(1));
        timer.async_wait(
            [this](const boost::system::error_code& error) { scanSensors(); });
    }
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.NVMeSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    NVMeScanner nvme(io, objectServer, *systemBus);
    nvme.createSensors();

    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> onReconfigureEvent =
        [&filterTimer,
         &nvme systemBus](sdbusplus::message::message& message) mutable {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            // this implicitly cancels the timer
            filterTimer.expires_from_now(std::chrono::seconds(1));

            filterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec)
                {
                    std::cerr << "Error: " << ec.message() << "\n";
                    return;
                }

                nvme.createSensors();
            });
        };

    sdbusplus::bus::match::match match{
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + sensorType + "'",
        onReconfigureEvent};

    io.run();
}
