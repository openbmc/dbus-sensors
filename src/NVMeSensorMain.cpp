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

#include "NVMeBasic.hpp"
#include "NVMeSubsys.hpp"

#include <boost/asio/steady_timer.hpp>

#include <optional>
#include <regex>

// a map with key value of {path, NVMeSubsystem}
using NVMEMap = std::map<std::string, std::shared_ptr<NVMeSubsystem>>;
static NVMEMap nvmeSubsysMap;

static std::optional<int>
    extractBusNumber(const std::string& path,
                     const SensorBaseConfigMap& properties)
{
    auto findBus = properties.find("Bus");
    if (findBus == properties.end())
    {
        std::cerr << "could not determine bus number for " << path << "\n";
        return std::nullopt;
    }

    return std::visit(VariantToIntVisitor(), findBus->second);
}

static std::optional<int> extractAddress(const std::string& path,
                                         const SensorBaseConfigMap& properties)
{
    auto findAddr = properties.find("Address");
    if (findAddr == properties.end())
    {
        std::cerr << "could not determine address for " << path << "\n";
        return std::nullopt;
    }

    return std::visit(VariantToIntVisitor(), findAddr->second);
}

static std::optional<std::string>
    extractName(const std::string& path, const SensorBaseConfigMap& properties)
{
    auto findName = properties.find("Name");
    if (findName == properties.end())
    {
        std::cerr << "could not determine configuration name for " << path
                  << "\n";
        return std::nullopt;
    }

    return std::get<std::string>(findName->second);
}

static std::optional<std::string>
    extractProtocol(const std::string& path,
                    const SensorBaseConfigMap& properties)
{
    auto findProtocol = properties.find("Protocol");
    if (findProtocol == properties.end())
    {
        std::cerr << "could not determine nvme protocl for " << path << "\n";
        return std::nullopt;
    }
    return std::get<std::string>(findProtocol->second);
}

static void handleConfigurations(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const ManagedObjectType& nvmeConfigurations)
{
    // todo: it'd be better to only update the ones we care about
    for (const auto& [_, nvmeSubsys] : nvmeSubsysMap)
    {
        if (nvmeSubsys)
        {
            nvmeSubsys->stop();
        }
    }
    nvmeSubsysMap.clear();

    // iterate through all found configurations
    for (const auto& [interfacePath, configData] : nvmeConfigurations)
    {
        // find base configuration
        auto sensorBase = configData.find(configInterfaceName(NVMeSubsystem::sensorType));
        if (sensorBase == configData.end())
        {
            continue;
        }

        const SensorBaseConfigMap& sensorConfig = sensorBase->second;
        std::optional<int> busNumber =
            extractBusNumber(interfacePath, sensorConfig);
        std::optional<int> address =
            extractAddress(interfacePath, sensorConfig);
        std::optional<std::string> sensorName =
            extractName(interfacePath, sensorConfig);
        std::optional<std::string> nvmeProtocol =
            extractProtocol(interfacePath, sensorConfig);

        if (!(busNumber && sensorName))
        {
            continue;
        }

        // the default protocol is mi_basic
        if (!nvmeProtocol)
        {
            nvmeProtocol.emplace("mi_basic");
        }
        if (*nvmeProtocol == "mi_basic")
        {
            // defualt i2c basic port is 0x6a
            if (!address)
            {
                address.emplace(0x6a);
            }
            try
            {
                NVMeIntf nvmeBasic = 
                NVMeIntf::create<NVMeBasic>(io, *busNumber, *address);

                auto nvmeSubsys = std::make_shared<NVMeSubsystem>(
                    io, objectServer, dbusConnection, interfacePath,
                    *sensorName, configData, std::move(nvmeBasic));
                nvmeSubsysMap.emplace(interfacePath, nvmeSubsys);
                nvmeSubsys->start();
            }
            catch (std::exception& ex)
            {
                std::cerr << "Failed to add subsystem for "
                          << std::string(interfacePath) << ": " << ex.what()
                          << "\n";
                continue;
            }
        }
    }
}

void createNVMeSubsystems(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{

    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection, [&io, &objectServer, &dbusConnection](
                            const ManagedObjectType& nvmeConfigurations) {
            handleConfigurations(io, objectServer, dbusConnection,
                                 nvmeConfigurations);
        });
    getter->getConfiguration(
        std::vector<std::string>{NVMeSubsystem::sensorType});
}

static void interfaceRemoved(sdbusplus::message_t& message, NVMEMap& subsystems)
{
    if (message.is_method_error())
    {
        std::cerr << "interfacesRemoved callback method error\n";
        return;
    }

    sdbusplus::message::object_path path;
    std::vector<std::string> interfaces;

    message.read(path, interfaces);

    auto interface = std::find(interfaces.begin(), interfaces.end(),
                               configInterfaceName(NVMeSubsystem::sensorType));
    if (interface == interfaces.end())
    {
        return;
    }

    auto subsys = subsystems.find(path);
    if (subsys == subsystems.end())
    {
        return;
    }

    subsys->second->stop();
    nvmeSubsysMap.erase(subsys);
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.NVMe");
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");

    io.post([&]() { createNVMeSubsystems(io, objectServer, systemBus); });

    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&filterTimer, &io, &objectServer, &systemBus](sdbusplus::message_t&) {
        // this implicitly cancels the timer
        filterTimer.expires_after(std::chrono::seconds(1));

        filterTimer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }

            if (ec)
            {
                std::cerr << "Error: " << ec.message() << "\n";
                return;
            }

            createNVMeSubsystems(io, objectServer, systemBus);
        });
    };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(
            *systemBus, std::to_array<const char*>({NVMeSensor::sensorType}),
            eventHandler);

    // Watch for entity-manager to remove configuration interfaces
    // so the corresponding sensors can be removed.
    auto ifaceRemovedMatch = std::make_unique<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        "type='signal',member='InterfacesRemoved',arg0path='" +
            std::string(inventoryPath) + "/'",
        [](sdbusplus::message_t& msg) {
        interfaceRemoved(msg, nvmeSubsysMap);
        });

    setupManufacturingModeMatch(*systemBus);
    io.run();
}
