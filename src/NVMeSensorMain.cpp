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
#include "NVMeMi.hpp"
#include "NVMeSubsys.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/asio/steady_timer.hpp>

#include <optional>
#include <regex>

// a map with key value of {path, NVMeSubsystem}
using NVMEMap = std::map<std::string, std::shared_ptr<NVMeSubsystem>>;
static NVMEMap nvmeSubsysMap;

// flag to set a single worker thread for all nvme eps under the same i2c bus
static bool singleThreadMode = false;

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

static PowerState extractPowerState(const std::string& path,
                                    const SensorBaseConfigMap& properties)
{
    auto find = properties.find("PowerState");
    if (find == properties.end())
    {
        std::cerr << "could not determine configuration of PowerState for "
                  << path << ", using default\n";
        // default to always
        return PowerState::always;
    }
    auto res = std::get<std::string>(find->second);
    if (boost::iequals(res, "on"))
    {
        return PowerState::on;
    }
    else if (boost::iequals(res, "biosPost"))
    {
        return PowerState::biosPost;
    }
    else if (boost::iequals(res, "always"))
    {
        return PowerState::always;
    }
    else if (boost::iequals(res, "chassisOn"))
    {
        return PowerState::chassisOn;
    }
    // default to always
    std::cerr << "could not determine config value for PowerState for " << path
              << ", using default\n";
    return PowerState::always;
}

static void handleConfigurations(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
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

    /* We perform two iterations for configurations here. The first iteration is
     * to set up NVMeIntf. The second iter is to setup NVMe subsystem.
     *
     * The reason to seperate these two processes is NVMeIntf initialization of
     * NVMeMI is via MCTPd, from which the mctp control msg should be relatively
     * short and should not be delayed by NVMe-MI protocol msg from NVMe
     * subsystem.
     */
    std::map<std::string, NVMeIntf> nvmeInterfaces;
    for (const auto& [interfacePath, configData] : nvmeConfigurations)
    {
        // find base configuration
        auto sensorBase =
            configData.find(configInterfaceName(NVMeSubsystem::sensorType));
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

                nvmeInterfaces.emplace(interfacePath, std::move(nvmeBasic));
            }
            catch (std::exception& ex)
            {
                std::cerr << "Failed to add nvme basic interface for "
                          << std::string(interfacePath) << ": " << ex.what()
                          << "\n";
                continue;
            }
        }
        else if (*nvmeProtocol == "mi_i2c")
        {
            // defualt i2c nvme-mi port is 0x1d
            if (!address)
            {
                address.emplace(0x1d);
            }

            PowerState powerState =
                extractPowerState(interfacePath, sensorConfig);

            try
            {
                NVMeIntf nvmeMi = NVMeIntf::create<NVMeMi>(
                    io, dbusConnection, *busNumber, *address, singleThreadMode,
                    powerState);

                nvmeInterfaces.emplace(interfacePath, nvmeMi);
            }
            catch (std::exception& ex)
            {
                std::cerr << "Failed to add nvme mi interface for "
                          << std::string(interfacePath) << ": " << ex.what()
                          << "\n";
                continue;
            }
        }
    }

    for (const auto& [interfacePath, configData] : nvmeConfigurations)
    {
        // find base configuration
        auto sensorBase =
            configData.find(configInterfaceName(NVMeSubsystem::sensorType));
        if (sensorBase == configData.end())
        {
            continue;
        }

        const SensorBaseConfigMap& sensorConfig = sensorBase->second;

        std::optional<std::string> sensorName =
            extractName(interfacePath, sensorConfig);

        auto find = nvmeInterfaces.find(interfacePath);
        if (find == nvmeInterfaces.end())
            continue;
        try
        {
            auto nvmeSubsys = std::make_shared<NVMeSubsystem>(
                io, objectServer, dbusConnection, interfacePath, *sensorName,
                configData, std::move(find->second));
            nvmeSubsysMap.emplace(interfacePath, nvmeSubsys);
            nvmeSubsys->start();
        }
        catch (std::exception& ex)
        {
            std::cerr << "Failed to add nvme subsystem for "
                      << std::string(interfacePath) << ": " << ex.what()
                      << "\n";
            continue;
        }
    }
}

void createNVMeSubsystems(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
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
    // TODO: set single thread mode according to input parameters

    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.NVMe");
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    objectServer.add_manager("/xyz/openbmc_project/inventory");

    io.post([&]() { createNVMeSubsystems(io, objectServer, systemBus); });

    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&filterTimer, &io, &objectServer, &systemBus](sdbusplus::message_t&) {
        // this implicitly cancels the timer
        filterTimer.expires_from_now(std::chrono::seconds(1));

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

    // The NVMe controller used pipe to transfer raw data. The pipe could be
    // closed by the client. It should not be considered as an error.
    boost::asio::signal_set signals(io, SIGPIPE);
    signals.async_wait(
        [](const boost::system::error_code& error, int signal_number) {
        std::cerr << "signal: " << strsignal(signal_number) << ", "
                  << error.message() << std::endl;
    });
    io.run();
}
