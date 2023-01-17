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

#include <NVMeBasicContext.hpp>
#include <NVMeContext.hpp>
#include <NVMeSensor.hpp>
#include <boost/asio/steady_timer.hpp>

#include <optional>
#include <regex>

static NVMEMap nvmeDeviceMap;

NVMEMap& getNVMEMap()
{
    return nvmeDeviceMap;
}

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

static std::optional<std::string>
    extractSensorName(const std::string& path,
                      const SensorBaseConfigMap& properties)
{
    auto findSensorName = properties.find("Name");
    if (findSensorName == properties.end())
    {
        std::cerr << "could not determine configuration name for " << path
                  << "\n";
        return std::nullopt;
    }

    return std::get<std::string>(findSensorName->second);
}

static std::filesystem::path deriveRootBusPath(int busNumber)
{
    return "/sys/bus/i2c/devices/i2c-" + std::to_string(busNumber) +
           "/mux_device";
}

static std::optional<int> deriveRootBus(std::optional<int> busNumber)
{
    if (!busNumber)
    {
        return std::nullopt;
    }

    std::filesystem::path muxPath = deriveRootBusPath(*busNumber);

    if (!std::filesystem::is_symlink(muxPath))
    {
        return *busNumber;
    }

    std::string rootName = std::filesystem::read_symlink(muxPath).filename();
    size_t dash = rootName.find('-');
    if (dash == std::string::npos)
    {
        std::cerr << "Error finding root bus for " << rootName << "\n";
        return std::nullopt;
    }

    return std::stoi(rootName.substr(0, dash));
}

static std::shared_ptr<NVMeContext>
    provideRootBusContext(boost::asio::io_service& io, NVMEMap& map,
                          int rootBus)
{
    auto findRoot = map.find(rootBus);
    if (findRoot != map.end())
    {
        return findRoot->second;
    }

    std::shared_ptr<NVMeContext> context =
        std::make_shared<NVMeBasicContext>(io, rootBus);
    map[rootBus] = context;

    return context;
}

static void handleSensorConfigurations(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const ManagedObjectType& sensorConfigurations)
{
    // todo: it'd be better to only update the ones we care about
    for (const auto& [_, nvmeContextPtr] : nvmeDeviceMap)
    {
        if (nvmeContextPtr)
        {
            nvmeContextPtr->close();
        }
    }
    nvmeDeviceMap.clear();

    // iterate through all found configurations
    for (const auto& [interfacePath, sensorData] : sensorConfigurations)
    {
        // find base configuration
        auto sensorBase =
            sensorData.find(configInterfaceName(NVMeSensor::sensorType));
        if (sensorBase == sensorData.end())
        {
            continue;
        }

        const SensorBaseConfigMap& sensorConfig = sensorBase->second;
        std::optional<int> busNumber =
            extractBusNumber(interfacePath, sensorConfig);
        std::optional<std::string> sensorName =
            extractSensorName(interfacePath, sensorConfig);
        std::optional<int> rootBus = deriveRootBus(busNumber);

        if (!(busNumber && sensorName && rootBus))
        {
            continue;
        }

        std::vector<thresholds::Threshold> sensorThresholds;
        if (!parseThresholdsFromConfig(sensorData, sensorThresholds))
        {
            std::cerr << "error populating thresholds for " << *sensorName
                      << "\n";
        }

        try
        {
            // May throw for an invalid rootBus
            std::shared_ptr<NVMeContext> context =
                provideRootBusContext(io, nvmeDeviceMap, *rootBus);

            // Construct the sensor after grabbing the context so we don't
            // glitch D-Bus May throw for an invalid busNumber
            std::shared_ptr<NVMeSensor> sensorPtr =
                std::make_shared<NVMeSensor>(
                    objectServer, io, dbusConnection, *sensorName,
                    std::move(sensorThresholds), interfacePath, *busNumber);

            context->addSensor(sensorPtr);
        }
        catch (const std::invalid_argument& ex)
        {
            std::cerr << "Failed to add sensor for "
                      << std::string(interfacePath) << ": " << ex.what()
                      << "\n";
        }
    }
    for (const auto& [_, context] : nvmeDeviceMap)
    {
        context->pollNVMeDevices();
    }
}

void createSensors(boost::asio::io_service& io,
                   sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{

    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection, [&io, &objectServer, &dbusConnection](
                            const ManagedObjectType& sensorConfigurations) {
            handleSensorConfigurations(io, objectServer, dbusConnection,
                                       sensorConfigurations);
        });
    getter->getConfiguration(std::vector<std::string>{NVMeSensor::sensorType});
}

static void interfaceRemoved(sdbusplus::message_t& message, NVMEMap& contexts)
{
    if (message.is_method_error())
    {
        std::cerr << "interfacesRemoved callback method error\n";
        return;
    }

    sdbusplus::message::object_path path;
    std::vector<std::string> interfaces;

    message.read(path, interfaces);

    for (auto& [_, context] : contexts)
    {
        std::optional<std::shared_ptr<NVMeSensor>> sensor =
            context->getSensorAtPath(path);
        if (!sensor)
        {
            continue;
        }

        auto interface = std::find(interfaces.begin(), interfaces.end(),
                                   (*sensor)->objectType);
        if (interface == interfaces.end())
        {
            continue;
        }

        context->removeSensor(sensor.value());
    }
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.NVMeSensor");
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");

    io.post([&]() { createSensors(io, objectServer, systemBus); });

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

            createSensors(io, objectServer, systemBus);
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
        interfaceRemoved(msg, nvmeDeviceMap);
        });

    setupManufacturingModeMatch(*systemBus);
    io.run();
}
