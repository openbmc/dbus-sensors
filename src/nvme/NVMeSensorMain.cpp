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

#include "NVMeBasicContext.hpp"
#include "NVMeContext.hpp"
#include "NVMeSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

static constexpr uint8_t nvmeMiDefaultSlaveAddr = 0x6A;

static NVMEMap nvmeDeviceMap;

NVMEMap& getNVMEMap()
{
    return nvmeDeviceMap;
}

static std::optional<int> extractBusNumber(
    const std::string& path, const SensorBaseConfigMap& properties)
{
    auto findBus = properties.find("Bus");
    if (findBus == properties.end())
    {
        lg2::error("could not determine bus number for '{PATH}'", "PATH", path);
        return std::nullopt;
    }

    return std::visit(VariantToIntVisitor(), findBus->second);
}

static uint8_t extractSlaveAddr(const std::string& path,
                                const SensorBaseConfigMap& properties)
{
    auto findSlaveAddr = properties.find("Address");
    if (findSlaveAddr == properties.end())
    {
        lg2::error(
            "could not determine slave address for '{PATH} 'using default as "
            "specified in nvme-mi",
            "PATH", path);
        return nvmeMiDefaultSlaveAddr;
    }

    return std::visit(VariantToUnsignedIntVisitor(), findSlaveAddr->second);
}

static std::optional<std::string> extractSensorName(
    const std::string& path, const SensorBaseConfigMap& properties)
{
    auto findSensorName = properties.find("Name");
    if (findSensorName == properties.end())
    {
        lg2::error("could not determine configuration name for '{PATH}'",
                   "PATH", path);
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
        return busNumber;
    }

    std::string rootName = std::filesystem::read_symlink(muxPath).filename();
    size_t dash = rootName.find('-');
    if (dash == std::string::npos)
    {
        lg2::error("Error finding root bus for '{NAME}'", "NAME", rootName);
        return std::nullopt;
    }

    return std::stoi(rootName.substr(0, dash));
}

static std::shared_ptr<NVMeContext> provideRootBusContext(
    boost::asio::io_context& io, NVMEMap& map, int rootBus)
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
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
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
        uint8_t slaveAddr = extractSlaveAddr(interfacePath, sensorConfig);
        std::optional<int> rootBus = deriveRootBus(busNumber);

        if (!(busNumber && sensorName && rootBus))
        {
            continue;
        }

        std::vector<thresholds::Threshold> sensorThresholds;
        if (!parseThresholdsFromConfig(sensorData, sensorThresholds))
        {
            lg2::error("error populating thresholds for '{NAME}'", "NAME",
                       *sensorName);
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
                    std::move(sensorThresholds), interfacePath, *busNumber,
                    slaveAddr);

            context->addSensor(sensorPtr);
        }
        catch (const std::invalid_argument& ex)
        {
            lg2::error("Failed to add sensor for '{PATH}': '{ERROR}'", "PATH",
                       interfacePath.str, "ERROR", ex);
        }
    }
    for (const auto& [_, context] : nvmeDeviceMap)
    {
        context->pollNVMeDevices();
    }
}

void createSensors(boost::asio::io_context& io,
                   sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection, [&io, &objectServer, &dbusConnection](
                            const ManagedObjectType& sensorConfigurations) {
            handleSensorConfigurations(io, objectServer, dbusConnection,
                                       sensorConfigurations);
        });
    static constexpr std::array<std::string_view, 1> sensorTypes{
        {NVMeSensor::sensorType}};
    getter->getConfiguration(sensorTypes);
}

static void interfaceRemoved(sdbusplus::message_t& message, NVMEMap& contexts)
{
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
                                   (*sensor)->configInterface);
        if (interface == interfaces.end())
        {
            continue;
        }

        context->removeSensor(sensor.value());
    }
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.NVMeSensor");
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");

    boost::asio::post(io,
                      [&]() { createSensors(io, objectServer, systemBus); });

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
                    lg2::error("Error: '{ERROR_MESSAGE}'", "ERROR_MESSAGE",
                               ec.message());
                    return;
                }

                createSensors(io, objectServer, systemBus);
            });
        };

    static constexpr std::array<std::string_view, 1> sensorTypes{
        {NVMeSensor::sensorType}};

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(*systemBus, sensorTypes, eventHandler);

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
