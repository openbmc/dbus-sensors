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
#include <NVMeMCTPContext.hpp>
#include <NVMeSensor.hpp>
#include <boost/asio/deadline_timer.hpp>

#include <optional>
#include <regex>

static constexpr const char* sensorType =
    "xyz.openbmc_project.Configuration.NVME1000";

static NVMEMap nvmeDeviceMap;

static constexpr bool debug = false;

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
#if HAVE_NVME_MI_MCTP
        std::make_shared<NVMeMCTPContext>(io, rootBus);
#else
        std::make_shared<NVMeBasicContext>(io, rootBus);
#endif
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
    for (const std::pair<sdbusplus::message::object_path, SensorData>& sensor :
         sensorConfigurations)
    {
        const SensorData& sensorData = sensor.second;
        const std::string& interfacePath = sensor.first.str;
        const std::pair<std::string, boost::container::flat_map<
                                         std::string, BasicVariantType>>*
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

        std::optional<int> busNumber =
            extractBusNumber(sensor.first, baseConfiguration->second);
        if (!busNumber)
        {
            continue;
        }

        std::optional<std::string> sensorName =
            extractSensorName(sensor.first, baseConfiguration->second);
        if (!sensorName)
        {
            continue;
        }

        std::vector<thresholds::Threshold> sensorThresholds;
        if (!parseThresholdsFromConfig(sensorData, sensorThresholds))
        {
            std::cerr << "error populating thresholds for " << *sensorName
                      << "\n";
        }

        std::optional<int> rootBus = deriveRootBus(busNumber);
        if (!rootBus)
        {
            continue;
        }

        std::shared_ptr<NVMeSensor> sensorPtr = std::make_shared<NVMeSensor>(
            objectServer, io, dbusConnection, *sensorName,
            std::move(sensorThresholds), interfacePath, *busNumber);

        std::shared_ptr<NVMeContext> context =
            provideRootBusContext(io, nvmeDeviceMap, *rootBus);
        context->addSensor(sensorPtr);
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
        dbusConnection,
        std::move([&io, &objectServer, &dbusConnection](
                      const ManagedObjectType& sensorConfigurations) {
            handleSensorConfigurations(io, objectServer, dbusConnection,
                                       sensorConfigurations);
        }));
    getter->getConfiguration(std::vector<std::string>{sensorType});
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.NVMeSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
#if HAVE_NVME_MI_MCTP
    nvmeMCTP::init();
#endif

    io.post([&]() { createSensors(io, objectServer, systemBus); });

    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&filterTimer, &io, &objectServer,
         &systemBus](sdbusplus::message::message&) {
            // this implicitly cancels the timer
            filterTimer.expires_from_now(boost::posix_time::seconds(1));

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

    sdbusplus::bus::match::match configMatch(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" +
            std::string(sensorType) + "'",
        eventHandler);

    setupManufacturingModeMatch(*systemBus);
    io.run();
}
