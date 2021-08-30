/*
// Copyright (c) 2021 Arm Ltd.
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
#include <getopt.h>

#include <PLDMSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

extern "C"
{
#include <libpldm/base.h>
#include <libpldm/platform.h>
#include <libpldm/pldm.h>
}

constexpr const bool debug = false;
constexpr const uint8_t localEidDefault = 8;
static constexpr float pollRateDefault = 1; // in seconds
static constexpr double resolutionDefault = 1;
static constexpr double offsetDefault = 0;
constexpr const char* configInterfacePldmNumericSensor =
    "xyz.openbmc_project.Configuration.PldmNumericSensor";
boost::container::flat_map<std::string, std::unique_ptr<PLDMSensor>> sensors;

void createSensors(
    boost::asio::io_service& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<PLDMSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    uint8_t instanceId)
{
    if (!dbusConnection)
    {
        std::cerr << "Connection not created\n";
        return;
    }

    dbusConnection->async_method_call(
        [&io, &objectServer, &dbusConnection, &sensors, instanceId](
            boost::system::error_code ec, const ManagedObjectType& resp) {
            if (ec)
            {
                std::cerr << "Error contacting entity manager\n";
                return;
            }

            for (const auto& pathPair : resp)
            {
                for (const auto& entry : pathPair.second)
                {
                    if (entry.first != configInterfacePldmNumericSensor)
                    {
                        continue;
                    }

                    float pollRate = pollRateDefault;
                    auto findPollRate = entry.second.find("PollRate");
                    if (findPollRate != entry.second.end())
                    {
                        pollRate = std::visit(VariantToFloatVisitor(),
                                              findPollRate->second);
                        if (pollRate <= 0.0f)
                        {
                            pollRate = pollRateDefault;
                        }
                    }

                    float resolution = resolutionDefault;
                    auto findResolution = entry.second.find("Resolution");
                    if (findResolution != entry.second.end())
                    {
                        resolution = std::visit(VariantToFloatVisitor(),
                                                findResolution->second);
                        if (resolution <= 0.0f)
                        {
                            resolution = resolutionDefault;
                        }
                    }

                    float offset = offsetDefault;
                    auto findOffset = entry.second.find("Offset");
                    if (findOffset != entry.second.end())
                    {
                        offset = std::visit(VariantToFloatVisitor(),
                                            findOffset->second);
                        if (offset <= 0.0f)
                        {
                            offset = offsetDefault;
                        }
                    }

                    std::string configPath = pathPair.first;
                    std::string configInf = entry.first;
                    std::string name =
                        loadVariant<std::string>(entry.second, "Name");
                    uint8_t eid = loadVariant<uint8_t>(entry.second, "Eid");
                    uint16_t sensorId =
                        loadVariant<uint16_t>(entry.second, "SensorId");
                    std::string sensorUnit =
                        loadVariant<std::string>(entry.second, "SensorUnit");
                    std::vector<thresholds::Threshold> sensorThresholds;
                    if (!parseThresholdsFromConfig(pathPair.second,
                                                   sensorThresholds))
                    {
                        std::cerr << "error populating thresholds for " << name
                                  << "\n";
                    }

                    if constexpr (debug)
                    {
                        std::cerr << "Configuration parsed for \n\t"
                                  << entry.first << "\n"
                                  << "with\n"
                                  << "\tName: " << name << "\n"
                                  << "\tEid: " << static_cast<int>(eid) << "\n"
                                  << "\tResolution: " << resolution << "\n"
                                  << "\tOffset: " << offset << "\n"
                                  << "\n";
                    }

                    int32_t temp;
                    uint8_t dataSize;
                    if (!getSensorReading(eid, instanceId, sensorId, &temp,
                                          &dataSize))
                    {
                        std::cerr << "failed to communicate to sensor " << name
                                  << "\n";
                        continue;
                    }
                    double maxValue;
                    double minValue;
                    switch (dataSize)
                    {
                        default:
                        case PLDM_SENSOR_DATA_SIZE_UINT8:
                            maxValue = std::numeric_limits<uint8_t>::max();
                            minValue = std::numeric_limits<uint8_t>::min();
                            break;
                        case PLDM_SENSOR_DATA_SIZE_SINT8:
                            maxValue = std::numeric_limits<int8_t>::max();
                            minValue = std::numeric_limits<int8_t>::min();
                            break;
                        case PLDM_SENSOR_DATA_SIZE_UINT16:
                            maxValue = std::numeric_limits<uint16_t>::max();
                            minValue = std::numeric_limits<uint16_t>::min();
                            break;
                        case PLDM_SENSOR_DATA_SIZE_SINT16:
                            maxValue = std::numeric_limits<int16_t>::max();
                            minValue = std::numeric_limits<int16_t>::min();
                            break;
                        case PLDM_SENSOR_DATA_SIZE_UINT32:
                            maxValue = std::numeric_limits<uint32_t>::max();
                            minValue = std::numeric_limits<uint32_t>::min();
                            break;
                        case PLDM_SENSOR_DATA_SIZE_SINT32:
                            maxValue = std::numeric_limits<int32_t>::max();
                            minValue = std::numeric_limits<int32_t>::min();
                            break;
                    }
                    maxValue = maxValue * resolution + offset;
                    minValue = minValue * resolution + offset;

                    auto& sensor = sensors[name];
                    sensor = std::make_unique<PLDMSensor>(
                        dbusConnection, io, name, configPath, configInf,
                        objectServer, std::move(sensorThresholds), sensorUnit,
                        eid, sensorId, instanceId, pollRate, resolution, offset,
                        maxValue, minValue);
                    sensor->init();
                }
            }
        },
        entityManagerName, "/", "org.freedesktop.DBus.ObjectManager",
        "GetManagedObjects");
}

void usage(void)
{
    std::cerr << "Usage: pldmsensor [OPTIONS]\n";
    std::cerr << "Options:\n";
    std::cerr << "  --eid=<8-255> - Local EID, default=8\n"
              << "  --help - Print this help message and exit\n";
}

int main(int argc, char* const* argv)
{
    uint8_t localEid = localEidDefault;
    static const struct option options[] = {
        {"eid", optional_argument, nullptr, 'e'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0}};

    auto argflag = getopt_long(argc, argv, "e:h", options, nullptr);
    switch (argflag)
    {
        case 'e':
            localEid = strtol(optarg, nullptr, 0);
            break;
        case 'h':
            usage();
            return 0;
        default:
            break;
    }

    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.PldmSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    uint8_t instanceId = 0;

    try
    {
        sdbusplus::message::message getInstanceId = systemBus->new_method_call(
            "xyz.openbmc_project.PLDM", "/xyz/openbmc_project/pldm",
            "xyz.openbmc_project.PLDM.Requester", "GetInstanceId");

        getInstanceId.append(localEid);
        sdbusplus::message::message reply = systemBus->call(getInstanceId);
        reply.read(instanceId);
    }
    catch (const std::exception& e)
    {
        std::cerr << "GetInstanceId D-Bus call failed, error = " << e.what()
                  << "\n";
        return -ECOMM;
    }

    io.post([&]() {
        createSensors(io, objectServer, sensors, systemBus, instanceId);
    });

    boost::asio::deadline_timer configTimer(io);

    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message&) {
            configTimer.expires_from_now(boost::posix_time::seconds(1));
            // create a timer because normally multiple properties change
            configTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }
                // config timer error
                if (ec)
                {
                    std::cerr << "timer error\n";
                    return;
                }
                createSensors(io, objectServer, sensors, systemBus, instanceId);
                if (sensors.empty())
                {
                    std::cout << "Configuration not detected\n";
                }
            });
        };

    sdbusplus::bus::match::match configMatch(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" +
            configInterfacePldmNumericSensor + "'",
        eventHandler);

    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
