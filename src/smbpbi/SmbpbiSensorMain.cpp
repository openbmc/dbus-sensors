/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "SmbpbiSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

constexpr const char* configInterface =
    "xyz.openbmc_project.Configuration.SmbpbiVirtualEeprom";
constexpr const char* objectType = "SmbpbiVirtualEeprom";

boost::container::flat_map<std::string, std::unique_ptr<SmbpbiSensor>> sensors;

static void createSensorCallback(
    boost::system::error_code ec, const ManagedObjectType& resp,
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    boost::container::flat_map<std::string, std::unique_ptr<SmbpbiSensor>>&
        sensors)
{
    if (ec)
    {
        lg2::error("Error contacting entity manager");
        return;
    }
    for (const auto& pathPair : resp)
    {
        for (const auto& entry : pathPair.second)
        {
            if (entry.first != configInterface)
            {
                continue;
            }
            std::string name = loadVariant<std::string>(entry.second, "Name");

            std::vector<thresholds::Threshold> sensorThresholds;
            if (!parseThresholdsFromConfig(pathPair.second, sensorThresholds))
            {
                lg2::error("error populating thresholds for {NAME}", "NAME",
                           name);
            }

            uint8_t busId = loadVariant<uint8_t>(entry.second, "Bus");

            uint8_t addr = loadVariant<uint8_t>(entry.second, "Address");

            uint16_t off = loadVariant<uint16_t>(entry.second, "ReadOffset");

            PowerState pwrState = getPowerState(entry.second);

            std::string sensorUnits =
                loadVariant<std::string>(entry.second, "Units");

            std::string valueType =
                loadVariant<std::string>(entry.second, "ValueType");
            if (valueType != "UINT64" && valueType != "FLOAT64")
            {
                lg2::error("Invalid ValueType for sensor: {NAME}", "NAME",
                           name);
                break;
            }

            size_t rate = loadVariant<uint8_t>(entry.second, "PollRate");

            double minVal = loadVariant<double>(entry.second, "MinValue");

            double maxVal = loadVariant<double>(entry.second, "MaxValue");
            lg2::debug("Configuration parsed for \n\t {CONF}\nwith\n"
                       "\tName: {NAME}\n"
                       "\tBus: {BUS}\n"
                       "\tAddress:{ADDR}\n"
                       "\tOffset: {OFF}\n"
                       "\tType : {TYPE}\n"
                       "\tValue Type : {VALUETYPE}\n"
                       "\tPollrate: {RATE}\n"
                       "\tMinValue: {MIN}\n"
                       "\tMaxValue: {MAX}\n"
                       "\tPowerState: {PWRSTATE}\n",
                       "CONF", entry.first, "NAME", name, "BUS",
                       static_cast<int>(busId), "ADDR", static_cast<int>(addr),
                       "OFF", static_cast<int>(off), "UNITS", sensorUnits,
                       "VALUETYPE", valueType, "RATE", rate, "MIN", minVal,
                       "MAX", maxVal, "PWRSTATE", pwrState);

            auto& sensor = sensors[name];
            sensor = nullptr;

            std::string path = "/dev/i2c-" + std::to_string(busId);

            sensor = std::make_unique<SmbpbiSensor>(
                dbusConnection, io, name, pathPair.first, objectType,
                objectServer, std::move(sensorThresholds), busId, addr, off,
                sensorUnits, valueType, rate, minVal, maxVal, path, pwrState);

            sensor->init();
        }
    }
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::unique_ptr<SmbpbiSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    if (!dbusConnection)
    {
        lg2::error("Connection not created");
        return;
    }

    dbusConnection->async_method_call(
        [&io, &objectServer, &dbusConnection, &sensors](
            boost::system::error_code ec, const ManagedObjectType& resp) {
            createSensorCallback(ec, resp, io, objectServer, dbusConnection,
                                 sensors);
        },
        entityManagerName, "/xyz/openbmc_project/inventory",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    systemBus->request_name("xyz.openbmc_project.SMBPBI");

    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, sensors, systemBus);
    });

    boost::asio::steady_timer configTimer(io);

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t&) {
            configTimer.expires_after(std::chrono::seconds(1));
            // create a timer because normally multiple properties change
            configTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }
                // config timer error
                if (ec)
                {
                    lg2::error("timer error");
                    return;
                }
                createSensors(io, objectServer, sensors, systemBus);
                if (sensors.empty())
                {
                    lg2::info("Configuration not detected");
                }
            });
        };

    sdbusplus::match configMatch(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        "type='signal',member='PropertiesChanged',"
        "path_namespace='" +
            std::string(inventoryPath) +
            "',"
            "arg0namespace='" +
            configInterface + "'",
        eventHandler);

    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
