/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuSensor.hpp"
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

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

boost::container::flat_map<std::string, std::shared_ptr<GpuTempSensor>> sensors;

void configTimerExpiryCallback(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted)
    {
        return; // we're being canceled
    }
    createSensors(io, objectServer, sensors, dbusConnection);
    if (sensors.empty())
    {
        lg2::info("Configuration not detected");
    }
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    systemBus->request_name("xyz.openbmc_project.GpuSensor");

    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, sensors, systemBus);
    });

    boost::asio::steady_timer configTimer(io);

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&configTimer, &io, &objectServer, &systemBus](sdbusplus::message_t&) {
            configTimer.expires_after(std::chrono::seconds(1));
            // create a timer because normally multiple properties change
            configTimer.async_wait(
                std::bind_front(configTimerExpiryCallback, std::ref(io),
                                std::ref(objectServer), std::ref(systemBus)));
        };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(
            *systemBus, std::to_array<const char*>({sensorType}), eventHandler);

    // Watch for entity-manager to remove configuration interfaces
    // so the corresponding sensors can be removed.
    auto ifaceRemovedMatch = std::make_shared<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        sdbusplus::bus::match::rules::interfacesRemovedAtPath(
            std::string(inventoryPath)),
        [](sdbusplus::message_t& msg) { interfaceRemoved(msg, sensors); });

    io.run();
    return 0;
}
