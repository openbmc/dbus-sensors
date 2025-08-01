/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpRequester.hpp"
#include "Utils.hpp"

#include <NvidiaDeviceDiscovery.hpp>
#include <NvidiaSmaDevice.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
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

boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>> gpuDevices;
boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>> smaDevices;

void configTimerExpiryCallback(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester, const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted)
    {
        return; // we're being canceled
    }
    createSensors(io, objectServer, gpuDevices, smaDevices, dbusConnection,
                  mctpRequester);
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    objectServer.add_manager("/xyz/openbmc_project/inventory");
    systemBus->request_name("xyz.openbmc_project.GpuSensor");

    mctp::MctpRequester mctpRequester(io);

    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, gpuDevices, smaDevices, systemBus,
                      mctpRequester);
    });

    boost::asio::steady_timer configTimer(io);

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&configTimer, &io, &objectServer, &systemBus,
         &mctpRequester](sdbusplus::message_t&) {
            configTimer.expires_after(std::chrono::seconds(1));
            // create a timer because normally multiple properties change
            configTimer.async_wait(std::bind_front(
                configTimerExpiryCallback, std::ref(io), std::ref(objectServer),
                std::ref(systemBus), std::ref(mctpRequester)));
        };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(
            *systemBus, std::to_array<const char*>({deviceType}), eventHandler);

    // Watch for entity-manager to remove configuration interfaces
    // so the corresponding sensors can be removed.
    auto ifaceRemovedMatch = std::make_shared<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        sdbusplus::bus::match::rules::interfacesRemovedAtPath(
            std::string(inventoryPath)),
        [](sdbusplus::message_t& msg) {
            interfaceRemoved(msg, gpuDevices, smaDevices);
        });

    io.run();
    return 0;
}
