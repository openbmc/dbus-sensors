/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpRequester.hpp"
#include "Utils.hpp"

#include <NvidiaDeviceDiscovery.hpp>
#include <NvidiaPcieDevice.hpp>
#include <NvidiaSmaDevice.hpp>
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
#include <cstdlib>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>> gpuDevices;
boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>> smaDevices;
boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>
    pcieDevices;

void configTimerExpiryCallback(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester, const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted)
    {
        return; // we're being canceled
    }
    createSensors(io, objectServer, gpuDevices, smaDevices, pcieDevices,
                  dbusConnection, mctpRequester);
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
        createSensors(io, objectServer, gpuDevices, smaDevices, pcieDevices,
                      systemBus, mctpRequester);
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
    std::array<std::string_view, 1> deviceTypes({deviceType});
    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(*systemBus, deviceTypes, eventHandler);

    // Watch for entity-manager to remove configuration interfaces
    // so the corresponding sensors can be removed.
    auto ifaceRemovedMatch = std::make_shared<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        sdbusplus::bus::match::rules::interfacesRemovedAtPath(
            std::string(inventoryPath)),
        [](sdbusplus::message_t& msg) {
            interfaceRemoved(msg, gpuDevices, smaDevices, pcieDevices);
        });

    try
    {
        io.run();
    }
    catch (const std::exception& e)
    {
        lg2::error("fatal error caught during processing: {MSG}", "MSG",
                   e.what());
        return EXIT_FAILURE;
    }

    return 0;
}
