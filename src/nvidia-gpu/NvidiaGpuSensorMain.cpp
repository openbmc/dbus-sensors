/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DeviceManager.hpp"
#include "MctpRequester.hpp"
#include "NvidiaSensorConfig.hpp"
#include "Utils.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/system/error_code.hpp>
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

// Debounce window to coalesce a burst of entity-manager config property
// changes (which normally fire several at once) into one rescan.
constexpr std::chrono::seconds configSettleInterval{1};

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    objectServer.add_manager("/xyz/openbmc_project/control");
    objectServer.add_manager("/xyz/openbmc_project/inventory");
    objectServer.add_manager("/xyz/openbmc_project/software");
    objectServer.add_manager("/xyz/openbmc_project/metric");
    systemBus->request_name("xyz.openbmc_project.GpuSensor");

    mctp::MctpRequester mctpRequester(io);

    DeviceManager deviceManager(io, objectServer, systemBus, mctpRequester);

    boost::asio::post(io, [&deviceManager]() { deviceManager.createSensors(); });

    boost::asio::steady_timer configTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&configTimer, &deviceManager](sdbusplus::message_t&) {
            // create a timer because normally multiple properties change at once
            configTimer.expires_after(configSettleInterval);
            configTimer.async_wait(
                [&deviceManager](const boost::system::error_code& ec) {
                    if (ec == boost::asio::error::operation_aborted)
                    {
                        return; // we're being canceled
                    }
                    deviceManager.createSensors();
                });
        };

    std::array<std::string_view, 1> sensorTypes({sensorType});
    auto configMatches =
        setupPropertiesChangedMatches(*systemBus, sensorTypes, eventHandler);

    // Watch for entity-manager to remove configuration interfaces so the
    // corresponding sensors can be removed.
    auto configIfaceRemovedMatch = std::make_unique<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        sdbusplus::bus::match::rules::interfacesRemovedAtPath(
            std::string(inventoryPath)),
        [&deviceManager](sdbusplus::message_t& msg) {
            deviceManager.onConfigInterfaceRemoved(msg);
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
