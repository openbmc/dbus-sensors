/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpRequester.hpp"

#include <NvidiaDeviceDiscovery.hpp>
#include <NvidiaPcieDevice.hpp>
#include <NvidiaSmaDevice.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <cstdlib>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>> gpuDevices;
boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>> smaDevices;
boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>
    pcieDevices;

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

    // Watch for new MCTP endpoints being added after startup
    auto mctpEndpointAddedMatch = std::make_shared<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*systemBus),
        sdbusplus::bus::match::rules::interfacesAdded() +
            sdbusplus::bus::match::rules::argNpath(0, "/au/com/codeconstruct/"),
        [&io, &objectServer, systemBus,
         &mctpRequester](sdbusplus::message_t& msg) {
            sdbusplus::message::object_path objPath;
            std::map<std::string,
                     std::map<std::string, std::variant<std::vector<uint8_t>,
                                                        uint8_t, std::string>>>
                interfaces;

            msg.read(objPath, interfaces);

            // Check if MCTP.Endpoint interface was added
            if (interfaces.contains("xyz.openbmc_project.MCTP.Endpoint"))
            {
                handleMctpEndpointAdded(io, objectServer, gpuDevices,
                                        smaDevices, pcieDevices, systemBus,
                                        mctpRequester, objPath.str);
            }
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
