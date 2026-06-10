/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DeviceManager.hpp"
#include "MctpRequester.hpp"

#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdlib>
#include <exception>
#include <memory>

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
    deviceManager.start();

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
