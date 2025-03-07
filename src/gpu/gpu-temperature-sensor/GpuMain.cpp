/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuSensorTask.hpp"
#include "MctpRequester.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    systemBus->request_name("xyz.openbmc_project.GpuSensor");

    mctp::MctpRequester mctpRequester{};

    boost::asio::post(io,
                      [&]() { createSensors(io, systemBus, mctpRequester); });

    io.run();
    return 0;
}
