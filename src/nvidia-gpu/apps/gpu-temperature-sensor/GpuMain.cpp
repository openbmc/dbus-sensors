/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuSensorTask.hpp"

#include <getopt.h>

#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdlib>
#include <iostream>
#include <memory>

void optionUsage()
{
    std::cerr << "Usage: gpusensor [options]\n";
    std::cerr << "Options:\n";
    std::cerr << " -h, --help        display this message\n";
    std::cerr << " -v, --verbose     enable verbosity\n";
}

static std::array<struct option, 3> longOptions = {
    {{"verbose", no_argument, nullptr, 'v'},
     {"help", no_argument, nullptr, 'h'},
     {nullptr, 0, nullptr, 0}}};

int main(int argc, char** argv)
{
    int argflag = 0;
    bool verbose = false;

    while ((argflag = getopt_long(argc, argv, "hv", longOptions.data(),
                                  nullptr)) >= 0)
    {
        switch (argflag)
        {
            case 'h':
                optionUsage();
                exit(EXIT_FAILURE);
                break;
            case 'v':
                verbose = true;
                break;
            default:
                exit(EXIT_FAILURE);
        }
    }

    if (verbose)
    {
        lg2::info("gpusensor app is started.");
    }

    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);

    objectServer.add_manager("/xyz/openbmc_project/sensors");

    systemBus->request_name("xyz.openbmc_project.GpuSensor");

    boost::asio::co_spawn(io, gpuSensorTask(io, systemBus, verbose),
                          boost::asio::detached);

    io.run();

    return 0;
}
