/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaPcieDevice.hpp"

#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaPcieInterface.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

std::shared_ptr<sdbusplus::asio::dbus_interface> PcieDevice::fabricInterface;

PcieDevice::PcieDevice(const SensorConfigs& configs, const std::string& name,
                       const std::string& path,
                       const std::shared_ptr<sdbusplus::asio::connection>& conn,
                       uint8_t eid, boost::asio::io_context& io,
                       mctp::MctpRequester& mctpRequester,
                       sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{configs.pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer),
    configs(configs), name(escapeName(name)), path(path)
{}

void PcieDevice::init()
{
    if (!fabricInterface)
    {
        fabricInterface = objectServer.add_interface(
            fabricPath, "xyz.openbmc_project.Inventory.Item.Fabric");

        if (!fabricInterface->initialize())
        {
            lg2::error(
                "Failed to initialize fabric interface for for eid {EID}",
                "EID", eid);
        }
    }

    makeSensors();
}

void PcieDevice::makeSensors()
{
    pcieInterface = std::make_shared<NvidiaPcieInterface>(
        conn, mctpRequester, name, path, fabricPath, eid, objectServer);

    lg2::info("Added PCIe {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);

    read();
}

void PcieDevice::read()
{
    pcieInterface->update();

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec)
        {
            return;
        }
        read();
    });
}
