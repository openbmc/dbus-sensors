/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
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

PcieDevice::PcieDevice(
    uint64_t pollRate, const std::string& name, const std::string& path,
    const std::shared_ptr<sdbusplus::asio::connection>& conn, uint8_t eid,
    boost::asio::io_context& io, mctp::MctpRequester& mctpRequester,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer),
    name(escapeName(name)), sensorConfiguration(path)
{}

void PcieDevice::init()
{
    makeSensors();
}

void PcieDevice::makeSensors()
{
    pcieInterface = std::make_shared<NvidiaPcieInterface>(
        conn, mctpRequester, name, sensorConfiguration, eid, objectServer);

    lg2::info("Added PCIe {NAME} Sensors.", "NAME", name);

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
