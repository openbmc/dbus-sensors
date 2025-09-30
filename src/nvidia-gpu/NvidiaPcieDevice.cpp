/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaPcieDevice.hpp"

#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaPcieInterface.hpp"
#include "NvidiaPciePort.hpp"
#include "NvidiaPciePortMetrics.hpp"
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
#include <unordered_map>

std::unordered_map<std::string,
                   std::shared_ptr<sdbusplus::asio::dbus_interface>>
    PcieDevice::fabricInterfaces = {};

PcieDevice::PcieDevice(const SensorConfigs& configs, const std::string& name,
                       const std::string& path,
                       const std::shared_ptr<sdbusplus::asio::connection>& conn,
                       uint8_t eid, boost::asio::io_context& io,
                       mctp::MctpRequester& mctpRequester,
                       sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{configs.pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer),
    configs(configs), name(escapeName(name)), path(path),
    fabricPath(fabricPathPrefix + escapeName(configs.fabricName))
{
    makeSensors();

    if (!configs.fabricName.empty() &&
        !fabricInterfaces.contains(configs.fabricName))
    {
        auto fabricInterface = objectServer.add_interface(
            fabricPath, "xyz.openbmc_project.Inventory.Item.Fabric");

        if (!fabricInterface->initialize())
        {
            lg2::error(
                "Failed to initialize fabric interface for {FABRIC} for eid {EID}",
                "FABRIC", configs.fabricName, "EID", eid);
        }
        else
        {
            fabricInterfaces[configs.fabricName] = fabricInterface;
        }
    }
}

void PcieDevice::makeSensors()
{
    pcieInterface = std::make_shared<NvidiaPcieInterface>(
        conn, mctpRequester, name, path, fabricPath, eid, objectServer);

    uint64_t downstreamPortIndex{};

    for (uint64_t i = 0; i < configs.nicPcieUpstreamPortCount; ++i)
    {
        const std::string portName = name + "/Ports/UP_" + std::to_string(i);

        pciePorts.emplace_back(std::make_shared<NvidiaPciePortInfo>(
            conn, mctpRequester, portName, name, path, eid,
            gpu::PciePortType::UPSTREAM, i, i, objectServer));

        pciePortErrors.emplace_back(std::make_shared<NvidiaPciePortErrors>(
            conn, mctpRequester, portName, path, eid,
            gpu::PciePortType::UPSTREAM, i, i, objectServer));

        pciePortCounters.emplace_back(std::make_shared<NvidiaPciePortCounters>(
            conn, mctpRequester, portName, path, eid,
            gpu::PciePortType::UPSTREAM, i, i, objectServer));

        pciePortL0ToRecoveryCounts.emplace_back(
            std::make_shared<NvidiaPciePortL0ToRecoveryCount>(
                conn, mctpRequester, portName, path, eid,
                gpu::PciePortType::UPSTREAM, i, i, objectServer));

        for (uint64_t j = 0;
             j < configs.nicPcieDownstreamPortCountPerUpstreamPort; ++j)
        {
            std::string portName =
                name + "/Ports/DOWN_" + std::to_string(downstreamPortIndex);

            pciePorts.emplace_back(std::make_shared<NvidiaPciePortInfo>(
                conn, mctpRequester, portName, name, path, eid,
                gpu::PciePortType::DOWNSTREAM, i, downstreamPortIndex,
                objectServer));

            pciePortErrors.emplace_back(std::make_shared<NvidiaPciePortErrors>(
                conn, mctpRequester, portName, path, eid,
                gpu::PciePortType::DOWNSTREAM, i, downstreamPortIndex,
                objectServer));

            pciePortCounters.emplace_back(
                std::make_shared<NvidiaPciePortCounters>(
                    conn, mctpRequester, portName, path, eid,
                    gpu::PciePortType::DOWNSTREAM, i, downstreamPortIndex,
                    objectServer));

            pciePortL0ToRecoveryCounts.emplace_back(
                std::make_shared<NvidiaPciePortL0ToRecoveryCount>(
                    conn, mctpRequester, portName, path, eid,
                    gpu::PciePortType::DOWNSTREAM, i, downstreamPortIndex,
                    objectServer));

            ++downstreamPortIndex;
        }
    }

    lg2::info("Added PCIe {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);

    read();
}

void PcieDevice::read()
{
    pcieInterface->update();

    for (auto& port : pciePorts)
    {
        port->update();
    }

    for (auto& portError : pciePortErrors)
    {
        portError->update();
    }

    for (auto& portCounter : pciePortCounters)
    {
        portCounter->update();
    }

    for (auto& portL0ToRecoveryCount : pciePortL0ToRecoveryCounts)
    {
        portL0ToRecoveryCount->update();
    }

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec)
        {
            return;
        }
        read();
    });
}
