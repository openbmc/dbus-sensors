/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaPcieDevice.hpp"

#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaEthPort.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaPcieInterface.hpp"
#include "NvidiaPciePort.hpp"
#include "NvidiaPciePortMetrics.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <chrono>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

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
    sdbusplus::message::object_path networkAdapterPath =
        sdbusplus::message::object_path(nicPathPrefix) / name;

    networkAdapterInterface = objectServer.add_interface(
        networkAdapterPath,
        "xyz.openbmc_project.Inventory.Item.NetworkAdapter");

    std::vector<Association> associations;
    associations.emplace_back(
        "chassis", "all_networkadapters",
        sdbusplus::message::object_path(path).parent_path());

    networkAdapterAssociationInterface =
        objectServer.add_interface(networkAdapterPath, association::interface);
    networkAdapterAssociationInterface->register_property(
        "Associations", associations);

    if (!networkAdapterInterface->initialize())
    {
        lg2::error(
            "Failed to initialize network adapter interface for for eid {EID}",
            "EID", eid);
    }

    if (!networkAdapterAssociationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association Interface for Network Adapter for eid {EID}",
            "EID", eid);
    }

    getPciePortCounts();
}

void PcieDevice::getPciePortCounts()
{
    auto rc = gpu::encodeListPciePortsRequest(0, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating PCIe Port Counts: encode failed, rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<PcieDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to PcieDevice, EID={EID}", "EID",
                           self->eid);
                return;
            }
            self->processPciePortCountsResponse(ec, buffer);
        });
}

void PcieDevice::processPciePortCountsResponse(
    const std::error_code& ec, std::span<const uint8_t> response)
{
    if (ec)
    {
        lg2::error(
            "Error processing PCIe Port Counts response: sending message over MCTP failed, rc={RC}, EID={EID}",
            "RC", ec.message(), "EID", eid);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    auto rc = gpu::decodeListPciePortsResponse(
        response, cc, reasonCode, pcieDeviceInfo.numUpstreamPorts,
        pcieDeviceInfo.numDownstreamPorts);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error processing PCIe Port Counts response: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid);
        return;
    }

    lg2::info("PCIe Device with eid {EID} has {UP} upstream ports.", "EID", eid,
              "UP", pcieDeviceInfo.numUpstreamPorts);

    makeSensors();
}

void PcieDevice::makeSensors()
{
    pcieInterface = std::make_shared<NvidiaPcieInterface>(
        conn, mctpRequester, name, path, eid, objectServer);

    uint64_t downstreamPortIndex = 0;

    for (uint64_t i = 0; i < pcieDeviceInfo.numUpstreamPorts; ++i)
    {
        sdbusplus::message::object_path portPath =
            sdbusplus::message::object_path(name) / "Ports" /
            std::format("UP_{}", i);

        pciePorts.emplace_back(std::make_shared<NvidiaPciePortInfo>(
            conn, mctpRequester, portPath, name, path, eid,
            gpu::PciePortType::UPSTREAM, i, i, objectServer));

        pciePortErrors.emplace_back(std::make_shared<NvidiaPciePortErrors>(
            conn, mctpRequester, portPath, path, eid,
            gpu::PciePortType::UPSTREAM, i, i, objectServer));

        pciePortCounters.emplace_back(std::make_shared<NvidiaPciePortCounters>(
            conn, mctpRequester, portPath, path, eid,
            gpu::PciePortType::UPSTREAM, i, i, objectServer));

        pciePortL0ToRecoveryCounts.emplace_back(
            std::make_shared<NvidiaPciePortL0ToRecoveryCount>(
                conn, mctpRequester, portPath, path, eid,
                gpu::PciePortType::UPSTREAM, i, i, objectServer));

        for (uint64_t j = 0; j < pcieDeviceInfo.numDownstreamPorts[i]; ++j)
        {
            sdbusplus::message::object_path portPath =
                sdbusplus::message::object_path(name) / "Ports" /
                std::format("DOWN_{}", downstreamPortIndex);

            pciePorts.emplace_back(std::make_shared<NvidiaPciePortInfo>(
                conn, mctpRequester, portPath, name, path, eid,
                gpu::PciePortType::DOWNSTREAM, i, downstreamPortIndex,
                objectServer));

            pciePortErrors.emplace_back(std::make_shared<NvidiaPciePortErrors>(
                conn, mctpRequester, portPath, path, eid,
                gpu::PciePortType::DOWNSTREAM, i, downstreamPortIndex,
                objectServer));

            pciePortCounters.emplace_back(
                std::make_shared<NvidiaPciePortCounters>(
                    conn, mctpRequester, portPath, path, eid,
                    gpu::PciePortType::DOWNSTREAM, i, downstreamPortIndex,
                    objectServer));

            pciePortL0ToRecoveryCounts.emplace_back(
                std::make_shared<NvidiaPciePortL0ToRecoveryCount>(
                    conn, mctpRequester, portPath, path, eid,
                    gpu::PciePortType::DOWNSTREAM, i, downstreamPortIndex,
                    objectServer));

            ++downstreamPortIndex;
        }
    }

    for (uint64_t k = 0; k < configs.nicNetworkPortCount; ++k)
    {
        sdbusplus::message::object_path portName =
            sdbusplus::message::object_path(name) / "Ports" /
            std::format("Port_{}", k);

        if (configs.nicNetworkPortType == "Ethernet")
        {
            ethPortMetrics.emplace_back(std::make_shared<NvidiaEthPortMetrics>(
                conn, mctpRequester, portName,
                sdbusplus::message::object_path(nicPathPrefix) / name, eid, k,
                objectServer));
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

    for (auto& ethPortMetric : ethPortMetrics)
    {
        ethPortMetric->update();
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
