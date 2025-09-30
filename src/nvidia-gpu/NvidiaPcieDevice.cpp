/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
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
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <system_error>

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
    getPciePortCounts();
}

void PcieDevice::getPciePortCounts()
{
    const int rc = gpu::encodeListPciePortsRequest(0, getPciePortCountsRequest);

    if (rc != 0)
    {
        lg2::error(
            "Error updating PCIe Port Counts: encode failed, rc={RC}, EID={EID}",
            "RC", rc, "EID", eid);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, getPciePortCountsRequest,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<PcieDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to PcieDevice");
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

    const int rc = gpu::decodeListPciePortsResponse(
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
        const std::string portName = std::format("UP_{}", i);

        pciePorts.emplace_back(std::make_shared<NvidiaPciePortInfo>(
            conn, mctpRequester, portName, name, path, eid,
            gpu::PciePortType::UPSTREAM, i, i, objectServer));

        pciePortMetrics.emplace_back(makeNvidiaPciePortErrors(
            conn, mctpRequester, portName, name, path, eid,
            gpu::PciePortType::UPSTREAM, i, i, objectServer));

        pciePortMetrics.emplace_back(makeNvidiaPciePortCounters(
            conn, mctpRequester, portName, name, path, eid,
            gpu::PciePortType::UPSTREAM, i, i, objectServer));

        pciePortMetrics.emplace_back(makeNvidiaPciePortL0ToRecoveryCount(
            conn, mctpRequester, portName, name, path, eid,
            gpu::PciePortType::UPSTREAM, i, i, objectServer));

        for (uint64_t j = 0; j < pcieDeviceInfo.numDownstreamPorts[i]; ++j)
        {
            const std::string portName =
                std::format("DOWN_{}", downstreamPortIndex);

            pciePorts.emplace_back(std::make_shared<NvidiaPciePortInfo>(
                conn, mctpRequester, portName, name, path, eid,
                gpu::PciePortType::DOWNSTREAM, i, downstreamPortIndex,
                objectServer));

            pciePortMetrics.emplace_back(makeNvidiaPciePortErrors(
                conn, mctpRequester, portName, name, path, eid,
                gpu::PciePortType::DOWNSTREAM, i, downstreamPortIndex,
                objectServer));

            pciePortMetrics.emplace_back(makeNvidiaPciePortCounters(
                conn, mctpRequester, portName, name, path, eid,
                gpu::PciePortType::DOWNSTREAM, i, downstreamPortIndex,
                objectServer));

            pciePortMetrics.emplace_back(makeNvidiaPciePortL0ToRecoveryCount(
                conn, mctpRequester, portName, name, path, eid,
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

    for (auto& portMetrics : pciePortMetrics)
    {
        portMetrics->update();
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
