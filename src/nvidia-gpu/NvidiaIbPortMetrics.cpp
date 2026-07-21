/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaIbPortMetrics.hpp"

#include "NvidiaUtils.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <format>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

static constexpr uint8_t nodeGuidTag = 3;
static constexpr uint8_t portGuidTag = 4;

static std::string formatGuid(uint64_t value)
{
    std::array<uint8_t, sizeof(uint64_t)> guid{};
    std::memcpy(guid.data(), &value, guid.size());
    // Redfish models an InfiniBand GUID as four colon-separated 16-bit
    // groups (XXXX:XXXX:XXXX:XXXX), matching the NetworkDeviceFunction
    // InfiniBand.PermanentNodeGUID / PermanentPortGUID pattern.
    return std::format("{:02X}{:02X}:{:02X}{:02X}:{:02X}{:02X}:{:02X}{:02X}",
                       guid[0], guid[1], guid[2], guid[3], guid[4], guid[5],
                       guid[6], guid[7]);
}

NvidiaIbPortMetrics::NvidiaIbPortMetrics(
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& deviceName, uint8_t eid, uint16_t portNumber,
    sdbusplus::asio::object_server& objectServer,
    const std::vector<std::pair<uint8_t, uint64_t>>& addresses) :
    eid(eid), portNumber(portNumber), mctpRequester(mctpRequester)
{
    const int rc =
        gpu::encodeGetPortTelemetryCountersRequest(0, portNumber, request);
    if (rc == 0)
    {
        requestEncoded = true;
    }
    else
    {
        lg2::error(
            "Failed to encode IB Port Metrics request for EID={EID}, PortNumber={PN}, rc={RC}",
            "EID", eid, "PN", portNumber, "RC", rc);
    }

    const sdbusplus::object_path deviceDbusPath = inventoryPrefix / deviceName;

    const sdbusplus::object_path portDbusPath =
        inventoryPrefix / deviceName / name;

    const std::string metricsDbusPathPrefix =
        metricPath + std::format("port_{}_{}", deviceName, name);

    portInterface = objectServer.add_interface(
        portDbusPath, "xyz.openbmc_project.Inventory.Connector.Port");
    portInterface->register_property(
        "PortProtocol",
        std::string("xyz.openbmc_project.Inventory.Connector.Port."
                    "PortProtocol.InfiniBand"));
    if (!portInterface->initialize())
    {
        lg2::error("Error initializing IB Port interface, eid={EID}, port={PN}",
                   "EID", eid, "PN", portNumber);
    }

    std::vector<Association> associations;
    associations.emplace_back("connected_to", "connecting", deviceDbusPath);
    associationInterface =
        objectServer.add_interface(portDbusPath, association::interface);
    associationInterface->register_property("Associations", associations);
    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing IB Port Association interface, eid={EID}, port={PN}",
            "EID", eid, "PN", portNumber);
    }

    std::string permanentNodeGuid;
    std::string permanentPortGuid;
    for (const auto& [tag, value] : addresses)
    {
        if (tag == nodeGuidTag)
        {
            permanentNodeGuid = formatGuid(value);
        }
        else if (tag == portGuidTag)
        {
            permanentPortGuid = formatGuid(value);
        }
    }

    const sdbusplus::object_path ndfDbusPath =
        inventoryPrefix / deviceName / "NetworkDeviceFunctions" / name;

    networkDeviceFunctionInterface = objectServer.add_interface(
        ndfDbusPath, "xyz.openbmc_project.Inventory.Item.NetworkInterface");
    networkDeviceFunctionInterface->register_property("PermanentNodeGUID",
                                                      permanentNodeGuid);
    networkDeviceFunctionInterface->register_property("PermanentPortGUID",
                                                      permanentPortGuid);
    if (!networkDeviceFunctionInterface->initialize())
    {
        lg2::error(
            "Error initializing IB NetworkDeviceFunction interface, eid={EID}, port={PN}",
            "EID", eid, "PN", portNumber);
    }

    std::vector<Association> ndfAssociations;
    ndfAssociations.emplace_back("exposed_by", "exposing", deviceDbusPath);
    ndfAssociations.emplace_back("assigned_to", "assigning", portDbusPath);
    networkDeviceFunctionAssociationInterface =
        objectServer.add_interface(ndfDbusPath, association::interface);
    networkDeviceFunctionAssociationInterface->register_property(
        "Associations", ndfAssociations);
    if (!networkDeviceFunctionAssociationInterface->initialize())
    {
        lg2::error(
            "Error initializing IB NetworkDeviceFunction Association interface, eid={EID}, port={PN}",
            "EID", eid, "PN", portNumber);
    }

    static constexpr auto telemetryMetrics =
        std::to_array<std::pair<uint8_t, const char*>>({
            {0, "/nic/rx_frames"},
            {1, "/nic/rx_bytes"},
            {2, "/nic/rx_multicast_frames"},
            {3, "/nic/rx_unicast_frames"},
            {6, "/nic/rx_errors"},
            {7, "/nic/tx_frames"},
            {9, "/nic/tx_bytes"},
            {11, "/nic/tx_unicast_frames"},
            {12, "/nic/tx_multicast_frames"},
            {14, "/nic/tx_discards"},
        });

    for (const auto& [tag, metricName] : telemetryMetrics)
    {
        const std::string metricObjectPath = metricsDbusPathPrefix + metricName;

        metricValueInterface[tag] = objectServer.add_interface(
            metricObjectPath, "xyz.openbmc_project.Metric.Value");

        metricValueInterface[tag]->register_property(
            "Unit", std::string("xyz.openbmc_project.Metric.Value.Unit.Count"));
        metricValueInterface[tag]->register_property("Value", 0.0);

        std::vector<Association> metricAssociations;
        metricAssociations.emplace_back("measuring", "measured_by",
                                        portDbusPath);

        metricAssociationInterfaces[tag] = objectServer.add_interface(
            metricObjectPath, association::interface);
        metricAssociationInterfaces[tag]->register_property("Associations",
                                                            metricAssociations);
        if (!metricValueInterface[tag]->initialize())
        {
            lg2::error(
                "Error initializing IB Port Metric interface, eid={EID}, port={PN}, metric={MN}",
                "EID", eid, "PN", portNumber, "MN", metricName);
        }

        if (!metricAssociationInterfaces[tag]->initialize())
        {
            lg2::error(
                "Error initializing IB Port Metric Association interface, eid={EID}, port={PN}, metric={MN}",
                "EID", eid, "PN", portNumber, "MN", metricName);
        }
    }
}

void NvidiaIbPortMetrics::processResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating IB Port Metrics: sending message over MCTP failed, "
            "rc={RC}, EID={EID}, PortNumber={PN}",
            "RC", sendRecvMsgResult.message(), "EID", eid, "PN", portNumber);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    std::vector<std::pair<uint8_t, uint64_t>> telemetryValues;

    const int rc = gpu::decodeGetPortTelemetryCountersResponse(
        response, cc, reasonCode, telemetryValues);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating IB Port Metrics: decode failed, "
            "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, PortNumber={PN}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid, "PN", portNumber);
        return;
    }

    for (const auto& [tag, value] : telemetryValues)
    {
        if (tag < maxTelemetryValues && metricValueInterface[tag])
        {
            metricValueInterface[tag]->set_property("Value",
                                                    static_cast<double>(value));
        }
    }
}

void NvidiaIbPortMetrics::update()
{
    if (!requestEncoded)
    {
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaIbPortMetrics> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaIbPortMetrics");
                return;
            }
            self->processResponse(ec, buffer);
        });
}
