/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaEthPort.hpp"

#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaPcieDevice.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

using std::string;

using namespace std::literals;

NvidiaEthPortMetrics::NvidiaEthPortMetrics(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& path, uint8_t eid, uint16_t portNumber,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), portNumber(portNumber), path(path), conn(conn),
    mctpRequester(mctpRequester)
{
    const std::string portDbusPath = nicPathPrefix + escapeName(name);
    const std::string metricsDbusPath = metricPath + escapeName(name);

    portInterface = objectServer.add_interface(
        portDbusPath, "xyz.openbmc_project.Inventory.Connector.Port");

    std::vector<Association> associations;
    associations.emplace_back("connected_to", "connecting", path);

    associationInterface =
        objectServer.add_interface(portDbusPath, association::interface);

    associationInterface->register_property("Associations", associations);

    constexpr std::array<std::pair<uint8_t, const char*>, 21> telemetryMetrics =
        {{
            {0, "/nic/rx_bytes"},
            {1, "/nic/tx_bytes"},
            {2, "/nic/rx_unicast_frames"},
            {3, "/nic/rx_multicast_frames"},
            {4, "/nic/rx_broadcast_frames"},
            {5, "/nic/tx_unicast_frames"},
            {6, "/nic/tx_multicast_frames"},
            {7, "/nic/tx_broadcast_frames"},
            {8, "/nic/rx_fcs_errors"},
            {9, "/nic/rx_frame_alignment_errors"},
            {10, "/nic/rx_false_carrier_errors"},
            {11, "/nic/rx_undersize_frames"},
            {12, "/nic/rx_oversize_frames"},
            {13, "/nic/rx_pause_xon_frames"},
            {14, "/nic/rx_pause_xoff_frames"},
            {15, "/nic/tx_pause_xon_frames"},
            {16, "/nic/tx_pause_xoff_frames"},
            {17, "/nic/tx_single_collisions"},
            {18, "/nic/tx_multiple_collisions"},
            {19, "/nic/tx_late_collisions"},
            {20, "/nic/tx_excessive_collisions"},
        }};

    for (const auto& [tag, metricName] : telemetryMetrics)
    {
        metricValueInterface[tag] = objectServer.add_interface(
            metricsDbusPath + metricName, "xyz.openbmc_project.Metric.Value");

        metricValueInterface[tag]->register_property(
            "Unit", "xyz.openbmc_project.Metric.Value.Unit.Count"s);
        metricValueInterface[tag]->register_property("Value", 0.0);

        std::vector<Association> associations;
        associations.emplace_back("measuring", "measured_by", portDbusPath);

        metricAssociationInterfaces[tag] = objectServer.add_interface(
            metricsDbusPath + metricName, association::interface);
        metricAssociationInterfaces[tag]->register_property("Associations",
                                                            associations);
        if (!metricValueInterface[tag]->initialize())
        {
            lg2::error(
                "Error initializing Ethernet Port Metric Interface for EID={EID}, PortNumber={PN}, Metric={MN}",
                "EID", eid, "PN", portNumber, "MN", metricName);
        }

        if (!metricAssociationInterfaces[tag]->initialize())
        {
            lg2::error(
                "Error initializing Ethernet Port Metric Association Interface for EID={EID}, PortNumber={PN}, Metric={MN}",
                "EID", eid, "PN", portNumber, "MN", metricName);
        }
    }

    if (!portInterface->initialize())
    {
        lg2::error(
            "Error initializing Ethernet Port Interface for EID={EID}, PortNumber={PN}",
            "EID", eid, "PN", portNumber);
    }

    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association Interface for Ethernet Port for EID={EID}, PortNumber={PN}",
            "EID", eid, "PN", portNumber);
    }
}

void NvidiaEthPortMetrics::processResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating Ethernet Port Metrics: sending message over MCTP failed, "
            "rc={RC}, EID={EID}, PortNumber={PN}",
            "RC", sendRecvMsgResult.message(), "EID", eid, "PN", portNumber);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    std::vector<std::pair<uint8_t, uint64_t>> telemetryValues;

    const int rc = gpu::decodeGetEthernetPortTelemetryCountersResponse(
        response, cc, reasonCode, telemetryValues);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Ethernet Port Metrics: decode failed, "
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

void NvidiaEthPortMetrics::update()
{
    const int rc = gpu::encodeGetEthernetPortTelemetryCountersRequest(
        0, portNumber, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating Ethernet Port Metrics: encode failed, rc={RC}, EID={EID}, PortNumber={PN}",
            "RC", rc, "EID", eid, "PN", portNumber);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaEthPortMetrics> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaEthPortMetrics");
                return;
            }
            self->processResponse(ec, buffer);
        });
}
