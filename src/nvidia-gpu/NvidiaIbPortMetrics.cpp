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
#include <format>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

// GetPortTelemetryCounters defines its counters by reference to the
// InfiniBand specification section 16.1.3.5 PortCounters, where
// port_rcv_data (counter 1) and port_xmit_data (counter 9) count data
// octets divided by four.
static constexpr uint8_t portRcvDataTag = 1;
static constexpr uint8_t portXmitDataTag = 9;
static constexpr double ibDataOctetsPerCount = 4.0;

static constexpr double telemetryScale(uint8_t tag)
{
    return (tag == portRcvDataTag || tag == portXmitDataTag)
               ? ibDataOctetsPerCount
               : 1.0;
}

NvidiaIbPortMetrics::NvidiaIbPortMetrics(
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& deviceName, uint8_t eid, uint16_t portNumber,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), portNumber(portNumber), mctpRequester(mctpRequester),
    objectServer(objectServer)
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

    const sdbusplus::object_path portDbusPath =
        inventoryPrefix / deviceName / name;

    const std::string metricsDbusPathPrefix =
        metricPath + std::format("port_{}_{}", deviceName, name);

    static constexpr auto telemetryMetrics =
        std::to_array<std::tuple<uint8_t, const char*, bool>>({
            {0, "/nic/rx_frames", false},
            {1, "/nic/rx_bytes", true},
            {2, "/nic/rx_multicast_frames", false},
            {3, "/nic/rx_unicast_frames", false},
            {6, "/nic/rx_errors", false},
            {7, "/nic/tx_frames", false},
            {9, "/nic/tx_bytes", true},
            {11, "/nic/tx_unicast_frames", false},
            {12, "/nic/tx_multicast_frames", false},
            {14, "/nic/tx_discards", false},
        });

    for (const auto& [tag, metricName, isByteTotal] : telemetryMetrics)
    {
        const std::string metricObjectPath = metricsDbusPathPrefix + metricName;

        metricValueInterface[tag] = objectServer.add_interface(
            metricObjectPath, "xyz.openbmc_project.Metric.Value");

        metricValueInterface[tag]->register_property(
            "Unit", isByteTotal ? metricUnitBytes : metricUnitCount);
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

NvidiaIbPortMetrics::~NvidiaIbPortMetrics()
{
    for (auto& interface : metricValueInterface)
    {
        objectServer.remove_interface(interface);
    }
    for (auto& interface : metricAssociationInterfaces)
    {
        objectServer.remove_interface(interface);
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
            metricValueInterface[tag]->set_property(
                "Value", static_cast<double>(value) * telemetryScale(tag));
        }
    }
}
