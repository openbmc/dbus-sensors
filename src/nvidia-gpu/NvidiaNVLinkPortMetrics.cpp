/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaNVLinkPortMetrics.hpp"

#include "NvidiaUtils.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

NvidiaNVLinkPortMetrics::NvidiaNVLinkPortMetrics(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    uint8_t portIndex, sdbusplus::asio::object_server& objectServer,
    const std::vector<NvidiaNVLinkMetricInfo>& metricsInfo) :
    eid(eid), portNumber(static_cast<uint8_t>(portIndex + 1)), gpuName(gpuName),
    conn(conn), mctpRequester(mctpRequester), objectServer(objectServer)
{
    request.resize(gpu::getPortTelemetryCounterRequestSize);
    if (gpu::encodeGetPortTelemetryCounterRequest(0, portNumber, request) != 0)
    {
        lg2::error(
            "Failed to encode NVLink Port Metrics request, eid={EID}, portNumber={PN}",
            "EID", eid, "PN", portNumber);
        request.clear();
    }

    const std::string portName = std::format("NVLink_{}", portIndex);

    const std::string metricsDbusPathPrefix =
        metricPath + std::format("port_{}_{}", gpuName, portName);

    const sdbusplus::object_path portDbusPath =
        inventoryPrefix / gpuName / portName;

    for (const auto& [counterIndex, metricName, unit] : metricsInfo)
    {
        const std::string metricsDbusPath = metricsDbusPathPrefix + metricName;

        metricValueInterfaces[counterIndex] =
            objectServer.add_interface(metricsDbusPath, metricInterface);
        metricValueInterfaces[counterIndex]->register_property(
            "Unit", std::string(unit));
        metricValueInterfaces[counterIndex]->register_property("Value", 0.0);

        std::vector<Association> associations;
        associations.emplace_back("measuring", "measured_by", portDbusPath);

        metricAssociationInterfaces[counterIndex] =
            objectServer.add_interface(metricsDbusPath, association::interface);
        metricAssociationInterfaces[counterIndex]->register_property(
            "Associations", associations);

        if (!metricValueInterfaces[counterIndex]->initialize())
        {
            lg2::error(
                "Error initializing NVLink Port Metric interface, eid={EID}, portNumber={PN}, metric={MN}",
                "EID", eid, "PN", portNumber, "MN", metricName);
        }

        if (!metricAssociationInterfaces[counterIndex]->initialize())
        {
            lg2::error(
                "Error initializing NVLink Port Metric Association interface, eid={EID}, portNumber={PN}, metric={MN}",
                "EID", eid, "PN", portNumber, "MN", metricName);
        }
    }
}

NvidiaNVLinkPortMetrics::~NvidiaNVLinkPortMetrics()
{
    for (size_t i = 0; i < metricValueInterfaces.size(); ++i)
    {
        if (metricValueInterfaces[i] != nullptr)
        {
            objectServer.remove_interface(metricValueInterfaces[i]);
            objectServer.remove_interface(metricAssociationInterfaces[i]);
        }
    }
}

void NvidiaNVLinkPortMetrics::processResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating NVLink Port Metrics: sending message over MCTP failed, rc={RC}, EID={EID}, portNumber={PN}",
            "RC", sendRecvMsgResult.message(), "EID", eid, "PN", portNumber);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t supportedCounters = 0;
    size_t numCounters = 0;

    const int rc = gpu::decodeGetPortTelemetryCounterResponse(
        response, cc, reasonCode, supportedCounters, numCounters, counters);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating NVLink Port Metrics: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, portNumber={PN}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid, "PN", portNumber);
        return;
    }

    for (size_t i = 0; i < numCounters; ++i)
    {
        // Only refresh counters the device reports as supported; the rest keep
        // their initial value.
        if (metricValueInterfaces[i] != nullptr &&
            (supportedCounters & (1U << i)) != 0)
        {
            metricValueInterfaces[i]->set_property(
                "Value", static_cast<double>(counters[i]));
        }
    }
}

void NvidiaNVLinkPortMetrics::update()
{
    if (request.empty())
    {
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaNVLinkPortMetrics> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaNVLinkPortMetrics");
                return;
            }
            self->processResponse(ec, buffer);
        });
}

std::shared_ptr<NvidiaNVLinkPortMetrics> makeNvidiaNVLinkPortMetrics(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& gpuName, uint8_t eid,
    uint8_t portIndex, sdbusplus::asio::object_server& objectServer)
{
    // DMTF PortMetrics counters. counterIndex is the position in the Get Port
    // Telemetry Counter response (and the supportedCounters bit) per the port
    // counter struct order.
    return std::make_shared<NvidiaNVLinkPortMetrics>(
        conn, mctpRequester, gpuName, eid, portIndex, objectServer,
        std::vector<NvidiaNVLinkMetricInfo>{
            {0, "/nvlink/rx_frames",
             "xyz.openbmc_project.Metric.Value.Unit.Count"},
            {1, "/nvlink/rx_bytes",
             "xyz.openbmc_project.Metric.Value.Unit.Bytes"},
            {6, "/nvlink/rx_errors",
             "xyz.openbmc_project.Metric.Value.Unit.Count"},
            {7, "/nvlink/tx_frames",
             "xyz.openbmc_project.Metric.Value.Unit.Count"},
            {9, "/nvlink/tx_bytes",
             "xyz.openbmc_project.Metric.Value.Unit.Bytes"},
            {14, "/nvlink/tx_discards",
             "xyz.openbmc_project.Metric.Value.Unit.Count"},
        });
}
