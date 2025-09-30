/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaPciePortMetrics.hpp"

#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaPcieDevice.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

using std::string;

using namespace std::literals;

constexpr const char* metricInterface = "xyz.openbmc_project.Metric.Value";

NvidiaPciePortMetrics::NvidiaPciePortMetrics(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& path, uint8_t eid, gpu::PciePortType portType,
    uint8_t upstreamPortNumber, uint8_t portNumber,
    sdbusplus::asio::object_server& objectServer, uint8_t scalarGroupId,
    const std::vector<NvidiaMetricInfo>& metricsInfo) :
    eid(eid), portType(portType), upstreamPortNumber(upstreamPortNumber),
    portNumber(portNumber), scalarGroupId(scalarGroupId), path(path),
    conn(conn), mctpRequester(mctpRequester)
{
    const std::string metricsDbusPathPrefix = metricPath + escapeName(name);

    const std::string pcieDeviceDbusPath =
        pcieDevicePathPrefix + escapeName(name);

    for (const auto& [id, name] : metricsInfo)
    {
        const std::string metricsDbusPath = metricsDbusPathPrefix + name;

        metricValueInterfaces[id] =
            objectServer.add_interface(metricsDbusPath, metricInterface);
        metricValueInterfaces[id]->register_property(
            "Unit", "xyz.openbmc_project.Metric.Value.Unit.Count"s);
        metricValueInterfaces[id]->register_property("Value", 0.0);

        std::vector<Association> associations;
        associations.emplace_back("measuring", "measured_by",
                                  pcieDeviceDbusPath);

        metricAssociationInterfaces[id] =
            objectServer.add_interface(metricsDbusPath, association::interface);
        metricAssociationInterfaces[id]->register_property("Associations",
                                                           associations);

        if (!metricValueInterfaces[id]->initialize())
        {
            lg2::error(
                "Error initializing PCIe Port Metric Interface for EID={EID}, "
                "PortType={PT}, PortNumber={PN}, ScalarGroup={SG}, Metric={MN}",
                "EID", eid, "PT", static_cast<uint8_t>(portType), "PN",
                portNumber, "EID", eid, "PN", portNumber, "SG", scalarGroupId,
                "MN", name);
        }

        if (!metricAssociationInterfaces[id]->initialize())
        {
            lg2::error(
                "Error initializing PCIe Port Metric Association Interface for EID={EID}, "
                "PortType={PT}, PortNumber={PN}, ScalarGroup={SG}, Metric={MN}",
                "EID", eid, "PT", static_cast<uint8_t>(portType), "PN",
                portNumber, "EID", eid, "PN", portNumber, "SG", scalarGroupId,
                "MN", name);
        }
    }
}

void NvidiaPciePortMetrics::processResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating PCIe Port Metrics: sending message over MCTP failed, "
            "rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}, ScalarGroup={SG}",
            "RC", sendRecvMsgResult.message(), "EID", eid, "PT",
            static_cast<uint8_t>(portType), "PN", portNumber, "SG",
            scalarGroupId);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    size_t numTelemetryValue = 0;

    int rc = gpu::decodeQueryScalarGroupTelemetryV2Response(
        response, cc, reasonCode, numTelemetryValue, telemetryValues);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating PCIe Port Errors: decode failed, "
            "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, PortType={PT}, PortNumber={PN}, ScalarGroup={SG}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid, "PT", static_cast<uint8_t>(portType), "PN", portNumber, "SG",
            scalarGroupId);
        return;
    }

    for (size_t i = 0; i < numTelemetryValue; ++i)
    {
        if (metricValueInterfaces[i] != nullptr)
        {
            metricValueInterfaces[i]->set_property(
                "Value", static_cast<double>(telemetryValues[i]));
        }
    }
}

void NvidiaPciePortMetrics::update()
{
    auto rc = gpu::encodeQueryScalarGroupTelemetryV2Request(
        0, portType, upstreamPortNumber, portNumber, scalarGroupId, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating PCIe Port Errors: encode failed, rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}, ScalarGroup={SG}",
            "RC", rc, "EID", eid, "PT", static_cast<uint8_t>(portType), "PN",
            portNumber, "SG", scalarGroupId);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaPciePortMetrics> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid reference to NvidiaPciePortErrors");
                return;
            }
            self->processResponse(ec, buffer);
        });
}

std::shared_ptr<NvidiaPciePortMetrics> makeNvidiaPciePortErrors(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& path, uint8_t eid, gpu::PciePortType portType,
    uint8_t upstreamPortNumber, uint8_t portNumber,
    sdbusplus::asio::object_server& objectServer)
{
    static constexpr uint8_t nvidiaPciePortErrorScalarGroupId = 2;

    return std::make_shared<NvidiaPciePortMetrics>(
        conn, mctpRequester, name, path, eid, portType, upstreamPortNumber,
        portNumber, objectServer, nvidiaPciePortErrorScalarGroupId,
        std::vector<NvidiaMetricInfo>{
            {0, "/pcie/non_fatal_error_count"},
            {1, "/pcie/fatal_error_count"},
            {2, "/pcie/unsupported_request_count"},
            {3, "/pcie/correctable_error_count"},
        });
}

std::shared_ptr<NvidiaPciePortMetrics> makeNvidiaPciePortCounters(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& path, uint8_t eid, gpu::PciePortType portType,
    uint8_t upstreamPortNumber, uint8_t portNumber,
    sdbusplus::asio::object_server& objectServer)
{
    static constexpr uint8_t nvidiaPciePortCounterScalarGroupId = 4;

    return std::make_shared<NvidiaPciePortMetrics>(
        conn, mctpRequester, name, path, eid, portType, upstreamPortNumber,
        portNumber, objectServer, nvidiaPciePortCounterScalarGroupId,
        std::vector<NvidiaMetricInfo>{
            {1, "/pcie/nak_received_count"},
            {2, "/pcie/nak_sent_count"},
            {4, "/pcie/replay_rollover_count"},
            {6, "/pcie/replay_count"},
        });
}

std::shared_ptr<NvidiaPciePortMetrics> makeNvidiaPciePortL0ToRecoveryCount(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& path, uint8_t eid, gpu::PciePortType portType,
    uint8_t upstreamPortNumber, uint8_t portNumber,
    sdbusplus::asio::object_server& objectServer)
{
    static constexpr uint8_t nvidiaPciePortL0ToRecoveryCountScalarGroupId = 3;

    return std::make_shared<NvidiaPciePortMetrics>(
        conn, mctpRequester, name, path, eid, portType, upstreamPortNumber,
        portNumber, objectServer, nvidiaPciePortL0ToRecoveryCountScalarGroupId,
        std::vector<NvidiaMetricInfo>{
            {0, "/pcie/l0_to_recovery_count"},
        });
}
