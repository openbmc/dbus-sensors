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

const std::string metricInterface = "xyz.openbmc_project.Metric.Value";

NvidiaPciePortErrors::NvidiaPciePortErrors(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& path, uint8_t eid, gpu::PciePortType portType,
    uint8_t upstreamPortNumber, uint8_t portNumber,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), portType(portType), upstreamPortNumber(upstreamPortNumber),
    portNumber(portNumber), path(path), conn(conn), mctpRequester(mctpRequester)
{
    const std::string dbusPath =
        pcieDevicePathPrefix + escapeName(name) + "/Metrics";

    nonFatalErrorsInterface = objectServer.add_interface(
        dbusPath + "/pcie/non_fatal_error_count", metricInterface);

    fatalErrorsInterface = objectServer.add_interface(
        dbusPath + "/pcie/fatal_error_count", metricInterface);

    correctableErrorsInterface = objectServer.add_interface(
        dbusPath + "/pcie/correctable_error_count", metricInterface);

    unsupportedRequestsInterface = objectServer.add_interface(
        dbusPath + "/pcie/unsupported_request_count", metricInterface);

    nonFatalErrorsInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Count"s);
    nonFatalErrorsInterface->register_property("Value", 0.0);

    fatalErrorsInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Count"s);
    fatalErrorsInterface->register_property("Value", 0.0);

    correctableErrorsInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Count"s);
    correctableErrorsInterface->register_property("Value", 0.0);

    unsupportedRequestsInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Count"s);
    unsupportedRequestsInterface->register_property("Value", 0.0);

    nonFatalErrorsInterface->initialize();
    fatalErrorsInterface->initialize();
    correctableErrorsInterface->initialize();
    unsupportedRequestsInterface->initialize();
}

void NvidiaPciePortErrors::processResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating PCIe Port Errors: sending message over MCTP failed, "
            "rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", sendRecvMsgResult.message(), "EID", eid, "PT",
            static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    size_t numTelemetryValue = 0;

    auto rc = gpu::decodeQueryScalarGroupTelemetryV2Response(
        response, cc, reasonCode, numTelemetryValue, telemetryValues);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating PCIe Port Errors: decode failed, "
            "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid, "PT", static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    if (telemetryValues.size() < 4)
    {
        lg2::error(
            "Error updating PCIe Port Errors: insufficient telemetry values, "
            "numValues={NV}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "NV", telemetryValues.size(), "EID", eid, "PT",
            static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    nonFatalErrorsInterface->set_property(
        "Value", static_cast<double>(telemetryValues[0]));

    fatalErrorsInterface->set_property("Value",
                                       static_cast<double>(telemetryValues[1]));

    unsupportedRequestsInterface->set_property(
        "Value", static_cast<double>(telemetryValues[2]));

    correctableErrorsInterface->set_property(
        "Value", static_cast<double>(telemetryValues[3]));
}

void NvidiaPciePortErrors::update()
{
    auto rc = gpu::encodeQueryScalarGroupTelemetryV2Request(
        0, portType, upstreamPortNumber, portNumber, 2, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating PCIe Port Errors: encode failed, rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", rc, "EID", eid, "PT", static_cast<uint8_t>(portType), "PN",
            portNumber);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaPciePortErrors> self = weak.lock();
            if (!self)
            {
                lg2::error(
                    "Invalid reference to NvidiaPciePortErrors, EID={EID}, PortType={PT}, PortNumber={PN}",
                    "EID", self->eid, "PT",
                    static_cast<uint8_t>(self->portType), "PN",
                    self->portNumber);
                return;
            }
            self->processResponse(ec, buffer);
        });
}

NvidiaPciePortCounters::NvidiaPciePortCounters(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& path, uint8_t eid, gpu::PciePortType portType,
    uint8_t upstreamPortNumber, uint8_t portNumber,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), portType(portType), upstreamPortNumber(upstreamPortNumber),
    portNumber(portNumber), path(path), conn(conn), mctpRequester(mctpRequester)
{
    const std::string dbusPath =
        pcieDevicePathPrefix + escapeName(name) + "/Metrics";

    replayCountInterface = objectServer.add_interface(
        dbusPath + "/pcie/replay_count", metricInterface);

    replayRolloverCountInterface = objectServer.add_interface(
        dbusPath + "/pcie/replay_rollover_count", metricInterface);

    nakSentCountInterface = objectServer.add_interface(
        dbusPath + "/pcie/nak_sent_count", metricInterface);

    nakReceivedCountInterface = objectServer.add_interface(
        dbusPath + "/pcie/nak_received_count", metricInterface);

    replayCountInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Count"s);
    replayCountInterface->register_property("Value", 0.0);

    replayRolloverCountInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Count"s);
    replayRolloverCountInterface->register_property("Value", 0.0);

    nakSentCountInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Count"s);
    nakSentCountInterface->register_property("Value", 0.0);

    nakReceivedCountInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Count"s);
    nakReceivedCountInterface->register_property("Value", 0.0);

    replayCountInterface->initialize();
    replayRolloverCountInterface->initialize();
    nakSentCountInterface->initialize();
    nakReceivedCountInterface->initialize();
}

void NvidiaPciePortCounters::processResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating PCIe Port Counters: sending message over MCTP failed, "
            "rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", sendRecvMsgResult.message(), "EID", eid, "PT",
            static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    size_t numTelemetryValue = 0;

    auto rc = gpu::decodeQueryScalarGroupTelemetryV2Response(
        response, cc, reasonCode, numTelemetryValue, telemetryValues);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating PCIe Port Counters: decode failed, "
            "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid, "PT", static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    if (telemetryValues.size() < 7)
    {
        lg2::error(
            "Error updating PCIe Port Counters: insufficient telemetry values, "
            "numValues={NV}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "NV", telemetryValues.size(), "EID", eid, "PT",
            static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    nakReceivedCountInterface->set_property(
        "Value", static_cast<double>(telemetryValues[1]));

    nakSentCountInterface->set_property(
        "Value", static_cast<double>(telemetryValues[2]));

    replayRolloverCountInterface->set_property(
        "Value", static_cast<double>(telemetryValues[4]));

    replayCountInterface->set_property("Value",
                                       static_cast<double>(telemetryValues[6]));
}

void NvidiaPciePortCounters::update()
{
    auto rc = gpu::encodeQueryScalarGroupTelemetryV2Request(
        0, portType, upstreamPortNumber, portNumber, 4, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating PCIe Port Counters: encode failed, rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", rc, "EID", eid, "PT", static_cast<uint8_t>(portType), "PN",
            portNumber);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaPciePortCounters> self = weak.lock();
            if (!self)
            {
                lg2::error(
                    "Invalid reference to NvidiaPciePortCounters, EID={EID}, PortType={PT}, PortNumber={PN}",
                    "EID", self->eid, "PT",
                    static_cast<uint8_t>(self->portType), "PN",
                    self->portNumber);
                return;
            }
            self->processResponse(ec, buffer);
        });
}

NvidiaPciePortL0ToRecoveryCount::NvidiaPciePortL0ToRecoveryCount(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& path, uint8_t eid, gpu::PciePortType portType,
    uint8_t upstreamPortNumber, uint8_t portNumber,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), portType(portType), upstreamPortNumber(upstreamPortNumber),
    portNumber(portNumber), path(path), conn(conn), mctpRequester(mctpRequester)
{
    const std::string dbusPath =
        pcieDevicePathPrefix + escapeName(name) + "/Metrics";

    l0ToRecoveryCountInterface = objectServer.add_interface(
        dbusPath + "/pcie/l0_to_recovery_count", metricInterface);

    l0ToRecoveryCountInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Count"s);
    l0ToRecoveryCountInterface->register_property("Value", 0.0);

    l0ToRecoveryCountInterface->initialize();
}

void NvidiaPciePortL0ToRecoveryCount::processResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating PCIe Port L0ToRecoveryCount: sending message over MCTP failed, "
            "rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", sendRecvMsgResult.message(), "EID", eid, "PT",
            static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    size_t numTelemetryValue = 0;

    auto rc = gpu::decodeQueryScalarGroupTelemetryV2Response(
        response, cc, reasonCode, numTelemetryValue, telemetryValues);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating PCIe Port L0ToRecoveryCount: decode failed, "
            "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid, "PT", static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    if (!telemetryValues.empty())
    {
        l0ToRecoveryCountInterface->set_property(
            "Value", static_cast<double>(telemetryValues[0]));
    }
}

void NvidiaPciePortL0ToRecoveryCount::update()
{
    auto rc = gpu::encodeQueryScalarGroupTelemetryV2Request(
        0, portType, upstreamPortNumber, portNumber, 3, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating PCIe Port L0ToRecoveryCount: encode failed, rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", rc, "EID", eid, "PT", static_cast<uint8_t>(portType), "PN",
            portNumber);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaPciePortL0ToRecoveryCount> self = weak.lock();
            if (!self)
            {
                lg2::error(
                    "Invalid reference to NvidiaPciePortL0ToRecoveryCount, EID={EID}, PortType={PT}, PortNumber={PN}",
                    "EID", self->eid, "PT",
                    static_cast<uint8_t>(self->portType), "PN",
                    self->portNumber);
                return;
            }
            self->processResponse(ec, buffer);
        });
}
