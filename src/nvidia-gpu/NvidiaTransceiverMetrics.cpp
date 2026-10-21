/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaTransceiverMetrics.hpp"

#include "MctpRequester.hpp"
#include "NvidiaUtils.hpp"
#include "Utils.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace
{

using LaneReadings =
    std::array<std::optional<uint16_t>, gpu::opticalModuleLaneCount>;

// Query Port Telemetry v2 reports optical power in 1 uW steps and laser bias
// current in 2 uA steps, the latter before the scaling factor is applied.
constexpr double microunitsPerUnit = 1'000'000.0;
constexpr double microampsPerBiasStep = 2.0;

// The scaling factor is an index: 0 for 1x, 1 for 2x and 2 for 4x.
constexpr uint8_t maxTxBiasScalingFactor = 2;

constexpr size_t maxPagesPerRead = 34;

double toWatts(const std::optional<uint16_t>& microwatts)
{
    if (!microwatts)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return *microwatts / microunitsPerUnit;
}

double toAmperes(const std::optional<uint16_t>& biasSteps,
                 const std::optional<uint8_t>& scalingFactor)
{
    // A device that does not report the scaling factor is read at 1x.
    const uint8_t factor = scalingFactor.value_or(0);
    if (!biasSteps || factor > maxTxBiasScalingFactor)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return *biasSteps * microampsPerBiasStep * (1U << factor) /
           microunitsPerUnit;
}

void mergeLanes(LaneReadings& lanes, const LaneReadings& page)
{
    for (size_t lane = 0; lane < lanes.size(); ++lane)
    {
        if (page[lane])
        {
            lanes[lane] = page[lane];
        }
    }
}

} // namespace

NvidiaTransceiverMetrics::NvidiaTransceiverMetrics(
    sdbusplus::asio::object_server& objectServer,
    mctp::MctpRequester& mctpRequester, const std::string& deviceName,
    const std::string& portName, const sdbusplus::object_path& transceiverPath,
    uint8_t eid, uint16_t portNumber) :
    objectServer(objectServer), mctpRequester(mctpRequester), eid(eid),
    portNumber(portNumber)
{
    const sdbusplus::object_path metricsPath =
        sdbusplus::object_path(metricPath) /
        std::format("transceiver_{}_{}", deviceName, portName);

    addLaneMetrics(metricsPath, transceiverPath, "tx_output_power_lane_",
                   metricUnitWatts, txPowerInterfaces,
                   txPowerAssociationInterfaces);
    addLaneMetrics(metricsPath, transceiverPath, "rx_input_power_lane_",
                   metricUnitWatts, rxPowerInterfaces,
                   rxPowerAssociationInterfaces);
    addLaneMetrics(metricsPath, transceiverPath, "tx_bias_current_lane_",
                   metricUnitAmperes, txBiasCurrentInterfaces,
                   txBiasCurrentAssociationInterfaces);
}

NvidiaTransceiverMetrics::~NvidiaTransceiverMetrics()
{
    for (const auto& iface : txPowerInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    for (const auto& iface : txPowerAssociationInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    for (const auto& iface : rxPowerInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    for (const auto& iface : rxPowerAssociationInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    for (const auto& iface : txBiasCurrentInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    for (const auto& iface : txBiasCurrentAssociationInterfaces)
    {
        objectServer.remove_interface(iface);
    }
}

void NvidiaTransceiverMetrics::addLaneMetrics(
    const sdbusplus::object_path& metricsPath,
    const sdbusplus::object_path& transceiverPath, std::string_view name,
    const std::string& unit, LaneInterfaces& interfaces,
    LaneInterfaces& associationInterfaces)
{
    for (size_t lane = 0; lane < interfaces.size(); ++lane)
    {
        const sdbusplus::object_path path =
            metricsPath / std::format("{}{}", name, lane);

        interfaces[lane] = objectServer.add_interface(
            path, "xyz.openbmc_project.Metric.Value");
        interfaces[lane]->register_property("Unit", unit);
        interfaces[lane]->register_property(
            "Value", std::numeric_limits<double>::quiet_NaN());

        std::vector<Association> associations;
        associations.emplace_back("measuring", "measured_by", transceiverPath);

        associationInterfaces[lane] =
            objectServer.add_interface(path, association::interface);
        associationInterfaces[lane]->register_property("Associations",
                                                       associations);

        if (!interfaces[lane]->initialize())
        {
            lg2::error(
                "Error initializing Transceiver Metric interface, eid={EID}, portNumber={PN}, metric={MN}",
                "EID", eid, "PN", portNumber, "MN", path.str);
        }

        if (!associationInterfaces[lane]->initialize())
        {
            lg2::error(
                "Error initializing Transceiver Metric Association interface, eid={EID}, portNumber={PN}, metric={MN}",
                "EID", eid, "PN", portNumber, "MN", path.str);
        }
    }
}

void NvidiaTransceiverMetrics::update()
{
    if (readInflight)
    {
        return;
    }

    readInflight = true;
    pagesRequested = 0;
    readings = gpu::OpticalModuleTelemetry{};

    requestPage(0);
}

void NvidiaTransceiverMetrics::requestPage(uint32_t sequenceToken)
{
    const int rc = gpu::encodeQueryPortTelemetryV2Request(
        0, sequenceToken, portNumber, gpu::PortTelemetryGroupId::OpticalModule,
        request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating Transceiver Metrics: encode failed, rc={RC}, EID={EID}, PortNumber={PN}",
            "RC", rc, "EID", eid, "PN", portNumber);
        readInflight = false;
        return;
    }

    ++pagesRequested;

    mctpRequester.sendRecvMsg(
        eid, request,
        [eid{this->eid}, portNumber{this->portNumber}, weak{weak_from_this()}](
            const std::error_code& ec, std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaTransceiverMetrics> self = weak.lock();
            if (!self)
            {
                lg2::error(
                    "Invalid reference to NvidiaTransceiverMetrics, EID={EID}, PortNumber={PN}",
                    "EID", eid, "PN", portNumber);
                return;
            }
            self->processResponse(ec, buffer);
        });
}

void NvidiaTransceiverMetrics::processResponse(
    const std::error_code& ec, std::span<const uint8_t> response)
{
    if (ec)
    {
        lg2::error(
            "Error updating Transceiver Metrics: sending message over MCTP failed, rc={RC}, EID={EID}, PortNumber={PN}",
            "RC", ec.message(), "EID", eid, "PN", portNumber);
        readInflight = false;
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::OpticalModuleTelemetry page;

    const int rc = gpu::decodeQueryPortTelemetryV2OpticalModuleResponse(
        response, cc, reasonCode, page);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        // A copper cable answers with an error completion code on every poll.
        if (rc != 0 || cc != lastCompletionCode)
        {
            lg2::error(
                "Error updating Transceiver Metrics: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, PortNumber={PN}",
                "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode,
                "EID", eid, "PN", portNumber);
        }
        if (rc == 0)
        {
            lastCompletionCode = cc;
            if (pagesRequested == 1)
            {
                publish();
            }
        }
        readInflight = false;
        return;
    }

    lastCompletionCode = cc;

    mergeLanes(readings.txPower, page.txPower);
    mergeLanes(readings.rxPower, page.rxPower);
    mergeLanes(readings.txBiasCurrent, page.txBiasCurrent);
    if (page.txBiasScalingFactor)
    {
        readings.txBiasScalingFactor = page.txBiasScalingFactor;
    }

    if (page.nextSequenceToken != 0)
    {
        if (pagesRequested < maxPagesPerRead)
        {
            requestPage(page.nextSequenceToken);
            return;
        }

        lg2::error(
            "Error updating Transceiver Metrics: no last page after {PAGES} pages, EID={EID}, PortNumber={PN}",
            "PAGES", pagesRequested, "EID", eid, "PN", portNumber);
        readInflight = false;
        return;
    }

    publish();
    readInflight = false;
}

void NvidiaTransceiverMetrics::publish()
{
    const bool scalingFactorChanged =
        readings.txBiasScalingFactor != reported.txBiasScalingFactor;

    for (size_t lane = 0; lane < gpu::opticalModuleLaneCount; ++lane)
    {
        if (readings.txPower[lane] != reported.txPower[lane])
        {
            txPowerInterfaces[lane]->set_property(
                "Value", toWatts(readings.txPower[lane]));
        }

        if (readings.rxPower[lane] != reported.rxPower[lane])
        {
            rxPowerInterfaces[lane]->set_property(
                "Value", toWatts(readings.rxPower[lane]));
        }

        if (readings.txBiasCurrent[lane] != reported.txBiasCurrent[lane] ||
            (readings.txBiasCurrent[lane] && scalingFactorChanged))
        {
            txBiasCurrentInterfaces[lane]->set_property(
                "Value", toAmperes(readings.txBiasCurrent[lane],
                                   readings.txBiasScalingFactor));
        }
    }

    reported = readings;
}
