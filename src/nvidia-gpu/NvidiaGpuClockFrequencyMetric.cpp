/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuClockFrequencyMetric.hpp"

#include "NvidiaUtils.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

using namespace std::literals;

static constexpr uint32_t mhzToHzFactor = 1'000'000;

NvidiaGpuClockFrequencyMetric::NvidiaGpuClockFrequencyMetric(
    mctp::MctpRequester& mctpRequester, const std::string& name, uint8_t eid,
    sdbusplus::asio::object_server& objectServer,
    const std::string& inventoryPath) :
    mctpRequester(mctpRequester), name(name), eid(eid),
    objectServer(objectServer)
{
    const std::string metricDbusPath =
        metricPath + this->name + "/OperatingFrequency";

    metricInterface = objectServer.add_interface(
        metricDbusPath, "xyz.openbmc_project.Metric.Value");
    metricInterface->register_property(
        "Unit", "xyz.openbmc_project.Metric.Value.Unit.Frequency"s);
    metricInterface->register_property("Value", 0.0);
    metricInterface->initialize();

    associationInterface =
        objectServer.add_interface(metricDbusPath, association::interface);
    std::vector<Association> associations;
    associations.emplace_back("measuring", "measured_by", inventoryPath);
    associationInterface->register_property("Associations", associations);
    associationInterface->initialize();
}

NvidiaGpuClockFrequencyMetric::~NvidiaGpuClockFrequencyMetric()
{
    objectServer.remove_interface(metricInterface);
    objectServer.remove_interface(associationInterface);
}

void NvidiaGpuClockFrequencyMetric::update()
{
    int rc = gpu::encodeGetCurrentClockFrequencyRequest(
        0, gpu::ClockType::GRAPHICS_CLOCK, requestBuffer);
    if (rc != 0)
    {
        lg2::error(
            "Failed to encode clock frequency request for {NAME}: rc={RC}",
            "NAME", name, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, requestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaGpuClockFrequencyMetric reference");
                return;
            }
            self->handleResponse(ec, buffer);
        });
}

void NvidiaGpuClockFrequencyMetric::handleResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error reading clock frequency for {NAME}: MCTP failed, rc={RC}",
            "NAME", name, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t clockFrequencyMhz = 0;

    int rc = gpu::decodeGetCurrentClockFrequencyResponse(buffer, cc, reasonCode,
                                                         clockFrequencyMhz);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding clock frequency for {NAME}: rc={RC}, cc={CC}, reasonCode={REASON}",
            "NAME", name, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    double freqHz = static_cast<double>(clockFrequencyMhz) * mhzToHzFactor;
    metricInterface->set_property("Value", freqHz);
}
