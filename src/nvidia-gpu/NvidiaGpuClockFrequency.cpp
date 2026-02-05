/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuClockFrequency.hpp"

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

NvidiaGpuClockFrequency::NvidiaGpuClockFrequency(
    mctp::MctpRequester& mctpRequester, const std::string& name, uint8_t eid,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>&
        operatingConfigIface) :
    mctpRequester(mctpRequester), name(name), eid(eid),
    operatingConfigInterface(operatingConfigIface)
{}

void NvidiaGpuClockFrequency::update()
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
                lg2::error("Invalid NvidiaGpuClockFrequency reference");
                return;
            }
            self->handleResponse(ec, buffer);
        });
}

void NvidiaGpuClockFrequency::handleResponse(const std::error_code& ec,
                                             std::span<const uint8_t> buffer)
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
    uint32_t clockFrequency = 0;

    int rc = gpu::decodeGetCurrentClockFrequencyResponse(buffer, cc, reasonCode,
                                                         clockFrequency);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding clock frequency for {NAME}: rc={RC}, cc={CC}, reasonCode={REASON}",
            "NAME", name, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    operatingConfigInterface->set_property("OperatingSpeed", clockFrequency);
}
