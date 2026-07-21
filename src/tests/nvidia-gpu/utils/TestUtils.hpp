/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace test_utils
{

constexpr uint8_t defaultEid = 10;

// Header(5) + command(1) + cc(1) + reasonCode(2)
constexpr size_t errorResponseSize =
    ocp::accelerator_management::messageHeaderSize + 4;

// Build a SUCCESS PLATFORM_ENVIRONMENTAL response with a single integral
// payload appended after the common response header. Used by the temperature,
// power, energy, voltage and peak-power sensor tests.
template <std::integral Payload>
inline std::vector<uint8_t> buildPlatformEnvSuccessResponse(
    gpu::PlatformEnvironmentalCommands command, Payload payload)
{
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + sizeof(Payload));
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(command));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(sizeof(Payload)));
    pack.pack(payload);
    return buf;
}

// Build an error PLATFORM_ENVIRONMENTAL response: header + command + cc +
// reasonCode (no dataSize / payload).
inline std::vector<uint8_t> buildPlatformEnvErrorResponse(
    gpu::PlatformEnvironmentalCommands command, uint8_t cc, uint16_t reasonCode)
{
    std::vector<uint8_t> buf(errorResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(command));
    pack.pack(cc);
    pack.pack(reasonCode);
    return buf;
}

// Build a SUCCESS QueryScalarGroupTelemetryV2 response on the PCIE_LINK
// message type with the supplied uint32_t telemetry values appended after the
// common response header.
inline std::vector<uint8_t> buildPcieScalarTelemetryResponse(
    const std::vector<uint32_t>& values)
{
    const size_t dataSize = values.size() * sizeof(uint32_t);
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + dataSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    pack.pack(static_cast<uint8_t>(
        gpu::PcieLinkCommands::QueryScalarGroupTelemetryV2));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(dataSize));
    for (const uint32_t val : values)
    {
        pack.pack(val);
    }
    return buf;
}

} // namespace test_utils
