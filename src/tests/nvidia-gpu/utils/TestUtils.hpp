/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
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

// Build a supported message type or command code bitfield with the bit of
// every listed code set.
inline std::array<uint8_t, gpu::supportedListBitfieldSize> bitsOf(
    std::initializer_list<uint8_t> codes)
{
    std::array<uint8_t, gpu::supportedListBitfieldSize> bits{};
    for (uint8_t code : codes)
    {
        bits[code / 8U] |= static_cast<uint8_t>(1U << (code % 8U));
    }
    return bits;
}

// Build a SUCCESS DEVICE_CAPABILITY_DISCOVERY response carrying a supported
// message type or command code bitfield.
inline std::vector<uint8_t> buildSupportedListResponse(
    gpu::DeviceCapabilityDiscoveryCommands command,
    const std::array<uint8_t, gpu::supportedListBitfieldSize>& bits)
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize +
                             gpu::supportedListBitfieldSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    pack.pack(static_cast<uint8_t>(command));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(gpu::supportedListBitfieldSize));
    for (uint8_t byte : bits)
    {
        pack.pack(byte);
    }
    return buf;
}

// Build a SUCCESS DEVICE_CAPABILITY_DISCOVERY response with no data.
inline std::vector<uint8_t> buildCapabilityDiscoverySuccessResponse(
    uint8_t command)
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    pack.pack(command);
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(0)); // data size
    return buf;
}

// Event class the device uses for a deferred long-running response.
constexpr uint8_t longRunningResponseEventClass = 128;

// Build a long-running response event data buffer:
//   instanceId(1) + completionCode(1) + reasonCode(2 LE) + payload
inline std::vector<uint8_t> buildLongRunningEventData(
    uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
    const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> buf;
    buf.push_back(instanceId);
    buf.push_back(cc);
    buf.push_back(static_cast<uint8_t>(reasonCode & 0xFF));
    buf.push_back(static_cast<uint8_t>((reasonCode >> 8) & 0xFF));
    buf.insert(buf.end(), payload.begin(), payload.end());
    return buf;
}

// Build the EventInfo carrying a long-running response: the originating
// command and message type are packed into eventState.
inline EventInfo makeEventInfo(uint8_t eventClass, uint8_t messageType,
                               uint8_t command)
{
    EventInfo info{};
    info.eventClass = eventClass;
    info.eventState = static_cast<uint16_t>(
        (static_cast<uint16_t>(command) << 8) | messageType);
    return info;
}

} // namespace test_utils
