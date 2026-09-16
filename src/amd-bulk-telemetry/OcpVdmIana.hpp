/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpAsyncRequester.hpp>

#include <cstddef>
#include <cstdint>
#include <span>

namespace ocp::bulk
{

// DSP0236 vendor-defined, IANA binding.
constexpr uint8_t mctpMessageType = 0x7F;

// IANA(4) + Rq/D/IID(1) + OCP type and version(1) + vendor message type(1).
constexpr size_t messageHeaderSize = 7;
constexpr size_t instanceIdOffset = 4;

inline constexpr mctp::VdmBinding mctpBinding{
    .msgType = mctpMessageType,
    .headerSize = messageHeaderSize,
    .instanceIdOffset = instanceIdOffset,
};

constexpr uint8_t requestBit = 0x80;
constexpr uint8_t datagramBit = 0x40;

enum class MessageType : uint8_t
{
    request = requestBit,
    response = 0,
    event = requestBit | datagramBit,
    eventAcknowledgment = datagramBit,
};

struct Header
{
    MessageType type;
    uint8_t instanceId;
};

enum class CompletionCode : uint8_t
{
    success = 0x00,
    error = 0x01,
    errInvalidData = 0x02,
    errInvalidDataLength = 0x03,
    errNotReady = 0x04,
    errUnsupportedCommandCode = 0x05,
    errUnsupportedMsgType = 0x06,
    accepted = 0x7D,
    errBusy = 0x7E,
    errBusAccess = 0x7F,
};

bool encodeHeader(const Header& header, std::span<uint8_t> buf);

bool decodeHeader(std::span<const uint8_t> buf, Header& header);

} // namespace ocp::bulk
