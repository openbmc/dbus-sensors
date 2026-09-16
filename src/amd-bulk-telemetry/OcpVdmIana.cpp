/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "OcpVdmIana.hpp"

#include <cstdint>
#include <span>

namespace ocp::bulk
{

namespace
{
// OCP's IANA Enterprise Number.
constexpr uint32_t ocpEnterpriseId = 0x0000A67F;

constexpr uint8_t ocpType = 1;
constexpr uint8_t ocpVersion = 1;

constexpr uint8_t vendorMessageType = 1;

constexpr uint8_t messageTypeMask = requestBit | datagramBit;
constexpr uint8_t instanceIdMask = 0x1F;
constexpr uint8_t ocpDesignatorBit = 0x80;
constexpr uint8_t ocpTypeBitOffset = 3;
constexpr uint8_t ocpTypeBitMask = 0b01111000;
constexpr uint8_t ocpVersionBitMask = 0b00000111;
} // namespace

bool encodeHeader(const Header& header, std::span<uint8_t> buf)
{
    if (buf.size() < messageHeaderSize)
    {
        return false;
    }

    buf[0] = static_cast<uint8_t>(ocpEnterpriseId >> 24);
    buf[1] = static_cast<uint8_t>(ocpEnterpriseId >> 16);
    buf[2] = static_cast<uint8_t>(ocpEnterpriseId >> 8);
    buf[3] = static_cast<uint8_t>(ocpEnterpriseId);
    buf[4] = static_cast<uint8_t>(static_cast<uint8_t>(header.type) |
                                  (header.instanceId & instanceIdMask));
    buf[5] = static_cast<uint8_t>(
        ocpDesignatorBit | ((ocpType << ocpTypeBitOffset) & ocpTypeBitMask) |
        (ocpVersion & ocpVersionBitMask));
    buf[6] = vendorMessageType;

    return true;
}

bool decodeHeader(std::span<const uint8_t> buf, Header& header)
{
    if (buf.size() < messageHeaderSize)
    {
        return false;
    }

    const uint32_t enterpriseId =
        (uint32_t{buf[0]} << 24) | (uint32_t{buf[1]} << 16) |
        (uint32_t{buf[2]} << 8) | uint32_t{buf[3]};
    if (enterpriseId != ocpEnterpriseId)
    {
        return false;
    }

    if ((buf[5] & ocpDesignatorBit) == 0 ||
        (buf[5] & ocpTypeBitMask) != (ocpType << ocpTypeBitOffset) ||
        (buf[5] & ocpVersionBitMask) != ocpVersion)
    {
        return false;
    }

    if (buf[6] != vendorMessageType)
    {
        return false;
    }

    header.type = static_cast<MessageType>(buf[4] & messageTypeMask);
    header.instanceId = buf[4] & instanceIdMask;
    return true;
}

} // namespace ocp::bulk
