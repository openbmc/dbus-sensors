/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <bit>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>

namespace ocp
{
namespace accelerator_management
{

int packHeader(uint16_t pciVendorId, const BindingPciVidInfo& hdr,
               BindingPciVid& msg)
{
    if (hdr.ocp_accelerator_management_msg_type !=
            static_cast<uint8_t>(MessageType::RESPONSE) &&
        hdr.ocp_accelerator_management_msg_type !=
            static_cast<uint8_t>(MessageType::REQUEST))
    {
        return EINVAL;
    }

    if (hdr.instance_id > instanceMax)
    {
        return EINVAL;
    }

    msg.instance_id = hdr.instance_id & instanceIdBitMask;

    if (hdr.ocp_accelerator_management_msg_type ==
        static_cast<uint8_t>(MessageType::REQUEST))
    {
        msg.instance_id |= requestBitMask;
    }
    else
    {
        msg.instance_id &= ~requestBitMask;
    }

    msg.pci_vendor_id = htobe16(pciVendorId);
    msg.instance_id &= ~instanceIdReservedBitMask;
    msg.ocp_version = ocpVersion & ocpVersionBitMask;
    msg.ocp_version |= (ocpType << ocpTypeBitOffset) & ocpTypeBitMask;
    msg.ocp_accelerator_management_msg_type = hdr.msg_type;

    return 0;
}

int decodeReasonCodeAndCC(const std::span<const uint8_t> buf,
                          CompletionCode& cc, uint16_t& reasonCode)
{
    if (buf.size() <
        sizeof(ocp::accelerator_management::CommonNonSuccessResponse))
    {
        return EINVAL;
    }

    const auto* response = std::bit_cast<
        const ocp::accelerator_management::CommonNonSuccessResponse*>(
        buf.data());

    cc = static_cast<CompletionCode>(response->completion_code);
    if (cc == CompletionCode::SUCCESS)
    {
        reasonCode = 0;
        return 0;
    }

    // reason code is expected to be present if CC != SUCCESS
    reasonCode = le16toh(response->reason_code);

    return 0;
}

int decodeAggregateResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    std::move_only_function<int(const uint8_t tag, const uint8_t length,
                                const uint8_t* value)>
        handler)
{
    auto rc =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (buf.size() <
        sizeof(ocp::accelerator_management::CommonAggregateResponse))
    {
        return EINVAL;
    }

    const auto* response = std::bit_cast<
        const ocp::accelerator_management::CommonAggregateResponse*>(
        buf.data());

    size_t index = sizeof(ocp::accelerator_management::CommonAggregateResponse);

    for (uint16_t telemetryCount = 0;
         telemetryCount < le16toh(response->telemetryCount); ++telemetryCount)
    {
        const size_t valueOffset = index + 2;

        if (buf.size() < valueOffset)
        {
            break;
        }

        const uint8_t tag = buf[index];
        const uint8_t tagInfo = buf[index + 1];
        const bool isValid = (0x01 & tagInfo) != 0;
        const bool isByteLengthEncoding = (0x80 & tagInfo) != 0;
        const uint8_t encodedLength = (tagInfo >> 1) & 0x07;

        uint8_t length = 0;
        if (isByteLengthEncoding)
        {
            length = encodedLength;
        }
        else
        {
            length = 1 << encodedLength;
        }

        index = valueOffset + length;

        if (isValid)
        {
            if (buf.size() < index)
            {
                break;
            }

            handler(tag, length, buf.data() + valueOffset);
        }
    }

    return 0;
}

} // namespace accelerator_management
} // namespace ocp
