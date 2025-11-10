/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "OcpMctpVdm.hpp"

#include "MessagePackUnpackUtils.hpp"

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

int packHeader(PackBuffer& buffer, uint16_t pciVendorId,
               MessageType ocpAcceleratorManagementMsgType, uint8_t instanceId,
               uint8_t msgType)
{
    if (ocpAcceleratorManagementMsgType != MessageType::RESPONSE &&
        ocpAcceleratorManagementMsgType != MessageType::REQUEST)
    {
        return EINVAL;
    }

    if (instanceId > instanceMax)
    {
        return EINVAL;
    }

    uint8_t messageInstanceId = instanceId & instanceIdBitMask;

    if (ocpAcceleratorManagementMsgType == MessageType::REQUEST)
    {
        messageInstanceId |= requestBitMask;
    }
    else
    {
        messageInstanceId &= ~requestBitMask;
    }

    messageInstanceId &= ~instanceIdReservedBitMask;

    uint8_t ocpVersionAndType = ocpVersion & ocpVersionBitMask;
    ocpVersionAndType |= (ocpType << ocpTypeBitOffset) & ocpTypeBitMask;

    buffer.pack(htobe16(pciVendorId));
    buffer.pack(messageInstanceId);
    buffer.pack(ocpVersionAndType);
    buffer.pack(msgType);

    return 0;
}

int unpackHeader(UnpackBuffer& buffer, uint16_t pciVendorId,
                 MessageType& ocpAcceleratorManagementMsgType,
                 uint8_t& instanceId, uint8_t& msgType)
{
    uint16_t receivedPciVendorId = 0;
    uint8_t messageInstanceId = 0;
    uint8_t ocpVersionAndType = 0;

    buffer.unpack(receivedPciVendorId);
    buffer.unpack(messageInstanceId);
    buffer.unpack(ocpVersionAndType);
    buffer.unpack(msgType);

    if (buffer.getError() != 0)
    {
        return buffer.getError();
    }

    if (receivedPciVendorId != htobe16(pciVendorId))
    {
        return EINVAL;
    }

    instanceId = messageInstanceId & instanceIdBitMask;

    if ((messageInstanceId & requestBitMask) != 0)
    {
        ocpAcceleratorManagementMsgType = MessageType::REQUEST;
    }
    else
    {
        ocpAcceleratorManagementMsgType = MessageType::RESPONSE;
    }

    if ((messageInstanceId & instanceIdReservedBitMask) != 0)
    {
        return EINVAL;
    }

    if ((ocpVersionAndType & ocpVersionBitMask) != ocpVersion)
    {
        return EINVAL;
    }

    if ((ocpVersionAndType & ocpTypeBitMask) != (ocpType << ocpTypeBitOffset))
    {
        return EINVAL;
    }

    return 0;
}

int unpackReasonCodeAndCC(UnpackBuffer& buffer, CompletionCode& cc,
                          uint16_t& reasonCode)
{
    uint8_t completionCode = 0;
    uint16_t receivedReasonCode = 0;

    buffer.unpack(completionCode);
    buffer.unpack(receivedReasonCode);

    if (buffer.getError() != 0)
    {
        return buffer.getError();
    }

    cc = static_cast<CompletionCode>(completionCode);

    if (cc == CompletionCode::SUCCESS)
    {
        reasonCode = 0;
    }
    else
    {
        reasonCode = receivedReasonCode;
    }

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

    // These expression decodes data communicated over the network
    // The use of reinterpret_cast enables direct memory access to raw byte
    // buffers without doing unnecessary data copying
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    const auto* response = reinterpret_cast<
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
