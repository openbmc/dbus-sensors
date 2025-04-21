/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <cstdint>
#include <cstring>

namespace ocp
{
namespace accelerator_management
{

CompletionCode packHeader(uint16_t pciVendorId, const BindingPciVidInfo& hdr,
                          BindingPciVid& msg)
{
    if (hdr.ocp_accelerator_management_msg_type !=
            static_cast<uint8_t>(MessageType::RESPONSE) &&
        hdr.ocp_accelerator_management_msg_type !=
            static_cast<uint8_t>(MessageType::REQUEST))
    {
        return CompletionCode::ERR_INVALID_DATA;
    }

    if (hdr.instance_id > instanceMax)
    {
        return CompletionCode::ERR_INVALID_DATA;
    }

    msg.datagram = 0;

    msg.request = 0;
    if (hdr.ocp_accelerator_management_msg_type ==
        static_cast<uint8_t>(MessageType::REQUEST))
    {
        msg.request = 1;
    }

    msg.pci_vendor_id = htobe16(pciVendorId);
    msg.reserved = 0;
    msg.instance_id = hdr.instance_id;
    msg.ocp_type = type;
    msg.ocp_version = version;
    msg.ocp_accelerator_management_msg_type = hdr.msg_type;

    return CompletionCode::SUCCESS;
}

CompletionCode encodeReasonCode(uint8_t cc, uint16_t reasonCode,
                                uint8_t commandCode, Message& msg)
{
    CommonNonSuccessResponse response{};
    response.command = commandCode;
    response.completion_code = cc;
    reasonCode = htole16(reasonCode);
    response.reason_code = reasonCode;

    std::memcpy(&msg.data, &response, sizeof(response));

    return CompletionCode::SUCCESS;
}

CompletionCode decodeReasonCodeAndCC(const Message& msg, size_t msgLen,
                                     uint8_t& cc, uint16_t& reasonCode)
{
    CommonNonSuccessResponse response{};
    std::memcpy(&response, &msg.data, sizeof(response));

    cc = response.completion_code;
    if (cc == static_cast<uint8_t>(CompletionCode::SUCCESS))
    {
        return CompletionCode::SUCCESS;
    }

    if (msgLen != (sizeof(BindingPciVid) + sizeof(CommonNonSuccessResponse)))
    {
        return CompletionCode::ERR_INVALID_DATA_LENGTH;
    }

    // reason code is expected to be present if CC != SUCCESS
    reasonCode = le16toh(response.reason_code);

    return CompletionCode::SUCCESS;
}
} // namespace accelerator_management
} // namespace ocp
