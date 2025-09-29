/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <cerrno>
#include <cstdint>
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

    msg.instance_id = hdr.instance_id;

    if (hdr.ocp_accelerator_management_msg_type ==
        static_cast<uint8_t>(MessageType::REQUEST))
    {
        msg.request_or_response = 1;
    }
    else
    {
        msg.request_or_response = 0;
    }

    msg.pci_vendor_id = htobe16(pciVendorId);
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
} // namespace accelerator_management
} // namespace ocp
