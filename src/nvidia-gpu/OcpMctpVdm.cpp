/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <bit>
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
} // namespace accelerator_management
} // namespace ocp
