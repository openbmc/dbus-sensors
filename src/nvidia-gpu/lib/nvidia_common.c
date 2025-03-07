/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include <nvidia_common.h>

#include <endian.h>

uint8_t
ocp_ami_oem_nvidia_pack_header(const struct ocp_ami_binding_pci_vid_info *hdr,
			       struct ocp_ami_binding_pci_vid *msg)
{
	return ocp_ami_pack_header(NVIDIA_PCI_VENDOR_ID, hdr, msg);
}

int ocp_ami_oem_nvidia_encode_reason_code(uint8_t cc, uint16_t reason_code,
					  uint8_t command_code,
					  struct ocp_ami_msg *msg)
{
	return ocp_ami_encode_reason_code(cc, reason_code, command_code, msg);
}

int ocp_ami_oem_nvidia_decode_reason_code_and_cc(const struct ocp_ami_msg *msg,
						 size_t msg_len, uint8_t *cc,
						 uint16_t *reason_code)
{
	if (msg != NULL &&
	    be16toh(msg->hdr.pci_vendor_id) != NVIDIA_PCI_VENDOR_ID) {
		return OCP_AMI_ERR_INVALID_DATA;
	}

	return ocp_ami_decode_reason_code_and_cc(msg, msg_len, cc, reason_code);
}
