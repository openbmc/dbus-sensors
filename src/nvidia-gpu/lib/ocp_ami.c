/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "ocp_ami.h"

#include <endian.h>
#include <string.h>

uint8_t ocp_ami_pack_header(uint16_t pci_vendor_id,
			    const struct ocp_ami_binding_pci_vid_info *hdr,
			    struct ocp_ami_binding_pci_vid *msg)
{
	if (msg == NULL || hdr == NULL) {
		return OCP_AMI_ERR_NULL;
	}

	if (hdr->ocp_ami_msg_type != OCP_AMI_RESPONSE &&
	    hdr->ocp_ami_msg_type != OCP_AMI_REQUEST &&
	    hdr->ocp_ami_msg_type != OCP_AMI_EVENT &&
	    hdr->ocp_ami_msg_type != OCP_AMI_EVENT_ACKNOWLEDGMENT) {
		return OCP_AMI_ERR_INVALID_DATA;
	}

	if (hdr->instance_id > OCP_AMI_INSTANCE_MAX) {
		return OCP_AMI_ERR_INVALID_DATA;
	}

	msg->datagram = 0;
	if (hdr->ocp_ami_msg_type == OCP_AMI_EVENT_ACKNOWLEDGMENT ||
	    hdr->ocp_ami_msg_type == OCP_AMI_EVENT) {
		msg->datagram = 1;
	}

	msg->request = 0;
	if (hdr->ocp_ami_msg_type == OCP_AMI_REQUEST ||
	    hdr->ocp_ami_msg_type == OCP_AMI_EVENT) {
		msg->request = 1;
	}

	msg->pci_vendor_id = htobe16(pci_vendor_id);
	msg->reserved = 0;
	msg->instance_id = hdr->instance_id;
	msg->ocp_type = OCP_TYPE;
	msg->ocp_version = OCP_VERSION;
	msg->ocp_ami_msg_type = hdr->msg_type;

	return OCP_AMI_SUCCESS;
}

int ocp_ami_encode_reason_code(uint8_t cc, uint16_t reason_code,
			       uint8_t command_code, struct ocp_ami_msg *msg)
{
	if (msg == NULL) {
		return OCP_AMI_ERR_NULL;
	}

	struct ocp_ami_common_non_success_resp *response =
		(struct ocp_ami_common_non_success_resp *)msg->payload;

	response->command = command_code;
	response->completion_code = cc;
	reason_code = htole16(reason_code);
	response->reason_code = reason_code;

	return OCP_AMI_SUCCESS;
}

int ocp_ami_decode_reason_code_and_cc(const struct ocp_ami_msg *msg,
				      size_t msg_len, uint8_t *cc,
				      uint16_t *reason_code)
{
	if (msg == NULL || cc == NULL || reason_code == NULL) {
		return OCP_AMI_ERR_NULL;
	}

	*cc = ((struct ocp_ami_common_resp *)msg->payload)->completion_code;
	if (*cc == OCP_AMI_SUCCESS) {
		return OCP_AMI_SUCCESS;
	}

	if (msg_len != (sizeof(struct ocp_ami_binding_pci_vid) +
			sizeof(struct ocp_ami_common_non_success_resp))) {
		return OCP_AMI_ERR_INVALID_DATA_LENGTH;
	}

	struct ocp_ami_common_non_success_resp *response =
		(struct ocp_ami_common_non_success_resp *)msg->payload;

	// reason code is expected to be present if CC != OCP_AMI_SUCCESS
	*reason_code = le16toh(response->reason_code);

	return OCP_AMI_SUCCESS;
}
