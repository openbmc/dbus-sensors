/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "ocp_ami.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(PackMessage, goodPathTest)
{
	struct ocp_ami_binding_pci_vid_info hdr;
	hdr.ocp_ami_msg_type = OCP_AMI_REQUEST;
	hdr.instance_id = 0;
	hdr.msg_type = 0x04;

	uint16_t pci_vendor_id{ 0x10de };

	struct ocp_ami_binding_pci_vid msg{};

	auto rc = ocp_ami_pack_header(pci_vendor_id, &hdr, &msg);
	EXPECT_EQ(rc, OCP_AMI_SUCCESS);

	EXPECT_EQ(msg.pci_vendor_id, htobe16(pci_vendor_id));
	EXPECT_EQ(msg.reserved, 0);
	EXPECT_EQ(msg.datagram, 0);
	EXPECT_EQ(msg.request, 1);
	EXPECT_EQ(msg.ocp_type, OCP_TYPE);
	EXPECT_EQ(msg.ocp_version, OCP_VERSION);
	EXPECT_EQ(msg.ocp_ami_msg_type, hdr.msg_type);
	EXPECT_EQ(msg.instance_id, hdr.instance_id);
}

TEST(PackMessage, badPathTest)
{
	struct ocp_ami_binding_pci_vid_info hdr;
	struct ocp_ami_binding_pci_vid_info *hdr_ptr = NULL;

	struct ocp_ami_binding_pci_vid msg{};
	uint16_t pci_vendor_id{};

	// header information pointer is NULL
	auto rc = ocp_ami_pack_header(pci_vendor_id, hdr_ptr, &msg);
	EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

	// message pointer is NULL
	rc = ocp_ami_pack_header(pci_vendor_id, &hdr, nullptr);
	EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

	// header information pointer and message pointer is NULL
	rc = ocp_ami_pack_header(pci_vendor_id, hdr_ptr, nullptr);
	EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

	// Instance ID out of range
	hdr.ocp_ami_msg_type = OCP_AMI_REQUEST;
	hdr.instance_id = 32;
	rc = ocp_ami_pack_header(pci_vendor_id, &hdr, &msg);
	EXPECT_EQ(rc, OCP_AMI_ERR_INVALID_DATA);
}

TEST(encodeReasonCode, testGoodEncodeReasonCode)
{
	std::vector<uint8_t> responseMsg(
		sizeof(ocp_ami_binding_pci_vid) +
		sizeof(ocp_ami_common_non_success_resp));
	auto response = new (responseMsg.data()) ocp_ami_msg;

	uint8_t cc = OCP_AMI_ERROR;
	uint16_t reasonCode = NONE;

	auto rc = ocp_ami_encode_reason_code(cc, reasonCode, 0x00, response);

	struct ocp_ami_common_non_success_resp *resp =
		new (response->payload) struct ocp_ami_common_non_success_resp;

	EXPECT_EQ(rc, OCP_AMI_SUCCESS);
	EXPECT_EQ(OCP_AMI_ERROR, resp->completion_code);
	EXPECT_EQ(0x00, resp->command);
	EXPECT_EQ(NONE, le16toh(resp->reason_code));
}

TEST(encodeReasonCode, testBadEncodeReasonCode)
{
	uint8_t cc = OCP_AMI_ERROR;
	uint16_t reasonCode = NONE;

	auto rc = ocp_ami_encode_reason_code(cc, reasonCode, 0x00, nullptr);

	EXPECT_EQ(rc, OCP_AMI_ERR_NULL);
}

TEST(decodeReasonCodeCC, testGoodDecodeReasonCode)
{
	std::vector<uint8_t> responseMsg{
		0x10,
		0xDE, // PCI VID
		0x00, // RQ=0, D=0, RSVD=0, INSTANCE_ID=0
		0x89, // OCP_TYPE=8, OCP_VER=9
		0x00, // MSG_TYPE
		0x09, // command
		0x01, // completion code != OCP_AMI_SUCCESS
		0x00, // reason code
		0x00
	};

	auto response = new (responseMsg.data()) ocp_ami_msg;
	size_t msg_len = responseMsg.size();

	uint8_t cc = OCP_AMI_ERROR;
	uint16_t reason_code = NONE;

	auto rc = ocp_ami_decode_reason_code_and_cc(response, msg_len, &cc,
						    &reason_code);

	EXPECT_EQ(rc, OCP_AMI_SUCCESS);
	EXPECT_EQ(cc, OCP_AMI_ERROR);
	EXPECT_EQ(reason_code, 0x0000);
}

TEST(decodeReasonCodeCC, testGoodDecodeCompletionCode)
{
	std::vector<uint8_t> responseMsg{
		0x10,
		0xDE, // PCI VID
		0x00, // RQ=0, D=0, RSVD=0, INSTANCE_ID=0
		0x89, // OCP_TYPE=8, OCP_VER=9
		0x00, // MSG_TYPE
		0x09, // command
		0x00, // completion code = OCP_AMI_SUCCESS
		0x00, // reason code
		0x02
	};

	auto response = new (responseMsg.data()) ocp_ami_msg;
	size_t msg_len = responseMsg.size();

	uint8_t cc = OCP_AMI_ERROR;
	uint16_t reason_code = NONE;

	auto rc = ocp_ami_decode_reason_code_and_cc(response, msg_len, &cc,
						    &reason_code);
	EXPECT_EQ(rc, OCP_AMI_SUCCESS);
	EXPECT_EQ(cc, OCP_AMI_SUCCESS);
	EXPECT_EQ(reason_code, NONE);
}

TEST(decodeReasonCode, testBadDecodeReasonCode)
{
	std::vector<uint8_t> responseMsg{
		0x10,
		0xDE, // PCI VID
		0x00, // RQ=0, D=0, RSVD=0, INSTANCE_ID=0
		0x89, // OCP_TYPE=8, OCP_VER=9
		0x00, // MSG_TYPE
		0x09, // command
		0x01, // completion code
		0x00, // reason code
		0x00
	};

	auto response = new (responseMsg.data()) ocp_ami_msg;
	size_t msg_len = responseMsg.size();

	uint8_t cc = OCP_AMI_SUCCESS;
	uint16_t reason_code = NONE;

	auto rc = ocp_ami_decode_reason_code_and_cc(nullptr, msg_len, &cc,
						    &reason_code);
	EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

	rc = ocp_ami_decode_reason_code_and_cc(response, msg_len, nullptr,
					       &reason_code);
	EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

	rc = ocp_ami_decode_reason_code_and_cc(response, msg_len, &cc, nullptr);
	EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

	rc = ocp_ami_decode_reason_code_and_cc(
		response, msg_len - 2, &cc,
		&reason_code); // sending msg len less then expected
	EXPECT_EQ(rc, OCP_AMI_ERR_INVALID_DATA_LENGTH);
}
