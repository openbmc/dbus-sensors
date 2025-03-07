/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include <gpu_common.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(GPUCommonPackTest, PackHeader)
{
	struct ocp_ami_binding_pci_vid_info hdr;
	struct ocp_ami_binding_pci_vid_info *hdr_ptr = NULL;

	struct ocp_ami_binding_pci_vid msg{};

	hdr.ocp_ami_msg_type = OCP_AMI_REQUEST;
	hdr.instance_id = 0x04;
	hdr.msg_type = 0x03;

	auto rc = gpu_pack_header(&hdr, &msg);
	EXPECT_EQ(rc, OCP_AMI_SUCCESS);
	EXPECT_EQ(msg.ocp_version, OCP_VERSION);

	// header information pointer is NULL
	rc = gpu_pack_header(hdr_ptr, &msg);
	EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

	// message pointer is NULL
	rc = gpu_pack_header(&hdr, nullptr);
	EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

	// header information pointer and message pointer is NULL
	rc = gpu_pack_header(hdr_ptr, nullptr);
	EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

	// Instance ID out of range
	hdr.instance_id = 32;
	rc = gpu_pack_header(&hdr, &msg);
	EXPECT_EQ(rc, OCP_AMI_ERR_INVALID_DATA);
}

class GPUCommonTest : public ::testing::Test {
    protected:
	ocp_ami_binding_pci_vid_info hdr;
	ocp_ami_msg *msg;
	std::vector<uint8_t> buf;
	uint8_t instance_id{};
	uint8_t type{};
	uint8_t command;
	uint8_t cc{};
	uint16_t reason_code{};
	uint16_t data_size{};
	size_t msg_len{};
	uint16_t pci_vendor_id = NVIDIA_PCI_VENDOR_ID;

	void SetUp() override
	{
		buf.resize(1024, 0);
		msg_len = buf.size();
		msg = new (buf.data()) ocp_ami_msg;
	}

	void setOcpVersionAndVendorId()
	{
		msg->hdr.ocp_type = OCP_TYPE;
		msg->hdr.ocp_version = OCP_VERSION;
		msg->hdr.pci_vendor_id = be16toh(NVIDIA_PCI_VENDOR_ID);
	}

	void changeVendorId()
	{
		msg->hdr.pci_vendor_id = 0x1234;
	}
};

TEST_F(GPUCommonTest, EncodeReasonCode)
{
	auto rc = gpu_encode_reason_code(cc, reason_code, command, msg);
	EXPECT_EQ(rc, OCP_AMI_SUCCESS);
}

TEST_F(GPUCommonTest, DecodeReasonCode)
{
	int rc{};

	setOcpVersionAndVendorId();
	rc = gpu_decode_reason_code_and_cc(msg, msg_len, &cc, &reason_code);
	EXPECT_EQ(rc, OCP_AMI_SUCCESS);

	changeVendorId();
	rc = gpu_decode_reason_code_and_cc(msg, msg_len, &cc, &reason_code);
	EXPECT_EQ(rc, OCP_AMI_ERR_INVALID_DATA);
}
