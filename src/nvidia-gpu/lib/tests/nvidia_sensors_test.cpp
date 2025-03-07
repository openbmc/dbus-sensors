/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include <nvidia_sensors.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(queryDeviceIdentification, testGoodEncodeRequest)
{
	std::vector<uint8_t> requestMsg(
		sizeof(ocp_ami_binding_pci_vid) +
		sizeof(ocp_ami_oem_nvidia_query_device_identification_req));

	auto request = reinterpret_cast<ocp_ami_msg *>(requestMsg.data());
	uint8_t instance_id = 0x12;
	auto rc = ocp_ami_oem_nvidia_encode_query_device_identification_req(
		instance_id, request);

	struct ocp_ami_common_req *req =
		reinterpret_cast<struct ocp_ami_common_req *>(request->payload);

	EXPECT_EQ(rc, OCP_AMI_SUCCESS);

	EXPECT_EQ(1, request->hdr.request);
	EXPECT_EQ(0, request->hdr.datagram);
	EXPECT_EQ(instance_id, request->hdr.instance_id);
	EXPECT_EQ(OCP_AMI_OEM_NVIDIA_TYPE_DEVICE_CAPABILITY_DISCOVERY,
		  request->hdr.ocp_ami_msg_type);

	EXPECT_EQ(OCP_AMI_OEM_NVIDIA_QUERY_DEVICE_IDENTIFICATION, req->command);
	EXPECT_EQ(0, req->data_size);
}

TEST(queryDeviceIdentification, testGoodEncodeResponse)
{
	std::vector<uint8_t> responseMsg(
		sizeof(ocp_ami_binding_pci_vid) +
		sizeof(ocp_ami_oem_nvidia_query_device_identification_resp));
	auto response = reinterpret_cast<ocp_ami_msg *>(responseMsg.data());

	uint8_t instance_id = 0x12;
	uint8_t cc = OCP_AMI_SUCCESS;
	uint16_t reason_code = NONE;
	uint8_t device_identification = OCP_AMI_OEM_NVIDIA_DEV_ID_GPU;
	uint8_t device_instance = 1;

	auto rc = ocp_ami_oem_nvidia_encode_query_device_identification_resp(
		instance_id, cc, reason_code, device_identification,
		device_instance, response);

	struct ocp_ami_oem_nvidia_query_device_identification_resp *resp =
		reinterpret_cast<
			struct ocp_ami_oem_nvidia_query_device_identification_resp
				*>(response->payload);

	EXPECT_EQ(rc, OCP_AMI_SUCCESS);

	EXPECT_EQ(0, response->hdr.request);
	EXPECT_EQ(0, response->hdr.datagram);
	EXPECT_EQ(instance_id, response->hdr.instance_id);
	EXPECT_EQ(OCP_AMI_OEM_NVIDIA_TYPE_DEVICE_CAPABILITY_DISCOVERY,
		  response->hdr.ocp_ami_msg_type);

	EXPECT_EQ(OCP_AMI_OEM_NVIDIA_QUERY_DEVICE_IDENTIFICATION,
		  resp->hdr.command);
	EXPECT_EQ(2, le16toh(resp->hdr.data_size));
	EXPECT_EQ(device_identification, resp->device_identification);
	EXPECT_EQ(device_instance, resp->instance_id);
}

TEST(queryDeviceIdentification, testGoodDecodeResponse)
{
	std::vector<uint8_t> responseMsg{
		0x10,
		0xDE, // PCI VID: NVIDIA 0x10DE
		0x00, // RQ=0, D=0, RSVD=0, INSTANCE_ID=0
		0x89, // OCP_TYPE=8, OCP_VER=9
		OCP_AMI_OEM_NVIDIA_TYPE_DEVICE_CAPABILITY_DISCOVERY, // NVIDIA_MSG_TYPE
		OCP_AMI_OEM_NVIDIA_QUERY_DEVICE_IDENTIFICATION, // command
		0, // completion code
		0,
		0,
		2,
		0,			       // data size
		OCP_AMI_OEM_NVIDIA_DEV_ID_GPU, // device _identification
		1			       // instance_id
	};

	auto response = reinterpret_cast<ocp_ami_msg *>(responseMsg.data());

	size_t msg_len = responseMsg.size();

	uint8_t cc = OCP_AMI_SUCCESS;
	uint16_t reason_code = NONE;
	uint8_t device_identification = 0;
	uint8_t device_instance = 0;

	auto rc = ocp_ami_oem_nvidia_decode_query_device_identification_resp(
		response, msg_len, &cc, &reason_code, &device_identification,
		&device_instance);

	EXPECT_EQ(rc, OCP_AMI_SUCCESS);
	EXPECT_EQ(cc, OCP_AMI_SUCCESS);
	EXPECT_EQ(OCP_AMI_OEM_NVIDIA_DEV_ID_GPU, device_identification);
	EXPECT_EQ(1, device_instance);
}

TEST(getTemperature, testGoodEncodeRequest)
{
	std::vector<uint8_t> requestMsg(
		sizeof(ocp_ami_binding_pci_vid) +
		sizeof(ocp_ami_oem_nvidia_get_temperature_reading_req));

	auto request = reinterpret_cast<ocp_ami_msg *>(requestMsg.data());
	uint8_t sensor_id = 0;

	auto rc = ocp_ami_oem_nvidia_encode_get_temperature_reading_req(
		0, sensor_id, request);

	ocp_ami_oem_nvidia_get_temperature_reading_req *req = reinterpret_cast<
		ocp_ami_oem_nvidia_get_temperature_reading_req *>(
		request->payload);

	EXPECT_EQ(rc, OCP_AMI_SUCCESS);

	EXPECT_EQ(1, request->hdr.request);
	EXPECT_EQ(0, request->hdr.datagram);
	EXPECT_EQ(OCP_AMI_OEM_NVIDIA_MSG_TYPE_PLATFORM_ENVIRONMENTAL,
		  request->hdr.ocp_ami_msg_type);

	EXPECT_EQ(OCP_AMI_OEM_NVIDIA_GET_TEMPERATURE_READING, req->hdr.command);
	EXPECT_EQ(sizeof(sensor_id), req->hdr.data_size);
	EXPECT_EQ(sensor_id, req->sensor_id);
}

TEST(getTemperature, testGoodDecodeRequest)
{
	std::vector<uint8_t> requestMsg{
		0x10,
		0xDE, // PCI VID: NVIDIA 0x10DE
		0x80, // RQ=1, D=0, RSVD=0, INSTANCE_ID=0
		0x89, // OCP_TYPE=8, OCP_VER=9
		OCP_AMI_OEM_NVIDIA_MSG_TYPE_PLATFORM_ENVIRONMENTAL, // NVIDIA_MSG_TYPE
		OCP_AMI_OEM_NVIDIA_GET_TEMPERATURE_READING,	    // command
		1,						    // data size
		1						    // sensor_id
	};

	auto request = reinterpret_cast<ocp_ami_msg *>(requestMsg.data());
	size_t msg_len = requestMsg.size();

	uint8_t sensor_id = 0;
	auto rc = ocp_ami_oem_nvidia_decode_get_temperature_reading_req(
		request, msg_len, &sensor_id);

	EXPECT_EQ(rc, OCP_AMI_SUCCESS);
	EXPECT_EQ(sensor_id, 1);
}

TEST(getTemperature, testGoodEncodeResponse)
{
	std::vector<uint8_t> responseMsg(
		sizeof(ocp_ami_binding_pci_vid) +
		sizeof(ocp_ami_oem_nvidia_get_temperature_reading_resp));
	auto response = reinterpret_cast<ocp_ami_msg *>(responseMsg.data());

	double temperature_reading = 12.34;
	uint16_t reasonCode = 0;

	auto rc = ocp_ami_oem_nvidia_encode_get_temperature_reading_resp(
		0, OCP_AMI_SUCCESS, reasonCode, temperature_reading, response);

	struct ocp_ami_oem_nvidia_get_temperature_reading_resp *resp =
		reinterpret_cast<
			struct ocp_ami_oem_nvidia_get_temperature_reading_resp *>(
			response->payload);

	EXPECT_EQ(rc, OCP_AMI_SUCCESS);

	EXPECT_EQ(0, response->hdr.request);
	EXPECT_EQ(0, response->hdr.datagram);
	EXPECT_EQ(OCP_AMI_OEM_NVIDIA_MSG_TYPE_PLATFORM_ENVIRONMENTAL,
		  response->hdr.ocp_ami_msg_type);

	EXPECT_EQ(OCP_AMI_OEM_NVIDIA_GET_TEMPERATURE_READING,
		  resp->hdr.command);
	EXPECT_EQ(sizeof(resp->reading), le16toh(resp->hdr.data_size));

	uint32_t data = 0;
	memcpy(&data, &resp->reading, sizeof(uint32_t));
	data = le32toh(data);
	double reading = data / (double)(1 << 8);
	EXPECT_NEAR(temperature_reading, reading, 0.01);
}

TEST(getTemperature, testGoodDecodeResponse)
{
	std::vector<uint8_t> responseMsg{
		0x10,
		0xDE, // PCI VID: NVIDIA 0x10DE
		0x00, // RQ=0, D=0, RSVD=0, INSTANCE_ID=0
		0x89, // OCP_TYPE=8, OCP_VER=9
		OCP_AMI_OEM_NVIDIA_MSG_TYPE_PLATFORM_ENVIRONMENTAL, // NVIDIA_MSG_TYPE
		OCP_AMI_OEM_NVIDIA_GET_TEMPERATURE_READING,	    // command
		0, // completion code
		0,
		0,
		4,
		0, // data size
		0x57,
		0x0c,
		0x00,
		0x00 // temperature reading=12.34
	};

	auto response = reinterpret_cast<ocp_ami_msg *>(responseMsg.data());
	size_t msg_len = responseMsg.size();

	uint8_t cc = OCP_AMI_ERROR;
	uint16_t reasonCode = NONE;
	double temperature_reading = 0;

	auto rc = ocp_ami_oem_nvidia_decode_get_temperature_reading_resp(
		response, msg_len, &cc, &reasonCode, &temperature_reading);

	EXPECT_EQ(rc, OCP_AMI_SUCCESS);
	EXPECT_EQ(cc, OCP_AMI_SUCCESS);
	EXPECT_NEAR(temperature_reading, 12.34, 0.01);
}

TEST(getTemperature, testBadDecodeResponseLength)
{
	std::vector<uint8_t> responseMsg{
		0x10,
		0xDE, // PCI VID: NVIDIA 0x10DE
		0x00, // RQ=0, D=0, RSVD=0, INSTANCE_ID=0
		0x89, // OCP_TYPE=8, OCP_VER=9
		OCP_AMI_OEM_NVIDIA_MSG_TYPE_PLATFORM_ENVIRONMENTAL, // NVIDIA_MSG_TYPE
		OCP_AMI_OEM_NVIDIA_GET_TEMPERATURE_READING,	    // command
		0, // completion code
		0,
		0,
		4,
		0,   // data size
		0x57,
		0x00 // temperature reading=12.34
	};

	auto response = reinterpret_cast<ocp_ami_msg *>(responseMsg.data());
	size_t msg_len = responseMsg.size();

	uint8_t cc = OCP_AMI_ERROR;
	uint16_t reasonCode = NONE;
	double temperature_reading = 0;

	auto rc = ocp_ami_oem_nvidia_decode_get_temperature_reading_resp(
		response, msg_len, &cc, &reasonCode, &temperature_reading);

	EXPECT_EQ(rc, OCP_AMI_ERR_INVALID_DATA_LENGTH);
	EXPECT_EQ(cc, OCP_AMI_SUCCESS);
	EXPECT_DOUBLE_EQ(temperature_reading, 0);
}

TEST(getTemperature, testBadDecodeResponseNull)
{
	std::vector<uint8_t> responseMsg{
		0x10,
		0xDE, // PCI VID: NVIDIA 0x10DE
		0x00, // RQ=0, D=0, RSVD=0, INSTANCE_ID=0
		0x89, // OCP_TYPE=8, OCP_VER=9
		OCP_AMI_OEM_NVIDIA_MSG_TYPE_PLATFORM_ENVIRONMENTAL, // NVIDIA_MSG_TYPE
		OCP_AMI_OEM_NVIDIA_GET_TEMPERATURE_READING,	    // command
		0, // completion code
		0,
		0,
		4,
		0, // data size
		0x57,
		0x0c,
		0x00,
		0x00 // temperature reading=12.34
	};

	auto response = reinterpret_cast<ocp_ami_msg *>(responseMsg.data());
	size_t msg_len = responseMsg.size();

	uint8_t cc = OCP_AMI_ERROR;
	uint16_t reasonCode = NONE;

	auto rc = ocp_ami_oem_nvidia_decode_get_temperature_reading_resp(
		response, msg_len, &cc, &reasonCode, nullptr);

	EXPECT_EQ(rc, OCP_AMI_ERR_NULL);
	EXPECT_EQ(cc, OCP_AMI_ERROR);
}

TEST(getTemperature, testBadDecodeResponseDataLength)
{
	std::vector<uint8_t> responseMsg{
		0x10,
		0xDE, // PCI VID: NVIDIA 0x10DE
		0x00, // RQ=0, D=0, RSVD=0, INSTANCE_ID=0
		0x89, // OCP_TYPE=8, OCP_VER=9
		OCP_AMI_OEM_NVIDIA_MSG_TYPE_PLATFORM_ENVIRONMENTAL, // NVIDIA_MSG_TYPE
		OCP_AMI_OEM_NVIDIA_GET_TEMPERATURE_READING,	    // command
		0, // completion code
		0,
		0,
		2,
		0, // data size
		0x57,
		0x0c,
		0x00,
		0x00 // temperature reading=12.34
	};

	auto response = reinterpret_cast<ocp_ami_msg *>(responseMsg.data());
	size_t msg_len = responseMsg.size();

	uint8_t cc = OCP_AMI_ERROR;
	uint16_t reasonCode = NONE;
	double temperature_reading = 0;

	auto rc = ocp_ami_oem_nvidia_decode_get_temperature_reading_resp(
		response, msg_len, &cc, &reasonCode, &temperature_reading);

	EXPECT_EQ(rc, OCP_AMI_ERR_INVALID_DATA);
	EXPECT_EQ(cc, OCP_AMI_SUCCESS);
	EXPECT_DOUBLE_EQ(temperature_reading, 0);
}
