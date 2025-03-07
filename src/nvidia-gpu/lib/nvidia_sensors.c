/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include <nvidia_sensors.h>

#include <endian.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

int ocp_ami_oem_nvidia_encode_query_device_identification_req(
	uint8_t instance_id, struct ocp_ami_msg *msg)
{
	int rc = 0;

	if (msg == NULL) {
		return OCP_AMI_ERR_NULL;
	}

	struct ocp_ami_binding_pci_vid_info header = { 0 };
	header.ocp_ami_msg_type = OCP_AMI_REQUEST;
	header.instance_id = instance_id & OCP_AMI_INSTANCEID_MASK;
	header.msg_type = OCP_AMI_OEM_NVIDIA_TYPE_DEVICE_CAPABILITY_DISCOVERY;

	rc = ocp_ami_oem_nvidia_pack_header(&header, &msg->hdr);
	if (rc != OCP_AMI_SUCCESS) {
		return rc;
	}

	struct ocp_ami_oem_nvidia_query_device_identification_req *request =
		(struct ocp_ami_oem_nvidia_query_device_identification_req *)
			msg->payload;

	request->hdr.command = OCP_AMI_OEM_NVIDIA_QUERY_DEVICE_IDENTIFICATION;
	request->hdr.data_size = 0;

	return OCP_AMI_SUCCESS;
}

int ocp_ami_oem_nvidia_encode_query_device_identification_resp(
	uint8_t instance_id, uint8_t cc, uint16_t reason_code,
	const uint8_t device_identification, const uint8_t device_instance,
	struct ocp_ami_msg *msg)
{
	int rc = 0;

	if (msg == NULL) {
		return OCP_AMI_ERR_NULL;
	}

	struct ocp_ami_binding_pci_vid_info header = { 0 };
	header.ocp_ami_msg_type = OCP_AMI_RESPONSE;
	header.instance_id = instance_id & OCP_AMI_INSTANCEID_MASK;
	header.msg_type = OCP_AMI_OEM_NVIDIA_TYPE_DEVICE_CAPABILITY_DISCOVERY;

	rc = ocp_ami_oem_nvidia_pack_header(&header, &msg->hdr);
	if (rc != OCP_AMI_SUCCESS) {
		return rc;
	}

	if (cc != OCP_AMI_SUCCESS) {
		return ocp_ami_oem_nvidia_encode_reason_code(
			cc, reason_code,
			OCP_AMI_OEM_NVIDIA_QUERY_DEVICE_IDENTIFICATION, msg);
	}

	struct ocp_ami_oem_nvidia_query_device_identification_resp *response =
		(struct ocp_ami_oem_nvidia_query_device_identification_resp *)
			msg->payload;

	response->hdr.command = OCP_AMI_OEM_NVIDIA_QUERY_DEVICE_IDENTIFICATION;
	response->hdr.completion_code = cc;
	response->hdr.data_size = htole16(2);

	response->device_identification = device_identification;
	response->instance_id = device_instance;

	return OCP_AMI_SUCCESS;
}

int ocp_ami_oem_nvidia_decode_query_device_identification_resp(
	const struct ocp_ami_msg *msg, size_t msg_len, uint8_t *cc,
	uint16_t *reason_code, uint8_t *device_identification,
	uint8_t *device_instance)
{
	int rc = 0;

	if (device_identification == NULL || device_instance == NULL) {
		return OCP_AMI_ERR_NULL;
	}

	rc = ocp_ami_oem_nvidia_decode_reason_code_and_cc(msg, msg_len, cc,
							  reason_code);
	if (rc != OCP_AMI_SUCCESS || *cc != OCP_AMI_SUCCESS) {
		return rc;
	}

	if (msg_len <
	    sizeof(struct ocp_ami_binding_pci_vid) +
		    sizeof(struct ocp_ami_oem_nvidia_query_device_identification_resp)) {
		return OCP_AMI_ERR_INVALID_DATA_LENGTH;
	}

	struct ocp_ami_oem_nvidia_query_device_identification_resp *response =
		(struct ocp_ami_oem_nvidia_query_device_identification_resp *)
			msg->payload;

	*device_identification = response->device_identification;
	*device_instance = response->instance_id;

	return OCP_AMI_SUCCESS;
}

int ocp_ami_oem_nvidia_encode_get_temperature_reading_req(
	uint8_t instance_id, uint8_t sensor_id, struct ocp_ami_msg *msg)
{
	int rc = 0;

	if (msg == NULL) {
		return OCP_AMI_ERR_NULL;
	}

	struct ocp_ami_binding_pci_vid_info header = { 0 };
	header.ocp_ami_msg_type = OCP_AMI_REQUEST;
	header.instance_id = instance_id & OCP_AMI_INSTANCEID_MASK;
	header.msg_type = OCP_AMI_OEM_NVIDIA_MSG_TYPE_PLATFORM_ENVIRONMENTAL;

	rc = ocp_ami_oem_nvidia_pack_header(&header, &msg->hdr);
	if (rc != OCP_AMI_SUCCESS) {
		return rc;
	}

	ocp_ami_oem_nvidia_get_temperature_reading_req *request =
		(ocp_ami_oem_nvidia_get_temperature_reading_req *)msg->payload;

	request->hdr.command = OCP_AMI_OEM_NVIDIA_GET_TEMPERATURE_READING;
	request->hdr.data_size = sizeof(sensor_id);
	request->sensor_id = sensor_id;

	return OCP_AMI_SUCCESS;
}

int ocp_ami_oem_nvidia_decode_get_temperature_reading_req(
	const struct ocp_ami_msg *msg, size_t msg_len, uint8_t *sensor_id)
{
	if (msg == NULL || sensor_id == NULL) {
		return OCP_AMI_ERR_NULL;
	}

	if (msg_len <
	    sizeof(struct ocp_ami_binding_pci_vid) +
		    sizeof(ocp_ami_oem_nvidia_get_temperature_reading_req)) {
		return OCP_AMI_ERR_INVALID_DATA_LENGTH;
	}

	ocp_ami_oem_nvidia_get_temperature_reading_req *request =
		(ocp_ami_oem_nvidia_get_temperature_reading_req *)msg->payload;

	if (request->hdr.data_size < sizeof(request->sensor_id)) {
		return OCP_AMI_ERR_INVALID_DATA;
	}

	*sensor_id = request->sensor_id;

	return OCP_AMI_SUCCESS;
}

int ocp_ami_oem_nvidia_encode_get_temperature_reading_resp(
	uint8_t instance_id, uint8_t cc, uint16_t reason_code,
	double temperature_reading, struct ocp_ami_msg *msg)
{
	int rc = 0;

	if (msg == NULL) {
		return OCP_AMI_ERR_NULL;
	}

	struct ocp_ami_binding_pci_vid_info header = { 0 };
	header.ocp_ami_msg_type = OCP_AMI_RESPONSE;
	header.instance_id = instance_id & OCP_AMI_INSTANCEID_MASK;
	header.msg_type = OCP_AMI_OEM_NVIDIA_MSG_TYPE_PLATFORM_ENVIRONMENTAL;

	rc = ocp_ami_oem_nvidia_pack_header(&header, &msg->hdr);
	if (rc != OCP_AMI_SUCCESS) {
		return rc;
	}

	if (cc != OCP_AMI_SUCCESS) {
		return ocp_ami_oem_nvidia_encode_reason_code(
			cc, reason_code,
			OCP_AMI_OEM_NVIDIA_GET_TEMPERATURE_READING, msg);
	}

	struct ocp_ami_oem_nvidia_get_temperature_reading_resp *response =
		(struct ocp_ami_oem_nvidia_get_temperature_reading_resp *)
			msg->payload;

	response->hdr.command = OCP_AMI_OEM_NVIDIA_GET_TEMPERATURE_READING;
	response->hdr.completion_code = cc;
	response->hdr.data_size = htole16(sizeof(uint32_t));

	int32_t reading = (int32_t)(temperature_reading * (1 << 8));
	response->reading = htole32(reading);

	return OCP_AMI_SUCCESS;
}

int ocp_ami_oem_nvidia_decode_get_temperature_reading_resp(
	const struct ocp_ami_msg *msg, size_t msg_len, uint8_t *cc,
	uint16_t *reason_code, double *temperature_reading)
{
	int rc = 0;

	if (temperature_reading == NULL) {
		return OCP_AMI_ERR_NULL;
	}

	rc = ocp_ami_oem_nvidia_decode_reason_code_and_cc(msg, msg_len, cc,
							  reason_code);
	if (rc != OCP_AMI_SUCCESS || *cc != OCP_AMI_SUCCESS) {
		return rc;
	}

	if (msg_len <
	    sizeof(struct ocp_ami_binding_pci_vid) +
		    sizeof(struct ocp_ami_oem_nvidia_get_temperature_reading_resp)) {
		return OCP_AMI_ERR_INVALID_DATA_LENGTH;
	}

	struct ocp_ami_oem_nvidia_get_temperature_reading_resp *response =
		(struct ocp_ami_oem_nvidia_get_temperature_reading_resp *)
			msg->payload;

	uint16_t data_size = le16toh(response->hdr.data_size);
	if (data_size != sizeof(int32_t)) {
		return OCP_AMI_ERR_INVALID_DATA;
	}

	int32_t reading = le32toh(response->reading);
	*temperature_reading = reading / (double)(1 << 8);

	return OCP_AMI_SUCCESS;
}
