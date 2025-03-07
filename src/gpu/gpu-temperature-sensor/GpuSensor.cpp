/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuSensor.hpp"

#include <endian.h>

#include <cstdint>
#include <cstring>

// Implementation of OcpAmi functions
OcpAmiCompletionCode ocpAmiPackHeader(uint16_t pciVendorId,
                                      const OcpAmiBindingPciVidInfo& hdr,
                                      OcpAmiBindingPciVid* msg)
{
    if (msg == nullptr)
    {
        return OCP_AMI_ERR_NULL;
    }

    if (hdr.ocp_ami_msg_type != OCP_AMI_RESPONSE &&
        hdr.ocp_ami_msg_type != OCP_AMI_REQUEST &&
        hdr.ocp_ami_msg_type != OCP_AMI_EVENT &&
        hdr.ocp_ami_msg_type != OCP_AMI_EVENT_ACKNOWLEDGMENT)
    {
        return OCP_AMI_ERR_INVALID_DATA;
    }

    if (hdr.instance_id > ocpAmiInstanceMax)
    {
        return OCP_AMI_ERR_INVALID_DATA;
    }

    msg->datagram = 0;
    if (hdr.ocp_ami_msg_type == OCP_AMI_EVENT_ACKNOWLEDGMENT ||
        hdr.ocp_ami_msg_type == OCP_AMI_EVENT)
    {
        msg->datagram = 1;
    }

    msg->request = 0;
    if (hdr.ocp_ami_msg_type == OCP_AMI_REQUEST ||
        hdr.ocp_ami_msg_type == OCP_AMI_EVENT)
    {
        msg->request = 1;
    }

    msg->pci_vendor_id = htobe16(pciVendorId);
    msg->reserved = 0;
    msg->instance_id = hdr.instance_id;
    msg->ocp_type = ocpAmiType;
    msg->ocp_version = ocpAmiVersion;
    msg->ocp_ami_msg_type = hdr.msg_type;

    return OCP_AMI_SUCCESS;
}

OcpAmiCompletionCode ocpAmiEncodeReasonCode(
    uint8_t cc, uint16_t reasonCode, uint8_t commandCode, OcpAmiMessage* msg)
{
    if (msg == nullptr)
    {
        return OCP_AMI_ERR_NULL;
    }

    OcpAmiCommonNonSuccessResponse* response =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<OcpAmiCommonNonSuccessResponse*>(&msg->data);

    response->command = commandCode;
    response->completion_code = cc;
    reasonCode = htole16(reasonCode);
    response->reason_code = reasonCode;

    return OCP_AMI_SUCCESS;
}

OcpAmiCompletionCode ocpAmiDecodeReasonCodeAndCC(
    const OcpAmiMessage* msg, size_t msgLen, uint8_t* cc, uint16_t* reasonCode)
{
    if (msg == nullptr || cc == nullptr || reasonCode == nullptr)
    {
        return OCP_AMI_ERR_NULL;
    }

    *cc =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<const OcpAmiCommonResponse*>(&msg->data)
            ->completion_code;
    if (*cc == OCP_AMI_SUCCESS)
    {
        return OCP_AMI_SUCCESS;
    }

    if (msgLen !=
        (sizeof(OcpAmiBindingPciVid) + sizeof(OcpAmiCommonNonSuccessResponse)))
    {
        return OCP_AMI_ERR_INVALID_DATA_LENGTH;
    }

    const OcpAmiCommonNonSuccessResponse* response =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<const OcpAmiCommonNonSuccessResponse*>(&msg->data);

    // reason code is expected to be present if CC != OCP_AMI_SUCCESS
    *reasonCode = le16toh(response->reason_code);

    return OCP_AMI_SUCCESS;
}

// Implementation of GpuCommon functions
OcpAmiCompletionCode gpuPackHeader(const OcpAmiBindingPciVidInfo& hdr,
                                   OcpAmiBindingPciVid* msg)
{
    return ocpAmiPackHeader(gpuNvidiaPciVendorId, hdr, msg);
}

OcpAmiCompletionCode gpuEncodeReasonCode(
    uint8_t cc, uint16_t reasonCode, uint8_t commandCode, OcpAmiMessage* msg)
{
    return ocpAmiEncodeReasonCode(cc, reasonCode, commandCode, msg);
}

OcpAmiCompletionCode gpuDecodeReasonCodeAndCC(
    const OcpAmiMessage* msg, size_t msgLen, uint8_t* cc, uint16_t* reasonCode)
{
    if (msg != nullptr &&
        be16toh(msg->hdr.pci_vendor_id) != gpuNvidiaPciVendorId)
    {
        return OCP_AMI_ERR_INVALID_DATA;
    }

    return ocpAmiDecodeReasonCodeAndCC(msg, msgLen, cc, reasonCode);
}

namespace
{
// Message type constants
constexpr uint8_t typeDeviceCapabilityDiscovery = 0;
constexpr uint8_t msgTypePlatformEnvironmental = 3;
} // anonymous namespace

OcpAmiCompletionCode gpuEncodeQueryDeviceIdentificationRequest(
    uint8_t instanceId, OcpAmiMessage* msg)
{
    if (msg == nullptr)
    {
        return OCP_AMI_ERR_NULL;
    }

    OcpAmiBindingPciVidInfo header{};
    header.ocp_ami_msg_type = OCP_AMI_REQUEST;
    header.instance_id = instanceId & ocpAmiInstanceIdMask;
    header.msg_type = typeDeviceCapabilityDiscovery;

    auto rc = gpuPackHeader(header, &msg->hdr);
    if (rc != OCP_AMI_SUCCESS)
    {
        return rc;
    }

    auto* request =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<GpuQueryDeviceIdentificationRequest*>(&msg->data);

    request->hdr.command = GPU_QUERY_DEVICE_IDENTIFICATION;
    request->hdr.data_size = 0;

    return OCP_AMI_SUCCESS;
}

OcpAmiCompletionCode gpuEncodeQueryDeviceIdentificationResponse(
    uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
    uint8_t deviceIdentification, uint8_t deviceInstance, OcpAmiMessage* msg)
{
    if (msg == nullptr)
    {
        return OCP_AMI_ERR_NULL;
    }

    OcpAmiBindingPciVidInfo header{};
    header.ocp_ami_msg_type = OCP_AMI_RESPONSE;
    header.instance_id = instanceId & ocpAmiInstanceIdMask;
    header.msg_type = typeDeviceCapabilityDiscovery;

    auto rc = gpuPackHeader(header, &msg->hdr);
    if (rc != OCP_AMI_SUCCESS)
    {
        return rc;
    }

    if (cc != OCP_AMI_SUCCESS)
    {
        return gpuEncodeReasonCode(cc, reasonCode,
                                   GPU_QUERY_DEVICE_IDENTIFICATION, msg);
    }

    auto* response =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<GpuQueryDeviceIdentificationResponse*>(&msg->data);

    response->hdr.command = GPU_QUERY_DEVICE_IDENTIFICATION;
    response->hdr.completion_code = cc;
    response->hdr.data_size = htole16(2);

    response->device_identification = deviceIdentification;
    response->instance_id = deviceInstance;

    return OCP_AMI_SUCCESS;
}

OcpAmiCompletionCode gpuDecodeQueryDeviceIdentificationResponse(
    const OcpAmiMessage* msg, size_t msgLen, uint8_t* cc, uint16_t* reasonCode,
    uint8_t* deviceIdentification, uint8_t* deviceInstance)
{
    if (deviceIdentification == nullptr || deviceInstance == nullptr)
    {
        return OCP_AMI_ERR_NULL;
    }

    auto rc = gpuDecodeReasonCodeAndCC(msg, msgLen, cc, reasonCode);
    if (rc != OCP_AMI_SUCCESS || *cc != OCP_AMI_SUCCESS)
    {
        return rc;
    }

    if (msgLen < sizeof(OcpAmiBindingPciVid) +
                     sizeof(GpuQueryDeviceIdentificationResponse))
    {
        return OCP_AMI_ERR_INVALID_DATA_LENGTH;
    }

    const auto* response =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<const GpuQueryDeviceIdentificationResponse*>(
            &msg->data);

    *deviceIdentification = response->device_identification;
    *deviceInstance = response->instance_id;

    return OCP_AMI_SUCCESS;
}

OcpAmiCompletionCode gpuEncodeGetTemperatureReadingRequest(
    uint8_t instanceId, uint8_t sensorId, OcpAmiMessage* msg)
{
    if (msg == nullptr)
    {
        return OCP_AMI_ERR_NULL;
    }

    OcpAmiBindingPciVidInfo header{};
    header.ocp_ami_msg_type = OCP_AMI_REQUEST;
    header.instance_id = instanceId & ocpAmiInstanceIdMask;
    header.msg_type = msgTypePlatformEnvironmental;

    auto rc = gpuPackHeader(header, &msg->hdr);
    if (rc != OCP_AMI_SUCCESS)
    {
        return rc;
    }

    auto* request =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<GpuGetTemperatureReadingRequest*>(&msg->data);

    request->hdr.command = GPU_GET_TEMPERATURE_READING;
    request->hdr.data_size = sizeof(sensorId);
    request->sensor_id = sensorId;

    return OCP_AMI_SUCCESS;
}

OcpAmiCompletionCode gpuDecodeGetTemperatureReadingRequest(
    const OcpAmiMessage* msg, size_t msgLen, uint8_t* sensorId)
{
    if (msg == nullptr || sensorId == nullptr)
    {
        return OCP_AMI_ERR_NULL;
    }

    if (msgLen <
        sizeof(OcpAmiBindingPciVid) + sizeof(GpuGetTemperatureReadingRequest))
    {
        return OCP_AMI_ERR_INVALID_DATA_LENGTH;
    }

    const auto* request =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<const GpuGetTemperatureReadingRequest*>(&msg->data);

    if (request->hdr.data_size < sizeof(request->sensor_id))
    {
        return OCP_AMI_ERR_INVALID_DATA;
    }

    *sensorId = request->sensor_id;

    return OCP_AMI_SUCCESS;
}

OcpAmiCompletionCode gpuEncodeGetTemperatureReadingResponse(
    uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
    double temperatureReading, OcpAmiMessage* msg)
{
    if (msg == nullptr)
    {
        return OCP_AMI_ERR_NULL;
    }

    OcpAmiBindingPciVidInfo header{};
    header.ocp_ami_msg_type = OCP_AMI_RESPONSE;
    header.instance_id = instanceId & ocpAmiInstanceIdMask;
    header.msg_type = msgTypePlatformEnvironmental;

    auto rc = gpuPackHeader(header, &msg->hdr);
    if (rc != OCP_AMI_SUCCESS)
    {
        return rc;
    }

    if (cc != OCP_AMI_SUCCESS)
    {
        return gpuEncodeReasonCode(cc, reasonCode, GPU_GET_TEMPERATURE_READING,
                                   msg);
    }

    auto* response =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<GpuGetTemperatureReadingResponse*>(&msg->data);

    response->hdr.command = GPU_GET_TEMPERATURE_READING;
    response->hdr.completion_code = cc;
    response->hdr.data_size = htole16(sizeof(uint32_t));

    int32_t reading = static_cast<int32_t>(temperatureReading * (1 << 8));
    response->reading = htole32(reading);

    return OCP_AMI_SUCCESS;
}

OcpAmiCompletionCode gpuDecodeGetTemperatureReadingResponse(
    const OcpAmiMessage* msg, size_t msgLen, uint8_t* cc, uint16_t* reasonCode,
    double* temperatureReading)
{
    if (temperatureReading == nullptr)
    {
        return OCP_AMI_ERR_NULL;
    }

    auto rc = gpuDecodeReasonCodeAndCC(msg, msgLen, cc, reasonCode);
    if (rc != OCP_AMI_SUCCESS || *cc != OCP_AMI_SUCCESS)
    {
        return rc;
    }

    if (msgLen <
        sizeof(OcpAmiBindingPciVid) + sizeof(GpuGetTemperatureReadingResponse))
    {
        return OCP_AMI_ERR_INVALID_DATA_LENGTH;
    }

    const auto* response =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<const GpuGetTemperatureReadingResponse*>(&msg->data);

    uint16_t dataSize = le16toh(response->hdr.data_size);
    if (dataSize != sizeof(int32_t))
    {
        return OCP_AMI_ERR_INVALID_DATA;
    }

    int32_t reading = le32toh(response->reading);
    *temperatureReading = reading / static_cast<double>(1 << 8);

    return OCP_AMI_SUCCESS;
}
