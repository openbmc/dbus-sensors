/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuMctpVdm.hpp"

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
            static_cast<uint8_t>(MessageType::REQUEST) &&
        hdr.ocp_accelerator_management_msg_type !=
            static_cast<uint8_t>(MessageType::EVENT) &&
        hdr.ocp_accelerator_management_msg_type !=
            static_cast<uint8_t>(MessageType::EVENT_ACKNOWLEDGMENT))
    {
        return CompletionCode::ERR_INVALID_DATA;
    }

    if (hdr.instance_id > instanceMax)
    {
        return CompletionCode::ERR_INVALID_DATA;
    }

    msg.datagram = 0;
    if (hdr.ocp_accelerator_management_msg_type ==
            static_cast<uint8_t>(MessageType::EVENT_ACKNOWLEDGMENT) ||
        hdr.ocp_accelerator_management_msg_type ==
            static_cast<uint8_t>(MessageType::EVENT))
    {
        msg.datagram = 1;
    }

    msg.request = 0;
    if (hdr.ocp_accelerator_management_msg_type ==
            static_cast<uint8_t>(MessageType::REQUEST) ||
        hdr.ocp_accelerator_management_msg_type ==
            static_cast<uint8_t>(MessageType::EVENT))
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

namespace gpu
{

ocp::accelerator_management::CompletionCode packHeader(
    const ocp::accelerator_management::BindingPciVidInfo& hdr,
    ocp::accelerator_management::BindingPciVid& msg)
{
    return ocp::accelerator_management::packHeader(nvidiaPciVendorId, hdr, msg);
}

ocp::accelerator_management::CompletionCode encodeReasonCode(
    uint8_t cc, uint16_t reasonCode, uint8_t commandCode,
    ocp::accelerator_management::Message& msg)
{
    return ocp::accelerator_management::encodeReasonCode(cc, reasonCode,
                                                         commandCode, msg);
}

ocp::accelerator_management::CompletionCode decodeReasonCodeAndCC(
    const ocp::accelerator_management::Message& msg, size_t msgLen, uint8_t& cc,
    uint16_t& reasonCode)
{
    if (be16toh(msg.hdr.pci_vendor_id) != nvidiaPciVendorId)
    {
        return ocp::accelerator_management::CompletionCode::ERR_INVALID_DATA;
    }

    return ocp::accelerator_management::decodeReasonCodeAndCC(
        msg, msgLen, cc, reasonCode);
}

ocp::accelerator_management::CompletionCode
    encodeQueryDeviceIdentificationRequest(
        uint8_t instanceId, ocp::accelerator_management::Message& msg)
{
    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdMask;
    header.msg_type =
        static_cast<uint8_t>(MessageType::DEVICE_CAPABILITY_DISCOVERY);

    auto rc = packHeader(header, msg.hdr);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    QueryDeviceIdentificationRequest request{};
    request.hdr.command = static_cast<uint8_t>(
        DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION);
    request.hdr.data_size = 0;

    std::memcpy(&msg.data, &request, sizeof(request));

    return ocp::accelerator_management::CompletionCode::SUCCESS;
}

ocp::accelerator_management::CompletionCode
    encodeQueryDeviceIdentificationResponse(
        uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
        uint8_t deviceIdentification, uint8_t deviceInstance,
        ocp::accelerator_management::Message& msg)
{
    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdMask;
    header.msg_type =
        static_cast<uint8_t>(MessageType::DEVICE_CAPABILITY_DISCOVERY);

    auto rc = packHeader(header, msg.hdr);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (cc != static_cast<uint8_t>(
                  ocp::accelerator_management::CompletionCode::SUCCESS))
    {
        return gpu::encodeReasonCode(
            cc, reasonCode,
            static_cast<uint8_t>(
                DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION),
            msg);
    }

    QueryDeviceIdentificationResponse response{};
    response.hdr.command = static_cast<uint8_t>(
        DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION);
    response.hdr.completion_code = cc;
    response.hdr.data_size = htole16(2);
    response.device_identification = deviceIdentification;
    response.instance_id = deviceInstance;

    std::memcpy(&msg.data, &response, sizeof(response));

    return ocp::accelerator_management::CompletionCode::SUCCESS;
}

ocp::accelerator_management::CompletionCode
    decodeQueryDeviceIdentificationResponse(
        const ocp::accelerator_management::Message& msg, size_t msgLen,
        uint8_t& cc, uint16_t& reasonCode, uint8_t& deviceIdentification,
        uint8_t& deviceInstance)
{
    auto rc = gpu::decodeReasonCodeAndCC(msg, msgLen, cc, reasonCode);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS ||
        cc != static_cast<uint8_t>(
                  ocp::accelerator_management::CompletionCode::SUCCESS))
    {
        return rc;
    }

    if (msgLen < sizeof(ocp::accelerator_management::BindingPciVid) +
                     sizeof(QueryDeviceIdentificationResponse))
    {
        return ocp::accelerator_management::CompletionCode::
            ERR_INVALID_DATA_LENGTH;
    }

    QueryDeviceIdentificationResponse response{};
    std::memcpy(&response, &msg.data, sizeof(response));

    deviceIdentification = response.device_identification;
    deviceInstance = response.instance_id;

    return ocp::accelerator_management::CompletionCode::SUCCESS;
}

ocp::accelerator_management::CompletionCode encodeGetTemperatureReadingRequest(
    uint8_t instanceId, uint8_t sensorId,
    ocp::accelerator_management::Message& msg)
{
    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL);

    auto rc = packHeader(header, msg.hdr);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    GetTemperatureReadingRequest request{};
    request.hdr.command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_TEMPERATURE_READING);
    request.hdr.data_size = sizeof(sensorId);
    request.sensor_id = sensorId;

    std::memcpy(&msg.data, &request, sizeof(request));

    return ocp::accelerator_management::CompletionCode::SUCCESS;
}

ocp::accelerator_management::CompletionCode decodeGetTemperatureReadingRequest(
    const ocp::accelerator_management::Message& msg, size_t msgLen,
    uint8_t& sensorId)
{
    if (msgLen < sizeof(ocp::accelerator_management::BindingPciVid) +
                     sizeof(GetTemperatureReadingRequest))
    {
        return ocp::accelerator_management::CompletionCode::
            ERR_INVALID_DATA_LENGTH;
    }

    GetTemperatureReadingRequest request{};
    std::memcpy(&request, &msg.data, sizeof(request));

    if (request.hdr.data_size < sizeof(request.sensor_id))
    {
        return ocp::accelerator_management::CompletionCode::ERR_INVALID_DATA;
    }

    sensorId = request.sensor_id;

    return ocp::accelerator_management::CompletionCode::SUCCESS;
}

ocp::accelerator_management::CompletionCode encodeGetTemperatureReadingResponse(
    uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
    double temperatureReading, ocp::accelerator_management::Message& msg)
{
    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL);

    auto rc = packHeader(header, msg.hdr);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (cc != static_cast<uint8_t>(
                  ocp::accelerator_management::CompletionCode::SUCCESS))
    {
        return gpu::encodeReasonCode(
            cc, reasonCode,
            static_cast<uint8_t>(
                PlatformEnvironmentalCommands::GET_TEMPERATURE_READING),
            msg);
    }

    GetTemperatureReadingResponse response{};
    response.hdr.command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_TEMPERATURE_READING);
    response.hdr.completion_code = cc;
    response.hdr.data_size = htole16(sizeof(uint32_t));

    int32_t reading = static_cast<int32_t>(temperatureReading * (1 << 8));
    response.reading = htole32(reading);

    std::memcpy(&msg.data, &response, sizeof(response));

    return ocp::accelerator_management::CompletionCode::SUCCESS;
}

ocp::accelerator_management::CompletionCode decodeGetTemperatureReadingResponse(
    const ocp::accelerator_management::Message& msg, size_t msgLen, uint8_t& cc,
    uint16_t& reasonCode, double& temperatureReading)
{
    auto rc = gpu::decodeReasonCodeAndCC(msg, msgLen, cc, reasonCode);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS ||
        cc != static_cast<uint8_t>(
                  ocp::accelerator_management::CompletionCode::SUCCESS))
    {
        return rc;
    }

    if (msgLen < sizeof(ocp::accelerator_management::BindingPciVid) +
                     sizeof(GetTemperatureReadingResponse))
    {
        return ocp::accelerator_management::CompletionCode::
            ERR_INVALID_DATA_LENGTH;
    }

    GetTemperatureReadingResponse response{};
    std::memcpy(&response, &msg.data, sizeof(response));

    uint16_t dataSize = le16toh(response.hdr.data_size);
    if (dataSize != sizeof(int32_t))
    {
        return ocp::accelerator_management::CompletionCode::ERR_INVALID_DATA;
    }

    int32_t reading = le32toh(response.reading);
    temperatureReading = reading / static_cast<double>(1 << 8);

    return ocp::accelerator_management::CompletionCode::SUCCESS;
}
} // namespace gpu
