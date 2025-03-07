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
namespace ami
{

CompletionCode packHeader(uint16_t pciVendorId, const BindingPciVidInfo& hdr,
                          BindingPciVid& msg)
{
    if (hdr.ocp_ami_msg_type != static_cast<uint8_t>(MessageType::RESPONSE) &&
        hdr.ocp_ami_msg_type != static_cast<uint8_t>(MessageType::REQUEST) &&
        hdr.ocp_ami_msg_type != static_cast<uint8_t>(MessageType::EVENT) &&
        hdr.ocp_ami_msg_type !=
            static_cast<uint8_t>(MessageType::EVENT_ACKNOWLEDGMENT))
    {
        return CompletionCode::ERR_INVALID_DATA;
    }

    if (hdr.instance_id > instanceMax)
    {
        return CompletionCode::ERR_INVALID_DATA;
    }

    msg.datagram = 0;
    if (hdr.ocp_ami_msg_type ==
            static_cast<uint8_t>(MessageType::EVENT_ACKNOWLEDGMENT) ||
        hdr.ocp_ami_msg_type == static_cast<uint8_t>(MessageType::EVENT))
    {
        msg.datagram = 1;
    }

    msg.request = 0;
    if (hdr.ocp_ami_msg_type == static_cast<uint8_t>(MessageType::REQUEST) ||
        hdr.ocp_ami_msg_type == static_cast<uint8_t>(MessageType::EVENT))
    {
        msg.request = 1;
    }

    msg.pci_vendor_id = htobe16(pciVendorId);
    msg.reserved = 0;
    msg.instance_id = hdr.instance_id;
    msg.ocp_type = type;
    msg.ocp_version = version;
    msg.ocp_ami_msg_type = hdr.msg_type;

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
} // namespace ami
} // namespace ocp

namespace gpu
{

ocp::ami::CompletionCode packHeader(const ocp::ami::BindingPciVidInfo& hdr,
                                    ocp::ami::BindingPciVid& msg)
{
    return ocp::ami::packHeader(nvidiaPciVendorId, hdr, msg);
}

ocp::ami::CompletionCode encodeReasonCode(uint8_t cc, uint16_t reasonCode,
                                          uint8_t commandCode,
                                          ocp::ami::Message& msg)
{
    return ocp::ami::encodeReasonCode(cc, reasonCode, commandCode, msg);
}

ocp::ami::CompletionCode decodeReasonCodeAndCC(const ocp::ami::Message& msg,
                                               size_t msgLen, uint8_t& cc,
                                               uint16_t& reasonCode)
{
    if (be16toh(msg.hdr.pci_vendor_id) != nvidiaPciVendorId)
    {
        return ocp::ami::CompletionCode::ERR_INVALID_DATA;
    }

    return ocp::ami::decodeReasonCodeAndCC(msg, msgLen, cc, reasonCode);
}

ocp::ami::CompletionCode encodeQueryDeviceIdentificationRequest(
    uint8_t instanceId, ocp::ami::Message& msg)
{
    ocp::ami::BindingPciVidInfo header{};
    header.ocp_ami_msg_type =
        static_cast<uint8_t>(ocp::ami::MessageType::REQUEST);
    header.instance_id = instanceId & ocp::ami::instanceIdMask;
    header.msg_type =
        static_cast<uint8_t>(MessageType::DEVICE_CAPABILITY_DISCOVERY);

    auto rc = packHeader(header, msg.hdr);
    if (rc != ocp::ami::CompletionCode::SUCCESS)
    {
        return rc;
    }

    QueryDeviceIdentificationRequest request{};
    request.hdr.command = static_cast<uint8_t>(
        DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION);
    request.hdr.data_size = 0;

    std::memcpy(&msg.data, &request, sizeof(request));

    return ocp::ami::CompletionCode::SUCCESS;
}

ocp::ami::CompletionCode encodeQueryDeviceIdentificationResponse(
    uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
    uint8_t deviceIdentification, uint8_t deviceInstance,
    ocp::ami::Message& msg)
{
    ocp::ami::BindingPciVidInfo header{};
    header.ocp_ami_msg_type =
        static_cast<uint8_t>(ocp::ami::MessageType::RESPONSE);
    header.instance_id = instanceId & ocp::ami::instanceIdMask;
    header.msg_type =
        static_cast<uint8_t>(MessageType::DEVICE_CAPABILITY_DISCOVERY);

    auto rc = packHeader(header, msg.hdr);
    if (rc != ocp::ami::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (cc != static_cast<uint8_t>(ocp::ami::CompletionCode::SUCCESS))
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

    return ocp::ami::CompletionCode::SUCCESS;
}

ocp::ami::CompletionCode decodeQueryDeviceIdentificationResponse(
    const ocp::ami::Message& msg, size_t msgLen, uint8_t& cc,
    uint16_t& reasonCode, uint8_t& deviceIdentification,
    uint8_t& deviceInstance)
{
    auto rc = gpu::decodeReasonCodeAndCC(msg, msgLen, cc, reasonCode);
    if (rc != ocp::ami::CompletionCode::SUCCESS ||
        cc != static_cast<uint8_t>(ocp::ami::CompletionCode::SUCCESS))
    {
        return rc;
    }

    if (msgLen < sizeof(ocp::ami::BindingPciVid) +
                     sizeof(QueryDeviceIdentificationResponse))
    {
        return ocp::ami::CompletionCode::ERR_INVALID_DATA_LENGTH;
    }

    QueryDeviceIdentificationResponse response{};
    std::memcpy(&response, &msg.data, sizeof(response));

    deviceIdentification = response.device_identification;
    deviceInstance = response.instance_id;

    return ocp::ami::CompletionCode::SUCCESS;
}

ocp::ami::CompletionCode encodeGetTemperatureReadingRequest(
    uint8_t instanceId, uint8_t sensorId, ocp::ami::Message& msg)
{
    ocp::ami::BindingPciVidInfo header{};
    header.ocp_ami_msg_type =
        static_cast<uint8_t>(ocp::ami::MessageType::REQUEST);
    header.instance_id = instanceId & ocp::ami::instanceIdMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL);

    auto rc = packHeader(header, msg.hdr);
    if (rc != ocp::ami::CompletionCode::SUCCESS)
    {
        return rc;
    }

    GetTemperatureReadingRequest request{};
    request.hdr.command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_TEMPERATURE_READING);
    request.hdr.data_size = sizeof(sensorId);
    request.sensor_id = sensorId;

    std::memcpy(&msg.data, &request, sizeof(request));

    return ocp::ami::CompletionCode::SUCCESS;
}

ocp::ami::CompletionCode decodeGetTemperatureReadingRequest(
    const ocp::ami::Message& msg, size_t msgLen, uint8_t& sensorId)
{
    if (msgLen <
        sizeof(ocp::ami::BindingPciVid) + sizeof(GetTemperatureReadingRequest))
    {
        return ocp::ami::CompletionCode::ERR_INVALID_DATA_LENGTH;
    }

    GetTemperatureReadingRequest request{};
    std::memcpy(&request, &msg.data, sizeof(request));

    if (request.hdr.data_size < sizeof(request.sensor_id))
    {
        return ocp::ami::CompletionCode::ERR_INVALID_DATA;
    }

    sensorId = request.sensor_id;

    return ocp::ami::CompletionCode::SUCCESS;
}

ocp::ami::CompletionCode encodeGetTemperatureReadingResponse(
    uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
    double temperatureReading, ocp::ami::Message& msg)
{
    ocp::ami::BindingPciVidInfo header{};
    header.ocp_ami_msg_type =
        static_cast<uint8_t>(ocp::ami::MessageType::RESPONSE);
    header.instance_id = instanceId & ocp::ami::instanceIdMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL);

    auto rc = packHeader(header, msg.hdr);
    if (rc != ocp::ami::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (cc != static_cast<uint8_t>(ocp::ami::CompletionCode::SUCCESS))
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

    return ocp::ami::CompletionCode::SUCCESS;
}

ocp::ami::CompletionCode decodeGetTemperatureReadingResponse(
    const ocp::ami::Message& msg, size_t msgLen, uint8_t& cc,
    uint16_t& reasonCode, double& temperatureReading)
{
    auto rc = gpu::decodeReasonCodeAndCC(msg, msgLen, cc, reasonCode);
    if (rc != ocp::ami::CompletionCode::SUCCESS ||
        cc != static_cast<uint8_t>(ocp::ami::CompletionCode::SUCCESS))
    {
        return rc;
    }

    if (msgLen <
        sizeof(ocp::ami::BindingPciVid) + sizeof(GetTemperatureReadingResponse))
    {
        return ocp::ami::CompletionCode::ERR_INVALID_DATA_LENGTH;
    }

    GetTemperatureReadingResponse response{};
    std::memcpy(&response, &msg.data, sizeof(response));

    uint16_t dataSize = le16toh(response.hdr.data_size);
    if (dataSize != sizeof(int32_t))
    {
        return ocp::ami::CompletionCode::ERR_INVALID_DATA;
    }

    int32_t reading = le32toh(response.reading);
    temperatureReading = reading / static_cast<double>(1 << 8);

    return ocp::ami::CompletionCode::SUCCESS;
}
} // namespace gpu
