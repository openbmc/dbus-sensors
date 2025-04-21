/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMctpVdm.hpp"

#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <span>

namespace gpu
{
// These functions encode/decode data communicated over the network
// The use of reinterpret_cast enables direct memory access to raw byte buffers
// without doing unnecessary data copying
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
int packHeader(const ocp::accelerator_management::BindingPciVidInfo& hdr,
               ocp::accelerator_management::BindingPciVid& msg)
{
    return ocp::accelerator_management::packHeader(nvidiaPciVendorId, hdr, msg);
}

int encodeQueryDeviceIdentificationRequest(uint8_t instanceId,
                                           const std::span<uint8_t> buf)
{
    if (buf.size() < sizeof(QueryDeviceIdentificationRequest))
    {
        return EINVAL;
    }

    auto* msg = reinterpret_cast<QueryDeviceIdentificationRequest*>(buf.data());

    ocp::accelerator_management::BindingPciVidInfo header{};

    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdBitMask;
    header.msg_type =
        static_cast<uint8_t>(MessageType::DEVICE_CAPABILITY_DISCOVERY);

    auto rc = packHeader(header, msg->hdr.msgHdr.hdr);

    if (rc != 0)
    {
        return rc;
    }

    msg->hdr.command = static_cast<uint8_t>(
        DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION);
    msg->hdr.data_size = 0;

    return 0;
}

int decodeQueryDeviceIdentificationResponse(
    const std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint8_t& deviceIdentification, uint8_t& deviceInstance)
{
    auto rc =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (buf.size() < sizeof(QueryDeviceIdentificationResponse))
    {
        return EINVAL;
    }

    const auto* response =
        reinterpret_cast<const QueryDeviceIdentificationResponse*>(buf.data());

    deviceIdentification = response->device_identification;
    deviceInstance = response->instance_id;

    return 0;
}

int encodeGetTemperatureReadingRequest(uint8_t instanceId, uint8_t sensorId,
                                       std::span<uint8_t> buf)
{
    if (buf.size() < sizeof(GetTemperatureReadingRequest))
    {
        return EINVAL;
    }

    auto* msg = reinterpret_cast<GetTemperatureReadingRequest*>(buf.data());

    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdBitMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL);

    auto rc = packHeader(header, msg->hdr.msgHdr.hdr);

    if (rc != 0)
    {
        return rc;
    }

    msg->hdr.command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_TEMPERATURE_READING);
    msg->hdr.data_size = sizeof(sensorId);
    msg->sensor_id = sensorId;

    return 0;
}

int decodeGetTemperatureReadingResponse(
    const std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    double& temperatureReading)
{
    auto rc =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (buf.size() < sizeof(GetTemperatureReadingResponse))
    {
        return EINVAL;
    }

    const auto* response =
        reinterpret_cast<const GetTemperatureReadingResponse*>(buf.data());

    uint16_t dataSize = le16toh(response->hdr.data_size);

    if (dataSize != sizeof(int32_t))
    {
        return EINVAL;
    }

    int32_t reading = le32toh(response->reading);
    temperatureReading = reading / static_cast<double>(1 << 8);

    return 0;
}
// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
} // namespace gpu
