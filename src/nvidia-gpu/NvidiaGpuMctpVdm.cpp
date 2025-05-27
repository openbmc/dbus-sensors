/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMctpVdm.hpp"

#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <vector>

namespace gpu
{

int packHeader(const ocp::accelerator_management::BindingPciVidInfo& hdr,
               ocp::accelerator_management::BindingPciVid& msg)
{
    return ocp::accelerator_management::packHeader(nvidiaPciVendorId, hdr, msg);
}

int encodeQueryDeviceIdentificationRequest(uint8_t instanceId,
                                           std::vector<uint8_t>& buf)
{
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdBitMask;
    header.msg_type =
        static_cast<uint8_t>(MessageType::DEVICE_CAPABILITY_DISCOVERY);

    auto rc = packHeader(header, msg.hdr);

    if (rc != 0)
    {
        return rc;
    }

    QueryDeviceIdentificationRequest request{};
    request.hdr.command = static_cast<uint8_t>(
        DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION);
    request.hdr.data_size = 0;

    buf.resize(sizeof(msg) + sizeof(request));
    std::memcpy(buf.data(), &msg, sizeof(msg));
    std::memcpy(buf.data() + sizeof(msg), &request, sizeof(request));

    return 0;
}

int decodeQueryDeviceIdentificationResponse(
    const std::vector<uint8_t>& buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint8_t& deviceIdentification, uint8_t& deviceInstance)
{
    auto msgLen = buf.size();
    ocp::accelerator_management::Message msg{};
    std::memcpy(&msg, buf.data(), sizeof(msg));

    if (msgLen <
        (sizeof(ocp::accelerator_management::Message) +
         sizeof(ocp::accelerator_management::CommonNonSuccessResponse)))
    {
        return EINVAL;
    }

    ocp::accelerator_management::CommonNonSuccessResponse nonSuccessResponse{};
    std::memcpy(&nonSuccessResponse, buf.data() + sizeof(msg),
                sizeof(nonSuccessResponse));
    auto rc = ocp::accelerator_management::decodeReasonCodeAndCC(
        nonSuccessResponse, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (msgLen < sizeof(ocp::accelerator_management::Message) +
                     sizeof(QueryDeviceIdentificationResponse))
    {
        return EINVAL;
    }

    QueryDeviceIdentificationResponse response{};
    std::memcpy(&response, buf.data() + sizeof(msg), sizeof(response));

    deviceIdentification = response.device_identification;
    deviceInstance = response.instance_id;

    return 0;
}

int encodeGetTemperatureReadingRequest(uint8_t instanceId, uint8_t sensorId,
                                       std::vector<uint8_t>& buf)
{
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdBitMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL);

    auto rc = packHeader(header, msg.hdr);

    if (rc != 0)
    {
        return rc;
    }

    GetTemperatureReadingRequest request{};
    request.hdr.command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_TEMPERATURE_READING);
    request.hdr.data_size = sizeof(sensorId);
    request.sensor_id = sensorId;

    buf.resize(sizeof(msg) + sizeof(request));
    std::memcpy(buf.data(), &msg, sizeof(msg));
    std::memcpy(buf.data() + sizeof(msg), &request, sizeof(request));

    return 0;
}

int decodeGetTemperatureReadingResponse(
    const std::vector<uint8_t>& buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    double& temperatureReading)
{
    auto msgLen = buf.size();
    ocp::accelerator_management::Message msg{};
    std::memcpy(&msg, buf.data(), sizeof(msg));

    if (msgLen <
        (sizeof(ocp::accelerator_management::Message) +
         sizeof(ocp::accelerator_management::CommonNonSuccessResponse)))
    {
        return EINVAL;
    }

    ocp::accelerator_management::CommonNonSuccessResponse nonSuccessResponse{};
    std::memcpy(&nonSuccessResponse, buf.data() + sizeof(msg),
                sizeof(nonSuccessResponse));
    auto rc = ocp::accelerator_management::decodeReasonCodeAndCC(
        nonSuccessResponse, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (msgLen < sizeof(ocp::accelerator_management::Message) +
                     sizeof(GetTemperatureReadingResponse))
    {
        return EINVAL;
    }

    GetTemperatureReadingResponse response{};
    std::memcpy(&response, buf.data() + sizeof(msg), sizeof(response));

    uint16_t dataSize = le16toh(response.hdr.data_size);

    if (dataSize != sizeof(int32_t))
    {
        return EINVAL;
    }

    int32_t reading = le32toh(response.reading);
    temperatureReading = reading / static_cast<double>(1 << 8);

    return 0;
}

int encodeReadThermalParametersRequest(uint8_t instanceId, uint8_t sensorId,
                                       std::vector<uint8_t>& buf)
{
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdBitMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL);

    auto rc = packHeader(header, msg.hdr);

    if (rc != 0)
    {
        return rc;
    }

    GetTemperatureReadingRequest request{};
    request.hdr.command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS);
    request.hdr.data_size = sizeof(sensorId);
    request.sensor_id = sensorId;

    buf.resize(sizeof(msg) + sizeof(request));
    std::memcpy(buf.data(), &msg, sizeof(msg));
    std::memcpy(buf.data() + sizeof(msg), &request, sizeof(request));

    return 0;
}

int decodeReadThermalParametersResponse(
    const std::vector<uint8_t>& buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    int& threshold)
{
    auto msgLen = buf.size();
    ocp::accelerator_management::Message msg{};
    std::memcpy(&msg, buf.data(), sizeof(msg));

    if (msgLen <
        (sizeof(ocp::accelerator_management::Message) +
         sizeof(ocp::accelerator_management::CommonNonSuccessResponse)))
    {
        return EINVAL;
    }

    ocp::accelerator_management::CommonNonSuccessResponse nonSuccessResponse{};
    std::memcpy(&nonSuccessResponse, buf.data() + sizeof(msg),
                sizeof(nonSuccessResponse));
    auto rc = ocp::accelerator_management::decodeReasonCodeAndCC(
        nonSuccessResponse, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (msgLen < sizeof(ocp::accelerator_management::Message) +
                     sizeof(ReadThermalParametersResponse))
    {
        return EINVAL;
    }

    ReadThermalParametersResponse response{};
    std::memcpy(&response, buf.data() + sizeof(msg), sizeof(response));

    uint16_t dataSize = le16toh(response.hdr.data_size);

    if (dataSize != sizeof(int32_t))
    {
        return EINVAL;
    }

    threshold = le32toh(response.threshold);

    return 0;
}

int encodeGetCurrentPowerDrawRequest(uint8_t instanceId, uint8_t sensorId,
                                     uint8_t averagingInterval,
                                     std::vector<uint8_t>& buf)
{
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdBitMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL);

    auto rc = packHeader(header, msg.hdr);

    if (rc != 0)
    {
        return rc;
    }

    GetCurrentPowerDrawRequest request{};
    request.hdr.command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW);
    request.hdr.data_size = sizeof(sensorId) + sizeof(averagingInterval);
    request.sensorId = sensorId;
    request.averagingInterval = averagingInterval;

    buf.resize(sizeof(msg) + sizeof(request));
    std::memcpy(buf.data(), &msg, sizeof(msg));
    std::memcpy(buf.data() + sizeof(msg), &request, sizeof(request));

    return 0;
}

int decodeGetCurrentPowerDrawResponse(
    const std::vector<uint8_t>& buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint32_t& power)
{
    auto msgLen = buf.size();
    ocp::accelerator_management::Message msg{};
    std::memcpy(&msg, buf.data(), sizeof(msg));

    if (msgLen <
        (sizeof(ocp::accelerator_management::Message) +
         sizeof(ocp::accelerator_management::CommonNonSuccessResponse)))
    {
        return EINVAL;
    }

    ocp::accelerator_management::CommonNonSuccessResponse nonSuccessResponse{};
    std::memcpy(&nonSuccessResponse, buf.data() + sizeof(msg),
                sizeof(nonSuccessResponse));
    auto rc = ocp::accelerator_management::decodeReasonCodeAndCC(
        nonSuccessResponse, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (msgLen < sizeof(ocp::accelerator_management::Message) +
                     sizeof(GetCurrentPowerDrawResponse))
    {
        return EINVAL;
    }

    GetCurrentPowerDrawResponse response{};
    std::memcpy(&response, buf.data() + sizeof(msg), sizeof(response));

    uint16_t dataSize = le16toh(response.hdr.data_size);

    if (dataSize != sizeof(int32_t))
    {
        return EINVAL;
    }

    power = le32toh(response.power);

    return 0;
}

int encodeGetCurrentEnergyCounterRequest(uint8_t instanceId, uint8_t sensorId,
                                         std::vector<uint8_t>& buf)
{
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdBitMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL);

    auto rc = packHeader(header, msg.hdr);

    if (rc != 0)
    {
        return rc;
    }

    GetCurrentEnergyCounterRequest request{};
    request.hdr.command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_CURRENT_ENERGY_COUNTER);
    request.hdr.data_size = sizeof(sensorId);
    request.sensor_id = sensorId;

    buf.resize(sizeof(msg) + sizeof(request));
    std::memcpy(buf.data(), &msg, sizeof(msg));
    std::memcpy(buf.data() + sizeof(msg), &request, sizeof(request));

    return 0;
}

int decodeGetCurrentEnergyCounterResponse(
    const std::vector<uint8_t>& buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint64_t& energy)
{
    auto msgLen = buf.size();
    ocp::accelerator_management::Message msg{};
    std::memcpy(&msg, buf.data(), sizeof(msg));

    if (msgLen <
        (sizeof(ocp::accelerator_management::Message) +
         sizeof(ocp::accelerator_management::CommonNonSuccessResponse)))
    {
        return EINVAL;
    }

    ocp::accelerator_management::CommonNonSuccessResponse nonSuccessResponse{};
    std::memcpy(&nonSuccessResponse, buf.data() + sizeof(msg),
                sizeof(nonSuccessResponse));
    auto rc = ocp::accelerator_management::decodeReasonCodeAndCC(
        nonSuccessResponse, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (msgLen < sizeof(ocp::accelerator_management::Message) +
                     sizeof(GetCurrentEnergyCounterResponse))
    {
        return EINVAL;
    }

    GetCurrentEnergyCounterResponse response{};
    std::memcpy(&response, buf.data() + sizeof(msg), sizeof(response));

    uint16_t dataSize = le16toh(response.hdr.data_size);

    if (dataSize != sizeof(uint64_t))
    {
        return EINVAL;
    }

    energy = le32toh(response.energy);

    return 0;
}

int encodeGetVoltageRequest(uint8_t instanceId, uint8_t sensorId,
                            std::vector<uint8_t>& buf)
{
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdBitMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL);

    auto rc = packHeader(header, msg.hdr);

    if (rc != 0)
    {
        return rc;
    }

    GetVoltageRequest request{};
    request.hdr.command =
        static_cast<uint8_t>(PlatformEnvironmentalCommands::GET_VOLTAGE);
    request.hdr.data_size = sizeof(sensorId);
    request.sensor_id = sensorId;

    buf.resize(sizeof(msg) + sizeof(request));
    std::memcpy(buf.data(), &msg, sizeof(msg));
    std::memcpy(buf.data() + sizeof(msg), &request, sizeof(request));

    return 0;
}

int decodeGetVoltageResponse(const std::vector<uint8_t>& buf,
                             ocp::accelerator_management::CompletionCode& cc,
                             uint16_t& reasonCode, uint32_t& voltage)
{
    auto msgLen = buf.size();
    ocp::accelerator_management::Message msg{};
    std::memcpy(&msg, buf.data(), sizeof(msg));

    if (msgLen <
        (sizeof(ocp::accelerator_management::Message) +
         sizeof(ocp::accelerator_management::CommonNonSuccessResponse)))
    {
        return EINVAL;
    }

    ocp::accelerator_management::CommonNonSuccessResponse nonSuccessResponse{};
    std::memcpy(&nonSuccessResponse, buf.data() + sizeof(msg),
                sizeof(nonSuccessResponse));
    auto rc = ocp::accelerator_management::decodeReasonCodeAndCC(
        nonSuccessResponse, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (msgLen < sizeof(ocp::accelerator_management::Message) +
                     sizeof(GetVoltageResponse))
    {
        return EINVAL;
    }

    GetVoltageResponse response{};
    std::memcpy(&response, buf.data() + sizeof(msg), sizeof(response));

    uint16_t dataSize = le16toh(response.hdr.data_size);

    if (dataSize != sizeof(uint32_t))
    {
        return EINVAL;
    }

    voltage = le32toh(response.voltage);

    return 0;
}
} // namespace gpu
