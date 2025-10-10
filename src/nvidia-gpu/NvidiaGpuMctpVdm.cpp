/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMctpVdm.hpp"

#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <cerrno>
#include <cstdint>
#include <span>
#include <vector>

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

int encodeReadThermalParametersRequest(uint8_t instanceId, uint8_t sensorId,
                                       std::span<uint8_t> buf)
{
    if (buf.size() < sizeof(ReadThermalParametersRequest))
    {
        return EINVAL;
    }

    auto* msg = reinterpret_cast<ReadThermalParametersRequest*>(buf.data());

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
        PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS);
    msg->hdr.data_size = sizeof(sensorId);
    msg->sensor_id = sensorId;

    return 0;
}

int decodeReadThermalParametersResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    int32_t& threshold)
{
    auto rc =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (buf.size() < sizeof(ReadThermalParametersResponse))
    {
        return EINVAL;
    }

    const auto* response =
        reinterpret_cast<const ReadThermalParametersResponse*>(buf.data());

    uint16_t dataSize = le16toh(response->hdr.data_size);

    if (dataSize != sizeof(int32_t))
    {
        return EINVAL;
    }

    threshold = le32toh(response->threshold);

    return 0;
}

int encodeGetPowerDrawRequest(PlatformEnvironmentalCommands commandCode,
                              uint8_t instanceId, uint8_t sensorId,
                              uint8_t averagingInterval, std::span<uint8_t> buf)
{
    if (buf.size() < sizeof(GetPowerDrawRequest))
    {
        return EINVAL;
    }

    auto* msg = reinterpret_cast<GetPowerDrawRequest*>(buf.data());

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

    msg->hdr.command = static_cast<uint8_t>(commandCode);
    msg->hdr.data_size = sizeof(sensorId) + sizeof(averagingInterval);
    msg->sensorId = sensorId;
    msg->averagingInterval = averagingInterval;

    return 0;
}

int decodeGetPowerDrawResponse(std::span<const uint8_t> buf,
                               ocp::accelerator_management::CompletionCode& cc,
                               uint16_t& reasonCode, uint32_t& power)
{
    auto rc =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (buf.size() < sizeof(GetPowerDrawResponse))
    {
        return EINVAL;
    }

    const auto* response =
        reinterpret_cast<const GetPowerDrawResponse*>(buf.data());

    const uint16_t dataSize = le16toh(response->hdr.data_size);

    if (dataSize != sizeof(uint32_t))
    {
        return EINVAL;
    }

    power = le32toh(response->power);

    return 0;
}

int encodeGetCurrentEnergyCounterRequest(uint8_t instanceId, uint8_t sensorId,
                                         std::span<uint8_t> buf)
{
    if (buf.size() < sizeof(GetTemperatureReadingRequest))
    {
        return EINVAL;
    }

    auto* msg = reinterpret_cast<GetCurrentEnergyCounterRequest*>(buf.data());

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
        PlatformEnvironmentalCommands::GET_CURRENT_ENERGY_COUNTER);
    msg->hdr.data_size = sizeof(sensorId);
    msg->sensor_id = sensorId;

    return 0;
}

int decodeGetCurrentEnergyCounterResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint64_t& energy)
{
    auto rc =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (buf.size() < sizeof(GetPowerDrawResponse))
    {
        return EINVAL;
    }

    const auto* response =
        reinterpret_cast<const GetCurrentEnergyCounterResponse*>(buf.data());

    const uint16_t dataSize = le16toh(response->hdr.data_size);

    if (dataSize != sizeof(uint64_t))
    {
        return EINVAL;
    }

    energy = le32toh(response->energy);

    return 0;
}

int encodeGetVoltageRequest(uint8_t instanceId, uint8_t sensorId,
                            std::span<uint8_t> buf)
{
    if (buf.size() < sizeof(GetVoltageRequest))
    {
        return EINVAL;
    }

    auto* msg = reinterpret_cast<GetVoltageRequest*>(buf.data());

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

    msg->hdr.command =
        static_cast<uint8_t>(PlatformEnvironmentalCommands::GET_VOLTAGE);
    msg->hdr.data_size = sizeof(sensorId);
    msg->sensor_id = sensorId;

    return 0;
}

int decodeGetVoltageResponse(std::span<const uint8_t> buf,
                             ocp::accelerator_management::CompletionCode& cc,
                             uint16_t& reasonCode, uint32_t& voltage)
{
    auto rc =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (buf.size() < sizeof(GetVoltageResponse))
    {
        return EINVAL;
    }

    const auto* response =
        reinterpret_cast<const GetVoltageResponse*>(buf.data());

    const uint16_t dataSize = le16toh(response->hdr.data_size);

    if (dataSize != sizeof(uint32_t))
    {
        return EINVAL;
    }

    voltage = le32toh(response->voltage);

    return 0;
}

int encodeGetInventoryInformationRequest(uint8_t instanceId, uint8_t propertyId,
                                         std::span<uint8_t> buf)
{
    if (buf.size() < sizeof(GetInventoryInformationRequest))
    {
        return EINVAL;
    }

    auto* msg = reinterpret_cast<GetInventoryInformationRequest*>(buf.data());

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
        PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION);
    msg->hdr.data_size = sizeof(propertyId);
    msg->property_id = propertyId;

    return 0;
}

int decodeGetInventoryInformationResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    InventoryPropertyId propertyId, InventoryValue& value)
{
    auto rc =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);
    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }
    // Expect at least one byte of inventory response data after common response
    if (buf.size() < (sizeof(ocp::accelerator_management::CommonResponse) + 1))
    {
        return EINVAL;
    }

    const auto* response =
        reinterpret_cast<const GetInventoryInformationResponse*>(buf.data());
    uint16_t dataSize = le16toh(response->hdr.data_size);

    if (dataSize == 0 || dataSize > maxInventoryDataSize)
    {
        return EINVAL;
    }

    const uint8_t* dataPtr = response->data.data();

    switch (propertyId)
    {
        case InventoryPropertyId::BOARD_PART_NUMBER:
        case InventoryPropertyId::SERIAL_NUMBER:
        case InventoryPropertyId::MARKETING_NAME:
        case InventoryPropertyId::DEVICE_PART_NUMBER:
            value =
                std::string(reinterpret_cast<const char*>(dataPtr), dataSize);
            break;
        case InventoryPropertyId::DEVICE_GUID:
            value = std::vector<uint8_t>(dataPtr, dataPtr + dataSize);
            break;
        default:
            return EINVAL;
    }
    return 0;
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
} // namespace gpu
