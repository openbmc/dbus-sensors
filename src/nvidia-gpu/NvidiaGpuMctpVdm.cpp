/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMctpVdm.hpp"

#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <bit>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <span>
#include <utility>
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

int encodeQueryScalarGroupTelemetryV2Request(
    uint8_t instanceId, PciePortType portType, uint8_t upstreamPortNumber,
    uint8_t portNumber, uint8_t groupId, std::span<uint8_t> buf)
{
    if (buf.size() < sizeof(QueryScalarGroupTelemetryV2Request))
    {
        return EINVAL;
    }

    auto* msg =
        reinterpret_cast<QueryScalarGroupTelemetryV2Request*>(buf.data());

    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdBitMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PCIE_LINK);

    auto rc = packHeader(header, msg->hdr.msgHdr.hdr);

    if (rc != 0)
    {
        return rc;
    }

    msg->hdr.command =
        static_cast<uint8_t>(PcieLinkCommands::QueryScalarGroupTelemetryV2);
    msg->hdr.data_size = 3;
    msg->upstreamPortNumber =
        (static_cast<uint8_t>(portType) << 7) | (upstreamPortNumber & 0x7F);
    msg->portNumber = portNumber;
    msg->groupId = groupId;

    return 0;
}

int decodeQueryScalarGroupTelemetryV2Response(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    size_t& numTelemetryValues, std::vector<uint32_t>& telemetryValues)
{
    auto rc =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (buf.size() < sizeof(ocp::accelerator_management::CommonResponse))
    {
        return EINVAL;
    }

    const auto* response =
        reinterpret_cast<const ocp::accelerator_management::CommonResponse*>(
            buf.data());

    const uint16_t dataSize = le16toh(response->data_size);

    if (buf.size() <
        dataSize + sizeof(ocp::accelerator_management::CommonResponse))
    {
        return EINVAL;
    }

    numTelemetryValues = dataSize / sizeof(uint32_t);

    if (telemetryValues.size() < numTelemetryValues)
    {
        telemetryValues.resize(numTelemetryValues);
    }

    const auto* telemetryDataPtr = reinterpret_cast<const uint32_t*>(
        buf.data() + sizeof(ocp::accelerator_management::CommonResponse));

    for (size_t i = 0; i < numTelemetryValues; i++)
    {
        telemetryValues[i] = le32toh(telemetryDataPtr[i]);
    }

    return 0;
}

int encodeListPciePortsRequest(uint8_t instanceId, std::span<uint8_t> buf)
{
    if (buf.size() < sizeof(ocp::accelerator_management::CommonRequest))
    {
        return EINVAL;
    }

    auto* msg = reinterpret_cast<ocp::accelerator_management::CommonRequest*>(
        buf.data());

    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdBitMask;
    header.msg_type = static_cast<uint8_t>(MessageType::PCIE_LINK);

    auto rc = packHeader(header, msg->msgHdr.hdr);

    if (rc != 0)
    {
        return rc;
    }

    msg->command = static_cast<uint8_t>(PcieLinkCommands::ListPCIePorts);
    msg->data_size = 0;

    return 0;
}

int decodeListPciePortsResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint16_t& numUpstreamPorts, std::vector<uint8_t>& numDownstreamPorts)
{
    auto rc =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (buf.size() < sizeof(ocp::accelerator_management::CommonResponse))
    {
        return EINVAL;
    }

    const auto* response =
        reinterpret_cast<const ocp::accelerator_management::CommonResponse*>(
            buf.data());

    const uint16_t dataSize = le16toh(response->data_size);

    if (buf.size() <
        dataSize + sizeof(ocp::accelerator_management::CommonResponse))
    {
        return EINVAL;
    }

    if (dataSize < sizeof(uint16_t))
    {
        return EINVAL;
    }

    const uint16_t upstreamPorts = le16toh(*reinterpret_cast<const uint16_t*>(
        buf.data() + sizeof(ocp::accelerator_management::CommonResponse)));

    numUpstreamPorts = 0;
    numDownstreamPorts.clear();
    numDownstreamPorts.reserve(upstreamPorts);

    for (size_t i = 0; i < upstreamPorts; i++)
    {
        const size_t offset =
            sizeof(ocp::accelerator_management::CommonResponse) +
            sizeof(uint16_t) + i * 2 * sizeof(uint8_t);

        if (offset + 2 * sizeof(uint8_t) > buf.size())
        {
            return EINVAL;
        }

        const uint8_t isInternal =
            *reinterpret_cast<const uint8_t*>(buf.data() + offset);

        // Count only external upstream ports
        if (isInternal == 0)
        {
            ++numUpstreamPorts;

            const uint8_t downstreamPorts = *reinterpret_cast<const uint8_t*>(
                buf.data() + offset + sizeof(uint8_t));
            numDownstreamPorts.push_back(downstreamPorts);
        }
    }

    return 0;
}

int encodeGetEthernetPortTelemetryCounters(
    uint8_t instanceId, uint16_t portNumber, std::span<uint8_t> buf)
{
    if (buf.size() < sizeof(GetEthernetPortTelemetryCountersRequest))
    {
        return EINVAL;
    }

    auto* msg =
        std::bit_cast<GetEthernetPortTelemetryCountersRequest*>(buf.data());

    ocp::accelerator_management::BindingPciVidInfo header{};
    header.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    header.instance_id = instanceId &
                         ocp::accelerator_management::instanceIdBitMask;
    header.msg_type = static_cast<uint8_t>(MessageType::NETWORK_PORT);

    auto rc = packHeader(header, msg->hdr.msgHdr.hdr);

    if (rc != 0)
    {
        return rc;
    }

    msg->hdr.command = static_cast<uint8_t>(
        NetworkPortCommands::GetEthernetPortTelemetryCounters);
    msg->hdr.data_size = 2;
    msg->portNumber = le16toh(portNumber);

    return 0;
}

int decodeAggregateResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    std::move_only_function<int(const uint8_t tag, const uint8_t length,
                                const uint8_t* value)>
        handler)
{
    auto rc =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    if (buf.size() <
        sizeof(ocp::accelerator_management::CommonAggregateResponse))
    {
        return EINVAL;
    }

    const auto* response = std::bit_cast<
        const ocp::accelerator_management::CommonAggregateResponse*>(
        buf.data());

    size_t index = sizeof(ocp::accelerator_management::CommonAggregateResponse);

    for (uint16_t telemetryCount = 0;
         telemetryCount < le16toh(response->telemetryCount); ++telemetryCount)
    {
        const size_t valueOffset = index + 2;

        if (buf.size() < valueOffset)
        {
            break;
        }

        const uint8_t tag = buf[index];
        const uint8_t tagInfo = buf[index + 1];
        const bool isValid = (0x01 & tagInfo) != 0;
        const bool isByteLengthEncoding = (0x80 & tagInfo) != 0;
        const uint8_t encodedLength = (tagInfo >> 1) & 0x07;

        uint8_t length = 0;
        if (isByteLengthEncoding)
        {
            length = encodedLength;
        }
        else
        {
            length = 1 << encodedLength;
        }

        index = valueOffset + length;

        if (isValid)
        {
            if (buf.size() < index)
            {
                break;
            }

            handler(tag, length, buf.data() + valueOffset);
        }
    }

    return 0;
}

int decodeGetEthernetPortTelemetryCounters(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    std::vector<std::pair<uint8_t, uint64_t>>& telemetryValues)
{
    telemetryValues.clear();
    telemetryValues.reserve(std::numeric_limits<uint8_t>::max());

    const int rc = decodeAggregateResponse(
        buf, cc, reasonCode,
        [&telemetryValues](const uint8_t tag, const uint8_t length,
                           const uint8_t* value) -> int {
            uint64_t telemetryData = 0;

            if (length == 4)
            {
                const auto* telemetryDataPtr =
                    reinterpret_cast<const uint32_t*>(value);

                telemetryData = le32toh(*telemetryDataPtr);
            }
            else if (length == 8)
            {
                const auto* telemetryDataPtr =
                    reinterpret_cast<const uint64_t*>(value);
                telemetryData = le64toh(*telemetryDataPtr);
            }
            else
            {
                return 0;
            }

            telemetryValues.emplace_back(tag, telemetryData);

            return 0;
        });

    return rc;
}
// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
} // namespace gpu
