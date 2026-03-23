/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMctpVdm.hpp"

#include "MessagePackUnpackUtils.hpp"
#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace gpu
{
int encodeRequestCommonHeader(PackBuffer& buffer, gpu::MessageType msgType,
                              uint8_t command, uint8_t instanceId)
{
    const int rc = ocp::accelerator_management::packHeader(
        buffer, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::REQUEST, instanceId,
        static_cast<uint8_t>(msgType));

    if (rc != 0)
    {
        return rc;
    }

    return buffer.pack(command);
}

int decodeResponseCommonHeader(
    UnpackBuffer& buffer, gpu::MessageType msgType, uint8_t command,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode)
{
    ocp::accelerator_management::MessageType receivedMsgType{};
    uint8_t instanceId = 0;
    uint8_t receivedMessageType = 0;

    int rc = ocp::accelerator_management::unpackHeader(
        buffer, gpu::nvidiaPciVendorId, receivedMsgType, instanceId,
        receivedMessageType);

    if (rc != 0)
    {
        return rc;
    }

    if (receivedMsgType != ocp::accelerator_management::MessageType::RESPONSE)
    {
        return EINVAL;
    }

    if (receivedMessageType != static_cast<uint8_t>(msgType))
    {
        return EINVAL;
    }

    uint8_t receivedCommand = 0;
    rc = buffer.unpack(receivedCommand);

    if (rc != 0)
    {
        return rc;
    }

    if (command != receivedCommand)
    {
        return EINVAL;
    }

    rc = ocp::accelerator_management::unpackReasonCodeAndCC(
        buffer, cc, reasonCode);

    return rc;
}

int decodeAggregateResponse(
    UnpackBuffer& buffer, gpu::MessageType msgType, uint8_t command,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    std::move_only_function<int(const uint8_t tag, const uint8_t length,
                                UnpackBuffer& buffer)>
        handler)
{
    ocp::accelerator_management::MessageType receivedMsgType{};
    uint8_t instanceId = 0;
    uint8_t receivedMessageType = 0;

    int rc = ocp::accelerator_management::unpackHeader(
        buffer, gpu::nvidiaPciVendorId, receivedMsgType, instanceId,
        receivedMessageType);

    if (rc != 0)
    {
        return rc;
    }

    if (receivedMsgType != ocp::accelerator_management::MessageType::RESPONSE)
    {
        return EINVAL;
    }

    if (receivedMessageType != static_cast<uint8_t>(msgType))
    {
        return EINVAL;
    }

    uint8_t receivedCommand = 0;
    rc = buffer.unpack(receivedCommand);

    if (rc != 0)
    {
        return rc;
    }

    if (command != receivedCommand)
    {
        return EINVAL;
    }

    uint8_t completionCode = 0;
    rc = buffer.unpack(completionCode);

    if (rc != 0)
    {
        return rc;
    }

    cc = static_cast<ocp::accelerator_management::CompletionCode>(
        completionCode);

    if (cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        uint16_t receivedReasonCode = 0;
        rc = buffer.unpack(receivedReasonCode);
        if (rc != 0)
        {
            return rc;
        }
        reasonCode = receivedReasonCode;
        return 0;
    }

    reasonCode = 0;

    rc = ocp::accelerator_management::unpackAggregateResponse(
        buffer, std::move(handler));

    return rc;
}

int encodeQueryDeviceIdentificationRequest(uint8_t instanceId,
                                           const std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    int rc = encodeRequestCommonHeader(
        buffer, MessageType::DEVICE_CAPABILITY_DISCOVERY,
        static_cast<uint8_t>(
            DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 0;
    buffer.pack(dataSize);

    return buffer.getError();
}

int decodeQueryDeviceIdentificationResponse(
    const std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint8_t& deviceIdentification, uint8_t& deviceInstance)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::DEVICE_CAPABILITY_DISCOVERY,
        static_cast<uint8_t>(
            DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION),
        cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    rc = buffer.unpack(dataSize);

    if (rc != 0)
    {
        return rc;
    }

    if (dataSize != sizeof(uint8_t) * 2)
    {
        return EINVAL;
    }

    buffer.unpack(deviceIdentification);
    buffer.unpack(deviceInstance);

    return buffer.getError();
}

int encodeGetTemperatureReadingRequest(uint8_t instanceId, uint8_t sensorId,
                                       std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_TEMPERATURE_READING),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 1;
    buffer.pack(dataSize);
    buffer.pack(sensorId);

    return buffer.getError();
}

int decodeGetTemperatureReadingResponse(
    const std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    double& temperatureReading)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_TEMPERATURE_READING),
        cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    rc = buffer.unpack(dataSize);

    if (rc != 0)
    {
        return rc;
    }

    if (dataSize != sizeof(int32_t))
    {
        return EINVAL;
    }

    int32_t reading = 0;
    rc = buffer.unpack(reading);

    if (rc != 0)
    {
        return rc;
    }

    temperatureReading = reading / static_cast<double>(1 << 8);

    return 0;
}

int encodeReadThermalParametersRequest(uint8_t instanceId, uint8_t sensorId,
                                       std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 1;
    buffer.pack(dataSize);
    buffer.pack(sensorId);

    return buffer.getError();
}

int decodeReadThermalParametersResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    int32_t& threshold)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS),
        cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    rc = buffer.unpack(dataSize);

    if (rc != 0)
    {
        return rc;
    }

    if (dataSize != sizeof(int32_t))
    {
        return EINVAL;
    }

    rc = buffer.unpack(threshold);

    return rc;
}

int encodeGetPowerDrawRequest(PlatformEnvironmentalCommands commandCode,
                              uint8_t instanceId, uint8_t sensorId,
                              uint8_t averagingInterval, std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(commandCode), instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = sizeof(sensorId) + sizeof(averagingInterval);
    buffer.pack(dataSize);
    buffer.pack(sensorId);
    buffer.pack(averagingInterval);

    return buffer.getError();
}

int decodeGetPowerDrawResponse(std::span<const uint8_t> buf,
                               ocp::accelerator_management::CompletionCode& cc,
                               uint16_t& reasonCode, uint32_t& power)
{
    UnpackBuffer buffer(buf);

    ocp::accelerator_management::MessageType receivedMsgType{};
    uint8_t instanceId = 0;
    uint8_t receivedMessageType = 0;

    int rc = ocp::accelerator_management::unpackHeader(
        buffer, gpu::nvidiaPciVendorId, receivedMsgType, instanceId,
        receivedMessageType);

    if (rc != 0)
    {
        return rc;
    }

    if (receivedMsgType != ocp::accelerator_management::MessageType::RESPONSE)
    {
        return EINVAL;
    }

    if (receivedMessageType !=
        static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL))
    {
        return EINVAL;
    }

    uint8_t receivedCommand = 0;
    buffer.unpack(receivedCommand);

    rc = ocp::accelerator_management::unpackReasonCodeAndCC(
        buffer, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    rc = buffer.unpack(dataSize);

    if (rc != 0)
    {
        return rc;
    }

    if (dataSize != sizeof(uint32_t))
    {
        return EINVAL;
    }

    rc = buffer.unpack(power);

    return rc;
}

int encodeGetCurrentEnergyCounterRequest(uint8_t instanceId, uint8_t sensorId,
                                         std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_CURRENT_ENERGY_COUNTER),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 1;
    buffer.pack(dataSize);
    buffer.pack(sensorId);

    return buffer.getError();
}

int decodeGetCurrentEnergyCounterResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint64_t& energy)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_CURRENT_ENERGY_COUNTER),
        cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    rc = buffer.unpack(dataSize);

    if (rc != 0)
    {
        return rc;
    }

    if (dataSize != sizeof(uint64_t))
    {
        return EINVAL;
    }

    rc = buffer.unpack(energy);

    return rc;
}

int encodeGetVoltageRequest(uint8_t instanceId, uint8_t sensorId,
                            std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(PlatformEnvironmentalCommands::GET_VOLTAGE),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 1;
    buffer.pack(dataSize);
    buffer.pack(sensorId);

    return buffer.getError();
}

int decodeGetVoltageResponse(std::span<const uint8_t> buf,
                             ocp::accelerator_management::CompletionCode& cc,
                             uint16_t& reasonCode, uint32_t& voltage)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(PlatformEnvironmentalCommands::GET_VOLTAGE), cc,
        reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    rc = buffer.unpack(dataSize);

    if (rc != 0)
    {
        return rc;
    }

    if (dataSize != sizeof(uint32_t))
    {
        return EINVAL;
    }

    rc = buffer.unpack(voltage);

    return rc;
}

int encodeGetDriverInformationRequest(uint8_t instanceId,
                                      std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 0;
    buffer.pack(dataSize);

    return buffer.getError();
}

int decodeGetDriverInformationResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    DriverState& driverState, std::string& driverVersion)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION),
        cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    rc = buffer.unpack(dataSize);

    if (rc != 0)
    {
        return rc;
    }

    if (dataSize < sizeof(DriverState) + sizeof(char))
    {
        return EINVAL;
    }

    uint8_t driverStateValue = 0;
    rc = buffer.unpack(driverStateValue);

    if (rc != 0)
    {
        return rc;
    }

    driverState = static_cast<DriverState>(driverStateValue);

    const size_t versionSize = buf.size() - getDriverInformationResponseSize;

    if (buf.size() < getDriverInformationResponseSize)
    {
        return EINVAL;
    }

    driverVersion.resize(versionSize);
    std::memcpy(driverVersion.data(),
                buf.data() + ocp::accelerator_management::commonResponseSize +
                    sizeof(uint8_t),
                versionSize);

    return 0;
}

int encodeGetInventoryInformationRequest(uint8_t instanceId, uint8_t propertyId,
                                         std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 1;
    buffer.pack(dataSize);
    buffer.pack(propertyId);

    return buffer.getError();
}

int decodeGetInventoryInformationResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    InventoryPropertyId propertyId, InventoryValue& value)
{
    UnpackBuffer buffer(buf);

    ocp::accelerator_management::MessageType receivedMsgType{};
    uint8_t instanceId = 0;
    uint8_t receivedMessageType = 0;

    int rc = ocp::accelerator_management::unpackHeader(
        buffer, gpu::nvidiaPciVendorId, receivedMsgType, instanceId,
        receivedMessageType);

    if (rc != 0)
    {
        return rc;
    }

    if (receivedMsgType != ocp::accelerator_management::MessageType::RESPONSE)
    {
        return EINVAL;
    }

    uint8_t receivedCommand = 0;
    buffer.unpack(receivedCommand);

    rc = ocp::accelerator_management::unpackReasonCodeAndCC(
        buffer, cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    rc = buffer.unpack(dataSize);

    if (rc != 0)
    {
        return rc;
    }

    if (dataSize == 0 || dataSize > maxInventoryDataSize)
    {
        return EINVAL;
    }

    const size_t dataOffset = ocp::accelerator_management::commonResponseSize;

    if (buf.size() < dataOffset + dataSize)
    {
        return EINVAL;
    }

    const uint8_t* dataPtr = buf.data() + dataOffset;

    switch (propertyId)
    {
        case InventoryPropertyId::BOARD_PART_NUMBER:
        case InventoryPropertyId::SERIAL_NUMBER:
        case InventoryPropertyId::MARKETING_NAME:
        case InventoryPropertyId::DEVICE_PART_NUMBER:
        {
            std::string str(dataSize, '\0');
            std::memcpy(str.data(), dataPtr, dataSize);
            value = std::move(str);
            break;
        }
        case InventoryPropertyId::DEVICE_GUID:
            value = std::vector<uint8_t>(dataPtr, dataPtr + dataSize);
            break;
        case InventoryPropertyId::DEFAULT_BOOST_CLOCKS:
        {
            if (dataSize != sizeof(uint32_t))
            {
                return EINVAL;
            }
            uint32_t clockMhz = 0;
            std::memcpy(&clockMhz, dataPtr, sizeof(uint32_t));
            clockMhz = le32toh(clockMhz);
            value = clockMhz;
            break;
        }
        default:
            return EINVAL;
    }
    return 0;
}

int encodeQueryScalarGroupTelemetryV2Request(
    uint8_t instanceId, PciePortType portType, uint8_t upstreamPortNumber,
    uint8_t portNumber, uint8_t groupId, std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::PCIE_LINK,
        static_cast<uint8_t>(PcieLinkCommands::QueryScalarGroupTelemetryV2),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 3;
    buffer.pack(dataSize);
    const uint8_t upstreamPortField =
        (static_cast<uint8_t>(portType) << 7) | (upstreamPortNumber & 0x7F);
    buffer.pack(upstreamPortField);
    buffer.pack(portNumber);
    buffer.pack(groupId);

    return buffer.getError();
}

int decodeQueryScalarGroupTelemetryV2Response(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    size_t& numTelemetryValues, std::vector<uint32_t>& telemetryValues)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PCIE_LINK,
        static_cast<uint8_t>(PcieLinkCommands::QueryScalarGroupTelemetryV2), cc,
        reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    rc = buffer.unpack(dataSize);

    if (rc != 0)
    {
        return rc;
    }

    if (buf.size() < dataSize + ocp::accelerator_management::commonResponseSize)
    {
        return EINVAL;
    }

    numTelemetryValues = dataSize / sizeof(uint32_t);

    if (telemetryValues.size() < numTelemetryValues)
    {
        telemetryValues.resize(numTelemetryValues);
    }

    for (size_t i = 0; i < numTelemetryValues; i++)
    {
        rc = buffer.unpack(telemetryValues[i]);

        if (rc != 0)
        {
            return rc;
        }
    }

    return 0;
}

int encodeListPciePortsRequest(uint8_t instanceId, std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::PCIE_LINK,
        static_cast<uint8_t>(PcieLinkCommands::ListPCIePorts), instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 0;
    buffer.pack(dataSize);

    return buffer.getError();
}

int decodeListPciePortsResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint16_t& numUpstreamPorts, std::vector<uint8_t>& numDownstreamPorts)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PCIE_LINK,
        static_cast<uint8_t>(PcieLinkCommands::ListPCIePorts), cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    rc = buffer.unpack(dataSize);

    if (rc != 0)
    {
        return rc;
    }

    if (dataSize < sizeof(uint16_t))
    {
        return EINVAL;
    }

    uint16_t upstreamPorts = 0;
    rc = buffer.unpack(upstreamPorts);

    if (rc != 0)
    {
        return rc;
    }

    numUpstreamPorts = 0;
    numDownstreamPorts.clear();
    numDownstreamPorts.reserve(upstreamPorts);

    for (size_t i = 0; i < upstreamPorts; i++)
    {
        uint8_t isInternal = 0;
        uint8_t count = 0;

        buffer.unpack(isInternal);
        buffer.unpack(count);

        if (buffer.getError() != 0)
        {
            return EINVAL;
        }

        // Count only external upstream ports
        if (isInternal == 0)
        {
            ++numUpstreamPorts;
            numDownstreamPorts.push_back(count);
        }
    }

    return 0;
}

int encodeGetPortNetworkAddressesRequest(
    uint8_t instanceId, uint16_t portNumber, std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::NETWORK_PORT,
        static_cast<uint8_t>(NetworkPortCommands::GetPortNetworkAddresses),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = sizeof(portNumber);
    buffer.pack(dataSize);
    buffer.pack(portNumber);

    return buffer.getError();
}

int decodeGetPortNetworkAddressesResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    NetworkPortLinkType& linkType,
    std::vector<std::pair<uint8_t, uint64_t>>& addresses)
{
    UnpackBuffer buffer(buf);

    addresses.clear();
    addresses.reserve(std::numeric_limits<uint8_t>::max());

    const int rc = decodeAggregateResponse(
        buffer, MessageType::NETWORK_PORT,
        static_cast<uint8_t>(NetworkPortCommands::GetPortNetworkAddresses), cc,
        reasonCode,
        [&linkType, &addresses](const uint8_t tag, const uint8_t length,
                                UnpackBuffer& buffer) -> int {
            if (tag == 0 && length == 1)
            {
                uint8_t linkTypeValue = 0;
                int rc = buffer.unpack(linkTypeValue);
                if (rc != 0)
                {
                    return rc;
                }
                linkType = static_cast<NetworkPortLinkType>(linkTypeValue);
                return 0;
            }

            if (length == sizeof(uint64_t))
            {
                uint64_t telemetryData = 0;
                int rc = buffer.unpack(telemetryData);
                if (rc != 0)
                {
                    return rc;
                }
                addresses.emplace_back(tag, telemetryData);
                return 0;
            }

            buffer.skip(length);
            return 0;
        });

    return rc;
}

int encodeGetEthernetPortTelemetryCountersRequest(
    uint8_t instanceId, uint16_t portNumber, std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::NETWORK_PORT,
        static_cast<uint8_t>(
            NetworkPortCommands::GetEthernetPortTelemetryCounters),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = sizeof(portNumber);
    buffer.pack(dataSize);
    buffer.pack(portNumber);

    return buffer.getError();
}

int decodeGetEthernetPortTelemetryCountersResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    std::vector<std::pair<uint8_t, uint64_t>>& telemetryValues)
{
    UnpackBuffer buffer(buf);

    telemetryValues.clear();
    telemetryValues.reserve(std::numeric_limits<uint8_t>::max());

    const int rc = decodeAggregateResponse(
        buffer, MessageType::NETWORK_PORT,
        static_cast<uint8_t>(
            NetworkPortCommands::GetEthernetPortTelemetryCounters),
        cc, reasonCode,
        [&telemetryValues](const uint8_t tag, const uint8_t length,
                           UnpackBuffer& buffer) -> int {
            uint64_t telemetryData = 0;

            if (length == sizeof(uint32_t))
            {
                uint32_t telemetryValue = 0;
                int rc = buffer.unpack(telemetryValue);
                if (rc != 0)
                {
                    return rc;
                }
                telemetryData = telemetryValue;
            }
            else if (length == sizeof(uint64_t))
            {
                int rc = buffer.unpack(telemetryData);
                if (rc != 0)
                {
                    return rc;
                }
            }
            else
            {
                buffer.skip(length);
                return 0;
            }

            telemetryValues.emplace_back(tag, telemetryData);

            return 0;
        });

    return rc;
}
} // namespace gpu
