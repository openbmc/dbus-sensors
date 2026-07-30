/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMctpVdm.hpp"

#include "MessagePackUnpackUtils.hpp"
#include "OcpMctpVdm.hpp"

#include <array>
#include <bit>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
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

int encodeSetEventSubscriptionRequest(uint8_t generationSetting, uint8_t eid,
                                      std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    int rc = encodeRequestCommonHeader(
        buffer, MessageType::DEVICE_CAPABILITY_DISCOVERY,
        static_cast<uint8_t>(
            DeviceCapabilityDiscoveryCommands::SET_EVENT_SUBSCRIPTION),
        0);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 2;
    buffer.pack(dataSize);
    buffer.pack(generationSetting);
    buffer.pack(eid);

    return buffer.getError();
}

int decodeSetEventSubscriptionResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::DEVICE_CAPABILITY_DISCOVERY,
        static_cast<uint8_t>(
            DeviceCapabilityDiscoveryCommands::SET_EVENT_SUBSCRIPTION),
        cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    return 0;
}

int encodeSetEventSourcesRequest(uint64_t sources, uint8_t messageType,
                                 std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    int rc = encodeRequestCommonHeader(
        buffer, MessageType::DEVICE_CAPABILITY_DISCOVERY,
        static_cast<uint8_t>(
            DeviceCapabilityDiscoveryCommands::SET_CURRENT_EVENT_SOURCES),
        0);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 9;
    buffer.pack(dataSize);
    buffer.pack(messageType);
    buffer.pack(sources);

    return buffer.getError();
}

int decodeSetEventSourcesResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::DEVICE_CAPABILITY_DISCOVERY,
        static_cast<uint8_t>(
            DeviceCapabilityDiscoveryCommands::SET_CURRENT_EVENT_SOURCES),
        cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    return 0;
}

int decodeLongRunningResponseEvent(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint8_t& instanceId, std::span<const uint8_t>& responseData)
{
    UnpackBuffer buffer(buf);

    uint8_t completionCode = 0;

    buffer.unpack(instanceId);
    buffer.unpack(completionCode);
    buffer.unpack(reasonCode);

    if (buffer.getError() != 0)
    {
        return buffer.getError();
    }

    cc = static_cast<ocp::accelerator_management::CompletionCode>(
        completionCode);

    responseData = buf.subspan(longRunningResponseEventSize);

    return 0;
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
    rc = buffer.unpack(receivedCommand);

    if (rc != 0)
    {
        return rc;
    }

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

int encodeGetPowerLimitsRequest(uint8_t instanceId, uint32_t powerLimitId,
                                std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(PlatformEnvironmentalCommands::GET_POWER_LIMITS),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = sizeof(powerLimitId);
    buffer.pack(dataSize);
    buffer.pack(powerLimitId);

    return buffer.getError();
}

int decodeGetPowerLimitsResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint32_t& persistentPowerLimitRequested,
    uint32_t& oneshotPowerLimitRequested, uint32_t& powerLimitEnforced)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(PlatformEnvironmentalCommands::GET_POWER_LIMITS),
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

    if (dataSize != sizeof(uint32_t) * 3)
    {
        return EINVAL;
    }

    buffer.unpack(persistentPowerLimitRequested);
    buffer.unpack(oneshotPowerLimitRequested);
    buffer.unpack(powerLimitEnforced);

    return buffer.getError();
}

int encodeSetPowerLimitsRequest(
    uint8_t instanceId, uint32_t powerLimitId, SetPowerLimitsAction action,
    SetPowerLimitsPersistence persistence, uint32_t powerLimitMilliwatts,
    std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(PlatformEnvironmentalCommands::SET_POWER_LIMITS),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = sizeof(powerLimitId) + sizeof(uint8_t) +
                             sizeof(uint8_t) + sizeof(powerLimitMilliwatts);
    buffer.pack(dataSize);
    buffer.pack(powerLimitId);
    buffer.pack(static_cast<uint8_t>(action));
    buffer.pack(static_cast<uint8_t>(persistence));
    buffer.pack(powerLimitMilliwatts);

    return buffer.getError();
}

int decodeSetPowerLimitsResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(PlatformEnvironmentalCommands::SET_POWER_LIMITS),
        cc, reasonCode);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    return 0;
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

    if (dataSize < 2)
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

    std::span<const uint8_t> remainingBuffer = buffer.getRemaining();

    if (remainingBuffer.empty())
    {
        return EINVAL;
    }

    driverVersion.clear();
    driverVersion.assign(remainingBuffer.begin(),
                         std::prev(remainingBuffer.end()));

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

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION),
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

    if (dataSize == 0 || dataSize > maxInventoryDataSize)
    {
        return EINVAL;
    }

    switch (propertyId)
    {
        case InventoryPropertyId::BOARD_PART_NUMBER:
        case InventoryPropertyId::SERIAL_NUMBER:
        case InventoryPropertyId::MARKETING_NAME:
        case InventoryPropertyId::DEVICE_PART_NUMBER:
        {
            std::span<const uint8_t> remainingBuffer = buffer.getRemaining();

            if (remainingBuffer.size() < dataSize)
            {
                return EINVAL;
            }

            value = std::string(remainingBuffer.begin(),
                                remainingBuffer.begin() + dataSize);
            break;
        }
        case InventoryPropertyId::DEVICE_GUID:
        {
            std::span<const uint8_t> remainingBuffer = buffer.getRemaining();

            if (remainingBuffer.size() < dataSize)
            {
                return EINVAL;
            }

            value = std::vector<uint8_t>(remainingBuffer.begin(),
                                         remainingBuffer.begin() + dataSize);
            break;
        }
        case InventoryPropertyId::DEFAULT_BOOST_CLOCKS:
        case InventoryPropertyId::DEFAULT_BASE_CLOCKS:
        case InventoryPropertyId::MIN_GRAPHICS_CLOCK:
        case InventoryPropertyId::MAX_GRAPHICS_CLOCK:
        case InventoryPropertyId::RATED_DEVICE_POWER_LIMIT:
        case InventoryPropertyId::MIN_DEVICE_POWER_LIMIT:
        case InventoryPropertyId::MAX_DEVICE_POWER_LIMIT:
        case InventoryPropertyId::MAX_MEMORY_CAPACITY:
        case InventoryPropertyId::MIN_MEMORY_CLOCK:
        case InventoryPropertyId::MAX_MEMORY_CLOCK:
        {
            uint32_t intValue = 0;
            rc = buffer.unpack(intValue);
            if (rc != 0)
            {
                return rc;
            }
            value = intValue;
            break;
        }
        default:
            return EINVAL;
    }
    return 0;
}

int encodeGetCurrentUtilizationModeRequest(uint8_t instanceId,
                                           std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 0;
    buffer.pack(dataSize);

    return buffer.getError();
}

int decodeGetCurrentUtilizationModeResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint32_t& gpuUtilization, uint32_t& memoryUtilization)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION),
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

    if (dataSize != sizeof(uint32_t) * 2)
    {
        return EINVAL;
    }

    buffer.unpack(gpuUtilization);
    buffer.unpack(memoryUtilization);

    return buffer.getError();
}

int decodeGetCurrentUtilizationModeResponse(std::span<const uint8_t> buf,
                                            uint32_t& gpuUtilization,
                                            uint32_t& memoryUtilization)
{
    UnpackBuffer buffer(buf);
    buffer.unpack(gpuUtilization);
    buffer.unpack(memoryUtilization);
    return buffer.getError();
}

int encodeGetClockLimitRequest(uint8_t instanceId, ClockType clockType,
                               std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(PlatformEnvironmentalCommands::GET_CLOCK_LIMIT),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = sizeof(uint8_t);
    buffer.pack(dataSize);
    buffer.pack(static_cast<uint8_t>(clockType));

    return buffer.getError();
}

int decodeGetClockLimitResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint32_t& requestedLimitMin, uint32_t& requestedLimitMax,
    uint32_t& presentLimitMin, uint32_t& presentLimitMax)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(PlatformEnvironmentalCommands::GET_CLOCK_LIMIT),
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

    if (dataSize != 4 * sizeof(uint32_t))
    {
        return EINVAL;
    }

    buffer.unpack(requestedLimitMin);
    buffer.unpack(requestedLimitMax);
    buffer.unpack(presentLimitMin);
    buffer.unpack(presentLimitMax);

    return buffer.getError();
}

int encodeGetViolationDurationRequest(uint8_t instanceId,
                                      std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_VIOLATION_DURATION),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 0;
    buffer.pack(dataSize);

    return buffer.getError();
}

int decodeGetViolationDurationResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint64_t& hwViolationDuration, uint64_t& globalSwViolationDuration,
    uint64_t& powerViolationDuration, uint64_t& thermalViolationDuration)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_VIOLATION_DURATION),
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

    if (dataSize != sizeof(uint64_t) * 4)
    {
        return EINVAL;
    }

    buffer.unpack(hwViolationDuration);
    buffer.unpack(globalSwViolationDuration);
    buffer.unpack(powerViolationDuration);
    buffer.unpack(thermalViolationDuration);

    return buffer.getError();
}

int decodeGetViolationDurationResponse(
    std::span<const uint8_t> buf, uint64_t& hwViolationDuration,
    uint64_t& globalSwViolationDuration, uint64_t& powerViolationDuration,
    uint64_t& thermalViolationDuration)
{
    UnpackBuffer buffer(buf);
    buffer.unpack(hwViolationDuration);
    buffer.unpack(globalSwViolationDuration);
    buffer.unpack(powerViolationDuration);
    buffer.unpack(thermalViolationDuration);
    return buffer.getError();
}

int encodeQueryScalarGroupTelemetryV1Request(
    uint8_t instanceId, uint8_t deviceIndex, PcieScalarGroupId groupId,
    std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    int rc = encodeRequestCommonHeader(
        buffer, MessageType::PCIE_LINK,
        static_cast<uint8_t>(PcieLinkCommands::QueryScalarGroupTelemetryV1),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 2;
    buffer.pack(dataSize);
    buffer.pack(deviceIndex);
    buffer.pack(static_cast<uint8_t>(groupId));

    return buffer.getError();
}

int encodeQueryScalarGroupTelemetryV2Request(
    uint8_t instanceId, PciePortType portType, uint8_t upstreamPortNumber,
    uint8_t portNumber, PcieScalarGroupId groupId, std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    int rc = encodeRequestCommonHeader(
        buffer, MessageType::PCIE_LINK,
        static_cast<uint8_t>(PcieLinkCommands::QueryScalarGroupTelemetryV2),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 3;
    buffer.pack(dataSize);
    const uint8_t encodedUpstreamPortNumber =
        (static_cast<uint8_t>(portType) << 7) | (upstreamPortNumber & 0x7F);
    buffer.pack(encodedUpstreamPortNumber);
    buffer.pack(portNumber);
    buffer.pack(static_cast<uint8_t>(groupId));

    return buffer.getError();
}

static int decodeQueryScalarGroupTelemetryResponseImpl(
    std::span<const uint8_t> buf, uint8_t expectedCommand,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    size_t& numTelemetryValues, std::vector<uint32_t>& telemetryValues)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(buffer, MessageType::PCIE_LINK,
                                        expectedCommand, cc, reasonCode);

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

int decodeQueryScalarGroupTelemetryV1Response(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    size_t& numTelemetryValues, std::vector<uint32_t>& telemetryValues)
{
    return decodeQueryScalarGroupTelemetryResponseImpl(
        buf,
        static_cast<uint8_t>(PcieLinkCommands::QueryScalarGroupTelemetryV1), cc,
        reasonCode, numTelemetryValues, telemetryValues);
}

int decodeQueryScalarGroupTelemetryV2Response(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    size_t& numTelemetryValues, std::vector<uint32_t>& telemetryValues)
{
    return decodeQueryScalarGroupTelemetryResponseImpl(
        buf,
        static_cast<uint8_t>(PcieLinkCommands::QueryScalarGroupTelemetryV2), cc,
        reasonCode, numTelemetryValues, telemetryValues);
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

int decodeXidEvent(std::span<const uint8_t> buf, uint8_t& flags,
                   uint32_t& eventMessageReason, uint32_t& sequenceNumber,
                   uint64_t& timestamp, std::string_view& messageTextString)
{
    UnpackBuffer buffer(buf);

    buffer.unpack(flags);

    // Skip 3 reserved bytes
    buffer.skip(3);

    buffer.unpack(eventMessageReason);
    buffer.unpack(sequenceNumber);
    buffer.unpack(timestamp);

    if (buffer.getError() != 0)
    {
        return buffer.getError();
    }

    std::span<const uint8_t> remainingData = buffer.getRemaining();
    messageTextString = std::string_view{
        std::bit_cast<const char*>(remainingData.data()), remainingData.size()};

    return 0;
}

int encodeGetEventLogRecordV2Request(
    uint8_t instanceId, EventLogRecordV2Mode mode, uint16_t eventHandle,
    uint16_t transferHandle, std::span<uint8_t> buf)
{
    if ((mode != EventLogRecordV2Mode::GET_DATA &&
         mode != EventLogRecordV2Mode::ACKNOWLEDGEMENT) ||
        eventHandle == std::numeric_limits<uint16_t>::max())
    {
        return EINVAL;
    }
    if (mode == EventLogRecordV2Mode::ACKNOWLEDGEMENT && transferHandle != 0)
    {
        return EINVAL;
    }

    PackBuffer buffer(buf);
    int rc = encodeRequestCommonHeader(
        buffer, MessageType::DEVICE_CAPABILITY_DISCOVERY,
        static_cast<uint8_t>(
            DeviceCapabilityDiscoveryCommands::GET_EVENT_LOG_RECORD_V2),
        instanceId);
    if (rc != 0)
    {
        return rc;
    }

    constexpr uint8_t dataSize = sizeof(uint8_t) + sizeof(uint16_t) * 2;
    buffer.pack(dataSize);
    buffer.pack(static_cast<uint8_t>(mode));
    buffer.pack(eventHandle);
    buffer.pack(transferHandle);
    return buffer.getError();
}

int decodeGetEventLogRecordV2FirstResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& completionCode,
    uint16_t& reasonCode, EventLogRecordV2FirstResponse& response)
{
    response = {};
    UnpackBuffer buffer(buf);
    int rc = decodeResponseCommonHeader(
        buffer, MessageType::DEVICE_CAPABILITY_DISCOVERY,
        static_cast<uint8_t>(
            DeviceCapabilityDiscoveryCommands::GET_EVENT_LOG_RECORD_V2),
        completionCode, reasonCode);
    if (rc != 0 ||
        completionCode != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    buffer.unpack(dataSize);
    if (buffer.getError() != 0 ||
        dataSize < getEventLogRecordV2NextResponseMinDataSize ||
        buffer.getRemaining().size() < dataSize)
    {
        return EINVAL;
    }

    buffer.unpack(response.nextTransferHandle);
    buffer.unpack(response.eventHandle);
    if (dataSize == getEventLogRecordV2NextResponseMinDataSize)
    {
        if (response.eventHandle != std::numeric_limits<uint16_t>::max() ||
            response.nextTransferHandle != 0)
        {
            return EINVAL;
        }
        return 0;
    }

    if (dataSize < getEventLogRecordV2FirstResponseMinDataSize ||
        response.eventHandle == std::numeric_limits<uint16_t>::max())
    {
        return EINVAL;
    }

    response.hasEventRecord = true;
    buffer.unpack(response.nvidiaMessageType);
    buffer.unpack(response.eventVersion);
    buffer.unpack(response.eventId);
    buffer.unpack(response.eventClass);
    buffer.unpack(response.eventState);
    if (buffer.getError() != 0)
    {
        return buffer.getError();
    }

    const size_t eventDataSize =
        dataSize - getEventLogRecordV2FirstResponseMinDataSize;
    if (buffer.getRemaining().size() < eventDataSize)
    {
        return EINVAL;
    }
    response.eventData = buffer.getRemaining().first(eventDataSize);
    return 0;
}

int decodeGetEventLogRecordV2NextResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& completionCode,
    uint16_t& reasonCode, EventLogRecordV2NextResponse& response)
{
    response = {};
    UnpackBuffer buffer(buf);
    int rc = decodeResponseCommonHeader(
        buffer, MessageType::DEVICE_CAPABILITY_DISCOVERY,
        static_cast<uint8_t>(
            DeviceCapabilityDiscoveryCommands::GET_EVENT_LOG_RECORD_V2),
        completionCode, reasonCode);
    if (rc != 0 ||
        completionCode != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        return rc;
    }

    uint16_t dataSize = 0;
    buffer.unpack(dataSize);
    if (buffer.getError() != 0 ||
        dataSize < getEventLogRecordV2NextResponseMinDataSize ||
        buffer.getRemaining().size() < dataSize)
    {
        return EINVAL;
    }

    buffer.unpack(response.nextTransferHandle);
    buffer.unpack(response.eventHandle);
    if (buffer.getError() != 0)
    {
        return buffer.getError();
    }

    const size_t eventDataSize =
        dataSize - getEventLogRecordV2NextResponseMinDataSize;
    response.eventData = buffer.getRemaining().first(eventDataSize);
    return 0;
}

namespace
{

template <typename T>
int readCperValue(std::span<const uint8_t> buf, size_t offset, T& value)
{
    if (offset > buf.size() || buf.size() - offset < sizeof(T))
    {
        return EINVAL;
    }
    UnpackBuffer buffer(buf.subspan(offset, sizeof(T)));
    return buffer.unpack(value);
}

std::string formatCperGuid(std::span<const uint8_t, 16> guid)
{
    return std::format("{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-"
                       "{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
                       guid[3], guid[2], guid[1], guid[0], guid[5], guid[4],
                       guid[7], guid[6], guid[8], guid[9], guid[10], guid[11],
                       guid[12], guid[13], guid[14], guid[15]);
}

bool decodeBcd(uint8_t value, uint8_t maximum, uint8_t& decoded)
{
    const uint8_t high = value >> 4;
    const uint8_t low = value & 0x0F;
    if (high > 9 || low > 9)
    {
        return false;
    }
    decoded = static_cast<uint8_t>(high * 10 + low);
    return decoded <= maximum;
}

bool isLeapYear(uint16_t year)
{
    return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

std::optional<std::string> decodeCperTimestamp(
    std::span<const uint8_t, 8> timestamp)
{
    uint8_t second = 0;
    uint8_t minute = 0;
    uint8_t hour = 0;
    uint8_t day = 0;
    uint8_t month = 0;
    uint8_t year = 0;
    uint8_t century = 0;
    if (!decodeBcd(timestamp[0], 59, second) ||
        !decodeBcd(timestamp[1], 59, minute) ||
        !decodeBcd(timestamp[2], 23, hour) ||
        !decodeBcd(timestamp[4], 31, day) ||
        !decodeBcd(timestamp[5], 12, month) ||
        !decodeBcd(timestamp[6], 99, year) ||
        !decodeBcd(timestamp[7], 99, century) || month == 0 || day == 0)
    {
        return std::nullopt;
    }

    const uint16_t fullYear = static_cast<uint16_t>(century * 100 + year);
    if (fullYear == 0)
    {
        return std::nullopt;
    }

    constexpr std::array<uint8_t, 12> daysPerMonth{31, 28, 31, 30, 31, 30,
                                                   31, 31, 30, 31, 30, 31};
    uint8_t maximumDay = daysPerMonth[month - 1];
    if (month == 2 && isLeapYear(fullYear))
    {
        maximumDay = 29;
    }
    if (day > maximumDay)
    {
        return std::nullopt;
    }

    return std::format("{:04}-{:02}-{:02}T{:02}:{:02}:{:02}+00:00", fullYear,
                       month, day, hour, minute, second);
}

} // namespace

int decodeCperRecord(std::span<const uint8_t> buf, CperRecordInfo& recordInfo)
{
    constexpr size_t headerSize = 128;
    constexpr size_t descriptorSize = 72;
    constexpr size_t signatureEndOffset = 6;
    constexpr size_t sectionCountOffset = 10;
    constexpr size_t severityOffset = 12;
    constexpr size_t validationBitsOffset = 16;
    constexpr size_t recordLengthOffset = 20;
    constexpr size_t timestampOffset = 24;
    constexpr size_t notificationTypeOffset = 80;
    constexpr size_t sectionOffsetOffset = 0;
    constexpr size_t sectionLengthOffset = 4;
    constexpr size_t sectionTypeOffset = 16;
    constexpr uint32_t timestampValid = 1U << 1;

    if (buf.size() < headerSize || buf.size() > maxCperRecordSize ||
        buf[0] != 'C' || buf[1] != 'P' || buf[2] != 'E' || buf[3] != 'R')
    {
        return EINVAL;
    }

    CperRecordInfo decoded{};
    uint32_t signatureEnd = 0;
    uint16_t sectionCount = 0;
    uint32_t validationBits = 0;
    uint32_t recordLength = 0;
    if (readCperValue(buf, signatureEndOffset, signatureEnd) != 0 ||
        readCperValue(buf, sectionCountOffset, sectionCount) != 0 ||
        readCperValue(buf, severityOffset, decoded.severity) != 0 ||
        readCperValue(buf, validationBitsOffset, validationBits) != 0 ||
        readCperValue(buf, recordLengthOffset, recordLength) != 0 ||
        signatureEnd != std::numeric_limits<uint32_t>::max() ||
        sectionCount == 0 || recordLength != buf.size())
    {
        return EINVAL;
    }

    if (sectionCount > (buf.size() - headerSize) / descriptorSize)
    {
        return EINVAL;
    }
    const size_t descriptorTableEnd =
        headerSize + static_cast<size_t>(sectionCount) * descriptorSize;

    decoded.notificationType = formatCperGuid(
        std::span<const uint8_t, 16>(buf.subspan(notificationTypeOffset, 16)));

    if ((validationBits & timestampValid) != 0)
    {
        decoded.timestamp = decodeCperTimestamp(
            std::span<const uint8_t, 8>(buf.subspan(timestampOffset, 8)));
        decoded.timestampInvalid = !decoded.timestamp.has_value();
    }

    decoded.sectionTypes.reserve(sectionCount);
    for (uint16_t index = 0; index < sectionCount; ++index)
    {
        const size_t descriptorOffset =
            headerSize + static_cast<size_t>(index) * descriptorSize;
        uint32_t sectionOffset = 0;
        uint32_t sectionLength = 0;
        if (readCperValue(buf, descriptorOffset + sectionOffsetOffset,
                          sectionOffset) != 0 ||
            readCperValue(buf, descriptorOffset + sectionLengthOffset,
                          sectionLength) != 0 ||
            sectionLength == 0 || sectionOffset < descriptorTableEnd ||
            sectionOffset > buf.size() ||
            sectionLength > buf.size() - sectionOffset)
        {
            return EINVAL;
        }

        decoded.sectionTypes.emplace_back(
            formatCperGuid(std::span<const uint8_t, 16>(
                buf.subspan(descriptorOffset + sectionTypeOffset, 16))));
    }

    recordInfo = std::move(decoded);
    return 0;
}

int encodeGetCurrentClockFrequencyRequest(uint8_t instanceId, ClockType clockId,
                                          std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = sizeof(uint8_t);
    buffer.pack(dataSize);
    buffer.pack(static_cast<uint8_t>(clockId));

    return buffer.getError();
}

int decodeGetCurrentClockFrequencyResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint32_t& clockFreqMHz)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY),
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

    if (dataSize != sizeof(uint32_t))
    {
        return EINVAL;
    }

    rc = buffer.unpack(clockFreqMHz);

    return rc;
}

int encodeGetEccErrorCountsRequest(uint8_t instanceId, std::span<uint8_t> buf)
{
    PackBuffer buffer(buf);

    const int rc = encodeRequestCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS),
        instanceId);

    if (rc != 0)
    {
        return rc;
    }

    const uint8_t dataSize = 0;
    buffer.pack(dataSize);

    return buffer.getError();
}

int decodeGetEccErrorCountsResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint16_t& flags, uint32_t& sramCorrected, uint32_t& sramUncorrectedSecded,
    uint32_t& sramUncorrectedParity, uint32_t& dramCorrected,
    uint32_t& dramUncorrected)
{
    UnpackBuffer buffer(buf);

    int rc = decodeResponseCommonHeader(
        buffer, MessageType::PLATFORM_ENVIRONMENTAL,
        static_cast<uint8_t>(
            PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS),
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

    constexpr uint16_t expectedSize = sizeof(uint16_t) + sizeof(uint32_t) * 5;
    if (dataSize != expectedSize)
    {
        return EINVAL;
    }

    buffer.unpack(flags);
    buffer.unpack(sramCorrected);
    buffer.unpack(sramUncorrectedSecded);
    buffer.unpack(sramUncorrectedParity);
    buffer.unpack(dramCorrected);
    buffer.unpack(dramUncorrected);

    return buffer.getError();
}

} // namespace gpu
