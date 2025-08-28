/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <OcpMctpVdm.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace gpu
{

using InventoryValue = std::variant<std::string, std::vector<uint8_t>>;
constexpr size_t maxInventoryDataSize = 256;

constexpr uint16_t nvidiaPciVendorId = 0x10de;

enum class MessageType : uint8_t
{
    DEVICE_CAPABILITY_DISCOVERY = 0,
    PCIE_LINK = 2,
    PLATFORM_ENVIRONMENTAL = 3
};

enum class DeviceCapabilityDiscoveryCommands : uint8_t
{
    QUERY_DEVICE_IDENTIFICATION = 0x09,
};

enum class PlatformEnvironmentalCommands : uint8_t
{
    GET_TEMPERATURE_READING = 0x00,
    READ_THERMAL_PARAMETERS = 0x02,
    GET_CURRENT_POWER_DRAW = 0x03,
    GET_MAX_OBSERVED_POWER = 0x04,
    GET_CURRENT_ENERGY_COUNTER = 0x06,
    GET_INVENTORY_INFORMATION = 0x0C,
    GET_VOLTAGE = 0x0F,
};

enum class PcieLinkCommands : uint8_t
{
    QueryScalarGroupTelemetryV2 = 0x24,
};

enum class DeviceIdentification : uint8_t
{
    DEVICE_GPU = 0,
    DEVICE_PCIE = 2,
    DEVICE_SMA = 5
};

enum class InventoryPropertyId : uint8_t
{
    BOARD_PART_NUMBER = 0,
    SERIAL_NUMBER = 1,
    MARKETING_NAME = 2,
    DEVICE_PART_NUMBER = 3,
    FRU_PART_NUMBER = 4,
    MEMORY_VENDOR = 5,
    MEMORY_PART_NUMBER = 6,
    MAX_MEMORY_CAPACITY = 7,
    BUILD_DATE = 8,
    FIRMWARE_VERSION = 9,
    DEVICE_GUID = 10,
    INFOROM_VERSION = 11,
    PRODUCT_LENGTH = 12,
    PRODUCT_WIDTH = 13,
    PRODUCT_HEIGHT = 14,
    RATED_DEVICE_POWER_LIMIT = 15,
    MIN_DEVICE_POWER_LIMIT = 16,
    MAX_DEVICE_POWER_LIMIT = 17,
    MAX_MODULE_POWER_LIMIT = 18,
    MIN_MODULE_POWER_LIMIT = 19,
    RATED_MODULE_POWER_LIMIT = 20,
    DEFAULT_BOOST_CLOCKS = 21,
    DEFAULT_BASE_CLOCKS = 22,
    DEFAULT_EDPP_SCALING = 23,
    MIN_EDPP_SCALING = 24,
    MAX_EDPP_SCALING = 25,
    MIN_GRAPHICS_CLOCK = 26,
    MAX_GRAPHICS_CLOCK = 27,
    MIN_MEMORY_CLOCK = 28,
    MAX_MEMORY_CLOCK = 29,
    INFINIBAND_GUID = 30,
    RACK_GUID = 31,
    RACK_SLOT_NUMBER = 32,
    COMPUTE_SLOT_INDEX = 33,
    NODE_INDEX = 34,
    GPU_NODE_ID = 35,
    NVLINK_PEER_TYPE = 36
};

enum class PciePortType : uint8_t
{
    UPSTREAM = 0,
    DOWNSTREAM = 1,
};

struct QueryDeviceIdentificationRequest
{
    ocp::accelerator_management::CommonRequest hdr;
} __attribute__((packed));

struct QueryDeviceIdentificationResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    uint8_t device_identification;
    uint8_t instance_id;
} __attribute__((packed));

struct GetNumericSensorReadingRequest
{
    ocp::accelerator_management::CommonRequest hdr;
    uint8_t sensor_id;
} __attribute__((packed));

using GetTemperatureReadingRequest = GetNumericSensorReadingRequest;

using ReadThermalParametersRequest = GetNumericSensorReadingRequest;

struct GetPowerDrawRequest
{
    ocp::accelerator_management::CommonRequest hdr;
    uint8_t sensorId;
    uint8_t averagingInterval;
} __attribute__((packed));

using GetCurrentEnergyCounterRequest = GetNumericSensorReadingRequest;

using GetVoltageRequest = GetNumericSensorReadingRequest;

struct QueryScalarGroupTelemetryV2Request
{
    ocp::accelerator_management::CommonRequest hdr;
    uint8_t upstreamPortNumber;
    uint8_t portNumber;
    uint8_t groupId;
} __attribute__((packed));

struct GetTemperatureReadingResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    int32_t reading;
} __attribute__((packed));

struct ReadThermalParametersResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    int32_t threshold;
} __attribute__((packed));

struct GetPowerDrawResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    uint32_t power;
} __attribute__((packed));

struct GetCurrentEnergyCounterResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    uint64_t energy;
} __attribute__((packed));

struct GetVoltageResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    uint32_t voltage;
} __attribute__((packed));

struct GetInventoryInformationRequest
{
    ocp::accelerator_management::CommonRequest hdr;
    uint8_t property_id;
} __attribute__((packed));

struct GetInventoryInformationResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    std::array<uint8_t, maxInventoryDataSize> data;
} __attribute__((packed));

int packHeader(const ocp::accelerator_management::BindingPciVidInfo& hdr,
               ocp::accelerator_management::BindingPciVid& msg);

int encodeQueryDeviceIdentificationRequest(uint8_t instanceId,
                                           std::span<uint8_t> buf);

int decodeQueryDeviceIdentificationResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint8_t& deviceIdentification, uint8_t& deviceInstance);

int encodeGetTemperatureReadingRequest(uint8_t instanceId, uint8_t sensorId,
                                       std::span<uint8_t> buf);

int decodeGetTemperatureReadingResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    double& temperatureReading);

int encodeReadThermalParametersRequest(uint8_t instanceId, uint8_t sensorId,
                                       std::span<uint8_t> buf);

int decodeReadThermalParametersResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    int32_t& threshold);

int encodeGetPowerDrawRequest(
    PlatformEnvironmentalCommands commandCode, uint8_t instanceId,
    uint8_t sensorId, uint8_t averagingInterval, std::span<uint8_t> buf);

int decodeGetPowerDrawResponse(std::span<const uint8_t> buf,
                               ocp::accelerator_management::CompletionCode& cc,
                               uint16_t& reasonCode, uint32_t& power);

int encodeGetCurrentEnergyCounterRequest(uint8_t instanceId, uint8_t sensorId,
                                         std::span<uint8_t> buf);

int decodeGetCurrentEnergyCounterResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint64_t& energy);

int encodeGetVoltageRequest(uint8_t instanceId, uint8_t sensorId,
                            std::span<uint8_t> buf);

int decodeGetVoltageResponse(std::span<const uint8_t> buf,
                             ocp::accelerator_management::CompletionCode& cc,
                             uint16_t& reasonCode, uint32_t& voltage);

int encodeGetInventoryInformationRequest(uint8_t instanceId, uint8_t propertyId,
                                         std::span<uint8_t> buf);

int decodeGetInventoryInformationResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    InventoryPropertyId propertyId, InventoryValue& value);

int encodeQueryScalarGroupTelemetryV2Request(
    uint8_t instanceId, PciePortType portType, uint8_t upstreamPortNumber,
    uint8_t portNumber, uint8_t groupId, std::span<uint8_t> buf);

int decodeQueryScalarGroupTelemetryV2Response(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    size_t& numTelemetryValues, std::vector<uint32_t>& telemetryValues);

} // namespace gpu
