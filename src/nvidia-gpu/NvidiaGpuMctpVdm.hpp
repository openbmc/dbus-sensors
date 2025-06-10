/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <OcpMctpVdm.hpp>

#include <cstdint>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace gpu
{

constexpr uint16_t nvidiaPciVendorId = 0x10de;

enum class MessageType : uint8_t
{
    DEVICE_CAPABILITY_DISCOVERY = 0,
    PLATFORM_ENVIRONMENTAL = 3
};

enum class DeviceCapabilityDiscoveryCommands : uint8_t
{
    QUERY_DEVICE_IDENTIFICATION = 0x09,
};

enum class PlatformEnvironmentalCommands : uint8_t
{
    GET_TEMPERATURE_READING = 0x00,
    GET_INVENTORY_INFORMATION = 0x0C,
};

enum class DeviceIdentification : uint8_t
{
    DEVICE_GPU = 0
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
    NVLINK_PEER_TYPE = 36,
    FPGA_IMAGE_VERSION = 128,
    FPGA_MCTP_BRIDGE_UUID = 129,
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

struct GetTemperatureReadingResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    int32_t reading;
} __attribute__((packed));

struct GetInventoryInformationRequest
{
    ocp::accelerator_management::CommonRequest hdr;
    uint8_t property_id;
} __attribute__((packed));

constexpr size_t MAX_INVENTORY_DATA_SIZE = 256;

struct GetInventoryInformationResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    uint8_t data[MAX_INVENTORY_DATA_SIZE];
} __attribute__((packed));

using InventoryInfo = std::variant<std::string, std::vector<uint8_t>>;

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

int encodeGetInventoryInformationRequest(uint8_t instanceId, uint8_t propertyId,
                                         std::span<uint8_t> buf);

int decodeGetInventoryInformationResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    InventoryPropertyId propertyId, InventoryInfo& info);

} // namespace gpu
