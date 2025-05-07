/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <OcpMctpVdm.hpp>

#include <cstdint>
#include <span>

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
    READ_THERMAL_PARAMETERS = 0x02,
};

enum class DeviceIdentification : uint8_t
{
    DEVICE_GPU = 0
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

} // namespace gpu
