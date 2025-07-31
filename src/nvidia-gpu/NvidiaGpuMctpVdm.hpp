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
    GET_CURRENT_POWER_DRAW = 0x03,
    GET_MAX_OBSERVED_POWER = 0x04,
    GET_CURRENT_ENERGY_COUNTER = 0x06,
    GET_VOLTAGE = 0x0F,
};

enum class DeviceIdentification : uint8_t
{
    DEVICE_GPU = 0,
    DEVICE_SMA = 5
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

struct GetCurrentPowerDrawRequest
{
    ocp::accelerator_management::CommonRequest hdr;
    uint8_t sensorId;
    uint8_t averagingInterval;
} __attribute__((packed));

using GetCurrentEnergyCounterRequest = GetNumericSensorReadingRequest;

using GetVoltageRequest = GetNumericSensorReadingRequest;

struct GetMaxObservedPowerRequest
{
    ocp::accelerator_management::CommonRequest hdr;
    uint8_t sensorId;
    uint8_t averagingInterval;
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

struct GetCurrentPowerDrawResponse
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

struct GetMaxObservedPowerResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    uint32_t peakPower;
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

int encodeGetCurrentPowerDrawRequest(uint8_t instanceId, uint8_t sensorId,
                                     uint8_t averagingInterval,
                                     std::span<uint8_t> buf);

int decodeGetCurrentPowerDrawResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint32_t& power);

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

int encodeGetMaxObservedPowerRequest(uint8_t instanceId, uint8_t sensorId,
                                     uint8_t averagingInterval,
                                     std::span<uint8_t> buf);

int decodeGetMaxObservedPowerResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint32_t& peakPower);
} // namespace gpu
