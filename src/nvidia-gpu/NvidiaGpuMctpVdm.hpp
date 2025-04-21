/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <OcpMctpVdm.hpp>

#include <cstdint>
#include <span>

namespace gpu
{

/** @brief NVIDIA PCI vendor ID */
constexpr uint16_t nvidiaPciVendorId = 0x10de;

/** @brief GPU message types
 *
 *  Enumeration of different message types used in GPU protocol.
 *  These types categorize different classes of messages for device management
 *  and monitoring.
 */
enum class MessageType : uint8_t
{
    DEVICE_CAPABILITY_DISCOVERY = 0,
    PLATFORM_ENVIRONMENTAL = 3
};

/** @brief Type0 Device Capability Discovery Commands
 */
enum class DeviceCapabilityDiscoveryCommands : uint8_t
{
    QUERY_DEVICE_IDENTIFICATION = 0x09,
};

/** @brief Type3 platform environmental commands
 */
enum class PlatformEnvironmentalCommands : uint8_t
{
    GET_TEMPERATURE_READING = 0x00,
};

/** @brief device identification types
 *
 *  Enumeration of different device types that can be identified in the system.
 *  This is used to distinguish between various components during device
 * discovery.
 */
enum class DeviceIdentification : uint8_t
{
    DEVICE_GPU = 0
};

/** @struct QueryDeviceIdentificationRequest
 *
 *  Structure representing query device identification request
 */
struct QueryDeviceIdentificationRequest
{
    ocp::accelerator_management::CommonRequest hdr;
} __attribute__((packed));

/** @struct QueryDeviceIdentificationResponse
 *
 *  Structure representing query device identification response.
 */
struct QueryDeviceIdentificationResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    uint8_t device_identification;
    uint8_t instance_id;
} __attribute__((packed));

/** @struct GetNumericSensorReadingRequest
 *
 *  Structure representing request to get reading of certain numeric
 * sensors.
 */
struct GetNumericSensorReadingRequest
{
    ocp::accelerator_management::CommonRequest hdr;
    uint8_t sensor_id;
} __attribute__((packed));

/** @struct GetTemperatureReadingRequest
 *
 *  Structure representing get temperature reading request.
 */
using GetTemperatureReadingRequest = GetNumericSensorReadingRequest;

/** @struct GetTemperatureReadingResponse
 *
 *  Structure representing get temperature reading response.
 */
struct GetTemperatureReadingResponse
{
    ocp::accelerator_management::CommonResponse hdr;
    int32_t reading;
} __attribute__((packed));

/**
 * @brief Populate the GPU message with the GPU header.
 *        The caller of this API allocates buffer for the GPU header
 *        when forming the GPU message.
 *        The buffer is passed to this API to pack the GPU header.
 *
 * @param[in] hdr - Reference to the OCP MCTP VDM header information
 * @param[out] msg - Reference to GPU message header
 *
 * @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 * otherwise appropriate error code.
 * @note   Caller is responsible for alloc and dealloc of msg
 *         and hdr params
 */
int packHeader(const ocp::accelerator_management::BindingPciVidInfo& hdr,
               ocp::accelerator_management::BindingPciVid& msg);

/** @brief Create a Query device identification request message
 *
 *  @param[in] instanceId - Instance ID
 *  @param[out] buf - Reference to buffer that will contain the request message
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
int encodeQueryDeviceIdentificationRequest(uint8_t instanceId,
                                           std::span<uint8_t> buf);

/** @brief Decode a Query device identification response message
 *
 *  @param[in] buf - Response message buffer
 *  @param[out] cc - Reference to completion code to be populated
 *  @param[out] reasonCode - Reference to reason code to be populated
 *  @param[out] deviceIdentification - Reference to device identification to be
 * populated
 *  @param[out] deviceInstance - Reference to instance ID to be populated
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
int decodeQueryDeviceIdentificationResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    uint8_t& deviceIdentification, uint8_t& deviceInstance);

/** @brief Encode a Get temperature readings request message
 *
 *  @param[in] instanceId - Instance ID
 *  @param[in] sensorId - Sensor ID
 *  @param[out] buf - Reference to buffer that will contain the request message
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
int encodeGetTemperatureReadingRequest(uint8_t instanceId, uint8_t sensorId,
                                       std::span<uint8_t> buf);

/** @brief Decode a Get temperature readings response message
 *
 *  @param[in] buf - Response message buffer
 *  @param[out] cc - Reference to completion code to be populated
 *  @param[out] reasonCode - Reference to reason code to be populated
 *  @param[out] temperatureReading - Reference to temperature reading to be
 * populated
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
int decodeGetTemperatureReadingResponse(
    std::span<const uint8_t> buf,
    ocp::accelerator_management::CompletionCode& cc, uint16_t& reasonCode,
    double& temperatureReading);

} // namespace gpu
