/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <asm/byteorder.h>

#include <OcpMctpVdm.hpp>

#include <cstddef>
#include <cstdint>

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
ocp::accelerator_management::CompletionCode packHeader(
    const ocp::accelerator_management::BindingPciVidInfo& hdr,
    ocp::accelerator_management::BindingPciVid& msg);

/** @brief Encode reason code
 *
 *  @param[in] cc - Completion Code
 *  @param[in] reason_code - reason code
 *  @param[in] command_code - command code
 *  @param[out] msg - Reference to message
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
ocp::accelerator_management::CompletionCode encodeReasonCode(
    uint8_t cc, uint16_t reasonCode, uint8_t commandCode,
    ocp::accelerator_management::Message& msg);

/** @brief Decode to get reason code
 *
 *  @param[in] msg - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - reference to completion code
 *  @param[out] reason_code - reference to reason_code
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
ocp::accelerator_management::CompletionCode decodeReasonCodeAndCC(
    const ocp::accelerator_management::Message& msg, size_t msgLen, uint8_t& cc,
    uint16_t& reasonCode);

/** @brief Create a Query device identification request message
 *
 *  @param[in] instance_id - instance ID
 *  @param[out] msg - Reference to message that will be written to
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
ocp::accelerator_management::CompletionCode
    encodeQueryDeviceIdentificationRequest(
        uint8_t instanceId, ocp::accelerator_management::Message& msg);

/** @brief Encode a Query device identification response message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] cc - completion code
 *  @param[in] reason_code - reason code
 *  @param[in] device_identification - device identification
 *  @param[in] device_instance - device instance id
 *  @param[out] msg - Reference to message that will be written to
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
ocp::accelerator_management::CompletionCode
    encodeQueryDeviceIdentificationResponse(
        uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
        uint8_t deviceIdentification, uint8_t deviceInstance,
        ocp::accelerator_management::Message& msg);

/** @brief Decode a Query device identification response message
 *
 *  @param[in] msg - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - reference to completion code
 *  @param[out] reason_code - reference to reason code
 *  @param[out] device_identification - reference to device_identification
 *  @param[out] device_instance - reference to instance id
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
ocp::accelerator_management::CompletionCode
    decodeQueryDeviceIdentificationResponse(
        const ocp::accelerator_management::Message& msg, size_t msgLen,
        uint8_t& cc, uint16_t& reasonCode, uint8_t& deviceIdentification,
        uint8_t& deviceInstance);

/** @brief Encode a Get temperature readings request message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] sensor_id - sensor id
 *  @param[out] msg - Reference to message that will be written to
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
ocp::accelerator_management::CompletionCode encodeGetTemperatureReadingRequest(
    uint8_t instanceId, uint8_t sensorId,
    ocp::accelerator_management::Message& msg);

/** @brief Decode a Get temperature readings request message
 *
 *  @param[in] msg - request message
 *  @param[in] msg_len - Length of request message
 *  @param[out] sensor_id - reference to sensor id
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
ocp::accelerator_management::CompletionCode decodeGetTemperatureReadingRequest(
    const ocp::accelerator_management::Message& msg, size_t msgLen,
    uint8_t& sensorId);

/** @brief Encode a Get temperature readings response message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] cc - pointer to response message completion code
 *  @param[in] reason_code - reason code
 *  @param[in] temperature_reading - temperature reading
 *  @param[out] msg - Reference to message that will be written to
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
ocp::accelerator_management::CompletionCode encodeGetTemperatureReadingResponse(
    uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
    double temperatureReading, ocp::accelerator_management::Message& msg);

/** @brief Decode a Get temperature readings response message
 *
 *  @param[in] msg - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - reference to response message completion code
 *  @param[out] reason_code - reference to reason code
 *  @param[out] temperature_reading - reference to temperature_reading
 *  @return ocp::accelerator_management::CompletionCode::SUCCESS on success,
 *  otherwise appropriate error code.
 */
ocp::accelerator_management::CompletionCode decodeGetTemperatureReadingResponse(
    const ocp::accelerator_management::Message& msg, size_t msgLen, uint8_t& cc,
    uint16_t& reasonCode, double& temperatureReading);

} // namespace gpu
