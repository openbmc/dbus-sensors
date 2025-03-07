/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#ifndef GPU_SENSORS_HPP
#define GPU_SENSORS_HPP

#include <asm/byteorder.h>

#include <cstddef>
#include <cstdint>

/** @brief OCPAMI Message Type
 *
 *  v1 spec section 3.6.1.2.1
 */
constexpr uint8_t ocpAmiMessageType = 0x7E;

/**
 * @defgroup OCP Version
 *
 * v1 spec section 3.6.1.2.1
 * @{
 */
constexpr uint8_t ocpAmiType = 8;
constexpr uint8_t ocpAmiVersion = 9;
/** @} */

/**
 * @defgroup OCPAMI Instance Id
 *
 * v1 spec section 3.6.1.2.1
 * @{
 */
constexpr uint8_t ocpAmiInstanceMin = 0;
constexpr uint8_t ocpAmiInstanceIdMask = 0x1F;
constexpr uint8_t ocpAmiInstanceMax = 31;
/** @} */

/** @brief NVIDIA PCI vendor ID */
constexpr uint16_t gpuNvidiaPciVendorId = 0x10de;

/** @brief OCPAMI completion codes
 *
 *  v1 spec section 3.6.2
 */
enum OcpAmiCompletionCode
{
    OCP_AMI_SUCCESS = 0x00,
    OCP_AMI_ERROR = 0x01,
    OCP_AMI_ERR_INVALID_DATA = 0x02,
    OCP_AMI_ERR_INVALID_DATA_LENGTH = 0x03,
    OCP_AMI_ERR_NOT_READY = 0x04,
    OCP_AMI_ERR_UNSUPPORTED_COMMAND_CODE = 0x05,
    OCP_AMI_ERR_UNSUPPORTED_MSG_TYPE = 0x06,
    OCP_AMI_ERR_BUS_ACCESS = 0x7f,
    OCP_AMI_ERR_NULL = 0x80,
};

/** @brief OCPAMI reason codes
 *
 *  v1 spec section 3.6.3
 */
enum OcpAmiReasonCode
{
    OCP_AMI_REASON_NONE = 0x00,
};

/** @brief OCPAMI MessageType
 *
 *  v1 spec section 3.6.1.2.1
 */
enum OcpAmiMessageType
{
    OCP_AMI_RESPONSE = 0,             //!< OCPAMI response message
    OCP_AMI_EVENT_ACKNOWLEDGMENT = 1, //!< OCPAMI event acknowledgement
    OCP_AMI_REQUEST = 2,              //!< OCPAMI request message
    OCP_AMI_EVENT = 3,                //!< OCPAMI event message
};

/** @brief GPU message types
 *
 *  Enumeration of different message types used in GPU protocol.
 *  These types categorize different classes of messages for device management
 *  and monitoring.
 */
enum GpuMessageType
{
    GPU_DEVICE_CAPABILITY_DISCOVERY = 0,
    GPU_PLATFORM_ENVIRONMENTAL = 3
};

/** @brief Type0 Device Capability Discovery Commands
 */
enum GpuDeviceCapabilityDiscoveryCommands
{
    GPU_QUERY_DEVICE_IDENTIFICATION = 0x09,
};

/** @brief Type3 platform environmental commands
 */
enum GpuPlatformEnvironmentalCommands
{
    GPU_GET_TEMPERATURE_READING = 0x00,
};

/** @brief device identification types
 *
 *  Enumeration of different device types that can be identified in the system.
 *  This is used to distinguish between various components during device
 * discovery.
 */
enum GpuDeviceIdentification
{
    GPU_DEVICE_GPU = 0,
    GPU_DEVICE_SWITCH = 1,
    GPU_DEVICE_PCIE_BRIDGE = 2,
    GPU_DEVICE_BASEBOARD = 3,
    GPU_DEVICE_EROT = 4,
    GPU_DEVICE_UNKNOWN = 0xff,
};

/** @struct OcpAmiBindingPciVid
 *
 * Structure representing OCPAMI VDM binding using PCI vendor ID
 * v1 spec section 3.6.1.2
 */
struct OcpAmiBindingPciVid
{
    uint16_t pci_vendor_id; //!< PCI defined vendor ID

#if defined(__LITTLE_ENDIAN_BITFIELD)
    uint8_t instance_id:5; //!< Instance ID
    uint8_t reserved:1;    //!< Reserved
    uint8_t datagram:1;    //!< Datagram bit
    uint8_t request:1;     //!< Request bit
#elif defined(__BIG_ENDIAN_BITFIELD)
    uint8_t request:1;     //!< Request bit
    uint8_t datagram:1;    //!< Datagram bit
    uint8_t reserved:1;    //!< Reserved
    uint8_t instance_id:5; //!< Instance ID
#endif

#if defined(__LITTLE_ENDIAN_BITFIELD)
    uint8_t ocp_version:4; //!< OCP version
    uint8_t ocp_type:4;    //!< OCP type
#elif defined(__BIG_ENDIAN_BITFIELD)
    uint8_t ocp_type:4;    //!< OCP type
    uint8_t ocp_version:4; //!< OCP version
#endif

    uint8_t ocp_ami_msg_type; //!< Message Type
} __attribute__((packed));

/** @struct OcpAmiMessage
 *
 * Structure representing OCPAMI message
 * v1 spec section 3.6.1.2
 */
struct OcpAmiMessage
{
    OcpAmiBindingPciVid hdr; //!< OCPAMI message header
    char data;               //!< beginning of the payload
} __attribute__((packed));

/** @struct OcpAmiBindingPciVidInfo
 *
 * The information needed to prepare OCPAMI header and this is passed to the
 * OcpAmiPackHeader API.
 * v1 spec section 3.6.1.2
 */
struct OcpAmiBindingPciVidInfo
{
    uint8_t ocp_ami_msg_type;
    uint8_t instance_id;
    uint8_t msg_type;
};

/** @struct OcpAmiCommonRequest
 *
 * Structure representing OCPAMI request without data (OCP version 1).
 * v1 spec section 3.6.1.4.1
 */
struct OcpAmiCommonRequest
{
    uint8_t command;
    uint8_t data_size;
} __attribute__((packed));

/** @struct OcpAmiCommonResponse
 *
 * Structure representing OCPAMI response with data
 * v1 spec section 3.6.1.4.4
 */
struct OcpAmiCommonResponse
{
    uint8_t command;
    uint8_t completion_code;
    uint16_t reserved;
    uint16_t data_size;
} __attribute__((packed));

/** @struct OcpAmiCommonNonSuccessResponse
 *
 * Structure representing OCPAMI response with reason code when CC != Success
 * v1 spec section 3.6.1.4.5
 */
struct OcpAmiCommonNonSuccessResponse
{
    uint8_t command;
    uint8_t completion_code;
    uint16_t reason_code;
} __attribute__((packed));

/** @struct GpuQueryDeviceIdentificationRequest
 *
 *  Structure representing query device identification request
 */
struct GpuQueryDeviceIdentificationRequest
{
    OcpAmiCommonRequest hdr;
} __attribute__((packed));

/** @struct GpuQueryDeviceIdentificationResponse
 *
 *  Structure representing query device identification response.
 */
struct GpuQueryDeviceIdentificationResponse
{
    OcpAmiCommonResponse hdr;
    uint8_t device_identification;
    uint8_t instance_id;
} __attribute__((packed));

/** @struct GpuGetNumericSensorReadingRequest
 *
 *  Structure representing request to get reading of certain numeric
 * sensors.
 */
struct GpuGetNumericSensorReadingRequest
{
    OcpAmiCommonRequest hdr;
    uint8_t sensor_id;
} __attribute__((packed));

/** @struct GpuGetTemperatureReadingRequest
 *
 *  Structure representing get temperature reading request.
 */
using GpuGetTemperatureReadingRequest = GpuGetNumericSensorReadingRequest;

/** @struct GpuGetTemperatureReadingResponse
 *
 *  Structure representing get temperature reading response.
 */
struct GpuGetTemperatureReadingResponse
{
    OcpAmiCommonResponse hdr;
    int32_t reading;
} __attribute__((packed));

/**
 * @brief Populate the OCPAMI message with the OCPAMI header. OCPAMI header OCP
 * Version will be populated with value 1. The caller of this API allocates
 * buffer for the OCPAMI header when forming the OCPAMI message. The buffer is
 * passed to this API to pack the OCPAMI header.
 *
 * @param[in] pci_vendor_id - PCI Vendor ID
 * @param[in] hdr - Pointer to the OCPAMI header information
 * @param[out] msg - Pointer to OCPAMI message header
 *
 * @return OCP_AMI_SUCCESS on success, otherwise appropriate error
 * code.
 * @note   Caller is responsible for alloc and dealloc of msg
 *         and hdr params
 */
OcpAmiCompletionCode ocpAmiPackHeader(uint16_t pciVendorId,
                                      const OcpAmiBindingPciVidInfo& hdr,
                                      OcpAmiBindingPciVid* msg);

/** @brief Encode reason code into an OCPAMI response message.
 *         This function does not populate or modifies the message header.
 *
 *  @param[in] cc - Completion Code
 *  @param[in] reason_code - reason code
 *  @param[in] command_code - command code
 *  @param[out] msg - msg
 *  @return OcpAmiCompletionCode
 */
OcpAmiCompletionCode ocpAmiEncodeReasonCode(
    uint8_t cc, uint16_t reasonCode, uint8_t commandCode, OcpAmiMessage* msg);

/** @brief Decode the reason code
 *
 *  @param[in] msg - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - pointer to completion code
 *  @param[out] reason_code - pointer to reason_code
 *  @return OcpAmiCompletionCode
 */
OcpAmiCompletionCode ocpAmiDecodeReasonCodeAndCC(
    const OcpAmiMessage* msg, size_t msgLen, uint8_t* cc, uint16_t* reasonCode);

/**
 * @brief Populate the GPU message with the GPU header.
 *        The caller of this API allocates buffer for the GPU header
 *        when forming the GPU message.
 *        The buffer is passed to this API to pack the GPU header.
 *
 * @param[in] hdr - Reference to the OCPAMI header information
 * @param[out] msg - Pointer to GPU message header
 *
 * @return OCP_AMI_SUCCESS on success, otherwise appropriate error
 * code.
 * @note   Caller is responsible for alloc and dealloc of msg
 *         and hdr params
 */
OcpAmiCompletionCode gpuPackHeader(const OcpAmiBindingPciVidInfo& hdr,
                                   OcpAmiBindingPciVid* msg);

/** @brief Encode reason code
 *
 *  @param[in] cc - Completion Code
 *  @param[in] reason_code - reason code
 *  @param[in] command_code - command code
 *  @param[out] msg - msg pointer
 *  @return OcpAmiCompletionCode
 */
OcpAmiCompletionCode gpuEncodeReasonCode(
    uint8_t cc, uint16_t reasonCode, uint8_t commandCode, OcpAmiMessage* msg);

/** @brief Decode to get reason code
 *
 *  @param[in] msg - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - pointer to completion code
 *  @param[out] reason_code - pointer to reason_code
 *  @return OcpAmiCompletionCode
 */
OcpAmiCompletionCode gpuDecodeReasonCodeAndCC(
    const OcpAmiMessage* msg, size_t msgLen, uint8_t* cc, uint16_t* reasonCode);

/** @brief Create a Query device identification request message
 *
 *  @param[in] instance_id - instance ID
 *  @param[out] msg - Message will be written to this
 *  @return OcpAmiCompletionCode
 */
OcpAmiCompletionCode gpuEncodeQueryDeviceIdentificationRequest(
    uint8_t instanceId, OcpAmiMessage* msg);

/** @brief Encode a Query device identification response message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] cc - completion code
 *  @param[in] reason_code - reason code
 *  @param[in] device_identification - device identification
 *  @param[in] device_instance - device instance id
 *  @param[out] msg - Message will be written to this
 *  @return OcpAmiCompletionCode
 */
OcpAmiCompletionCode gpuEncodeQueryDeviceIdentificationResponse(
    uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
    uint8_t deviceIdentification, uint8_t deviceInstance, OcpAmiMessage* msg);

/** @brief Decode a Query device identification response message
 *
 *  @param[in] msg - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - pointer to completion code
 *  @param[out] reason_code - pointer to reason code
 *  @param[out] device_identification - pointer to device_identification
 *  @param[out] device_instance - pointer to instance id
 *  @return OcpAmiCompletionCode
 */
OcpAmiCompletionCode gpuDecodeQueryDeviceIdentificationResponse(
    const OcpAmiMessage* msg, size_t msgLen, uint8_t* cc, uint16_t* reasonCode,
    uint8_t* deviceIdentification, uint8_t* deviceInstance);

/** @brief Encode a Get temperature readings request message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] sensor_id - sensor id
 *  @param[out] msg - Message will be written to this
 *  @return OcpAmiCompletionCode
 */
OcpAmiCompletionCode gpuEncodeGetTemperatureReadingRequest(
    uint8_t instanceId, uint8_t sensorId, OcpAmiMessage* msg);

/** @brief Decode a Get temperature readings request message
 *
 *  @param[in] msg - request message
 *  @param[in] msg_len - Length of request message
 *  @param[out] sensor_id - sensor id
 *  @return OcpAmiCompletionCode
 */
OcpAmiCompletionCode gpuDecodeGetTemperatureReadingRequest(
    const OcpAmiMessage* msg, size_t msgLen, uint8_t* sensorId);

/** @brief Encode a Get temperature readings response message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] cc - pointer to response message completion code
 *  @param[in] reason_code - reason code
 *  @param[in] temperature_reading - temperature reading
 *  @param[out] msg - Message will be written to this
 *  @return OcpAmiCompletionCode
 */
OcpAmiCompletionCode gpuEncodeGetTemperatureReadingResponse(
    uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
    double temperatureReading, OcpAmiMessage* msg);

/** @brief Decode a Get temperature readings response message
 *
 *  @param[in] msg - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - pointer to response message completion code
 *  @param[out] reason_code - pointer to reason code
 *  @param[out] temperature_reading - temperature_reading
 *  @return OcpAmiCompletionCode
 */
OcpAmiCompletionCode gpuDecodeGetTemperatureReadingResponse(
    const OcpAmiMessage* msg, size_t msgLen, uint8_t* cc, uint16_t* reasonCode,
    double* temperatureReading);

#endif /* GPU_SENSORS_HPP */
