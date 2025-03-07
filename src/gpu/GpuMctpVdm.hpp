/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <asm/byteorder.h>

#include <cstddef>
#include <cstdint>

namespace ocp
{
namespace ami
{

/** @brief OCPAMI Message Type
 *
 *  v1 spec section 3.6.1.2.1
 */
constexpr uint8_t messageType = 0x7E;

/**
 * @defgroup OCP Version
 *
 * v1 spec section 3.6.1.2.1
 * @{
 */
constexpr uint8_t type = 8;
constexpr uint8_t version = 9;
/** @} */

/**
 * @defgroup OCPAMI Instance Id
 *
 * v1 spec section 3.6.1.2.1
 * @{
 */
constexpr uint8_t instanceMin = 0;
constexpr uint8_t instanceIdMask = 0x1F;
constexpr uint8_t instanceMax = 31;
/** @} */

/** @brief OCPAMI completion codes
 *
 *  v1 spec section 3.6.2
 */
enum class CompletionCode : uint8_t
{
    SUCCESS = 0x00,
    ERROR = 0x01,
    ERR_INVALID_DATA = 0x02,
    ERR_INVALID_DATA_LENGTH = 0x03,
    ERR_NOT_READY = 0x04,
    ERR_UNSUPPORTED_COMMAND_CODE = 0x05,
    ERR_UNSUPPORTED_MSG_TYPE = 0x06,
    ERR_BUS_ACCESS = 0x7f,
    ERR_NULL = 0x80,
};

/** @brief OCPAMI reason codes
 *
 *  v1 spec section 3.6.3
 */
enum class ReasonCode : uint16_t
{
    REASON_NONE = 0x00,
};

/** @brief OCPAMI MessageType
 *
 *  v1 spec section 3.6.1.2.1
 */
enum class MessageType : uint8_t
{
    RESPONSE = 0,             //!< OCPAMI response message
    EVENT_ACKNOWLEDGMENT = 1, //!< OCPAMI event acknowledgement
    REQUEST = 2,              //!< OCPAMI request message
    EVENT = 3,                //!< OCPAMI event message
};

/** @struct BindingPciVid
 *
 * Structure representing OCPAMI VDM binding using PCI vendor ID
 * v1 spec section 3.6.1.2
 */
struct BindingPciVid
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

/** @struct Message
 *
 * Structure representing OCPAMI message
 * v1 spec section 3.6.1.2
 */
struct Message
{
    BindingPciVid hdr; //!< OCPAMI message header
    char data;         //!< beginning of the payload
} __attribute__((packed));

/** @struct BindingPciVidInfo
 *
 * The information needed to prepare OCPAMI header and this is passed to the
 * PackHeader API.
 * v1 spec section 3.6.1.2
 */
struct BindingPciVidInfo
{
    uint8_t ocp_ami_msg_type;
    uint8_t instance_id;
    uint8_t msg_type;
};

/** @struct CommonRequest
 *
 * Structure representing OCPAMI request without data (OCP version 1).
 * v1 spec section 3.6.1.4.1
 */
struct CommonRequest
{
    uint8_t command;
    uint8_t data_size;
} __attribute__((packed));

/** @struct CommonResponse
 *
 * Structure representing OCPAMI response with data
 * v1 spec section 3.6.1.4.4
 */
struct CommonResponse
{
    uint8_t command;
    uint8_t completion_code;
    uint16_t reserved;
    uint16_t data_size;
} __attribute__((packed));

/** @struct CommonNonSuccessResponse
 *
 * Structure representing OCPAMI response with reason code when CC != Success
 * v1 spec section 3.6.1.4.5
 */
struct CommonNonSuccessResponse
{
    uint8_t command;
    uint8_t completion_code;
    uint16_t reason_code;
} __attribute__((packed));

/**
 * @brief Populate the OCPAMI message with the OCPAMI header. OCPAMI header OCP
 * Version will be populated with value 1. The caller of this API allocates
 * buffer for the OCPAMI header when forming the OCPAMI message. The buffer is
 * passed to this API to pack the OCPAMI header.
 *
 * @param[in] pci_vendor_id - PCI Vendor ID
 * @param[in] hdr - Pointer to the OCPAMI header information
 * @param[out] msg - Reference to OCPAMI message header
 *
 * @return CompletionCode::SUCCESS on success, otherwise appropriate error
 * code.
 * @note   Caller is responsible for alloc and dealloc of msg
 *         and hdr params
 */
CompletionCode packHeader(uint16_t pciVendorId, const BindingPciVidInfo& hdr,
                          BindingPciVid& msg);

/** @brief Encode reason code into an OCPAMI response message.
 *         This function does not populate or modifies the message header.
 *
 *  @param[in] cc - Completion Code
 *  @param[in] reason_code - reason code
 *  @param[in] command_code - command code
 *  @param[out] msg - Reference to message
 *  @return CompletionCode
 */
CompletionCode encodeReasonCode(uint8_t cc, uint16_t reasonCode,
                                uint8_t commandCode, Message& msg);

/** @brief Decode the reason code
 *
 *  @param[in] msg - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - reference to completion code
 *  @param[out] reason_code - reference to reason_code
 *  @return CompletionCode
 */
CompletionCode decodeReasonCodeAndCC(const Message& msg, size_t msgLen,
                                     uint8_t& cc, uint16_t& reasonCode);

} // namespace ami
} // namespace ocp

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
    DEVICE_GPU = 0,
    DEVICE_SWITCH = 1,
    DEVICE_PCIE_BRIDGE = 2,
    DEVICE_BASEBOARD = 3,
    DEVICE_EROT = 4,
    DEVICE_UNKNOWN = 0xff,
};

/** @struct QueryDeviceIdentificationRequest
 *
 *  Structure representing query device identification request
 */
struct QueryDeviceIdentificationRequest
{
    ocp::ami::CommonRequest hdr;
} __attribute__((packed));

/** @struct QueryDeviceIdentificationResponse
 *
 *  Structure representing query device identification response.
 */
struct QueryDeviceIdentificationResponse
{
    ocp::ami::CommonResponse hdr;
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
    ocp::ami::CommonRequest hdr;
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
    ocp::ami::CommonResponse hdr;
    int32_t reading;
} __attribute__((packed));

/**
 * @brief Populate the GPU message with the GPU header.
 *        The caller of this API allocates buffer for the GPU header
 *        when forming the GPU message.
 *        The buffer is passed to this API to pack the GPU header.
 *
 * @param[in] hdr - Reference to the OCPAMI header information
 * @param[out] msg - Reference to GPU message header
 *
 * @return ocp::ami::CompletionCode::SUCCESS on success, otherwise appropriate
 * error code.
 * @note   Caller is responsible for alloc and dealloc of msg
 *         and hdr params
 */
ocp::ami::CompletionCode packHeader(const ocp::ami::BindingPciVidInfo& hdr,
                                    ocp::ami::BindingPciVid& msg);

/** @brief Encode reason code
 *
 *  @param[in] cc - Completion Code
 *  @param[in] reason_code - reason code
 *  @param[in] command_code - command code
 *  @param[out] msg - Reference to message
 *  @return ocp::ami::CompletionCode
 */
ocp::ami::CompletionCode encodeReasonCode(uint8_t cc, uint16_t reasonCode,
                                          uint8_t commandCode,
                                          ocp::ami::Message& msg);

/** @brief Decode to get reason code
 *
 *  @param[in] msg - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - reference to completion code
 *  @param[out] reason_code - reference to reason_code
 *  @return ocp::ami::CompletionCode
 */
ocp::ami::CompletionCode decodeReasonCodeAndCC(const ocp::ami::Message& msg,
                                               size_t msgLen, uint8_t& cc,
                                               uint16_t& reasonCode);

/** @brief Create a Query device identification request message
 *
 *  @param[in] instance_id - instance ID
 *  @param[out] msg - Reference to message that will be written to
 *  @return ocp::ami::CompletionCode
 */
ocp::ami::CompletionCode encodeQueryDeviceIdentificationRequest(
    uint8_t instanceId, ocp::ami::Message& msg);

/** @brief Encode a Query device identification response message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] cc - completion code
 *  @param[in] reason_code - reason code
 *  @param[in] device_identification - device identification
 *  @param[in] device_instance - device instance id
 *  @param[out] msg - Reference to message that will be written to
 *  @return ocp::ami::CompletionCode
 */
ocp::ami::CompletionCode encodeQueryDeviceIdentificationResponse(
    uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
    uint8_t deviceIdentification, uint8_t deviceInstance,
    ocp::ami::Message& msg);

/** @brief Decode a Query device identification response message
 *
 *  @param[in] msg - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - reference to completion code
 *  @param[out] reason_code - reference to reason code
 *  @param[out] device_identification - reference to device_identification
 *  @param[out] device_instance - reference to instance id
 *  @return ocp::ami::CompletionCode
 */
ocp::ami::CompletionCode decodeQueryDeviceIdentificationResponse(
    const ocp::ami::Message& msg, size_t msgLen, uint8_t& cc,
    uint16_t& reasonCode, uint8_t& deviceIdentification,
    uint8_t& deviceInstance);

/** @brief Encode a Get temperature readings request message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] sensor_id - sensor id
 *  @param[out] msg - Reference to message that will be written to
 *  @return ocp::ami::CompletionCode
 */
ocp::ami::CompletionCode encodeGetTemperatureReadingRequest(
    uint8_t instanceId, uint8_t sensorId, ocp::ami::Message& msg);

/** @brief Decode a Get temperature readings request message
 *
 *  @param[in] msg - request message
 *  @param[in] msg_len - Length of request message
 *  @param[out] sensor_id - reference to sensor id
 *  @return ocp::ami::CompletionCode
 */
ocp::ami::CompletionCode decodeGetTemperatureReadingRequest(
    const ocp::ami::Message& msg, size_t msgLen, uint8_t& sensorId);

/** @brief Encode a Get temperature readings response message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] cc - pointer to response message completion code
 *  @param[in] reason_code - reason code
 *  @param[in] temperature_reading - temperature reading
 *  @param[out] msg - Reference to message that will be written to
 *  @return ocp::ami::CompletionCode
 */
ocp::ami::CompletionCode encodeGetTemperatureReadingResponse(
    uint8_t instanceId, uint8_t cc, uint16_t reasonCode,
    double temperatureReading, ocp::ami::Message& msg);

/** @brief Decode a Get temperature readings response message
 *
 *  @param[in] msg - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - reference to response message completion code
 *  @param[out] reason_code - reference to reason code
 *  @param[out] temperature_reading - reference to temperature_reading
 *  @return ocp::ami::CompletionCode
 */
ocp::ami::CompletionCode decodeGetTemperatureReadingResponse(
    const ocp::ami::Message& msg, size_t msgLen, uint8_t& cc,
    uint16_t& reasonCode, double& temperatureReading);

} // namespace gpu
