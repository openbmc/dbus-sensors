/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <asm/byteorder.h>

#include <cstdint>

namespace ocp
{
namespace accelerator_management
{

/** @brief OCP MCTP VDM Message Type
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
constexpr uint8_t ocpType = 8;
constexpr uint8_t ocpVersion = 9;
constexpr uint8_t ocpTypeBitOffset = 4;
constexpr uint8_t ocpTypeBitMask = 0b11110000;
constexpr uint8_t ocpVersionBitMask = 0b00001111;
constexpr uint8_t instanceIdBitMask = 0b00011111;
constexpr uint8_t instanceIdReservedBitMask = 0b00100000;
constexpr uint8_t datagramBitMask = 0b01000000;
constexpr uint8_t requestBitMask = 0b10000000;
/** @} */

/**
 * @defgroup OCP MCTP VDM Instance Id
 *
 * v1 spec section 3.6.1.2.1
 * @{
 */
constexpr uint8_t instanceMin = 0;
constexpr uint8_t instanceMax = 31;
/** @} */

/** @brief OCP MCTP VDM completion codes
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

/** @brief OCP MCTP VDM reason codes
 *
 *  v1 spec section 3.6.3
 */
enum class ReasonCode : uint16_t
{
    REASON_NONE = 0x00,
};

/** @brief OCP MCTP VDM MessageType
 *
 *  v1 spec section 3.6.1.2.1
 */
enum class MessageType : uint8_t
{
    RESPONSE = 0, //!< OCP MCTP VDM response message
    REQUEST = 2,  //!< OCP MCTP VDM request message
};

/** @struct BindingPciVid
 *
 * Structure representing OCP MCTP VDM VDM binding using PCI vendor ID
 * v1 spec section 3.6.1.2
 */
struct BindingPciVid
{
    uint16_t pci_vendor_id;                      //!< PCI defined vendor ID
    uint8_t instance_id;                         //!< Instance ID
    uint8_t ocp_version;                         //!< OCP version
    uint8_t ocp_accelerator_management_msg_type; //!< Message Type
} __attribute__((packed));

/** @struct Message
 *
 * Structure representing OCP MCTP VDM message
 * v1 spec section 3.6.1.2
 */
struct Message
{
    BindingPciVid hdr; //!< OCP MCTP VDM message header
} __attribute__((packed));

/** @struct BindingPciVidInfo
 *
 * The information needed to prepare OCP MCTP VDM header and this is passed to
 * the PackHeader API. v1 spec section 3.6.1.2
 */
struct BindingPciVidInfo
{
    uint8_t ocp_accelerator_management_msg_type;
    uint8_t instance_id;
    uint8_t msg_type;
};

/** @struct CommonRequest
 *
 * Structure representing OCP MCTP VDM request without data (OCP version 1).
 * v1 spec section 3.6.1.4.1
 */
struct CommonRequest
{
    uint8_t command;
    uint8_t data_size;
} __attribute__((packed));

/** @struct CommonResponse
 *
 * Structure representing OCP MCTP VDM response with data
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
 * Structure representing OCP MCTP VDM response with reason code when CC !=
 * Success v1 spec section 3.6.1.4.5
 */
struct CommonNonSuccessResponse
{
    uint8_t command;
    uint8_t completion_code;
    uint16_t reason_code;
} __attribute__((packed));

/**
 * @brief Populate the OCP MCTP VDM message with the OCP MCTP VDM header. OCP
 * MCTP VDM header OCP Version will be populated with value 1. The caller of
 * this API allocates buffer for the OCP MCTP VDM header when forming the OCP
 * MCTP VDM message. The buffer is passed to this API to pack the OCP MCTP VDM
 * header.
 *
 * @param[in] pciVendorId - PCI Vendor ID
 * @param[in] hdr - OCP MCTP VDM header information
 * @param[out] msg - Reference to OCP MCTP VDM message header to be populated
 *
 * @return 0 on success, otherwise appropriate system error code
 * @note   Caller is responsible for alloc and dealloc of msg
 *         and hdr params
 */
int packHeader(uint16_t pciVendorId, const BindingPciVidInfo& hdr,
               BindingPciVid& msg);

/** @brief Encode reason code into an OCP MCTP VDM response message.
 *         This function does not populate or modifies the message header.
 *
 *  @param[in] cc - Completion Code
 *  @param[in] reasonCode - Reason code
 *  @param[in] commandCode - Command code
 *  @param[out] response - Reference to response message to be populated
 *  @return 0 on success, otherwise appropriate system error code
 */
int encodeReasonCode(uint8_t cc, uint16_t reasonCode, uint8_t commandCode,
                     CommonNonSuccessResponse& response);

/** @brief Decode the reason code and completion code from a response message
 *
 *  @param[in] response - Response message containing reason code and completion
 * code
 *  @param[out] cc - Reference to completion code to be populated
 *  @param[out] reasonCode - Reference to reason code to be populated
 *  @return 0 on success, otherwise appropriate system error code
 */
int decodeReasonCodeAndCC(const CommonNonSuccessResponse& response,
                          CompletionCode& cc, uint16_t& reasonCode);

} // namespace accelerator_management
} // namespace ocp
