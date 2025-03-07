/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#ifndef OCP_AMI_H
#define OCP_AMI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <asm/byteorder.h>

/** @brief OCPAMI Message Type
 *
 *  v1 spec section 3.6.1.2.1
 */
#define OCP_AMI_MESSAGE_TYPE 0x7E

/**
 * @defgroup OCP Version
 *
 * v1 spec section 3.6.1.2.1
 * @{
 */
#define OCP_TYPE    8
#define OCP_VERSION 9
/** @} */

/**
 * @defgroup OCPAMI Instance Id
 *
 * v1 spec section 3.6.1.2.1
 * @{
 */
#define OCP_AMI_INSTANCE_MIN	0
#define OCP_AMI_INSTANCEID_MASK 0x1F
#define OCP_AMI_INSTANCE_MAX	31
/** @} */

/** @brief OCPAMI completion codes
 *
 *  v1 spec section 3.6.2
 */
enum ocp_ami_completion_codes {
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
enum ocp_ami_reason_codes {
	NONE = 0x00,
};

/** @brief OCPAMI MessageType
 *
 *  v1 spec section 3.6.1.2.1
 */
enum ocp_ami_message_type {
	OCP_AMI_RESPONSE = 0,		  //!< OCPAMI response message
	OCP_AMI_EVENT_ACKNOWLEDGMENT = 1, //!< OCPAMI event acknowledgement
	OCP_AMI_REQUEST = 2,		  //!< OCPAMI request message
	OCP_AMI_EVENT = 3,		  //!< OCPAMI event message
};

/** @struct ocp_ami_binding_pci_vid
 *
 * Structure representing OCPAMI VDM binding using PCI vendor ID
 * v1 spec section 3.6.1.2
 */
struct ocp_ami_binding_pci_vid {
	uint16_t pci_vendor_id; //!< PCI defined vendor ID

#if defined(__LITTLE_ENDIAN_BITFIELD)
	uint8_t instance_id : 5; //!< Instance ID
	uint8_t reserved : 1;	 //!< Reserved
	uint8_t datagram : 1;	 //!< Datagram bit
	uint8_t request : 1;	 //!< Request bit
#elif defined(__BIG_ENDIAN_BITFIELD)
	uint8_t request : 1;	 //!< Request bit
	uint8_t datagram : 1;	 //!< Datagram bit
	uint8_t reserved : 1;	 //!< Reserved
	uint8_t instance_id : 5; //!< Instance ID
#endif

#if defined(__LITTLE_ENDIAN_BITFIELD)
	uint8_t ocp_version : 4; //!< OCP version
	uint8_t ocp_type : 4;	 //!< OCP type
#elif defined(__BIG_ENDIAN_BITFIELD)
	uint8_t ocp_type : 4;	 //!< OCP type
	uint8_t ocp_version : 4; //!< OCP version
#endif

	uint8_t ocp_ami_msg_type; //!< Message Type
} __attribute__((packed));

/** @struct ocp_ami_msg
 *
 * Structure representing OCPAMI message
 * v1 spec section 3.6.1.2
 */
struct ocp_ami_msg {
	struct ocp_ami_binding_pci_vid hdr; //!< OCPAMI message header
	uint8_t payload[1]; //!< &payload[0] is the beginning of the payload
} __attribute__((packed));

/** @struct ocp_ami_header_info
 *
 * The information needed to prepare OCPAMI header and this is passed to the
 * ocp_ami_pack_header and ocp_ami_unpack_header API.
 * v1 spec section 3.6.1.2
 */
struct ocp_ami_binding_pci_vid_info {
	uint8_t ocp_ami_msg_type;
	uint8_t instance_id;
	uint8_t msg_type;
};

/** @struct ocp_ami_common_req
 *
 * Structure representing OCPAMI request without data (OCP version 1).
 * v1 spec section 3.6.1.4.1
 */
struct ocp_ami_common_req {
	uint8_t command;
	uint8_t data_size;
} __attribute__((packed));

/** @struct ocp_ami_common_resp
 *
 * Structure representing OCPAMI response with data
 * v1 spec section 3.6.1.4.4
 */
struct ocp_ami_common_resp {
	uint8_t command;
	uint8_t completion_code;
	uint16_t reserved;
	uint16_t data_size;
} __attribute__((packed));

/** @struct ocp_ami_common_non_success_resp
 *
 * Structure representing OCPAMI response with reason code when CC != Success
 * v1 spec section 3.6.1.4.5
 */
struct ocp_ami_common_non_success_resp {
	uint8_t command;
	uint8_t completion_code;
	uint16_t reason_code;
} __attribute__((packed));

/**
 * @brief Populate the OCPAMI message with the OCPAMI header. OCPAMI header OCP Version will be populated with value 1.
The caller of this API
 *        allocates buffer for the OCPAMI header when forming the OCPAMI message.
 *        The buffer is passed to this API to pack the OCPAMI header.
 *
 * @param[in] pci_vendor_id - PCI Vendor ID
 * @param[in] hdr - Pointer to the OCPAMI header information
 * @param[out] msg - Pointer to OCPAMI message header
 *
 * @return 0 on success, otherwise OCPAMI error codes.
 * @note   Caller is responsible for alloc and dealloc of msg
 *         and hdr params
 */
uint8_t ocp_ami_pack_header(uint16_t pci_vendor_id,
			    const struct ocp_ami_binding_pci_vid_info *hdr,
			    struct ocp_ami_binding_pci_vid *msg);

/** @brief Encode reason code into an OCPAMI response message.
 *         This function does not populate or modifies the message header.
 *
 *  @param[in] cc     - Completion Code
 *  @param[in] reason_code - reason code
 *  @param[out] msg     - msg
 *  @return ocp_ami_completion_code
 */
int ocp_ami_encode_reason_code(uint8_t cc, uint16_t reason_code,
			       uint8_t command_code, struct ocp_ami_msg *msg);

/** @brief Decode the reason code
 *
 *  @param[in] msg    - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc     - pointer to completion code
 *  @param[out] reason_code - pointer to reason_code
 *  @return ocp_ami_completion_code
 */
int ocp_ami_decode_reason_code_and_cc(const struct ocp_ami_msg *msg,
				      size_t msg_len, uint8_t *cc,
				      uint16_t *reason_code);

#ifdef __cplusplus
}
#endif
#endif /* OCP_AMI_H */
