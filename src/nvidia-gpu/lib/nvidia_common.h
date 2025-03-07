/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#ifndef NVIDIA_COMMON_H
#define NVIDIA_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ocp_ami.h>

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define NVIDIA_PCI_VENDOR_ID 0x10de

/** @brief NVIDIA OCP AMI OEM message types
 *  
 *  Enumeration of different message types used in NVIDIA OCP AMI OEM protocol.
 *  These types categorize different classes of messages for device management
 *  and monitoring.
 */
enum ocp_ami_oem_nvidia_msg_type {
	OCP_AMI_OEM_NVIDIA_TYPE_DEVICE_CAPABILITY_DISCOVERY = 0,
	OCP_AMI_OEM_NVIDIA_MSG_TYPE_PLATFORM_ENVIRONMENTAL = 3
};

/**
 * @brief Populate the OCP_AMI OEM NVIDIA message with the OCP_AMI OEM NVIDIA header.The caller of this API
 *        allocates buffer for the OCP_AMI OEM NVIDIA header when forming the OCP_AMI OEM NVIDIA message.
 *        The buffer is passed to this API to pack the OCP_AMI OEM NVIDIA header.
 *
 * @param[in] hdr - Pointer to the OCPAMI header information
 * @param[out] msg - Pointer to OCP_AMI OEM NVIDIA message header
 *
 * @return 0 on success, otherwise OCP_AMI OEM NVIDIA error codes.
 * @note   Caller is responsible for alloc and dealloc of msg
 *         and hdr params
 */
uint8_t
ocp_ami_oem_nvidia_pack_header(const struct ocp_ami_binding_pci_vid_info *hdr,
			       struct ocp_ami_binding_pci_vid *msg);

/** @brief Encode reason code
 *
 *  @param[in] cc     - Completion Code
 *  @param[in] reason_code - reason code
 *  @param[out] msg     - msg
 *  @return ocp_ami_completion_code
 */
int ocp_ami_oem_nvidia_encode_reason_code(uint8_t cc, uint16_t reason_code,
					  uint8_t command_code,
					  struct ocp_ami_msg *msg);

/** @brief Decode to get reason code
 *
 *  @param[in] msg    - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc     - pointer to completion code
 *  @param[out] reason_code - pointer to reason_code
 *  @return ocp_ami_completion_code
 */
int ocp_ami_oem_nvidia_decode_reason_code_and_cc(const struct ocp_ami_msg *msg,
						 size_t msg_len, uint8_t *cc,
						 uint16_t *reason_code);

#ifdef __cplusplus
}
#endif
#endif /* NVIDIA_COMMON_H */
