/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#ifndef GPU_SENSORS_H
#define GPU_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <gpu_common.h>

/** @brief Type0 Device Capability Discovery Commands
 */
enum gpu_device_capability_discovery_commands {
	GPU_QUERY_DEVICE_IDENTIFICATION = 0x09,
};

/** @brief device identification types
 *  
 *  Enumeration of different device types that can be identified in the system.
 *  This is used to distinguish between various components during device discovery.
 */
typedef enum {
	GPU_DEV_ID_GPU = 0,
	GPU_DEV_ID_SWITCH = 1,
	GPU_DEV_ID_PCIE_BRIDGE = 2,
	GPU_DEV_ID_BASEBOARD = 3,
	GPU_DEV_ID_EROT = 4,
	GPU_DEV_ID_UNKNOWN = 0xff,
} gpu_device_identification;

/** @brief Type3 platform environmental commands
 */
enum gpu_platform_environmental_commands {
	GPU_GET_TEMPERATURE_READING = 0x00,
};

/** @struct nsm_query_device_identification_req
 *
 *  Structure representing query device identification request
 */
struct gpu_query_device_identification_req {
	struct ocp_ami_common_req hdr;
} __attribute__((packed));

/** @struct nsm_query_device_identification_resp
 *
 *  Structure representing query device identification response.
 */
struct gpu_query_device_identification_resp {
	struct ocp_ami_common_resp hdr;
	uint8_t device_identification;
	uint8_t instance_id;
} __attribute__((packed));

/** @struct gpu_get_numeric_sensor_reading_req
 *
 *  Structure representing request to get reading of certain numeric
 * sensors.
 */
struct gpu_get_numeric_sensor_reading_req {
	struct ocp_ami_common_req hdr;
	uint8_t sensor_id;
} __attribute__((packed));

/** @struct gpu_get_temperature_reading_req
 *
 *  Structure representing get temperature reading request.
 */
typedef struct gpu_get_numeric_sensor_reading_req
	gpu_get_temperature_reading_req;

/** @struct gpu_get_temperature_reading_resp
 *
 *  Structure representing get temperature reading response.
 */
struct gpu_get_temperature_reading_resp {
	struct ocp_ami_common_resp hdr;
	int32_t reading;
} __attribute__((packed));

/** @brief Create a Query device identification request message
 *
 *  @param[in] instance_id - instance ID
 *  @param[out] msg - Message will be written to this
 *  @return ocp_ami_completion_codes
 */
int gpu_encode_query_device_identification_req(uint8_t instance_id,
					       struct ocp_ami_msg *msg);

/** @brief Encode a Query device identification response message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] cc     - completion code
 *  @param[in] reason_code     - reason code
 *  @param[in] device_identification - device identification
 *  @param[in] instance_id - instance id
 *  @param[out] msg - Message will be written to this
 *  @return ocp_ami_completion_codes
 */
int gpu_encode_query_device_identification_resp(uint8_t instance, uint8_t cc,
						uint16_t reason_code,
						uint8_t device_identification,
						uint8_t device_instance,
						struct ocp_ami_msg *msg);

/** @brief Decode a Query device identification response message
 *
 *  @param[in] msg    - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc     - pointer to completion code
 *  @param[out] reason_code     - pointer to reason code
 *  @param[out] device_identification - pointer to device_identification
 *  @param[out] instance_id - pointer to instance id
 *  @return ocp_ami_completion_codes
 */
int gpu_decode_query_device_identification_resp(const struct ocp_ami_msg *msg,
						size_t msg_len, uint8_t *cc,
						uint16_t *reason_code,
						uint8_t *device_identification,
						uint8_t *device_instance);

/** @brief Encode a Get temperature readings request message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] sensor_id - sensor id
 *  @param[out] msg - Message will be written to this
 *  @return ocp_ami_completion_codes
 */
int gpu_encode_get_temperature_reading_req(uint8_t instance, uint8_t sensor_id,
					   struct ocp_ami_msg *msg);

/** @brief Decode a Get temperature readings request message
 *
 *  @param[in] msg    - request message
 *  @param[in] msg_len - Length of request message
 *  @param[out] sensor_id - sensor id
 *  @return ocp_ami_completion_codes
 */
int gpu_decode_get_temperature_reading_req(const struct ocp_ami_msg *msg,
					   size_t msg_len, uint8_t *sensor_id);

/** @brief Encode a Get temperature readings response message
 *
 *  @param[in] instance_id - instance ID
 *  @param[in] cc - pointer to response message completion code
 *  @param[in] reason_code - reason code
 *  @param[in] temperature_reading - temperature reading
 *  @param[out] msg - Message will be written to this
 *  @return ocp_ami_completion_codes
 */
int gpu_encode_get_temperature_reading_resp(uint8_t instance_id, uint8_t cc,
					    uint16_t reason_code,
					    double temperature_reading,
					    struct ocp_ami_msg *msg);

/** @brief Decode a Get temperature readings response message
 *
 *  @param[in] msg    - response message
 *  @param[in] msg_len - Length of response message
 *  @param[out] cc - pointer to response message completion code
 *  @param[out] reason_code     - pointer to reason code
 *  @param[out] temperature_reading - temperature_reading
 *  @return ocp_ami_completion_codes
 */
int gpu_decode_get_temperature_reading_resp(const struct ocp_ami_msg *msg,
					    size_t msg_len, uint8_t *cc,
					    uint16_t *reason_code,
					    double *temperature_reading);

#ifdef __cplusplus
}
#endif
#endif /* GPU_SENSORS_H */
