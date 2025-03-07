/* SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @typedef mctp_eid_t
 *
 *  Type definition for MCTP endpoint IDs.
 *  These IDs uniquely identify devices participating in MCTP communication.
 */
typedef uint8_t mctp_eid_t;

/** @brief MCTP transport error codes
 *
 *  Enumeration of possible error codes that can be returned by MCTP transport operations.
 *  These codes indicate the success or specific failure modes of transport operations.
 */
typedef enum mctp_transport_error_codes {
	MCTP_TRANSPORT_SUCCESS = 0,
	MCTP_TRANSPORT_FAIL = -1,
	MCTP_TRANSPORT_OPEN_FAIL = -2,
	MCTP_TRANSPORT_SEND_FAIL = -3,
	MCTP_TRANSPORT_RECV_FAIL = -4,
	MCTP_TRANSPORT_INVALID_RECV_LEN = -5,
	MCTP_TRANSPORT_MISMATCHED_MSG_TYPE = -6,
} mctp_transport_rc_t;

/** @struct mctp_transport
 *
 *  Structure representing the MCTP transport layer.
 *  This structure defines the operations for sending and receiving messages over MCTP.
 */
struct mctp_transport {
	int (*send_msg)(struct mctp_transport *transport, mctp_eid_t eid,
			const void *msg, size_t msg_len);
	int (*recv_msg)(struct mctp_transport *transport, mctp_eid_t *eid,
			void **msg, size_t *msg_len);
	int (*get_fd)(struct mctp_transport *transport);
};

/** @brief Send a message over MCTP
 *
 *  This function sends a message to a specific endpoint ID (EID) using the provided transport layer.
 *  It returns a transport error code indicating the success or failure of the operation.
 *
 *  @param[in] transport - Pointer to the MCTP transport layer structure
 *  @param[in] eid - Endpoint ID to send the message to
 *  @param[in] msg - Pointer to the message to send
 *  @param[in] msg_len - Length of the message to send
 *  @return mctp_transport_rc_t - Transport error code
 */
int mctp_transport_send_msg(struct mctp_transport *transport, mctp_eid_t eid,
			    const void *msg, size_t msg_len);

/** @brief Receive a message over MCTP
 *
 *  This function receives a message from the MCTP transport layer.
 *  It returns a transport error code indicating the success or failure of the operation.
 *
 *  @param[in] transport - Pointer to the MCTP transport layer structure
 *  @param[out] eid - Pointer to store the endpoint ID of the message sender
 *  @param[out] msg - Pointer to store the received message
 *  @param[out] msg_len - Pointer to store the length of the received message
 *  @return mctp_transport_rc_t - Transport error code
 */
int mctp_transport_recv_msg(struct mctp_transport *transport, mctp_eid_t *eid,
			    void **msg, size_t *msg_len);

/** @brief Get the file descriptor of the MCTP transport
 *
 *  This function returns the file descriptor of the MCTP transport layer.
 *  It returns a transport error code indicating the success or failure of the operation.
 *
 *  @param[in] transport - Pointer to the MCTP transport layer structure
 *  @return int - File descriptor of the MCTP transport
 */
int mctp_transport_get_fd(struct mctp_transport *transport);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_H */
