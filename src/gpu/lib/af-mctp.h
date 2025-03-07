/* SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later */

#ifndef AF_MCTP_H
#define AF_MCTP_H

#include <transport.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @struct mctp_transport_af_mctp
 *
 *  Structure representing the AF MCTP transport layer.
 *  This structure defines the operations for sending and receiving messages over AF MCTP.
 */
struct mctp_transport_af_mctp;

/** @brief Initialize the AF MCTP transport layer
 *
 *  This function initializes the AF MCTP transport layer.
 *  It returns a transport error code indicating the success or failure of the operation.
 *
 *  @param[in] ctx - Pointer to the AF MCTP transport layer structure
 *  @param[in] msg_type - Message type to be used for the transport layer
 *  @return mctp_transport_rc_t - Transport error code
 */
int mctp_transport_af_mctp_init(struct mctp_transport_af_mctp **ctx,
				uint8_t msg_type);

/** @brief Destroy the AF MCTP transport layer
 *
 *  This function destroys the AF MCTP transport layer and releases any resources
 *  associated with it.
 *
 *  @param[in] ctx - Pointer to the AF MCTP transport layer structure
 */
void mctp_transport_af_mctp_destroy(struct mctp_transport_af_mctp *ctx);

/** @brief Return the core MCTP transport layer data structure
 *
 *  This function returns the core transport layer of the AF MCTP transport layer.
 *
 *  @param[in] ctx - Pointer to the AF MCTP transport layer structure
 *  @return struct mctp_transport* - Pointer to the core transport layer
 */
struct mctp_transport *
mctp_transport_af_mctp_core(struct mctp_transport_af_mctp *ctx);

#ifdef __cplusplus
}
#endif

#endif /* MCTP_DEMUX_H */
