/* SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later */

#include <af-mctp.h>
#include <errno.h>
#include <linux/mctp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <transport.h>
#include <unistd.h>

struct mctp_transport_af_mctp {
	struct mctp_transport transport;
	int sockfd;
	uint8_t msg_type;
};

#ifndef container_of
#define container_of(ptr, type, member)                                        \
	(type *)((char *)(ptr)-offsetof(type, member))
#endif

int mctp_transport_af_mctp_send_msg(struct mctp_transport *transport,
				    mctp_eid_t eid, const void *msg,
				    size_t msg_len)
{
	struct mctp_transport_af_mctp *af_mctp = container_of(
		transport, struct mctp_transport_af_mctp, transport);

	struct sockaddr_mctp addr = { 0 };
	addr.smctp_family = AF_MCTP;
	addr.smctp_addr.s_addr = eid;
	addr.smctp_type = af_mctp->msg_type;
	addr.smctp_tag = MCTP_TAG_OWNER;

	ssize_t rc = sendto(af_mctp->sockfd, msg, msg_len, 0,
			    (struct sockaddr *)&addr, sizeof(addr));
	if (rc == -1) {
		return MCTP_TRANSPORT_SEND_FAIL;
	}

	return MCTP_TRANSPORT_SUCCESS;
}

int mctp_transport_af_mctp_recv_msg(struct mctp_transport *transport,
				    mctp_eid_t *eid, void **msg,
				    size_t *msg_len)
{
	struct mctp_transport_af_mctp *af_mctp = container_of(
		transport, struct mctp_transport_af_mctp, transport);

	struct sockaddr_mctp addr = { 0 };
	socklen_t addrlen = sizeof(addr);
	ssize_t peeked_length = 0;
	ssize_t received_length = 0;
	uint8_t *buf = NULL;
	int rc;

	peeked_length = recv(af_mctp->sockfd, NULL, 0, MSG_PEEK | MSG_TRUNC);
	if (peeked_length <= 0) {
		return MCTP_TRANSPORT_RECV_FAIL;
	}

	buf = malloc(peeked_length);
	if (buf == NULL) {
		return MCTP_TRANSPORT_RECV_FAIL;
	}

	received_length = recvfrom(af_mctp->sockfd, buf, peeked_length,
				   MSG_TRUNC, (struct sockaddr *)&addr,
				   &addrlen);
	if (addr.smctp_type != af_mctp->msg_type) {
		rc = MCTP_TRANSPORT_MISMATCHED_MSG_TYPE;
		goto cleanup_buf;
	}

	*eid = addr.smctp_addr.s_addr;
	*msg = buf;
	*msg_len = received_length;
	return MCTP_TRANSPORT_SUCCESS;

cleanup_buf:
	free(buf);

	return rc;
}

int mctp_transport_af_mctp_get_fd(struct mctp_transport *transport)
{
	struct mctp_transport_af_mctp *af_mctp = container_of(
		transport, struct mctp_transport_af_mctp, transport);

	return af_mctp->sockfd;
}

int mctp_transport_af_mctp_init(struct mctp_transport_af_mctp **ctx,
				uint8_t msg_type)
{
	if (!ctx || *ctx) {
		return -EINVAL;
	}

	struct mctp_transport_af_mctp *af_mctp =
		calloc(1, sizeof(struct mctp_transport_af_mctp));
	if (!af_mctp) {
		return -ENOMEM;
	}

	af_mctp->transport.send_msg = mctp_transport_af_mctp_send_msg;
	af_mctp->transport.recv_msg = mctp_transport_af_mctp_recv_msg;
	af_mctp->transport.get_fd = mctp_transport_af_mctp_get_fd;
	af_mctp->msg_type = msg_type;
	af_mctp->sockfd = socket(AF_MCTP, SOCK_DGRAM, 0);
	if (af_mctp->sockfd < 0) {
		free(af_mctp);
		return -1;
	}

	*ctx = af_mctp;
	return 0;
}

void mctp_transport_af_mctp_destroy(struct mctp_transport_af_mctp *ctx)
{
	if (!ctx) {
		return;
	}
	close(ctx->sockfd);
	free(ctx);
}

struct mctp_transport *
mctp_transport_af_mctp_core(struct mctp_transport_af_mctp *ctx)
{
	if (!ctx) {
		return NULL;
	}
	return &ctx->transport;
}
