/* SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later */

#include <ocp_ami.h>
#include <stdlib.h>
#include <transport.h>

int mctp_transport_send_msg(struct mctp_transport *transport, mctp_eid_t eid,
			    const void *msg, size_t msg_len)
{
	if (!transport || !msg_len) {
		return -1;
	}

	if (msg_len < sizeof(struct ocp_ami_binding_pci_vid)) {
		return -2;
	}

	return transport->send_msg(transport, eid, msg, msg_len);
}

int mctp_transport_recv_msg(struct mctp_transport *transport, mctp_eid_t *eid,
			    void **msg, size_t *msg_len)
{
	if (!transport || !msg_len) {
		return -1;
	}

	int rc = transport->recv_msg(transport, eid, msg, msg_len);
	if (rc < 0) {
		return rc;
	}

	if (*msg_len < sizeof(struct ocp_ami_binding_pci_vid)) {
		free(*msg);
		*msg = NULL;
		return -2;
	}

	return 0;
}

int mctp_transport_get_fd(struct mctp_transport *transport)
{
	if (!transport) {
		return -1;
	}

	return transport->get_fd(transport);
}
