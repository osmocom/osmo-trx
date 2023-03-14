/*
* Copyright 2020 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
* Author: Pau Espin Pedrol <pespin@sysmocom.de>
*
* SPDX-License-Identifier: 0BSD
*
* Permission to use, copy, modify, and/or distribute this software for any purpose
* with or without fee is hereby granted.THE SOFTWARE IS PROVIDED "AS IS" AND THE
* AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
* BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
* WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF
* CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
* USE OR PERFORMANCE OF THIS SOFTWARE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <inttypes.h>
#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <fcntl.h> /* For O_* constants */

#include <debug.h>
#include <osmocom/core/application.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/select.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>

#include "shm.h"
#include "ipc-driver-test.h"

extern volatile int ipc_exit_requested;
static int ipc_rx(uint8_t msg_type, struct ipc_sk_if *ipc_prim)
{
	int rc = 0;

	switch (msg_type) {
	case IPC_IF_MSG_GREETING_REQ:
		rc = ipc_rx_greeting_req(&ipc_prim->u.greeting_req);
		break;
	case IPC_IF_MSG_INFO_REQ:
		rc = ipc_rx_info_req(&ipc_prim->u.info_req);
		break;
	case IPC_IF_MSG_OPEN_REQ:
		rc = ipc_rx_open_req(&ipc_prim->u.open_req);
		break;
	default:
		LOGP(DDEV, LOGL_ERROR, "Received unknown IPC msg type 0x%02x\n", msg_type);
		rc = -EINVAL;
	}

	return rc;
}

int ipc_sock_send(struct msgb *msg)
{
	struct ipc_sock_state *state = global_ipc_sock_state;
	struct osmo_fd *conn_bfd;

	if (!state) {
		LOGP(DDEV, LOGL_INFO,
		     "IPC socket not created, "
		     "dropping message\n");
		msgb_free(msg);
		return -EINVAL;
	}
	conn_bfd = &state->conn_bfd;
	if (conn_bfd->fd <= 0) {
		LOGP(DDEV, LOGL_NOTICE,
		     "IPC socket not connected, "
		     "dropping message\n");
		msgb_free(msg);
		return -EIO;
	}
	msgb_enqueue(&state->upqueue, msg);
	osmo_fd_write_enable(conn_bfd);

	return 0;
}

void ipc_sock_close(struct ipc_sock_state *state)
{
	struct osmo_fd *bfd = &state->conn_bfd;

	LOGP(DDEV, LOGL_NOTICE, "IPC socket has LOST connection\n");

	ipc_exit_requested = 1;

	osmo_fd_unregister(bfd);
	close(bfd->fd);
	bfd->fd = -1;

	/* re-enable the generation of ACCEPT for new connections */
	osmo_fd_read_enable(&state->listen_bfd);

	/* flush the queue */
	while (!llist_empty(&state->upqueue)) {
		struct msgb *msg = msgb_dequeue(&state->upqueue);
		msgb_free(msg);
	}
}

int ipc_sock_read(struct osmo_fd *bfd)
{
	struct ipc_sock_state *state = (struct ipc_sock_state *)bfd->data;
	struct ipc_sk_if *ipc_prim;
	struct msgb *msg;
	int rc;

	msg = msgb_alloc(sizeof(*ipc_prim) + 1000, "ipc_sock_rx");
	if (!msg)
		return -ENOMEM;

	ipc_prim = (struct ipc_sk_if *)msg->tail;

	rc = recv(bfd->fd, msg->tail, msgb_tailroom(msg), 0);
	if (rc == 0)
		goto close;

	if (rc < 0) {
		if (errno == EAGAIN) {
			msgb_free(msg);
			return 0;
		}
		goto close;
	}

	if (rc < (int)sizeof(*ipc_prim)) {
		LOGP(DDEV, LOGL_ERROR,
		     "Received %d bytes on Unix Socket, but primitive size "
		     "is %zu, discarding\n",
		     rc, sizeof(*ipc_prim));
		msgb_free(msg);
		return 0;
	}

	rc = ipc_rx(ipc_prim->msg_type, ipc_prim);

	/* as we always synchronously process the message in IPC_rx() and
	 * its callbacks, we can free the message here. */
	msgb_free(msg);

	return rc;

close:
	msgb_free(msg);
	ipc_sock_close(state);
	return -1;
}

static int ipc_sock_write(struct osmo_fd *bfd)
{
	struct ipc_sock_state *state = (struct ipc_sock_state *)bfd->data;
	int rc;

	while (!llist_empty(&state->upqueue)) {
		struct msgb *msg, *msg2;
		struct ipc_sk_if *ipc_prim;

		/* peek at the beginning of the queue */
		msg = llist_entry(state->upqueue.next, struct msgb, list);
		ipc_prim = (struct ipc_sk_if *)msg->data;

		osmo_fd_write_disable(bfd);

		/* bug hunter 8-): maybe someone forgot msgb_put(...) ? */
		if (!msgb_length(msg)) {
			LOGP(DDEV, LOGL_ERROR,
			     "message type (%d) with ZERO "
			     "bytes!\n",
			     ipc_prim->msg_type);
			goto dontsend;
		}

		/* try to send it over the socket */
		rc = write(bfd->fd, msgb_data(msg), msgb_length(msg));
		if (rc == 0)
			goto close;
		if (rc < 0) {
			if (errno == EAGAIN) {
				osmo_fd_write_enable(bfd);
				break;
			}
			goto close;
		}

	dontsend:
		/* _after_ we send it, we can deueue */
		msg2 = msgb_dequeue(&state->upqueue);
		assert(msg == msg2);
		msgb_free(msg);
	}
	return 0;

close:
	ipc_sock_close(state);
	return -1;
}

static int ipc_sock_cb(struct osmo_fd *bfd, unsigned int flags)
{
	int rc = 0;

	if (flags & OSMO_FD_READ)
		rc = ipc_sock_read(bfd);
	if (rc < 0)
		return rc;

	if (flags & OSMO_FD_WRITE)
		rc = ipc_sock_write(bfd);

	return rc;
}

/* accept connection coming from IPC */
int ipc_sock_accept(struct osmo_fd *bfd, unsigned int flags)
{
	struct ipc_sock_state *state = (struct ipc_sock_state *)bfd->data;
	struct osmo_fd *conn_bfd = &state->conn_bfd;
	struct sockaddr_un un_addr;
	socklen_t len;
	int rc;

	len = sizeof(un_addr);
	rc = accept(bfd->fd, (struct sockaddr *)&un_addr, &len);
	if (rc < 0) {
		LOGP(DDEV, LOGL_ERROR, "Failed to accept a new connection\n");
		return -1;
	}

	if (conn_bfd->fd >= 0) {
		LOGP(DDEV, LOGL_NOTICE,
		     "ip clent connects but we already have "
		     "another active connection ?!?\n");
		/* We already have one IPC connected, this is all we support */
		osmo_fd_read_disable(&state->listen_bfd);
		close(rc);
		return 0;
	}

	osmo_fd_setup(conn_bfd, rc, OSMO_FD_READ, ipc_sock_cb, state, 0);

	if (osmo_fd_register(conn_bfd) != 0) {
		LOGP(DDEV, LOGL_ERROR,
		     "Failed to register new connection "
		     "fd\n");
		close(conn_bfd->fd);
		conn_bfd->fd = -1;
		return -1;
	}

	LOGP(DDEV, LOGL_NOTICE, "Unix socket connected to external osmo-trx\n");

	return 0;
}
