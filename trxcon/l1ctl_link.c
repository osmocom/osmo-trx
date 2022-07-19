/*
 * OsmocomBB <-> SDR connection bridge
 * GSM L1 control socket (/tmp/osmocom_l2) handlers
 *
 * (C) 2013 by Sylvain Munaut <tnt@246tNt.com>
 * (C) 2016-2017 by Vadim Yanitskiy <axilirator@gmail.com>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <sys/un.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <osmocom/core/fsm.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/select.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/write_queue.h>

#include "trxcon.h"
#include "logging.h"
#include "l1ctl_link.h"
#include "l1ctl.h"

static struct value_string l1ctl_evt_names[] = {
	{ 0, NULL } /* no events? */
};

static struct osmo_fsm_state l1ctl_fsm_states[] = {
	[L1CTL_STATE_IDLE] = {
		.out_state_mask = GEN_MASK(L1CTL_STATE_CONNECTED),
		.name = "IDLE",
	},
	[L1CTL_STATE_CONNECTED] = {
		.out_state_mask = GEN_MASK(L1CTL_STATE_IDLE),
		.name = "CONNECTED",
	},
};

static struct osmo_fsm l1ctl_fsm = {
	.name = "l1ctl_link_fsm",
	.states = l1ctl_fsm_states,
	.num_states = ARRAY_SIZE(l1ctl_fsm_states),
	.log_subsys = DL1C,
	.event_names = l1ctl_evt_names,
};

static int l1ctl_link_read_cb(struct osmo_fd *bfd)
{
	struct l1ctl_link *l1l = (struct l1ctl_link *) bfd->data;
	struct msgb *msg;
	uint16_t len;
	int rc;

	/* Attempt to read from socket */
	rc = read(bfd->fd, &len, L1CTL_MSG_LEN_FIELD);
	if (rc < L1CTL_MSG_LEN_FIELD) {
		LOGP(DL1D, LOGL_NOTICE, "L1CTL has lost connection\n");
		if (rc >= 0)
			rc = -EIO;
		l1ctl_link_close_conn(l1l);
		return rc;
	}

	/* Check message length */
	len = ntohs(len);
	if (len > L1CTL_LENGTH) {
		LOGP(DL1D, LOGL_ERROR, "Length is too big: %u\n", len);
		return -EINVAL;
	}

	/* Allocate a new msg */
	msg = msgb_alloc_headroom(L1CTL_LENGTH + L1CTL_HEADROOM,
		L1CTL_HEADROOM, "l1ctl_rx_msg");
	if (!msg) {
		LOGP(DL1D, LOGL_ERROR, "Failed to allocate msg\n");
		return -ENOMEM;
	}

	msg->l1h = msgb_put(msg, len);
	rc = read(bfd->fd, msg->l1h, msgb_l1len(msg));
	if (rc != len) {
		LOGP(DL1D, LOGL_ERROR, "Can not read data: len=%d < rc=%d: "
			"%s\n", len, rc, strerror(errno));
		msgb_free(msg);
		return rc;
	}

	/* Debug print */
	LOGP(DL1D, LOGL_DEBUG, "RX: '%s'\n",
		osmo_hexdump(msg->data, msg->len));

	/* Call L1CTL handler */
	l1ctl_rx_cb(l1l, msg);

	return 0;
}

static int l1ctl_link_write_cb(struct osmo_fd *bfd, struct msgb *msg)
{
	int len;

	if (bfd->fd <= 0)
		return -EINVAL;

	len = write(bfd->fd, msg->data, msg->len);
	if (len != msg->len) {
		LOGP(DL1D, LOGL_ERROR, "Failed to write data: "
			"written (%d) < msg_len (%d)\n", len, msg->len);
		return -1;
	}

	return 0;
}

/* Connection handler */
static int l1ctl_link_accept(struct osmo_fd *bfd, unsigned int flags)
{
	struct l1ctl_link *l1l = (struct l1ctl_link *) bfd->data;
	struct osmo_fd *conn_bfd = &l1l->wq.bfd;
	struct sockaddr_un un_addr;
	socklen_t len;
	int cfd;

	len = sizeof(un_addr);
	cfd = accept(bfd->fd, (struct sockaddr *) &un_addr, &len);
	if (cfd < 0) {
		LOGP(DL1C, LOGL_ERROR, "Failed to accept a new connection\n");
		return -1;
	}

	/* Check if we already have an active connection */
	if (conn_bfd->fd != -1) {
		LOGP(DL1C, LOGL_NOTICE, "A new connection rejected: "
			"we already have another active\n");
		close(cfd);
		return 0;
	}

	osmo_wqueue_init(&l1l->wq, 100);
	INIT_LLIST_HEAD(&conn_bfd->list);

	l1l->wq.write_cb = l1ctl_link_write_cb;
	l1l->wq.read_cb = l1ctl_link_read_cb;
	osmo_fd_setup(conn_bfd, cfd, OSMO_FD_READ, osmo_wqueue_bfd_cb, l1l, 0);

	if (osmo_fd_register(conn_bfd) != 0) {
		LOGP(DL1C, LOGL_ERROR, "Failed to register new connection fd\n");
		close(conn_bfd->fd);
		conn_bfd->fd = -1;
		return -1;
	}

	osmo_fsm_inst_dispatch(trxcon_fsm, L1CTL_EVENT_CONNECT, l1l);
	osmo_fsm_inst_state_chg(l1l->fsm, L1CTL_STATE_CONNECTED, 0, 0);

	LOGP(DL1C, LOGL_NOTICE, "L1CTL has a new connection\n");

	return 0;
}

int l1ctl_link_send(struct l1ctl_link *l1l, struct msgb *msg)
{
	uint8_t *len;

	/* Debug print */
	LOGP(DL1D, LOGL_DEBUG, "TX: '%s'\n",
		osmo_hexdump(msg->data, msg->len));

	if (msg->l1h != msg->data)
		LOGP(DL1D, LOGL_INFO, "Message L1 header != Message Data\n");

	/* Prepend 16-bit length before sending */
	len = msgb_push(msg, L1CTL_MSG_LEN_FIELD);
	osmo_store16be(msg->len - L1CTL_MSG_LEN_FIELD, len);

	if (osmo_wqueue_enqueue(&l1l->wq, msg) != 0) {
		LOGP(DL1D, LOGL_ERROR, "Failed to enqueue msg!\n");
		msgb_free(msg);
		return -EIO;
	}

	return 0;
}

int l1ctl_link_close_conn(struct l1ctl_link *l1l)
{
	struct osmo_fd *conn_bfd = &l1l->wq.bfd;

	if (conn_bfd->fd <= 0)
		return -EINVAL;

	/* Close connection socket */
	osmo_fd_unregister(conn_bfd);
	close(conn_bfd->fd);
	conn_bfd->fd = -1;

	/* Clear pending messages */
	osmo_wqueue_clear(&l1l->wq);

	osmo_fsm_inst_dispatch(trxcon_fsm, L1CTL_EVENT_DISCONNECT, l1l);
	osmo_fsm_inst_state_chg(l1l->fsm, L1CTL_STATE_IDLE, 0, 0);

	return 0;
}

struct l1ctl_link *l1ctl_link_init(void *tall_ctx, const char *sock_path)
{
	struct l1ctl_link *l1l;
	struct osmo_fd *bfd;
	int rc;

	LOGP(DL1C, LOGL_NOTICE, "Init L1CTL link (%s)\n", sock_path);

	l1l = talloc_zero(tall_ctx, struct l1ctl_link);
	if (!l1l) {
		LOGP(DL1C, LOGL_ERROR, "Failed to allocate memory\n");
		return NULL;
	}

	/* Allocate a new dedicated state machine */
	l1l->fsm = osmo_fsm_inst_alloc(&l1ctl_fsm, l1l,
		NULL, LOGL_DEBUG, "l1ctl_link");
	if (l1l->fsm == NULL) {
		LOGP(DTRX, LOGL_ERROR, "Failed to allocate an instance "
			"of FSM '%s'\n", l1ctl_fsm.name);
		talloc_free(l1l);
		return NULL;
	}

	/* Create a socket and bind handlers */
	bfd = &l1l->listen_bfd;

	/* Bind connection handler */
	osmo_fd_setup(bfd, -1, OSMO_FD_READ, l1ctl_link_accept, l1l, 0);

	rc = osmo_sock_unix_init_ofd(bfd, SOCK_STREAM, 0, sock_path,
		OSMO_SOCK_F_BIND);
	if (rc < 0) {
		LOGP(DL1C, LOGL_ERROR, "Could not create UNIX socket: %s\n",
			strerror(errno));
		osmo_fsm_inst_free(l1l->fsm);
		talloc_free(l1l);
		return NULL;
	}

	/* Bind shutdown handler */
	l1l->shutdown_cb = l1ctl_shutdown_cb;

	/**
	 * To be able to accept first connection and
	 * drop others, it should be set to -1
	 */
	l1l->wq.bfd.fd = -1;

	return l1l;
}

void l1ctl_link_shutdown(struct l1ctl_link *l1l)
{
	struct osmo_fd *listen_bfd;

	/* May be unallocated due to init error */
	if (!l1l)
		return;

	LOGP(DL1C, LOGL_NOTICE, "Shutdown L1CTL link\n");

	/* Call shutdown callback */
	if (l1l->shutdown_cb != NULL)
		l1l->shutdown_cb(l1l);

	listen_bfd = &l1l->listen_bfd;

	/* Check if we have an established connection */
	if (l1l->wq.bfd.fd != -1)
		l1ctl_link_close_conn(l1l);

	/* Unbind listening socket */
	if (listen_bfd->fd != -1) {
		osmo_fd_unregister(listen_bfd);
		close(listen_bfd->fd);
		listen_bfd->fd = -1;
	}

	osmo_fsm_inst_free(l1l->fsm);
	talloc_free(l1l);
}

static __attribute__((constructor)) void on_dso_load(void)
{
	OSMO_ASSERT(osmo_fsm_register(&l1ctl_fsm) == 0);
}
