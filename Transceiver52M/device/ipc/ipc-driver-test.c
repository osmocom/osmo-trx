/*
* Copyright 2020 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
* Author: Pau Espin Pedrol <pespin@sysmocom.de>
*
* SPDX-License-Identifier: AGPL-3.0+
*
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */

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

#define DEFAULT_SHM_NAME "/osmo-trx-ipc-driver-shm2"

struct ipc_sock_state {
	struct osmo_fd listen_bfd;	/* fd for listen socket */
        struct osmo_fd conn_bfd;	/* fd for connection to lcr */
	struct llist_head upqueue;	/* queue for sending messages */
};

static void *tall_ctx;
static struct ipc_sock_state *global_ipc_sock_state;

void *shm;


/* Debug Areas of the code */
enum {
	DMAIN,
};
static const struct log_info_cat default_categories[] = {
	[DMAIN] = {
		.name = "DMAIN",
		.description = "Main generic category",
		.color = NULL,
		.enabled = 1, .loglevel = LOGL_DEBUG,
	},
};

const struct log_info log_info = {
	.cat = default_categories,
	.num_cat = ARRAY_SIZE(default_categories),
};

static int ipc_shm_setup(const char *shm_name, size_t shm_len) {
        int fd;
        int rc;

        LOGP(DMAIN, LOGL_NOTICE, "Opening shm path %s\n", shm_name);
        if ((fd = shm_open(shm_name, O_CREAT|O_RDWR|O_TRUNC, S_IRUSR|S_IWUSR)) < 0) {
                LOGP(DMAIN, LOGL_ERROR, "shm_open %d: %s\n", errno, strerror(errno));
                rc = -errno;
                goto err_shm_open;
        }

        LOGP(DMAIN, LOGL_NOTICE, "Truncating %d to size %zu\n", fd, shm_len);
        if (ftruncate(fd, shm_len) < 0) {
                LOGP(DMAIN, LOGL_ERROR, "ftruncate %d: %s\n", errno, strerror(errno));
                rc = -errno;
                goto err_mmap;
        }

        LOGP(DMAIN, LOGL_NOTICE, "mmaping shared memory fd %d\n", fd);
        if ((shm = mmap(NULL, shm_len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0)) == MAP_FAILED) {
                LOGP(DMAIN, LOGL_ERROR, "mmap %d: %s\n", errno, strerror(errno));
                rc = -errno;
                goto err_mmap;
        }

        LOGP(DMAIN, LOGL_NOTICE, "mmap'ed shared memory at addr %p\n", shm);
        /* After a call to mmap(2) the file descriptor may be closed without affecting the memory mapping. */
        close(fd);
        return 0;
err_mmap:
        shm_unlink(shm_name);
        close(fd);
err_shm_open:
        return rc;
}

static int ipc_sock_send(struct msgb *msg);

struct msgb *ipc_msgb_alloc(uint8_t msg_type)
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;

	msg = msgb_alloc(sizeof(struct ipc_sk_if) + 1000, "ipc_sock_tx");
	if (!msg)
		return NULL;
	msgb_put(msg, sizeof(struct ipc_sk_if) + 1000);
	ipc_prim = (struct ipc_sk_if *) msg->data;
	ipc_prim->msg_type = msg_type;

	return msg;
}

static int ipc_tx_greeting_cnf(uint8_t req_version)
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;

	msg = ipc_msgb_alloc(IPC_IF_MSG_GREETING_CNF);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_if *) msg->data;
	ipc_prim->u.greeting_cnf.req_version = req_version;

	return ipc_sock_send(msg);
}

static int ipc_tx_info_cnf()
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;
        struct ipc_sk_if_info_chan *chan_info;
        int i;

	msg = ipc_msgb_alloc(IPC_IF_MSG_INFO_CNF);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_if *) msg->data;
        ipc_prim->u.info_cnf.feature_mask = FEATURE_MASK_CLOCKREF_INTERNAL |
                                            FEATURE_MASK_CLOCKREF_EXTERNAL;
        ipc_prim->u.info_cnf.min_rx_gain = 0.0;
        ipc_prim->u.info_cnf.max_rx_gain = 70.0;
        ipc_prim->u.info_cnf.min_tx_gain = 0.0;
        ipc_prim->u.info_cnf.max_tx_gain = 63.0;
	ipc_prim->u.info_cnf.max_num_chans = 2;
        OSMO_STRLCPY_ARRAY(ipc_prim->u.info_cnf.dev_desc, "Hello To my Virtual device!");
        chan_info = ipc_prim->u.info_cnf.chan_info;
        for (i = 0; i < ipc_prim->u.info_cnf.max_num_chans; i++) {
                OSMO_STRLCPY_ARRAY(chan_info->tx_path[0], "TxAntenna1");
                OSMO_STRLCPY_ARRAY(chan_info->tx_path[1], "TxAntenna2");
                OSMO_STRLCPY_ARRAY(chan_info->tx_path[2], "TxAntenna3");
                OSMO_STRLCPY_ARRAY(chan_info->rx_path[0], "RxAntenna1");
                OSMO_STRLCPY_ARRAY(chan_info->rx_path[1], "RxAntenna2");
                chan_info++;
        }

	return ipc_sock_send(msg);
}

static int ipc_tx_open_cnf(int rc, uint32_t num_chans)
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;
        struct ipc_sk_if_open_cnf_chan *chan_info;
        int i;

	msg = ipc_msgb_alloc(IPC_IF_MSG_OPEN_CNF);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_if *) msg->data;
        ipc_prim->u.open_cnf.return_code = rc;
        OSMO_STRLCPY_ARRAY(ipc_prim->u.open_cnf.shm_name, DEFAULT_SHM_NAME);

        chan_info = ipc_prim->u.open_cnf.chan_info;
        for (i = 0; i < num_chans; i++) {
                snprintf(chan_info->chan_ipc_sk_path, sizeof(chan_info->chan_ipc_sk_path), "%s_%d", IPC_SOCK_PATH, i+1);
                chan_info++;
        }

	return ipc_sock_send(msg);
}

static int ipc_rx_greeting_req(const struct ipc_sk_if_greeting *greeting_req)
{
        if (greeting_req->req_version == IPC_SOCK_API_VERSION)
                ipc_tx_greeting_cnf(IPC_SOCK_API_VERSION);
        else
                ipc_tx_greeting_cnf(0);
        return 0;
}

static int ipc_rx_info_req(const struct ipc_sk_if_info_req *info_req)
{
        ipc_tx_info_cnf();
        return 0;
}

static int ipc_rx_open_req(const struct ipc_sk_if_open_req *open_req)
{
        /* calculate size needed */
        unsigned int len;
        len = ipc_shm_encode_region(NULL, open_req->num_chans, 4, 4096);
        /* Here we verify num_chans, rx_path, tx_path, clockref, etc. */
        int rc = ipc_shm_setup(DEFAULT_SHM_NAME, len);
        len = ipc_shm_encode_region((struct ipc_shm_raw_region *)shm, open_req->num_chans, 4, 4096);
        LOGP(DMAIN, LOGL_NOTICE, "%s\n", osmo_hexdump((const unsigned char *) shm, 80));
        ipc_tx_open_cnf(-rc, open_req->num_chans);
        return 0;
}

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
		LOGP(DMAIN, LOGL_ERROR, "Received unknown PCU msg type %d\n",
			msg_type);
		rc = -EINVAL;
	}

	return rc;
}

static int ipc_sock_send(struct msgb *msg)
{
	struct ipc_sock_state *state = global_ipc_sock_state;
	struct osmo_fd *conn_bfd;
	//struct ipc_sk_if *ipc_prim = (struct ipc_sk_if *) msg->data;

	if (!state) {
		LOGP(DMAIN, LOGL_INFO, "PCU socket not created, "
			"dropping message\n");
		msgb_free(msg);
		return -EINVAL;
	}
	conn_bfd = &state->conn_bfd;
	if (conn_bfd->fd <= 0) {
		LOGP(DMAIN, LOGL_NOTICE, "PCU socket not connected, "
			"dropping message\n");
		msgb_free(msg);
		return -EIO;
	}
	msgb_enqueue(&state->upqueue, msg);
	conn_bfd->when |= BSC_FD_WRITE;

	return 0;
}

static void ipc_sock_close(struct ipc_sock_state *state)
{
	struct osmo_fd *bfd = &state->conn_bfd;

	LOGP(DMAIN, LOGL_NOTICE, "PCU socket has LOST connection\n");

	close(bfd->fd);
	bfd->fd = -1;
	osmo_fd_unregister(bfd);

	/* re-enable the generation of ACCEPT for new connections */
	state->listen_bfd.when |= BSC_FD_READ;

	/* flush the queue */
	while (!llist_empty(&state->upqueue)) {
		struct msgb *msg = msgb_dequeue(&state->upqueue);
		msgb_free(msg);
	}
}

static int ipc_sock_read(struct osmo_fd *bfd)
{
	struct ipc_sock_state *state = (struct ipc_sock_state *)bfd->data;
	struct ipc_sk_if *ipc_prim;
	struct msgb *msg;
	int rc;

	msg = msgb_alloc(sizeof(*ipc_prim) + 1000, "ipc_sock_rx");
	if (!msg)
		return -ENOMEM;

	ipc_prim = (struct ipc_sk_if *) msg->tail;

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

	if (rc < sizeof(*ipc_prim)) {
		LOGP(DMAIN, LOGL_ERROR, "Received %d bytes on Unix Socket, but primitive size "
		     "is %zu, discarding\n", rc, sizeof(*ipc_prim));
		msgb_free(msg);
		return 0;
	}

	rc = ipc_rx(ipc_prim->msg_type, ipc_prim);

	/* as we always synchronously process the message in pcu_rx() and
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
	struct ipc_sock_state *state = bfd->data;
	int rc;

	while (!llist_empty(&state->upqueue)) {
		struct msgb *msg, *msg2;
		struct ipc_sk_if *ipc_prim;

		/* peek at the beginning of the queue */
		msg = llist_entry(state->upqueue.next, struct msgb, list);
		ipc_prim = (struct ipc_sk_if *)msg->data;

		bfd->when &= ~BSC_FD_WRITE;

		/* bug hunter 8-): maybe someone forgot msgb_put(...) ? */
		if (!msgb_length(msg)) {
			LOGP(DMAIN, LOGL_ERROR, "message type (%d) with ZERO "
				"bytes!\n", ipc_prim->msg_type);
			goto dontsend;
		}

		/* try to send it over the socket */
		rc = write(bfd->fd, msgb_data(msg), msgb_length(msg));
		if (rc == 0)
			goto close;
		if (rc < 0) {
			if (errno == EAGAIN) {
				bfd->when |= BSC_FD_WRITE;
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

	if (flags & BSC_FD_READ)
		rc = ipc_sock_read(bfd);
	if (rc < 0)
		return rc;

	if (flags & BSC_FD_WRITE)
		rc = ipc_sock_write(bfd);

	return rc;
}

/* accept connection coming from PCU */
static int ipc_sock_accept(struct osmo_fd *bfd, unsigned int flags)
{
	struct ipc_sock_state *state = (struct ipc_sock_state *)bfd->data;
	struct osmo_fd *conn_bfd = &state->conn_bfd;
	struct sockaddr_un un_addr;
	socklen_t len;
	int rc;

	len = sizeof(un_addr);
	rc = accept(bfd->fd, (struct sockaddr *) &un_addr, &len);
	if (rc < 0) {
		LOGP(DMAIN, LOGL_ERROR, "Failed to accept a new connection\n");
		return -1;
	}

	if (conn_bfd->fd >= 0) {
		LOGP(DMAIN, LOGL_NOTICE, "osmo-trx connects but we already have "
			"another active connection ?!?\n");
		/* We already have one PCU connected, this is all we support */
		state->listen_bfd.when &= ~BSC_FD_READ;
		close(rc);
		return 0;
	}

	conn_bfd->fd = rc;
	conn_bfd->when = BSC_FD_READ;
	conn_bfd->cb = ipc_sock_cb;
	conn_bfd->data = state;

	if (osmo_fd_register(conn_bfd) != 0) {
		LOGP(DMAIN, LOGL_ERROR, "Failed to register new connection "
			"fd\n");
		close(conn_bfd->fd);
		conn_bfd->fd = -1;
		return -1;
	}

	LOGP(DMAIN, LOGL_NOTICE, "Unix socket connected to external osmo-trx\n");

	/* send current info */
	//pcu_tx_info_ind();

	return 0;
}

int ipc_sock_init(const char *path)
{
	struct ipc_sock_state *state;
	struct osmo_fd *bfd;
	int rc;

	state = talloc_zero(NULL, struct ipc_sock_state);
	if (!state)
		return -ENOMEM;
        global_ipc_sock_state = state;

	INIT_LLIST_HEAD(&state->upqueue);
        state->conn_bfd.fd = -1;

	bfd = &state->listen_bfd;

	bfd->fd = osmo_sock_unix_init(SOCK_SEQPACKET, 0, path,
		OSMO_SOCK_F_BIND);
	if (bfd->fd < 0) {
		LOGP(DMAIN, LOGL_ERROR, "Could not create %s unix socket: %s\n",
		     path, strerror(errno));
		talloc_free(state);
		return -1;
	}

	bfd->when = BSC_FD_READ;
	bfd->cb = ipc_sock_accept;
	bfd->data = state;

	rc = osmo_fd_register(bfd);
	if (rc < 0) {
		LOGP(DMAIN, LOGL_ERROR, "Could not register listen fd: %d\n",
			rc);
		close(bfd->fd);
		talloc_free(state);
		return rc;
	}

	//osmo_signal_register_handler(SS_GLOBAL, pcu_if_signal_cb, NULL);

	LOGP(DMAIN, LOGL_INFO, "Started listening on IPC socket: %s\n", path);

	return 0;
}


int main(int argc, char** argv) {
        tall_ctx = talloc_named_const(NULL, 0, "OsmoTRX");
        msgb_talloc_ctx_init(tall_ctx, 0);
        osmo_init_logging2(tall_ctx, &log_info);
        log_enable_multithread();
        LOGP(DMAIN, LOGL_INFO, "Starting %s\n", argv[0]);
        ipc_sock_init(IPC_SOCK_PATH);
        while (true)
                osmo_select_main(0);
        //ipc_sock_close()
        return 0;
}
