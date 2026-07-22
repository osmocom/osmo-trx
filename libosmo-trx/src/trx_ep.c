/*! \file src/trx_ep.c
 * TRX protocol socket endpoint management (clock/ctrl/data UDP sockets).
 * Based on the socket handling in osmo-bts-trx (trx_if.c) and the
 * port numbering convention of trx_toolkit (transceiver.py). */

/*
 * (C) 2013 Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2016-2017 Harald Welte <laforge@gnumonks.org>
 * (C) 2019 Vadim Yanitskiy <axilirator@gmail.com>
 * (C) 2021-2026 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>

#include <netinet/in.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/osmo_io.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/utils.h>

#include <osmocom/trx/trxc.h>
#include <osmocom/trx/trxd.h>
#include <osmocom/trx/ep.h>

/*! The L1 side uses a fixed base port offset (see trx_toolkit) */
#define TRX_EP_L1_PORT_OFS	100

/*! Maximum TRXD datagram size: 8 batched PDUs with the largest bursts */
#define TRX_EP_DATA_BUF_SIZE	4096

struct osmo_trx_ep_chan {
	struct osmo_trx_ep *ep;		/* back-pointer */
	unsigned int num;		/* channel index */
	struct osmo_io_fd *ctrl_iofd;
	struct osmo_io_fd *data_iofd;
	uint8_t pdu_ver;		/* TRXD PDU version in use */
	struct msgb *tx_msg;		/* pending TRXDv2 Tx batch */
};

/*! Sockets currently bound */
#define OSMO_TRX_EP_F_OPEN			(1 << 0)
/*! Enable the clock socket (base_port + 0) */
#define OSMO_TRX_EP_F_CLOCK_SOCKET		(1 << 1)

struct osmo_trx_ep {
	uint32_t flags;			/* see OSMO_TRX_EP_F_* */
	enum osmo_trx_ep_mode mode;
	char *laddr;			/* local address */
	char *raddr;			/* remote address */
	uint16_t base_port;
	void *priv;
	char *name;			/* log prefix */
	int log_cat;			/* logging category (default DLGLOBAL) */
	struct osmo_io_fd *clck_iofd;
	struct osmo_trx_ep_chan *chans;	/* array of num_chans channels */
	unsigned int num_chans;
};

/*! Default base UDP port, see osmo_trx_ep_set_base_port() */
#define TRX_EP_DEFAULT_BASE_PORT	5700

#define LOGEP(ep, level, fmt, args...) \
	LOGP((ep) ? (ep)->log_cat : DLGLOBAL, level, "(ep=%s) " fmt, \
	     (ep) ? (ep)->name : "NULL", ## args)

#define LOGEPCH(ep, chan, level, fmt, args...) \
	LOGP((ep) ? (ep)->log_cat : DLGLOBAL, level, "(ep=%s, chan=%u) " fmt, \
	     (ep) ? (ep)->name : "NULL", chan, ## args)

/* Default (weak) RX handler stubs.  Applications override the handlers
 * for the directions they consume by defining their own (strong) versions;
 * these weak defaults keep the library link-complete (-no-undefined) and
 * cover the unused directions. */
__attribute__((weak))
void osmo_trx_ep_rx_clck_ind(struct osmo_trx_ep *ep, uint32_t fn)
{
	LOGEP(ep, LOGL_ERROR, "Unhandled clock indication (fn=%u)\n", fn);
}

__attribute__((weak))
void osmo_trx_ep_rx_ctrl_msg(struct osmo_trx_ep *ep, unsigned int chan,
			     const struct osmo_trxc_msg *msg)
{
	LOGEPCH(ep, chan, LOGL_ERROR,
		"Unhandled TRXC message '%s'\n",
		osmo_trxc_msg_name(msg));
}

__attribute__((weak))
void osmo_trx_ep_rx_burst_ind(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxd_burst_ind *bi)
{
	LOGEPCH(ep, chan, LOGL_ERROR, "Unhandled %s\n", osmo_trxd_burst_ind_name(bi));
}

__attribute__((weak))
void osmo_trx_ep_rx_burst_req(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxd_burst_req *br)
{
	LOGEPCH(ep, chan, LOGL_ERROR, "Unhandled %s\n", osmo_trxd_burst_req_name(br));
}

/* Compute a local/remote UDP port: the L1 side uses a fixed offset of +100 */
static uint16_t trx_ep_port(const struct osmo_trx_ep *ep, bool local, uint16_t ofs)
{
	const bool l1 = (ep->mode == OSMO_TRX_EP_MODE_L1);

	if (local == l1) /* L1/local or TRX/remote */
		return ep->base_port + TRX_EP_L1_PORT_OFS + ofs;
	return ep->base_port + ofs;
}

/***********************************************************************
 * RX paths
 ***********************************************************************/

static void trx_ep_clck_read_cb(struct osmo_io_fd *iofd, int res, struct msgb *msg)
{
	struct osmo_trx_ep *ep = osmo_iofd_get_data(iofd);
	uint32_t fn;
	int rc;

	if (res <= 0)
		goto ret_free_msg;

	rc = osmo_trxc_clock_ind_parse(&fn, (const char *)msgb_data(msg), msgb_length(msg));
	if (rc < 0) {
		LOGEP(ep, LOGL_NOTICE, "Rx malformed clock indication (rc=%d)\n", rc);
		goto ret_free_msg;
	}

	osmo_trx_ep_rx_clck_ind(ep, fn);

ret_free_msg:
	msgb_free(msg);
}

static void trx_ep_ctrl_read_cb(struct osmo_io_fd *iofd, int res, struct msgb *msg)
{
	struct osmo_trx_ep_chan *chan = osmo_iofd_get_data(iofd);
	struct osmo_trx_ep *ep = chan->ep;
	struct osmo_trxc_msg tmsg;
	int rc;

	if (res <= 0)
		goto ret_free_msg;

	rc = osmo_trxc_msg_parse(&tmsg, (const char *)msgb_data(msg), msgb_length(msg));
	if (rc < 0) {
		LOGEPCH(ep, chan->num, LOGL_NOTICE, "Rx malformed TRXC message (rc=%d)\n", rc);
		goto ret_free_msg;
	}

	osmo_trx_ep_rx_ctrl_msg(ep, chan->num, &tmsg);

ret_free_msg:
	msgb_free(msg);
}

static void trx_ep_data_read_cb(struct osmo_io_fd *iofd, int res, struct msgb *msg)
{
	struct osmo_trx_ep_chan *chan = osmo_iofd_get_data(iofd);
	struct osmo_trx_ep *ep = chan->ep;
	struct osmo_trxd_parse_state st;
	const uint8_t *buf = msgb_data(msg);
	size_t buf_len = msgb_length(msg);
	int rc;

	if (res <= 0)
		goto ret_free_msg;

	osmo_trxd_parse_state_init(&st);

	/* starting from TRXDv2, a datagram may batch multiple PDUs */
	if (ep->mode == OSMO_TRX_EP_MODE_L1) {
		while (buf_len > 0) {
			struct osmo_trxd_burst_ind bi;

			rc = osmo_trxd_burst_ind_parse(&st, &bi, buf, buf_len);
			if (rc < 0)
				goto parse_error;
			osmo_trx_ep_rx_burst_ind(ep, chan->num, &bi);
			buf += rc;
			buf_len -= rc;
		}
	} else {
		while (buf_len > 0) {
			struct osmo_trxd_burst_req br;

			rc = osmo_trxd_burst_req_parse(&st, &br, buf, buf_len);
			if (rc < 0)
				goto parse_error;
			osmo_trx_ep_rx_burst_req(ep, chan->num, &br);
			buf += rc;
			buf_len -= rc;
		}
	}

	msgb_free(msg);
	return;

parse_error:
	LOGEPCH(ep, chan->num, LOGL_NOTICE, "Rx malformed TRXD PDU (rc=%d)\n", rc);
ret_free_msg:
	msgb_free(msg);
}

static void trx_ep_write_cb(struct osmo_io_fd *iofd, int res, struct msgb *msg)
{
	/* nothing to do, but osmo_io requires a write call-back */
}

/***********************************************************************
 * open/close
 ***********************************************************************/

static const struct osmo_io_ops trx_ep_clck_ioops = {
	.read_cb = &trx_ep_clck_read_cb,
	.write_cb = &trx_ep_write_cb,
};

static const struct osmo_io_ops trx_ep_ctrl_ioops = {
	.read_cb = &trx_ep_ctrl_read_cb,
	.write_cb = &trx_ep_write_cb,
};

static const struct osmo_io_ops trx_ep_data_ioops = {
	.read_cb = &trx_ep_data_read_cb,
	.write_cb = &trx_ep_write_cb,
};

/* Open a single UDP socket (base port + ofs) and set up osmo_io for it */
static struct osmo_io_fd *trx_ep_open_iofd(struct osmo_trx_ep *ep, uint16_t ofs,
					   const struct osmo_io_ops *ioops,
					   unsigned int buf_size, void *data)
{
	char sock_name[OSMO_SOCK_NAME_MAXLEN];
	struct osmo_io_fd *iofd;
	int fd;

	fd = osmo_sock_init2(AF_UNSPEC, SOCK_DGRAM, IPPROTO_UDP,
			     ep->laddr, trx_ep_port(ep, true, ofs),
			     ep->raddr, trx_ep_port(ep, false, ofs),
			     OSMO_SOCK_F_BIND | OSMO_SOCK_F_CONNECT | OSMO_SOCK_F_NONBLOCK);
	if (fd < 0) {
		LOGEP(ep, LOGL_ERROR, "Failed to open a socket (ofs=%u): %d\n", ofs, fd);
		return NULL;
	}

	osmo_sock_get_name_buf(sock_name, sizeof(sock_name), fd);
	iofd = osmo_iofd_setup(ep, fd, sock_name, OSMO_IO_FD_MODE_READ_WRITE, ioops, data);
	if (iofd == NULL) {
		close(fd);
		return NULL;
	}

	osmo_iofd_set_alloc_info(iofd, buf_size, 0);

	if (osmo_iofd_register(iofd, -1) < 0) {
		osmo_iofd_free(iofd);
		return NULL;
	}

	return iofd;
}

/* Initialize a channel's state (back-pointer and index only, no sockets) */
static void trx_ep_chan_init(struct osmo_trx_ep_chan *chan, struct osmo_trx_ep *ep, unsigned int num)
{
	*chan = (struct osmo_trx_ep_chan){
		.ep = ep,
		.num = num,
	};
}

/* Open a channel's ctrl+data sockets.
 * \returns 0 on success; -EIO on error */
static int trx_ep_chan_open(struct osmo_trx_ep_chan *chan)
{
	struct osmo_trx_ep *ep = chan->ep;

	chan->ctrl_iofd = trx_ep_open_iofd(ep, 2 * chan->num + 1, &trx_ep_ctrl_ioops,
					   OSMO_TRXC_MSG_BUF_SIZE, chan);
	if (chan->ctrl_iofd == NULL) {
		LOGEPCH(ep, chan->num, LOGL_ERROR, "Failed to open TRXC socket\n");
		return -EIO;
	}

	chan->data_iofd = trx_ep_open_iofd(ep, 2 * chan->num + 2, &trx_ep_data_ioops,
					   TRX_EP_DATA_BUF_SIZE, chan);
	if (chan->data_iofd == NULL) {
		LOGEPCH(ep, chan->num, LOGL_ERROR, "Failed to open TRXD socket\n");
		return -EIO;
	}

	return 0;
}

/* Close a channel's ctrl+data sockets and drop its pending Tx batch */
static void trx_ep_chan_close(struct osmo_trx_ep_chan *chan)
{
	osmo_iofd_free(chan->ctrl_iofd);
	chan->ctrl_iofd = NULL;
	osmo_iofd_free(chan->data_iofd);
	chan->data_iofd = NULL;
	msgb_free(chan->tx_msg);
	chan->tx_msg = NULL;
}

/*! Allocate a TRX endpoint instance.  The mode, addresses and base port
 *  default to zero/unset and must be configured via the osmo_trx_ep_set_*()
 *  accessors before osmo_trx_ep_open(); num_chans may be changed later via
 *  osmo_trx_ep_set_num_chans(), but before osmo_trx_ep_open().
 *  \param[in] ctx talloc context to allocate from
 *  \param[in] num_chans number of channels (> 0)
 *  \returns pointer to the allocated instance; NULL on error */
struct osmo_trx_ep *osmo_trx_ep_alloc(void *ctx, unsigned int num_chans)
{
	struct osmo_trx_ep *ep;

	if (num_chans == 0)
		return NULL;

	ep = talloc_zero(ctx, struct osmo_trx_ep);
	if (ep == NULL)
		return NULL;

	ep->base_port = TRX_EP_DEFAULT_BASE_PORT;
	ep->name = talloc_strdup(ep, "trx_ep");
	ep->log_cat = DLGLOBAL;

	if (osmo_trx_ep_set_num_chans(ep, num_chans) != 0) {
		talloc_free(ep);
		return NULL;
	}

	return ep;
}

/*! Open the clock/ctrl/data sockets of the given endpoint.
 *  \returns 0 on success; -EALREADY if already open; other negative
 *  values on error (all sockets closed) */
int osmo_trx_ep_open(struct osmo_trx_ep *ep)
{
	if (ep->flags & OSMO_TRX_EP_F_OPEN)
		return -EALREADY;

	if (ep->laddr == NULL || ep->raddr == NULL)
		return -EINVAL;

	LOGEP(ep, LOGL_INFO, "Opening TRXC/TRXD connections l=%s:%u<->r=%s:%u\n",
	      ep->laddr, trx_ep_port(ep, true, 0),
	      ep->raddr, trx_ep_port(ep, false, 0));

	/* Set early (not just on success) so a partial-open failure below
	 * still runs the osmo_trx_ep_close() cleanup instead of it being
	 * skipped as a no-op on an endpoint that OSMO_TRX_EP_F_OPEN says
	 * isn't open yet. */
	ep->flags |= OSMO_TRX_EP_F_OPEN;

	if (ep->flags & OSMO_TRX_EP_F_CLOCK_SOCKET) {
		ep->clck_iofd = trx_ep_open_iofd(ep, 0, &trx_ep_clck_ioops,
						 OSMO_TRXC_MSG_BUF_SIZE, ep);
		if (ep->clck_iofd == NULL)
			goto ret_error;
	}

	for (unsigned int i = 0; i < ep->num_chans; i++) {
		if (trx_ep_chan_open(&ep->chans[i]) < 0)
			goto ret_error;
	}

	return 0;

ret_error:
	osmo_trx_ep_close(ep);
	return -EIO;
}

/*! Close all sockets of the given endpoint (drops pending Tx batches).
 *  No-op if not opened. */
void osmo_trx_ep_close(struct osmo_trx_ep *ep)
{
	if (~ep->flags & OSMO_TRX_EP_F_OPEN)
		return; /* nothing to do here */

	LOGEP(ep, LOGL_INFO, "Closing TRXC/TRXD connections l=%s:%u<->r=%s:%u\n",
	      ep->laddr, trx_ep_port(ep, true, 0),
	      ep->raddr, trx_ep_port(ep, false, 0));

	osmo_iofd_free(ep->clck_iofd);
	ep->clck_iofd = NULL;

	for (unsigned int i = 0; i < ep->num_chans; i++)
		trx_ep_chan_close(&ep->chans[i]);

	ep->flags &= ~OSMO_TRX_EP_F_OPEN;
}

/*! Free the given endpoint instance (closes all sockets) */
void osmo_trx_ep_free(struct osmo_trx_ep *ep)
{
	if (ep == NULL)
		return;
	osmo_trx_ep_close(ep);
	talloc_free(ep);
}

/*! Whether the endpoint's sockets are currently open */
bool osmo_trx_ep_is_open(const struct osmo_trx_ep *ep)
{
	return ep->flags & OSMO_TRX_EP_F_OPEN;
}

/*! Set the application-private data */
void osmo_trx_ep_set_priv(struct osmo_trx_ep *ep, void *priv)
{
	ep->priv = priv;
}

/*! Obtain the application-private data */
void *osmo_trx_ep_get_priv(const struct osmo_trx_ep *ep)
{
	return ep->priv;
}

/*! Obtain the number of channels */
unsigned int osmo_trx_ep_get_num_chans(const struct osmo_trx_ep *ep)
{
	return ep->num_chans;
}

/*! Change the number of channels; only valid before osmo_trx_ep_open().
 *  \returns 0 on success; -EBUSY if the endpoint is already open; -EINVAL
 *  if num_chans is 0; -ENOMEM on allocation failure */
int osmo_trx_ep_set_num_chans(struct osmo_trx_ep *ep, unsigned int num_chans)
{
	if (ep->flags & OSMO_TRX_EP_F_OPEN)
		return -EBUSY;

	if (num_chans == 0)
		return -EINVAL;
	if (num_chans == ep->num_chans)
		return 0; /* nothing to do */

	talloc_free(ep->chans); /* re-alloc: free() and allocate again */
	ep->chans = talloc_array(ep, struct osmo_trx_ep_chan, num_chans);
	if (ep->chans == NULL)
		return -ENOMEM;

	for (unsigned int n = 0; n < num_chans; n++)
		trx_ep_chan_init(&ep->chans[n], ep, n);

	ep->num_chans = num_chans;
	return 0;
}

/*! Set the endpoint mode (default: OSMO_TRX_EP_MODE_L1) */
int osmo_trx_ep_set_mode(struct osmo_trx_ep *ep, enum osmo_trx_ep_mode mode)
{
	if (ep->flags & OSMO_TRX_EP_F_OPEN)
		return -EBUSY;

	ep->mode = mode;
	return 0;
}

/*! Obtain the endpoint mode */
enum osmo_trx_ep_mode osmo_trx_ep_get_mode(const struct osmo_trx_ep *ep)
{
	return ep->mode;
}

/*! Set the local IP address (copied) */
int osmo_trx_ep_set_laddr(struct osmo_trx_ep *ep, const char *addr)
{
	if (ep->flags & OSMO_TRX_EP_F_OPEN)
		return -EBUSY;
	if (addr == NULL)
		return -EINVAL;
	osmo_talloc_replace_string(ep, &ep->laddr, addr);
	return 0;
}

/*! Obtain the local IP address */
const char *osmo_trx_ep_get_laddr(const struct osmo_trx_ep *ep)
{
	return ep->laddr;
}

/*! Set the remote IP address (copied) */
int osmo_trx_ep_set_raddr(struct osmo_trx_ep *ep, const char *addr)
{
	if (ep->flags & OSMO_TRX_EP_F_OPEN)
		return -EBUSY;
	if (addr == NULL)
		return -EINVAL;
	osmo_talloc_replace_string(ep, &ep->raddr, addr);
	return 0;
}

/*! Obtain the remote IP address */
const char *osmo_trx_ep_get_raddr(const struct osmo_trx_ep *ep)
{
	return ep->raddr;
}

/*! Set the base UDP port (default: 5700), see ep.h for the port layout */
int osmo_trx_ep_set_base_port(struct osmo_trx_ep *ep, uint16_t base_port)
{
	if (ep->flags & OSMO_TRX_EP_F_OPEN)
		return -EBUSY;

	ep->base_port = base_port;
	return 0;
}

/*! Obtain the base UDP port */
uint16_t osmo_trx_ep_get_base_port(const struct osmo_trx_ep *ep)
{
	return ep->base_port;
}

/*! Enable/disable the clock socket (default: false) */
int osmo_trx_ep_set_clock_socket(struct osmo_trx_ep *ep, bool enable)
{
	if (ep->flags & OSMO_TRX_EP_F_OPEN)
		return -EBUSY;

	if (enable)
		ep->flags |= OSMO_TRX_EP_F_CLOCK_SOCKET;
	else
		ep->flags &= ~OSMO_TRX_EP_F_CLOCK_SOCKET;
	return 0;
}

/*! Whether the clock socket is enabled */
bool osmo_trx_ep_get_clock_socket(const struct osmo_trx_ep *ep)
{
	return ep->flags & OSMO_TRX_EP_F_CLOCK_SOCKET;
}

/*! Set the name (log prefix) of the given instance, e.g. "phy0" */
int osmo_trx_ep_set_name(struct osmo_trx_ep *ep, const char *fmt, ...)
{
	char name[64];
	va_list ap;
	int rc;

	va_start(ap, fmt);
	rc = vsnprintf(name, sizeof(name), fmt, ap);
	va_end(ap);

	if (rc < 0 || rc >= (int)sizeof(name))
		return -EMSGSIZE;
	osmo_talloc_replace_string(ep, &ep->name, name);

	return 0;
}

/*! Obtain the name (log prefix) of the given instance */
const char *osmo_trx_ep_get_name(const struct osmo_trx_ep *ep)
{
	return ep->name;
}

/*! Set the logging category (e.g. DTRX in osmo-bts; default: DLGLOBAL) */
void osmo_trx_ep_set_log_cat(struct osmo_trx_ep *ep, int log_cat)
{
	ep->log_cat = log_cat;
}

/*! Set the TRXD PDU version in use for the given channel (default: 0),
 *  usually after the negotiation (see osmo_trxc_client_negotiate_format()) */
int osmo_trx_ep_set_pdu_ver(struct osmo_trx_ep *ep, unsigned int chan, uint8_t ver)
{
	if (chan >= ep->num_chans)
		return -ENOENT;
	if (ver > OSMO_TRXD_PDU_VER_MAX)
		return -ENOTSUP;

	LOGEPCH(ep, chan, LOGL_INFO, "Using TRXD PDU version %u\n", ver);
	ep->chans[chan].pdu_ver = ver;
	return 0;
}

/*! Get the TRXD PDU version in use for the given channel */
int osmo_trx_ep_get_pdu_ver(const struct osmo_trx_ep *ep, unsigned int chan)
{
	if (chan >= ep->num_chans)
		return -ENOENT;

	return ep->chans[chan].pdu_ver;
}

/***********************************************************************
 * TX paths
 ***********************************************************************/

/*! Send a clock indication ("IND CLOCK <fn>") on the clock socket.
 *  \returns 0 on success; negative on error */
int osmo_trx_ep_send_clck_ind(struct osmo_trx_ep *ep, uint32_t fn)
{
	struct msgb *msg;
	int rc;

	if (ep->clck_iofd == NULL)
		return -ENOTSUP;

	msg = msgb_alloc_c(ep, OSMO_TRXC_MSG_BUF_SIZE, "trx_ep_clck_tx");
	rc = osmo_trxc_clock_ind_build((char *)msgb_data(msg), msgb_tailroom(msg), fn);
	if (rc < 0) {
		msgb_free(msg);
		return rc;
	}
	msgb_put(msg, rc);

	rc = osmo_iofd_write_msgb(ep->clck_iofd, msg);
	if (rc < 0)
		msgb_free(msg);
	return rc;
}

/*! Send a TRXC message on the ctrl socket of the given channel.
 *  \returns 0 on success; negative on error */
int osmo_trx_ep_send_ctrl_msg(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxc_msg *tmsg)
{
	struct msgb *msg;
	int rc;

	OSMO_ASSERT(chan < ep->num_chans);

	msg = msgb_alloc_c(ep, OSMO_TRXC_MSG_BUF_SIZE, "trx_ep_ctrl_tx");
	rc = osmo_trxc_msg_build((char *)msgb_data(msg), msgb_tailroom(msg), tmsg);
	if (rc < 0) {
		msgb_free(msg);
		return rc;
	}
	msgb_put(msg, rc);

	rc = osmo_iofd_write_msgb(ep->chans[chan].ctrl_iofd, msg);
	if (rc < 0)
		msgb_free(msg);
	return rc;
}

/*! Flush a channel's pending Tx batching buffer: write the accumulated msgb
 *  to the data socket and clear c->tx_msg.  Shared by
 *  osmo_trx_ep_send_burst_fin() and the immediate (non-batched, TRXDv0/v1)
 *  send path. */
static int trx_ep_data_tx_flush(struct osmo_trx_ep_chan *c)
{
	struct msgb *msg = c->tx_msg;
	int rc;

	c->tx_msg = NULL;
	/* unset BATCH.ind in the last accumulated PDU */
	osmo_trxd_build_fin(msg, c->pdu_ver);
	rc = osmo_iofd_write_msgb(c->data_iofd, msg);
	if (rc < 0)
		msgb_free(msg);
	return rc;
}

/*! Send a burst indication on the data socket of the given channel
 *  (OSMO_TRX_EP_MODE_TRX only).
 *
 *  For TRXDv2, the PDUs are batched: they get accumulated until
 *  osmo_trx_ep_send_burst_fin() is called, which transmits all
 *  accumulated PDUs in a single datagram.  For TRXDv0/v1, each PDU is
 *  transmitted immediately.
 *
 *  \returns 0 on success; negative on error */
int osmo_trx_ep_send_burst_ind(struct osmo_trx_ep *ep, unsigned int chan,
			       const struct osmo_trxd_burst_ind *bi)
{
	struct osmo_trx_ep_chan *c;
	struct msgb *msg;
	int rc;

	OSMO_ASSERT(chan < ep->num_chans);
	c = &ep->chans[chan];

	if (c->tx_msg != NULL) {
		msg = c->tx_msg;
	} else {
		msg = msgb_alloc_c(ep, TRX_EP_DATA_BUF_SIZE, "trx_ep_data_tx");
		if (msg == NULL)
			return -ENOMEM;
	}

	rc = osmo_trxd_burst_ind_build(msg, c->pdu_ver, bi);
	if (rc < 0) {
		if (c->tx_msg == NULL)
			msgb_free(msg);
		return rc;
	}

	/* TRXDv2 and higher: wait for osmo_trx_ep_send_burst_fin() */
	if (c->pdu_ver >= 2) {
		c->tx_msg = msg;
		return 0;
	}

	c->tx_msg = msg;
	return trx_ep_data_tx_flush(c);
}

/*! Send a burst transmit request on the data socket of the given channel
 *  (OSMO_TRX_EP_MODE_L1 only).
 *
 *  For TRXDv2, the PDUs are batched: they get accumulated until
 *  osmo_trx_ep_send_burst_fin() is called, which transmits all
 *  accumulated PDUs in a single datagram.  For TRXDv0/v1, each PDU is
 *  transmitted immediately.
 *
 *  \returns 0 on success; negative on error */
int osmo_trx_ep_send_burst_req(struct osmo_trx_ep *ep, unsigned int chan,
			       const struct osmo_trxd_burst_req *br)
{
	struct osmo_trx_ep_chan *c;
	struct msgb *msg;
	int rc;

	OSMO_ASSERT(chan < ep->num_chans);
	c = &ep->chans[chan];

	if (c->tx_msg != NULL) {
		msg = c->tx_msg;
	} else {
		msg = msgb_alloc_c(ep, TRX_EP_DATA_BUF_SIZE, "trx_ep_data_tx");
		if (msg == NULL)
			return -ENOMEM;
	}

	rc = osmo_trxd_burst_req_build(msg, c->pdu_ver, br);
	if (rc < 0) {
		if (c->tx_msg == NULL)
			msgb_free(msg);
		return rc;
	}

	/* TRXDv2 and higher: wait for osmo_trx_ep_send_burst_fin() */
	if (c->pdu_ver >= 2) {
		c->tx_msg = msg;
		return 0;
	}

	c->tx_msg = msg;
	return trx_ep_data_tx_flush(c);
}

/*! Flush the TRXDv2 batching buffer accumulated by osmo_trx_ep_send_burst_ind()
 *  or osmo_trx_ep_send_burst_req() (whichever applies to this endpoint's
 *  mode), transmitting it in a single datagram; no-op (returns -ENOMSG) if
 *  nothing was accumulated, and for TRXDv0/v1 (each PDU is already sent
 *  immediately by osmo_trx_ep_send_burst_ind()/_req()).
 *  \returns 0 on success; -ENOMSG if there was nothing to flush;
 *	     other negative on error */
int osmo_trx_ep_send_burst_fin(struct osmo_trx_ep *ep, unsigned int chan)
{
	struct osmo_trx_ep_chan *c;

	OSMO_ASSERT(chan < ep->num_chans);
	c = &ep->chans[chan];

	if (c->tx_msg == NULL)
		return -ENOMSG;
	return trx_ep_data_tx_flush(c);
}
