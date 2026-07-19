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

struct osmo_trx_ep {
	struct osmo_trx_ep_cfg cfg;	/* a copy of the config */
	void *priv;
	char *name;			/* log prefix */
	int log_cat;			/* logging category (default DLGLOBAL) */
	struct osmo_io_fd *clck_iofd;
	struct osmo_trx_ep_chan *chans;	/* array of cfg.num_chans channels */
};

#define LOGEP(ep, level, fmt, args...) \
	LOGP((ep)->log_cat, level, "%s: " fmt, (ep)->name, ## args)

/* Default (weak) RX handler stubs.  Applications override the handlers
 * for the directions they consume by defining their own (strong) versions;
 * these weak defaults keep the library link-complete (-no-undefined) and
 * cover the unused directions. */
__attribute__((weak))
void osmo_trx_ep_rx_clck_ind(struct osmo_trx_ep *ep, uint32_t fn)
{
	LOGEP(ep, LOGL_ERROR, "unhandled clock indication (fn=%u)\n", fn);
}

__attribute__((weak))
void osmo_trx_ep_rx_ctrl_msg(struct osmo_trx_ep *ep, unsigned int chan,
			     const struct osmo_trxc_msg *msg)
{
	LOGEP(ep, LOGL_ERROR, "chan=%u: unhandled TRXC message '%s'\n",
	      chan, osmo_trxc_msg_name(msg));
}

__attribute__((weak))
void osmo_trx_ep_rx_burst_ind(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxd_burst_ind *bi)
{
	LOGEP(ep, LOGL_ERROR, "chan=%u: unhandled %s\n",
	      chan, osmo_trxd_burst_ind_name(bi));
}

__attribute__((weak))
void osmo_trx_ep_rx_burst_req(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxd_burst_req *br)
{
	LOGEP(ep, LOGL_ERROR, "chan=%u: unhandled %s\n",
	      chan, osmo_trxd_burst_req_name(br));
}

/* Compute a local/remote UDP port: the L1 side uses a fixed offset of +100 */
static uint16_t trx_ep_port(const struct osmo_trx_ep *ep, bool local, uint16_t ofs)
{
	const bool l1 = (ep->cfg.mode == OSMO_TRX_EP_MODE_L1);

	if (local == l1) /* L1/local or TRX/remote */
		return ep->cfg.base_port + TRX_EP_L1_PORT_OFS + ofs;
	return ep->cfg.base_port + ofs;
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
		LOGEP(ep, LOGL_NOTICE, "chan=%u: Rx malformed TRXC message (rc=%d)\n",
		      chan->num, rc);
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
	if (ep->cfg.mode == OSMO_TRX_EP_MODE_L1) {
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
	LOGEP(ep, LOGL_NOTICE, "chan=%u: Rx malformed TRXD PDU (rc=%d)\n",
	      chan->num, rc);
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
			     ep->cfg.laddr, trx_ep_port(ep, true, ofs),
			     ep->cfg.raddr, trx_ep_port(ep, false, ofs),
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

/*! Allocate a TRX endpoint instance.
 *  \param[in] ctx talloc context to allocate from
 *  \param[in] cfg endpoint configuration (copied, incl. the addresses)
 *  \param[in] priv opaque application-private data
 *  \returns pointer to the allocated instance; NULL on error */
struct osmo_trx_ep *osmo_trx_ep_alloc(void *ctx, const struct osmo_trx_ep_cfg *cfg,
				      void *priv)
{
	struct osmo_trx_ep *ep;

	if (cfg == NULL || cfg->num_chans == 0)
		return NULL;
	if (cfg->laddr == NULL || cfg->raddr == NULL)
		return NULL;

	ep = talloc_zero(ctx, struct osmo_trx_ep);
	if (ep == NULL)
		return NULL;

	ep->cfg = *cfg;
	ep->cfg.laddr = talloc_strdup(ep, cfg->laddr);
	ep->cfg.raddr = talloc_strdup(ep, cfg->raddr);
	ep->priv = priv;
	ep->name = talloc_strdup(ep, "trx_ep");
	ep->log_cat = DLGLOBAL;

	ep->chans = talloc_zero_array(ep, struct osmo_trx_ep_chan, cfg->num_chans);
	if (ep->chans == NULL) {
		talloc_free(ep);
		return NULL;
	}
	for (unsigned int i = 0; i < cfg->num_chans; i++) {
		ep->chans[i] = (struct osmo_trx_ep_chan){
			.ep = ep,
			.num = i,
		};
	}

	return ep;
}

/*! Open the clock/ctrl/data sockets of the given endpoint.
 *  \returns 0 on success; negative on error (all sockets closed) */
int osmo_trx_ep_open(struct osmo_trx_ep *ep)
{
	LOGEP(ep, LOGL_NOTICE, "Opening TRXC/TRXD connections l=%s:%u<->r=%s:%u\n",
	      ep->cfg.laddr, trx_ep_port(ep, true, 0),
	      ep->cfg.raddr, trx_ep_port(ep, false, 0));

	if (ep->cfg.clock_socket) {
		ep->clck_iofd = trx_ep_open_iofd(ep, 0, &trx_ep_clck_ioops,
						 OSMO_TRXC_MSG_BUF_SIZE, ep);
		if (ep->clck_iofd == NULL)
			goto ret_error;
	}

	for (unsigned int i = 0; i < ep->cfg.num_chans; i++) {
		struct osmo_trx_ep_chan *chan = &ep->chans[i];

		chan->ctrl_iofd = trx_ep_open_iofd(ep, 2 * i + 1, &trx_ep_ctrl_ioops,
						   OSMO_TRXC_MSG_BUF_SIZE, chan);
		if (chan->ctrl_iofd == NULL)
			goto ret_error;

		chan->data_iofd = trx_ep_open_iofd(ep, 2 * i + 2, &trx_ep_data_ioops,
						   TRX_EP_DATA_BUF_SIZE, chan);
		if (chan->data_iofd == NULL)
			goto ret_error;
	}

	return 0;

ret_error:
	osmo_trx_ep_close(ep);
	return -EIO;
}

/*! Close all sockets of the given endpoint.
 *  Pending TRXC messages ('goodbye' commands like POWEROFF) are flushed
 *  to the socket (best-effort); pending Tx data batches are dropped. */
void osmo_trx_ep_close(struct osmo_trx_ep *ep)
{
	LOGEP(ep, LOGL_NOTICE, "Closing TRXC/TRXD connections l=%s:%u<->r=%s:%u\n",
	      ep->cfg.laddr, trx_ep_port(ep, true, 0),
	      ep->cfg.raddr, trx_ep_port(ep, false, 0));

	osmo_iofd_free(ep->clck_iofd);
	ep->clck_iofd = NULL;

	for (unsigned int i = 0; i < ep->cfg.num_chans; i++) {
		struct osmo_trx_ep_chan *chan = &ep->chans[i];

		if (chan->ctrl_iofd != NULL)
			osmo_iofd_flush(chan->ctrl_iofd);
		osmo_iofd_free(chan->ctrl_iofd);
		chan->ctrl_iofd = NULL;
		osmo_iofd_free(chan->data_iofd);
		chan->data_iofd = NULL;
		msgb_free(chan->tx_msg);
		chan->tx_msg = NULL;
	}
}

/*! Free the given endpoint instance (closes all sockets) */
void osmo_trx_ep_free(struct osmo_trx_ep *ep)
{
	if (ep == NULL)
		return;
	osmo_trx_ep_close(ep);
	talloc_free(ep);
}

/*! Obtain the application-private data */
void *osmo_trx_ep_get_priv(const struct osmo_trx_ep *ep)
{
	return ep->priv;
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

/*! Set the logging category (e.g. DTRX in osmo-bts; default: DLGLOBAL) */
void osmo_trx_ep_set_log_cat(struct osmo_trx_ep *ep, int log_cat)
{
	ep->log_cat = log_cat;
}

/*! Set the TRXD PDU version in use for the given channel (default: 0),
 *  usually after the negotiation (see osmo_trxc_client_negotiate_format()) */
void osmo_trx_ep_set_pdu_ver(struct osmo_trx_ep *ep, unsigned int chan, uint8_t ver)
{
	OSMO_ASSERT(chan < ep->cfg.num_chans);
	LOGEP(ep, LOGL_INFO, "chan=%u: using TRXD PDU version %u\n", chan, ver);
	ep->chans[chan].pdu_ver = ver;
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

	OSMO_ASSERT(chan < ep->cfg.num_chans);

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

/*! Send a burst indication on the data socket of the given channel
 *  (OSMO_TRX_EP_MODE_TRX only).
 *  \returns 0 on success; negative on error */
int osmo_trx_ep_send_burst_ind(struct osmo_trx_ep *ep, unsigned int chan,
			       const struct osmo_trxd_burst_ind *bi)
{
	struct osmo_trx_ep_chan *c;
	struct msgb *msg;
	int rc;

	OSMO_ASSERT(chan < ep->cfg.num_chans);
	c = &ep->chans[chan];

	msg = msgb_alloc_c(ep, TRX_EP_DATA_BUF_SIZE, "trx_ep_data_tx");
	rc = osmo_trxd_burst_ind_build(msg, c->pdu_ver, bi);
	if (rc < 0) {
		msgb_free(msg);
		return rc;
	}
	osmo_trxd_build_fin(msg, c->pdu_ver);

	rc = osmo_iofd_write_msgb(c->data_iofd, msg);
	if (rc < 0)
		msgb_free(msg);
	return rc;
}

/*! Send a burst transmit request on the data socket of the given channel
 *  (OSMO_TRX_EP_MODE_L1 only).
 *
 *  For TRXDv2, the PDUs are batched: they get accumulated until this
 *  function is called with br == NULL (the batching breaker), which
 *  transmits all accumulated PDUs in a single datagram.  For TRXDv0/v1,
 *  each PDU is transmitted immediately; the breaker is a no-op.
 *
 *  \returns 0 on success; -ENOMSG for a breaker with nothing to send;
 *	     other negative on error */
int osmo_trx_ep_send_burst_req(struct osmo_trx_ep *ep, unsigned int chan,
			       const struct osmo_trxd_burst_req *br)
{
	struct osmo_trx_ep_chan *c;
	struct msgb *msg;
	int rc;

	OSMO_ASSERT(chan < ep->cfg.num_chans);
	c = &ep->chans[chan];

	/* the batching breaker */
	if (br == NULL) {
		if (c->tx_msg == NULL)
			return -ENOMSG;
		msg = c->tx_msg;
		c->tx_msg = NULL;
		/* unset BATCH.ind in the last accumulated PDU */
		osmo_trxd_build_fin(msg, c->pdu_ver);
		rc = osmo_iofd_write_msgb(c->data_iofd, msg);
		if (rc < 0)
			msgb_free(msg);
		return rc;
	}

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

	/* TRXDv2 and higher: wait for the batching breaker */
	if (c->pdu_ver >= 2) {
		c->tx_msg = msg;
		return 0;
	}

	osmo_trxd_build_fin(msg, c->pdu_ver); /* no-op for TRXDv0/v1 */
	rc = osmo_iofd_write_msgb(c->data_iofd, msg);
	if (rc < 0)
		msgb_free(msg);
	return rc;
}
