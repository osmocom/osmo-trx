/*! \file src/trxc_client.c
 * TRXC client command queue engine: queue, retransmission, RSP matching.
 * Based on the TRXC command queue logic in osmo-bts-trx (trx_if.c). */

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
#include <stdbool.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/core/utils.h>

#include <osmocom/trx/trxc.h>
#include <osmocom/trx/trxc_client.h>

/*! Default retransmit timeout (in seconds) */
#define TRXC_CLIENT_RETRANS_SEC		2

static int trxc_client_fatal(struct osmo_trxc_client *client,
			     const struct osmo_trxc_msg *rsp);

/*! A single command in the queue */
struct trxc_cmd_entry {
	struct llist_head list;
	struct osmo_trxc_msg msg;	/* type == OSMO_TRXC_MT_CMD */
	uint32_t flags;			/* OSMO_TRXC_F_* */
	unsigned int n_retrans;	/* number of retransmissions so far */
	osmo_trxc_client_rsp_cb *rsp_cb;
	void *cb_data;
};

struct osmo_trxc_client {
	const struct osmo_trxc_client_ops *ops;
	char *name;			/* log prefix */
	int log_cat;			/* logging category (default DLGLOBAL) */
	unsigned int retrans_sec;	/* retransmit timeout */
	unsigned int max_retrans;	/* maximum number of retransmissions (0 = no limit) */

	struct llist_head cmd_queue;	/* list of struct trxc_cmd_entry */
	struct trxc_cmd_entry *last_acked;
	struct osmo_timer_list retrans_timer;

	/* guards for osmo_trxc_client_flush() from within a rsp_cb */
	bool in_rx;
	bool flushed_in_rx;

	/* TRXD PDU version negotiation state */
	uint8_t setformat_ver_req;
	osmo_trxc_setformat_cb *setformat_cb;
	void *setformat_cb_data;
};

#define LOGCL(client, level, fmt, args...) \
	LOGP((client)->log_cat, level, "%s: " fmt, (client)->name, ## args)

#define CMD_NAME_FMT "CMD %s%s%s"
#define CMD_NAME_ARGS(e) \
	(e)->msg.cmd, (e)->msg.params[0] != '\0' ? " " : "", (e)->msg.params

/* Transmit the first command in the queue (if any), (re)start the timer */
static void trxc_client_send_next(struct osmo_trxc_client *client)
{
	struct trxc_cmd_entry *e;
	int rc;

	if (llist_empty(&client->cmd_queue))
		return;
	e = llist_first_entry(&client->cmd_queue, struct trxc_cmd_entry, list);

	LOGCL(client, LOGL_DEBUG, "Tx '%s'\n", osmo_trxc_msg_name(&e->msg));
	rc = client->ops->tx_msg(client, &e->msg);
	if (rc < 0)
		LOGCL(client, LOGL_ERROR, "tx_msg() failed with rc=%d\n", rc);

	osmo_timer_schedule(&client->retrans_timer, client->retrans_sec, 0);
}

static void trxc_client_retrans_timer_cb(void *data)
{
	struct osmo_trxc_client *client = data;
	struct trxc_cmd_entry *e;

	OSMO_ASSERT(!llist_empty(&client->cmd_queue));
	e = llist_first_entry(&client->cmd_queue, struct trxc_cmd_entry, list);

	LOGCL(client, LOGL_NOTICE, "No response from transceiver for '" CMD_NAME_FMT "'\n",
	      CMD_NAME_ARGS(e));

	if (client->max_retrans != 0 && e->n_retrans >= client->max_retrans) {
		LOGCL(client, LOGL_FATAL, "Giving up on '" CMD_NAME_FMT "' after %u "
		      "retransmissions, transceiver offline?\n",
		      CMD_NAME_ARGS(e), e->n_retrans);
		trxc_client_fatal(client, NULL);
		return; /* keep the command queue frozen, do not re-arm the timer */
	}

	e->n_retrans++;
	trxc_client_send_next(client);
}

/*! Allocate a TRXC client instance.
 *  \param[in] ctx talloc context to allocate from
 *  \param[in] ops call-backs (with ops->tx_msg being mandatory) and
 *		   application-private data; must remain valid throughout
 *		   the lifetime of the instance
 *  \returns pointer to the allocated instance; NULL on error */
struct osmo_trxc_client *osmo_trxc_client_alloc(void *ctx,
						const struct osmo_trxc_client_ops *ops)
{
	struct osmo_trxc_client *client;

	if (ops == NULL || ops->tx_msg == NULL)
		return NULL;

	client = talloc_zero(ctx, struct osmo_trxc_client);
	if (client == NULL)
		return NULL;

	client->ops = ops;
	client->name = talloc_strdup(client, "trxc_client");
	client->log_cat = DLGLOBAL;
	client->retrans_sec = TRXC_CLIENT_RETRANS_SEC;

	INIT_LLIST_HEAD(&client->cmd_queue);
	osmo_timer_setup(&client->retrans_timer, &trxc_client_retrans_timer_cb, client);

	return client;
}

/*! Free the given TRXC client instance (flushes the command queue).
 *  Must not be called from within a response call-back. */
void osmo_trxc_client_free(struct osmo_trxc_client *client)
{
	if (client == NULL)
		return;
	OSMO_ASSERT(!client->in_rx);
	osmo_trxc_client_flush(client);
	talloc_free(client);
}

/*! Obtain the application-private data (ops->priv) */
void *osmo_trxc_client_get_priv(const struct osmo_trxc_client *client)
{
	return client->ops->priv;
}

/*! Set the name (log prefix) of the given instance, e.g. "phy0.trx0" */
int osmo_trxc_client_set_name(struct osmo_trxc_client *client, const char *fmt, ...)
{
	char name[64];
	va_list ap;
	int rc;

	va_start(ap, fmt);
	rc = vsnprintf(name, sizeof(name), fmt, ap);
	va_end(ap);

	if (rc < 0 || rc >= (int)sizeof(name))
		return -EMSGSIZE;
	osmo_talloc_replace_string(client, &client->name, name);

	return 0;
}

/*! Set the logging category (e.g. DTRX in osmo-bts; default: DLGLOBAL) */
void osmo_trxc_client_set_log_cat(struct osmo_trxc_client *client, int log_cat)
{
	client->log_cat = log_cat;
}

/*! Set the retransmit timeout in seconds (default: 2) */
void osmo_trxc_client_set_retrans(struct osmo_trxc_client *client, unsigned int sec)
{
	client->retrans_sec = sec;
}

/*! Set the maximum number of retransmissions of a command (default: 0).
 *  Once a command has been retransmitted the given number of times without
 *  a response, the engine gives up and escalates to the fatal_error
 *  call-back (with rsp == NULL); the command queue remains frozen.
 *  The special value 0 means no limit: retransmit indefinitely. */
void osmo_trxc_client_set_max_retrans(struct osmo_trxc_client *client, unsigned int n)
{
	client->max_retrans = n;
}

/*! Enqueue a new command for transmission.
 *
 *  The new command is added to the end of the queue; there's at most one
 *  command in flight at any time.  Consecutive duplicate commands are not
 *  enqueued.  Commands are retransmitted until a matching response is
 *  received (see osmo_trxc_client_rx()).
 *
 *  \param[in] client TRXC client instance
 *  \param[in] flags OSMO_TRXC_F_*
 *  \param[in] cb call-back invoked on the response (optional); without it,
 *		  a NACKed OSMO_TRXC_F_CRITICAL command is escalated to the
 *		  fatal_error call-back, other responses are just logged
 *  \param[in] cb_data opaque data for the response call-back
 *  \param[in] cmd command verb, e.g. "POWERON"
 *  \param[in] fmt format string for the parameters (optional, may be NULL)
 *  \returns 0 on success; negative on error */
int osmo_trxc_client_send_cmd(struct osmo_trxc_client *client, uint32_t flags,
			      osmo_trxc_client_rsp_cb *cb, void *cb_data,
			      const char *cmd, const char *fmt, ...)
{
	struct trxc_cmd_entry *e, *prev = NULL;
	va_list ap;
	int rc;

	e = talloc_zero(client, struct trxc_cmd_entry);
	if (e == NULL)
		return -ENOMEM;

	e->msg.type = OSMO_TRXC_MT_CMD;
	if (osmo_strlcpy(e->msg.cmd, cmd, sizeof(e->msg.cmd)) >= sizeof(e->msg.cmd)) {
		talloc_free(e);
		return -EMSGSIZE;
	}
	if (fmt != NULL && fmt[0] != '\0') {
		va_start(ap, fmt);
		rc = vsnprintf(e->msg.params, sizeof(e->msg.params), fmt, ap);
		va_end(ap);
		if (rc < 0 || rc >= (int)sizeof(e->msg.params)) {
			talloc_free(e);
			return -EMSGSIZE;
		}
	}

	e->flags = flags;
	e->rsp_cb = cb;
	e->cb_data = cb_data;

	/* avoid enqueueing consecutive duplicates, e.g. two POWEROFF */
	if (!llist_empty(&client->cmd_queue))
		prev = llist_last_entry(&client->cmd_queue, struct trxc_cmd_entry, list);
	if (prev != NULL && strcmp(prev->msg.cmd, e->msg.cmd) == 0
			 && strcmp(prev->msg.params, e->msg.params) == 0) {
		LOGCL(client, LOGL_DEBUG, "Not enqueueing duplicate '" CMD_NAME_FMT "'\n",
		      CMD_NAME_ARGS(e));
		talloc_free(e);
		return 0;
	}

	LOGCL(client, LOGL_INFO, "Enqueueing '" CMD_NAME_FMT "'\n", CMD_NAME_ARGS(e));
	llist_add_tail(&e->list, &client->cmd_queue);

	/* transmit, unless we already have a command in flight.
	 * If we are in the rx code path, skip transmitting: it's done
	 * when returning from the response handling. */
	if (prev == NULL && !client->in_rx)
		trxc_client_send_next(client);

	return 0;
}

/*! Flush (drop) all pending commands.  May be called from within
 *  a response call-back. */
void osmo_trxc_client_flush(struct osmo_trxc_client *client)
{
	struct trxc_cmd_entry *e, *e2;

	llist_for_each_entry_safe(e, e2, &client->cmd_queue, list) {
		llist_del(&e->list);
		talloc_free(e);
	}

	TALLOC_FREE(client->last_acked);

	/* the queue is empty now, no point in keeping the timer armed */
	osmo_timer_del(&client->retrans_timer);

	/* if we are in the rx code path, signal to the returning code path */
	if (client->in_rx)
		client->flushed_in_rx = true;
}

static bool cmd_matches_rsp(const struct trxc_cmd_entry *e,
			    const struct osmo_trxc_msg *rsp)
{
	if (strcmp(e->msg.cmd, rsp->cmd) != 0)
		return false;
	/* Some commands (e.g. SETSLOT) may be pending for different params,
	 * so the response shall additionally be matched by the params. */
	if ((e->flags & OSMO_TRXC_F_MATCH_PARAMS) && strcmp(e->msg.params, rsp->params) != 0)
		return false;
	return true;
}

/* Default response handling, when no rsp_cb was given */
static int trxc_client_default_rsp_cb(struct osmo_trxc_client *client,
				      const struct trxc_cmd_entry *e,
				      const struct osmo_trxc_msg *rsp)
{
	if (rsp->status == 0)
		return 0;

	LOGCL(client, (e->flags & OSMO_TRXC_F_CRITICAL) ? LOGL_FATAL : LOGL_NOTICE,
	      "Transceiver rejected '" CMD_NAME_FMT "' with response '%s'\n",
	      CMD_NAME_ARGS(e), osmo_trxc_msg_name(rsp));

	if (e->flags & OSMO_TRXC_F_CRITICAL)
		return -EINVAL;
	return 0;
}

static int trxc_client_fatal(struct osmo_trxc_client *client,
			     const struct osmo_trxc_msg *rsp)
{
	if (client->ops->fatal_error != NULL) {
		client->ops->fatal_error(client, rsp);
	} else {
		LOGCL(client, LOGL_FATAL, "A critical command failed ('%s'), "
		      "and no fatal_error call-back is given\n",
		      rsp ? osmo_trxc_msg_name(rsp) : "timeout");
	}

	/* keep the command queue frozen, so the processing is stopped */
	return -EINVAL;
}

/*! Feed a received (parsed) response message into the engine.
 *
 *  The engine filters duplicate responses caused by retransmissions,
 *  matches the response against the command in flight, invokes its
 *  response call-back and transmits the next queued command (if any).
 *  E.g. an osmo_trx_ep user calls this from osmo_trx_ep_rx_ctrl_msg().
 *
 *  \param[in] client TRXC client instance
 *  \param[in] rsp received message (rsp->type must be OSMO_TRXC_MT_RSP)
 *  \returns 0 on success; negative on error */
int osmo_trxc_client_rx_msg(struct osmo_trxc_client *client,
			    const struct osmo_trxc_msg *rsp)
{
	struct trxc_cmd_entry *e;
	bool flushed;
	int rc;

	if (rsp->type != OSMO_TRXC_MT_RSP) {
		LOGCL(client, LOGL_NOTICE, "Rx unexpected TRXC message '%s'\n",
		      osmo_trxc_msg_name(rsp));
		return -EINVAL;
	}

	LOGCL(client, LOGL_INFO, "Rx '%s'\n", osmo_trxc_msg_name(rsp));

	/* abort the retransmit timer */
	osmo_timer_del(&client->retrans_timer);

	if (llist_empty(&client->cmd_queue)) {
		/* a response from a retransmission, skip it */
		if (client->last_acked != NULL && cmd_matches_rsp(client->last_acked, rsp)) {
			LOGCL(client, LOGL_NOTICE, "Discarding duplicate response '%s'\n",
			      osmo_trxc_msg_name(rsp));
			return 0;
		}
		LOGCL(client, LOGL_NOTICE, "Rx response without a pending command\n");
		return -ENOENT;
	}

	e = llist_first_entry(&client->cmd_queue, struct trxc_cmd_entry, list);

	if (!cmd_matches_rsp(e, rsp)) {
		/* a response from a retransmission, skip it */
		if (client->last_acked != NULL && cmd_matches_rsp(client->last_acked, rsp)) {
			LOGCL(client, LOGL_NOTICE, "Discarding duplicate response '%s'\n",
			      osmo_trxc_msg_name(rsp));
			/* the command in flight still awaits its response */
			osmo_timer_schedule(&client->retrans_timer, client->retrans_sec, 0);
			return 0;
		}

		LOGCL(client, (e->flags & OSMO_TRXC_F_CRITICAL) ? LOGL_FATAL : LOGL_NOTICE,
		      "Response '%s' does not match pending '" CMD_NAME_FMT "'\n",
		      osmo_trxc_msg_name(rsp), CMD_NAME_ARGS(e));

		if (e->flags & OSMO_TRXC_F_CRITICAL)
			return trxc_client_fatal(client, rsp);

		/* We may get 'RSP ERR 1' for non-critical commands not
		 * supported by the transceiver.  Deliver such responses to
		 * the call-back of the command in flight, so that it can
		 * implement a fallback (see the SETFORMAT negotiation). */
	}

	/* the transceiver is responsive (again) */
	e->n_retrans = 0;

	client->in_rx = true;
	if (e->rsp_cb != NULL)
		rc = e->rsp_cb(client, rsp, e->cb_data);
	else
		rc = trxc_client_default_rsp_cb(client, e, rsp);
	flushed = client->flushed_in_rx;
	client->flushed_in_rx = false;
	client->in_rx = false;

	if (rc < 0)
		return trxc_client_fatal(client, rsp);

	/* the call-back requested a re-transmission in rc seconds */
	if (rc > 0) {
		/* the queue may have been flushed by the call-back */
		if (!flushed && !llist_empty(&client->cmd_queue))
			osmo_timer_schedule(&client->retrans_timer, rc, 0);
		return 0;
	}

	if (!flushed) {
		/* dequeue the command, keep it for duplicate-RSP filtering */
		llist_del(&e->list);
		talloc_free(client->last_acked);
		client->last_acked = e;
	} /* else: e was freed by osmo_trxc_client_flush(), do not access it */

	/* transmit the next command waiting in the queue */
	trxc_client_send_next(client);

	return 0;
}

/*! Feed a datagram received on the ctrl socket into the engine.
 *
 *  A convenience wrapper around osmo_trxc_client_rx_msg() for applications
 *  managing the TRXC socket themselves: parses the given datagram first.
 *
 *  \param[in] client TRXC client instance
 *  \param[in] buf received datagram (not necessarily zero-terminated)
 *  \param[in] len length of the datagram
 *  \returns 0 on success; negative on error */
int osmo_trxc_client_rx(struct osmo_trxc_client *client, const char *buf, size_t len)
{
	struct osmo_trxc_msg rsp;
	int rc;

	rc = osmo_trxc_msg_parse(&rsp, buf, len);
	if (rc < 0) {
		LOGCL(client, LOGL_NOTICE, "Rx malformed TRXC message (rc=%d)\n", rc);
		return rc;
	}

	return osmo_trxc_client_rx_msg(client, &rsp);
}

/***********************************************************************
 * TRXD PDU version negotiation (SETFORMAT)
 ***********************************************************************/

static int setformat_rsp_cb(struct osmo_trxc_client *client,
			    const struct osmo_trxc_msg *rsp, void *cb_data)
{
	/* Old transceivers reject 'SETFORMAT' with 'RSP ERR 1' */
	if (strcmp(rsp->cmd, OSMO_TRXC_CMD_SETFORMAT) != 0) {
		LOGCL(client, LOGL_NOTICE, "Transceiver rejected the format "
		      "negotiation command, using TRXD PDU version 0\n");
		if (client->setformat_cb != NULL)
			client->setformat_cb(client, 0, client->setformat_cb_data);
		return 0;
	}

	/* Status shall indicate a proper version supported by the transceiver */
	if (rsp->status < 0 || rsp->status > client->setformat_ver_req) {
		LOGCL(client, LOGL_ERROR, "Transceiver indicated an out of range "
		      "TRXD PDU version %d (requested %u)\n",
		      rsp->status, client->setformat_ver_req);
		return -EINVAL;
	}

	LOGCL(client, LOGL_INFO, "Using TRXD PDU version %d\n", rsp->status);
	if (client->setformat_cb != NULL)
		client->setformat_cb(client, rsp->status, client->setformat_cb_data);

	return 0;
}

/*! Negotiate the TRXD PDU version with the transceiver (SETFORMAT).
 *
 *  If the transceiver does not support the format negotiation at all,
 *  it rejects the command with 'RSP ERR 1' and version 0 is assumed.
 *  If the requested version is not supported by the transceiver, the
 *  status code of the response indicates a preferred lower version.
 *
 *  \param[in] client TRXC client instance
 *  \param[in] ver_max the maximum (desired) TRXD PDU version
 *  \param[in] cb call-back invoked with the negotiated version
 *  \param[in] cb_data opaque data for the call-back
 *  \returns 0 on success; negative on error */
int osmo_trxc_client_negotiate_format(struct osmo_trxc_client *client,
				      uint8_t ver_max,
				      osmo_trxc_setformat_cb *cb, void *cb_data)
{
	client->setformat_ver_req = ver_max;
	client->setformat_cb = cb;
	client->setformat_cb_data = cb_data;

	LOGCL(client, LOGL_INFO, "Requesting TRXD PDU version %u\n", ver_max);

	return osmo_trxc_client_send_cmd(client, OSMO_TRXC_F_MATCH_PARAMS,
					 &setformat_rsp_cb, NULL,
					 OSMO_TRXC_CMD_SETFORMAT, "%u", ver_max);
}
