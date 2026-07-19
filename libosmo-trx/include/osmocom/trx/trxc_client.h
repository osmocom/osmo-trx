/*! \file osmocom/trx/trxc_client.h
 * TRXC client command queue engine: queue, retransmission, RSP matching. */
#pragma once

#include <stdint.h>
#include <stddef.h>

#include <osmocom/trx/trxc.h>

/*! TRXC client engine, driving the ctrl connection towards a transceiver:
 * command queue with a single command in flight, retransmit timer,
 * consecutive-duplicate suppression, RSP<->CMD matching and duplicate-RSP
 * filtering.  This structure is opaque.
 *
 * The engine is transport-agnostic: it neither opens nor owns a socket.
 * The application transmits serialized messages in the tx_msg call-back
 * and feeds received datagrams into osmo_trxc_client_rx(). */
struct osmo_trxc_client;

/*! Response call-back, invoked when a response to a command is received.
 *  \param[in] client TRXC client instance
 *  \param[in] rsp received response message
 *  \param[in] cb_data opaque data passed to osmo_trxc_client_send_cmd()
 *  \returns 0 when done (the command gets dequeued);
 *	     N > 0 to re-send the same command after N seconds;
 *	     negative to indicate a fatal error (like a NACKed critical
 *	     command, ends up in the fatal_error call-back) */
typedef int osmo_trxc_client_rsp_cb(struct osmo_trxc_client *client,
				    const struct osmo_trxc_msg *rsp,
				    void *cb_data);

struct osmo_trxc_client_ops {
	/*! transmit a TRXC message (mandatory).
	 *  E.g. osmo_trx_ep_send_ctrl_msg() for osmo_trx_ep users, or
	 *  osmo_trxc_msg_build() + write() on a self-managed ctrl socket. */
	int (*tx_msg)(struct osmo_trxc_client *client, const struct osmo_trxc_msg *msg);
	/*! a critical command definitively failed (optional; default: log).
	 *  \param[in] rsp the offending response (may be NULL) */
	void (*fatal_error)(struct osmo_trxc_client *client,
			    const struct osmo_trxc_msg *rsp);
	/*! opaque application-private data (see osmo_trxc_client_get_priv()) */
	void *priv;
};

struct osmo_trxc_client *osmo_trxc_client_alloc(void *ctx,
						const struct osmo_trxc_client_ops *ops);
void osmo_trxc_client_free(struct osmo_trxc_client *client);
void *osmo_trxc_client_get_priv(const struct osmo_trxc_client *client);
int osmo_trxc_client_set_name(struct osmo_trxc_client *client, const char *fmt, ...);
void osmo_trxc_client_set_log_cat(struct osmo_trxc_client *client, int log_cat);
void osmo_trxc_client_set_retrans(struct osmo_trxc_client *client, unsigned int sec);
void osmo_trxc_client_set_max_retrans(struct osmo_trxc_client *client, unsigned int n);

/*! escalate to the fatal_error call-back on NACK */
#define OSMO_TRXC_F_CRITICAL		(1 << 0)
/*! RSP params must echo CMD params (e.g. SETSLOT, SETFORMAT) */
#define OSMO_TRXC_F_MATCH_PARAMS	(1 << 1)

int osmo_trxc_client_send_cmd(struct osmo_trxc_client *client, uint32_t flags,
			      osmo_trxc_client_rsp_cb *cb, void *cb_data,
			      const char *cmd, const char *fmt, ...);
void osmo_trxc_client_flush(struct osmo_trxc_client *client);

int osmo_trxc_client_rx_msg(struct osmo_trxc_client *client,
			    const struct osmo_trxc_msg *rsp);
int osmo_trxc_client_rx(struct osmo_trxc_client *client, const char *buf, size_t len);

/*! TRXD PDU version negotiation result call-back.
 *  \param[in] ver_use the negotiated version to be used */
typedef void osmo_trxc_setformat_cb(struct osmo_trxc_client *client,
				    uint8_t ver_use, void *cb_data);
int osmo_trxc_client_negotiate_format(struct osmo_trxc_client *client,
				      uint8_t ver_max,
				      osmo_trxc_setformat_cb *cb, void *cb_data);

/* Convenience wrappers for the common command set (thin, optional) */
static inline int osmo_trxc_client_poweron(struct osmo_trxc_client *client,
					   osmo_trxc_client_rsp_cb *cb, void *cb_data)
{
	return osmo_trxc_client_send_cmd(client, OSMO_TRXC_F_CRITICAL, cb, cb_data,
					 OSMO_TRXC_CMD_POWERON, NULL);
}
static inline int osmo_trxc_client_poweroff(struct osmo_trxc_client *client,
					    osmo_trxc_client_rsp_cb *cb, void *cb_data)
{
	return osmo_trxc_client_send_cmd(client, OSMO_TRXC_F_CRITICAL, cb, cb_data,
					 OSMO_TRXC_CMD_POWEROFF, NULL);
}
static inline int osmo_trxc_client_rxtune(struct osmo_trxc_client *client,
					  unsigned int freq_khz,
					  osmo_trxc_client_rsp_cb *cb, void *cb_data)
{
	return osmo_trxc_client_send_cmd(client, OSMO_TRXC_F_CRITICAL, cb, cb_data,
					 OSMO_TRXC_CMD_RXTUNE, "%u", freq_khz);
}
static inline int osmo_trxc_client_txtune(struct osmo_trxc_client *client,
					  unsigned int freq_khz,
					  osmo_trxc_client_rsp_cb *cb, void *cb_data)
{
	return osmo_trxc_client_send_cmd(client, OSMO_TRXC_F_CRITICAL, cb, cb_data,
					 OSMO_TRXC_CMD_TXTUNE, "%u", freq_khz);
}
