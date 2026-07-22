/*! \file osmocom/trx/ep.h
 * TRX protocol socket endpoint management (clock/ctrl/data UDP sockets). */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <osmocom/trx/trxc.h>
#include <osmocom/trx/trxd.h>

/*! One TRX protocol endpoint: an (optional) clock socket + N channels,
 * each channel being a ctrl + data UDP socket pair (osmo_io based).
 * Role-neutral: serves the L1 side (osmo-bts), the TRX side (osmo-trx)
 * and a bridge (fake_trx) alike. */
struct osmo_trx_ep;

/*! Endpoint mode: which side of the TRX protocol this endpoint implements.
 * The mode determines how datagrams received on the data sockets are
 * parsed: the wire format alone does not identify the direction. */
enum osmo_trx_ep_mode {
	/*! L1 side (e.g. osmo-bts): Rx BURST.ind, Tx BURST.req */
	OSMO_TRX_EP_MODE_L1,
	/*! TRX side (e.g. osmo-trx): Rx BURST.req, Tx BURST.ind */
	OSMO_TRX_EP_MODE_TRX,
};

struct osmo_trx_ep_cfg {
	enum osmo_trx_ep_mode mode;
	const char *laddr, *raddr;	/*!< local/remote IP address */
	/*! base UDP port (default: 5700).  All local and remote ports are
	 * derived from it, given that the L1 side uses a fixed offset of
	 * +100: the TRX side binds base + ofs and sends to base + 100 + ofs,
	 * while the L1 side binds base + 100 + ofs and sends to base + ofs,
	 * where ofs = 0 for the clock socket, and for each channel N:
	 * ofs = 2 * N + 1 (ctrl), ofs = 2 * N + 2 (data). */
	uint16_t base_port;
	unsigned int num_chans;
	bool clock_socket;	/*!< open the clock socket (base port + 0) */
};

/* Decoded RX handlers: to avoid indirect calls on the hot path, these are
 * plain function prototypes the API user implements (link-time binding,
 * like bts_model_*() in osmo-bts), not function pointers.  The library
 * provides weak default stubs (logging an error), so an application only
 * needs to implement the handlers for the directions it consumes. */

/*! Handle a received clock indication ("IND CLOCK <fn>").
 *  Called for datagrams received on the clock socket (if configured).
 *  \param[in] ep TRX endpoint instance
 *  \param[in] fn indicated TDMA frame number (< GSM_TDMA_HYPERFRAME) */
extern void osmo_trx_ep_rx_clck_ind(struct osmo_trx_ep *ep, uint32_t fn);

/*! Handle a received TRXC message (CMD on the TRX side, RSP on the L1 side).
 *  Called for datagrams received on the per-channel ctrl socket.  An L1 side
 *  application would typically feed responses into the TRXC client engine
 *  (see osmo_trxc_client_rx()).
 *  \param[in] ep TRX endpoint instance
 *  \param[in] chan channel index (0 .. num_chans - 1)
 *  \param[in] msg parsed TRXC message */
extern void osmo_trx_ep_rx_ctrl_msg(struct osmo_trx_ep *ep, unsigned int chan,
				    const struct osmo_trxc_msg *msg);

/*! Handle a received burst indication (OSMO_TRX_EP_MODE_L1 only).
 *  Called for each PDU parsed from a datagram received on the per-channel
 *  data socket, also for each PDU of a TRXDv2 batch.
 *  \param[in] ep TRX endpoint instance
 *  \param[in] chan channel index (0 .. num_chans - 1)
 *  \param[in] bi parsed burst indication */
extern void osmo_trx_ep_rx_burst_ind(struct osmo_trx_ep *ep, unsigned int chan,
				     const struct osmo_trxd_burst_ind *bi);

/*! Handle a received burst transmit request (OSMO_TRX_EP_MODE_TRX only).
 *  Called for each PDU parsed from a datagram received on the per-channel
 *  data socket, also for each PDU of a TRXDv2 batch.
 *  \param[in] ep TRX endpoint instance
 *  \param[in] chan channel index (0 .. num_chans - 1)
 *  \param[in] br parsed burst transmit request */
extern void osmo_trx_ep_rx_burst_req(struct osmo_trx_ep *ep, unsigned int chan,
				     const struct osmo_trxd_burst_req *br);

struct osmo_trx_ep *osmo_trx_ep_alloc(void *ctx, const struct osmo_trx_ep_cfg *cfg,
				      void *priv);
int osmo_trx_ep_open(struct osmo_trx_ep *ep);
void osmo_trx_ep_close(struct osmo_trx_ep *ep);
void osmo_trx_ep_free(struct osmo_trx_ep *ep);
void *osmo_trx_ep_get_priv(const struct osmo_trx_ep *ep);
int osmo_trx_ep_set_name(struct osmo_trx_ep *ep, const char *fmt, ...);
void osmo_trx_ep_set_log_cat(struct osmo_trx_ep *ep, int log_cat);

/*! per-channel TRXD PDU version in use (set after SETFORMAT negotiation) */
void osmo_trx_ep_set_pdu_ver(struct osmo_trx_ep *ep, unsigned int chan, uint8_t ver);

/* TX paths (encode + transmit) */
int osmo_trx_ep_send_clck_ind(struct osmo_trx_ep *ep, uint32_t fn);
int osmo_trx_ep_send_ctrl_msg(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxc_msg *msg);
int osmo_trx_ep_send_burst_ind(struct osmo_trx_ep *ep, unsigned int chan,
			       const struct osmo_trxd_burst_ind *bi);
/*! br == NULL acts as the batching breaker (flush) for TRXDv2 */
int osmo_trx_ep_send_burst_req(struct osmo_trx_ep *ep, unsigned int chan,
			       const struct osmo_trxd_burst_req *br);
