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

struct osmo_trx_ep *osmo_trx_ep_alloc(void *ctx, unsigned int num_chans);
int osmo_trx_ep_open(struct osmo_trx_ep *ep);
void osmo_trx_ep_close(struct osmo_trx_ep *ep);
void osmo_trx_ep_free(struct osmo_trx_ep *ep);
bool osmo_trx_ep_is_open(const struct osmo_trx_ep *ep);
bool osmo_trx_ep_is_closing(const struct osmo_trx_ep *ep);

/*! Called once osmo_trx_ep_close() has fully completed (see
 *  osmo_trx_ep_set_closed_cb()) */
typedef void (*osmo_trx_ep_closed_cb_t)(struct osmo_trx_ep *ep);
void osmo_trx_ep_set_closed_cb(struct osmo_trx_ep *ep, osmo_trx_ep_closed_cb_t closed_cb);

void osmo_trx_ep_set_priv(struct osmo_trx_ep *ep, void *priv);
void *osmo_trx_ep_get_priv(const struct osmo_trx_ep *ep);
int osmo_trx_ep_set_name(struct osmo_trx_ep *ep, const char *fmt, ...);
const char *osmo_trx_ep_get_name(const struct osmo_trx_ep *ep);
void osmo_trx_ep_set_log_cat(struct osmo_trx_ep *ep, int log_cat);
int osmo_trx_ep_set_num_chans(struct osmo_trx_ep *ep, unsigned int num_chans);
unsigned int osmo_trx_ep_get_num_chans(const struct osmo_trx_ep *ep);

int osmo_trx_ep_set_mode(struct osmo_trx_ep *ep, enum osmo_trx_ep_mode mode);
enum osmo_trx_ep_mode osmo_trx_ep_get_mode(const struct osmo_trx_ep *ep);

/*! Local/remote IP address (copied); must be set before osmo_trx_ep_open() */
int osmo_trx_ep_set_laddr(struct osmo_trx_ep *ep, const char *addr);
const char *osmo_trx_ep_get_laddr(const struct osmo_trx_ep *ep);
int osmo_trx_ep_set_raddr(struct osmo_trx_ep *ep, const char *addr);
const char *osmo_trx_ep_get_raddr(const struct osmo_trx_ep *ep);

/*! Base UDP port (default: 5700).  All local and remote ports are
 * derived from it, given that the L1 side uses a fixed offset of
 * +100: the TRX side binds base + ofs and sends to base + 100 + ofs,
 * while the L1 side binds base + 100 + ofs and sends to base + ofs,
 * where ofs = 0 for the clock socket, and for each channel N:
 * ofs = 2 * N + 1 (ctrl), ofs = 2 * N + 2 (data). */
int osmo_trx_ep_set_base_port(struct osmo_trx_ep *ep, uint16_t base_port);
uint16_t osmo_trx_ep_get_base_port(const struct osmo_trx_ep *ep);

/*! Open the clock socket (base port + 0); default: false */
int osmo_trx_ep_set_clock_socket(struct osmo_trx_ep *ep, bool enable);
bool osmo_trx_ep_get_clock_socket(const struct osmo_trx_ep *ep);

/*! Unconnected ctrl socket accepting/replying to any peer; default: false */
int osmo_trx_ep_set_ctrl_promisc(struct osmo_trx_ep *ep, bool enable);
bool osmo_trx_ep_get_ctrl_promisc(const struct osmo_trx_ep *ep);

void osmo_trx_ep_set_pdu_batch(struct osmo_trx_ep *ep, bool enable);
bool osmo_trx_ep_get_pdu_batch(const struct osmo_trx_ep *ep);

/*! Per-channel TRXD PDU version in use (set after SETFORMAT negotiation) */
int osmo_trx_ep_set_pdu_ver(struct osmo_trx_ep *ep, unsigned int chan, uint8_t ver);
int osmo_trx_ep_get_pdu_ver(const struct osmo_trx_ep *ep, unsigned int chan);

/* TX paths (encode + transmit) */
int osmo_trx_ep_send_clck_ind(struct osmo_trx_ep *ep, uint32_t fn);
int osmo_trx_ep_send_ctrl_msg(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxc_msg *msg);
int osmo_trx_ep_send_burst_ind(struct osmo_trx_ep *ep, unsigned int chan,
			       const struct osmo_trxd_burst_ind *bi);
int osmo_trx_ep_send_burst_req(struct osmo_trx_ep *ep, unsigned int chan,
			       const struct osmo_trxd_burst_req *br);
int osmo_trx_ep_send_burst_fin(struct osmo_trx_ep *ep, unsigned int chan);
