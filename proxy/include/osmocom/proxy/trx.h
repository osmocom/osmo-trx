#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/logging.h>

#include <osmocom/proxy/path_sim.h>

struct osmo_trx_ep;
struct proxy_ctx;

/*! Per-channel state: each channel is conceptually its own (child)
 * transceiver with an independent Rx/Tx frequency, sharing the endpoint's
 * power state and clock. */
struct proxy_trx_chan {
	uint32_t rx_freq;	/*!< Rx frequency in Hz, 0 if not (yet) tuned */
	uint32_t tx_freq;	/*!< Tx frequency in Hz, 0 if not (yet) tuned */
	bool rf_muted;		/*!< RFMUTE: force NOPE.ind on bursts this channel transmits */
	struct path_sim_state path_sim; /*!< RF path simulation state (path_sim.c) */
};

/*! One virtual transceiver endpoint */
struct proxy_trx {
	struct llist_head list;
	char *name;
	struct osmo_trx_ep *ep;
	bool powered;		/*!< POWERON/POWEROFF applies to all channels at once */
	unsigned int num_chans;	/*!< mirrors osmo_trx_ep_get_num_chans(ep) */
	struct proxy_trx_chan *chans; /*!< array of num_chans entries, allocated on open */
};

#define LOGP_TRX(trx, ss, level, fmt, args...) \
	LOGP(ss, level, "(trx=%s) " fmt, (trx)->name, ##args)

#define LOGP_TRXCH(trx, chan, ss, level, fmt, args...) \
	LOGP(ss, level, "(trx=%s, chan=%u) " fmt, (trx)->name, chan, ##args)

struct proxy_trx *proxy_trx_find(struct proxy_ctx *proxy, const char *name);
struct proxy_trx *proxy_trx_alloc(struct proxy_ctx *proxy, const char *name);
void proxy_trx_free(struct proxy_trx *trx);

int proxy_trx_open(struct proxy_trx *trx);
void proxy_trx_close(struct proxy_trx *trx);

int proxy_trx_set_num_chans(struct proxy_trx *trx, unsigned int num_chans);
void proxy_trx_set_power(struct proxy_trx *trx, bool on);
