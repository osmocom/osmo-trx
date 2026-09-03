#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/logging.h>

#include <osmocom/trx/trxc.h>

#include <osmocom/proxy/path_sim.h>

struct osmo_trx_ep;
struct proxy_ctx;

/*! dBm, nominal Tx power (per endpoint) */
#define PROXY_TRX_DEFAULT_TX_POWER	50

/*! TDMA timeslots per GSM frame */
#define PROXY_TRX_NUM_TS	8

/*! Per-timeslot config received via SETSLOT. */
struct proxy_trx_ts {
	struct osmo_trxc_setslot cfg;
	bool valid;		/*!< has SETSLOT been received for this TS? */
};

/*! One Mobile Allocation entry: the Rx/Tx frequency pair for one ARFCN. */
struct proxy_trx_fh_freq {
	uint32_t rx_freq;	/*!< Rx frequency in Hz */
	uint32_t tx_freq;	/*!< Tx frequency in Hz */
};

/*! Synthesizer frequency hopping parameters (SETFH), per 3GPP TS 45.002:
 * HSN, MAIO and the Mobile Allocation (list of Rx/Tx frequency pairs). */
struct proxy_trx_fh {
	uint8_t hsn;
	uint8_t maio;
	unsigned int ma_len;
	struct proxy_trx_fh_freq *ma; /*!< talloc array of ma_len entries */
};

/*! Per-channel state: each channel is conceptually its own (child)
 * transceiver with an independent Rx/Tx frequency, sharing the endpoint's
 * power state and clock. */
struct proxy_trx_chan {
	uint32_t rx_freq;	/*!< Rx frequency in Hz, 0 if not (yet) tuned */
	uint32_t tx_freq;	/*!< Tx frequency in Hz, 0 if not (yet) tuned */
	bool rf_muted;		/*!< RFMUTE: force NOPE.ind on bursts this channel transmits */
	struct path_sim_state path_sim; /*!< RF path simulation state (path_sim.c) */
	struct proxy_trx_ts ts[PROXY_TRX_NUM_TS]; /*!< per-timeslot config (SETSLOT) */
	struct proxy_trx_fh *fh; /*!< frequency hopping config (SETFH), NULL if disabled */
};

/*! One virtual transceiver endpoint */
struct proxy_trx {
	struct llist_head list;
	char *name;
	struct osmo_trx_ep *ep;
	bool powered;		/*!< POWERON/POWEROFF applies to all channels at once */
	int tx_power;		/*!< dBm, nominal Tx power (path_sim_state::tx_power) */
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

struct proxy_trx_fh *proxy_trx_fh_alloc(void *talloc_ctx, uint8_t hsn, uint8_t maio,
					const struct proxy_trx_fh_freq *ma, unsigned int ma_len);
void proxy_trx_fh_resolve(const struct proxy_trx_fh *fh, uint32_t fn,
			  uint32_t *rx_freq, uint32_t *tx_freq);
