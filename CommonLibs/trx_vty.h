#pragma once

#include <osmocom/vty/command.h>

#include "config_defs.h"

extern struct vty_app_info g_vty_info;
extern const struct value_string filler_names[];

/* Maximum number of physical RF channels */
#define TRX_CHAN_MAX 8
/* Maximum number of carriers in multi-ARFCN mode */
#define TRX_MCHAN_MAX 3

/* Samples-per-symbol for downlink path
 *     4 - Uses precision modulator (more computation, less distortion)
 *     1 - Uses minimized modulator (less computation, more distortion)
 *
 *     Other values are invalid. Receive path (uplink) is always
 *     downsampled to 1 sps. Default to 4 sps for all cases.
 */
#define DEFAULT_TX_SPS		4

/*
 * Samples-per-symbol for uplink (receiver) path
 *     Do not modify this value. EDGE configures 4 sps automatically on
 *     B200/B210 devices only. Use of 4 sps on the receive path for other
 *     configurations is not supported.
 */
#define DEFAULT_RX_SPS		1

/* Default configuration parameters */
#define DEFAULT_TRX_PORT	5700
#define DEFAULT_TRX_IP		"127.0.0.1"
#define DEFAULT_CHANS		1

struct trx_ctx;

struct trx_chan {
	struct trx_ctx *trx; /* backpointer */
	unsigned int idx; /* channel index */
	char *rx_path;
	char *tx_path;
};

struct trx_ctx {
	struct {
		char *bind_addr;
		char *remote_addr;
		char *dev_args;
		unsigned int base_port;
		unsigned int tx_sps;
		unsigned int rx_sps;
		unsigned int rtsc;
		unsigned int rach_delay;
		enum ReferenceType clock_ref;
		enum FillerType filler;
		bool multi_arfcn;
		double offset;
		double rssi_offset;
		bool swap_channels;
		bool ext_rach;
		bool egprs;
		unsigned int sched_rr;
		unsigned int stack_size;
		unsigned int num_chans;
		struct trx_chan chans[TRX_CHAN_MAX];
	} cfg;
};

int trx_vty_init(struct trx_ctx* trx);
struct trx_ctx *vty_trx_ctx_alloc(void *talloc_ctx);
