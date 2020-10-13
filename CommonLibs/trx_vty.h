#pragma once

#include <osmocom/vty/command.h>

#include "config_defs.h"

extern struct vty_app_info g_vty_info;
extern const struct value_string clock_ref_names[];
extern const struct value_string filler_names[];

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

struct trx_ctx {
	struct trx_cfg cfg;
};

int trx_vty_init(struct trx_ctx* trx);
struct trx_ctx *vty_trx_ctx_alloc(void *talloc_ctx);
