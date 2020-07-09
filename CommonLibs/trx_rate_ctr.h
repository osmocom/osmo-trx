#pragma once

#include <osmocom/core/rate_ctr.h>
#include <osmocom/vty/command.h>

enum TrxCtr {
	TRX_CTR_DEV_RX_OVERRUNS,
	TRX_CTR_DEV_TX_UNDERRUNS,
	TRX_CTR_DEV_RX_DROP_EV,
	TRX_CTR_DEV_RX_DROP_SMPL,
	TRX_CTR_DEV_TX_DROP_EV,
	TRX_CTR_DEV_TX_DROP_SMPL,
	TRX_CTR_TRX_TX_STALE_BURSTS,
	TRX_CTR_TRX_TRXD_FN_REPEATED,
	TRX_CTR_TRX_TRXD_FN_OUTOFORDER,
	TRX_CTR_TRX_TRXD_FN_SKIPPED,
};

struct ctr_threshold {
	/*! Linked list of all counter groups in the system */
	struct llist_head list;
	enum rate_ctr_intv intv;
	enum TrxCtr ctr_id;
	uint32_t val;
};

extern const struct value_string rate_ctr_intv[];
extern const struct value_string trx_chan_ctr_names[];

struct trx_ctx;
void trx_rate_ctr_init(void *ctx, struct trx_ctx* trx_ctx);
void trx_rate_ctr_threshold_add(struct ctr_threshold *ctr);
int trx_rate_ctr_threshold_del(struct ctr_threshold *del_ctr);
void trx_rate_ctr_threshold_write_config(struct vty *vty, char *indent_prefix);
