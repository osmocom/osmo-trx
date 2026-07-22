#pragma once

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/logging.h>

struct osmo_trx_ep;
struct proxy_ctx;

/*! One virtual transceiver endpoint */
struct proxy_trx {
	struct llist_head list;
	char *name;
	struct osmo_trx_ep *ep;
};

#define LOGP_TRX(trx, ss, level, fmt, args...) \
	LOGP(ss, level, "(trx=%s) " fmt, (trx)->name, ##args)

struct proxy_trx *proxy_trx_find(struct proxy_ctx *proxy, const char *name);
struct proxy_trx *proxy_trx_alloc(struct proxy_ctx *proxy, const char *name);
void proxy_trx_free(struct proxy_trx *trx);

int proxy_trx_open(struct proxy_trx *trx);
void proxy_trx_close(struct proxy_trx *trx);
