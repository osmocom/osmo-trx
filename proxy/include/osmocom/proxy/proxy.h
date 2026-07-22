#pragma once

#include <stdbool.h>

#include <osmocom/core/linuxlist.h>

#define PROXY_DEFAULT_BIND_ADDR		"0.0.0.0"

/* Endpoints created when the config file defines none explicitly */
#define PROXY_DEFAULT_REMOTE_ADDR	"127.0.0.1"
#define PROXY_DEFAULT_BTS_NAME		"bts"
#define PROXY_DEFAULT_BTS_PORT		5700
#define PROXY_DEFAULT_MS_NAME		"ms"
#define PROXY_DEFAULT_MS_PORT		6700

struct proxy_ctx {
	char *bind_addr;
	struct llist_head trx_list; /*!< struct proxy_trx::list */
};

extern struct proxy_ctx *g_proxy_ctx;

struct proxy_ctx *proxy_ctx_alloc(void *talloc_ctx);
void proxy_ctx_free(struct proxy_ctx *proxy);
int proxy_ctx_add_defaults_if_empty(struct proxy_ctx *proxy);
