/*! \file src/proxy.c
 * osmo-trx-proxy: top-level daemon context (struct proxy_ctx). */

/*
 * (C) 2026 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 * Author: Vadim Yanitskiy <vyanitskiy@sysmocom.de>
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/linuxlist.h>

#include <osmocom/trx/ep.h>

#include <osmocom/proxy/proxy.h>
#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/logging.h>

struct proxy_ctx *g_proxy_ctx = NULL;

struct proxy_ctx *proxy_ctx_alloc(void *talloc_ctx)
{
	struct proxy_ctx *proxy;

	proxy = talloc_zero(talloc_ctx, struct proxy_ctx);
	if (proxy == NULL)
		return NULL;

	proxy->bind_addr = talloc_strdup(proxy, PROXY_DEFAULT_BIND_ADDR);
	INIT_LLIST_HEAD(&proxy->trx_list);

	return proxy;
}

void proxy_ctx_free(struct proxy_ctx *proxy)
{
	struct proxy_trx *trx, *trx2;

	if (!proxy)
		return;

	llist_for_each_entry_safe(trx, trx2, &proxy->trx_list, list) {
		proxy_trx_close(trx);
		proxy_trx_free(trx);
	}

	talloc_free(proxy);
}

/*! Populate the config with the default BTS/MS endpoints (base ports 5700
 * and 6700) and open them, iff the config file did not declare any
 * endpoint explicitly.
 * \returns 0 on success; negative on error (see proxy_trx_open()) */
int proxy_ctx_add_defaults_if_empty(struct proxy_ctx *proxy)
{
	struct proxy_trx *trx;
	int rc;

	if (!llist_empty(&proxy->trx_list))
		return 0;

	LOGP(DPROXY, LOGL_NOTICE,
	     "No endpoints configured, creating defaults ('%s' and '%s')\n",
	     PROXY_DEFAULT_BTS_NAME, PROXY_DEFAULT_MS_NAME);

	trx = proxy_trx_alloc(proxy, PROXY_DEFAULT_BTS_NAME);
	osmo_trx_ep_set_raddr(trx->ep, PROXY_DEFAULT_REMOTE_ADDR);
	osmo_trx_ep_set_base_port(trx->ep, PROXY_DEFAULT_BTS_PORT);
	rc = proxy_trx_open(trx);
	if (rc != 0)
		return rc;

	trx = proxy_trx_alloc(proxy, PROXY_DEFAULT_MS_NAME);
	osmo_trx_ep_set_raddr(trx->ep, PROXY_DEFAULT_REMOTE_ADDR);
	osmo_trx_ep_set_base_port(trx->ep, PROXY_DEFAULT_MS_PORT);
	/* MS (trxcon) side has no use for TRX clock indications */
	osmo_trx_ep_set_clock_socket(trx->ep, false);
	rc = proxy_trx_open(trx);
	if (rc != 0)
		return rc;

	return 0;
}
