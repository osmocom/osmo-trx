/*! \file src/trx.c
 * osmo-trx-proxy: virtual transceiver management. */

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
#include <string.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/linuxlist.h>

#include <osmocom/trx/ep.h>

#include <osmocom/proxy/proxy.h>
#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/logging.h>

struct proxy_trx *proxy_trx_alloc(struct proxy_ctx *proxy, const char *name)
{
	struct proxy_trx *trx;

	trx = talloc_zero(proxy, struct proxy_trx);
	if (trx == NULL)
		return NULL;

	trx->name = talloc_strdup(trx, name);
	trx->ep = osmo_trx_ep_alloc(trx, 1);
	if (trx->ep == NULL) {
		talloc_free(trx);
		return NULL;
	}

	osmo_trx_ep_set_priv(trx->ep, trx);
	osmo_trx_ep_set_name(trx->ep, "%s", name);
	osmo_trx_ep_set_log_cat(trx->ep, DPROXY);
	osmo_trx_ep_set_mode(trx->ep, OSMO_TRX_EP_MODE_TRX);
	osmo_trx_ep_set_laddr(trx->ep, proxy->bind_addr);
	osmo_trx_ep_set_clock_socket(trx->ep, true);
	osmo_trx_ep_set_ctrl_promisc(trx->ep, true);

	llist_add_tail(&trx->list, &proxy->trx_list);

	return trx;
}

struct proxy_trx *proxy_trx_find(struct proxy_ctx *proxy, const char *name)
{
	struct proxy_trx *trx;

	llist_for_each_entry(trx, &proxy->trx_list, list) {
		if (strcmp(trx->name, name) == 0)
			return trx;
	}

	return NULL;
}

void proxy_trx_close(struct proxy_trx *trx)
{
	if (!trx)
		return;
	osmo_trx_ep_close(trx->ep);
}

void proxy_trx_free(struct proxy_trx *trx)
{
	if (!trx)
		return;
	osmo_trx_ep_free(trx->ep);
	llist_del(&trx->list);
	talloc_free(trx);
}

int proxy_trx_open(struct proxy_trx *trx)
{
	int rc;

	rc = osmo_trx_ep_open(trx->ep);
	if (rc < 0 && rc != -EALREADY) {
		LOGP_TRX(trx, DPROXY, LOGL_ERROR,
			 "Failed to open TRX endpoint %s:%u\n",
			 osmo_trx_ep_get_raddr(trx->ep),
			 osmo_trx_ep_get_base_port(trx->ep));
		return rc;
	}

	return 0;
}
