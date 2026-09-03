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

#include <osmocom/core/utils.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/linuxlist.h>

#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/gsm0502.h>

#include <osmocom/trx/ep.h>

#include <osmocom/proxy/proxy.h>
#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/clck_gen.h>
#include <osmocom/proxy/path_sim.h>
#include <osmocom/proxy/logging.h>

struct proxy_trx *proxy_trx_alloc(struct proxy_ctx *proxy, const char *name)
{
	const unsigned int num_chans = 1;
	struct proxy_trx *trx;

	trx = talloc_zero(proxy, struct proxy_trx);
	if (trx == NULL)
		return NULL;

	trx->name = talloc_strdup(trx, name);
	trx->tx_power = PROXY_TRX_DEFAULT_TX_POWER;
	trx->ep = osmo_trx_ep_alloc(trx, num_chans);
	if (trx->ep == NULL) {
		talloc_free(trx);
		return NULL;
	}
	proxy_trx_set_num_chans(trx, num_chans);

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
	proxy_trx_set_power(trx, false);
	osmo_trx_ep_free(trx->ep);
	llist_del(&trx->list);
	talloc_free(trx);
}

/*! Update the endpoint's power state;
 * (de)registers it with the TDMA clock generator as appropriate. */
void proxy_trx_set_power(struct proxy_trx *trx, bool on)
{
	if (trx->powered == on)
		return;

	LOGP_TRX(trx, DTRXC, LOGL_INFO,
		 "Power %s\n", on ? "on" : "off");

	trx->powered = on;
	clck_gen_trx_list_updated();
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

	if (rc == 0) { /* first successful open: num_chans is now fixed */
		trx->chans = talloc_zero_array(trx, struct proxy_trx_chan, trx->num_chans);
		OSMO_ASSERT(trx->chans != NULL);
		for (unsigned int i = 0; i < trx->num_chans; i++) {
			path_sim_state_reset(&trx->chans[i].path_sim, trx->tx_power,
					     path_sim_cfg_get_nom_toa256(g_proxy_ctx->path_sim),
					     path_sim_cfg_get_nom_ci_cb(g_proxy_ctx->path_sim));
		}
	}

	return 0;
}

/*! Change the number of channels; must be called before proxy_trx_open(). */
int proxy_trx_set_num_chans(struct proxy_trx *trx, unsigned int num_chans)
{
	int rc;

	rc = osmo_trx_ep_set_num_chans(trx->ep, num_chans);
	if (rc)
		return rc;

	trx->num_chans = num_chans;
	return 0;
}

/*! Allocate a frequency hopping configuration; ma is copied. */
struct proxy_trx_fh *proxy_trx_fh_alloc(void *talloc_ctx, uint8_t hsn, uint8_t maio,
					const struct proxy_trx_fh_freq *ma, unsigned int ma_len)
{
	struct proxy_trx_fh *fh;

	if (ma_len == 0)
		return NULL;

	fh = talloc_zero(talloc_ctx, struct proxy_trx_fh);
	if (fh == NULL)
		return NULL;

	fh->ma = talloc_memdup(fh, ma, ma_len * sizeof(*ma));
	if (fh->ma == NULL) {
		talloc_free(fh);
		return NULL;
	}

	fh->hsn = hsn;
	fh->maio = maio;
	fh->ma_len = ma_len;

	return fh;
}

/*! Resolve the Rx/Tx frequencies to use for the given TDMA frame number. */
void proxy_trx_fh_resolve(const struct proxy_trx_fh *fh, uint32_t fn,
			  uint32_t *rx_freq, uint32_t *tx_freq)
{
	struct gsm_time gt;
	uint16_t mai;

	gsm_fn2gsmtime(&gt, fn);
	mai = gsm0502_hop_seq_gen(&gt, fh->hsn, fh->maio, fh->ma_len, NULL);

	if (rx_freq)
		*rx_freq = fh->ma[mai].rx_freq;
	if (tx_freq)
		*tx_freq = fh->ma[mai].tx_freq;
}
