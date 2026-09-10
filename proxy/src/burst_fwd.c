/*! \file src/burst_fwd.c
 * osmo-trx-proxy: forward Tx bursts by Rx/Tx frequency match. */

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

#include <string.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/linuxlist.h>

#include <osmocom/trx/ep.h>

#include <osmocom/proxy/proxy.h>
#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/path_sim.h>
#include <osmocom/proxy/burst_synch.h>
#include <osmocom/proxy/logging.h>

#define OSMO_TRXD_F_COMMON_MASK	( \
	OSMO_TRXD_F_NOPE_REQ	| \
	OSMO_TRXD_F_MOD_TYPE	| \
	OSMO_TRXD_F_TS_INFO	| \
	OSMO_TRXD_F_TRX_NUM	  \
	)

static void burst_fwd_to_chan(struct proxy_trx *src, unsigned int src_chan,
			      struct proxy_trx *dst, unsigned int dst_chan,
			      const struct osmo_trxd_burst_req *br)
{
	struct osmo_trxd_burst_ind bi = {
		.flags = br->flags & OSMO_TRXD_F_COMMON_MASK,
		.fn = br->fn,
		.tn = br->tn,
		.burst_len = br->burst_len,
		.mod = br->mod,
		.tsc_set = br->tsc_set,
		.tsc = br->tsc,
		.trx_num = br->trx_num,
	};

	/* TRXDv0/v1 BURST.req PDUs carry no MTS field at all, so the true
	 * modulation/TSC of the burst is unknown here: detect it instead of
	 * asserting a fixed, possibly wrong TSC downstream. */
	if (~br->flags & OSMO_TRXD_F_TS_INFO)
		burst_synch_detect(&bi, br);

	osmo_ubit2sbit(bi.burst, br->burst, br->burst_len);
	path_sim_apply(&bi, &dst->chans[dst_chan],
		       br, &src->chans[src_chan],
		       g_proxy_ctx->path_sim);

	osmo_trx_ep_send_burst_ind(dst->ep, dst_chan, &bi);
}

/*! Forward a Tx burst request to every powered-on endpoint/channel
 * whose Rx frequency matches the source channel's Tx frequency (resolved
 * per TDMA frame number if frequency hopping (SETFH) is configured). */
void osmo_trx_ep_rx_burst_req(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxd_burst_req *br)
{
	struct proxy_trx *src = osmo_trx_ep_get_priv(ep);
	struct proxy_trx *dst = NULL;
	uint32_t tx_freq;

	if (!src->powered) {
		LOGP_TRXCH(src, chan, DTRXD, LOGL_NOTICE,
			   "Rx BURST.req while not powered on, dropping\n");
		return;
	}

	if (src->chans[chan].fh != NULL)
		proxy_trx_fh_resolve(src->chans[chan].fh, br->fn, NULL, &tx_freq);
	else
		tx_freq = src->chans[chan].tx_freq;

	llist_for_each_entry(dst, &g_proxy_ctx->trx_list, list) {
		unsigned int dst_chan;

		if (dst == src || !dst->powered)
			continue;

		for (dst_chan = 0; dst_chan < dst->num_chans; dst_chan++) {
			struct proxy_trx_chan *dc = &dst->chans[dst_chan];
			uint32_t rx_freq;

			if (dc->fh != NULL)
				proxy_trx_fh_resolve(dc->fh, br->fn, &rx_freq, NULL);
			else
				rx_freq = dc->rx_freq;

			if (rx_freq != tx_freq)
				continue;
			burst_fwd_to_chan(src, chan, dst, dst_chan, br);
		}
	}
}
