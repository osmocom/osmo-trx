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
#include <osmocom/proxy/logging.h>

/* Placeholder measurement values, until path_sim.c computes them for real
 * (matches fake_trx.py's FakeTRX nominal defaults: no timing offset, and
 * -60 dBm = 50 dBm nominal Tx power - 0 dB Tx attenuation - 110 dB path loss). */
#define BURST_FWD_TOA256_DEFAULT	0
#define BURST_FWD_RSSI_DEFAULT		-60

#define OSMO_TRXD_F_COMMON_MASK	( \
	OSMO_TRXD_F_NOPE_REQ	| \
	OSMO_TRXD_F_MOD_TYPE	| \
	OSMO_TRXD_F_TS_INFO	| \
	OSMO_TRXD_F_TRX_NUM	  \
	)

static void burst_fwd_to_chan(struct proxy_trx *dst, unsigned int dst_chan,
			      const struct osmo_trxd_burst_req *br)
{
	struct osmo_trxd_burst_ind bi = {
		.flags = br->flags & OSMO_TRXD_F_COMMON_MASK,
		.fn = br->fn,
		.tn = br->tn,
		.toa256 = BURST_FWD_TOA256_DEFAULT,
		.rssi = BURST_FWD_RSSI_DEFAULT,
		.burst_len = br->burst_len,
		.mod = br->mod,
		.tsc_set = br->tsc_set,
		.tsc = br->tsc,
		.trx_num = br->trx_num,
	};

	osmo_ubit2sbit(bi.burst, br->burst, br->burst_len);

	osmo_trx_ep_send_burst_ind(dst->ep, dst_chan, &bi);
}

/*! Forward a Tx burst request to every powered-on endpoint/channel
 * whose Rx frequency matches the source channel's Tx frequency.
 * TODO: (no frequency hopping support yet, see SETFH in fake_trx.py). */
void osmo_trx_ep_rx_burst_req(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxd_burst_req *br)
{
	const struct proxy_trx *src = osmo_trx_ep_get_priv(ep);
	const uint32_t tx_freq = src->chans[chan].tx_freq;
	struct proxy_trx *dst;

	if (!src->powered) {
		LOGP_TRXCH(src, chan, DTRXD, LOGL_NOTICE,
			   "Rx BURST.req while not powered on, dropping\n");
		return;
	}

	llist_for_each_entry(dst, &g_proxy_ctx->trx_list, list) {
		unsigned int dst_chan;

		if (dst == src || !dst->powered)
			continue;

		for (dst_chan = 0; dst_chan < dst->num_chans; dst_chan++) {
			if (dst->chans[dst_chan].rx_freq != tx_freq)
				continue;
			burst_fwd_to_chan(dst, dst_chan, br);
		}
	}
}
