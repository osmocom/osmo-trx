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
#include <osmocom/core/talloc.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/core/msgb.h>

#include <osmocom/gsm/gsm0502.h>

#include <osmocom/trx/ep.h>

#include <osmocom/proxy/proxy.h>
#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/path_sim.h>
#include <osmocom/proxy/burst_synch.h>
#include <osmocom/proxy/burst_fwd.h>
#include <osmocom/proxy/logging.h>

#define OSMO_TRXD_F_COMMON_MASK	( \
	OSMO_TRXD_F_NOPE_REQ	| \
	OSMO_TRXD_F_MOD_TYPE	| \
	OSMO_TRXD_F_TS_INFO	| \
	OSMO_TRXD_F_TRX_NUM	  \
	)

/*! The BURST.req copy wrapped in a queued struct msgb
 * (see struct proxy_trx_chan::tx_burst_queue). */
static inline struct osmo_trxd_burst_req *msgb_burst_req(const struct msgb *msg)
{
	return (struct osmo_trxd_burst_req *)msgb_data(msg);
}

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

	/* Stage the burst rather than sending it right away: dispatch sends
	 * every dst channel's Rx indications in ascending tn order once all
	 * sources have been processed for this TDMA frame tick (see
	 * burst_fwd_dispatch()). */
	dst->chans[dst_chan].ts[br->tn].bi = bi;
}

/*! Forward a Tx burst request to every powered-on endpoint/channel
 * whose Rx frequency matches the source channel's Tx frequency (resolved
 * per TDMA frame number if frequency hopping (SETFH) is configured). */
static void burst_fwd_burst_req(struct proxy_trx *src, unsigned int chan,
				const struct osmo_trxd_burst_req *br)
{
	struct proxy_trx *dst = NULL;
	uint32_t tx_freq;

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

/*! Enqueue a Tx burst request for later dispatch by the TDMA clock
 * generator: real transceivers typically receive bursts a few frames ahead
 * of their actual air time, so forwarding must happen at the matching TDMA
 * frame tick, not immediately upon receipt (see burst_fwd_dispatch()). */
void osmo_trx_ep_rx_burst_req(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxd_burst_req *br)
{
	struct proxy_trx *src = osmo_trx_ep_get_priv(ep);
	struct proxy_trx_chan *c = &src->chans[chan];
	struct msgb *msg, *cur;

	if (!src->powered) {
		LOGP_TRXCH(src, chan, DTRXD, LOGL_NOTICE,
			   "Rx BURST.req while not powered on, dropping\n");
		return;
	}

	msg = msgb_alloc_c(src, sizeof(*br), "trxd_burst_req");
	OSMO_ASSERT(msg != NULL);
	memcpy(msgb_put(msg, sizeof(*br)), br, sizeof(*br));

	/* The queue is kept sorted by TDMA FN (ascending) so that dispatch,
	 * which is timing critical, only ever needs to look at the head of
	 * the queue instead of scanning it in full on every TDMA frame tick.
	 * Bursts normally arrive already in ascending FN order (a few frames
	 * ahead of their air time), so a new entry almost always belongs at
	 * the tail: scan backwards from there so the common case is O(1). */
	llist_for_each_entry_reverse(cur, &c->tx_burst_queue, list) {
		if (gsm0502_fncmp(msgb_burst_req(cur)->fn, br->fn) <= 0) {
			llist_add(&msg->list, &cur->list);
			return;
		}
	}
	llist_add(&msg->list, &c->tx_burst_queue);
}

/*! Reset every powered channel's per-tn staging area to NOPE.ind for the
 * given TDMA frame number, so real forwarded bursts only need to overwrite
 * the entries they land on (see burst_fwd_to_chan()). */
static void burst_fwd_reset_bi(uint32_t fn)
{
	struct proxy_trx *trx;

	llist_for_each_entry(trx, &g_proxy_ctx->trx_list, list) {
		if (!trx->powered)
			continue;

		for (unsigned int chan = 0; chan < trx->num_chans; chan++) {
			struct proxy_trx_chan *c = &trx->chans[chan];

			for (unsigned int tn = 0; tn < PROXY_TRX_NUM_TS; tn++) {
				c->ts[tn].bi = (struct osmo_trxd_burst_ind){ .fn = fn, .tn = tn };
				path_sim_fill_nope(&c->ts[tn].bi, g_proxy_ctx->path_sim);
			}
		}
	}
}

/*! Send every powered channel's staged Rx indications (see
 * burst_fwd_reset_bi(), burst_fwd_to_chan()) for this TDMA frame tick,
 * in ascending tn order. */
static void burst_fwd_send_bi(void)
{
	struct proxy_trx *trx;

	llist_for_each_entry(trx, &g_proxy_ctx->trx_list, list) {
		if (!trx->powered)
			continue;

		for (unsigned int chan = 0; chan < trx->num_chans; chan++) {
			struct proxy_trx_chan *c = &trx->chans[chan];

			for (unsigned int tn = 0; tn < PROXY_TRX_NUM_TS; tn++) {
				struct proxy_trx_ts *ts = &c->ts[tn];
				struct osmo_trxd_burst_ind *bi = &ts->bi;

				/* unconfigured (no SETSLOT) timeslot: send nothing at all */
				if (!ts->valid)
					continue;

				/* TRXDv0 has no MTS field to carry the NOPE.ind flag in */
				if ((bi->flags & OSMO_TRXD_F_NOPE_IND) &&
				    osmo_trx_ep_get_pdu_ver(trx->ep, chan) < 1)
					continue;

				osmo_trx_ep_send_burst_ind(trx->ep, chan, bi);
			}

			/* flush batched PDUs (no-op below TRXDv2) */
			osmo_trx_ep_send_burst_fin(trx->ep, chan);
		}
	}
}

/*! Dispatch every queued BURST.req whose TDMA frame number is due:
 * forward those with fn == fn, drop those with fn < fn (arrived too late),
 * and leave the rest queued. */
void burst_fwd_dispatch(uint32_t fn)
{
	struct proxy_trx *trx;

	/* Three passes over this TDMA frame tick:
	 * 1. pre-fill every dst channel's Rx staging area with NOPE.ind; */
	burst_fwd_reset_bi(fn);

	/* 2. dispatch the per-channel Tx queues, overwriting the staged
	 *    NOPE.ind of whatever real bursts land; */
	llist_for_each_entry(trx, &g_proxy_ctx->trx_list, list) {
		if (!trx->powered)
			continue;

		for (unsigned int chan = 0; chan < trx->num_chans; chan++) {
			struct proxy_trx_chan *c = &trx->chans[chan];
			struct msgb *msg, *msg2;

			/* tx_burst_queue is sorted by fn (ascending), so it's
			 * enough to consume its head and stop at the first
			 * entry that is not yet due. */
			llist_for_each_entry_safe(msg, msg2, &c->tx_burst_queue, list) {
				struct osmo_trxd_burst_req *br = msgb_burst_req(msg);
				int rc = gsm0502_fncmp(br->fn, fn);

				if (rc > 0) /* br->fn is still ahead of fn */
					break;

				llist_del(&msg->list);

				if (OSMO_UNLIKELY(rc < 0)) { /* br->fn is behind fn */
					LOGP_TRXCH(trx, chan, DTRXD, LOGL_ERROR,
						   "Rx BURST.req for fn=%u too late (now fn=%u), "
						   "dropping\n", br->fn, fn);
				} else {
					burst_fwd_burst_req(trx, chan, br);
				}

				msgb_free(msg);
			}
		}
	}

	/* 3. send every staged Rx indication out to L1. */
	burst_fwd_send_bi();
}
