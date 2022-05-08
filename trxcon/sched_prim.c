/*
 * OsmocomBB <-> SDR connection bridge
 * TDMA scheduler: primitive management
 *
 * (C) 2017 by Vadim Yanitskiy <axilirator@gmail.com>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <talloc.h>

#include <osmocom/core/msgb.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/linuxlist.h>

#include <osmocom/gsm/protocol/gsm_04_08.h>

#include "scheduler.h"
#include "sched_trx.h"
#include "trx_if.h"
#include "logging.h"

/**
 * Initializes a new primitive by allocating memory
 * and filling some meta-information (e.g. lchan type).
 *
 * @param  ctx     parent talloc context
 * @param  prim    external prim pointer (will point to the allocated prim)
 * @param  pl_len  prim payload length
 * @param  chan_nr RSL channel description (used to set a proper chan)
 * @param  link_id RSL link description (used to set a proper chan)
 * @return         zero in case of success, otherwise a error number
 */
int sched_prim_init(void *ctx, struct trx_ts_prim **prim,
	size_t pl_len, uint8_t chan_nr, uint8_t link_id)
{
	enum trx_lchan_type lchan_type;
	struct trx_ts_prim *new_prim;
	uint8_t len;

	/* Determine lchan type */
	lchan_type = sched_trx_chan_nr2lchan_type(chan_nr, link_id);
	if (!lchan_type) {
		LOGP(DSCH, LOGL_ERROR, "Couldn't determine lchan type "
			"for chan_nr=%02x and link_id=%02x\n", chan_nr, link_id);
		return -EINVAL;
	}

	/* How much memory do we need? */
	len  = sizeof(struct trx_ts_prim); /* Primitive header */
	len += pl_len; /* Requested payload size */

	/* Allocate a new primitive */
	new_prim = talloc_zero_size(ctx, len);
	if (new_prim == NULL) {
		LOGP(DSCH, LOGL_ERROR, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Init primitive header */
	new_prim->payload_len = pl_len;
	new_prim->chan = lchan_type;

	/* Set external pointer */
	*prim = new_prim;

	return 0;
}

/**
 * Adds a primitive to the end of transmit queue of a particular
 * timeslot, whose index is parsed from chan_nr.
 *
 * @param  trx     TRX instance
 * @param  prim    to be enqueued primitive
 * @param  chan_nr RSL channel description
 * @return         zero in case of success, otherwise a error number
 */
int sched_prim_push(struct trx_instance *trx,
	struct trx_ts_prim *prim, uint8_t chan_nr)
{
	struct trx_ts *ts;
	uint8_t tn;

	/* Determine TS index */
	tn = chan_nr & 0x7;

	/* Check whether required timeslot is allocated and configured */
	ts = trx->ts_list[tn];
	if (ts == NULL || ts->mf_layout == NULL) {
		LOGP(DSCH, LOGL_ERROR, "Timeslot %u isn't configured\n", tn);
		return -EINVAL;
	}

	/**
	 * Change talloc context of primitive
	 * from trx to the parent ts
	 */
	talloc_steal(ts, prim);

	/* Add primitive to TS transmit queue */
	llist_add_tail(&prim->list, &ts->tx_prims);

	return 0;
}

/**
 * Composes a new primitive using either cached (if populated),
 * or "dummy" Measurement Report message.
 *
 * @param  lchan lchan to assign a primitive
 * @return       SACCH primitive to be transmitted
 */
static struct trx_ts_prim *prim_compose_mr(struct trx_lchan_state *lchan)
{
	struct trx_ts_prim *prim;
	uint8_t *mr_src_ptr;
	bool cached;
	int rc;

	/* "Dummy" Measurement Report */
	static const uint8_t meas_rep_dummy[] = {
		/* L1 SACCH pseudo-header */
		0x0f, 0x00,

		/* LAPDm header */
		0x01, 0x03, 0x49,

		/* RR Management messages, Measurement Report */
		0x06, 0x15,

		/* Measurement results (see 3GPP TS 44.018, section 10.5.2.20):
		 *   0... .... = BA-USED: 0
		 *   .0.. .... = DTX-USED: DTX was not used
		 *   ..11 0110 = RXLEV-FULL-SERVING-CELL: -57 <= x < -56 dBm (54)
		 *   0... .... = 3G-BA-USED: 0
		 *   .1.. .... = MEAS-VALID: The measurement results are not valid
		 *   ..11 0110 = RXLEV-SUB-SERVING-CELL: -57 <= x < -56 dBm (54)
		 *   0... .... = SI23_BA_USED: 0
		 *   .000 .... = RXQUAL-FULL-SERVING-CELL: BER < 0.2%, Mean value 0.14% (0)
		 *   .... 000. = RXQUAL-SUB-SERVING-CELL: BER < 0.2%, Mean value 0.14% (0)
		 *   .... ...1  11.. .... = NO-NCELL-M: Neighbour cell information not available */
		0x36, 0x76, 0x01, 0xc0,

		/* 0** -- Padding with zeroes */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};

	/* Allocate a new primitive */
	rc = sched_prim_init(lchan, &prim, GSM_MACBLOCK_LEN,
		trx_lchan_desc[lchan->type].chan_nr, TRX_CH_LID_SACCH);
	OSMO_ASSERT(rc == 0);

	/* Check if the MR cache is populated (verify LAPDm header) */
	cached = (lchan->sacch.mr_cache[2] != 0x00
		&& lchan->sacch.mr_cache[3] != 0x00
		&& lchan->sacch.mr_cache[4] != 0x00);
	if (cached) { /* Use the cached one */
		mr_src_ptr = lchan->sacch.mr_cache;
		lchan->sacch.mr_cache_usage++;
	} else { /* Use "dummy" one */
		mr_src_ptr = (uint8_t *) meas_rep_dummy;
	}

	/* Compose a new Measurement Report primitive */
	memcpy(prim->payload, mr_src_ptr, GSM_MACBLOCK_LEN);

	/**
	 * Update the L1 SACCH pseudo-header (only for cached MRs)
	 *
	 * TODO: filling of the actual values into cached Measurement
	 * Reports would break the distance spoofing feature. If it
	 * were known whether the spoofing is enabled or not, we could
	 * decide whether to update the cached L1 SACCH header here.
	 */
	if (!cached) {
		prim->payload[0] = lchan->ts->trx->tx_power;
		prim->payload[1] = lchan->ts->trx->ta;
	}

	/* Inform about the cache usage count */
	if (cached && lchan->sacch.mr_cache_usage > 5) {
		LOGP(DSCHD, LOGL_NOTICE, "SACCH MR cache usage count=%u > 5 "
			"on lchan=%s => ancient measurements, please fix!\n",
			lchan->sacch.mr_cache_usage,
			trx_lchan_desc[lchan->type].name);
	}

	LOGP(DSCHD, LOGL_NOTICE, "Using a %s Measurement Report "
		"on lchan=%s\n", (cached ? "cached" : "dummy"),
		trx_lchan_desc[lchan->type].name);

	return prim;
}

/**
 * Dequeues a SACCH primitive from transmit queue, if present.
 * Otherwise dequeues a cached Measurement Report (the last
 * received one). Finally, if the cache is empty, a "dummy"
 * measurement report is used.
 *
 * According to 3GPP TS 04.08, section 3.4.1, SACCH channel
 * accompanies either a traffic or a signaling channel. It
 * has the particularity that continuous transmission must
 * occur in both directions, so on the Uplink direction
 * measurement result messages are sent at each possible
 * occasion when nothing else has to be sent. The LAPDm
 * fill frames (0x01, 0x03, 0x01, 0x2b, ...) are not
 * applicable on SACCH channels!
 *
 * Unfortunately, 3GPP TS 04.08 doesn't clearly state
 * which "else messages" besides Measurement Reports
 * can be send by the MS on SACCH channels. However,
 * in sub-clause 3.4.1 it's stated that the interval
 * between two successive measurement result messages
 * shall not exceed one L2 frame.
 *
 * @param  queue transmit queue to take a prim from
 * @param  lchan lchan to assign a primitive
 * @return       SACCH primitive to be transmitted
 */
static struct trx_ts_prim *prim_dequeue_sacch(struct llist_head *queue,
	struct trx_lchan_state *lchan)
{
	struct trx_ts_prim *prim_nmr = NULL;
	struct trx_ts_prim *prim_mr = NULL;
	struct trx_ts_prim *prim;
	bool mr_now;

	/* Shall we transmit MR now? */
	mr_now = !lchan->sacch.mr_tx_last;

#define PRIM_IS_MR(prim) \
	(prim->payload[5] == GSM48_PDISC_RR \
		&& prim->payload[6] == GSM48_MT_RR_MEAS_REP)

	/* Iterate over all primitives in the queue */
	llist_for_each_entry(prim, queue, list) {
		/* We are looking for particular channel */
		if (prim->chan != lchan->type)
			continue;

		/* Look for a Measurement Report */
		if (!prim_mr && PRIM_IS_MR(prim))
			prim_mr = prim;

		/* Look for anything else */
		if (!prim_nmr && !PRIM_IS_MR(prim))
			prim_nmr = prim;

		/* Should we look further? */
		if (mr_now && prim_mr)
			break; /* MR was found */
		else if (!mr_now && prim_nmr)
			break; /* something else was found */
	}

	LOGP(DSCHD, LOGL_DEBUG, "SACCH MR selection on lchan=%s: "
		"mr_tx_last=%d prim_mr=%p prim_nmr=%p\n",
		trx_lchan_desc[lchan->type].name,
		lchan->sacch.mr_tx_last,
		prim_mr, prim_nmr);

	/* Prioritize non-MR prim if possible */
	if (mr_now && prim_mr)
		prim = prim_mr;
	else if (!mr_now && prim_nmr)
		prim = prim_nmr;
	else if (!mr_now && prim_mr)
		prim = prim_mr;
	else /* Nothing was found */
		prim = NULL;

	/* Have we found what we were looking for? */
	if (prim) /* Dequeue if so */
		llist_del(&prim->list);
	else /* Otherwise compose a new MR */
		prim = prim_compose_mr(lchan);

	/* Update the cached report */
	if (prim == prim_mr) {
		memcpy(lchan->sacch.mr_cache,
			prim->payload, GSM_MACBLOCK_LEN);
		lchan->sacch.mr_cache_usage = 0;

		LOGP(DSCHD, LOGL_DEBUG, "SACCH MR cache has been updated "
			"for lchan=%s\n", trx_lchan_desc[lchan->type].name);
	}

	/* Update the MR transmission state */
	lchan->sacch.mr_tx_last = PRIM_IS_MR(prim);

	LOGP(DSCHD, LOGL_DEBUG, "SACCH decision on lchan=%s: %s\n",
		trx_lchan_desc[lchan->type].name, PRIM_IS_MR(prim) ?
			"Measurement Report" : "data frame");

	return prim;
}

/* Dequeues a primitive of a given channel type */
static struct trx_ts_prim *prim_dequeue_one(struct llist_head *queue,
	enum trx_lchan_type lchan_type)
{
	struct trx_ts_prim *prim;

	/**
	 * There is no need to use the 'safe' list iteration here
	 * as an item removal is immediately followed by return.
	 */
	llist_for_each_entry(prim, queue, list) {
		if (prim->chan == lchan_type) {
			llist_del(&prim->list);
			return prim;
		}
	}

	return NULL;
}

/**
 * Dequeues either a FACCH, or a speech TCH primitive
 * of a given channel type (Lm or Bm).
 *
 * Note: we could avoid 'lchan_type' parameter and just
 * check the prim's channel type using CHAN_IS_TCH(),
 * but the current approach is a bit more flexible,
 * and allows one to have both sub-slots of TCH/H
 * enabled on same timeslot e.g. for testing...
 *
 * @param  queue      transmit queue to take a prim from
 * @param  lchan_type required channel type of a primitive,
 *                    e.g. TRXC_TCHF, TRXC_TCHH_0, or TRXC_TCHH_1
 * @param  facch      FACCH (true) or speech (false) prim?
 * @return            either a FACCH, or a TCH primitive if found,
 *                    otherwise NULL
 */
static struct trx_ts_prim *prim_dequeue_tch(struct llist_head *queue,
	enum trx_lchan_type lchan_type, bool facch)
{
	struct trx_ts_prim *prim;

	/**
	 * There is no need to use the 'safe' list iteration here
	 * as an item removal is immediately followed by return.
	 */
	llist_for_each_entry(prim, queue, list) {
		if (prim->chan != lchan_type)
			continue;

		/* Either FACCH, or not FACCH */
		if (PRIM_IS_FACCH(prim) != facch)
			continue;

		llist_del(&prim->list);
		return prim;
	}

	return NULL;
}

/**
 * Dequeues either a TCH/F, or a FACCH/F prim (preferred).
 * If a FACCH/F prim is found, one TCH/F prim is being
 * dropped (i.e. replaced).
 *
 * @param  queue a transmit queue to take a prim from
 * @return       either a FACCH/F, or a TCH/F primitive,
 *               otherwise NULL
 */
static struct trx_ts_prim *prim_dequeue_tch_f(struct llist_head *queue)
{
	struct trx_ts_prim *facch;
	struct trx_ts_prim *tch;

	/* Attempt to find a pair of both FACCH/F and TCH/F frames */
	facch = prim_dequeue_tch(queue, TRXC_TCHF, true);
	tch = prim_dequeue_tch(queue, TRXC_TCHF, false);

	/* Prioritize FACCH/F, if found */
	if (facch) {
		/* One TCH/F prim is replaced */
		if (tch)
			talloc_free(tch);
		return facch;
	} else if (tch) {
		/* Only TCH/F prim was found */
		return tch;
	} else {
		/* Nothing was found, e.g. when only SACCH frames are in queue */
		return NULL;
	}
}

/**
 * Dequeues either a TCH/H, or a FACCH/H prim (preferred).
 * If a FACCH/H prim is found, two TCH/H prims are being
 * dropped (i.e. replaced).
 *
 * According to GSM 05.02, the following blocks can be used
 * to carry FACCH/H data (see clause 7, table 1 of 9):
 *
 * UL FACCH/H0:
 * B0(0,2,4,6,8,10), B1(8,10,13,15,17,19), B2(17,19,21,23,0,2)
 *
 * UL FACCH/H1:
 * B0(1,3,5,7,9,11), B1(9,11,14,16,18,20), B2(18,20,22,24,1,3)
 *
 * where the numbers within brackets are fn % 26.
 *
 * @param  queue      transmit queue to take a prim from
 * @param  fn         the current frame number
 * @param  lchan_type required channel type of a primitive,
 * @return            either a FACCH/H, or a TCH/H primitive,
 *                    otherwise NULL
 */
static struct trx_ts_prim *prim_dequeue_tch_h(struct llist_head *queue,
	uint32_t fn, enum trx_lchan_type lchan_type)
{
	struct trx_ts_prim *facch;
	struct trx_ts_prim *tch;
	bool facch_now;

	/* May we initiate an UL FACCH/H frame transmission now? */
	facch_now = sched_tchh_facch_start(lchan_type, fn, true);
	if (!facch_now) /* Just dequeue a TCH/H prim */
		goto no_facch;

	/* If there are no FACCH/H prims in the queue */
	facch = prim_dequeue_tch(queue, lchan_type, true);
	if (!facch) /* Just dequeue a TCH/H prim */
		goto no_facch;

	/* FACCH/H prim replaces two TCH/F prims */
	tch = prim_dequeue_tch(queue, lchan_type, false);
	if (tch) {
		/* At least one TCH/H prim is dropped */
		talloc_free(tch);

		/* Attempt to find another */
		tch = prim_dequeue_tch(queue, lchan_type, false);
		if (tch) /* Drop the second TCH/H prim */
			talloc_free(tch);
	}

	return facch;

no_facch:
	return prim_dequeue_tch(queue, lchan_type, false);
}

/**
 * Dequeues a single primitive of required type
 * from a specified transmit queue.
 *
 * @param  queue      a transmit queue to take a prim from
 * @param  fn         the current frame number (used for FACCH/H)
 * @param  lchan      logical channel state
 * @return            a primitive or NULL if not found
 */
struct trx_ts_prim *sched_prim_dequeue(struct llist_head *queue,
	uint32_t fn, struct trx_lchan_state *lchan)
{
	/* SACCH is unorthodox, see 3GPP TS 04.08, section 3.4.1 */
	if (CHAN_IS_SACCH(lchan->type))
		return prim_dequeue_sacch(queue, lchan);

	/* There is nothing to dequeue */
	if (llist_empty(queue))
		return NULL;

	switch (lchan->type) {
	/* TCH/F requires FACCH/F prioritization */
	case TRXC_TCHF:
		return prim_dequeue_tch_f(queue);

	/* FACCH/H prioritization is a bit more complex */
	case TRXC_TCHH_0:
	case TRXC_TCHH_1:
		return prim_dequeue_tch_h(queue, fn, lchan->type);

	/* Other kinds of logical channels */
	default:
		return prim_dequeue_one(queue, lchan->type);
	}
}

/**
 * Drops the current primitive of specified logical channel
 *
 * @param lchan a logical channel to drop prim from
 */
void sched_prim_drop(struct trx_lchan_state *lchan)
{
	/* Forget this primitive */
	talloc_free(lchan->prim);
	lchan->prim = NULL;
}

/**
 * Assigns a dummy primitive to a lchan depending on its type.
 * Could be used when there is nothing to transmit, but
 * CBTX (Continuous Burst Transmission) is assumed.
 *
 * @param  lchan lchan to assign a primitive
 * @return       zero in case of success, otherwise a error code
 */
int sched_prim_dummy(struct trx_lchan_state *lchan)
{
	enum trx_lchan_type chan = lchan->type;
	uint8_t tch_mode = lchan->tch_mode;
	struct trx_ts_prim *prim;
	uint8_t prim_buffer[40];
	size_t prim_len = 0;
	int i;

	/**
	 * TS 144.006, section 8.4.2.3 "Fill frames"
	 * A fill frame is a UI command frame for SAPI 0, P=0
	 * and with an information field of 0 octet length.
	 */
	static const uint8_t lapdm_fill_frame[] = {
		0x01, 0x03, 0x01, 0x2b,
		/* Pending part is to be randomized */
	};

	/* Make sure that there is no existing primitive */
	OSMO_ASSERT(lchan->prim == NULL);
	/* Not applicable for SACCH! */
	OSMO_ASSERT(!CHAN_IS_SACCH(lchan->type));

	/**
	 * Determine what actually should be generated:
	 * TCH in GSM48_CMODE_SIGN: LAPDm fill frame;
	 * TCH in other modes: silence frame;
	 * other channels: LAPDm fill frame.
	 */
	if (CHAN_IS_TCH(chan) && TCH_MODE_IS_SPEECH(tch_mode)) {
		/* Bad frame indication */
		prim_len = sched_bad_frame_ind(prim_buffer, lchan);
	} else if (CHAN_IS_TCH(chan) && TCH_MODE_IS_DATA(tch_mode)) {
		/* FIXME: should we do anything for CSD? */
		return 0;
	} else {
		/* Copy LAPDm fill frame's header */
		memcpy(prim_buffer, lapdm_fill_frame, sizeof(lapdm_fill_frame));

		/**
		 * TS 144.006, section 5.2 "Frame delimitation and fill bits"
		 * Except for the first octet containing fill bits which shall
		 * be set to the binary value "00101011", each fill bit should
		 * be set to a random value when sent by the network.
		 */
		for (i = sizeof(lapdm_fill_frame); i < GSM_MACBLOCK_LEN; i++)
			prim_buffer[i] = (uint8_t) rand();

		/* Define a prim length */
		prim_len = GSM_MACBLOCK_LEN;
	}

	/* Nothing to allocate / assign */
	if (!prim_len)
		return 0;

	/* Allocate a new primitive */
	prim = talloc_zero_size(lchan, sizeof(struct trx_ts_prim) + prim_len);
	if (prim == NULL)
		return -ENOMEM;

	/* Init primitive header */
	prim->payload_len = prim_len;
	prim->chan = lchan->type;

	/* Fill in the payload */
	memcpy(prim->payload, prim_buffer, prim_len);

	/* Assign the current prim */
	lchan->prim = prim;

	LOGP(DSCHD, LOGL_DEBUG, "Transmitting a dummy / silence frame "
		"on lchan=%s\n", trx_lchan_desc[chan].name);

	return 0;
}

/**
 * Flushes a queue of primitives
 *
 * @param list list of prims going to be flushed
 */
void sched_prim_flush_queue(struct llist_head *list)
{
	struct trx_ts_prim *prim, *prim_next;

	llist_for_each_entry_safe(prim, prim_next, list, list) {
		llist_del(&prim->list);
		talloc_free(prim);
	}
}
