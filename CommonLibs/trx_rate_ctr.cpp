/*
 * Copyright (C) 2019 sysmocom - s.f.m.c. GmbH
 * All Rights Reserved
 *
 * SPDX-License-Identifier: AGPL-3.0+
 *
 * Author: Pau Espin Pedrol <pespin@sysmocom.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 */

/*
 * rate_ctr API uses several osmocom select loop features, and as a result,
 * calls to it must be done through the main thread (the one running the osmocom
 * loop in osmo-trx).
 * Since read/write from/to SDR is done in separate threads (even read and write
 * each use a different thread), we must use some sort of message passing system
 * between main thread feeding rate_ctr structures and the Rx/Tx threads
 * generating the events.
 * The idea is that upon read/write issues, lower layers (SDR APIs) provide us with
 * underrun/overrun/droppedPackets information, and in that case we pass that up
 * the stack through signal <SS_DEVICE,S_DEVICE_COUNTER_CHANGE> with signal_cb
 * being a pointer to a "struct device_counters" structure, which contains
 * device (implementation agnostic) statful counters for different kind of
 * statistics.
 * That signal is processed here in device_sig_cb, where a copy of the "struct
 * device_counters" structure is held and the main thread is instructed through
 * a timerfd to update rate_ctr APIs against this copy. All this is done inside
 * a mutex to avoid different race conditions (between Rx andTx threads, and
 * between Rx/Tx and main thread). For the same reason, callers of signal
 * <SS_DEVICE,S_DEVICE_COUNTER_CHANGE> (device_sig_cb), that is Rx/Tx threads,
 * must do so with PTHREAD_CANCEL_DISABLE, in order to avoid possible deadlocks
 * in case the main thread decides to cancel other threads due to a shutdown
 * operation (fi SIGKILL received)
 */

#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <arpa/inet.h>

extern "C" {
#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/rate_ctr.h>
#include <osmocom/core/select.h>
#include <osmocom/core/stats.h>
#include <osmocom/core/timer.h>

#include "osmo_signal.h"
#include "trx_vty.h"
#include "trx_rate_ctr.h"
}
#include "Threads.h"
#include "Logger.h"

/* Used in dev_ctrs_pending, when set it means that channel slot contains unused
   (non-pending) counter data */
#define PENDING_CHAN_NONE SIZE_MAX

static void *trx_rate_ctr_ctx;

static struct rate_ctr_group** rate_ctrs;
static struct device_counters* dev_ctrs_pending;
static struct trx_counters* trx_ctrs_pending;
static size_t chan_len;
static struct osmo_fd dev_rate_ctr_timerfd;
static struct osmo_fd trx_rate_ctr_timerfd;
static Mutex dev_rate_ctr_mutex;
static Mutex trx_rate_ctr_mutex;

struct osmo_timer_list threshold_timer;
static LLIST_HEAD(threshold_list);
static unsigned int threshold_timer_sched_secs;
static bool threshold_initied;

const struct value_string rate_ctr_intv[] = {
	{ RATE_CTR_INTV_SEC,	"per-second" },
	{ RATE_CTR_INTV_MIN,	"per-minute" },
	{ RATE_CTR_INTV_HOUR,	"per-hour" },
	{ RATE_CTR_INTV_DAY, 	"per-day" },
	{ 0, NULL }
};

const struct value_string trx_chan_ctr_names[] = {
	{ TRX_CTR_DEV_RX_OVERRUNS,	"rx_overruns" },
	{ TRX_CTR_DEV_TX_UNDERRUNS,	"tx_underruns" },
	{ TRX_CTR_DEV_RX_DROP_EV,	"rx_drop_events" },
	{ TRX_CTR_DEV_RX_DROP_SMPL,	"rx_drop_samples" },
	{ TRX_CTR_DEV_TX_DROP_EV,	"tx_drop_events" },
	{ TRX_CTR_DEV_TX_DROP_SMPL,	"tx_drop_samples" },
	{ TRX_CTR_TRX_TX_STALE_BURSTS,	"tx_stale_bursts" },
	{ 0, NULL }
};

static const struct rate_ctr_desc trx_chan_ctr_desc[] = {
	[TRX_CTR_DEV_RX_OVERRUNS]		= { "device:rx_overruns",	"Number of Rx overruns in FIFO queue" },
	[TRX_CTR_DEV_TX_UNDERRUNS]		= { "device:tx_underruns",	"Number of Tx underruns in FIFO queue" },
	[TRX_CTR_DEV_RX_DROP_EV]		= { "device:rx_drop_events",	"Number of times Rx samples were dropped by HW" },
	[TRX_CTR_DEV_RX_DROP_SMPL]		= { "device:rx_drop_samples",	"Number of Rx samples dropped by HW" },
	[TRX_CTR_DEV_TX_DROP_EV]		= { "device:tx_drop_events",	"Number of times Tx samples were dropped by HW" },
	[TRX_CTR_DEV_TX_DROP_SMPL]		= { "device:tx_drop_samples",	"Number of Tx samples dropped by HW" },
	[TRX_CTR_TRX_TX_STALE_BURSTS]		= { "trx:tx_stale_bursts",	"Number of Tx burts dropped by TRX due to arriving too late" },
};

static const struct rate_ctr_group_desc trx_chan_ctr_group_desc = {
	.group_name_prefix		= "trx:chan",
	.group_description		= "osmo-trx statistics",
	.class_id			= OSMO_STATS_CLASS_GLOBAL,
	.num_ctr			= ARRAY_SIZE(trx_chan_ctr_desc),
	.ctr_desc			= trx_chan_ctr_desc,
};

static int dev_rate_ctr_timerfd_cb(struct osmo_fd *ofd, unsigned int what) {
	size_t chan;
	struct rate_ctr *ctr;
	LOGC(DMAIN, NOTICE) << "Main thread is updating Device counters";
	dev_rate_ctr_mutex.lock();
	for (chan = 0; chan < chan_len; chan++) {
		if (dev_ctrs_pending[chan].chan == PENDING_CHAN_NONE)
			continue;
		LOGCHAN(chan, DMAIN, INFO) << "rate_ctr update";
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_DEV_RX_OVERRUNS];
		rate_ctr_add(ctr, dev_ctrs_pending[chan].rx_overruns - ctr->current);
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_DEV_TX_UNDERRUNS];
		rate_ctr_add(ctr, dev_ctrs_pending[chan].tx_underruns - ctr->current);
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_DEV_RX_DROP_EV];
		rate_ctr_add(ctr, dev_ctrs_pending[chan].rx_dropped_events - ctr->current);
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_DEV_RX_DROP_SMPL];
		rate_ctr_add(ctr, dev_ctrs_pending[chan].rx_dropped_samples - ctr->current);
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_DEV_TX_DROP_EV];
		rate_ctr_add(ctr, dev_ctrs_pending[chan].tx_dropped_events - ctr->current);
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_DEV_TX_DROP_SMPL];
		rate_ctr_add(ctr, dev_ctrs_pending[chan].tx_dropped_samples - ctr->current);

		/* Mark as done */
		dev_ctrs_pending[chan].chan = PENDING_CHAN_NONE;
	}
	if (osmo_timerfd_disable(&dev_rate_ctr_timerfd) < 0)
		LOGC(DMAIN, ERROR) << "Failed to disable timerfd";
	dev_rate_ctr_mutex.unlock();
	return 0;
}

static int trx_rate_ctr_timerfd_cb(struct osmo_fd *ofd, unsigned int what) {
	size_t chan;
	struct rate_ctr *ctr;
	LOGC(DMAIN, NOTICE) << "Main thread is updating Transceiver counters";
	dev_rate_ctr_mutex.lock();
	for (chan = 0; chan < chan_len; chan++) {
		if (trx_ctrs_pending[chan].chan == PENDING_CHAN_NONE)
			continue;
		LOGCHAN(chan, DMAIN, INFO) << "rate_ctr update";
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_TRX_TX_STALE_BURSTS];
		rate_ctr_add(ctr, trx_ctrs_pending[chan].tx_stale_bursts - ctr->current);
		/* Mark as done */
		trx_ctrs_pending[chan].chan = PENDING_CHAN_NONE;
	}
	if (osmo_timerfd_disable(&trx_rate_ctr_timerfd) < 0)
		LOGC(DMAIN, ERROR) << "Failed to disable timerfd";
	trx_rate_ctr_mutex.unlock();
	return 0;
}

/* Callback function to be called every time we receive a signal from DEVICE */
static int device_sig_cb(unsigned int subsys, unsigned int signal,
			 void *handler_data, void *signal_data)
{
	struct device_counters *dev_ctr;
	struct trx_counters *trx_ctr;
	/* Delay sched around 20 ms, in case we receive several calls from several
	 * channels batched */
	struct timespec next_sched = {.tv_sec = 0, .tv_nsec = 20*1000*1000};
	/* no automatic re-trigger */
	struct timespec intv_sched = {.tv_sec = 0, .tv_nsec = 0};

	switch (signal) {
	case S_DEVICE_COUNTER_CHANGE:
		dev_ctr = (struct device_counters *)signal_data;
		LOGCHAN(dev_ctr->chan, DMAIN, NOTICE) << "Received counter change from radioDevice";
		dev_rate_ctr_mutex.lock();
		dev_ctrs_pending[dev_ctr->chan] = *dev_ctr;
		if (osmo_timerfd_schedule(&dev_rate_ctr_timerfd, &next_sched, &intv_sched) < 0) {
			LOGC(DMAIN, ERROR) << "Failed to schedule timerfd: " << errno << " = "<< strerror(errno);
		}
		dev_rate_ctr_mutex.unlock();
		break;
	case S_TRX_COUNTER_CHANGE:
		trx_ctr = (struct trx_counters *)signal_data;
		LOGCHAN(trx_ctr->chan, DMAIN, NOTICE) << "Received counter change from Transceiver";
		trx_rate_ctr_mutex.lock();
		trx_ctrs_pending[trx_ctr->chan] = *trx_ctr;
		if (osmo_timerfd_schedule(&trx_rate_ctr_timerfd, &next_sched, &intv_sched) < 0) {
			LOGC(DMAIN, ERROR) << "Failed to schedule timerfd: " << errno << " = "<< strerror(errno);
		}
		trx_rate_ctr_mutex.unlock();
		break;
	default:
		break;
	}
	return 0;
}

/************************************
 * ctr_threshold  APIs
 ************************************/
static const char* ctr_threshold_2_vty_str(struct ctr_threshold *ctr)
{
	static char buf[256];
	int rc = 0;
	rc += snprintf(buf, sizeof(buf), "ctr-error-threshold %s", get_value_string(trx_chan_ctr_names, ctr->ctr_id));
	rc += snprintf(buf + rc, sizeof(buf) - rc, " %d %s", ctr->val, get_value_string(rate_ctr_intv, ctr->intv));
	return buf;
}

static void threshold_timer_cb(void *data)
{
	struct ctr_threshold *ctr_thr;
	struct rate_ctr *rate_ctr;
	size_t chan;
	LOGC(DMAIN, DEBUG) << "threshold_timer_cb fired!";

	llist_for_each_entry(ctr_thr, &threshold_list, list) {
		for (chan = 0; chan < chan_len; chan++) {
			rate_ctr = &rate_ctrs[chan]->ctr[ctr_thr->ctr_id];
			LOGCHAN(chan, DMAIN, INFO) << "checking threshold: " << ctr_threshold_2_vty_str(ctr_thr)
						   << " ("<< rate_ctr->intv[ctr_thr->intv].rate << " vs " << ctr_thr->val << ")";
			if (rate_ctr->intv[ctr_thr->intv].rate >= ctr_thr->val) {
				LOGCHAN(chan, DMAIN, FATAL) << "threshold reached, stopping! " << ctr_threshold_2_vty_str(ctr_thr)
							   << " ("<< rate_ctr->intv[ctr_thr->intv].rate << " vs " << ctr_thr->val << ")";
				osmo_signal_dispatch(SS_MAIN, S_MAIN_STOP_REQUIRED, NULL);
				return;
			}
		}
	}
	osmo_timer_schedule(&threshold_timer, threshold_timer_sched_secs, 0);
}

static size_t ctr_threshold_2_seconds(struct ctr_threshold *ctr)
{
	size_t mult = 0;
	switch (ctr->intv) {
	case RATE_CTR_INTV_SEC:
		mult = 1;
		break;
	case RATE_CTR_INTV_MIN:
		mult = 60;
		break;
	case RATE_CTR_INTV_HOUR:
		mult = 60*60;
		break;
	case RATE_CTR_INTV_DAY:
		mult = 60*60*24;
		break;
	default:
		OSMO_ASSERT(false);
	}
	return mult;
}

static void threshold_timer_update_intv() {
	struct ctr_threshold *ctr, *min_ctr;
	size_t secs, min_secs;

	/* Avoid scheduling timer until itself and other structures are prepared
	   by trx_rate_ctr_init */
	if (!threshold_initied)
		return;

	if (llist_empty(&threshold_list)) {
		if (osmo_timer_pending(&threshold_timer))
			osmo_timer_del(&threshold_timer);
		return;
	}

	min_ctr = llist_first_entry(&threshold_list, struct ctr_threshold, list);
	min_secs = ctr_threshold_2_seconds(min_ctr);

	llist_for_each_entry(ctr, &threshold_list, list) {
		secs = ctr_threshold_2_seconds(ctr);
		if (min_secs > secs)
			min_secs = secs;
	}


	threshold_timer_sched_secs = OSMO_MAX((int)(min_secs / 2 - 1), 1);
	LOGC(DMAIN, INFO) << "New ctr-error-threshold check interval: "
			  << threshold_timer_sched_secs << " seconds";
	osmo_timer_schedule(&threshold_timer, threshold_timer_sched_secs, 0);
}

/* Init rate_ctr subsystem. Expected to be called during process start by main thread before VTY is ready */
void trx_rate_ctr_init(void *ctx, struct trx_ctx* trx_ctx)
{
	size_t  i;
	trx_rate_ctr_ctx = ctx;
	chan_len = trx_ctx->cfg.num_chans;
	dev_ctrs_pending = (struct device_counters*) talloc_zero_size(ctx, chan_len * sizeof(struct device_counters));
	trx_ctrs_pending = (struct trx_counters*) talloc_zero_size(ctx, chan_len * sizeof(struct trx_counters));
	rate_ctrs = (struct rate_ctr_group**) talloc_zero_size(ctx, chan_len * sizeof(struct rate_ctr_group*));

	for (i = 0; i < chan_len; i++) {
		dev_ctrs_pending[i].chan = PENDING_CHAN_NONE;
		trx_ctrs_pending[i].chan = PENDING_CHAN_NONE;
		rate_ctrs[i] = rate_ctr_group_alloc(ctx, &trx_chan_ctr_group_desc, i);
		if (!rate_ctrs[i]) {
			LOGCHAN(i, DMAIN, ERROR) << "Failed to allocate rate ctr";
			exit(1);
		}
	}
	dev_rate_ctr_timerfd.fd = -1;
	if (osmo_timerfd_setup(&dev_rate_ctr_timerfd, dev_rate_ctr_timerfd_cb, NULL) < 0) {
		LOGC(DMAIN, ERROR) << "Failed to setup timerfd";
		exit(1);
	}
	trx_rate_ctr_timerfd.fd = -1;
	if (osmo_timerfd_setup(&trx_rate_ctr_timerfd, trx_rate_ctr_timerfd_cb, NULL) < 0) {
		LOGC(DMAIN, ERROR) << "Failed to setup timerfd";
		exit(1);
	}
	osmo_signal_register_handler(SS_DEVICE, device_sig_cb, NULL);

	/* Now set up threshold checks */
	threshold_initied = true;
	osmo_timer_setup(&threshold_timer, threshold_timer_cb, NULL);
	threshold_timer_update_intv();
}

void trx_rate_ctr_threshold_add(struct ctr_threshold *ctr)
{
	struct ctr_threshold *new_ctr;

	new_ctr = talloc_zero(trx_rate_ctr_ctx, struct ctr_threshold);
	*new_ctr = *ctr;
	LOGC(DMAIN, NOTICE) << "Adding new threshold check: " << ctr_threshold_2_vty_str(new_ctr);
	llist_add(&new_ctr->list, &threshold_list);
	threshold_timer_update_intv();
}

int trx_rate_ctr_threshold_del(struct ctr_threshold *del_ctr)
{
	struct ctr_threshold *ctr;

	llist_for_each_entry(ctr, &threshold_list, list) {
		if (ctr->intv != del_ctr->intv ||
		    ctr->ctr_id != del_ctr->ctr_id ||
		    ctr->val != del_ctr->val)
			continue;

		LOGC(DMAIN, NOTICE) << "Deleting threshold check: " << ctr_threshold_2_vty_str(del_ctr);
		llist_del(&ctr->list);
		talloc_free(ctr);
		threshold_timer_update_intv();
		return 0;
	}
	return -1;
}

void trx_rate_ctr_threshold_write_config(struct vty *vty, char *indent_prefix)
{
	struct ctr_threshold *ctr;

	llist_for_each_entry(ctr, &threshold_list, list) {
		vty_out(vty, "%s%s%s", indent_prefix, ctr_threshold_2_vty_str(ctr), VTY_NEWLINE);
	}
}
