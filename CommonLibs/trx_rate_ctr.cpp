/*
 * Copyright (C) 2019 sysmocom - s.f.m.c. GmbH
 * All Rights Reserved
 *
 * Author: Pau Espin Pedrol <pespin@sysmocom.de>
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
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
 * a mutex to avoid different race conditons (between Rx andTx threads, and
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

#include "osmo_signal.h"
#include "trx_vty.h"
#include "trx_rate_ctr.h"
}
#include "Threads.h"
#include "Logger.h"

/* Used in ctrs_pending, when set it means that channel slot contains unused
   (non-pending) counter data */
#define PENDING_CHAN_NONE SIZE_MAX

static struct rate_ctr_group** rate_ctrs;
static struct device_counters* ctrs_pending;
static size_t chan_len;
static struct osmo_fd rate_ctr_timerfd;
static Mutex rate_ctr_mutex;

enum {
	TRX_CTR_RX_UNDERRUNS,
	TRX_CTR_RX_OVERRUNS,
	TRX_CTR_TX_UNDERRUNS,
	TRX_CTR_RX_DROP_EV,
	TRX_CTR_RX_DROP_SMPL,
};

static const struct rate_ctr_desc trx_chan_ctr_desc[] = {
	[TRX_CTR_RX_UNDERRUNS]		= { "device:rx_underruns",	"Number of Rx underruns" },
	[TRX_CTR_RX_OVERRUNS]		= { "device:rx_overruns",	"Number of Rx overruns" },
	[TRX_CTR_TX_UNDERRUNS]		= { "device:tx_underruns",	"Number of Tx underruns" },
	[TRX_CTR_RX_DROP_EV]		= { "device:rx_drop_events",	"Number of times Rx samples were dropped by HW" },
	[TRX_CTR_RX_DROP_SMPL]		= { "device:rx_drop_samples",	"Number of Rx samples dropped by HW" },
};

static const struct rate_ctr_group_desc trx_chan_ctr_group_desc = {
	.group_name_prefix		= "trx:chan",
	.group_description		= "osmo-trx statistics",
	.class_id			= OSMO_STATS_CLASS_GLOBAL,
	.num_ctr			= ARRAY_SIZE(trx_chan_ctr_desc),
	.ctr_desc			= trx_chan_ctr_desc,
};

static int rate_ctr_timerfd_cb(struct osmo_fd *ofd, unsigned int what) {
	size_t chan;
	struct rate_ctr *ctr;
	LOGC(DMAIN, NOTICE) << "Main thread is updating counters";
	rate_ctr_mutex.lock();
	for (chan = 0; chan < chan_len; chan++) {
		if (ctrs_pending[chan].chan == PENDING_CHAN_NONE)
			continue;
		LOGCHAN(chan, DMAIN, INFO) << "rate_ctr update";
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_RX_UNDERRUNS];
		rate_ctr_add(ctr, ctrs_pending[chan].rx_underruns - ctr->current);
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_RX_OVERRUNS];
		rate_ctr_add(ctr, ctrs_pending[chan].rx_overruns - ctr->current);
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_TX_UNDERRUNS];
		rate_ctr_add(ctr, ctrs_pending[chan].tx_underruns - ctr->current);
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_RX_DROP_EV];
		rate_ctr_add(ctr, ctrs_pending[chan].rx_dropped_events - ctr->current);
		ctr = &rate_ctrs[chan]->ctr[TRX_CTR_RX_DROP_SMPL];
		rate_ctr_add(ctr, ctrs_pending[chan].rx_dropped_samples - ctr->current);

		/* Mark as done */
		ctrs_pending[chan].chan = PENDING_CHAN_NONE;
	}
	if (osmo_timerfd_disable(&rate_ctr_timerfd) < 0)
		LOGC(DMAIN, ERROR) << "Failed to disable timerfd";
	rate_ctr_mutex.unlock();
	return 0;
}

/* Callback function to be called every time we receive a signal from DEVICE */
static int device_sig_cb(unsigned int subsys, unsigned int signal,
			 void *handler_data, void *signal_data)
{
	struct device_counters *ctr;
	/* Delay sched around 20 ms, in case we receive several calls from several
	 * channels batched */
	struct timespec next_sched = {.tv_sec = 0, .tv_nsec = 20*1000*1000};
	/* no automatic re-trigger */
	struct timespec intv_sched = {.tv_sec = 0, .tv_nsec = 0};

	switch (signal) {
	case S_DEVICE_COUNTER_CHANGE:
		ctr = (struct device_counters *)signal_data;
		LOGCHAN(ctr->chan, DMAIN, NOTICE) << "Received counter change from radioDevice";
		rate_ctr_mutex.lock();
		ctrs_pending[ctr->chan] = *ctr;
		if (osmo_timerfd_schedule(&rate_ctr_timerfd, &next_sched, &intv_sched) < 0) {
			LOGC(DMAIN, ERROR) << "Failed to schedule timerfd: " << errno << " = "<< strerror(errno);
		}
		rate_ctr_mutex.unlock();
		break;
	default:
		break;
	}
	return 0;
}

/* Init rate_ctr subsystem. Expected to be called during process start by main thread */
void trx_rate_ctr_init(void *ctx, struct trx_ctx* trx_ctx)
{
	size_t  i;
	chan_len = trx_ctx->cfg.num_chans;
	ctrs_pending = (struct device_counters*) talloc_zero_size(ctx, chan_len * sizeof(struct device_counters));
	rate_ctrs = (struct rate_ctr_group**) talloc_zero_size(ctx, chan_len * sizeof(struct rate_ctr_group*));

	for (i = 0; i < chan_len; i++) {
		ctrs_pending[i].chan = PENDING_CHAN_NONE;
		rate_ctrs[i] = rate_ctr_group_alloc(ctx, &trx_chan_ctr_group_desc, i);
		if (!rate_ctrs[i]) {
			LOGCHAN(i, DMAIN, ERROR) << "Failed to allocate rate ctr";
			exit(1);
		}
	}
	rate_ctr_timerfd.fd = -1;
	if (osmo_timerfd_setup(&rate_ctr_timerfd, rate_ctr_timerfd_cb, NULL) < 0) {
		LOGC(DMAIN, ERROR) << "Failed to setup timerfd";
		exit(1);
	}
	osmo_signal_register_handler(SS_DEVICE, device_sig_cb, NULL);
}
