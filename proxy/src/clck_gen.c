/*! \file src/clck_gen.c
 * osmo-trx-proxy: shared TDMA clock generator. */

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
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <inttypes.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/select.h>
#include <osmocom/core/timer_compat.h>
#include <osmocom/gsm/gsm0502.h>

#include <osmocom/trx/ep.h>

#include <osmocom/proxy/proxy.h>
#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/clck_gen.h>
#include <osmocom/proxy/burst_fwd.h>
#include <osmocom/proxy/logging.h>

/*! Default "IND CLOCK" period, in frames */
#define CLCK_GEN_IND_PERIOD	102

/*! Maximum number of 'missed' frame periods we can tolerate before assuming
 * the PC clock skewed (e.g. someone changed the system time, or the process
 * got stuck/suspended for a while). */
#define CLCK_GEN_MAX_FN_SKEW	50

struct clck_gen {
	struct osmo_fd timerfd;
	uint32_t fn;
	bool running;
	int start_fn;			/*!< CLCK_GEN_START_FN_RANDOM, or a fixed FN */
	uint32_t ind_period;		/*!< send "IND CLOCK" every N frames */
	struct timespec last_tick;	/*!< CLOCK_MONOTONIC time of the last timer_cb() */
};

/*! Number of micro-seconds elapsed between \a last and \a now */
static int64_t compute_elapsed_us(const struct timespec *last, const struct timespec *now)
{
	struct timespec elapsed;

	timespecsub(now, last, &elapsed);
	return (int64_t)(elapsed.tv_sec * 1000000) + (elapsed.tv_nsec / 1000);
}

static struct clck_gen g_clck_gen = {
	.start_fn = CLCK_GEN_START_FN_RANDOM,
	.ind_period = CLCK_GEN_IND_PERIOD,
};

static void clck_gen_stop(struct clck_gen *gen);

static int clck_gen_timer_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct clck_gen *gen = ofd->data;
	struct timespec tv_now;
	uint64_t expire_count;
	int64_t elapsed_us;

	if (read(ofd->fd, &expire_count, sizeof(expire_count)) != sizeof(expire_count))
		return 0;

	/* check for PC clock skew (system time change, process stalled/suspended, ...) */
	clock_gettime(CLOCK_MONOTONIC, &tv_now);
	elapsed_us = compute_elapsed_us(&gen->last_tick, &tv_now);
	gen->last_tick = tv_now;
	if (elapsed_us > GSM_TDMA_FN_DURATION_uS * CLCK_GEN_MAX_FN_SKEW || elapsed_us < 0) {
		LOGP(DTRXC, LOGL_FATAL,
		     "PC clock skew too high (elapsed %" PRId64 " us): "
		     "stopping the TDMA clock generator\n",
		     elapsed_us);
		clck_gen_stop(gen);
		return 0;
	}

	while (expire_count-- > 0) {
		const struct proxy_trx *trx;

		if (gen->fn % gen->ind_period == 0) {
			llist_for_each_entry(trx, &g_proxy_ctx->trx_list, list) {
				if (!trx->powered)
					continue;
				if (!osmo_trx_ep_get_clock_socket(trx->ep))
					continue;
				LOGP_TRX(trx, DTRXC, LOGL_DEBUG,
					 "Tx CLCK.ind (fn=%u)\n", gen->fn);
				osmo_trx_ep_send_clck_ind(trx->ep, gen->fn);
			}
		}

		burst_fwd_dispatch(gen->fn);

		GSM_TDMA_FN_INC(gen->fn);
	}

	return 0;
}

static void clck_gen_start(struct clck_gen *gen)
{
	const struct timespec first = { .tv_nsec = GSM_TDMA_FN_DURATION_nS };
	const struct timespec interval = { .tv_nsec = GSM_TDMA_FN_DURATION_nS };

	if (gen->running)
		return;

	if (gen->start_fn == CLCK_GEN_START_FN_RANDOM)
		gen->fn = rand() % GSM_TDMA_HYPERFRAME;
	else
		gen->fn = (uint32_t)gen->start_fn;

	clock_gettime(CLOCK_MONOTONIC, &gen->last_tick);

	if (osmo_timerfd_schedule(&gen->timerfd, &first, &interval) < 0) {
		LOGP(DTRXC, LOGL_ERROR, "Failed to start the TDMA clock generator\n");
		return;
	}

	gen->running = true;
	LOGP(DTRXC, LOGL_NOTICE, "TDMA clock generator started (fn=%u, period=%u)\n",
	     gen->fn, gen->ind_period);
}

static void clck_gen_stop(struct clck_gen *gen)
{
	if (!gen->running)
		return;

	osmo_timerfd_disable(&gen->timerfd);
	gen->running = false;
	LOGP(DTRXC, LOGL_NOTICE, "TDMA clock generator stopped\n");
}

/*! Re-evaluate g_proxy_ctx->trx_list and (re)start or stop the (single,
 * process-wide) TDMA clock generator: running iff at least one endpoint is
 * currently powered on.  Call this after changing any struct proxy_trx's
 * ->powered field. */
void clck_gen_trx_list_updated(void)
{
	bool any_active = false;
	struct proxy_trx *trx;

	llist_for_each_entry(trx, &g_proxy_ctx->trx_list, list) {
		any_active |= trx->powered;
	}

	if (any_active)
		clck_gen_start(&g_clck_gen);
	else
		clck_gen_stop(&g_clck_gen);
}

int clck_gen_init(void)
{
	g_clck_gen.timerfd.fd = -1;
	return osmo_timerfd_setup(&g_clck_gen.timerfd,
				  &clck_gen_timer_cb,
				  &g_clck_gen);
}

/*! Set the frame number the clock generator starts counting from the next
 * time it (re)starts: either a fixed FN, or CLCK_GEN_START_FN_RANDOM to
 * pick a random one (default).  Returns 0 on success, -EINVAL if fn is
 * neither CLCK_GEN_START_FN_RANDOM nor a valid FN. */
int clck_gen_set_start_fn(int fn)
{
	if (fn < CLCK_GEN_START_FN_RANDOM || fn >= GSM_TDMA_HYPERFRAME)
		return -EINVAL;

	g_clck_gen.start_fn = fn;
	return 0;
}

/*! Get the currently configured starting frame number, or
 * CLCK_GEN_START_FN_RANDOM if set to pick a random one. */
int clck_gen_get_start_fn(void)
{
	return g_clck_gen.start_fn;
}

/*! Set how many frames apart "IND CLOCK" is sent (default: 102).  Returns 0
 * on success, -EINVAL if period is 0 or exceeds the TDMA hyperframe length. */
int clck_gen_set_ind_period(unsigned int period)
{
	if (period == 0 || period > GSM_TDMA_HYPERFRAME)
		return -EINVAL;

	g_clck_gen.ind_period = period;
	return 0;
}

/*! Get the currently configured "IND CLOCK" period, in frames. */
unsigned int clck_gen_get_ind_period(void)
{
	return g_clck_gen.ind_period;
}
