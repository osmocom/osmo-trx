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

#include <unistd.h>
#include <stdint.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/select.h>
#include <osmocom/gsm/gsm0502.h>

#include <osmocom/trx/ep.h>

#include <osmocom/proxy/proxy.h>
#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/clck_gen.h>
#include <osmocom/proxy/logging.h>

/*! Send "IND CLOCK" every N frames (matches fake_trx.py's CLCKGen default) */
#define CLCK_GEN_IND_PERIOD	102

struct clck_gen {
	struct osmo_fd timerfd;
	uint32_t fn;
	bool running;
};

static struct clck_gen g_clck_gen;

static int clck_gen_timer_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct clck_gen *gen = ofd->data;
	uint64_t expire_count;

	if (read(ofd->fd, &expire_count, sizeof(expire_count)) != sizeof(expire_count))
		return 0;

	while (expire_count-- > 0) {
		const struct proxy_trx *trx;

		if (gen->fn % CLCK_GEN_IND_PERIOD == 0) {
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

		/* TODO: drive per-frame burst forwarding (burst_queue/burst_fwd) */

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

	gen->fn = 0;
	if (osmo_timerfd_schedule(&gen->timerfd, &first, &interval) < 0) {
		LOGP(DTRXC, LOGL_ERROR, "Failed to start the TDMA clock generator\n");
		return;
	}

	gen->running = true;
	LOGP(DTRXC, LOGL_NOTICE, "TDMA clock generator started\n");
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
