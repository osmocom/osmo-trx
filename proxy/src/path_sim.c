/*! \file src/path_sim.c
 * osmo-trx-proxy: RF path simulation (ToA/RSSI/C-I, burst dropping, RF mute). */

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

#include <stdlib.h>

#include <osmocom/trx/trxd.h>

#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/path_sim.h>

#define PATH_SIM_NOMINAL_TX_POWER_DEFAULT	50	/* dBm */
#define PATH_SIM_TX_ATT_DEFAULT			0	/* dB */
#define PATH_SIM_PATH_LOSS_DEFAULT		110	/* dB */
#define PATH_SIM_CI_DEFAULT			90	/* cB */

/* Values reported for NOPE.ind (burst dropped / RF muted) */
#define PATH_SIM_TOA256_NOISE_DEFAULT		0
#define PATH_SIM_RSSI_NOISE_DEFAULT		-110
#define PATH_SIM_CI_NOISE_DEFAULT		-30

/*! Reset a channel's RF path simulation state to nominal defaults. */
void path_sim_state_reset(struct path_sim_state *ps)
{
	ps->flags = 0;
	ps->tx_power = PATH_SIM_NOMINAL_TX_POWER_DEFAULT;
	ps->tx_att = PATH_SIM_TX_ATT_DEFAULT;
	ps->ta = 0;
	ps->toa256 = 0;
	ps->toa256_jitter = 0;
	ps->rssi = PATH_SIM_NOMINAL_TX_POWER_DEFAULT -
		   PATH_SIM_TX_ATT_DEFAULT - PATH_SIM_PATH_LOSS_DEFAULT;
	ps->rssi_jitter = 0;
	ps->ci = PATH_SIM_CI_DEFAULT;
	ps->ci_jitter = 0;
	ps->burst_drop_amount = 0;
	ps->burst_drop_period = 1;
}

/* Path loss simulation: burst dropping. Returns true if the burst at the
 * given frame number is to be dropped (and consumes one drop credit). */
static bool path_sim_burst_drop(struct path_sim_state *dst, uint32_t fn)
{
	if (dst->burst_drop_amount == 0)
		return false;

	if (fn % dst->burst_drop_period == 0) {
		dst->burst_drop_amount--;
		return true;
	}

	return false;
}

/* Uniform random offset in [-threshold, +threshold], or 0 if threshold <= 0. */
static int path_sim_jitter(int threshold)
{
	if (threshold <= 0)
		return 0;

	return (rand() % (2 * threshold + 1)) - threshold;
}

/*! Fill in the RF path simulation results (ToA, RSSI, C/I, burst dropping,
 * RF mute) of a BURST.ind that is about to be forwarded from src to dst. */
void path_sim_apply(struct osmo_trxd_burst_ind *bi,
		    struct proxy_trx_chan *dst,
		    const struct osmo_trxd_burst_req *br,
		    const struct proxy_trx_chan *src)
{
	bool nope = bi->flags & OSMO_TRXD_F_NOPE_IND;

	if (src->rf_muted)
		nope = true;
	else if (!nope)
		nope = path_sim_burst_drop(&dst->path_sim, br->fn);

	if (nope) {
		bi->flags |= OSMO_TRXD_F_NOPE_IND | OSMO_TRXD_F_CI_CB;
		bi->burst_len = 0;
		bi->toa256 = PATH_SIM_TOA256_NOISE_DEFAULT;
		bi->rssi = PATH_SIM_RSSI_NOISE_DEFAULT;
		bi->ci_cb = PATH_SIM_CI_NOISE_DEFAULT;
		return;
	}

	bi->toa256 = dst->path_sim.toa256 + path_sim_jitter(dst->path_sim.toa256_jitter);
	if (src->path_sim.ta != 0)
		bi->toa256 -= src->path_sim.ta * 256;

	if (dst->path_sim.flags & PATH_SIM_F_FAKE_RSSI) {
		bi->rssi = dst->path_sim.rssi + path_sim_jitter(dst->path_sim.rssi_jitter);
	} else {
		int tx_power = src->path_sim.tx_power - src->path_sim.tx_att;
		bi->rssi = tx_power - (int)br->att - PATH_SIM_PATH_LOSS_DEFAULT;
	}

	bi->ci_cb = dst->path_sim.ci + path_sim_jitter(dst->path_sim.ci_jitter);
	bi->flags |= OSMO_TRXD_F_CI_CB;
}
