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

#include <osmocom/core/talloc.h>

#include <osmocom/trx/trxd.h>

#include <osmocom/proxy/proxy.h>
#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/path_sim.h>

/* Defaults for struct path_sim_cfg */
#define PATH_SIM_CFG_DEFAULT_NOISE_DBM		-110
#define PATH_SIM_CFG_DEFAULT_PATH_LOSS_DB	110
#define PATH_SIM_CFG_DEFAULT_TOA256		0
#define PATH_SIM_CFG_DEFAULT_CI_CB		90

/* Values reported for NOPE.ind (burst dropped / RF muted) */
#define PATH_SIM_TOA256_NOISE_DEFAULT		0
#define PATH_SIM_RSSI_NOISE_DEFAULT		-110
#define PATH_SIM_CI_NOISE_DEFAULT		-30

/*! Global RF path simulation configuration (see path_sim.h). */
struct path_sim_cfg {
	int noise_dbm;		/*!< dBm, MEASURE result for frequencies with no Tx found */
	int path_loss_db;	/*!< dB, RF path loss used by the RSSI formula */
	int nom_toa256;		/*!< default reported ToA for BURST.ind, 1/256 symbol periods */
	int nom_ci_cb;		/*!< default reported C/I for BURST.ind, in cB */
};

/*! Global RF path simulation configuration. Opaque; use the accessors below. */
struct path_sim_cfg *path_sim_cfg_alloc(void *talloc_ctx)
{
	struct path_sim_cfg *cfg;

	cfg = talloc_zero(talloc_ctx, struct path_sim_cfg);
	if (cfg == NULL)
		return NULL;

	cfg->noise_dbm = PATH_SIM_CFG_DEFAULT_NOISE_DBM;
	cfg->path_loss_db = PATH_SIM_CFG_DEFAULT_PATH_LOSS_DB;
	cfg->nom_toa256 = PATH_SIM_CFG_DEFAULT_TOA256;
	cfg->nom_ci_cb = PATH_SIM_CFG_DEFAULT_CI_CB;

	return cfg;
}

void path_sim_cfg_set_noise_dbm(struct path_sim_cfg *cfg, int noise_dbm)
{
	cfg->noise_dbm = noise_dbm;
}

int path_sim_cfg_get_noise_dbm(const struct path_sim_cfg *cfg)
{
	return cfg->noise_dbm;
}

void path_sim_cfg_set_path_loss_db(struct path_sim_cfg *cfg, int path_loss_db)
{
	cfg->path_loss_db = path_loss_db;
}

int path_sim_cfg_get_path_loss_db(const struct path_sim_cfg *cfg)
{
	return cfg->path_loss_db;
}

void path_sim_cfg_set_nom_toa256(struct path_sim_cfg *cfg, int nom_toa256)
{
	cfg->nom_toa256 = nom_toa256;
}

int path_sim_cfg_get_nom_toa256(const struct path_sim_cfg *cfg)
{
	return cfg->nom_toa256;
}

void path_sim_cfg_set_nom_ci_cb(struct path_sim_cfg *cfg, int nom_ci_cb)
{
	cfg->nom_ci_cb = nom_ci_cb;
}

int path_sim_cfg_get_nom_ci_cb(const struct path_sim_cfg *cfg)
{
	return cfg->nom_ci_cb;
}

/*! Reset a channel's RF path simulation state to the given nominal Tx power,
 * ToA and C/I defaults. */
void path_sim_state_reset(struct path_sim_state *ps, int tx_power, int toa256, int ci)
{
	ps->flags = 0;
	ps->tx_power = tx_power;
	ps->tx_att = 0;
	ps->ta = 0;
	ps->toa256 = toa256;
	ps->toa256_jitter = 0;
	ps->rssi = 0; /* unused unless PATH_SIM_F_FAKE_RSSI is set */
	ps->rssi_jitter = 0;
	ps->ci = ci;
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
		    const struct proxy_trx_chan *src,
		    const struct path_sim_cfg *cfg)
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
		bi->rssi = tx_power - (int)br->att - cfg->path_loss_db;
	}

	bi->ci_cb = dst->path_sim.ci + path_sim_jitter(dst->path_sim.ci_jitter);
	bi->flags |= OSMO_TRXD_F_CI_CB;
}

/*! Fill in a NOPE.ind for a timeslot where no BURST.req was received at all
 * this TDMA frame tick (as opposed to path_sim_apply(), which reports on an
 * actually forwarded burst that got dropped/muted along the way). Reports
 * the configured noise floor, same as path_sim_measure() would for an idle
 * frequency. */
void path_sim_fill_nope(struct osmo_trxd_burst_ind *bi, const struct path_sim_cfg *cfg)
{
	bi->flags |= OSMO_TRXD_F_NOPE_IND | OSMO_TRXD_F_CI_CB;
	bi->burst_len = 0;
	bi->toa256 = PATH_SIM_TOA256_NOISE_DEFAULT;
	bi->rssi = cfg->noise_dbm;
	bi->ci_cb = PATH_SIM_CI_NOISE_DEFAULT;
}

static int path_sim_measure_rssi(const struct proxy_trx_chan *tx, const struct path_sim_cfg *cfg)
{
	if (tx->path_sim.flags & PATH_SIM_F_FAKE_RSSI)
		return tx->path_sim.rssi;

	return (tx->path_sim.tx_power - tx->path_sim.tx_att) - cfg->path_loss_db;
}

/*! Emulate a power measurement (MEASURE CTRL command) on a given Tx
 * frequency: if some powered-on channel is currently transmitting on it,
 * return the RSSI it would be measured at (same path-loss formula, or
 * FAKE_RSSI override, as path_sim_apply()); otherwise return the configured
 * noise floor. */
int path_sim_measure(uint32_t freq_hz, const struct path_sim_cfg *cfg)
{
	struct proxy_trx *trx;

	llist_for_each_entry(trx, &g_proxy_ctx->trx_list, list) {
		unsigned int chan;

		if (!trx->powered)
			continue;

		for (chan = 0; chan < trx->num_chans; chan++) {
			if (trx->chans[chan].tx_freq == freq_hz)
				return path_sim_measure_rssi(&trx->chans[chan], cfg);
		}
	}

	return cfg->noise_dbm;
}
