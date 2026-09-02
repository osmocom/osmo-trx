/*! \file src/ctrl_cmd.c
 * osmo-trx-proxy: TRXC command handling. */

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

#include <stdio.h>
#include <string.h>

#include <osmocom/trx/ep.h>
#include <osmocom/trx/trxc.h>

#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/path_sim.h>
#include <osmocom/proxy/logging.h>

/* Not part of the well-known OSMO_TRXC_CMD_* verbs (libosmo-trx/trxc.h) since
 * they are specific to this transceiver's RF path simulation. */
#define CTRL_CMD_SETTA		"SETTA"
#define CTRL_CMD_FAKE_TOA	"FAKE_TOA"
#define CTRL_CMD_FAKE_RSSI	"FAKE_RSSI"
#define CTRL_CMD_FAKE_CI	"FAKE_CI"
#define CTRL_CMD_FAKE_DROP	"FAKE_DROP"

static void ctrl_cmd_poweron(struct proxy_trx *trx, struct osmo_trxc_msg *rsp)
{
	if (trx->powered) {
		LOGP_TRX(trx, DTRXC, LOGL_ERROR,
			 "Rx POWERON: already powered on\n");
		rsp->status = 1;
		return;
	}

	for (unsigned int chan = 0; chan < trx->num_chans; chan++) {
		if (trx->chans[chan].rx_freq == 0 || trx->chans[chan].tx_freq == 0) {
			LOGP_TRXCH(trx, chan, DTRXC, LOGL_ERROR,
				   "Rx POWERON: Rx/Tx frequency not (yet) set\n");
			rsp->status = 1;
			return;
		}
	}

	proxy_trx_set_power(trx, true);
}

static void ctrl_cmd_poweroff(struct proxy_trx *trx, struct osmo_trxc_msg *rsp)
{
	proxy_trx_set_power(trx, false);
}

static void ctrl_cmd_rxtune(struct proxy_trx *trx, unsigned int chan,
			    const struct osmo_trxc_msg *cmd, struct osmo_trxc_msg *rsp)
{
	unsigned int freq_khz = 0;

	if (osmo_trxc_msg_params_scan(cmd, "%u", &freq_khz) != 1) {
		LOGP_TRXCH(trx, chan, DTRXC, LOGL_ERROR,
			   "%s(): Failed to parse Rx frequency: '%s'\n",
			   __func__, osmo_trxc_msg_name(cmd));
		rsp->status = 1;
		return;
	}

	trx->chans[chan].rx_freq = freq_khz * 1000;
	LOGP_TRXCH(trx, chan, DTRXC, LOGL_INFO,
		   "Rx frequency set to %u kHz\n", freq_khz);
	snprintf(rsp->params, sizeof(rsp->params), "%u", freq_khz);
}

static void ctrl_cmd_txtune(struct proxy_trx *trx, unsigned int chan,
			    const struct osmo_trxc_msg *cmd, struct osmo_trxc_msg *rsp)
{
	unsigned int freq_khz = 0;

	if (osmo_trxc_msg_params_scan(cmd, "%u", &freq_khz) != 1) {
		LOGP_TRXCH(trx, chan, DTRXC, LOGL_ERROR,
			   "%s(): Failed to parse Tx frequency: '%s'\n",
			   __func__, osmo_trxc_msg_name(cmd));
		rsp->status = 1;
		return;
	}

	trx->chans[chan].tx_freq = freq_khz * 1000;
	LOGP_TRXCH(trx, chan, DTRXC, LOGL_INFO,
		   "Tx frequency set to %u kHz\n", freq_khz);
	snprintf(rsp->params, sizeof(rsp->params), "%u", freq_khz);
}

static void ctrl_cmd_setta(struct proxy_trx *trx, unsigned int chan,
			   const struct osmo_trxc_msg *cmd, struct osmo_trxc_msg *rsp)
{
	int ta;

	if (osmo_trxc_msg_params_scan(cmd, "%d", &ta) != 1) {
		LOGP_TRXCH(trx, chan, DTRXC, LOGL_ERROR,
			   "%s(): failed to parse Timing Advance: '%s'\n",
			   __func__, osmo_trxc_msg_name(cmd));
		rsp->status = 1;
		return;
	}

	trx->chans[chan].path_sim.ta = ta;

	LOGP_TRXCH(trx, chan, DTRXC, LOGL_INFO, "Timing Advance set to %d\n", ta);
}

static void ctrl_cmd_setpower(struct proxy_trx *trx, unsigned int chan,
			      const struct osmo_trxc_msg *cmd, struct osmo_trxc_msg *rsp)
{
	int att;

	if (osmo_trxc_msg_params_scan(cmd, "%d", &att) != 1) {
		LOGP_TRXCH(trx, chan, DTRXC, LOGL_ERROR,
			   "%s(): Failed to parse Tx power attenuation: '%s'\n",
			   __func__, osmo_trxc_msg_name(cmd));
		rsp->status = 1;
		return;
	}

	trx->chans[chan].path_sim.tx_att = att;

	LOGP_TRXCH(trx, chan, DTRXC, LOGL_INFO, "Tx power attenuation set to %d dB\n", att);
}

static void ctrl_cmd_nomtxpower(struct proxy_trx *trx, unsigned int chan, struct osmo_trxc_msg *rsp)
{
	snprintf(rsp->params, sizeof(rsp->params), "%d", trx->chans[chan].path_sim.tx_power);
}

static void ctrl_cmd_rfmute(struct proxy_trx *trx, unsigned int chan,
			    const struct osmo_trxc_msg *cmd, struct osmo_trxc_msg *rsp)
{
	int mute;

	if (osmo_trxc_msg_params_scan(cmd, "%d", &mute) != 1) {
		LOGP_TRXCH(trx, chan, DTRXC, LOGL_ERROR,
			   "%s(): Failed to parse command arguments: '%s'\n",
			   __func__, osmo_trxc_msg_name(cmd));
		rsp->status = 1;
		return;
	}

	trx->chans[chan].rf_muted = mute > 0;

	LOGP_TRXCH(trx, chan, DTRXC, LOGL_INFO,
		   "RF mute %s\n", trx->chans[chan].rf_muted ? "on" : "off");
}

/* FAKE_TOA/FAKE_RSSI/FAKE_CI: "<delta>" adjusts the current value by delta;
 * "<value> <threshold>" sets an absolute value with a +/-threshold random
 * jitter applied on every forwarded burst (see path_sim_apply()). */

static void ctrl_cmd_fake_toa(struct proxy_trx *trx, unsigned int chan,
			      const struct osmo_trxc_msg *cmd, struct osmo_trxc_msg *rsp)
{
	struct path_sim_state *ps = &trx->chans[chan].path_sim;
	int value, threshold;

	if (osmo_trxc_msg_params_scan(cmd, "%d %d", &value, &threshold) == 2) {
		if (threshold < 0) {
			rsp->status = 1;
			return;
		}
		ps->toa256 = value;
		ps->toa256_jitter = threshold;
	} else if (osmo_trxc_msg_params_scan(cmd, "%d", &value) == 1) {
		ps->toa256 += value;
	} else {
		LOGP_TRXCH(trx, chan, DTRXC, LOGL_ERROR,
			   "%s(): Failed to parse command arguments: '%s'\n",
			   __func__, osmo_trxc_msg_name(cmd));
		rsp->status = 1;
	}
}

static void ctrl_cmd_fake_rssi(struct proxy_trx *trx, unsigned int chan,
			       const struct osmo_trxc_msg *cmd, struct osmo_trxc_msg *rsp)
{
	struct path_sim_state *ps = &trx->chans[chan].path_sim;
	int value, threshold;

	if (osmo_trxc_msg_params_scan(cmd, "%d %d", &value, &threshold) == 2) {
		if (threshold < 0) {
			rsp->status = 1;
			return;
		}
		ps->rssi = value;
		ps->rssi_jitter = threshold;
	} else if (osmo_trxc_msg_params_scan(cmd, "%d", &value) == 1) {
		ps->rssi = value;
		ps->rssi_jitter = 0;
	} else {
		LOGP_TRXCH(trx, chan, DTRXC, LOGL_ERROR,
			   "%s(): Failed to parse command arguments: '%s'\n",
			   __func__, osmo_trxc_msg_name(cmd));
		rsp->status = 1;
		return;
	}

	ps->flags |= PATH_SIM_F_FAKE_RSSI;
}

static void ctrl_cmd_fake_ci(struct proxy_trx *trx, unsigned int chan,
			     const struct osmo_trxc_msg *cmd, struct osmo_trxc_msg *rsp)
{
	struct path_sim_state *ps = &trx->chans[chan].path_sim;
	int value, threshold;

	if (osmo_trxc_msg_params_scan(cmd, "%d %d", &value, &threshold) == 2) {
		if (threshold < 0) {
			rsp->status = 1;
			return;
		}
		ps->ci = value;
		ps->ci_jitter = threshold;
	} else if (osmo_trxc_msg_params_scan(cmd, "%d", &value) == 1) {
		ps->ci += value;
	} else {
		LOGP_TRXCH(trx, chan, DTRXC, LOGL_ERROR,
			   "%s(): Failed to parse command arguments: '%s'\n",
			   __func__, osmo_trxc_msg_name(cmd));
		rsp->status = 1;
	}
}

static void ctrl_cmd_fake_drop(struct proxy_trx *trx, unsigned int chan,
			       const struct osmo_trxc_msg *cmd, struct osmo_trxc_msg *rsp)
{
	struct path_sim_state *ps = &trx->chans[chan].path_sim;
	int amount, period;

	if (osmo_trxc_msg_params_scan(cmd, "%d %d", &amount, &period) == 2) {
		if (amount < 0 || period <= 0) {
			rsp->status = 1;
			return;
		}
	} else if (osmo_trxc_msg_params_scan(cmd, "%d", &amount) == 1) {
		if (amount < 0) {
			rsp->status = 1;
			return;
		}
		period = 1;
	} else {
		LOGP_TRXCH(trx, chan, DTRXC, LOGL_ERROR,
			   "%s(): Failed to parse command arguments: '%s'\n",
			   __func__, osmo_trxc_msg_name(cmd));
		rsp->status = 1;
		return;
	}

	ps->burst_drop_amount = amount;
	ps->burst_drop_period = period;

	LOGP_TRXCH(trx, chan, DTRXC, LOGL_INFO,
		  "Dropping %d burst(s), every %d frame(s)\n", amount, period);
}

void osmo_trx_ep_rx_ctrl_msg(struct osmo_trx_ep *ep, unsigned int chan,
			     const struct osmo_trxc_msg *cmd)
{
	struct proxy_trx *trx = osmo_trx_ep_get_priv(ep);
	struct osmo_trxc_msg rsp = *cmd;

	LOGP_TRXCH(trx, chan, DTRXC, LOGL_DEBUG,
		   "Rx '%s'\n", osmo_trxc_msg_name(cmd));

	rsp.type = OSMO_TRXC_MT_RSP;
	rsp.status = 0; /* ACK all commands by default */
	OSMO_STRLCPY_ARRAY(rsp.cmd, cmd->cmd);
	OSMO_STRLCPY_ARRAY(rsp.params, cmd->params);

	if (!strcmp(cmd->cmd, OSMO_TRXC_CMD_POWERON)) {
		ctrl_cmd_poweron(trx, &rsp);
	} else if (!strcmp(cmd->cmd, OSMO_TRXC_CMD_POWEROFF)) {
		ctrl_cmd_poweroff(trx, &rsp);
	} else if (!strcmp(cmd->cmd, OSMO_TRXC_CMD_RXTUNE)) {
		ctrl_cmd_rxtune(trx, chan, cmd, &rsp);
	} else if (!strcmp(cmd->cmd, OSMO_TRXC_CMD_TXTUNE)) {
		ctrl_cmd_txtune(trx, chan, cmd, &rsp);
	} else if (!strcmp(cmd->cmd, OSMO_TRXC_CMD_SETPOWER)) {
		ctrl_cmd_setpower(trx, chan, cmd, &rsp);
	} else if (!strcmp(cmd->cmd, OSMO_TRXC_CMD_NOMTXPOWER)) {
		ctrl_cmd_nomtxpower(trx, chan, &rsp);
	} else if (!strcmp(cmd->cmd, OSMO_TRXC_CMD_RFMUTE)) {
		ctrl_cmd_rfmute(trx, chan, cmd, &rsp);
	} else if (!strcmp(cmd->cmd, CTRL_CMD_SETTA)) {
		ctrl_cmd_setta(trx, chan, cmd, &rsp);
	} else if (!strcmp(cmd->cmd, CTRL_CMD_FAKE_TOA)) {
		ctrl_cmd_fake_toa(trx, chan, cmd, &rsp);
	} else if (!strcmp(cmd->cmd, CTRL_CMD_FAKE_RSSI)) {
		ctrl_cmd_fake_rssi(trx, chan, cmd, &rsp);
	} else if (!strcmp(cmd->cmd, CTRL_CMD_FAKE_CI)) {
		ctrl_cmd_fake_ci(trx, chan, cmd, &rsp);
	} else if (!strcmp(cmd->cmd, CTRL_CMD_FAKE_DROP)) {
		ctrl_cmd_fake_drop(trx, chan, cmd, &rsp);
	} else {
		LOGP_TRXCH(trx, chan, DTRXC, LOGL_INFO,
			   "Unhandled command '%s'\n", cmd->cmd);
	}

	LOGP_TRXCH(trx, chan, DTRXC, LOGL_DEBUG,
		   "Tx '%s'\n", osmo_trxc_msg_name(&rsp));

	osmo_trx_ep_send_ctrl_msg(ep, chan, &rsp);
}
