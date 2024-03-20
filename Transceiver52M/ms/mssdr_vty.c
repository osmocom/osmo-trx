/*
 * (C) 2024 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
 * All Rights Reserved
 *
 * Author: Eric Wild <ewild@sysmocom.de>
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
 *
 */

#include <osmocom/vty/command.h>
#include <osmocom/vty/logging.h>
#include "../config.h"
#include "mssdr_vty.h"

static struct mssdr_ctx *g_mssdr_ctx;

enum mssdr_vty_node {
	MSSDR_NODE = _LAST_OSMOVTY_NODE + 1,
};

static const char mssdr_copyright[] =
	"Copyright (C) 2007-2014 Free Software Foundation, Inc.\r\n"
	"Copyright (C) 2013 Thomas Tsou <tom@tsou.cc>\r\n"
	"Copyright (C) 2013-2019 Fairwaves, Inc.\r\n"
	"Copyright (C) 2015 Ettus Research LLC\r\n"
	"Copyright (C) 2017-2024 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>\r\n"
	"License AGPLv3+: GNU AGPL version 3 or later <http://gnu.org/licenses/agpl-3.0.html>\r\n"
	"This is free software: you are free to change and redistribute it.\r\n"
	"There is NO WARRANTY, to the extent permitted by law.\r\n";

static int mssdr_vty_go_parent(struct vty *vty)
{
	switch (vty->node) {
	case MSSDR_NODE:
		vty->node = CONFIG_NODE;
		vty->index = NULL;
		vty->index_sub = NULL;
		break;
	default:
		vty->node = CONFIG_NODE;
		vty->index = NULL;
		vty->index_sub = NULL;
	}

	return vty->node;
}

struct mssdr_ctx *mssdr_from_vty(struct vty *v)
{
	OSMO_ASSERT(g_mssdr_ctx);
	return g_mssdr_ctx;
}

struct vty_app_info g_mssdr_vty_info = {
	.name = "OsmoMSSDR",
	.version = PACKAGE_VERSION,
	.copyright = mssdr_copyright,
	.go_parent_cb = mssdr_vty_go_parent,
};

struct mssdr_ctx *vty_mssdr_ctx_alloc(void *talloc_ctx)
{
	struct mssdr_ctx *trx = talloc_zero(talloc_ctx, struct mssdr_ctx);
	trx->cfg.use_va = true;
	trx->cfg.use_agc = true;
	return trx;
}

static void mssdr_dump_vty(struct vty *vty, struct mssdr_ctx *trx)
{
	// vty_out(vty, "TRX Config:%s", VTY_NEWLINE);
	// vty_out(vty, " Local IP: %s%s", trx->cfg.bind_addr, VTY_NEWLINE);
	// vty_out(vty, " Remote IP: %s%s", trx->cfg.remote_addr, VTY_NEWLINE);
	// vty_out(vty, " TRX Base Port: %u%s", trx->cfg.base_port, VTY_NEWLINE);
	// vty_out(vty, " Device args: %s%s", trx->cfg.dev_args, VTY_NEWLINE);
	// vty_out(vty, " Tx Samples-per-Symbol: %u%s", trx->cfg.tx_sps, VTY_NEWLINE);
	// vty_out(vty, " Rx Samples-per-Symbol: %u%s", trx->cfg.rx_sps, VTY_NEWLINE);
	// vty_out(vty, " Filler Burst Type: %s%s", get_value_string(filler_names, trx->cfg.filler), VTY_NEWLINE);
	vty_out(vty, "trx%s", VTY_NEWLINE);
	if (trx->cfg.overrides.dl_freq_override)
		vty_out(vty, " dl-freq-override %f%s", trx->cfg.overrides.dl_freq, VTY_NEWLINE);
	if (trx->cfg.overrides.ul_freq_override)
		vty_out(vty, " ul-freq-override %f%s", trx->cfg.overrides.ul_freq, VTY_NEWLINE);
	if (trx->cfg.overrides.dl_gain_override)
		vty_out(vty, " dl-gain-override %f%s", trx->cfg.overrides.dl_gain, VTY_NEWLINE);
	if (trx->cfg.overrides.ul_gain_override)
		vty_out(vty, " ul-gain-override %f%s", trx->cfg.overrides.ul_gain, VTY_NEWLINE);
	if (trx->cfg.use_va)
		vty_out(vty, " viterbi-eq %s%s", trx->cfg.use_va ? "enable" : "disable", VTY_NEWLINE);
	if (trx->cfg.use_agc)
		vty_out(vty, " rx-agc %s%s", trx->cfg.use_agc ? "enable" : "disable", VTY_NEWLINE);
}

static int config_write_mssdr(struct vty *vty)
{
	struct mssdr_ctx *trx = mssdr_from_vty(vty);

	vty_out(vty, "trx%s", VTY_NEWLINE);
	if (trx->cfg.overrides.dl_freq_override)
		vty_out(vty, " dl-freq-override %f%s", trx->cfg.overrides.dl_freq, VTY_NEWLINE);
	if (trx->cfg.overrides.ul_freq_override)
		vty_out(vty, " ul-freq-override %f%s", trx->cfg.overrides.ul_freq, VTY_NEWLINE);
	if (trx->cfg.overrides.dl_gain_override)
		vty_out(vty, " dl-gain-override %f%s", trx->cfg.overrides.dl_gain, VTY_NEWLINE);
	if (trx->cfg.overrides.ul_gain_override)
		vty_out(vty, " ul-gain-override %f%s", trx->cfg.overrides.ul_gain, VTY_NEWLINE);
	if (trx->cfg.use_va)
		vty_out(vty, " viterbi-eq %s%s", trx->cfg.use_va ? "enable" : "disable", VTY_NEWLINE);
	if (trx->cfg.use_agc)
		vty_out(vty, " rx-agc %s%s", trx->cfg.use_agc ? "enable" : "disable", VTY_NEWLINE);
	return CMD_SUCCESS;
}

DEFUN(show_mssdr, show_mssdr_cmd,
	"show mssdr",
	SHOW_STR "Display information on the TRX\n")
{
	struct mssdr_ctx *trx = mssdr_from_vty(vty);

	mssdr_dump_vty(vty, trx);

	return CMD_SUCCESS;
}

DEFUN(cfg_mssdr, cfg_mssdr_cmd,
	"mssdr",
	"Configure the mssdr\n")
{
	struct mssdr_ctx *trx = mssdr_from_vty(vty);

	if (!trx)
		return CMD_WARNING;

	vty->node = MSSDR_NODE;

	return CMD_SUCCESS;
}

DEFUN_ATTR(cfg_ul_freq_override, cfg_ul_freq_override_cmd,
	   "ul-freq-override FLOAT",
	   "Overrides Tx carrier frequency\n"
	   "Frequency in Hz (e.g. 145300000)\n",
	   CMD_ATTR_HIDDEN)
{
	struct mssdr_ctx *trx = mssdr_from_vty(vty);

	trx->cfg.overrides.ul_freq_override = true;
	trx->cfg.overrides.ul_freq = atof(argv[0]);

	return CMD_SUCCESS;
}
DEFUN_ATTR(cfg_dl_freq_override, cfg_dl_freq_override_cmd,
	   "dl-freq-override FLOAT",
	   "Overrides Rx carrier frequency\n"
	   "Frequency in Hz (e.g. 145300000)\n",
	   CMD_ATTR_HIDDEN)
{
	struct mssdr_ctx *trx = mssdr_from_vty(vty);

	trx->cfg.overrides.dl_freq_override = true;
	trx->cfg.overrides.dl_freq = atof(argv[0]);

	return CMD_SUCCESS;
}

DEFUN_ATTR(cfg_ul_gain_override, cfg_ul_gain_override_cmd,
	   "ul-gain-override FLOAT",
	   "Overrides Tx gain\n"
	   "gain in dB\n",
	   CMD_ATTR_HIDDEN)
{
	struct mssdr_ctx *trx = mssdr_from_vty(vty);

	trx->cfg.overrides.ul_gain_override = true;
	trx->cfg.overrides.ul_gain = atof(argv[0]);

	return CMD_SUCCESS;
}
DEFUN_ATTR(cfg_dl_gain_override, cfg_dl_gain_override_cmd,
	   "dl-gain-override FLOAT",
	   "Overrides Rx gain\n"
	   "gain in dB\n",
	   CMD_ATTR_HIDDEN)
{
	struct mssdr_ctx *trx = mssdr_from_vty(vty);

	trx->cfg.overrides.dl_gain_override = true;
	trx->cfg.overrides.dl_gain = atof(argv[0]);

	return CMD_SUCCESS;
}

DEFUN_ATTR(cfg_use_viterbi, cfg_use_viterbi_cmd,
	"viterbi-eq (disable|enable)",
	"Use viterbi equalizer for gmsk (default=enable)\n"
	"Disable VA\n"
	"Enable VA\n",
	CMD_ATTR_HIDDEN)
{
	struct mssdr_ctx *trx = mssdr_from_vty(vty);

	if (strcmp("disable", argv[0]) == 0)
		trx->cfg.use_va = false;
	else if (strcmp("enable", argv[0]) == 0)
		trx->cfg.use_va = true;
	else
		return CMD_WARNING;

	return CMD_SUCCESS;
}

DEFUN_ATTR(cfg_use_agc, cfg_use_agc_cmd,
	"rx-agc (disable|enable)",
	"Use the transceiver rx agc (default=enable)\n"
	"Disable agc\n"
	"Enable agc\n",
	CMD_ATTR_HIDDEN)
{
	struct mssdr_ctx *trx = mssdr_from_vty(vty);

	if (strcmp("disable", argv[0]) == 0)
		trx->cfg.use_agc = false;
	else if (strcmp("enable", argv[0]) == 0)
		trx->cfg.use_agc = true;
	else
		return CMD_WARNING;

	return CMD_SUCCESS;
}

static struct cmd_node mssdr_node = {
	MSSDR_NODE,
	"%s(config-mssdr)# ",
	1,
};

int mssdr_vty_init(struct mssdr_ctx *trx)
{
	g_mssdr_ctx = trx;
	install_element_ve(&show_mssdr_cmd);
	install_element(CONFIG_NODE, &cfg_mssdr_cmd);

	install_node(&mssdr_node, config_write_mssdr);
	install_element(MSSDR_NODE, &cfg_ul_freq_override_cmd);
	install_element(MSSDR_NODE, &cfg_dl_freq_override_cmd);
	install_element(MSSDR_NODE, &cfg_ul_gain_override_cmd);
	install_element(MSSDR_NODE, &cfg_dl_gain_override_cmd);
	install_element(MSSDR_NODE, &cfg_use_viterbi_cmd);
	install_element(MSSDR_NODE, &cfg_use_agc_cmd);

	return 0;
}
