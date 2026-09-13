/*! \file src/vty.c
 * osmo-trx-proxy: VTY configuration interface. */

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
#include <stdlib.h>
#include <string.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>

#include <osmocom/vty/command.h>
#include <osmocom/vty/vty.h>
#include <osmocom/vty/misc.h>
#include <osmocom/vty/logging.h>
#include <osmocom/vty/cpu_sched_vty.h>

#include <osmocom/trx/ep.h>

#include <osmocom/proxy/proxy.h>
#include <osmocom/proxy/vty.h>
#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/path_sim.h>
#include <osmocom/proxy/clck_gen.h>

extern void *g_talloc_ctx;

enum proxy_vty_node {
	PROXY_NODE = _LAST_OSMOVTY_NODE + 1,
	EP_NODE,
};

static struct cmd_node proxy_node = {
	PROXY_NODE,
	"%s(config-proxy)# ",
	1,
};

static struct cmd_node ep_node = {
	EP_NODE,
	"%s(config-proxy-ep)# ",
	1,
};

static int dummy_config_write(struct vty *vty)
{
	return CMD_SUCCESS;
}

DEFUN(cfg_proxy,
      cfg_proxy_cmd,
      "proxy",
      "Configure osmo-trx-proxy\n")
{
	vty->node = PROXY_NODE;
	vty->index = g_proxy_ctx;

	return CMD_SUCCESS;
}

DEFUN(cfg_proxy_bind_addr,
      cfg_proxy_bind_addr_cmd,
      "bind-addr " VTY_IPV46_CMD,
      "Set the default local bind address for endpoints without one of their own\n"
      "IPv4 Address\n"
      "IPv6 Address\n")
{
	struct proxy_ctx *proxy = vty->index;

	osmo_talloc_replace_string(proxy, &proxy->bind_addr, argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_proxy_pm_rssi_noise,
      cfg_proxy_pm_rssi_noise_cmd,
      "pm-rssi-noise <-150-0>",
      "Set the RSSI reported by MEASURE for a frequency with no Tx found\n"
      "RSSI in dBm\n")
{
	struct proxy_ctx *proxy = vty->index;

	path_sim_cfg_set_noise_dbm(proxy->path_sim, atoi(argv[0]));

	return CMD_SUCCESS;
}

DEFUN(cfg_proxy_path_loss,
      cfg_proxy_path_loss_cmd,
      "path-loss <0-200>",
      "Set the simulated RF path loss used to compute the reported RSSI\n"
      "Path loss in dB\n")
{
	struct proxy_ctx *proxy = vty->index;

	path_sim_cfg_set_path_loss_db(proxy->path_sim, atoi(argv[0]));

	return CMD_SUCCESS;
}

DEFUN(cfg_proxy_nom_toa256,
      cfg_proxy_nom_toa256_cmd,
      "nominal-toa256 <-32768-32767>",
      "Set the default reported ToA for a forwarded burst\n"
      "ToA in 1/256 of a symbol period\n")
{
	struct proxy_ctx *proxy = vty->index;

	path_sim_cfg_set_nom_toa256(proxy->path_sim, atoi(argv[0]));

	return CMD_SUCCESS;
}

DEFUN(cfg_proxy_nom_ci,
      cfg_proxy_nom_ci_cmd,
      "nominal-ci <-1280-1280>",
      "Set the default reported C/I for a forwarded burst\n"
      "C/I in centiBels\n")
{
	struct proxy_ctx *proxy = vty->index;

	path_sim_cfg_set_nom_ci_cb(proxy->path_sim, atoi(argv[0]));

	return CMD_SUCCESS;
}

DEFUN(cfg_proxy_clck_gen_start_fn,
      cfg_proxy_clck_gen_start_fn_cmd,
      "clck-gen start-fn (random|<0-2715647>)",
      "Configure the shared TDMA clock generator\n"
      "Set the frame number the clock generator starts counting from\n"
      "Start from a random frame number every time it (re)starts (default)\n"
      "Start from this fixed frame number\n")
{
	int fn = !strcmp(argv[0], "random") ? CLCK_GEN_START_FN_RANDOM : atoi(argv[0]);

	if (clck_gen_set_start_fn(fn) < 0) {
		vty_out(vty, "%% Invalid starting frame number: %d%s", fn, VTY_NEWLINE);
		return CMD_WARNING;
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_proxy_clck_gen_ind_period,
      cfg_proxy_clck_gen_ind_period_cmd,
      "clck-gen ind-period <1-2715648>",
      "Configure the shared TDMA clock generator\n"
      "Set how many frames apart \"IND CLOCK\" is sent (default: 102)\n"
      "Period in frames\n")
{
	unsigned int period = atoi(argv[0]);

	if (clck_gen_set_ind_period(period) < 0) {
		vty_out(vty, "%% Invalid IND CLOCK period: %u%s", period, VTY_NEWLINE);
		return CMD_WARNING;
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_proxy_ep,
      cfg_proxy_ep_cmd,
      "ep NAME",
      "Configure a virtual TRX endpoint\n"
      "Endpoint name (used for logging)\n")
{
	struct proxy_ctx *proxy = vty->index;
	struct proxy_trx *trx = proxy_trx_find(proxy, argv[0]);

	if (!trx)
		trx = proxy_trx_alloc(proxy, argv[0]);

	vty->node = EP_NODE;
	vty->index = trx;

	return CMD_SUCCESS;
}

DEFUN(cfg_no_proxy_ep,
      cfg_no_proxy_ep_cmd,
      "no ep NAME",
      NO_STR "Remove a virtual TRX endpoint, closing its sockets if currently open\n"
      "Endpoint name\n")
{
	struct proxy_ctx *proxy = vty->index;
	struct proxy_trx *trx = proxy_trx_find(proxy, argv[0]);

	if (!trx) {
		vty_out(vty, "%% Endpoint '%s' does not exist%s", argv[0], VTY_NEWLINE);
		return CMD_WARNING;
	}

	proxy_trx_free(trx);

	return CMD_SUCCESS;
}

DEFUN(cfg_ep_remote_addr,
      cfg_ep_remote_addr_cmd,
      "remote-addr " VTY_IPV46_CMD,
      "Set the remote (L1 peer) IP address\n"
      "IPv4 Address\n"
      "IPv6 Address\n")
{
	struct proxy_trx *trx = vty->index;

	osmo_trx_ep_set_raddr(trx->ep, argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_ep_bind_addr,
      cfg_ep_bind_addr_cmd,
      "bind-addr " VTY_IPV46_CMD,
      "Set the local bind address for this endpoint, overriding the 'proxy' default\n"
      "IPv4 Address\n"
      "IPv6 Address\n")
{
	struct proxy_trx *trx = vty->index;

	osmo_trx_ep_set_laddr(trx->ep, argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_ep_base_port,
      cfg_ep_base_port_cmd,
      "base-port <1-65535>",
      "Set the TRX base port\n"
      "Base port (clock: +0, per channel N: ctrl +2N+1, data +2N+2)\n")
{
	struct proxy_trx *trx = vty->index;

	osmo_trx_ep_set_base_port(trx->ep, atoi(argv[0]));

	return CMD_SUCCESS;
}

DEFUN(cfg_ep_num_chans,
      cfg_ep_num_chans_cmd,
      "num-chans <1-16>",
      "Set the number of channels (1 + number of child transceivers)\n"
      "Number of channels\n")
{
	struct proxy_trx *trx = vty->index;
	int num_chans = atoi(argv[0]);
	int rc;

	rc = proxy_trx_set_num_chans(trx, num_chans);
	if (rc) {
		vty_out(vty, "%% proxy_trx_set_num_chans(%d) failed: rc=%d%s",
			num_chans, rc, VTY_NEWLINE);
		return CMD_WARNING;
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_ep_tx_power,
      cfg_ep_tx_power_cmd,
      "tx-power <-100-100>",
      "Set the nominal Tx power reported by NOMTXPOWER and used to compute "
      "the RSSI of bursts sent from this endpoint\n"
      "Tx power in dBm\n")
{
	struct proxy_trx *trx = vty->index;

	trx->tx_power = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_ep_trxd_max_version,
      cfg_ep_trxd_max_version_cmd,
      "trxd-max-version <0-" OSMO_STRINGIFY_VAL(OSMO_TRXD_PDU_VER_MAX) ">",
      "Set the highest TRXD PDU version SETFORMAT may negotiate "
      "(default: " OSMO_STRINGIFY_VAL(OSMO_TRXD_PDU_VER_MAX) ")\n"
      "TRXD PDU version\n")
{
	struct proxy_trx *trx = vty->index;

	trx->trxd_max_ver = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_ep_trxd_pdu_batch,
      cfg_ep_trxd_pdu_batch_cmd,
      "trxd-pdu-batch",
      "Batch all BURST.ind PDUs of a TDMA frame into one datagram "
      "(TRXDv2 and higher, default)\n")
{
	struct proxy_trx *trx = vty->index;

	osmo_trx_ep_set_pdu_batch(trx->ep, true);

	return CMD_SUCCESS;
}

DEFUN(cfg_ep_no_trxd_pdu_batch,
      cfg_ep_no_trxd_pdu_batch_cmd,
      "no trxd-pdu-batch",
      NO_STR "Send one datagram per BURST.ind PDU instead of batching a "
      "TDMA frame's worth into one\n")
{
	struct proxy_trx *trx = vty->index;

	osmo_trx_ep_set_pdu_batch(trx->ep, false);

	return CMD_SUCCESS;
}

DEFUN(cfg_ep_clock_socket,
      cfg_ep_clock_socket_cmd,
      "clock-socket",
      "Enable the clock socket for this endpoint (default)\n")
{
	struct proxy_trx *trx = vty->index;

	osmo_trx_ep_set_clock_socket(trx->ep, true);

	return CMD_SUCCESS;
}

DEFUN(cfg_ep_no_clock_socket,
      cfg_ep_no_clock_socket_cmd,
      "no clock-socket",
      NO_STR "Disable the clock socket for this endpoint\n")
{
	struct proxy_trx *trx = vty->index;

	osmo_trx_ep_set_clock_socket(trx->ep, false);

	return CMD_SUCCESS;
}

static int config_write_proxy(struct vty *vty)
{
	struct proxy_trx *trx;

	vty_out(vty, "proxy%s", VTY_NEWLINE);
	if (g_proxy_ctx->bind_addr)
		vty_out(vty, " bind-addr %s%s", g_proxy_ctx->bind_addr, VTY_NEWLINE);
	if (clck_gen_get_start_fn() == CLCK_GEN_START_FN_RANDOM)
		vty_out(vty, " clck-gen start-fn random%s", VTY_NEWLINE);
	else
		vty_out(vty, " clck-gen start-fn %d%s", clck_gen_get_start_fn(), VTY_NEWLINE);
	vty_out(vty, " clck-gen ind-period %u%s", clck_gen_get_ind_period(), VTY_NEWLINE);
	vty_out(vty, " pm-rssi-noise %d%s", path_sim_cfg_get_noise_dbm(g_proxy_ctx->path_sim), VTY_NEWLINE);
	vty_out(vty, " path-loss %d%s", path_sim_cfg_get_path_loss_db(g_proxy_ctx->path_sim), VTY_NEWLINE);
	vty_out(vty, " nominal-toa256 %d%s", path_sim_cfg_get_nom_toa256(g_proxy_ctx->path_sim), VTY_NEWLINE);
	vty_out(vty, " nominal-ci %d%s", path_sim_cfg_get_nom_ci_cb(g_proxy_ctx->path_sim), VTY_NEWLINE);

	llist_for_each_entry(trx, &g_proxy_ctx->trx_list, list) {
		const char *raddr = osmo_trx_ep_get_raddr(trx->ep);
		const char *laddr = osmo_trx_ep_get_laddr(trx->ep);

		vty_out(vty, " ep %s%s", trx->name, VTY_NEWLINE);
		if (raddr)
			vty_out(vty, "  remote-addr %s%s", raddr, VTY_NEWLINE);
		if (laddr && strcmp(laddr, g_proxy_ctx->bind_addr) != 0)
			vty_out(vty, "  bind-addr %s%s", laddr, VTY_NEWLINE);
		vty_out(vty, "  base-port %u%s", osmo_trx_ep_get_base_port(trx->ep), VTY_NEWLINE);
		vty_out(vty, "  num-chans %u%s", trx->num_chans, VTY_NEWLINE);
		vty_out(vty, "  tx-power %d%s", trx->tx_power, VTY_NEWLINE);
		if (trx->trxd_max_ver < OSMO_TRXD_PDU_VER_MAX)
			vty_out(vty, "  trxd-max-version %u%s", trx->trxd_max_ver, VTY_NEWLINE);
		if (!osmo_trx_ep_get_pdu_batch(trx->ep))
			vty_out(vty, "  no trxd-pdu-batch%s", VTY_NEWLINE);

		if (!osmo_trx_ep_get_clock_socket(trx->ep))
			vty_out(vty, "  no clock-socket%s", VTY_NEWLINE);
	}

	return CMD_SUCCESS;
}

DEFUN(show_proxy,
      show_proxy_cmd,
      "show proxy",
      SHOW_STR "Display configured virtual TRX endpoints\n")
{
	struct proxy_trx *trx;

	llist_for_each_entry(trx, &g_proxy_ctx->trx_list, list) {
		vty_out(vty, "Endpoint '%s': %s:%u, %u channel(s), %s%s",
			trx->name,
			osmo_trx_ep_get_raddr(trx->ep),
			osmo_trx_ep_get_base_port(trx->ep),
			osmo_trx_ep_get_num_chans(trx->ep),
			osmo_trx_ep_is_open(trx->ep) ? "open" : "closed",
			VTY_NEWLINE);
	}

	return CMD_SUCCESS;
}

/*! Called when leaving EP_NODE (interactively via 'exit'/'end', or on dedent
 * while reading a config file): validate the endpoint config and open it,
 * unless it is already open.  Returns 0 on success, non-zero if the node
 * should not be left (so the user/config can fix the problem). */
static int proxy_vty_ep_node_exit(struct vty *vty, struct proxy_trx *trx)
{
	int rc = proxy_trx_open(trx);

	if (rc < 0 && rc != -EALREADY) {
		vty_out(vty, "%% Endpoint '%s': failed to open%s",
			trx->name, VTY_NEWLINE);
		return -1;
	}

	return 0;
}

static int proxy_vty_go_parent(struct vty *vty)
{
	switch (vty->node) {
	case PROXY_NODE:
		vty->node = CONFIG_NODE;
		break;
	case EP_NODE:
		if (proxy_vty_ep_node_exit(vty, vty->index) != 0)
			break;
		vty->node = PROXY_NODE;
		vty->index = g_proxy_ctx;
		break;
	default:
		vty->node = CONFIG_NODE;
	}

	return vty->node;
}

static struct vty_app_info g_vty_info = {
	.name = "OsmoTRXProxy",
	.copyright =
		"Copyright (C) 2026 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>\r\n"
		"Author: Vadim Yanitskiy <vyanitskiy@sysmocom.de>\r\n"
		"License AGPLv3+: GNU AGPL version 3 or later "
		"<http://gnu.org/licenses/agpl-3.0.html>\r\n"
		"This is free software: you are free to change and redistribute it.\r\n"
		"There is NO WARRANTY, to the extent permitted by law.\r\n",
	.go_parent_cb = proxy_vty_go_parent,
};

int proxy_vty_init(void)
{
	g_vty_info.tall_ctx = g_talloc_ctx;
	vty_init(&g_vty_info);

	logging_vty_add_cmds();
	osmo_talloc_vty_add_cmds();
	osmo_cpu_sched_vty_init(g_talloc_ctx);

	install_element_ve(&show_proxy_cmd);

	install_element(CONFIG_NODE, &cfg_proxy_cmd);
	install_node(&proxy_node, config_write_proxy);
	install_element(PROXY_NODE, &cfg_proxy_bind_addr_cmd);
	install_element(PROXY_NODE, &cfg_proxy_pm_rssi_noise_cmd);
	install_element(PROXY_NODE, &cfg_proxy_path_loss_cmd);
	install_element(PROXY_NODE, &cfg_proxy_nom_toa256_cmd);
	install_element(PROXY_NODE, &cfg_proxy_nom_ci_cmd);
	install_element(PROXY_NODE, &cfg_proxy_clck_gen_start_fn_cmd);
	install_element(PROXY_NODE, &cfg_proxy_clck_gen_ind_period_cmd);
	install_element(PROXY_NODE, &cfg_proxy_ep_cmd);
	install_element(PROXY_NODE, &cfg_no_proxy_ep_cmd);

	install_node(&ep_node, dummy_config_write);
	install_element(EP_NODE, &cfg_ep_remote_addr_cmd);
	install_element(EP_NODE, &cfg_ep_bind_addr_cmd);
	install_element(EP_NODE, &cfg_ep_base_port_cmd);
	install_element(EP_NODE, &cfg_ep_num_chans_cmd);
	install_element(EP_NODE, &cfg_ep_tx_power_cmd);
	install_element(EP_NODE, &cfg_ep_trxd_max_version_cmd);
	install_element(EP_NODE, &cfg_ep_trxd_pdu_batch_cmd);
	install_element(EP_NODE, &cfg_ep_no_trxd_pdu_batch_cmd);
	install_element(EP_NODE, &cfg_ep_clock_socket_cmd);
	install_element(EP_NODE, &cfg_ep_no_clock_socket_cmd);

	return 0;
}
