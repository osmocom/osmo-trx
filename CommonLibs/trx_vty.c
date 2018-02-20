/*
 * Copyright (C) 2012-2017 sysmocom - s.f.m.c. GmbH
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/rate_ctr.h>

#include <osmocom/vty/command.h>
#include <osmocom/vty/vty.h>
#include <osmocom/vty/misc.h>

#include "trx_vty.h"
#include "../config.h"

static struct trx_ctx* g_trx_ctx;

struct trx_ctx *trx_from_vty(struct vty *v)
{
        /* It can't hurt to force callers to continue to pass the vty instance
         * to this function, in case we'd like to retrieve the global
         * trx instance from the vty at some point in the future. But
         * until then, just return the global pointer, which should have been
         * initialized by trx_vty_init().
         */
        OSMO_ASSERT(g_trx_ctx);
        return g_trx_ctx;
}

enum trx_vty_node {
	TRX_NODE = _LAST_OSMOVTY_NODE + 1,
};

static struct cmd_node trx_node = {
	TRX_NODE,
	"%s(config-trx)# ",
	1,
};

DEFUN(cfg_trx, cfg_trx_cmd,
	"trx",
	"Configure the TRX\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	if (!trx)
		return CMD_WARNING;

	vty->node = TRX_NODE;

	return CMD_SUCCESS;
}

DEFUN(cfg_bind_ip, cfg_bind_ip_cmd,
	"bind-ip A.B.C.D",
	"Set the IP address for the local bind\n"
	"IPv4 Address\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	osmo_talloc_replace_string(trx, &trx->cfg.bind_addr, argv[0]);

	return CMD_SUCCESS;
}

static int config_write_trx(struct vty *vty)
{
	struct trx_ctx *trx = trx_from_vty(vty);

	vty_out(vty, "trx%s", VTY_NEWLINE);
	if (trx->cfg.bind_addr)
		vty_out(vty, " bind-ip %s%s", trx->cfg.bind_addr, VTY_NEWLINE);

	return CMD_SUCCESS;
}

DEFUN(show_trx, show_trx_cmd,
	"show trx",
	SHOW_STR "Display information on the TRX\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	vty_out(vty, "TRX: Bound to %s%s", trx->cfg.bind_addr, VTY_NEWLINE);

	return CMD_SUCCESS;
}

static int trx_vty_is_config_node(struct vty *vty, int node)
{
	switch (node) {
	case TRX_NODE:
		return 1;
	default:
		return 0;
	}
}

static int trx_vty_go_parent(struct vty *vty)
{
	switch (vty->node) {
	case TRX_NODE:
		vty->node = CONFIG_NODE;
		vty->index = NULL;
		vty->index_sub = NULL;
		break;
	default:
		OSMO_ASSERT(0);
	}

	return vty->node;
}

static const char trx_copyright[] =
	"Copyright (C) 2007-2014 Free Software Foundation, Inc.\r\n"
	"Copyright (C) 2013 Thomas Tsou <tom@tsou.cc>\r\n"
	"Copyright (C) 2015 Ettus Research LLC\r\n"
	"Copyright (C) 2017-2018 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>\r\n"
	"License AGPLv3+: GNU AGPL version 3 or later <http://gnu.org/licenses/agpl-3.0.html>\r\n"
	"This is free software: you are free to change and redistribute it.\r\n"
	"There is NO WARRANTY, to the extent permitted by law.\r\n";

struct vty_app_info g_vty_info = {
	.name		= "OsmoTRX",
	.version	= PACKAGE_VERSION,
	.copyright	= trx_copyright,
	.go_parent_cb	= trx_vty_go_parent,
	.is_config_node	= trx_vty_is_config_node,
};

int trx_vty_init(struct trx_ctx* trx)
{
	g_trx_ctx = trx;
	install_element_ve(&show_trx_cmd);

	install_element(CONFIG_NODE, &cfg_trx_cmd);

	install_node(&trx_node, config_write_trx);
	install_element(TRX_NODE, &cfg_bind_ip_cmd);

	return 0;
}
