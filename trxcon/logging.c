/*
 * OsmocomBB <-> SDR connection bridge
 *
 * (C) 2016-2017 by Vadim Yanitskiy <axilirator@gmail.com>
 *
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <osmocom/core/application.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>

#include "logging.h"

static struct log_info_cat trx_log_info_cat[] = {
	[DAPP] = {
		.name = "DAPP",
		.description = "Application",
		.color = "\033[1;35m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DL1C] = {
		.name = "DL1C",
		.description = "Layer 1 control interface",
		.color = "\033[1;31m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DL1D] = {
		.name = "DL1D",
		.description = "Layer 1 data",
		.color = "\033[1;31m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DTRX] = {
		.name = "DTRX",
		.description = "Transceiver control interface",
		.color = "\033[1;33m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DTRXD] = {
		.name = "DTRXD",
		.description = "Transceiver data interface",
		.color = "\033[1;33m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DSCH] = {
		.name = "DSCH",
		.description = "Scheduler management",
		.color = "\033[1;36m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DSCHD] = {
		.name = "DSCHD",
		.description = "Scheduler data",
		.color = "\033[1;36m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
};

static const struct log_info trx_log_info = {
	.cat = trx_log_info_cat,
	.num_cat = ARRAY_SIZE(trx_log_info_cat),
};

int trx_log_init(void *tall_ctx, const char *category_mask)
{
	osmo_init_logging2(tall_ctx, &trx_log_info);

	if (category_mask)
		log_parse_category_mask(osmo_stderr_target, category_mask);

	return 0;
}
