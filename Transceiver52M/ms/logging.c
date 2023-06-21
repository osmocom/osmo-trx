/*
 * (C) 2016-2022 by Vadim Yanitskiy <axilirator@gmail.com>
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
 */

#include <osmocom/core/application.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>

#include <osmocom/bb/trxcon/logging.h>
#include <osmocom/bb/trxcon/trxcon.h>

static struct log_info_cat trxcon_log_info_cat[] = {
	[DAPP] = {
		.name = "DAPP",
		.color = "\033[1;35m",
		.description = "Application",
		.loglevel = LOGL_NOTICE,
		.enabled = 1,
	},
	[DL1C] = {
		.name = "DL1C",
		.color = "\033[1;31m",
		.description = "Layer 1 control interface",
		.loglevel = LOGL_NOTICE,
		.enabled = 1,
	},
	[DL1D] = {
		.name = "DL1D",
		.color = "\033[1;31m",
		.description = "Layer 1 data",
		.loglevel = LOGL_NOTICE,
		.enabled = 1,
	},
	[DSCH] = {
		.name = "DSCH",
		.color = "\033[1;36m",
		.description = "Scheduler management",
		.loglevel = LOGL_NOTICE,
		.enabled = 0,
	},
	[DSCHD] = {
		.name = "DSCHD",
		.color = "\033[1;36m",
		.description = "Scheduler data",
		.loglevel = LOGL_NOTICE,
		.enabled = 0,
	},
};

static struct log_info trxcon_log_info = {
	.cat = trxcon_log_info_cat,
	.num_cat = ARRAY_SIZE(trxcon_log_info_cat),
};

static const int trxcon_log_cfg[] = {
	[TRXCON_LOGC_FSM] = DAPP,
	[TRXCON_LOGC_L1C] = DL1C,
	[TRXCON_LOGC_L1D] = DL1D,
	[TRXCON_LOGC_SCHC] = DSCH,
	[TRXCON_LOGC_SCHD] = DSCHD,
};

void trxc_log_init(void *tallctx)
{
	osmo_init_logging2(tallctx, &trxcon_log_info);
	log_target_file_switch_to_wqueue(osmo_stderr_target);

	trxcon_set_log_cfg(&trxcon_log_cfg[0], ARRAY_SIZE(trxcon_log_cfg));
}
