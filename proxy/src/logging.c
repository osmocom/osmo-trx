/*! \file src/logging.c
 * osmo-trx-proxy: log category definitions. */

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

#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>

#include <osmocom/proxy/logging.h>

static const struct log_info_cat default_categories[] = {
	[DPROXY] = {
		.name = "DPROXY",
		.description = "Main generic category",
		.loglevel = LOGL_INFO,
		.enabled = 1,
	},
	[DTRXC] = {
		.name = "DTRXC",
		.description = "TRXC (control) interface + clock handling",
		.color = "\033[1;33m",
		.loglevel = LOGL_NOTICE,
		.enabled = 1,
	},
	[DTRXD] = {
		.name = "DTRXD",
		.description = "TRXD (burst data) interface handling / forwarding",
		.color = "\033[1;33m",
		.loglevel = LOGL_NOTICE,
		.enabled = 1,
	},
};

const struct log_info log_info = {
	.cat = default_categories,
	.num_cat = ARRAY_SIZE(default_categories),
};
