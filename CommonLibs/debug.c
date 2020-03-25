/*
 * Copyright (C) 2018-2019 sysmocom - s.f.m.c. GmbH
 * All Rights Reserved
 *
 * SPDX-License-Identifier: AGPL-3.0+
 *
 * Author: Pau Espin Pedrol <pespin@sysmocom.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 */

#include "config.h"

/* If HAVE_GETTID, then "_GNU_SOURCE" may need to be defined to use gettid() */
#if HAVE_GETTID
#define _GNU_SOURCE
#endif

#include <sys/types.h>
#include <unistd.h>
#include <sys/syscall.h>
#include "config.h"

#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>
#include "debug.h"

/* default categories */
static const struct log_info_cat default_categories[] = {
	[DMAIN] = {
		.name = "DMAIN",
		.description = "Main generic category",
		.color = NULL,
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DTRXCLK] = {
			.name = "DTRXCLK",
			.description = "TRX Master Clock",
			.color = NULL,
			.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DTRXCTRL] = {
			.name = "DTRXCTRL",
			.description = "TRX CTRL interface",
			.color = "\033[1;33m",
			.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DTRXDDL] = {
			.name = "DTRXDDL",
			.description = "TRX Data interface Downlink",
			.color = NULL,
			.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DTRXDUL] = {
			.name = "DTRXDUL",
			.description = "TRX CTRL interface Uplink",
			.color = NULL,
			.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DDEV] = {
		.name = "DDEV",
		.description = "Device/Driver specific code",
		.color = NULL,
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DDEVDRV] = {
		.name = "DDEVDRV",
		.description = "Logging from external device driver library implementing lower level specifics",
		.color = NULL,
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
};

const struct log_info log_info = {
	.cat = default_categories,
	.num_cat = ARRAY_SIZE(default_categories),
};

pid_t my_gettid(void)
{
#if HAVE_GETTID
	return gettid();
#elif defined(LINUX) && defined(__NR_gettid)
	return (pid_t) syscall(__NR_gettid);
#else
	#pragma message ("use pid as tid")
	return getpid();
#endif
}
