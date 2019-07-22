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

#include <pthread.h>

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
	[DTRXCTRL] = {
			.name = "DTRXCTRL",
			.description = "TRX CTRL interface",
			.color = "\033[1;33m",
			.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DDEV] = {
		.name = "DDEV",
		.description = "Device/Driver specific code",
		.color = NULL,
		.enabled = 1, .loglevel = LOGL_INFO,
	},
	[DLMS] = {
		.name = "DLMS",
		.description = "Logging from within LimeSuite itself",
		.color = NULL,
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
};

const struct log_info log_info = {
	.cat = default_categories,
	.num_cat = ARRAY_SIZE(default_categories),
};

pthread_mutex_t log_mutex;

bool log_mutex_init() {
	int rc;
	pthread_mutexattr_t attr;

	if ((rc = pthread_mutexattr_init(&attr))) {
		fprintf(stderr, "pthread_mutexattr_init() failed: %d\n", rc);
		return false;
	}
	if ((rc = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE))) {
		fprintf(stderr, "pthread_mutexattr_settype() failed: %d\n", rc);
		return false;
	}
	if ((rc = pthread_mutex_init(&log_mutex, &attr))) {
		fprintf(stderr, "pthread_mutex_init() failed: %d\n", rc);
		return false;
	}
	if ((rc = pthread_mutexattr_destroy(&attr))) {
		fprintf(stderr, "pthread_mutexattr_destroy() failed: %d\n", rc);
		return false;
	}
	return true;
	/* FIXME: do we need to call pthread_mutex_destroy() during process exit? */
}

/* If called inside a C++ destructor, use log_mutex_(un)lock_canceldisable() APIs instead.
   See osmo-trx commit 86be40b4eb762d5c12e8e3f7388ca9f254e77b36 for more information */
void log_mutex_lock() {
	OSMO_ASSERT(!pthread_mutex_lock(&log_mutex));
}

void log_mutex_unlock() {
	OSMO_ASSERT(!pthread_mutex_unlock(&log_mutex));
}

void log_mutex_lock_canceldisable(int *st) {
	pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, st);
	log_mutex_lock();
}

void log_mutex_unlock_canceldisable(int st) {
	log_mutex_unlock();
	pthread_setcancelstate(st, NULL);
}
