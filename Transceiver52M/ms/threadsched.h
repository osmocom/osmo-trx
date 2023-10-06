#pragma once
/*
 * (C) 2023 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
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

extern "C" {
#include <pthread.h>
#include <sched.h>
}

static struct sched_params {
	enum thread_names { U_CTL = 0, U_RX, U_TX, SCH_SEARCH, MAIN, LEAKCHECK, RXRUN, TXRUN, _THRD_NAME_COUNT };
	enum target { ODROID = 0, PI4 };
	const char *name;
	int core;
	int schedtype;
	int prio;
} schdp[][sched_params::_THRD_NAME_COUNT]{
	{
		{ "upper_ctrl", 2, SCHED_RR, sched_get_priority_max(SCHED_RR) },
		{ "upper_rx", 2, SCHED_RR, sched_get_priority_max(SCHED_RR) - 5 },
		{ "upper_tx", 2, SCHED_RR, sched_get_priority_max(SCHED_RR) - 1 },

		{ "sch_search", 3, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 5 },
		{ "main", 3, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 5 },
		{ "leakcheck", 3, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 10 },

		{ "rxrun", 4, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 2 },
		{ "txrun", 5, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 1 },
	},
	{
		{ "upper_ctrl", 1, SCHED_RR, sched_get_priority_max(SCHED_RR) },
		{ "upper_rx", 1, SCHED_RR, sched_get_priority_max(SCHED_RR) - 5 },
		{ "upper_tx", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 1 },

		{ "sch_search", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 5 },
		{ "main", 3, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 5 },
		{ "leakcheck", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 10 },

		{ "rxrun", 2, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 2 },
		{ "txrun", 3, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 1 },
	},
};

void set_sched_target(sched_params::target t);

using worker_func_sig = void *(*)(void *);

void set_name_aff_sched(sched_params::thread_names name);

pthread_t spawn_worker_thread(sched_params::thread_names name, worker_func_sig fun, void *arg);
