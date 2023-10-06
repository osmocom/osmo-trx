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

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

extern "C" {
#include <pthread.h>
}

#include "threadsched.h"

sched_params::target scheduling_target;

void set_sched_target(sched_params::target t)
{
	scheduling_target = t;
}

void set_name_aff_sched(std::thread::native_handle_type h, const char *name, int cpunum, int schedtype, int prio)
{
	pthread_setname_np(h, name);

	cpu_set_t cpuset;

	CPU_ZERO(&cpuset);
	CPU_SET(cpunum, &cpuset);

	if (pthread_setaffinity_np(h, sizeof(cpuset), &cpuset) < 0) {
		std::cerr << name << " affinity: errreur! " << std::strerror(errno);
		return exit(0);
	}

	sched_param sch_params;
	sch_params.sched_priority = prio;
	if (pthread_setschedparam(h, schedtype, &sch_params) < 0) {
		std::cerr << name << " sched: errreur! " << std::strerror(errno);
		return exit(0);
	}
}

static pthread_t do_spawn_thr(const char *name, int cpunum, int schedtype, int prio, worker_func_sig fun, void *arg)
{
	pthread_t thread;

	pthread_attr_t attr;
	pthread_attr_init(&attr);

	sched_param sch_params;
	sch_params.sched_priority = prio;
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(cpunum, &cpuset);
	auto a = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
	a |= pthread_attr_setschedpolicy(&attr, schedtype);
	a |= pthread_attr_setschedparam(&attr, &sch_params);
	a |= pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (a)
		std::cerr << "thread arg rc:" << a << std::endl;
	pthread_create(&thread, &attr, fun, arg);
	pthread_setname_np(thread, name);
	pthread_attr_destroy(&attr);
	return thread;
}

void set_name_aff_sched(std::thread::native_handle_type h, sched_params::thread_names name)
{
	auto tgt = schdp[scheduling_target][name];
	// std::cerr << "scheduling for: " << tgt.name << ":" << tgt.core << std::endl;
	set_name_aff_sched(h, tgt.name, tgt.core, tgt.schedtype, tgt.prio);
}

void set_name_aff_sched(sched_params::thread_names name)
{
	set_name_aff_sched(pthread_self(), name);
}

pthread_t spawn_worker_thread(sched_params::thread_names name, worker_func_sig fun, void *arg)
{
	auto tgt = schdp[scheduling_target][name];
	// std::cerr << "scheduling for: " << tgt.name << ":" << tgt.core << " prio:" << tgt.prio << std::endl;
	return do_spawn_thr(tgt.name, tgt.core, tgt.schedtype, tgt.prio, fun, arg);
}
