/*
* Copyright 2008 Free Software Foundation, Inc.
*
* SPDX-License-Identifier: AGPL-3.0+
*
* This software is distributed under the terms of the GNU Affero Public License.
* See the COPYING file in the main directory for details.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#include <string.h>
#include <sys/types.h>

#include "Threads.h"
#include "Timeval.h"
#include "Logger.h"

extern "C" {
#include <osmocom/core/thread.h>
}

using namespace std;

#ifndef HAVE_ATOMIC_OPS
	pthread_mutex_t atomic_ops_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif



void set_selfthread_name(const char *name)
{
	pthread_t selfid = pthread_self();
	pid_t tid = osmo_gettid();
	if (pthread_setname_np(selfid, name) == 0) {
		LOG(INFO) << "Thread "<< selfid << " (task " << tid << ") set name: " << name;
	} else {
		char buf[256];
		int err = errno;
		char* err_str = strerror_r(err, buf, sizeof(buf));
		LOG(NOTICE) << "Thread "<< selfid << " (task " << tid << ") set name \"" << name << "\" failed: (" << err << ") " << err_str;
	}
}

void thread_enable_cancel(bool cancel)
{
	cancel ? pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL) :
		 pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
}

void Thread::start(void *(*task)(void*), void *arg)
{
	assert(mThread==((pthread_t)0));
	bool res;
	// (pat) Moved initialization to constructor to avoid crash in destructor.
	//res = pthread_attr_init(&mAttrib);
	//assert(!res);
	if (mStackSize != 0) {
		res = pthread_attr_setstacksize(&mAttrib, mStackSize);
		assert(!res);
	}
	res = pthread_create(&mThread, &mAttrib, task, arg);
	assert(!res);
}



// vim: ts=4 sw=4
