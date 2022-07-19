/*
* Copyright 2008, 2011 Free Software Foundation, Inc.
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


#ifndef THREADS_H
#define THREADS_H

#include <chrono>
#include <mutex>
#include <condition_variable>
#include <pthread.h>
#include <iostream>
#include <cassert>
#include <unistd.h>

#include "config.h"
#include "Timeval.h"

class Mutex;

/**@defgroup C++ wrappers for pthread mechanisms. */
//@{

/** A class for recursive mutexes. */
class Mutex {
	std::recursive_mutex m;

    public:

	void lock() {
		m.lock();
	}

	bool trylock() {
		return m.try_lock();
	}

	void unlock() {
		m.unlock();
	}

	friend class Signal;
};

class ScopedLock {
	Mutex &mMutex;

    public:
	ScopedLock(Mutex &wMutex) : mMutex(wMutex) {
		mMutex.lock();
	}
	~ScopedLock() {
		mMutex.unlock();
	}
};

/** A C++ interthread signal. */
class Signal {
	/* any, because for some reason our mutex is recursive... */
	std::condition_variable_any mSignal;

    public:

	void wait(Mutex &wMutex, unsigned timeout) {
		mSignal.wait_for(wMutex.m, std::chrono::milliseconds(timeout));
	}

	void wait(Mutex &wMutex) {
		mSignal.wait(wMutex.m);
	}

	void signal() {
		mSignal.notify_one();
	}

	void broadcast() {
		mSignal.notify_all();
	}
};

void set_selfthread_name(const char *name);
void thread_enable_cancel(bool cancel);

/** A C++ wrapper for pthread threads.  */
class Thread {
    private:
	pthread_t mThread;
	pthread_attr_t mAttrib;
	// FIXME -- Can this be reduced now?
	size_t mStackSize;

    public:
	/** Create a thread in a non-running state. */
	Thread(size_t wStackSize = 0) : mThread((pthread_t)0)
	{
		pthread_attr_init(&mAttrib); // (pat) moved this here.
		mStackSize = wStackSize;
	}

	/**
		Destroy the Thread.
		It should be stopped and joined.
	*/
	// (pat) If the Thread is destroyed without being started, then mAttrib is undefined.  Oops.
	~Thread()
	{
		pthread_attr_destroy(&mAttrib);
	}

	/** Start the thread on a task. */
	void start(void *(*task)(void *), void *arg);

	/** Join a thread that will stop on its own. */
	void join()
	{
		if (mThread) {
			int s = pthread_join(mThread, NULL);
			assert(!s);
		}
	}

	/** Send cancellation to thread */
	void cancel()
	{
		pthread_cancel(mThread);
	}
};

#ifdef HAVE_ATOMIC_OPS
#define osmo_trx_sync_fetch_and_and(ptr, value) __sync_fetch_and_and((ptr), (value))
#define osmo_trx_sync_or_and_fetch(ptr, value) __sync_or_and_fetch((ptr), (value))
#else
extern pthread_mutex_t atomic_ops_mutex;
static inline int osmo_trx_sync_fetch_and_and(int *ptr, int value)
{
	pthread_mutex_lock(&atomic_ops_mutex);
	int tmp = *ptr;
	*ptr &= value;
	pthread_mutex_unlock(&atomic_ops_mutex);
	return tmp;
}

static inline int osmo_trx_sync_or_and_fetch(int *ptr, int value)
{
	int tmp;
	pthread_mutex_lock(&atomic_ops_mutex);
	*ptr |= value;
	tmp = *ptr;
	pthread_mutex_unlock(&atomic_ops_mutex);
	return tmp;
}
#endif

#endif
// vim: ts=4 sw=4
