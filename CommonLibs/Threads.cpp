/*
* Copyright 2008 Free Software Foundation, Inc.
* Copyright 2013 Alexander Chemeris <Alexander.Chemeris@fairwaves.ru>
*
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





#include "Threads.h"
#include "Timeval.h"
#include "Logger.h"
#include <pthread.h>
#include <sys/types.h>
#include <errno.h> // for ETIMEDOUT
#include <sys/syscall.h> // for SYS_gettid
#include <sys/prctl.h> // Linux specific, for prctl(PR_SET_NAME)

// Make sure we get MCL_CURRENT and MCL_FUTURE (for mlockall) on OS X 10.3
#define _P1003_1B_VISIBLE
#include <sys/mman.h>
#undef _P1003_1B_VISIBLE


using namespace std;

#define POSIX_OK           0
#define POSIX_NO_WAIT      0
#define POSIX_WAIT_FOREVER (-1)

static inline int gettid() {return syscall(SYS_gettid);}


Mutex gStreamLock;		///< Global lock to control access to cout and cerr.

void lockCout()
{
	gStreamLock.lock();
	Timeval entryTime;
	cout << entryTime << " " << pthread_self() << ": ";
}


void unlockCout()
{
	cout << dec << endl << flush;
	gStreamLock.unlock();
}


void lockCerr()
{
	gStreamLock.lock();
	Timeval entryTime;
	cerr << entryTime << " " << pthread_self() << ": ";
}

void unlockCerr()
{
	cerr << dec << endl << flush;
	gStreamLock.unlock();
}







Mutex::Mutex()
{
	bool res;
	res = pthread_mutexattr_init(&mAttribs);
	assert(!res);
	res = pthread_mutexattr_settype(&mAttribs,PTHREAD_MUTEX_RECURSIVE);
	assert(!res);
	res = pthread_mutex_init(&mMutex,&mAttribs);
	assert(!res);
}


Mutex::~Mutex()
{
	pthread_mutex_destroy(&mMutex);
	bool res = pthread_mutexattr_destroy(&mAttribs);
	assert(!res);
}




/** Block for the signal up to the cancellation timeout. */
int Signal::wait(Mutex& wMutex, unsigned timeout) const
{
	Timeval then(timeout);
	struct timespec waitTime = then.timespec();
	return pthread_cond_timedwait(&mSignal,&wMutex.mMutex,&waitTime);
}

Thread::Thread(const string &name, size_t stackSize)
: mThreadId((pthread_t)0)
, mThreadName(name)
, mStackSize(stackSize)
, mThreadState(THREAD_STATE_IDLE)
, mThreadData(NULL)
{
}

Thread::~Thread()
{
    stopThread();
}

void *Thread::threadAdaptor(void *data)
{
    Thread *pThread = (Thread*)data;

    // If we ever receive a thread cancel request, it means that the Thread
    // object is in the process of being destroyed.  To avoid the situation
    // where a thread attempts to run after its containing Thread object has
    // been freed, we set the thread up so that the cancel takes effect
    // immediately (as opposed to waiting until the next thread cancellation
    // point).
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    // =====================================================================
    // Synchronize with the start() in the parent thread.
    {
        // 1. Lock synchronization mutex.
        ScopedLock lock(pThread->mThreadStartupMutex);

        // 2. Btw, set the thread name, while we're inside the mutex.
        // FIXME: This works on Linux with glibc >= 2.12. Under *BSD and MacOS X
        //        this function has different arguments.
//        pthread_setname_np(pThread->mThreadId, pThread->mThreadName.c_str());
        // FIXME: For some reason the previous call doesn't work on my Ubuntu 12.04,
        //        so we use this one which works.
        prctl(PR_SET_NAME, pThread->mThreadName.c_str());

        // 3. Signal that we've started.
        pThread->mThreadStartStopEvent.signal();

        // 4. Wait until start() finishes its initialization.
        //
        // The actual thread is created and started with pthread_create(), then
        // start() does its housekeeping and sets mThreadState=THREAD_STATE_RUNNING.
        // If we allow Thread::run() to start before this initialization completes,
        // callers might think (among other things) that the thread is not started
        // while it's actually started.
        pThread->mThreadInitializedEvent.wait(pThread->mThreadStartupMutex);
    }
    // Synchronization with the parent thread is finished.
    // =====================================================================

    // Log Thread ID for debugging purposes
    LOG(INFO) << "Thread started: " << pThread->mThreadName
              << " with lwp=" << gettid() << ", pid=" << getpid();

    // Keep all memory locked into physical mem, to guarantee realtime-behaviour
    int res = mlockall(MCL_CURRENT|MCL_FUTURE);
    if (res != POSIX_OK) {
      LOG(WARNING) << "Failed to lock memory for thread: " << pThread->mThreadName;
    }

    // Run the actual code
    pThread->runThread();

    // Huh, we're done. Signal to a (potentially) waiting stop()'s.
    {
        ScopedLock lock(pThread->mThreadStateMutex);
        pThread->mThreadState = THREAD_STATE_IDLE;
        pThread->mThreadStartStopEvent.broadcast();
    }

    return NULL;
}

Thread::ReturnStatus Thread::startThread(void *data)
{
    pthread_attr_t attrib;
//    timeval        threadStartTime;
//    timespec       threadStartTimeout;
    bool res;

    // Lock startup synchronization mutex. It will be used in conjunction with
    // mThreadInitializedEvent and mThreadStartStopEvent conditional variables.
    ScopedLock lock(mThreadStartupMutex);

    {
        ScopedLock lock(mThreadStateMutex);
        if (mThreadState != THREAD_STATE_IDLE)
            return ALREADY_STARTED;
        mThreadState = THREAD_STATE_STARTING;
    }

    // Save thread data pointer
    mThreadData = data;

    LOG(DEBUG) <<  "Starting thread " << mThreadName << " (" << this << ")";

    // construct thread attribute
    res = pthread_attr_init(&attrib);
    if (res != POSIX_OK) {
        LOG(ALERT) <<  "pthread_attr_init failed, returned " << res
                   << " in " << mThreadName << " (" << this << ")";
    }


    // Set the thread stack size
    res = pthread_attr_setstacksize(&attrib, mStackSize);
    if (res != POSIX_OK)
    {
        LOG(ALERT) <<  "pthread_attr_setstacksize failed, returned " << res
                   << " in " << mThreadName << " (" << this << ")";
    }

    // Create the thread detached
    res = pthread_attr_setdetachstate(&attrib, PTHREAD_CREATE_DETACHED);
    if (res != POSIX_OK)
    {
       LOG(ALERT) <<  "pthread_attr_setdetachstate failed, returned " << res
                  << " in " << mThreadName << " (" << this << ")";
    }

    // =====================================================================
    // Start the thread and synchronize with it

    // Start the thread!
    res = pthread_create(&mThreadId, &attrib, threadAdaptor, (void *)this);
    // Attributes are no longer needed.
    pthread_attr_destroy(&attrib);

    if (res != POSIX_OK)
    {
        LOG(ALERT) << "pthread_create failed, returned " << res
                   << " in " << mThreadName << " (" << this << ")";

        return PTHREAD_ERROR;
    }

    // Wait for the thread to startup.
    res = mThreadStartStopEvent.wait(mThreadStartupMutex, THREAD_STARTUP_TIMEOUT*1000);

    // If the thread does not start in THREAD_STARTUP_TIMEOUT seconds,
    // then something is terribly wrong here.
    if (res == ETIMEDOUT)
    {
       LOG(ALERT) << "thread " << mThreadName << " (" << this << ") hasn't started up in "
                  << THREAD_STARTUP_TIMEOUT << " seconds. Bailing out.";

       return RETURN_TIMEOUT;
    }

    // We're done with the initialization.
    ackThreadStart();

    // ToDo: Add other initialization here, e.g. adding this thread to a list of all threads.

    // Startup initialization finished. Signal this to started thread, so
    // it could go on.
    mThreadInitializedEvent.signal();

    return RETURN_OK;
}

Thread::ReturnStatus Thread::stopThread()
{
    int res;

    LOG(DEBUG) <<  "Stopping thread " << mThreadName << " (" << this << ")";

    while (1) {
        ScopedLock lock(mThreadStateMutex);

        switch (mThreadState) {
        case THREAD_STATE_IDLE:
            // Nothing to do.
            return RETURN_OK;

        case THREAD_STATE_STARTING:
            // Something is wrong in thi world.
            assert(mThreadState != THREAD_STATE_STARTING);
            LOG(ALERT) << "Trying to stop thread " << mThreadName
                       << " (" << this << ") while it's trying to start.";
            return WRONG_STATE;

        case THREAD_STATE_RUNNING:
            // Request shudown
            mThreadState = THREAD_STATE_STOPPING;
            // no "break" here to fall through to the next case

        case THREAD_STATE_STOPPING:
            // Wait for the thread to stop.
            LOG(DEBUG) << "Waiting for thread " << mThreadName << " (" << this << ") to stop.";
            res = mThreadStartStopEvent.wait(mThreadStateMutex, THREAD_STOP_TIMEOUT*1000);
            LOG(DEBUG) << "Thread " << mThreadName << " (" << this << ") signalled stop "
                       << "with res=" << res << " and mThreadState=" << mThreadState;

            // If the thread does not stop in THREAD_STOP_TIMEOUT seconds,
            // return error. It may be waiting for something.
            if (res == ETIMEDOUT)
            {
               LOG(ALERT) << "thread " << mThreadName << " (" << this << ") hasn't stopped in "
                          << THREAD_STARTUP_TIMEOUT << " seconds. Bailing out.";

               return RETURN_TIMEOUT;
            }

            // Conditional variable could return in case of a signal, so we should
            // double check that the thread has indeed stopped.
            if (mThreadState == THREAD_STATE_IDLE)
                return RETURN_OK;
            else
                // Try again...
                break;
        }
    }

    // We should never reach this line
    assert(false);
    return RETURN_OK;
}

// vim: ts=4 sw=4
