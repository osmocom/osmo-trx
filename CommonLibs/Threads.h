/*
* Copyright 2008, 2011 Free Software Foundation, Inc.
* Copyright 2013 Alexander Chemeris <Alexander.Chemeris@fairwaves.ru>
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

#include <pthread.h>
#include <iostream>
#include <assert.h>
#include <unistd.h>

class Mutex;


/**@name Multithreaded access for standard streams. */
//@{

/**@name Functions for gStreamLock. */
//@{
extern Mutex gStreamLock;	///< global lock for cout and cerr
void lockCerr();		///< call prior to writing cerr
void unlockCerr();		///< call after writing cerr
void lockCout();		///< call prior to writing cout
void unlockCout();		///< call after writing cout
//@}

/**@name Macros for standard messages. */
//@{
#define COUT(text) { lockCout(); std::cout << text; unlockCout(); }
#define CERR(text) { lockCerr(); std::cerr << __FILE__ << ":" << __LINE__ << ": " << text; unlockCerr(); }
#ifdef NDEBUG
#define DCOUT(text) {}
#define OBJDCOUT(text) {}
#else
#define DCOUT(text) { COUT(__FILE__ << ":" << __LINE__ << " " << text); }
#define OBJDCOUT(text) { DCOUT(this << " " << text); } 
#endif
//@}
//@}



/**@defgroup C++ wrappers for pthread mechanisms. */
//@{

/** A class for recursive mutexes based on pthread_mutex. */
class Mutex {

	private:

	pthread_mutex_t mMutex;
	pthread_mutexattr_t mAttribs;

	public:

	Mutex();

	~Mutex();

	void lock() { pthread_mutex_lock(&mMutex); }

	bool trylock() { return pthread_mutex_trylock(&mMutex)==0; }

	void unlock() { pthread_mutex_unlock(&mMutex); }

	friend class Signal;

};


class ScopedLock {

	private:
	Mutex& mMutex;

	public:
	ScopedLock(Mutex& wMutex) :mMutex(wMutex) { mMutex.lock(); }
	~ScopedLock() { mMutex.unlock(); }

};




/** A C++ interthread signal based on pthread condition variables. */
class Signal {

	private:

	mutable pthread_cond_t mSignal;

	public:

	Signal() { int s = pthread_cond_init(&mSignal,NULL); assert(!s); }

	~Signal() { pthread_cond_destroy(&mSignal); }

	/**
		Block for the signal up to the cancellation timeout.
		Under Linux, spurious returns are possible.
	*/
	int wait(Mutex& wMutex, unsigned timeout) const;

	/**
		Block for the signal.
		Under Linux, spurious returns are possible.
	*/
	int wait(Mutex& wMutex) const
		{ return pthread_cond_wait(&mSignal,&wMutex.mMutex); }

	void signal() { pthread_cond_signal(&mSignal); }

	void broadcast() { pthread_cond_broadcast(&mSignal); }

};


/** A C++ wrapper for pthread threads.  */
class Thread {

public:

    typedef void *(*Adaptor)(void*);
    enum ReturnStatus {
        RETURN_OK = 0,
        ALREADY_STARTED,
        ALREADY_IDLE,
        PTHREAD_ERROR,
        WRONG_STATE,
        RETURN_TIMEOUT
    };
    enum ThreadState {
        THREAD_STATE_IDLE,      ///< Thread is not started. On start() => STARTING
        THREAD_STATE_STARTING,  ///< Thread is about to start. When actually started => RUNNING
        THREAD_STATE_RUNNING,   ///< Thread is active. On stop() => STOPPING
        THREAD_STATE_STOPPING   ///< Thread is about to stop. When actually stopped => IDLE
    };
    enum {
       THREAD_STARTUP_TIMEOUT=5, ///< Time to wait for thread startup (in seconds).
       THREAD_STOP_TIMEOUT=5     ///< Time to wait for thread stop (in seconds).
    };

    /** Create a thread in a non-running state. */
    Thread(const std::string &name, size_t stackSize = (65536*4));

    /** Destroy the Thread. */
    virtual ~Thread();

    /** Start the thread. */
    ReturnStatus startThread(void *data=NULL);

    /** Stop the thread. */
    ReturnStatus stopThread();

    ThreadState getThreadState() const
    {
        ScopedLock lock(mThreadStateMutex);
        return mThreadState;
    }

    bool isThreadRunning() const
    {
        ScopedLock lock(mThreadStateMutex);
        return mThreadState == THREAD_STATE_RUNNING;
    }
    void requestThreadStop()
    {
        ScopedLock lock(mThreadStateMutex);
        if (mThreadState == THREAD_STATE_RUNNING)
            mThreadState = THREAD_STATE_STOPPING;
    }
    bool isThreadStopping() const
    {
        ScopedLock lock(mThreadStateMutex);
        return mThreadState == THREAD_STATE_STOPPING;
    }

    const std::string &getThreadName() const {return mThreadName;}

protected:

    pthread_t mThreadId; ///< OS id of the thread.
    const std::string mThreadName; ///< Name of the thread.
    size_t mStackSize;  ///< Requested stack size for the thread.
    ThreadState mThreadState; ///< The current state of the thread.
    mutable Mutex mThreadStateMutex; ///< Mutex to protect ThreadState variable
    void *mThreadData;  ///< Data to be passed to the thread loop.
    Mutex mThreadStartupMutex; ///< Mutex, used with the next two conditional
                        ///< variables to synchronize thread startup.
    Signal  mThreadInitializedEvent; ///< Conditional variable, signaling
                        ///< that this thread object initialization is completed
                        ///< and the thread could go on.
    Signal  mThreadStartStopEvent; ///< Conditional variable, signaling
                        ///< that the thread is started and start() method could
                        ///< return to caller.

    /** Function with the actual thread loop.
     *  Override this function in child classes to do real work.
     */
    virtual void runThread() =0;

    // Static funciton which actually starts the run() method.
    static void *threadAdaptor(void *data);

    void ackThreadStart() {
        ScopedLock lock(mThreadStateMutex);
        assert(mThreadState == THREAD_STATE_STARTING);
        mThreadState = THREAD_STATE_RUNNING;
    }
    void ackThreadStop() {
        ScopedLock lock(mThreadStateMutex);
        assert(mThreadState == THREAD_STATE_STOPPING);
        mThreadState = THREAD_STATE_IDLE;
    }
};


#endif
// vim: ts=4 sw=4
