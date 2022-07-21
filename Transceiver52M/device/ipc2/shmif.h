/*
 * (C) 2022 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
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

#pragma once

#include <atomic>
#include <iostream>
#include <cassert>
#include <cstring>
#include <mutex>
#include <sstream>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <cerrno>

namespace shm
{

namespace mtx_log
{
#if defined(MTX_LOG_ENABLED)
	class print_guard : public std::ostringstream {
		static std::mutex thread_print_lock;

	    public:
		~print_guard()
		{
			std::lock_guard<std::mutex> guard(thread_print_lock);
			std::cerr << str();
		}
	};

#else
	struct print_guard {};

	template <typename T> constexpr print_guard operator<<(const print_guard dummy, T &&value)
	{
		return dummy;
	}

	constexpr print_guard operator<<(const print_guard &dummy, std::ostream &(*f)(std::ostream &))
	{
		return dummy;
	}

#endif
} // namespace mtx_log

class shmmutex {
	pthread_mutex_t mutex;

    public:
	shmmutex()
	{
		pthread_mutexattr_t attr;
		pthread_mutexattr_init(&attr);
		pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
		pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
		pthread_mutex_init(&mutex, &attr);
		pthread_mutexattr_destroy(&attr);
	}

	~shmmutex()
	{
		pthread_mutex_destroy(&mutex);
	}

	void lock()
	{
		pthread_mutex_lock(&mutex);
	}

	bool try_lock()
	{
		return pthread_mutex_trylock(&mutex);
	}

	void unlock()
	{
		pthread_mutex_unlock(&mutex);
	}

	pthread_mutex_t *p()
	{
		return &mutex;
	}
	shmmutex(const shmmutex &) = delete;
	shmmutex &operator=(const shmmutex &) = delete;
};

class shmcond {
	pthread_cond_t cond;

    public:
	shmcond()
	{
		pthread_condattr_t attr;
		pthread_condattr_init(&attr);
		pthread_condattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
		pthread_cond_init(&cond, &attr);
		pthread_condattr_destroy(&attr);
	}

	~shmcond()
	{
		pthread_cond_destroy(&cond);
	}

	void wait(shmmutex *lock)
	{
		pthread_cond_wait(&cond, lock->p());
	}

	void signal()
	{
		pthread_cond_signal(&cond);
	}

	void signal_all()
	{
		pthread_cond_broadcast(&cond);
	}
	shmcond(const shmcond &) = delete;
	shmcond &operator=(const shmcond &) = delete;
};

class signal_guard {
	shmmutex &m;
	shmcond &s;

    public:
	signal_guard() = delete;
	explicit signal_guard(shmmutex &m, shmcond &wait_for, shmcond &to_signal) : m(m), s(to_signal)
	{
		m.lock();
		wait_for.wait(&m);
	}
	~signal_guard()
	{
		s.signal();
		m.unlock();
	}
	signal_guard(const signal_guard &) = delete;
	signal_guard &operator=(const signal_guard &) = delete;
};

class mutex_guard {
	shmmutex &m;

    public:
	mutex_guard() = delete;
	explicit mutex_guard(shmmutex &m) : m(m)
	{
		m.lock();
	}
	~mutex_guard()
	{
		m.unlock();
	}
	mutex_guard(const mutex_guard &) = delete;
	mutex_guard &operator=(const mutex_guard &) = delete;
};

class sema {
	std::atomic<int> value;
	shmmutex m;
	shmcond c;

    public:
	sema() : value(0)
	{
	}
	explicit sema(int v) : value(v)
	{
	}

	void wait()
	{
		wait(1);
	}
	void wait(int v)
	{
		mtx_log::print_guard() << __FUNCTION__ << value << std::endl;
		mutex_guard g(m);
		assert(value <= v);
		while (value != v)
			c.wait(&m);
	}
	void wait_and_reset()
	{
		wait_and_reset(1);
	}
	void wait_and_reset(int v)
	{
		mtx_log::print_guard() << __FUNCTION__ << value << std::endl;
		mutex_guard g(m);
		assert(value <= v);
		while (value != v)
			c.wait(&m);
		value = 0;
	}
	void set()
	{
		set(1);
	}
	void set(int v)
	{
		mtx_log::print_guard() << __FUNCTION__ << value << std::endl;
		mutex_guard g(m);
		value = v;
		c.signal();
	}
	void reset_unsafe()
	{
		value = 0;
	}
	sema(const sema &) = delete;
	sema &operator=(const sema &) = delete;
};

class sema_wait_guard {
	sema &a;
	sema &b;

    public:
	sema_wait_guard() = delete;
	explicit sema_wait_guard(sema &wait, sema &signal) : a(wait), b(signal)
	{
		a.wait_and_reset(1);
	}
	~sema_wait_guard()
	{
		b.set(1);
	}
	sema_wait_guard(const sema_wait_guard &) = delete;
	sema_wait_guard &operator=(const sema_wait_guard &) = delete;
};

class sema_signal_guard {
	sema &a;
	sema &b;

    public:
	sema_signal_guard() = delete;
	explicit sema_signal_guard(sema &wait, sema &signal) : a(wait), b(signal)
	{
		a.wait_and_reset(1);
	}
	~sema_signal_guard()
	{
		b.set(1);
	}
	sema_signal_guard(const sema_signal_guard &) = delete;
	sema_signal_guard &operator=(const sema_signal_guard &) = delete;
};

template <typename IFT> class shm {
	char shmname[512];
	size_t IFT_sz = sizeof(IFT);
	IFT *shmptr;
	bool good;
	int ipc_shm_setup(const char *shm_name)
	{
		int fd;
		int rc;
		void *ptr;

		if ((fd = shm_open(shm_name, O_CREAT | O_RDWR | O_TRUNC, S_IRUSR | S_IWUSR)) < 0) {
			rc = -errno;
			return rc;
		}

		if (ftruncate(fd, IFT_sz) < 0) {
			rc = -errno;
			shm_unlink(shm_name);
			::close(fd);
		}

		if ((ptr = mmap(NULL, IFT_sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) == MAP_FAILED) {
			rc = -errno;
			shm_unlink(shm_name);
			::close(fd);
		}

		shmptr = new (ptr) IFT(); //static_cast<IFT *>(ptr);
		::close(fd);
		return 0;
	}

	int ipc_shm_connect(const char *shm_name)
	{
		int fd;
		int rc;
		void *ptr;

		if ((fd = shm_open(shm_name, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR)) < 0) {
			rc = -errno;
			return rc;
		}

		struct stat shm_stat;
		if (fstat(fd, &shm_stat) < 0) {
			rc = -errno;
			shm_unlink(shm_name);
			::close(fd);
		}

		if ((ptr = mmap(NULL, shm_stat.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) == MAP_FAILED) {
			rc = -errno;
			shm_unlink(shm_name);
			::close(fd);
		}

		shmptr = static_cast<IFT *>(ptr);
		::close(fd);
		return 0;
	}

    public:
	using IFT_t = IFT;
	explicit shm(const char *name) : good(false)
	{
		strncpy((char *)shmname, name, 512);
	}
	void create()
	{
		if (ipc_shm_setup(shmname) == 0)
			good = true;
	}
	void open()
	{
		if (ipc_shm_connect(shmname) == 0)
			good = true;
	}
	bool isgood() const
	{
		return good;
	}
	void close()
	{
		if (isgood())
			shm_unlink(shmname);
	}
	IFT *p()
	{
		return shmptr;
	}
};

} // namespace shm
