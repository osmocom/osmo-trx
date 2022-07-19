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

#include <cstring>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <cerrno>

namespace shm
{

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

class signal_guard {
	shmmutex &m;
	shmcond &s;

    public:
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
};

} // namespace shm
