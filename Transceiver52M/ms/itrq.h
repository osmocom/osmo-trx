#pragma once
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

#include <atomic>

#include <condition_variable>
#include <mutex>
#include <sys/eventfd.h>
#include <unistd.h>

namespace spsc_detail
{
template <bool block_read, bool block_write>
class spsc_cond_timeout_detail {
	std::condition_variable cond_r, cond_w;
	std::mutex lr, lw;
	std::atomic_int r_flag, w_flag;
	const int timeout_ms = 200;

    public:
	explicit spsc_cond_timeout_detail() : r_flag(0), w_flag(0)
	{
	}

	~spsc_cond_timeout_detail()
	{
	}

	ssize_t spsc_check_r()
	{
		std::unique_lock<std::mutex> lk(lr);
		if (cond_r.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] { return r_flag != 0; })) {
			r_flag--;
			return 1;
		} else {
			return 0;
		}
	}
	ssize_t spsc_check_w()
	{
		std::unique_lock<std::mutex> lk(lw);
		if (cond_w.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] { return w_flag != 0; })) {
			w_flag--;
			return 1;
		} else {
			return 0;
		}
	}
	void spsc_notify_r()
	{
		std::unique_lock<std::mutex> lk(lr);
		r_flag++;
		cond_r.notify_one();
	}
	void spsc_notify_w()
	{
		std::unique_lock<std::mutex> lk(lw);
		w_flag++;
		cond_w.notify_one();
	}
};

template <bool block_read, bool block_write>
class spsc_cond_detail {
	std::condition_variable cond_r, cond_w;
	std::mutex lr, lw;
	std::atomic_int r_flag, w_flag;

    public:
	explicit spsc_cond_detail() : r_flag(0), w_flag(0)
	{
	}

	~spsc_cond_detail()
	{
	}

	ssize_t spsc_check_r()
	{
		std::unique_lock<std::mutex> lk(lr);
		while (r_flag == 0)
			cond_r.wait(lk);
		r_flag--;
		return 1;
	}
	ssize_t spsc_check_w()
	{
		std::unique_lock<std::mutex> lk(lw);
		while (w_flag == 0)
			cond_w.wait(lk);
		w_flag--;
		return 1;
	}
	void spsc_notify_r()
	{
		std::unique_lock<std::mutex> lk(lr);
		r_flag++;
		cond_r.notify_one();
	}
	void spsc_notify_w()
	{
		std::unique_lock<std::mutex> lk(lw);
		w_flag++;
		cond_w.notify_one();
	}
};

// originally designed for select loop integration
template <bool block_read, bool block_write>
class spsc_efd_detail {
	int efd_r, efd_w; /* eventfds used to block/notify readers/writers */

    public:
	explicit spsc_efd_detail()
		: efd_r(eventfd(0, block_read ? 0 : EFD_NONBLOCK)), efd_w(eventfd(1, block_write ? 0 : EFD_NONBLOCK))
	{
	}

	~spsc_efd_detail()
	{
		close(efd_r);
		close(efd_w);
	}

	ssize_t spsc_check_r()
	{
		uint64_t efdr;
		return read(efd_r, &efdr, sizeof(uint64_t));
	}
	ssize_t spsc_check_w()
	{
		uint64_t efdr;
		return read(efd_w, &efdr, sizeof(uint64_t));
	}
	void spsc_notify_r()
	{
		uint64_t efdu = 1;
		write(efd_r, &efdu, sizeof(uint64_t));
	}
	void spsc_notify_w()
	{
		uint64_t efdu = 1;
		write(efd_w, &efdu, sizeof(uint64_t));
	}
	int get_r_efd()
	{
		return efd_r;
	}
	int get_w_efd()
	{
		return efd_w;
	}
};

template <unsigned int SZ, typename ELEM, bool block_read, bool block_write, template <bool, bool> class T>
class spsc : public T<block_read, block_write> {
	static_assert(SZ > 0, "queues need a size...");
	std::atomic<unsigned int> readptr;
	std::atomic<unsigned int> writeptr;

	ELEM buf[SZ];

    public:
	using base_t = T<block_read, block_write>;
	using elem_t = ELEM;
	explicit spsc() : readptr(0), writeptr(0)
	{
	}

	~spsc()
	{
	}

	/*! Adds element to the queue by copying the data.
 *  \param[in] elem input buffer, must match the originally configured queue buffer size!.
 *  \returns true if queue was not full and element was successfully pushed */
	bool spsc_push(const ELEM *elem)
	{
		size_t cur_wp, cur_rp;
		cur_wp = writeptr.load(std::memory_order_relaxed);
		cur_rp = readptr.load(std::memory_order_acquire);
		if ((cur_wp + 1) % SZ == cur_rp) {
			if (block_write)
				base_t::spsc_check_w(); /* blocks, ensures next (!) call succeeds */
			return false;
		}
		buf[cur_wp] = *elem;
		writeptr.store((cur_wp + 1) % SZ, std::memory_order_release);
		if (block_read)
			base_t::spsc_notify_r(); /* fine after release */
		return true;
	}

	/*! Removes element from the queue by copying the data.
 *  \param[in] elem output buffer, must match the originally configured queue buffer size!.
 *  \returns true if queue was not empty and element was successfully removed */
	bool spsc_pop(ELEM *elem)
	{
		size_t cur_wp, cur_rp;
		cur_wp = writeptr.load(std::memory_order_acquire);
		cur_rp = readptr.load(std::memory_order_relaxed);

		if (cur_wp == cur_rp) /* blocks via prep_pop */
			return false;

		*elem = buf[cur_rp];
		readptr.store((cur_rp + 1) % SZ, std::memory_order_release);
		if (block_write)
			base_t::spsc_notify_w();
		return true;
	}

	/*! Reads the read-fd of the queue, which, depending on settings passed on queue creation, blocks.
    * This function can be used to deliberately wait for a non-empty queue on the read side.
    *  \returns result of reading the fd. */
	ssize_t spsc_prep_pop()
	{
		return base_t::spsc_check_r();
	}
};

} // namespace spsc_detail

template <unsigned int SZ, typename ELEM, bool block_read, bool block_write>
class spsc_evfd : public spsc_detail::spsc<SZ, ELEM, block_read, block_write, spsc_detail::spsc_efd_detail> {};
template <unsigned int SZ, typename ELEM, bool block_read, bool block_write>
class spsc_cond : public spsc_detail::spsc<SZ, ELEM, block_read, block_write, spsc_detail::spsc_cond_detail> {};
template <unsigned int SZ, typename ELEM, bool block_read, bool block_write>
class spsc_cond_timeout
	: public spsc_detail::spsc<SZ, ELEM, block_read, block_write, spsc_detail::spsc_cond_timeout_detail> {};