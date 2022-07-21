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
#include <complex>
#include <cassert>
#include "shmif.h"

#include <mutex>

const int max_ul_rdlen = 1024 * 10;
const int max_dl_rdlen = 1024 * 10;
using sample_t = std::complex<int16_t>;
struct shm_if {
	std::atomic<bool> ms_connected;
	struct {
		shm::sema r;
		shm::sema w;
		std::atomic<uint64_t> ts;
		std::atomic<size_t> len_req; // <-
		std::atomic<size_t> len_written; // ->
		sample_t buffer[max_ul_rdlen];
	} ul;
	struct {
		shm::sema r;
		shm::sema w;
		std::atomic<uint64_t> ts;
		std::atomic<size_t> len_req;
		std::atomic<size_t> len_written;
		sample_t buffer[max_dl_rdlen];
	} dl;
};

// unique up to signed_type/2 diff
template <typename A> auto unsigned_diff(A a, A b) -> typename std::make_signed<A>::type
{
	using stype = typename std::make_signed<A>::type;
	return (a > b) ? static_cast<stype>(a - b) : -static_cast<stype>(b - a);
};

class trxmsif {
	shm::shm<shm_if> m;
	shm_if *ptr;
	volatile int dl_readoffset;
	bool first;

	int samp2byte(int v)
	{
		return v * sizeof(sample_t);
	}

    public:
	trxmsif() : m("trx-ms-if"), dl_readoffset(0), first(true)
	{
	}

	bool create()
	{
		m.create();
		ptr = m.p();
		return m.isgood();
	}
	bool connect()
	{
		m.open();
		ptr = m.p();
		ptr->ms_connected = true;
		ptr->dl.w.set(1);
		return m.isgood();
	}
	bool good()
	{
		return m.isgood();
	}
	bool is_connected()
	{
		return ptr->ms_connected == true;
	}

	void write_dl(size_t howmany, uint64_t write_ts, sample_t *inbuf)
	{
		auto &dl = ptr->dl;
		auto buf = &dl.buffer[0];
		// if (ptr->ms_connected != true)
		// 	return;

		assert(sizeof(dl.buffer) >= samp2byte(howmany));
		// print_guard() << "####w " << std::endl;

		{
			shm::sema_wait_guard g(dl.w, dl.r);

			memcpy(buf, inbuf, samp2byte(howmany));
			dl.ts.store(write_ts);
			dl.len_written.store(howmany);
		}
		shm::mtx_log::print_guard() << std::endl << "####w+ " << write_ts << " " << howmany << std::endl << std::endl;
	}

	void signal_read_start()
	{ /* nop */
	}

	void read_dl(size_t howmany, uint64_t *read_ts, sample_t *outbuf)
	{
		auto &dl = ptr->dl;
		auto buf = &dl.buffer[0];
		size_t len_avail = dl.len_written.load();

		auto left_to_read = len_avail - dl_readoffset;

		shm::mtx_log::print_guard() << "\tr @" << dl.ts.load() << " " << dl_readoffset << std::endl;

		// no data, wait for new buffer, maybe some data left afterwards
		if (!left_to_read) {
			assert(dl_readoffset == len_avail);
			dl_readoffset = 0;
			dl.r.reset_unsafe();
			dl.w.set(1);
			dl.r.wait_and_reset(1);
			assert(*read_ts != dl.ts.load());
			// shm::sema_guard g(dl.r, dl.w);
			*read_ts = dl.ts.load();
			len_avail = dl.len_written.load();
			dl_readoffset += howmany;
			assert(len_avail >= howmany);
			memcpy(outbuf, buf, samp2byte(howmany));

			shm::mtx_log::print_guard() << "\tr+ " << *read_ts << " " << howmany << std::endl;
			return;
		}

		*read_ts = dl.ts.load() + dl_readoffset;
		left_to_read = len_avail - dl_readoffset;

		// data left from prev read
		if (left_to_read >= howmany) {
			memcpy(outbuf, buf, samp2byte(howmany));
			dl_readoffset += howmany;

			shm::mtx_log::print_guard() << "\tr++ " << *read_ts << " " << howmany << std::endl;
			return;
		} else {
			memcpy(outbuf, buf, samp2byte(left_to_read));
			dl_readoffset = 0;
			auto still_left_to_read = howmany - left_to_read;
			{
				dl.r.reset_unsafe();
				dl.w.set(1);
				dl.r.wait_and_reset(1);
				assert(*read_ts != dl.ts.load());
				len_avail = dl.len_written.load();
				dl_readoffset += still_left_to_read;
				assert(len_avail >= still_left_to_read);
				memcpy(outbuf + left_to_read, buf, samp2byte(still_left_to_read));
				shm::mtx_log::print_guard() << "\tr+++2 " << *read_ts << " " << howmany << " "
						   << still_left_to_read << " new @" << dl.ts.load() << std::endl;
			}
		}
	}

	void read_ul(size_t howmany, uint64_t *read_ts, sample_t *outbuf)
	{
		// if (ptr->ms_connected != true) {
		memset(outbuf, 0, samp2byte(howmany));
		return;
		// }
	}
};
