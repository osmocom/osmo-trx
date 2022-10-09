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
#include <deque>
#include <mutex>
#include <vector>

#include "shmif.h"

const int max_ul_rdlen = 1024 * 10;
const int max_dl_rdlen = 1024 * 10;
using sample_t = std::complex<int16_t>;
struct shm_if {
	std::atomic<bool> ms_connected;
	struct {
		shm::sema r;
		shm::sema w;
		std::atomic<uint64_t> ts;
		std::atomic<uint64_t> ts_req;
		std::atomic<size_t> len_written_sps; // ->
		sample_t buffer[max_ul_rdlen];
	} ul;
	struct {
		shm::sema r;
		shm::sema w;
		std::atomic<uint64_t> ts;
		std::atomic<uint64_t> ts_req;
		std::atomic<size_t> len_written_sps;
		sample_t buffer[max_dl_rdlen];
	} dl;
};

// unique up to signed_type/2 diff
// ex: uint8/int8 (250, 0) = -6
template <typename A> auto unsigned_diff(A a, A b) -> typename std::make_signed<A>::type
{
	using stype = typename std::make_signed<A>::type;
	return (a > b) ? static_cast<stype>(a - b) : -static_cast<stype>(b - a);
};

constexpr inline int samp2byte(int v)
{
	return v * sizeof(sample_t);
}
constexpr inline int byte2samp(int v)
{
	return v / sizeof(sample_t);
}

struct ulentry {
	bool done;
	uint64_t ts;
	unsigned int len_in_sps;
	unsigned int read_pos_in_sps;
	sample_t buf[1000];
};
/*
		write: find read index +.. until marked free = "end" of current list

		check:
		within begin, end AND not free?
			y:
			copy (chunk)
				if chunk advance burst buf ptr
			n: next, advance, remove old.
		*/
template <unsigned int num_bursts> class ulburstprovider {
	std::mutex ul_q_m;
	// std::deque<ulentry> ul_q;

	// classic circular buffer
	ulentry foo[num_bursts];
	int current_index; // % num_bursts

	void cur_buf_done()
	{
		foo[current_index].done = true;
		current_index = current_index + 1 % num_bursts;
	}
	bool is_empty()
	{
		return foo[current_index].done = true;
	}
	void reset()
	{
		for (auto &i : foo)
			i = {};
		current_index = 0;
	}
	ulentry &find_free_at_end()
	{
		for (int i = current_index, max_to_search = 0; max_to_search < num_bursts;
		     i = (i + 1 % num_bursts), max_to_search++) {
			if (foo[i].done)
				return foo[i];
		}
		return foo[0]; // FIXME actually broken, q full, wat do?
	}

	void push_back(ulentry &e)
	{
		auto free_buf = find_free_at_end();
		free_buf = e;
		e.done = false;
	}

    public:
	void add(ulentry &e)
	{
		std::lock_guard<std::mutex> foo(ul_q_m);
		push_back(e);
	}
	void get(uint64_t requested_ts, unsigned int req_len_in_sps, sample_t *buf, unsigned int max_buf_write_len)
	{
		std::lock_guard<std::mutex> g(ul_q_m);

		/*
		1) if empty return
		2) if not empty prune stale bursts
		3) if only future bursts also return and zero buf
		*/
		for (int i = current_index, max_to_search = 0; max_to_search < num_bursts;
		     i = (i + 1 % num_bursts), max_to_search++) {
			auto cur_entry = foo[i];
			if (is_empty()) { // might be empty due to advance below!
				memset(buf, 0, samp2byte(req_len_in_sps));
				return;
			}

			if (cur_entry.ts + cur_entry.len_in_sps < requested_ts) { // remove late bursts
				if (i == current_index) // only advance if we are at the front
					cur_buf_done();
				else
					assert(true);
			} else if (cur_entry.ts >= requested_ts + byte2samp(max_buf_write_len)) { // not in range
				memset(buf, 0, samp2byte(req_len_in_sps));
				return;

				// FIXME: what about requested_ts <= entry.ts <= ts + reqlen?
			} else {
				// requested_ts <= cur_entry.ts <= requested_ts + byte2samp(max_write_len)

				auto before_sps = unsigned_diff(cur_entry.ts, requested_ts);

				// at least one whole buffer before our most recent "head" burst?
				// set 0, return.
				if (-before_sps >= byte2samp(max_buf_write_len)) {
					memset(buf, 0, samp2byte(req_len_in_sps));
					return;
				}
				// less than one full buffer before: pad 0
				auto to_pad_sps = -before_sps;
				memset(buf, 0, samp2byte(to_pad_sps));
				requested_ts += to_pad_sps;
				req_len_in_sps -= to_pad_sps;

				if (!req_len_in_sps)
					return;

				// actual burst data after possible 0 pad
				auto max_sps_to_write = std::min(cur_entry.len_in_sps, req_len_in_sps);
				memcpy(&buf[samp2byte(to_pad_sps)], cur_entry.buf, samp2byte(max_sps_to_write));
				requested_ts += max_sps_to_write;
				req_len_in_sps -= max_sps_to_write;
				cur_entry.read_pos_in_sps += max_sps_to_write;

				//this buf is done...
				if (cur_entry.read_pos_in_sps == cur_entry.len_in_sps) {
					cur_buf_done();
				}

				if (!req_len_in_sps)
					return;
			}
		}
	}
};

class trxmsif {
	shm::shm<shm_if> m;
	shm_if *ptr;

	ulburstprovider<10> p;

	template <typename T> void read(T &direction, size_t howmany_sps, uint64_t *read_ts, sample_t *outbuf)
	{
		static int readoffset_sps;
		// auto &direction = ptr->dl;
		auto buf = &direction.buffer[0];
		size_t len_avail_sps = direction.len_written_sps.load();

		auto left_to_read = len_avail_sps - readoffset_sps;

		shm::mtx_log::print_guard() << "\tr @" << direction.ts.load() << " " << readoffset_sps << std::endl;

		// no data, wait for new buffer, maybe some data left afterwards
		if (!left_to_read) {
			assert(readoffset_sps == len_avail_sps);
			readoffset_sps = 0;
			direction.r.reset_unsafe();
			direction.ts_req = (*read_ts);
			direction.w.set(1);
			direction.r.wait_and_reset(1);
			assert(*read_ts != direction.ts.load());
			// shm::sema_guard g(dl.r, dl.w);
			*read_ts = direction.ts.load();
			len_avail_sps = direction.len_written_sps.load();
			readoffset_sps += howmany_sps;
			assert(len_avail_sps >= howmany_sps);
			memcpy(outbuf, buf, samp2byte(howmany_sps));

			shm::mtx_log::print_guard() << "\tr+ " << *read_ts << " " << howmany_sps << std::endl;
			return;
		}

		*read_ts = direction.ts.load() + readoffset_sps;
		left_to_read = len_avail_sps - readoffset_sps;

		// data left from prev read
		if (left_to_read >= howmany_sps) {
			memcpy(outbuf, &buf[readoffset_sps], samp2byte(howmany_sps));
			readoffset_sps += howmany_sps;

			shm::mtx_log::print_guard() << "\tr++ " << *read_ts << " " << howmany_sps << std::endl;
			return;
		} else {
			memcpy(outbuf, &buf[readoffset_sps], samp2byte(left_to_read));
			readoffset_sps = 0;
			auto still_left_to_read = howmany_sps - left_to_read;
			{
				direction.r.reset_unsafe();
				direction.ts_req = (*read_ts);
				direction.w.set(1);
				direction.r.wait_and_reset(1);
				assert(*read_ts != direction.ts.load());
				len_avail_sps = direction.len_written_sps.load();
				assert(len_avail_sps >= still_left_to_read);
				memcpy(&outbuf[left_to_read], buf, samp2byte(still_left_to_read));
				readoffset_sps += still_left_to_read;
				shm::mtx_log::print_guard()
					<< "\tr+++2 " << *read_ts << " " << howmany_sps << " " << still_left_to_read
					<< " new @" << direction.ts.load() << std::endl;
			}
		}
	}

    public:
	trxmsif() : m("trx-ms-if")
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

	/* is being read from ms side */
	void read_dl(size_t howmany_sps, uint64_t *read_ts, sample_t *outbuf)
	{
		return read(ptr->dl, howmany_sps, read_ts, outbuf);
	}

	/* is being read from trx/network side */
	void read_ul(size_t howmany_sps, uint64_t *read_ts, sample_t *outbuf)
	{
		// if (ptr->ms_connected != true) {
			memset(outbuf, 0, samp2byte(howmany_sps));
		// 	return;
		// }
		// return read(ptr->ul, howmany_sps, read_ts, outbuf);
	}

	void write_dl(size_t howmany_sps, uint64_t write_ts, sample_t *inbuf)
	{
		auto &dl = ptr->dl;
		auto buf = &dl.buffer[0];
		if (ptr->ms_connected != true)
			return;

		assert(sizeof(dl.buffer) >= samp2byte(howmany_sps));
		// print_guard() << "####w " << std::endl;

		{
			shm::sema_wait_guard g(dl.w, dl.r);

			memcpy(buf, inbuf, samp2byte(howmany_sps));
			dl.ts.store(write_ts);
			dl.len_written_sps.store(howmany_sps);
		}
		shm::mtx_log::print_guard() << std::endl
					    << "####w+ " << write_ts << " " << howmany_sps << std::endl
					    << std::endl;
	}

	void write_ul(size_t howmany_sps_sps, uint64_t write_ts, sample_t *inbuf)
	{
		auto &ul = ptr->ul;
		assert(sizeof(ul.buffer) >= samp2byte(howmany_sps_sps));
		// print_guard() << "####w " << std::endl;

		ulentry e;
		e.ts = write_ts;
		e.len_in_sps = howmany_sps_sps;
		e.done = false;
		e.read_pos_in_sps = 0;
		assert(sizeof(e.buf) >= samp2byte(howmany_sps_sps));
		memcpy(e.buf, inbuf, samp2byte(howmany_sps_sps));
		p.add(e);

		shm::mtx_log::print_guard() << std::endl
					    << "####q+ " << write_ts << " " << howmany_sps_sps << std::endl
					    << std::endl;
	}

	void drive_tx()
	{
		auto &ul = ptr->ul;
		auto buf = &ul.buffer[0];
		const auto max_write_len = sizeof(ul.buffer);

		// ul_q_m.lock();
		// ul_q.push_front(e);
		// ul_q_m.unlock();
		// ul.w.wait_and_reset();

		// no read waiting for a write
		if (!ul.w.check_unsafe(1))
			return;

		// FIXME: store written, notify after get!

		auto requested_ts = ul.ts_req.load();

		p.get(requested_ts, byte2samp(max_write_len), buf, max_write_len);

		// memset(buf, 0, max_write_len);
		ul.ts.store(requested_ts);
		ul.len_written_sps.store(byte2samp(max_write_len));
		ul.w.reset_unsafe();
		ul.r.set(1);
	}

	void signal_read_start()
	{ /* nop */
	}
};
