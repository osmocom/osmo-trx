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

const int max_ul_rdlen = 1024 * 10;
const int max_dl_rdlen = 1024 * 10;
using sample_t = std::complex<int16_t>;
struct shm_if {
	std::atomic<bool> ms_connected;
	struct {
		shm::shmmutex m;
		shm::shmcond c;
		std::atomic<uint64_t> ts;
		std::atomic<size_t> len_req; // <-
		std::atomic<size_t> len_written; // ->
		sample_t buffer[max_ul_rdlen];
	} ul;
	struct {
		shm::shmmutex writemutex;
		shm::shmcond rdy2write;
		shm::shmmutex readmutex;
		shm::shmcond rdy2read;
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
	int dl_readoffset;

	int samp2byte(int v)
	{
		return v * sizeof(sample_t);
	}

    public:
	trxmsif() : m("trx-ms-if"), dl_readoffset(0)
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
		return m.isgood();
	}
	bool good()
	{
		return m.isgood();
	}

	void write_dl(size_t howmany, uint64_t write_ts, sample_t *inbuf)
	{
		auto &dl = ptr->dl;
		auto buf = &dl.buffer[0];
		// if (ptr->ms_connected != true)
		// 	return;

		assert(sizeof(dl.buffer) >= samp2byte(howmany));

		{
			shm::signal_guard g(dl.writemutex, dl.rdy2write, dl.rdy2read);

			memcpy(buf, inbuf, samp2byte(howmany));
			dl.ts = write_ts;
			dl.len_written = howmany;
		}
	}

	void read_dl(size_t howmany, uint64_t* read_ts, sample_t *outbuf)
	{
		auto &dl = ptr->dl;
		auto buf = &dl.buffer[0];
		size_t len_avail = dl.len_written;
		uint64_t ts = dl.ts;

		auto left_to_read = len_avail - dl_readoffset;

		// no data, wait for new buffer, maybe some data left afterwards
		if (!left_to_read) {
			shm::signal_guard g(dl.readmutex, dl.rdy2read, dl.rdy2write);
			*read_ts = dl.ts;
			len_avail = dl.len_written;
			dl_readoffset += howmany;
			assert(len_avail >= howmany);
			memcpy(outbuf, buf, samp2byte(howmany));
			return;
		}

		*read_ts = dl.ts + dl_readoffset;
		left_to_read = len_avail - dl_readoffset;

		// data left from prev read
		if (left_to_read >= howmany) {
			memcpy(outbuf, buf, samp2byte(howmany));
			dl_readoffset += howmany;
			return;
		} else {
			memcpy(outbuf, buf, samp2byte(left_to_read));
			dl_readoffset = 0;
			auto still_left_to_read = howmany - left_to_read;
			{
				shm::signal_guard g(dl.readmutex, dl.rdy2read, dl.rdy2write);
				len_avail = dl.len_written;
				dl_readoffset += still_left_to_read;
				assert(len_avail >= still_left_to_read);
				memcpy(outbuf + left_to_read, buf, samp2byte(still_left_to_read));
			}
		}
	}

	void read_ul(size_t howmany, uint64_t* read_ts, sample_t *outbuf)
	{
		// if (ptr->ms_connected != true) {
		memset(outbuf, 0, samp2byte(howmany));
		return;
		// }
	}
};
