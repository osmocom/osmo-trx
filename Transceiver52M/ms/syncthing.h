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
#include <cassert>
#include <complex>
#include <cstdint>
#include <mutex>
#include <iostream>
#include <thread>

#if defined(BUILDBLADE)
#include "bladerf_specific.h"
#define BASET blade_hw<ms_trx>
#elif defined(BUILDUHD)
#include "uhd_specific.h"
#define BASET uhd_hw<ms_trx>
#elif defined(BUILDIPC)
#include "ipc_specific.h"
#define BASET ipc_hw<ms_trx>
#else
#error wat? no device..
#endif

#include "GSMCommon.h"
#include "itrq.h"

const unsigned int ONE_TS_BURST_LEN = (3 + 58 + 26 + 58 + 3 + 8.25) * 4 /*sps*/;
const unsigned int NUM_RXQ_FRAMES = 12 * 1; // rx thread <-> upper rx queue
const unsigned int SCH_LEN_SPS = (ONE_TS_BURST_LEN * 8 /*ts*/ * 12 /*frames*/);

template <typename T> void clamp_array(T *start2, unsigned int len, T max)
{
	for (int i = 0; i < len; i++) {
		const T t1 = start2[i] < -max ? -max : start2[i];
		const T t2 = t1 > max ? max : t1;
		start2[i] = t2;
	}
}
template <typename DST_T, typename SRC_T, typename ST>
void convert_and_scale(void *dst, void *src, unsigned int src_len, ST scale)
{
	for (unsigned int i = 0; i < src_len; i++)
		reinterpret_cast<DST_T *>(dst)[i] = static_cast<DST_T>((reinterpret_cast<SRC_T *>(src)[i])) * scale;
}
template <typename DST_T, typename SRC_T> void convert_and_scale_default(void *dst, void *src, unsigned int src_len)
{
	return convert_and_scale<DST_T, SRC_T>(dst, src, src_len, SAMPLE_SCALE_FACTOR);
}

struct one_burst {
	GSM::Time gsmts;
	blade_sample_type burst[ONE_TS_BURST_LEN];
};

using rx_queue_t = spsc_cond<8 * NUM_RXQ_FRAMES, one_burst, true, false>;

enum class SCH_STATE { SEARCHING, FOUND };

class dummylog : private std::streambuf {
	std::ostream null_stream;

    public:
	dummylog() : null_stream(this){};
	~dummylog() override{};
	std::ostream &operator()()
	{
		return null_stream;
	}
	int overflow(int c) override
	{
		return c;
	}
};

// keeps relationship between gsm time and (continuously adjusted) ts
class time_keeper {
	GSM::Time global_time_keeper;
	int64_t global_ts_keeper;
	std::mutex m;

    public:
	time_keeper() : global_time_keeper(0), global_ts_keeper(0)
	{
	}

	void set(GSM::Time t, int64_t ts)
	{
		std::lock_guard<std::mutex> g(m);
		global_time_keeper = t;
		global_ts_keeper = ts;
	}
	void inc_both()
	{
		std::lock_guard<std::mutex> g(m);
		global_time_keeper.incTN(1);
		global_ts_keeper += ONE_TS_BURST_LEN;
	}
	void inc_and_update(int64_t new_ts)
	{
		std::lock_guard<std::mutex> g(m);
		global_time_keeper.incTN(1);
		global_ts_keeper = new_ts;
		// std::cerr << "u " << new_ts << std::endl;
	}
	void inc_and_update_safe(int64_t new_ts)
	{
		std::lock_guard<std::mutex> g(m);
		auto diff = new_ts - global_ts_keeper;
		assert(diff < 1.5 * ONE_TS_BURST_LEN);
		assert(diff > 0.5 * ONE_TS_BURST_LEN);
		global_time_keeper.incTN(1);
		global_ts_keeper = new_ts;
		// std::cerr << "s " << new_ts << std::endl;
	}
	void dec_by_one()
	{
		std::lock_guard<std::mutex> g(m);
		global_time_keeper.decTN(1);
		global_ts_keeper -= ONE_TS_BURST_LEN;
	}
	auto get_ts()
	{
		std::lock_guard<std::mutex> g(m);
		return global_ts_keeper;
	}
	auto gsmtime()
	{
		std::lock_guard<std::mutex> g(m);
		return global_time_keeper;
	}
	void get_both(GSM::Time *t, int64_t *ts)
	{
		std::lock_guard<std::mutex> g(m);
		*t = global_time_keeper;
		*ts = global_ts_keeper;
	}
};

using ts_hitter_q_t = spsc_cond<64, GSM::Time, true, false>;

struct ms_trx : public BASET {
	using base = BASET;
	static dummylog dummy_log;
	unsigned int mTSC;
	unsigned int mBSIC;
	int timing_advance;
	bool do_auto_gain;

	std::thread rx_task;
	std::thread tx_task;
	std::thread *calcrval_task;

	// provides bursts to upper rx thread
	rx_queue_t rxqueue;
#ifdef SYNCTHINGONLY
	ts_hitter_q_t ts_hitter_q;
#endif
	blade_sample_type *first_sch_buf;
	blade_sample_type *burst_copy_buffer;

	uint64_t first_sch_buf_rcv_ts;
	std::atomic<bool> rcv_done;
	std::atomic<bool> sch_thread_done;

	int64_t temp_ts_corr_offset = 0;
	int64_t first_sch_ts_start = -1;

	time_keeper timekeeper;

	void start();

	bool handle_sch_or_nb(bool first = false);
	bool handle_sch(bool first = false);
	bool decode_sch(float *bits, bool update_global_clock);
	SCH_STATE search_for_sch(dev_buf_t *rcd);
	void grab_bursts(dev_buf_t *rcd);

	int init_device();
	int init_streams(void *rx_cb, void *tx_cb);
	int init_dev_and_streams(void *rx_cb, void *tx_cb);
	void stop_threads();
	void *rx_cb(ms_trx *t);
	void *tx_cb();
	void maybe_update_gain(one_burst &brst);

	ms_trx()
		: timing_advance(0), do_auto_gain(false), rxqueue(), first_sch_buf(new blade_sample_type[SCH_LEN_SPS]),
		  burst_copy_buffer(new blade_sample_type[ONE_TS_BURST_LEN]), rcv_done{ false }, sch_thread_done{ false }
	{
	}

	virtual ~ms_trx()
	{
		delete[] burst_copy_buffer;
		delete[] first_sch_buf;
	}
	bh_fn_t rx_bh();
	bh_fn_t tx_bh();

	void submit_burst(blade_sample_type *buffer, int len, GSM::Time);
	void set_ta(int val)
	{
		assert(val > -127 && val < 128);
		timing_advance = val * 4;
	}
};
