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
#else
#error wat? no device..
#endif

#include "Complex.h"
#include "GSMCommon.h"
#include "itrq.h"
#include "threadpool.h"

const unsigned int ONE_TS_BURST_LEN = (3 + 58 + 26 + 58 + 3 + 8.25) * 4 /*sps*/;
const unsigned int NUM_RXQ_FRAMES = 1; // rx thread <-> upper rx queue
const unsigned int SCH_LEN_SPS = (ONE_TS_BURST_LEN * 8 /*ts*/ * 12 /*frames*/);

template <typename T>
void clamp_array(T *start2, unsigned int len, T max)
{
	for (int i = 0; i < len; i++) {
		const T t1 = start2[i] < -max ? -max : start2[i];
		const T t2 = t1 > max ? max : t1;
		start2[i] = t2;
	}
}

namespace cvt_internal
{

template <typename SRC_T, typename ST>
void convert_and_scale_i(float *dst, const SRC_T *src, unsigned int src_len, ST scale)
{
	for (unsigned int i = 0; i < src_len; i++)
		dst[i] = static_cast<float>(src[i]) * scale;
}

template <typename DST_T, typename ST>
void convert_and_scale_i(DST_T *dst, const float *src, unsigned int src_len, ST scale)
{
	for (unsigned int i = 0; i < src_len; i++)
		dst[i] = static_cast<DST_T>(src[i] * scale);
}

template <typename ST>
void convert_and_scale_i(float *dst, const float *src, unsigned int src_len, ST scale)
{
	for (unsigned int i = 0; i < src_len; i++)
		dst[i] = src[i] * scale;
}

template <typename T>
struct is_complex : std::false_type {
	using baset = T;
};

template <typename T>
struct is_complex<std::complex<T>> : std::true_type {
	using baset = typename std::complex<T>::value_type;
};

template <typename T>
struct is_complex<Complex<T>> : std::true_type {
	using baset = typename Complex<T>::value_type;
};

} // namespace cvt_internal

template <typename DST_T, typename SRC_T, typename ST>
void convert_and_scale(DST_T *dst, const SRC_T *src, unsigned int src_len, ST scale)
{
	using vd = typename cvt_internal::is_complex<DST_T>::baset;
	using vs = typename cvt_internal::is_complex<SRC_T>::baset;
	return cvt_internal::convert_and_scale_i((vd *)dst, (vs *)src, src_len, scale);
}

struct one_burst {
	one_burst()
	{
	}
	GSM::Time gsmts;
	union {
		blade_sample_type burst[ONE_TS_BURST_LEN];
		char sch_bits[148];
	};
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

static struct sched_params {
	enum thread_names { U_CTL = 0, U_RX, U_TX, SCH_SEARCH, MAIN, LEAKCHECK, RXRUN, TXRUN, _THRD_NAME_COUNT };
	enum target { ODROID = 0, PI4 };
	const char *name;
	int core;
	int schedtype;
	int prio;
} schdp[][sched_params::_THRD_NAME_COUNT]{
	{
		{ "upper_ctrl", 2, SCHED_RR, sched_get_priority_max(SCHED_RR) },
		{ "upper_rx", 2, SCHED_RR, sched_get_priority_max(SCHED_RR) - 5 },
		{ "upper_tx", 2, SCHED_RR, sched_get_priority_max(SCHED_RR) - 1 },

		{ "sch_search", 3, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 5 },
		{ "main", 3, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 5 },
		{ "leakcheck", 3, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 10 },

		{ "rxrun", 4, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 2 },
		{ "txrun", 5, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 1 },
	},
	{
		{ "upper_ctrl", 1, SCHED_RR, sched_get_priority_max(SCHED_RR) },
		{ "upper_rx", 1, SCHED_RR, sched_get_priority_max(SCHED_RR) - 5 },
		{ "upper_tx", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 1 },

		{ "sch_search", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 5 },
		{ "main", 3, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 5 },
		{ "leakcheck", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 10 },

		{ "rxrun", 2, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 2 },
		{ "txrun", 3, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 1 },
	},
};

using ts_hitter_q_t = spsc_cond<64, GSM::Time, true, false>;

struct ms_trx : public BASET {
	using base = BASET;
	static dummylog dummy_log;
	unsigned int mTSC;
	unsigned int mBSIC;
	int timing_advance;
	bool do_auto_gain;

	std::thread lower_rx_task;
	std::thread lower_tx_task;
	std::thread *calcrval_task;

	// provides bursts to upper rx thread
	rx_queue_t rxqueue;

	blade_sample_type *first_sch_buf;
	blade_sample_type *burst_copy_buffer;

	uint64_t first_sch_buf_rcv_ts;
	std::atomic<bool> rcv_done;
	std::atomic<bool> sch_thread_done;

	int64_t temp_ts_corr_offset = 0;
	int64_t first_sch_ts_start = -1;

	time_keeper timekeeper;
	int hw_cpus;
	sched_params::target hw_target;
	single_thread_pool worker_thread;

	void start();
	std::atomic<bool> upper_is_ready;
	void set_upper_ready(bool is_ready);

	bool handle_sch_or_nb();
	bool handle_sch(bool first = false);
	bool decode_sch(float *bits, bool update_global_clock);
	SCH_STATE search_for_sch(dev_buf_t *rcd);
	void grab_bursts(dev_buf_t *rcd);

	int init_device();
	int init_dev_and_streams();
	void stop_threads();
	void *rx_cb(ms_trx *t);
	void *tx_cb();
	void maybe_update_gain(one_burst &brst);

	ms_trx()
		: timing_advance(0), do_auto_gain(false), rxqueue(), first_sch_buf(new blade_sample_type[SCH_LEN_SPS]),
		  burst_copy_buffer(new blade_sample_type[ONE_TS_BURST_LEN]), rcv_done{ false },
		  sch_thread_done{ false }, hw_cpus(std::thread::hardware_concurrency()),
		  hw_target(hw_cpus > 4 ? sched_params::target::ODROID : sched_params::target::PI4)
	{
		std::cerr << "scheduling for: " << (hw_cpus > 4 ? "odroid" : "pi4") << std::endl;
		set_name_aff_sched(worker_thread.get_handle(), sched_params::thread_names::SCH_SEARCH);
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

	void set_name_aff_sched(sched_params::thread_names name)
	{
		set_name_aff_sched(pthread_self(), name);
	}

	void set_name_aff_sched(std::thread::native_handle_type h, sched_params::thread_names name)
	{
		auto tgt = schdp[hw_target][name];
		// std::cerr << "scheduling for: " << tgt.name << ":" << tgt.core << std::endl;
		set_name_aff_sched(h, tgt.name, tgt.core, tgt.schedtype, tgt.prio);
	}

    private:
	void set_name_aff_sched(std::thread::native_handle_type h, const char *name, int cpunum, int schedtype,
				int prio)
	{
		pthread_setname_np(h, name);

		cpu_set_t cpuset;

		CPU_ZERO(&cpuset);
		CPU_SET(cpunum, &cpuset);

		if (pthread_setaffinity_np(h, sizeof(cpuset), &cpuset) < 0) {
			std::cerr << name << " affinity: errreur! " << std::strerror(errno);
			return exit(0);
		}

		sched_param sch_params;
		sch_params.sched_priority = prio;
		if (pthread_setschedparam(h, schedtype, &sch_params) < 0) {
			std::cerr << name << " sched: errreur! " << std::strerror(errno);
			return exit(0);
		}
	}
};
