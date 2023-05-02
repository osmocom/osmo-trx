
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
#include "GSMCommon.h"
#include <atomic>
#include <cassert>
#include <complex>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <thread>
#include <fstream>

#include "ms.h"

extern "C" {
#include "sch.h"
}

dummylog ms_trx::dummy_log;

#ifdef DBGXX
const int offsetrange = 200;
const int offset_start = -15;
static int offset_ctr = 0;
#endif

template <>
std::atomic<bool> ms_trx::base::stop_lower_threads_flag(false);

int ms_trx::init_dev_and_streams()
{
	int status = 0;
	status = base::init_device(rx_bh(), tx_bh());
	if (status < 0) {
		std::cerr << "failed to init dev!" << std::endl;
		return -1;
	}
	return status;
}

bh_fn_t ms_trx::rx_bh()
{
	return [this](dev_buf_t *rcd) -> int {
		if (this->search_for_sch(rcd) == SCH_STATE::FOUND)
			this->grab_bursts(rcd);
		return 0;
	};
}

bh_fn_t ms_trx::tx_bh()
{
	return [this](dev_buf_t *rcd) -> int {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
		auto y = this;
#pragma GCC diagnostic pop
		/* nothing to do here */
		return 0;
	};
}

void ms_trx::start()
{
	if (stop_lower_threads_flag)
		return;
	auto fn = get_rx_burst_handler_fn(rx_bh());
	lower_rx_task = std::thread(fn);
	set_name_aff_sched(lower_rx_task.native_handle(), sched_params::thread_names::RXRUN);

	usleep(1000);
	auto fn2 = get_tx_burst_handler_fn(tx_bh());
	lower_tx_task = std::thread(fn2);
	set_name_aff_sched(lower_tx_task.native_handle(), sched_params::thread_names::TXRUN);

	actually_enable_streams();
}

void ms_trx::set_upper_ready(bool is_ready)
{
	upper_is_ready = is_ready;
}

void ms_trx::stop_threads()
{
	std::cerr << "killing threads..." << std::endl;
	stop_lower_threads_flag = true;
	close_device();
	std::cerr << "dev closed..." << std::endl;
	lower_rx_task.join();
	std::cerr << "L rx dead..." << std::endl;
	lower_tx_task.join();
	std::cerr << "L tx dead..." << std::endl;
}

void ms_trx::submit_burst(blade_sample_type *buffer, int len, GSM::Time target)
{
	int64_t now_ts;
	GSM::Time now_time;
	target.incTN(3); // ul dl offset
	int target_fn = target.FN();
	int target_tn = target.TN();
	timekeeper.get_both(&now_time, &now_ts);

	auto diff_fn = GSM::FNDelta(target_fn, now_time.FN());
	int diff_tn = (target_tn - (int)now_time.TN()) % 8;
	auto tosend = GSM::Time(diff_fn, 0);

	if (diff_tn > 0)
		tosend.incTN(diff_tn);
	else
		tosend.decTN(-diff_tn);

	// in theory fn equal and tn+3 equal is also a problem...
	if (diff_fn < 0 || (diff_fn == 0 && (now_time.TN() - target_tn < 1))) {
		std::cerr << "## TX too late?! fn DIFF:" << diff_fn << " tn LOCAL: " << now_time.TN()
			  << " tn OTHER: " << target_tn << std::endl;
		return;
	}

	auto check = now_time + tosend;
	int64_t send_ts = now_ts + tosend.FN() * 8 * ONE_TS_BURST_LEN + tosend.TN() * ONE_TS_BURST_LEN - timing_advance;
#ifdef DBGXX
	std::cerr << "## fn DIFF: " << diff_fn << " ## tn DIFF: " << diff_tn << " tn LOCAL/OTHER: " << now_time.TN()
		  << "/" << target_tn << " tndiff" << diff_tn << " tosend:" << tosend.FN() << ":" << tosend.TN()
		  << " check: " << check.FN() << ":" << check.TN() << " target: " << target.FN() << ":" << target.TN()
		  << " ts now: " << now_ts << " target ts:" << send_ts << std::endl;
#endif
#if 1
	unsigned int pad = 4 * 4;
	blade_sample_type buf2[len + pad];
	std::fill(buf2, buf2 + pad, 0);
	memcpy(&buf2[pad], buffer, len * sizeof(blade_sample_type));

	assert(target.FN() == check.FN());
	assert(target.TN() == check.TN());
	submit_burst_ts(buf2, len + pad, send_ts - pad);
#else
	submit_burst_ts(buffer, len, send_ts);
#endif
}
