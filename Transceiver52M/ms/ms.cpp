
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

#include "sigProcLib.h"

#include "ms.h"
#include "ms_rx_burst.h"
#include "grgsm_vitac/grgsm_vitac.h"

extern "C" {
#include "sch.h"
#include "convolve.h"
#include "convert.h"
}

dummylog ms_trx::dummy_log;

#ifdef DBGXX
const int offsetrange = 200;
const int offset_start = -15;
static int offset_ctr = 0;
#endif

void tx_test(ms_trx *t, ts_hitter_q_t *q, unsigned int *tsc)
{
	sched_param sch_params;
	sch_params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch_params);

	auto burst = genRandAccessBurst(0, 4, 0);
	scaleVector(*burst, t->txFullScale * 0.7);

	// float -> int16
	blade_sample_type burst_buf[burst->size()];
	convert_and_scale<int16_t, float>(burst_buf, burst->begin(), burst->size() * 2, 1);

	while (1) {
		GSM::Time target;
		while (!q->spsc_pop(&target)) {
			q->spsc_prep_pop();
		}

		std::cerr << std::endl << "\x1B[32m hitting " << target.FN() << "\033[0m" << std::endl;

		int timing_advance = 0;
		int64_t now_ts;
		GSM::Time now_time;
		target.incTN(3); // ul dl offset
		int target_fn = target.FN();
		int target_tn = target.TN();
		t->timekeeper.get_both(&now_time, &now_ts);

		auto diff_fn = GSM::FNDelta(target_fn, now_time.FN());
		int diff_tn = (target_tn - (int)now_time.TN()) % 8;
		auto tosend = GSM::Time(diff_fn, 0);

		if (diff_tn > 0)
			tosend.incTN(diff_tn);
		else if (diff_tn < 0)
			tosend.decTN(-diff_tn);

		// in thory fn equal and tn+3 equal is also a problem...
		if (diff_fn < 0 || (diff_fn == 0 && (now_time.TN() - target_tn < 1))) {
			std::cerr << "## TX too late?! fn DIFF:" << diff_fn << " tn LOCAL: " << now_time.TN()
				  << " tn OTHER: " << target_tn << std::endl;
			return;
		}

		auto check = now_time + tosend;
		int64_t send_ts =
			now_ts + tosend.FN() * 8 * ONE_TS_BURST_LEN + tosend.TN() * ONE_TS_BURST_LEN - timing_advance;

#ifdef DBGXX
		std::cerr << "## fn DIFF: " << diff_fn << " ## tn DIFF: " << diff_tn << " tn LOCAL: " << now_time.TN()
			  << " tn OTHER: " << target_tn << " tndiff" << diff_tn << " tosend:" << tosend.FN() << ":"
			  << tosend.TN() << " calc: " << check.FN() << ":" << check.TN() << " target: " << target.FN()
			  << ":" << target.TN() << " ts now: " << now_ts << " target ts:" << send_ts << std::endl;
#endif

		unsigned int pad = 4 * 25;
		blade_sample_type buf2[burst->size() + pad];
		memset(buf2, 0, pad * sizeof(blade_sample_type));
		memcpy(&buf2[pad], burst_buf, burst->size() * sizeof(blade_sample_type));

		assert(target.FN() == check.FN());
		assert(target.TN() == check.TN());
		assert(target.FN() % 51 == 21);
#ifdef DBGXX
		auto this_offset = offset_start + (offset_ctr++ % offsetrange);
		std::cerr << "-- O " << this_offset << std::endl;
		send_ts = now_ts - timing_advance +
			  ((target.FN() * 8 + (int)target.TN()) - (now_time.FN() * 8 + (int)now_time.TN())) *
				  ONE_TS_BURST_LEN;
#endif
		t->submit_burst_ts(buf2, burst->size() + pad, send_ts - pad);
#ifdef DBGXX
		signalVector test(burst->size() + pad);
		convert_and_scale<float, int16_t>(test.begin(), buf2, burst->size() * 2 + pad, 1.f / float(scale));
		estim_burst_params ebp;
		auto det = detectAnyBurst(test, 0, 4, 4, CorrType::RACH, 40, &ebp);
		if (det > 0)
			std::cerr << "## Y " << ebp.toa << std::endl;
		else
			std::cerr << "## NOOOOOOOOO " << ebp.toa << std::endl;
#endif
	}
}
#ifdef SYNCTHINGONLY
template <typename A> auto parsec(std::vector<std::string> &v, A &itr, std::string arg, bool *rv)
{
	if (*itr == arg) {
		*rv = true;
		return true;
	}
	return false;
}

template <typename A, typename B, typename C>
bool parsec(std::vector<std::string> &v, A &itr, std::string arg, B f, C *rv)
{
	if (*itr == arg) {
		itr++;
		if (itr != v.end()) {
			*rv = f(itr->c_str());
			return true;
		}
	}
	return false;
}
template <typename A> bool parsec(std::vector<std::string> &v, A &itr, std::string arg, int scale, int *rv)
{
	return parsec(
		v, itr, arg, [scale](const char *v) -> auto{ return atoi(v) * scale; }, rv);
}
template <typename A> bool parsec(std::vector<std::string> &v, A &itr, std::string arg, int scale, unsigned int *rv)
{
	return parsec(
		v, itr, arg, [scale](const char *v) -> auto{ return atoi(v) * scale; }, rv);
}

int main(int argc, char *argv[])
{
	cpu_set_t cpuset;

	CPU_ZERO(&cpuset);
	CPU_SET(2, &cpuset);

	auto rv = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
	if (rv < 0) {
		std::cerr << "affinity: errreur! " << std::strerror(errno);
		return 0;
	}

	unsigned int default_tx_freq(881000 * 1000), default_rx_freq(926000 * 1000);
	unsigned int grx = 20, gtx = 20;
	bool tx_flag = false;
	pthread_setname_np(pthread_self(), "main");
	convolve_init();
	convert_init();
	sigProcLibSetup();
	initvita();

	int status = 0;
	auto trx = new ms_trx();
	trx->do_auto_gain = true;

	std::vector<std::string> args(argv + 1, argv + argc);
	for (auto i = args.begin(); i != args.end(); ++i) {
		parsec(args, i, "-r", 1000, &default_rx_freq);
		parsec(args, i, "-t", 1000, &default_tx_freq);
		parsec(args, i, "-gr", 1, &grx);
		parsec(args, i, "-gt", 1, &gtx);
		parsec(args, i, "-tx", &tx_flag);
	}

	std::cerr << "usage: " << argv[0] << " <rxfreq in khz, i.e. 926000> [txfreq in khz, i.e. 881000] [TX]"
		  << std::endl
		  << "rx" << (argc == 1 ? " (default) " : " ") << default_rx_freq << "hz, tx " << default_tx_freq
		  << "hz" << std::endl
		  << "gain rx " << grx << " gain tx " << gtx << std::endl
		  << (tx_flag ? "##!!## RACH TX ACTIVE ##!!##" : "-- no rach tx --") << std::endl;

	status = trx->init_dev_and_streams();
	if (status < 0)
		return status;
	trx->tuneRx(default_rx_freq);
	trx->tuneTx(default_tx_freq);
	trx->setRxGain(grx);
	trx->setTxGain(gtx);

	if (status == 0) {
		// FIXME: hacks! needs exit flag for detached threads!

		std::thread(rcv_bursts_test, &trx->rxqueue, &trx->mTSC, trx->rxFullScale).detach();
		if (tx_flag)
			std::thread(tx_test, trx, &trx->ts_hitter_q, &trx->mTSC).detach();
		trx->start();
		do {
			sleep(1);
		} while (1);

		trx->stop_threads();
	}
	delete trx;

	return status;
}
#endif

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
#pragma unused(rcd)
		auto y = this;
#pragma unused(y)
		/* nothing to do here */
		return 0;
	};
}

void ms_trx::start()
{
	auto fn = get_rx_burst_handler_fn(rx_bh());
	rx_task = std::thread(fn);
	set_name_aff_sched(rx_task.native_handle(), "rxrun", 2, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 2);

	usleep(1000);
	auto fn2 = get_tx_burst_handler_fn(tx_bh());
	tx_task = std::thread(fn2);
	set_name_aff_sched(tx_task.native_handle(), "txrun", 2, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 1);
}

void ms_trx::set_upper_ready(bool is_ready)
{
	upper_is_ready = is_ready;
}

void ms_trx::stop_threads()
{
	std::cerr << "killing threads...\r\n" << std::endl;
	close_device();
	rx_task.join();
	tx_task.join();
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

	// in thory fn equal and tn+3 equal is also a problem...
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
	memset(buf2, 0, pad * sizeof(blade_sample_type));
	memcpy(&buf2[pad], buffer, len * sizeof(blade_sample_type));

	assert(target.FN() == check.FN());
	assert(target.TN() == check.TN());
	submit_burst_ts(buf2, len + pad, send_ts - pad);
#else
	submit_burst_ts(buffer, len, send_ts);
#endif
}
