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

#include "sigProcLib.h"
#include "signalVector.h"
#include <atomic>
#include <cassert>
#include <complex>
#include <iostream>
#include <future>

#include "ms.h"
#include "grgsm_vitac/grgsm_vitac.h"

extern "C" {
#include "sch.h"
}

#ifdef LOG
#undef LOG
#endif

#if !defined(SYNCTHINGONLY) //|| !defined(NODAMNLOG)
#define DBGLG(...) ms_trx::dummy_log()
#else
#define DBGLG(...) std::cerr
#endif

#if !defined(SYNCTHINGONLY) || !defined(NODAMNLOG)
#define DBGLG2(...) ms_trx::dummy_log()
#else
#define DBGLG2(...) std::cerr
#endif

#define PRINT_Q_OVERFLOW
bool ms_trx::decode_sch(float *bits, bool update_global_clock)
{
	int fn;
	struct sch_info sch;
	ubit_t info[GSM_SCH_INFO_LEN];
	sbit_t data[GSM_SCH_CODED_LEN];

	float_to_sbit(&bits[3], &data[0], 1, 39);
	float_to_sbit(&bits[106], &data[39], 1, 39);

	if (!gsm_sch_decode(info, data)) {
		gsm_sch_parse(info, &sch);

		if (update_global_clock) {
			DBGLG() << "SCH : Decoded values" << std::endl;
			DBGLG() << "    BSIC: " << sch.bsic << std::endl;
			DBGLG() << "    TSC: " << (sch.bsic & 0x7) << std::endl;
			DBGLG() << "    T1  : " << sch.t1 << std::endl;
			DBGLG() << "    T2  : " << sch.t2 << std::endl;
			DBGLG() << "    T3p : " << sch.t3p << std::endl;
			DBGLG() << "    FN  : " << gsm_sch_to_fn(&sch) << std::endl;
		}

		fn = gsm_sch_to_fn(&sch);
		if (fn < 0) { // how? wh?
			DBGLG() << "SCH : Failed to convert FN " << std::endl;
			return false;
		}

		if (update_global_clock) {
			mBSIC = sch.bsic;
			mTSC = sch.bsic & 0x7;
			timekeeper.set(fn, 0);
			// global_time_keeper.FN(fn);
			// global_time_keeper.TN(0);
		}
#ifdef SYNCTHINGONLY
		else {
			int t3 = sch.t3p * 10 + 1;
			if (t3 == 11) {
				// timeslot hitter attempt @ fn 21 in mf
				DBGLG2() << "sch @ " << t3 << std::endl;
				auto e = GSM::Time(fn, 0);
				e += 10;
				ts_hitter_q.spsc_push(&e);
			}
		}
#endif

		return true;
	}
	return false;
}

void ms_trx::maybe_update_gain(one_burst &brst)
{
	static_assert((sizeof(brst.burst) / sizeof(brst.burst[0])) == ONE_TS_BURST_LEN, "wtf, buffer size mismatch?");
	const int avgburst_num = 8 * 20; // ~ 50*4.5ms = 90ms?
	static_assert(avgburst_num * 577 > (50 * 1000), "can't update faster then blade wait time?");
	const unsigned int rx_max_cutoff = (rxFullScale * 2) / 3;
	static int gain_check = 0;
	static float runmean = 0;
	float sum = 0;
	for (auto i : brst.burst)
		sum += abs(i.real()) + abs(i.imag());
	sum /= ONE_TS_BURST_LEN * 2;

	runmean = gain_check ? (runmean * (gain_check + 2) - 1 + sum) / (gain_check + 2) : sum;

	if (gain_check == avgburst_num - 1) {
		DBGLG2() << "\x1B[32m #RXG \033[0m" << rxgain << " " << runmean << " " << sum << std::endl;
		auto gainoffset = runmean < (rxFullScale / 4 ? 4 : 2);
		gainoffset = runmean < (rxFullScale / 2 ? 2 : 1);
		float newgain = runmean < rx_max_cutoff ? rxgain + gainoffset : rxgain - gainoffset;
		// FIXME: gian cutoff
		if (newgain != rxgain && newgain <= 60)
			std::thread([this, newgain] { setRxGain(newgain); }).detach();
		runmean = 0;
	}
	gain_check = (gain_check + 1) % avgburst_num;
}

static char sch_demod_bits[148];

bool ms_trx::handle_sch_or_nb()
{
	one_burst brst;
	const auto current_gsm_time = timekeeper.gsmtime();
	const auto is_sch = gsm_sch_check_ts(current_gsm_time.TN(), current_gsm_time.FN());

	//either pass burst to upper layer for demod, OR pass demodded SCH to upper layer so we don't waste time processing it twice
	brst.gsmts = current_gsm_time;

	if (!is_sch) {
		memcpy(brst.burst, burst_copy_buffer, sizeof(blade_sample_type) * ONE_TS_BURST_LEN);
	} else {
		handle_sch(false);
		memcpy(brst.sch_bits, sch_demod_bits, sizeof(sch_demod_bits));
	}
#ifndef SYNCTHINGONLY
	if (upper_is_ready) { // this is blocking, so only submit if there is a reader - only if upper exists!
#endif
		while (!rxqueue.spsc_push(&brst))
			;
#ifndef SYNCTHINGONLY
	}
#endif

	if (do_auto_gain)
		maybe_update_gain(brst);

	return false;
}

static float sch_acq_buffer[SCH_LEN_SPS * 2];

bool ms_trx::handle_sch(bool is_first_sch_acq)
{
	auto current_gsm_time = timekeeper.gsmtime();
	const auto buf_len = is_first_sch_acq ? SCH_LEN_SPS : ONE_TS_BURST_LEN;
	const auto which_in_buffer = is_first_sch_acq ? first_sch_buf : burst_copy_buffer;
	const auto which_out_buffer = is_first_sch_acq ? sch_acq_buffer : &sch_acq_buffer[40 * 2];
	const auto ss = reinterpret_cast<std::complex<float> *>(which_out_buffer);
	std::complex<float> channel_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];

	int start;
	memset((void *)&sch_acq_buffer[0], 0, sizeof(sch_acq_buffer));
	if (is_first_sch_acq) {
		float max_corr = 0;
		convert_and_scale(which_out_buffer, which_in_buffer, buf_len * 2, 1.f / float(rxFullScale));
		start = get_sch_buffer_chan_imp_resp(ss, &channel_imp_resp[0], buf_len, &max_corr);
		detect_burst(&ss[start], &channel_imp_resp[0], 0, sch_demod_bits);
	} else {
		convert_and_scale(which_out_buffer, which_in_buffer, buf_len * 2, 1.f / float(rxFullScale));
		start = get_sch_chan_imp_resp(ss, &channel_imp_resp[0]);
		start = start < 39 ? start : 39;
		start = start > -39 ? start : -39;
		detect_burst(&ss[start], &channel_imp_resp[0], 0, sch_demod_bits);
	}

	SoftVector bitss(148);
	for (int i = 0; i < 148; i++) {
		bitss[i] = (sch_demod_bits[i]);
	}

	auto sch_decode_success = decode_sch(bitss.begin(), is_first_sch_acq);

	if (sch_decode_success) {
		const auto ts_offset_symb = 0;
		if (is_first_sch_acq) {
			// update ts to first sample in sch buffer, to allow delay calc for current ts
			first_sch_ts_start = first_sch_buf_rcv_ts + start - (ts_offset_symb * 4) - 1;
		} else if (abs(start) > 1) {
			// continuous sch tracking, only update if off too much
			temp_ts_corr_offset += -start;
			std::cerr << "offs: " << start << " " << temp_ts_corr_offset << std::endl;
		}

		return true;
	} else {
		DBGLG2() << "L SCH : \x1B[31m decode fail \033[0m @ toa:" << start << " " << current_gsm_time.FN()
			 << ":" << current_gsm_time.TN() << std::endl;
	}
	return false;
}

SCH_STATE ms_trx::search_for_sch(dev_buf_t *rcd)
{
	static unsigned int sch_pos = 0;
	if (sch_thread_done)
		return SCH_STATE::FOUND;

	if (rcv_done)
		return SCH_STATE::SEARCHING;

	auto to_copy = SCH_LEN_SPS - sch_pos;

	if (SCH_LEN_SPS == to_copy) // first time
		first_sch_buf_rcv_ts = rcd->get_first_ts();

	if (!to_copy) {
		sch_pos = 0;
		rcv_done = true;
		std::thread([this] {
			set_name_aff_sched("sch_search", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 5);

			auto ptr = reinterpret_cast<const int16_t *>(first_sch_buf);
			const auto target_val = rxFullScale / 8;
			float sum = 0;
			for (unsigned int i = 0; i < SCH_LEN_SPS * 2; i++)
				sum += std::abs(ptr[i]);
			sum /= SCH_LEN_SPS * 2;

			//FIXME: arbitrary value, gain cutoff
			if (sum > target_val || rxgain >= 60) // enough ?
				sch_thread_done = this->handle_sch(true);
			else {
				std::cerr << "\x1B[32m #RXG \033[0m gain " << rxgain << " -> " << rxgain + 4
					  << " sample avg:" << sum << " target: >=" << target_val << std::endl;
				setRxGain(rxgain + 4);
			}

			if (!sch_thread_done)
				rcv_done = false; // retry!
			return (bool)sch_thread_done;
		}).detach();
	}

	auto spsmax = rcd->actual_samples_per_buffer();
	if (to_copy > (unsigned int)spsmax)
		sch_pos += rcd->readall(first_sch_buf + sch_pos);
	else
		sch_pos += rcd->read_n(first_sch_buf + sch_pos, 0, to_copy);

	return SCH_STATE::SEARCHING;
}

void ms_trx::grab_bursts(dev_buf_t *rcd)
{
	// partial burst samples read from the last buffer
	static int partial_rdofs = 0;
	static bool first_call = true;
	int to_skip = 0;

	// round up to next burst by calculating the time between sch detection and now
	if (first_call) {
		const auto next_burst_start = rcd->get_first_ts() - first_sch_ts_start;
		const auto fullts = next_burst_start / ONE_TS_BURST_LEN;
		const auto fracts = next_burst_start % ONE_TS_BURST_LEN;
		to_skip = ONE_TS_BURST_LEN - fracts;

		for (unsigned int i = 0; i < fullts; i++)
			timekeeper.inc_and_update(first_sch_ts_start + i * ONE_TS_BURST_LEN);

		if (fracts)
			timekeeper.inc_both();
		// timekeeper.inc_and_update(first_sch_ts_start + 1 * ONE_TS_BURST_LEN);

		timekeeper.dec_by_one(); // oops, off by one?

		timekeeper.set(timekeeper.gsmtime(), rcd->get_first_ts() - ONE_TS_BURST_LEN + to_skip);

		DBGLG() << "this ts: " << rcd->get_first_ts() << " diff full TN: " << fullts << " frac TN: " << fracts
			<< " GSM now: " << timekeeper.gsmtime().FN() << ":" << timekeeper.gsmtime().TN() << " is sch? "
			<< gsm_sch_check_fn(timekeeper.gsmtime().FN()) << std::endl;
		first_call = false;
	}

	if (partial_rdofs) {
		auto first_remaining = ONE_TS_BURST_LEN - partial_rdofs;
		auto rd = rcd->read_n(burst_copy_buffer + partial_rdofs, 0, first_remaining);
		if (rd != (int)first_remaining) {
			partial_rdofs += rd;
			return;
		}

		timekeeper.inc_and_update_safe(rcd->get_first_ts() - partial_rdofs);
		handle_sch_or_nb();
		to_skip = first_remaining;
	}

	// apply sample rate slippage compensation
	to_skip -= temp_ts_corr_offset;

	// FIXME: happens rarely, read_n start -1 blows up
	// this is fine: will just be corrected one buffer later
	if (to_skip < 0)
		to_skip = 0;
	else
		temp_ts_corr_offset = 0;

	const auto left_after_burst = rcd->actual_samples_per_buffer() - to_skip;

	const int full = left_after_burst / ONE_TS_BURST_LEN;
	const int frac = left_after_burst % ONE_TS_BURST_LEN;

	for (int i = 0; i < full; i++) {
		rcd->read_n(burst_copy_buffer, to_skip + i * ONE_TS_BURST_LEN, ONE_TS_BURST_LEN);
		timekeeper.inc_and_update_safe(rcd->get_first_ts() + to_skip + i * ONE_TS_BURST_LEN);
		handle_sch_or_nb();
	}

	if (frac)
		rcd->read_n(burst_copy_buffer, to_skip + full * ONE_TS_BURST_LEN, frac);
	partial_rdofs = frac;
}
