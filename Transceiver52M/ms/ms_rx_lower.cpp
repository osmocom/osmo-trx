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

#include "threadpool.h"

extern "C" {
#include "sch.h"
}

#ifdef LOG
#undef LOG
#endif

#if !defined(NODAMNLOG)
#define DBGLG(...) ms_trx::dummy_log()
#else
#define DBGLG(...) std::cerr
#endif

#if !defined(NODAMNLOG)
#define DBGLG2(...) ms_trx::dummy_log()
#else
#define DBGLG2(...) std::cerr
#endif

#define PRINT_Q_OVERFLOW

extern std::atomic<bool> g_exit_flag;

bool ms_trx::decode_sch(char *bits, bool update_global_clock)
{
	int fn;
	struct sch_info sch;
	ubit_t info[GSM_SCH_INFO_LEN];
	sbit_t data[GSM_SCH_CODED_LEN];

	memcpy(&data[0], &bits[3], 39);
	memcpy(&data[39], &bits[106], 39);

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
	float sum = normed_abs_sum(&brst.burst[0], ONE_TS_BURST_LEN);
	runmean = gain_check ? (runmean * (gain_check + 2) - 1 + sum) / (gain_check + 2) : sum;

	if (gain_check == avgburst_num - 1) {
		DBGLG2() << "\x1B[32m #RXG \033[0m" << cfg.rxgain << " " << runmean << " " << sum << std::endl;
		auto gainoffset = runmean < (rxFullScale / 4 ? 4 : 2);
		gainoffset = runmean < (rxFullScale / 2 ? 2 : 1);
		float newgain = runmean < rx_max_cutoff ? cfg.rxgain + gainoffset : cfg.rxgain - gainoffset;
		// FIXME: gian cutoff
		if (newgain != cfg.rxgain && newgain <= 60) {
			auto gain_fun = [this, newgain] { setRxGain(newgain); };
			worker_thread.add_task(gain_fun);
		}

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

	while (!g_exit_flag && upper_is_ready && !rxqueue.spsc_push(&brst))
		;

	if (!use_agc)
		maybe_update_gain(brst);

	return false;
}

static float sch_acq_buffer[SCH_LEN_SPS * 2];

bool ms_trx::handle_sch(bool is_first_sch_acq)
{
	auto current_gsm_time = timekeeper.gsmtime();
	const auto buf_len = is_first_sch_acq ? SCH_LEN_SPS : ONE_TS_BURST_LEN;
	const auto which_in_buffer = is_first_sch_acq ? first_sch_buf : burst_copy_buffer;
	memset((void *)&sch_acq_buffer[0], 0, sizeof(sch_acq_buffer));
	if (use_va) {
		const auto which_out_buffer = is_first_sch_acq ? sch_acq_buffer : &sch_acq_buffer[40 * 2];
		const auto ss = reinterpret_cast<std::complex<float> *>(which_out_buffer);
		std::complex<float> channel_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];
		int start;
		convert_and_scale(which_out_buffer, which_in_buffer, buf_len * 2, 1.f / float(rxFullScale));
		if (is_first_sch_acq) {
			float max_corr = 0;
			start = get_sch_buffer_chan_imp_resp(ss, &channel_imp_resp[0], buf_len, &max_corr);
		} else {
			start = get_sch_chan_imp_resp(ss, &channel_imp_resp[0]);
			start = start < 39 ? start : 39;
			start = start > -39 ? start : -39;
		}
		detect_burst_nb(&ss[start], &channel_imp_resp[0], 0, sch_demod_bits);

		auto sch_decode_success = decode_sch(sch_demod_bits, is_first_sch_acq);
#if 0 // useful to debug offset shifts
	auto burst = new signalVector(buf_len, 50);
	const auto corr_type = is_first_sch_acq ? sch_detect_type::SCH_DETECT_BUFFER : sch_detect_type::SCH_DETECT_FULL;
	struct estim_burst_params ebp;

	// scale like uhd, +-2k -> +-32k
	convert_and_scale(burst->begin(), which_in_buffer, buf_len * 2, SAMPLE_SCALE_FACTOR);

	auto rv = detectSCHBurst(*burst, 4, 4, corr_type, &ebp);

	int howmuchdelay = ebp.toa * 4;
	std::cerr << "ooffs: " << howmuchdelay << " " << std::endl;
	std::cerr << "voffs: " << start << " " << sch_decode_success << std::endl;
#endif
		if (sch_decode_success) {
			const auto ts_offset_symb = 4;
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
			DBGLG2() << "L SCH : \x1B[31m decode fail \033[0m @ toa:" << start << " "
				 << current_gsm_time.FN() << ":" << current_gsm_time.TN() << std::endl;
		}
	} else {
		const auto ts_offset_symb = 4;
		auto burst = new signalVector(buf_len, 50);
		const auto corr_type =
			is_first_sch_acq ? sch_detect_type::SCH_DETECT_BUFFER : sch_detect_type::SCH_DETECT_FULL;
		struct estim_burst_params ebp;

		// scale like uhd, +-2k -> +-32k
		convert_and_scale(burst->begin(), which_in_buffer, buf_len * 2, SAMPLE_SCALE_FACTOR);

		auto rv = detectSCHBurst(*burst, 4, 4, corr_type, &ebp);

		int howmuchdelay = ebp.toa * 4;

		if (!rv) {
			delete burst;
			DBGLG() << "SCH : \x1B[31m detect fail \033[0m NOOOOOOOOOOOOOOOOOO toa:" << ebp.toa << " "
				<< current_gsm_time.FN() << ":" << current_gsm_time.TN() << std::endl;
			return false;
		}

		SoftVector *bits;
		if (is_first_sch_acq) {
			// can't be legit with a buf size spanning _at least_ one SCH but delay that implies partial sch burst
			if (howmuchdelay < 0 || (buf_len - howmuchdelay) < ONE_TS_BURST_LEN) {
				delete burst;
				return false;
			}

			struct estim_burst_params ebp2;
			// auto sch_chunk = new signalVector(ONE_TS_BURST_LEN, 50);
			// auto sch_chunk_start = sch_chunk->begin();
			// memcpy(sch_chunk_start, sch_buf_f.data() + howmuchdelay, sizeof(std::complex<float>) * ONE_TS_BURST_LEN);

			auto delay = delayVector(burst, NULL, -howmuchdelay);

			scaleVector(*delay, (complex)1.0 / ebp.amp);

			auto rv2 = detectSCHBurst(*delay, 4, 4, sch_detect_type::SCH_DETECT_FULL, &ebp2);
			DBGLG() << "FIRST SCH : " << (rv2 ? "yes " : "   ") << "Timing offset     " << ebp2.toa
				<< " symbols" << std::endl;

			bits = demodAnyBurst(*delay, SCH, 4, &ebp2);
			delete delay;
		} else {
			bits = demodAnyBurst(*burst, SCH, 4, &ebp);
		}

		delete burst;

		// clamp to +-1.5 because +-127 softbits scaled by 64 after -0.5 can be at most +-1.5
		clamp_array(bits->begin(), 148, 1.5f);

		float_to_sbit(&bits->begin()[0], (signed char *)&sch_demod_bits[0], 62, 148);
		// float_to_sbit(&bits->begin()[106], &data[39], 62, 39);

		if (decode_sch((char *)sch_demod_bits, is_first_sch_acq)) {
			auto current_gsm_time_updated = timekeeper.gsmtime();
			if (is_first_sch_acq) {
				// update ts to first sample in sch buffer, to allow delay calc for current ts
				first_sch_ts_start = first_sch_buf_rcv_ts + howmuchdelay - (ts_offset_symb * 4);
			} else {
				// continuous sch tracking, only update if off too much
				auto diff = [](float x, float y) { return x > y ? x - y : y - x; };

				auto d = diff(ebp.toa, ts_offset_symb);
				if (abs(d) > 0.3) {
					if (ebp.toa < ts_offset_symb)
						ebp.toa = d;
					else
						ebp.toa = -d;
					temp_ts_corr_offset += ebp.toa * 4;

					DBGLG() << "offs: " << ebp.toa << " " << temp_ts_corr_offset << std::endl;
				}
			}

			auto a = gsm_sch_check_fn(current_gsm_time_updated.FN() - 1);
			auto b = gsm_sch_check_fn(current_gsm_time_updated.FN());
			auto c = gsm_sch_check_fn(current_gsm_time_updated.FN() + 1);
			DBGLG() << "L SCH : Timing offset     " << rv << " " << ebp.toa << " " << a << b << c << "fn "
				<< current_gsm_time_updated.FN() << ":" << current_gsm_time_updated.TN() << std::endl;

			delete bits;
			return true;
		} else {
			DBGLG2() << "L SCH : \x1B[31m decode fail \033[0m @ toa:" << ebp.toa << " "
				 << current_gsm_time.FN() << ":" << current_gsm_time.TN() << std::endl;
		}

		delete bits;
	}
	return false;
}

/*
accumulates a full big buffer consisting of 8*12 timeslots, then:
either
1) adjusts gain if necessary and starts over
2) searches and finds SCH and is done
*/
SCH_STATE ms_trx::search_for_sch(dev_buf_t *rcd)
{
	static unsigned int sch_pos = 0;
	auto to_copy = SCH_LEN_SPS - sch_pos;

	if (sch_thread_done)
		return SCH_STATE::FOUND;

	if (rcv_done)
		return SCH_STATE::SEARCHING;

	if (sch_pos == 0) // keep first ts for time delta calc
		first_sch_buf_rcv_ts = rcd->get_first_ts();

	if (to_copy) {
		auto spsmax = rcd->actual_samples_per_buffer();
		if (to_copy > (unsigned int)spsmax)
			sch_pos += rcd->readall(first_sch_buf + sch_pos);
		else
			sch_pos += rcd->read_n(first_sch_buf + sch_pos, 0, to_copy);
	} else { // (!to_copy)
		sch_pos = 0;
		rcv_done = true;
		auto sch_search_fun = [this] {
			const auto target_val = rxFullScale / 8;
			float sum = normed_abs_sum(first_sch_buf, SCH_LEN_SPS);

			//FIXME: arbitrary value, gain cutoff
			if (sum > target_val || cfg.rxgain >= 60) // enough ?
				sch_thread_done = this->handle_sch(true);
			else {
				std::cerr << "\x1B[32m #RXG \033[0m gain " << cfg.rxgain << " -> " << cfg.rxgain + 4
					  << " sample avg:" << sum << " target: >=" << target_val << std::endl;
				setRxGain(cfg.rxgain + 4);
			}

			if (!sch_thread_done)
				rcv_done = false; // retry!
		};
		worker_thread.add_task(sch_search_fun);
	}
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
