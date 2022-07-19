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
#include "syncthing.h"
#include "sigProcLib.h"
#include "signalVector.h"
#include "grgsm_vitac/grgsm_vitac.h"
extern "C" {
#include "sch.h"
}

#if !defined(SYNCTHINGONLY) || !defined(NODAMNLOG)
#define DBGLG(...) ms_trx::dummy_log()
#else
#define DBGLG(...) std::cerr
#endif

#if !defined(SYNCTHINGONLY)
#define DBGLG2(...) ms_trx::dummy_log()
#else
#define DBGLG2(...) std::cerr
#endif

__attribute__((xray_always_instrument)) __attribute__((noinline)) static bool decode_sch(float *bits,
											 bool update_global_clock)
{
	struct sch_info sch;
	ubit_t info[GSM_SCH_INFO_LEN];
	sbit_t data[GSM_SCH_CODED_LEN];

	float_to_sbit(&bits[3], &data[0], 62, 39);
	float_to_sbit(&bits[106], &data[39], 62, 39);

	if (!gsm_sch_decode(info, data)) {
		gsm_sch_parse(info, &sch);

		DBGLG() << "SCH : Decoded values" << std::endl;
		DBGLG() << "    BSIC: " << sch.bsic << std::endl;
		DBGLG() << "    TSC: " << (sch.bsic & 0x7) << std::endl;
		DBGLG() << "    T1  : " << sch.t1 << std::endl;
		DBGLG() << "    T2  : " << sch.t2 << std::endl;
		DBGLG() << "    T3p : " << sch.t3p << std::endl;
		DBGLG() << "    FN  : " << gsm_sch_to_fn(&sch) << std::endl;
		return true;
	}
	return false;
}

static void check_rcv_fn(GSM::Time t, bool first, unsigned int &lastfn, unsigned int &fnbm)
{
	if (first && t.TN() == 0) {
		lastfn = t.FN();
		fnbm = 1 << 0;
		first = false;
	}
	if (!first && t.FN() != lastfn) {
		if (fnbm != 255)
			std::cerr << "rx " << lastfn << ":" << fnbm << " " << __builtin_popcount(fnbm) << std::endl;
		lastfn = t.FN();
		fnbm = 1 << t.TN();
	}

	fnbm |= 1 << t.TN();
}

__attribute__((xray_always_instrument)) __attribute__((noinline)) static void
handle_it(one_burst &e, signalVector &burst, unsigned int tsc, int scale)
{
	memset(burst.begin(), 0, burst.size() * sizeof(std::complex<float>));
	auto is_sch = gsm_sch_check_fn(e.gsmts.FN()) && e.gsmts.TN() == 0;
	auto is_fcch = gsm_fcch_check_fn(e.gsmts.FN()) && e.gsmts.TN() == 0;

	// if (is_sch)
	// 	return;

	if (is_fcch)
		return;

	if (is_sch) {
		unsigned char outbin[148];
		convert_and_scale_default<float, int16_t>(burst.begin(), e.burst, ONE_TS_BURST_LEN * 2);
		std::stringstream dbgout;
#if 0
		{
			struct estim_burst_params ebp;
			auto rv2 = detectSCHBurst(burst, 4, 4, sch_detect_type::SCH_DETECT_FULL, &ebp);
			auto bits = demodAnyBurst(burst, SCH, 4, &ebp);
			// clamp_array(bits->begin(), 148, 1.5f);
			for (auto &i : *bits)
				i = (i > 0 ? 1 : -1);

			auto rv = decode_sch(bits->begin(), false);
			dbgout << "U DET@" << (rv2 ? "yes " : "   ") << "Timing offset     " << ebp.toa
			       << " symbols, DECODE: " << (rv ? "yes" : "---") << " ";

			delete bits;
		}
#endif
		{
			convert_and_scale<float, float>(burst.begin(), burst.begin(), ONE_TS_BURST_LEN * 2,
							1.f / float(scale));

			std::complex<float> channel_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];
			auto ss = reinterpret_cast<std::complex<float> *>(burst.begin());
			int d_c0_burst_start = get_sch_chan_imp_resp(ss, &channel_imp_resp[0]);
			detect_burst(ss, &channel_imp_resp[0], d_c0_burst_start, outbin);

			SoftVector bits;
			bits.resize(148);
			for (int i = 0; i < 148; i++) {
				bits[i] = (!outbin[i]) < 1 ? -1 : 1;
			}

			auto rv = decode_sch(bits.begin(), false);
			dbgout << "U SCH@"
			       << " " << e.gsmts.FN() << ":" << e.gsmts.TN() << " " << d_c0_burst_start
			       << " DECODE:" << (rv ? "yes" : "---") << std::endl;
		}

		DBGLG() << dbgout.str();
		return;
	}
#if 1
	convert_and_scale<float, int16_t>(burst.begin(), e.burst, ONE_TS_BURST_LEN * 2, 1.f / float(scale));
	// std::cerr << "@" << tsc << " " << e.gsmts.FN() << ":" << e.gsmts.TN() << " " << ebp.toa << " "
	// 	  << std::endl;

	unsigned char outbin[148];
	auto ss = reinterpret_cast<std::complex<float> *>(burst.begin());
	float ncmax, dcmax;
	std::complex<float> chan_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR], chan_imp_resp2[CHAN_IMP_RESP_LENGTH * d_OSR];
	auto normal_burst_start = get_norm_chan_imp_resp(ss, &chan_imp_resp[0], &ncmax, tsc);
	auto dummy_burst_start = get_norm_chan_imp_resp(ss, &chan_imp_resp2[0], &dcmax, TS_DUMMY);
	auto is_nb = ncmax > dcmax;

	DBGLG() << " U " << (is_nb ? "NB" : "DB") << "@ o nb: " << normal_burst_start << " o db: " << dummy_burst_start
		<< std::endl;

	if (is_nb)
		detect_burst(ss, &chan_imp_resp[0], normal_burst_start, outbin);
	else
		detect_burst(ss, &chan_imp_resp2[0], dummy_burst_start, outbin);
	;

	auto bits = SoftVector(148);
	for (int i = 0; i < 148; i++)
		(bits)[i] = outbin[i] < 1 ? -1 : 1;

#endif
}

__attribute__((xray_always_instrument)) __attribute__((noinline)) void rcv_bursts_test(rx_queue_t *q, unsigned int *tsc, int scale)
{
	static bool first = true;
	unsigned int lastfn = 0;
	unsigned int fnbm = 0;
	signalVector burst(ONE_TS_BURST_LEN, 100, 100);

	cpu_set_t cpuset;

	CPU_ZERO(&cpuset);
	CPU_SET(1, &cpuset);

	auto rv = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
	if (rv < 0) {
		std::cerr << "affinity: errreur! " << std::strerror(errno);
		exit(0);
	}

	int prio = sched_get_priority_max(SCHED_RR);
	struct sched_param param;
	param.sched_priority = prio;
	rv = sched_setscheduler(0, SCHED_RR, &param);
	if (rv < 0) {
		std::cerr << "scheduler: errreur! " << std::strerror(errno);
		exit(0);
	}

	while (1) {
		one_burst e;
		while (!q->spsc_pop(&e)) {
			q->spsc_prep_pop();
		}

		check_rcv_fn(e.gsmts, first, lastfn, fnbm);

		handle_it(e, burst, *tsc, scale);

		// rv = detectSCHBurst(*burst, 4, 4, sch_detect_type::SCH_DETECT_FULL, &ebp);
		// if (rv > 0)
		// 	std::cerr << "#" << e.gsmts.FN() << ":" << e.gsmts.TN() << " " << ebp.toa << std::endl;
		// sched_yield();
	}
}