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
#include "syncthing.h"
#include "l1if.h"
#include <signalVector.h>
#include <radioVector.h>
#include <radioInterface.h>
#include "grgsm_vitac/grgsm_vitac.h"
#include "ms_state.h"
#include "ms_rx_upper.h"

extern "C" {
#include "sch.h"
#include "convolve.h"
#include "convert.h"
#include "proto_trxd.h"

void __lsan_do_recoverable_leak_check();
}

#ifdef LOG
#undef LOG
#define LOG(...) upper_trx::dummy_log()
#endif

void upper_trx::start_threads()
{
	thr_rx = std::thread([this] {
		set_name_aff_sched("upper_rx", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 5);
		while (1) {
			driveReceiveFIFO();
			pthread_testcancel();
		}
	});
	msleep(1);

	thr_control = std::thread([this] {
		set_name_aff_sched("upper_ctrl", 1, SCHED_RR, sched_get_priority_max(SCHED_RR));
		while (1) {
			driveControl();
			pthread_testcancel();
		}
	});
	msleep(1);
	thr_tx = std::thread([this] {
		set_name_aff_sched("upper_tx", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 1);

		while (1) {
			driveTx();
			pthread_testcancel();
		}
	});

	// std::thread([this] {
	// 	set_name_aff_sched("leakcheck", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 10);

	// 	while (1) {
	// 		std::this_thread::sleep_for(std::chrono::seconds{ 5 });
	// 		__lsan_do_recoverable_leak_check();
	// 	}
	// }).detach();
}

void upper_trx::start_ms()
{
	ms_trx::start();
}

/* Detect SCH synchronization sequence within a burst */
bool upper_trx::detectSCH(ms_TransceiverState *state, signalVector &burst, struct estim_burst_params *ebp)
{
	int shift;
	sch_detect_type full;
	float mag, threshold = 4.0;

	full = (state->mode == trx_mode::TRX_MODE_MS_TRACK) ? sch_detect_type::SCH_DETECT_NARROW :
							      sch_detect_type::SCH_DETECT_FULL;

	if (!detectSCHBurst(burst, threshold, rx_sps, full, ebp))
		return false;

	std::clog << "SCH : Timing offset     " << ebp->toa << " symbols" << std::endl;

	mag = fabsf(ebp->toa);
	if (mag < 1.0f)
		return true;

	shift = (int)(mag / 2.0f);
	if (!shift)
		shift++;

	shift = ebp->toa > 0 ? shift : -shift;
	std::clog << "SCH : shift ->     " << shift << " symbols" << std::endl;
	// mRadioInterface->applyOffset(shift);
	return false;
}

SoftVector *upper_trx::pullRadioVector(GSM::Time &wTime, int &RSSI, int &timingOffset) __attribute__((optnone))
{
	float pow, avg = 1.0;
	static SoftVector bits(148);
	static complex workbuf[40 + 625 + 40];
	static signalVector sv(workbuf, 40, 625);

	GSM::Time burst_time;
	auto ss = reinterpret_cast<std::complex<float> *>(&workbuf[40]);

	memset((void *)&workbuf[0], 0, sizeof(workbuf));
	// assert(sv.begin() == &workbuf[40]);

	one_burst e;
	unsigned char outbin[148];

	std::stringstream dbgout;

	while (!rxqueue.spsc_pop(&e)) {
		rxqueue.spsc_prep_pop();
	}

	burst_time = e.gsmts;
	wTime = burst_time;

	auto is_sch = (burst_time.TN() == 0 && gsm_sch_check_fn(burst_time.FN()));
	auto is_fcch = (burst_time.TN() == 0 && gsm_fcch_check_fn(burst_time.FN()));

	if (is_fcch) {
		// return trash
		// fprintf(stderr, "c %d\n",burst_time.FN());
		return &bits;
	}

	if (is_sch) {
		for (int i = 0; i < 148; i++)
			(bits)[i] = (!e.sch_bits[i]) < 1 ? -1 : 1;
		RSSI = 10;
		timingOffset = 0;
		// fprintf(stderr, "s %d\n", burst_time.FN());
		return &bits;
	}

	CorrType type = TSC;

	// tickle UL by returning null bursts if demod is skipped due to unused TS
	switch (mStates.mode) {
	case trx_mode::TRX_MODE_MS_TRACK:
		if (mStates.chanType[burst_time.TN()] == ChannelCombination::NONE_INACTIVE) {
			type = OFF;
			goto release;
		} else if (is_sch)
			type = SCH;
		else if (!is_fcch) // all ts0, but not fcch or sch..
			type = TSC;
		break;

	case trx_mode::TRX_MODE_OFF:
	default:
		goto release;
	}

	convert_and_scale<float, int16_t>(ss, e.burst, ONE_TS_BURST_LEN * 2, 1.f / float(rxFullScale));

	pow = energyDetect(sv, 20 * rx_sps);
	if (pow < -1) {
		LOG(ALERT) << "Received empty burst";
		goto release;
	}

	avg = sqrt(pow);

	if (type == SCH) {
		std::complex<float> chan_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];
		int d_c0_burst_start = get_sch_chan_imp_resp(ss, &chan_imp_resp[0]);
		detect_burst(ss, &chan_imp_resp[0], d_c0_burst_start, outbin);

		for (int i = 0; i < 148; i++)
			(bits)[i] = (!outbin[i]) < 1 ? -1 : 1;

		// auto rv = decode_sch(bits->begin(), false);
		// dbgout << "U SCH@"
		//        << " " << e.gsmts.FN() << ":" << e.gsmts.TN() << " " << d_c0_burst_start
		//        << " DECODE:" << (rv ? "yes" : "---") << std::endl;

		// std::cerr << dbgout.str();
	} else {
		float ncmax, dcmax;
		std::complex<float> chan_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];
		std::complex<float> chan_imp_resp2[CHAN_IMP_RESP_LENGTH * d_OSR];
		auto normal_burst_start = get_norm_chan_imp_resp(ss, &chan_imp_resp[0], &ncmax, mTSC);
		auto dummy_burst_start = get_norm_chan_imp_resp(ss, &chan_imp_resp2[0], &dcmax, TS_DUMMY);
		auto is_nb = ncmax > dcmax;

		// std::cerr << " U " << (is_nb ? "NB" : "DB") << "@ o nb: " << normal_burst_start
		// 	  << " o db: " << dummy_burst_start << std::endl;

		normal_burst_start = normal_burst_start < 39 ? normal_burst_start : 39;
		normal_burst_start = normal_burst_start > -39 ? normal_burst_start : -39;

		// fprintf(stderr, "%s %d\n", (is_nb ? "N":"D"), burst_time.FN());
		// if (is_nb)
		detect_burst(ss, &chan_imp_resp[0], normal_burst_start, outbin);
		// else
		// 	detect_burst(ss, &chan_imp_resp2[0], dummy_burst_start, outbin);

		for (int i = 0; i < 148; i++)
			(bits)[i] = (outbin[i]) < 1 ? -1 : 1;
	}

	RSSI = (int)floor(20.0 * log10(rxFullScale / avg));
	timingOffset = (int)round(0);

	return &bits;

release:
	return NULL;
}

void upper_trx::driveReceiveFIFO()
{
	int RSSI;
	int TOA; // in 1/256 of a symbol
	GSM::Time burstTime;
	SoftVector *rxBurst = pullRadioVector(burstTime, RSSI, TOA);

	if (!mOn)
		return;

	// _only_ return useless fcch trash to tickle trxcons tx path
	// auto is_fcch = [&burstTime]{ return burstTime.TN() == 0 && gsm_fcch_check_fn(burstTime.FN());};
	// if(!rxBurst && !is_fcch())
	// 	return;

	trxd_from_trx response;
	response.ts = burstTime.TN();
	response.fn = htonl(burstTime.FN());
	response.rssi = RSSI;
	response.toa = htons(TOA);
	if (rxBurst) {
		SoftVector::const_iterator burstItr = rxBurst->begin();
		if (burstTime.TN() == 0 && gsm_sch_check_fn(burstTime.FN())) {
			clamp_array(rxBurst->begin(), 148, 1.5f);
			for (unsigned int i = 0; i < gSlotLen; i++) {
				auto val = *burstItr++;
				auto vval = isnan(val) ? 0 : val;
				((int8_t *)response.symbols)[i] = round((vval - 0.5) * 64.0);
			}
		} else {
			// invert and fix to +-127 sbits
			for (int i = 0; i < 148; i++)
				((int8_t *)response.symbols)[i] = *burstItr++ > 0.0f ? -127 : 127;
		}
	}

#ifdef IPCIF
	push_d(response);
#else
	int rv = sendto(mDataSockets, &response, sizeof(trxd_from_trx), 0, (struct sockaddr *)&datadest,
			sizeof(struct sockaddr_in));
	if (rv < 0) {
		std::cerr << "fuck, send?" << std::endl;
		exit(0);
	}

#endif
}

void upper_trx::driveTx()
{
#ifdef IPCIF
	auto burst = pop_d();
	if (!burst) {
		// std::cerr << "wtf no tx burst?" << std::endl;
		// exit(0);
		continue;
	}
#else
	trxd_to_trx buffer;

	socklen_t addr_len = sizeof(datasrc);
	int rdln = recvfrom(mDataSockets, (void *)&buffer, sizeof(trxd_to_trx), 0, &datasrc, &addr_len);
	if (rdln < 0 && errno == EAGAIN) {
		std::cerr << "fuck, rcv?" << std::endl;
		exit(0);
	}
	if(rdln < sizeof(buffer)) // nope ind has len 6 or something like that
		return;


	trxd_to_trx *burst = &buffer;
#endif
	auto proper_fn = ntohl(burst->fn);
	// std::cerr << "got burst!" << proper_fn << ":" << burst->ts
	// 	  << " current: " << timekeeper.gsmtime().FN()
	// 	  << " dff: " << (int64_t)((int64_t)timekeeper.gsmtime().FN() - (int64_t)proper_fn)
	// 	  << std::endl;

	auto currTime = GSM::Time(proper_fn, burst->ts);
	int RSSI = (int)burst->txlev;

	static BitVector newBurst(gSlotLen);
	BitVector::iterator itr = newBurst.begin();
	auto *bufferItr = burst->symbols;
	while (itr < newBurst.end())
		*itr++ = *bufferItr++;

	auto txburst = modulateBurst(newBurst, 8 + (currTime.TN() % 4 == 0), 4);
	scaleVector(*txburst, txFullScale * 0.7 /* * pow(10, -RSSI / 10)*/);

	// float -> int16
	blade_sample_type burst_buf[txburst->size()];
	convert_and_scale<int16_t, float>(burst_buf, txburst->begin(), txburst->size() * 2, 1);

	// auto check = signalVector(txburst->size(), 40);
	// convert_and_scale<float, int16_t, 1>(check.begin(), burst_buf, txburst->size() * 2);
	// estim_burst_params ebp;
	// auto d = detectAnyBurst(check, 2, 4, 4, CorrType::RACH, 40, &ebp);
	// if(d)
	// 	std::cerr << "RACH D! " << ebp.toa << std::endl;
	// else
	// 	std::cerr << "RACH NOOOOOOOOOO D! " << ebp.toa << std::endl;

	// memory read --binary --outfile /tmp/mem.bin &burst_buf[0] --count 2500 --force

	submit_burst(burst_buf, txburst->size(), currTime);
	delete txburst;

#ifdef IPCIF
	free(burst);
#endif
}

// __attribute__((xray_always_instrument)) static void *rx_stream_callback(struct bladerf *dev,
// 									struct bladerf_stream *stream,
// 									struct bladerf_metadata *meta, void *samples,
// 									size_t num_samples, void *user_data)
// {
// 	struct ms_trx *trx = (struct ms_trx *)user_data;
// 	return trx->rx_cb(dev, stream, meta, samples, num_samples, user_data);
// }

// __attribute__((xray_always_instrument)) static void *tx_stream_callback(struct bladerf *dev,
// 									struct bladerf_stream *stream,
// 									struct bladerf_metadata *meta, void *samples,
// 									size_t num_samples, void *user_data)
// {
// 	struct ms_trx *trx = (struct ms_trx *)user_data;
// 	return BLADERF_STREAM_NO_DATA;
// }

int trxc_main(int argc, char *argv[])
{
	pthread_setname_np(pthread_self(), "main_trxc");

	convolve_init();
	convert_init();
	sigProcLibSetup();
	initvita();

	int status = 0;
	auto trx = new upper_trx();
	trx->do_auto_gain = true;

	status = trx->init_dev_and_streams(0, 0);
	trx->start_threads();

	return status;
}

extern "C" volatile bool gshutdown = false;
extern "C" void init_external_transceiver(int argc, char **argv)
{
	std::cout << "init?" << std::endl;
	trxc_main(argc, argv);
}

extern "C" void stop_trx()
{
	std::cout << "Shutting down transceiver..." << std::endl;
}
