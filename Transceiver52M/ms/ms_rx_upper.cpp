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
#include <signalVector.h>
#include <radioVector.h>
#include <radioInterface.h>
#include "grgsm_vitac/grgsm_vitac.h"
#include "ms_rx_upper.h"

extern "C" {
#include <osmocom/core/select.h>
#include "sch.h"
#include "convolve.h"
#include "convert.h"
#include "proto_trxd.h"

void __lsan_do_recoverable_leak_check();
}

namespace trxcon
{
extern "C" {
#include <trxcon/trx_if.h>
}
trx_instance *trxcon_instance; // local handle
static tx_queue_t txq;
} // namespace trxcon

#ifdef LOG
#undef LOG
#define LOG(...) upper_trx::dummy_log()
#endif

void upper_trx::start_threads()
{
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

	// atomic ensures data is not written to q until loop reads
	start_ms();

	set_name_aff_sched("upper_rx", 1, SCHED_FIFO, sched_get_priority_max(SCHED_RR) - 5);
	while (1) {
		// set_upper_ready(true);
		driveReceiveFIFO();
		pthread_testcancel();
		osmo_select_main(1);
	}

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

	auto ts = trxcon::trxcon_instance->ts_list[burst_time.TN()];
	if (ts == NULL || ts->mf_layout == NULL)
		return 0;

	convert_and_scale<float, int16_t>(ss, e.burst, ONE_TS_BURST_LEN * 2, 1.f / float(rxFullScale));

	pow = energyDetect(sv, 20 * rx_sps);
	if (pow < -1) {
		LOG(ALERT) << "Received empty burst";
		return NULL;
	}

	avg = sqrt(pow);
	{
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
}

void upper_trx::driveReceiveFIFO()
{
	int RSSI;
	int TOA; // in 1/256 of a symbol
	GSM::Time burstTime;

	if (!mOn)
		return;

	SoftVector *rxBurst = pullRadioVector(burstTime, RSSI, TOA);

	if (rxBurst) {
		trxd_from_trx response;
		response.ts = burstTime.TN();
		response.fn = htonl(burstTime.FN());
		response.rssi = RSSI;
		response.toa = htons(TOA);

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
		trxcon::trx_data_rx_handler(trxcon::trxcon_instance, (uint8_t *)&response);
	}
}

void upper_trx::driveTx()
{
	trxd_to_trx e;
	while (!trxcon::txq.spsc_pop(&e)) {
		trxcon::txq.spsc_prep_pop();
	}

	trxd_to_trx *burst = &e;

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
}

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

extern "C" {
void init_external_transceiver(struct trx_instance *trx, int argc, char **argv)
{
	trxcon::trxcon_instance = (trxcon::trx_instance *)trx;
	std::cout << "init?" << std::endl;
	trxc_main(argc, argv);
}

void close_external_transceiver(int argc, char **argv)
{
	std::cout << "Shutting down transceiver..." << std::endl;
}

void tx_external_transceiver(uint8_t *burst)
{
	trxcon::txq.spsc_push((trxd_to_trx *)burst);
}
}