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
#include "ms.h"
#include <signalVector.h>
#include <radioVector.h>
#include <radioInterface.h>
#include <grgsm_vitac/grgsm_vitac.h>

extern "C" {
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <fenv.h>

#include "sch.h"
#include "convolve.h"
#include "convert.h"

#ifdef LSANDEBUG
void __lsan_do_recoverable_leak_check();
#endif
}

#include "ms_upper.h"

namespace trxcon
{
extern "C" {
#include <osmocom/core/fsm.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/signal.h>
#include <osmocom/core/select.h>
#include <osmocom/gsm/gsm_utils.h>

#include <osmocom/core/logging.h>
#include <osmocom/bb/trxcon/logging.h>

#include <osmocom/bb/trxcon/trxcon.h>
#include <osmocom/bb/trxcon/trxcon_fsm.h>
#include <osmocom/bb/trxcon/phyif.h>
#include <osmocom/bb/trxcon/l1ctl_server.h>
}
struct trxcon_inst *g_trxcon;
// trx_instance *trxcon_instance; // local handle
struct internal_q_tx_buf {
	trxcon_phyif_burst_req r;
	uint8_t buf[148];
};
using tx_queue_t = spsc_cond<8 * 1, internal_q_tx_buf, true, false>;
using cmd_queue_t = spsc_cond<8 * 1, trxcon_phyif_cmd, true, false>;
using cmdr_queue_t = spsc_cond<8 * 1, trxcon_phyif_rsp, false, false>;
static tx_queue_t txq;
static cmd_queue_t cmdq_to_phy;
static cmdr_queue_t cmdq_from_phy;

extern bool trxc_l1ctl_init(void *tallctx);

} // namespace trxcon
extern "C" void trxc_log_init(void *tallctx);

#ifdef LOG
#undef LOG
#define LOG(...) upper_trx::dummy_log()
#endif

#define DBGLG(...) upper_trx::dummy_log()

std::atomic<bool> g_exit_flag;

void upper_trx::start_threads()
{
	thr_control = std::thread([this] {
		set_name_aff_sched("upper_ctrl", 1, SCHED_RR, sched_get_priority_max(SCHED_RR));
		while (!g_exit_flag) {
			driveControl();
		}
	});
	msleep(1);
	thr_tx = std::thread([this] {
		set_name_aff_sched("upper_tx", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 1);
		while (!g_exit_flag) {
			driveTx();
		}
	});

	// atomic ensures data is not written to q until loop reads
	start_lower_ms();

	set_name_aff_sched("upper_rx", 1, SCHED_FIFO, sched_get_priority_max(SCHED_RR) - 5);
	while (!g_exit_flag) {
		// set_upper_ready(true);
		driveReceiveFIFO();
		trxcon::osmo_select_main(1);

		trxcon::trxcon_phyif_rsp r;
		if (trxcon::cmdq_from_phy.spsc_pop(&r)) {
			DBGLG() << "HAVE RESP:" << r.type << std::endl;
			trxcon_phyif_handle_rsp(trxcon::g_trxcon, &r);
		}
	}

#ifdef LSANDEBUG
	std::thread([this] {
		set_name_aff_sched("leakcheck", 1, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 10);

		while (1) {
			std::this_thread::sleep_for(std::chrono::seconds{ 5 });
			__lsan_do_recoverable_leak_check();
		}
	}).detach();
#endif
}

void upper_trx::start_lower_ms()
{
	ms_trx::start();
}

// signalvector is owning despite claiming not to, but we can pretend, too..
static void static_free(void *wData){};
static void *static_alloc(size_t newSize)
{
	return 0;
};

bool upper_trx::pullRadioVector(GSM::Time &wTime, int &RSSI, int &timingOffset)
{
	float pow, avg = 1.0;
	const auto zero_pad_len = 40; // give the VA some runway for misaligned bursts
	const auto workbuf_size = zero_pad_len + ONE_TS_BURST_LEN + zero_pad_len;
	static complex workbuf[workbuf_size];

	static signalVector sv(workbuf, zero_pad_len, ONE_TS_BURST_LEN, static_alloc, static_free);
	one_burst e;
	auto ss = reinterpret_cast<std::complex<float> *>(&workbuf[zero_pad_len]);
	std::fill(workbuf, workbuf + workbuf_size, 0);
	// assert(sv.begin() == &workbuf[40]);

	while (!rxqueue.spsc_pop(&e)) {
		rxqueue.spsc_prep_pop();
	}

	wTime = e.gsmts;

	const auto is_sch = gsm_sch_check_ts(wTime.TN(), wTime.FN());
	const auto is_fcch = gsm_fcch_check_ts(wTime.TN(), wTime.FN());

	trxcon::trxcon_phyif_rtr_ind i = { static_cast<uint32_t>(wTime.FN()), static_cast<uint8_t>(wTime.TN()) };
	trxcon::trxcon_phyif_rtr_rsp r = {};
	trxcon_phyif_handle_rtr_ind(trxcon::g_trxcon, &i, &r);
	if (!(r.flags & TRXCON_PHYIF_RTR_F_ACTIVE))
		return false;

	if (is_fcch) {
		// return trash
		return true;
	}

	if (is_sch) {
		for (int i = 0; i < 148; i++)
			(demodded_softbits)[i] = (e.sch_bits[i]);
		RSSI = 10;
		timingOffset = 0;
		return true;
	}

	convert_and_scale(ss, e.burst, ONE_TS_BURST_LEN * 2, 1.f / float(rxFullScale));

	pow = energyDetect(sv, 20 * 4 /*sps*/);
	if (pow < -1) {
		LOG(ALERT) << "Received empty burst";
		return false;
	}

	avg = sqrt(pow);
	{
		float ncmax;
		std::complex<float> chan_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];
		auto normal_burst_start = get_norm_chan_imp_resp(ss, &chan_imp_resp[0], &ncmax, mTSC);
#ifdef DBGXX
		float dcmax;
		std::complex<float> chan_imp_resp2[CHAN_IMP_RESP_LENGTH * d_OSR];
		auto dummy_burst_start = get_norm_chan_imp_resp(ss, &chan_imp_resp2[0], &dcmax, TS_DUMMY);
		auto is_nb = ncmax > dcmax;
		// DBGLG() << " U " << (is_nb ? "NB" : "DB") << "@ o nb: " << normal_burst_start
		// 	  << " o db: " << dummy_burst_start << std::endl;
#endif
		normal_burst_start = normal_burst_start < 39 ? normal_burst_start : 39;
		normal_burst_start = normal_burst_start > -39 ? normal_burst_start : -39;
#ifdef DBGXX
		// fprintf(stderr, "%s %d\n", (is_nb ? "N":"D"), burst_time.FN());
		// if (is_nb)
#endif
		detect_burst(ss, &chan_imp_resp[0], normal_burst_start, demodded_softbits);
#ifdef DBGXX
		// else
		// 	detect_burst(ss, &chan_imp_resp2[0], dummy_burst_start, outbin);
#endif
	}
	RSSI = (int)floor(20.0 * log10(rxFullScale / avg));
	// FIXME: properly handle offset, sch/nb alignment diff? handled by lower anyway...
	timingOffset = (int)round(0);

	return true;
}

void upper_trx::driveReceiveFIFO()
{
	int RSSI;
	int TOA; // in 1/256 of a symbol
	GSM::Time burstTime;

	if (!mOn)
		return;

	if (pullRadioVector(burstTime, RSSI, TOA)) {
		trxcon::trxcon_phyif_burst_ind bi;
		bi.fn = burstTime.FN();
		bi.tn = burstTime.TN();
		bi.rssi = RSSI;
		bi.toa256 = TOA;
		bi.burst = (sbit_t *)demodded_softbits;
		bi.burst_len = sizeof(demodded_softbits);
		trxcon_phyif_handle_burst_ind(trxcon::g_trxcon, &bi);
	}

	struct trxcon::trxcon_phyif_rts_ind rts {
		static_cast<uint32_t>(burstTime.FN()), static_cast<uint8_t>(burstTime.TN())
	};
	trxcon_phyif_handle_rts_ind(trxcon::g_trxcon, &rts);
}

void upper_trx::driveTx()
{
	trxcon::internal_q_tx_buf e;
	static BitVector newBurst(sizeof(e.buf));
	while (!trxcon::txq.spsc_pop(&e)) {
		trxcon::txq.spsc_prep_pop();
	}

	// ensure our tx cb is tickled and can exit
	if (g_exit_flag) {
		blade_sample_type dummy[10] = {};
		submit_burst_ts(dummy, 10, 1);
		return;
	}

	trxcon::internal_q_tx_buf *burst = &e;

#ifdef TXDEBUG
	DBGLG() << "got burst!" << burst->r.fn << ":" << burst->ts << " current: " << timekeeper.gsmtime().FN()
		<< " dff: " << (int64_t)((int64_t)timekeeper.gsmtime().FN() - (int64_t)burst->r.fn) << std::endl;
#endif

	auto currTime = GSM::Time(burst->r.fn, burst->r.tn);
	int RSSI = (int)burst->r.pwr;

	BitVector::iterator itr = newBurst.begin();
	auto *bufferItr = burst->buf;
	while (itr < newBurst.end())
		*itr++ = *bufferItr++;

	auto txburst = modulateBurst(newBurst, 8 + (currTime.TN() % 4 == 0), 4);
	scaleVector(*txburst, txFullScale * pow(10, -RSSI / 10));

	// float -> int16
	blade_sample_type burst_buf[txburst->size()];
	convert_and_scale(burst_buf, txburst->begin(), txburst->size() * 2, 1);
#ifdef TXDEBUG
	auto check = signalVector(txburst->size(), 40);
	convert_and_scale(check.begin(), burst_buf, txburst->size() * 2, 1);
	estim_burst_params ebp;
	auto d = detectAnyBurst(check, 2, 4, 4, CorrType::RACH, 40, &ebp);
	if (d)
		DBGLG() << "RACH D! " << ebp.toa << std::endl;
	else
		DBGLG() << "RACH NOOOOOOOOOO D! " << ebp.toa << std::endl;

		// memory read --binary --outfile /tmp/mem.bin &burst_buf[0] --count 2500 --force
#endif
	submit_burst(burst_buf, txburst->size(), currTime);
	delete txburst;
}

#ifdef TXDEBUG
static const char *cmd2str(trxcon::trxcon_phyif_cmd_type c)
{
	switch (c) {
	case trxcon::TRXCON_PHYIF_CMDT_RESET:
		return "TRXCON_PHYIF_CMDT_RESET";
	case trxcon::TRXCON_PHYIF_CMDT_POWERON:
		return "TRXCON_PHYIF_CMDT_POWERON";
	case trxcon::TRXCON_PHYIF_CMDT_POWEROFF:
		return "TRXCON_PHYIF_CMDT_POWEROFF";
	case trxcon::TRXCON_PHYIF_CMDT_MEASURE:
		return "TRXCON_PHYIF_CMDT_MEASURE";
	case trxcon::TRXCON_PHYIF_CMDT_SETFREQ_H0:
		return "TRXCON_PHYIF_CMDT_SETFREQ_H0";
	case trxcon::TRXCON_PHYIF_CMDT_SETFREQ_H1:
		return "TRXCON_PHYIF_CMDT_SETFREQ_H1";
	case trxcon::TRXCON_PHYIF_CMDT_SETSLOT:
		return "TRXCON_PHYIF_CMDT_SETSLOT";
	case trxcon::TRXCON_PHYIF_CMDT_SETTA:
		return "TRXCON_PHYIF_CMDT_SETTA";
	default:
		return "UNKNOWN COMMAND!";
	}
}

static void print_cmd(trxcon::trxcon_phyif_cmd_type c)
{
	DBGLG() << cmd2str(c) << std::endl;
}
#endif

bool upper_trx::driveControl()
{
	trxcon::trxcon_phyif_rsp r;
	trxcon::trxcon_phyif_cmd cmd;
	while (!trxcon::cmdq_to_phy.spsc_pop(&cmd)) {
		trxcon::cmdq_to_phy.spsc_prep_pop();
	}

	if (g_exit_flag)
		return false;

#ifdef TXDEBUG
	print_cmd(cmd.type);
#endif

	switch (cmd.type) {
	case trxcon::TRXCON_PHYIF_CMDT_RESET:
		set_ta(0);
		break;
	case trxcon::TRXCON_PHYIF_CMDT_POWERON:

		if (!mOn) {
			set_upper_ready(true);
			mOn = true;
		}
		break;
	case trxcon::TRXCON_PHYIF_CMDT_POWEROFF:
		break;
	case trxcon::TRXCON_PHYIF_CMDT_MEASURE:
		r.type = trxcon::trxcon_phyif_cmd_type::TRXCON_PHYIF_CMDT_MEASURE;
		r.param.measure.band_arfcn = cmd.param.measure.band_arfcn;
		// FIXME: do we want to measure anything, considering the transceiver just syncs by.. syncing?
		r.param.measure.dbm = -80;
		tuneRx(trxcon::gsm_arfcn2freq10(cmd.param.measure.band_arfcn, 0) * 1000 * 100);
		tuneTx(trxcon::gsm_arfcn2freq10(cmd.param.measure.band_arfcn, 1) * 1000 * 100);
		trxcon::cmdq_from_phy.spsc_push(&r);
		break;
	case trxcon::TRXCON_PHYIF_CMDT_SETFREQ_H0:
		tuneRx(trxcon::gsm_arfcn2freq10(cmd.param.setfreq_h0.band_arfcn, 0) * 1000 * 100);
		tuneTx(trxcon::gsm_arfcn2freq10(cmd.param.setfreq_h0.band_arfcn, 1) * 1000 * 100);
		break;
	case trxcon::TRXCON_PHYIF_CMDT_SETFREQ_H1:
		break;
	case trxcon::TRXCON_PHYIF_CMDT_SETSLOT:
		break;
	case trxcon::TRXCON_PHYIF_CMDT_SETTA:
		set_ta(cmd.param.setta.ta);
		break;
	}
	return false;
}

// trxcon C call(back) if
extern "C" {
int trxcon_phyif_handle_burst_req(void *phyif, const struct trxcon::trxcon_phyif_burst_req *br)
{
	if (br->burst_len == 0) // dummy/nope
		return 0;
	OSMO_ASSERT(br->burst != 0);

	trxcon::internal_q_tx_buf b;
	b.r = *br;
	memcpy(b.buf, (void *)br->burst, br->burst_len);

	if (!g_exit_flag)
		trxcon::txq.spsc_push(&b);
	return 0;
}

int trxcon_phyif_handle_cmd(void *phyif, const struct trxcon::trxcon_phyif_cmd *cmd)
{
#ifdef TXDEBUG
	DBGLG() << "TOP C: " << cmd2str(cmd->type) << std::endl;
#endif
	if (!g_exit_flag)
		trxcon::cmdq_to_phy.spsc_push(cmd);
	// q for resp polling happens in main loop
	return 0;
}

void trxcon_phyif_close(void *phyif)
{
}

void trxcon_l1ctl_close(struct trxcon::trxcon_inst *trxcon)
{
	/* Avoid use-after-free: both *fi and *trxcon are children of
	 * the L2IF (L1CTL connection), so we need to re-parent *fi
	 * to NULL before calling l1ctl_client_conn_close(). */
	talloc_steal(NULL, trxcon->fi);
	trxcon::l1ctl_client_conn_close((struct trxcon::l1ctl_client *)trxcon->l2if);
}

int trxcon_l1ctl_send(struct trxcon::trxcon_inst *trxcon, struct trxcon::msgb *msg)
{
	struct trxcon::l1ctl_client *l1c = (struct trxcon::l1ctl_client *)trxcon->l2if;

	return trxcon::l1ctl_client_send(l1c, msg);
}
}

void sighandler(int sigset)
{
	// we might get a sigpipe in case the l1ctl ud socket disconnects because mobile quits
	if (sigset == SIGPIPE) {
		g_exit_flag = true;

		// we know the flag is atomic and it prevents the trxcon cb handlers from writing
		// to the queues, so submit some trash to unblock the threads & exit
		trxcon::trxcon_phyif_cmd cmd = {};
		trxcon::internal_q_tx_buf b = {};
		trxcon::txq.spsc_push(&b);
		trxcon::cmdq_to_phy.spsc_push(&cmd);

		return;
	}
}

int main(int argc, char *argv[])
{
	auto tall_trxcon_ctx = talloc_init("trxcon context");
	signal(SIGPIPE, sighandler);

	trxcon::msgb_talloc_ctx_init(tall_trxcon_ctx, 0);
	trxc_log_init(tall_trxcon_ctx);

	trxcon::g_trxcon = trxcon::trxcon_inst_alloc(tall_trxcon_ctx, 0, 0);
	trxcon::g_trxcon->gsmtap = nullptr;
	trxcon::g_trxcon->phyif = nullptr;
	trxcon::g_trxcon->phy_quirks.fbsb_extend_fns = 866; // 4 seconds, known to work.

	convolve_init();
	convert_init();
	sigProcLibSetup();
	initvita();

	int status = 0;
	auto trx = new upper_trx();
	trx->do_auto_gain = true;

	status = trx->init_dev_and_streams();
	if (status < 0) {
		std::cerr << "Error initializing hardware, quitting.." << std::endl;
		return -1;
	}
	trx->set_name_aff_sched("main", 3, SCHED_FIFO, sched_get_priority_max(SCHED_FIFO) - 5);

	if (!trxcon::trxc_l1ctl_init(tall_trxcon_ctx)) {
		std::cerr << "Error initializing l1ctl, quitting.." << std::endl;
		return -1;
	}

	trx->start_threads();
	trx->stop_threads();

	return status;
}
