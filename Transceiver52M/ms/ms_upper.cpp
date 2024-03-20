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

#include <csignal>
#include "sigProcLib.h"
#include "ms.h"
#include <signalVector.h>
#include <radioVector.h>
#include <radioInterface.h>
#include <grgsm_vitac/grgsm_vitac.h>

// #define TXDEBUG

extern "C" {

#include "sch.h"
#include "convolve.h"
#include "convert.h"

#include <osmocom/core/application.h>
#include <osmocom/gsm/gsm_utils.h>

#include <osmocom/bb/trxcon/trxcon.h>
#include <osmocom/bb/trxcon/trxcon_fsm.h>
#include <osmocom/bb/trxcon/l1ctl_server.h>

extern void trxc_log_init(void *tallctx);
#ifdef LSANDEBUG
void __lsan_do_recoverable_leak_check();
#endif
}

#include "ms_trxcon_if.h"
#include "ms_upper.h"
#include "threadsched.h"

extern bool trxc_l1ctl_init(void *tallctx);
struct trxcon_inst *g_trxcon;
tx_queue_t txq;
cmd_queue_t cmdq_to_phy;
cmdr_queue_t cmdq_from_phy;

#ifdef LOG
#undef LOG
#define LOG(...) upper_trx::dummy_log()
#endif

#define DBGLG(...) upper_trx::dummy_log()

std::atomic<bool> g_exit_flag;

void upper_trx::stop_upper_threads()
{
	g_exit_flag = true;

	pthread_join(thr_control, NULL);
	pthread_join(thr_tx, NULL);
}

void upper_trx::start_threads()
{
	DBGLG(...) << "spawning threads.." << std::endl;

	thr_control = spawn_worker_thread(
		sched_params::thread_names::U_CTL,
		[](void *args) -> void * {
			upper_trx *t = reinterpret_cast<upper_trx *>(args);
#ifdef TXDEBUG
			struct sched_param param;
			int policy;
			pthread_getschedparam(pthread_self(), &policy, &param);
			printf("ID: %lu, CPU: %d policy = %d priority = %d\n", pthread_self(), sched_getcpu(), policy,
			       param.sched_priority);
#endif
			std::cerr << "started U control!" << std::endl;
			while (!g_exit_flag) {
				t->driveControl();
			}
			std::cerr << "exit U control!" << std::endl;

			return 0;
		},
		this);
	thr_tx = spawn_worker_thread(
		sched_params::thread_names::U_TX,
		[](void *args) -> void * {
			upper_trx *t = reinterpret_cast<upper_trx *>(args);
#ifdef TXDEBUG
			struct sched_param param;
			int policy;
			pthread_getschedparam(pthread_self(), &policy, &param);
			printf("ID: %lu, CPU: %d policy = %d priority = %d\n", pthread_self(), sched_getcpu(), policy,
			       param.sched_priority);
#endif
			std::cerr << "started U tx!" << std::endl;
			while (!g_exit_flag) {
				t->driveTx();
			}
			std::cerr << "exit U tx!" << std::endl;

			return 0;
		},
		this);

#ifdef LSANDEBUG
	std::thread([this] {
		set_name_aff_sched(sched_params::thread_names::LEAKCHECK);

		while (1) {
			std::this_thread::sleep_for(std::chrono::seconds{ 5 });
			__lsan_do_recoverable_leak_check();
		}
	}).detach();
#endif
}

void upper_trx::main_loop()
{
	set_name_aff_sched(sched_params::thread_names::U_RX);
	set_upper_ready(true);
	while (!g_exit_flag) {
		driveReceiveFIFO();
		osmo_select_main(1);

		trxcon_phyif_rsp r;
		if (cmdq_from_phy.spsc_pop(&r)) {
			DBGLG() << "HAVE RESP:" << r.type << std::endl;
			trxcon_phyif_handle_rsp(g_trxcon, &r);
		}
	}
	set_upper_ready(false);
	std::cerr << "exit U rx!" << std::endl;
	mOn = false;
}

// signalvector is owning despite claiming not to, but we can pretend, too..
static void static_free(void *wData){};
static void *static_alloc(size_t newSize)
{
	return 0;
};

bool upper_trx::pullRadioVector(GSM::Time &wTime, int &RSSI, int &timingOffset)
{
	// float pow, avg = 1.0;
	const auto zero_pad_len = 40; // give the VA some runway for misaligned bursts
	const auto workbuf_size = zero_pad_len + ONE_TS_BURST_LEN + zero_pad_len;
	static complex workbuf[workbuf_size];
	static int32_t meas_p, meas_rssi;

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

	trxcon_phyif_rtr_ind i = { static_cast<uint32_t>(wTime.FN()), static_cast<uint8_t>(wTime.TN()) };
	trxcon_phyif_rtr_rsp r = {};
	trxcon_phyif_handle_rtr_ind(g_trxcon, &i, &r);
	if (!(r.flags & TRXCON_PHYIF_RTR_F_ACTIVE)) {
		bladerf_get_rfic_rssi(dev, 0, &meas_p, &meas_rssi);
		// std::cerr << "G : \x1B[31m rx fail \033[0m @:" << meas_rssi << std::endl;
		return false;
	}

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

	if (use_va) {
		convert_and_scale(ss, e.burst, ONE_TS_BURST_LEN * 2, 1.f / float(rxFullScale));

		// pow = energyDetect(sv, 20 * 4 /*sps*/);
		// if (pow < -1) {
		// 	LOG(ALERT) << "Received empty burst";
		// 	return false;
		// }

		// avg = sqrt(pow);
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
			detect_burst_nb(ss, &chan_imp_resp[0], normal_burst_start, demodded_softbits);
#ifdef DBGXX
			// else
			// 	detect_burst(ss, &chan_imp_resp2[0], dummy_burst_start, outbin);
#endif
		}
	} else {
		// lower layer sch detection offset, easy to verify by just printing the detected value using both the va+sigproc code.
		convert_and_scale(ss + 16, e.burst, ONE_TS_BURST_LEN * 2, 15);

		// pow = energyDetect(sv, 20 * 4 /*sps*/);
		// if (pow < -1) {
		// 	LOG(ALERT) << "Received empty burst";
		// 	return false;
		// }

		// avg = sqrt(pow);

		/* Detect normal or RACH bursts */
		CorrType type = CorrType::TSC;
		struct estim_burst_params ebp;
		auto rc = detectAnyBurst(sv, mTSC, 3, 4, type, 48, &ebp);
		if (rc > 0) {
			type = (CorrType)rc;
		}

		if (rc < 0) {
			std::cerr << "UR : \x1B[31m rx fail \033[0m @ toa:" << ebp.toa << " " << e.gsmts.FN() << ":"
				  << e.gsmts.TN() << std::endl;
			return false;
		}
		SoftVector *bits = demodAnyBurst(sv, type, 4, &ebp);

		SoftVector::const_iterator burstItr = bits->begin();
		// invert and fix to +-127 sbits
		for (int ii = 0; ii < 148; ii++) {
			demodded_softbits[ii] = *burstItr++ > 0.0f ? -127 : 127;
		}
		delete bits;
	}
	RSSI = meas_rssi; // (int)floor(20.0 * log10(rxFullScale / avg));
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
		trxcon_phyif_burst_ind bi;
		bi.fn = burstTime.FN();
		bi.tn = burstTime.TN();
		bi.rssi = RSSI;
		bi.toa256 = TOA;
		bi.burst = (sbit_t *)demodded_softbits;
		bi.burst_len = sizeof(demodded_softbits);
		trxcon_phyif_handle_burst_ind(g_trxcon, &bi);
	}

	burstTime.incTN(2);
	struct trxcon_phyif_rts_ind rts {
		static_cast<uint32_t>(burstTime.FN()), static_cast<uint8_t>(burstTime.TN())
	};
	trxcon_phyif_handle_rts_ind(g_trxcon, &rts);
}

void upper_trx::driveTx()
{
	internal_q_tx_buf e;
	static BitVector newBurst(sizeof(e.buf));
	while (!txq.spsc_pop(&e)) {
		txq.spsc_prep_pop();
	}

	// ensure our tx cb is tickled and can exit
	if (g_exit_flag) {
		submit_burst_ts(0, 1337, 1);
		return;
	}

	internal_q_tx_buf *burst = &e;

#ifdef TXDEBUG2
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
#ifdef TXDEBUG2
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
static const char *cmd2str(trxcon_phyif_cmd_type c)
{
	switch (c) {
	case TRXCON_PHYIF_CMDT_RESET:
		return "TRXCON_PHYIF_CMDT_RESET";
	case TRXCON_PHYIF_CMDT_POWERON:
		return "TRXCON_PHYIF_CMDT_POWERON";
	case TRXCON_PHYIF_CMDT_POWEROFF:
		return "TRXCON_PHYIF_CMDT_POWEROFF";
	case TRXCON_PHYIF_CMDT_MEASURE:
		return "TRXCON_PHYIF_CMDT_MEASURE";
	case TRXCON_PHYIF_CMDT_SETFREQ_H0:
		return "TRXCON_PHYIF_CMDT_SETFREQ_H0";
	case TRXCON_PHYIF_CMDT_SETFREQ_H1:
		return "TRXCON_PHYIF_CMDT_SETFREQ_H1";
	case TRXCON_PHYIF_CMDT_SETSLOT:
		return "TRXCON_PHYIF_CMDT_SETSLOT";
	case TRXCON_PHYIF_CMDT_SETTA:
		return "TRXCON_PHYIF_CMDT_SETTA";
	default:
		return "UNKNOWN COMMAND!";
	}
}

static void print_cmd(trxcon_phyif_cmd_type c)
{
	DBGLG() << "handling " << cmd2str(c) << std::endl;
}
#endif

bool upper_trx::driveControl()
{
	trxcon_phyif_rsp r;
	trxcon_phyif_cmd cmd;
	while (!cmdq_to_phy.spsc_pop(&cmd)) {
		cmdq_to_phy.spsc_prep_pop();
		if (g_exit_flag)
			return false;
	}

	if (g_exit_flag)
		return false;

#ifdef TXDEBUG
	print_cmd(cmd.type);
#endif

	switch (cmd.type) {
	case TRXCON_PHYIF_CMDT_RESET:
		set_ta(0);
		break;
	case TRXCON_PHYIF_CMDT_POWERON:
		if (!mOn) {
			mOn = true;
			start_lower_ms();
		}
		break;
	case TRXCON_PHYIF_CMDT_POWEROFF:
		break;
	case TRXCON_PHYIF_CMDT_MEASURE:
		r.type = trxcon_phyif_cmd_type::TRXCON_PHYIF_CMDT_MEASURE;
		r.param.measure.band_arfcn = cmd.param.measure.band_arfcn;
		// FIXME: do we want to measure anything, considering the transceiver just syncs by.. syncing?
		r.param.measure.dbm = -80;
		// tuneRx(gsm_arfcn2freq10(cmd.param.measure.band_arfcn, 0) * 1000 * 100);
		// tuneTx(gsm_arfcn2freq10(cmd.param.measure.band_arfcn, 1) * 1000 * 100);
		cmdq_from_phy.spsc_push(&r);
		break;
	case TRXCON_PHYIF_CMDT_SETFREQ_H0:
		// tuneRx(gsm_arfcn2freq10(cmd.param.setfreq_h0.band_arfcn, 0) * 1000 * 100);
		// tuneTx(gsm_arfcn2freq10(cmd.param.setfreq_h0.band_arfcn, 1) * 1000 * 100);
		break;
	case TRXCON_PHYIF_CMDT_SETFREQ_H1:
		break;
	case TRXCON_PHYIF_CMDT_SETSLOT:
		break;
	case TRXCON_PHYIF_CMDT_SETTA:
		set_ta(cmd.param.setta.ta);
		break;
	}
	return false;
}

void sighandler(int sigset)
{
	// we might get a sigpipe in case the l1ctl ud socket disconnects because mobile quits
	if (sigset == SIGPIPE || sigset == SIGINT) {
		g_exit_flag = true;

		// we know the flag is atomic and it prevents the trxcon cb handlers from writing
		// to the queues, so submit some trash to unblock the threads & exit
		trxcon_phyif_cmd cmd = {};
		internal_q_tx_buf b = {};
		txq.spsc_push(&b);
		cmdq_to_phy.spsc_push(&cmd);
		msleep(200);

		return;
	}
}

extern "C" {
#include <osmocom/vty/command.h>
#include <osmocom/vty/logging.h>
#include "mssdr_vty.h"
}

int main(int argc, char *argv[])
{
	auto tall_trxcon_ctx = talloc_init("trxcon context");
	signal(SIGPIPE, sighandler);
	signal(SIGINT, sighandler);

	msgb_talloc_ctx_init(tall_trxcon_ctx, 0);
	trxc_log_init(tall_trxcon_ctx);

	/* Configure pretty logging */
	log_set_print_extended_timestamp(osmo_stderr_target, 1);
	log_set_print_category_hex(osmo_stderr_target, 0);
	log_set_print_category(osmo_stderr_target, 1);
	log_set_print_level(osmo_stderr_target, 1);

	log_set_print_filename2(osmo_stderr_target, LOG_FILENAME_BASENAME);
	log_set_print_filename_pos(osmo_stderr_target, LOG_FILENAME_POS_LINE_END);

	osmo_fsm_log_timeouts(true);

	auto g_mssdr_ctx = vty_mssdr_ctx_alloc(tall_trxcon_ctx);
	vty_init(&g_mssdr_vty_info);
	logging_vty_add_cmds();
	mssdr_vty_init(g_mssdr_ctx);

	const char *home_dir = getenv("HOME");
	if (!home_dir)
		home_dir = "~";
	auto config_file = talloc_asprintf(tall_trxcon_ctx, "%s/%s", home_dir, ".osmocom/bb/mssdr.cfg");

	int rc = vty_read_config_file(config_file, NULL);
	if (rc < 0) {
		fprintf(stderr, "Failed to parse config file: '%s'\n", config_file);
		exit(2);
	}

	g_trxcon = trxcon_inst_alloc(tall_trxcon_ctx, 0);
	g_trxcon->gsmtap = nullptr;
	g_trxcon->phyif = nullptr;
	g_trxcon->phy_quirks.fbsb_extend_fns = 866; // 4 seconds, known to work.

	convolve_init();
	convert_init();
	sigProcLibSetup();
	initvita();

	int status = 0;
	auto trx = new upper_trx(&g_mssdr_ctx->cfg);

	status = trx->init_dev_and_streams();
	if (status < 0) {
		std::cerr << "Error initializing hardware, quitting.." << std::endl;
		return -1;
	}
	set_name_aff_sched(sched_params::thread_names::MAIN);

	if (!trxc_l1ctl_init(tall_trxcon_ctx)) {
		std::cerr << "Error initializing l1ctl, quitting.." << std::endl;
		return -1;
	}

	// blocking, will return when global exit is requested
	trx->start_threads();
	trx->main_loop();
	trx->stop_threads();
	trx->stop_upper_threads();

	return status;
}
