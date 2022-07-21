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

#include <complex>
#include <cstring>
#include <functional>
#include <iostream>
#include <thread>

#include <Timeval.h>
#include <vector>

// #define MTX_LOG_ENABLED
#include <ipcif.h>

// typedef unsigned long long TIMESTAMP;
using blade_sample_type = std::complex<int16_t>;
const int SAMPLE_SCALE_FACTOR = 1;

struct uhd_buf_wrap {
	uint64_t ts;
	uint32_t num_samps;
	blade_sample_type *buf;
	auto actual_samples_per_buffer()
	{
		return num_samps;
	}
	long get_first_ts()
	{
		return ts; //md->time_spec.to_ticks(rxticks);
	}
	int readall(blade_sample_type *outaddr)
	{
		memcpy(outaddr, buf, num_samps * sizeof(blade_sample_type));
		return num_samps;
	}
	int read_n(blade_sample_type *outaddr, int start, int num)
	{
		// assert(start >= 0);
		auto to_read = std::min((int)num_samps - start, num);
		// assert(to_read >= 0);
		memcpy(outaddr, buf + start, to_read * sizeof(blade_sample_type));
		return to_read;
	}
};

using dev_buf_t = uhd_buf_wrap;
using bh_fn_t = std::function<int(dev_buf_t *)>;

template <typename T> struct ipc_hw {
	// uhd::usrp::multi_usrp::sptr dev;
	// uhd::rx_streamer::sptr rx_stream;
	// uhd::tx_streamer::sptr tx_stream;
	blade_sample_type *one_pkt_buf;
	std::vector<blade_sample_type *> pkt_ptrs;
	size_t rx_spp;
	double rxticks;
	const unsigned int rxFullScale, txFullScale;
	const int rxtxdelay;
	float rxgain, txgain;
	trxmsif m;

	virtual ~ipc_hw()
	{
		delete[] one_pkt_buf;
	}
	ipc_hw() : rxFullScale(32767), txFullScale(32767), rxtxdelay(-67)
	{
	}

	bool tuneTx(double freq, size_t chan = 0)
	{
		msleep(25);
		// dev->set_tx_freq(freq, chan);
		msleep(25);
		return true;
	};
	bool tuneRx(double freq, size_t chan = 0)
	{
		msleep(25);
		// dev->set_rx_freq(freq, chan);
		msleep(25);
		return true;
	};
	bool tuneRxOffset(double offset, size_t chan = 0)
	{
		return true;
	};

	double setRxGain(double dB, size_t chan = 0)
	{
		rxgain = dB;
		msleep(25);
		// dev->set_rx_gain(dB, chan);
		msleep(25);
		return dB;
	};
	double setTxGain(double dB, size_t chan = 0)
	{
		txgain = dB;
		msleep(25);
		// dev->set_tx_gain(dB, chan);
		msleep(25);
		return dB;
	};
	int setPowerAttenuation(int atten, size_t chan = 0)
	{
		return atten;
	};

	int init_device(bh_fn_t rxh, bh_fn_t txh)
	{
		// std::thread([] {
		// 	osmo_ctx_init("bernd");
		// 	osmo_select_init();
		// 	main_ipc();
		// 	while (true)
		// 		osmo_select_main(0);
		// }).detach();

		return m.connect() ? 0 : -1;
	}

	void *rx_cb(bh_fn_t burst_handler)
	{
		void *ret;
		static int to_skip = 0;

		blade_sample_type pbuf[508 * 2];

		uint64_t t;

		int len = 508 * 2;
		m.read_dl(508 * 2, &t, pbuf);
		// auto len = ipc_shm_read(ios_tx_to_device[0], (uint16_t *)&pbuf, 508 * 2, &t, 1);
		// if(len < 0) {
		// 	std::cerr << "fuck, rx fail!" << std::endl;
		// 	exit(0);
		// }

		// uhd::rx_metadata_t md;
		// auto num_rx_samps = rx_stream->recv(pkt_ptrs.front(), rx_spp, md, 3.0, true);

		// if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
		// 	std::cerr << boost::format("Timeout while streaming") << std::endl;
		// 	exit(0);
		// }
		// if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
		// 	std::cerr << boost::format("Got an overflow indication. Please consider the following:\n"
		// 				   "  Your write medium must sustain a rate of %fMB/s.\n"
		// 				   "  Dropped samples will not be written to the file.\n"
		// 				   "  Please modify this example for your purposes.\n"
		// 				   "  This message will not appear again.\n") %
		// 			     1.f;
		// 	exit(0);
		// 	;
		// }
		// if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
		// 	std::cerr << str(boost::format("Receiver error: %s") % md.strerror());
		// 	exit(0);
		// }

		dev_buf_t rcd = { t, static_cast<uint32_t>(len), pbuf };

		if (to_skip < 120) // prevents weird overflows on startup
			to_skip++;
		else {
			burst_handler(&rcd);
		}

		return ret;
	}

	auto get_rx_burst_handler_fn(bh_fn_t burst_handler)
	{
		auto fn = [this, burst_handler] {
			pthread_setname_np(pthread_self(), "rxrun");
			// wait_for_shm_open();
			// uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
			// stream_cmd.stream_now = true;
			// stream_cmd.time_spec = uhd::time_spec_t();
			// rx_stream->issue_stream_cmd(stream_cmd);

			while (1) {
				rx_cb(burst_handler);
			}
		};
		return fn;
	}
	auto get_tx_burst_handler_fn(bh_fn_t burst_handler)
	{
		auto fn = [] {
			// wait_for_shm_open();
			// dummy
		};
		return fn;
	}
	void submit_burst_ts(blade_sample_type *buffer, int len, uint64_t ts)
	{
		// uhd::tx_metadata_t m = {};
		// m.end_of_burst = true;
		// m.start_of_burst = true;
		// m.has_time_spec = true;
		// m.time_spec = m.time_spec.from_ticks(ts + rxtxdelay, rxticks); // uhd specific b210 delay!
		// std::vector<void *> ptrs(1, buffer);

		// tx_stream->send(ptrs, len, m);

		// uhd::async_metadata_t async_md;
		// bool tx_ack = false;
		// while (!tx_ack && tx_stream->recv_async_msg(async_md)) {
		// 	tx_ack = (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_BURST_ACK);
		// }
		// std::cout << (tx_ack ? "yay" : "nay") << " " << async_md.time_spec.to_ticks(rxticks) << std::endl;
	}

	void set_name_aff_sched(const char *name, int cpunum, int schedtype, int prio)
	{
		pthread_setname_np(pthread_self(), name);

		// cpu_set_t cpuset;

		// CPU_ZERO(&cpuset);
		// CPU_SET(cpunum, &cpuset);

		// auto rv = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
		// if (rv < 0) {
		// 	std::cerr << name << " affinity: errreur! " << std::strerror(errno);
		// 	return exit(0);
		// }

		// sched_param sch_params;
		// sch_params.sched_priority = prio;
		// rv = pthread_setschedparam(pthread_self(), schedtype, &sch_params);
		// if (rv < 0) {
		// 	std::cerr << name << " sched: errreur! " << std::strerror(errno);
		// 	return exit(0);
		// }
	}
	void signal_start()
	{
		m.signal_read_start();
	}
};