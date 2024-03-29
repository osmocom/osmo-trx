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

#include <uhd/version.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/types/metadata.hpp>
#include <complex>
#include <cstring>
#include <iostream>
#include <thread>

#include <Timeval.h>
#include <vector>

using blade_sample_type = std::complex<int16_t>;
const int SAMPLE_SCALE_FACTOR = 1;

struct uhd_buf_wrap {
	double rxticks;
	size_t num_samps;
	uhd::rx_metadata_t *md;
	blade_sample_type *buf;
	auto actual_samples_per_buffer()
	{
		return num_samps;
	}
	long get_first_ts()
	{
		return md->time_spec.to_ticks(rxticks);
	}
	int readall(blade_sample_type *outaddr)
	{
		memcpy(outaddr, buf, num_samps * sizeof(blade_sample_type));
		return num_samps;
	}
	int read_n(blade_sample_type *outaddr, int start, int num)
	{
		assert(start >= 0);
		auto to_read = std::min((int)num_samps - start, num);
		assert(to_read >= 0);
		memcpy(outaddr, buf + start, to_read * sizeof(blade_sample_type));
		return to_read;
	}
};

using dev_buf_t = uhd_buf_wrap;
using bh_fn_t = std::function<int(dev_buf_t *)>;

template <typename T>
struct uhd_hw {
	uhd::usrp::multi_usrp::sptr dev;
	uhd::rx_streamer::sptr rx_stream;
	uhd::tx_streamer::sptr tx_stream;
	blade_sample_type *one_pkt_buf;
	std::vector<blade_sample_type *> pkt_ptrs;
	size_t rx_spp;
	double rxticks;
	const unsigned int rxFullScale, txFullScale;
	const int rxtxdelay;
	float rxgain, txgain;
	static std::atomic<bool> stop_lower_threads_flag;
	double rxfreq_cache, txfreq_cache;

	virtual ~uhd_hw()
	{
		delete[] one_pkt_buf;
	}
	uhd_hw() : rxFullScale(32767), txFullScale(32767 * 0.3), rxtxdelay(-67), rxfreq_cache(0), txfreq_cache(0)
	{
	}

	void close_device()
	{
	}

	bool tuneTx(double freq, size_t chan = 0)
	{
		if (txfreq_cache == freq)
			return true;
		msleep(25);
		dev->set_tx_freq(freq, chan);
		txfreq_cache = freq;
		msleep(25);
		return true;
	};
	bool tuneRx(double freq, size_t chan = 0)
	{
		if (rxfreq_cache == freq)
			return true;
		msleep(25);
		dev->set_rx_freq(freq, chan);
		rxfreq_cache = freq;
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
		dev->set_rx_gain(dB, chan);
		msleep(25);
		return dB;
	};
	double setTxGain(double dB, size_t chan = 0)
	{
		txgain = dB;
		msleep(25);
		dev->set_tx_gain(dB, chan);
		msleep(25);
		return dB;
	};
	int setPowerAttenuation(int atten, size_t chan = 0)
	{
		return atten;
	};

	int init_device(bh_fn_t rxh, bh_fn_t txh)
	{
		auto const lock_delay_ms = 500;
		auto clock_lock_attempts = 15; // x lock_delay_ms
		auto const mcr = 26e6;
		auto const rate = (1625e3 / 6) * 4;
		auto const ref = "external";
		auto const gain = 35;
		auto const freq = 931.4e6; // 936.8e6
		auto bw = 0.5e6;
		auto const channel = 0;
		// aligned to blade: 1020 samples per transfer
		std::string args = { "recv_frame_size=4092,send_frame_size=4092" };

		dev = uhd::usrp::multi_usrp::make(args);
		std::cout << "Using Device: " << dev->get_pp_string() << std::endl;
		dev->set_clock_source(ref);
		dev->set_master_clock_rate(mcr);
		dev->set_rx_rate(rate, channel);
		dev->set_tx_rate(rate, channel);
		uhd::tune_request_t tune_request(freq, 0);
		dev->set_rx_freq(tune_request, channel);
		dev->set_rx_gain(gain, channel);
		dev->set_tx_gain(60, channel);
		dev->set_rx_bandwidth(bw, channel);
		dev->set_tx_bandwidth(bw, channel);

		while (!(dev->get_rx_sensor("lo_locked", channel).to_bool() &&
			 dev->get_mboard_sensor("ref_locked").to_bool()) &&
		       clock_lock_attempts > 0) {
			std::cerr << "clock source lock attempts remaining: " << clock_lock_attempts << ".."
				  << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(lock_delay_ms));
			clock_lock_attempts--;
		}

		if (clock_lock_attempts <= 0) {
			std::cerr << "Error locking clock, gpsdo missing? quitting.." << std::endl;
			return -1;
		}

		uhd::stream_args_t stream_args("sc16", "sc16");
		rx_stream = dev->get_rx_stream(stream_args);
		uhd::stream_args_t stream_args2("sc16", "sc16");
		tx_stream = dev->get_tx_stream(stream_args2);

		rx_spp = rx_stream->get_max_num_samps();
		rxticks = dev->get_rx_rate();
		assert(rxticks == dev->get_tx_rate());
		one_pkt_buf = new blade_sample_type[rx_spp];
		pkt_ptrs = { 1, &one_pkt_buf[0] };
		return 0;
	}

	void actually_enable_streams()
	{
		// nop: stream cmd in handler
	}

	void *rx_cb(bh_fn_t burst_handler)
	{
		void *ret = nullptr;
		static int to_skip = 0;

		uhd::rx_metadata_t md;
		auto num_rx_samps = rx_stream->recv(pkt_ptrs.front(), rx_spp, md, 1.0, true);

		if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
			std::cerr << boost::format("Timeout while streaming") << std::endl;
			exit(0);
		}
		if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
			std::cerr << boost::format("Got an overflow indication\n") << std::endl;
			exit(0);
		}
		if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
			std::cerr << str(boost::format("Receiver error: %s") % md.strerror());
			exit(0);
		}

		dev_buf_t rcd = { rxticks, num_rx_samps, &md, &one_pkt_buf[0] };

		if (to_skip < 120) // prevents weird overflows on startup
			to_skip++;
		else {
			burst_handler(&rcd);
		}

		return ret;
	}

	auto get_rx_burst_handler_fn(bh_fn_t burst_handler)
	{
		// C cb -> ghetto closure capture, which is fine, the args never change.
		static auto rx_burst_cap_this = this;
		static auto rx_burst_cap_bh = burst_handler;
		auto fn = [](void *args) -> void * {
			pthread_setname_np(pthread_self(), "rxrun");

			uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
			stream_cmd.stream_now = true;
			stream_cmd.time_spec = uhd::time_spec_t();
			rx_burst_cap_this->rx_stream->issue_stream_cmd(stream_cmd);

			while (!rx_burst_cap_this->stop_lower_threads_flag) {
				rx_burst_cap_this->rx_cb(rx_burst_cap_bh);
			}
			return 0;
		};
		return fn;
	}
	auto get_tx_burst_handler_fn(bh_fn_t burst_handler)
	{
		auto fn = [](void *args) -> void * {
			// dummy
			return 0;
		};
		return fn;
	}
	void submit_burst_ts(blade_sample_type *buffer, int len, uint64_t ts)
	{
		uhd::tx_metadata_t m = {};
		m.end_of_burst = true;
		m.start_of_burst = true;
		m.has_time_spec = true;
		m.time_spec = m.time_spec.from_ticks(ts + rxtxdelay, rxticks); // uhd specific b210 delay!
		std::vector<void *> ptrs(1, buffer);

		tx_stream->send(ptrs, len, m, 1.0);
#ifdef DBGXX
		uhd::async_metadata_t async_md;
		bool tx_ack = false;
		while (!tx_ack && tx_stream->recv_async_msg(async_md)) {
			tx_ack = (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_BURST_ACK);
		}
		std::cout << (tx_ack ? "yay" : "nay") << " " << async_md.time_spec.to_ticks(rxticks) << std::endl;
#endif
	}
};