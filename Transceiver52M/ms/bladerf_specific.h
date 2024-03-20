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

#include "itrq.h"
#include <atomic>
#include <complex>
#include <cstdint>
#include <functional>
#include <iostream>
#include <cassert>
#include <cstring>

#include <libbladeRF.h>
#include <Timeval.h>
#include <unistd.h>
extern "C" {
#include "mssdr_vty.h"
}

const size_t BLADE_BUFFER_SIZE = 1024 * 1;
const size_t BLADE_NUM_BUFFERS = 32 * 1;
const size_t NUM_TRANSFERS = 16 * 2;
const int SAMPLE_SCALE_FACTOR = 15; // actually 16 but sigproc complains about clipping..

// see https://en.cppreference.com/w/cpp/language/parameter_pack  "Brace-enclosed initializers" example
template <typename Arg, typename... Args>
void expand_args(std::ostream &out, Arg &&arg, Args &&...args)
{
	out << '(' << std::forward<Arg>(arg);
	(void)(int[]){ 0, (void((out << "," << std::forward<Args>(args))), 0)... };
	out << ')' << std::endl;
}

template <class R, class... Args>
using RvalFunc = R (*)(Args...);

template <class R, class... Args>
R exec_and_check(RvalFunc<R, Args...> func, const char *fname, const char *finame, const char *funcname, int line,
		 Args... args)
{
	R rval = func(std::forward<Args>(args)...);
	if (rval != 0) {
		std::cerr << ((rval >= 0) ? "OK:" : bladerf_strerror(rval)) << ':' << finame << ':' << line << ':'
			  << funcname << ':' << fname;
		expand_args(std::cerr, args...);
	}
	return rval;
}

// only macros can pass a func name string
#define blade_check(func, ...) exec_and_check(func, #func, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)

#pragma pack(push, 1)
using blade_sample_type = std::complex<int16_t>;
enum class blade_speed_buffer_type { HS, SS };
template <blade_speed_buffer_type T>
struct blade_usb_message {
	uint32_t reserved;
	uint64_t ts;
	uint32_t meta_flags;
	blade_sample_type d[(T == blade_speed_buffer_type::SS ? 512 : 256) - 4];
};

static_assert(sizeof(blade_usb_message<blade_speed_buffer_type::SS>) == 2048, "blade buffer mismatch!");
static_assert(sizeof(blade_usb_message<blade_speed_buffer_type::HS>) == 1024, "blade buffer mismatch!");
template <unsigned int SZ, blade_speed_buffer_type T>
struct blade_otw_buffer {
	static_assert((SZ >= 2 && !(SZ % 2)), "min size is 2x usb buffer!");
	blade_usb_message<T> m[SZ];
	int actual_samples_per_msg()
	{
		return sizeof(blade_usb_message<T>::d) / sizeof(typeof(blade_usb_message<T>::d[0]));
	}
	int actual_samples_per_buffer()
	{
		return SZ * actual_samples_per_msg();
	}
	int samples_per_buffer()
	{
		return SZ * sizeof(blade_usb_message<T>) / sizeof(typeof(blade_usb_message<T>::d[0]));
	}
	int num_msgs_per_buffer()
	{
		return SZ;
	}
	auto get_first_ts()
	{
		return m[0].ts;
	}
	constexpr auto *getsampleoffset(int ofs)
	{
		auto full = ofs / actual_samples_per_msg();
		auto rem = ofs % actual_samples_per_msg();
		return &m[full].d[rem];
	}
	int readall(blade_sample_type *outaddr)
	{
		blade_sample_type *addr = outaddr;
		for (unsigned int i = 0; i < SZ; i++) {
			memcpy(addr, &m[i].d[0], actual_samples_per_msg() * sizeof(blade_sample_type));
			addr += actual_samples_per_msg();
		}
		return actual_samples_per_buffer();
	}
	int read_n(blade_sample_type *outaddr, int start, int num)
	{
		assert((start + num) <= actual_samples_per_buffer());
		assert(start >= 0);

		if (!num)
			return 0;

		// which buffer?
		int start_buf_idx = (start > 0) ? start / actual_samples_per_msg() : 0;
		// offset from actual buffer start
		auto start_offset_in_buf = (start - (start_buf_idx * actual_samples_per_msg()));
		auto samp_rem_in_first_buf = actual_samples_per_msg() - start_offset_in_buf;
		auto remaining_first_buf = num > samp_rem_in_first_buf ? samp_rem_in_first_buf : num;

		memcpy(outaddr, &m[start_buf_idx].d[start_offset_in_buf],
		       remaining_first_buf * sizeof(blade_sample_type));
		outaddr += remaining_first_buf;

		auto remaining = num - remaining_first_buf;

		if (!remaining)
			return num;

		start_buf_idx++;

		auto rem_full_bufs = remaining / actual_samples_per_msg();
		remaining -= rem_full_bufs * actual_samples_per_msg();

		for (int i = 0; i < rem_full_bufs; i++) {
			memcpy(outaddr, &m[start_buf_idx++].d[0], actual_samples_per_msg() * sizeof(blade_sample_type));
			outaddr += actual_samples_per_msg();
		}

		if (remaining)
			memcpy(outaddr, &m[start_buf_idx].d[0], remaining * sizeof(blade_sample_type));
		return num;
	}
	int write_n_burst(blade_sample_type *in, int num, uint64_t first_ts)
	{
		assert(num <= actual_samples_per_buffer());
		int len_rem = num;
		for (unsigned int i = 0; i < SZ; i++) {
			m[i] = {};
			m[i].ts = first_ts + i * actual_samples_per_msg();
			if (len_rem) {
				int max_to_copy =
					len_rem > actual_samples_per_msg() ? actual_samples_per_msg() : len_rem;
				memcpy(&m[i].d[0], in, max_to_copy * sizeof(blade_sample_type));
				len_rem -= max_to_copy;
				in += actual_samples_per_msg();
			}
		}
		return num;
	}
};
#pragma pack(pop)

template <unsigned int SZ, blade_speed_buffer_type T>
struct blade_otw_buffer_helper {
	static_assert((SZ >= 1024 && ((SZ & (SZ - 1)) == 0)), "only buffer size multiples of 1024 allowed!");
	static blade_otw_buffer<SZ / 512, T> x;
};

using dev_buf_t = typeof(blade_otw_buffer_helper<BLADE_BUFFER_SIZE, blade_speed_buffer_type::SS>::x);
// using buf_in_use = blade_otw_buffer<2, blade_speed_buffer_type::SS>;
using bh_fn_t = std::function<int(dev_buf_t *)>;

template <typename T>
struct blade_hw {
	struct bladerf *dev;
	struct bladerf_stream *rx_stream;
	struct bladerf_stream *tx_stream;
	// using pkt2buf = blade_otw_buffer<2, blade_speed_buffer_type::SS>;
	using tx_buf_q_type = spsc_cond_timeout<BLADE_NUM_BUFFERS, dev_buf_t *, true, false>;
	const unsigned int rxFullScale, txFullScale;
	const int rxtxdelay;
	bool use_agc;

	static std::atomic<bool> stop_lower_threads_flag;
	double rxfreq_cache, txfreq_cache;

	struct ms_trx_config {
		int tx_freq;
		int rx_freq;
		int sample_rate;
		int bandwidth;
		float rxgain;
		float txgain;

	    public:
		ms_trx_config()
			: tx_freq(881e6), rx_freq(926e6), sample_rate(((1625e3 / 6) * 4)), bandwidth(1e6), rxgain(30),
			  txgain(30)
		{
		}
	} cfg;

	struct buf_mgmt {
		void **rx_samples;
		void **tx_samples;
		tx_buf_q_type bufptrqueue;

	} buf_mgmt;

	virtual ~blade_hw()
	{
		close_device();
	}
	blade_hw(struct mssdr_cfg *cfgdata)
		: rxFullScale(2047), txFullScale(2047), rxtxdelay(-60), use_agc(cfgdata->use_agc), rxfreq_cache(0),
		  txfreq_cache(0)
	{
		cfg.tx_freq = cfgdata->overrides.ul_freq;
		cfg.rx_freq = cfgdata->overrides.dl_freq;
		cfg.rxgain = cfgdata->overrides.dl_gain;
		cfg.txgain = cfgdata->overrides.ul_gain;
	}

	void close_device()
	{
		if (dev) {
			if (tx_stream) {
				bladerf_deinit_stream(tx_stream);
			}

			if (rx_stream) {
				bladerf_deinit_stream(rx_stream);
			}

			bladerf_enable_module(dev, BLADERF_MODULE_RX, false);
			bladerf_enable_module(dev, BLADERF_MODULE_TX, false);

			bladerf_close(dev);
			dev = NULL;
		}
	}

	int init_device(bh_fn_t rxh, bh_fn_t txh)
	{
		struct bladerf_rational_rate rate = { 0, static_cast<uint64_t>((1625e3 * 4)) * 64, 6 * 64 }, actual;
		std::cerr << "cfg: ul " << cfg.tx_freq << " dl " << cfg.rx_freq << std::endl;

		bladerf_log_set_verbosity(BLADERF_LOG_LEVEL_DEBUG);
		bladerf_set_usb_reset_on_open(true);

		blade_check(bladerf_open, &dev, "");
		if (!dev) {
			std::cerr << "open failed, device missing?" << std::endl;
			exit(0);
		}
		if (bladerf_device_speed(dev) != bladerf_dev_speed::BLADERF_DEVICE_SPEED_SUPER) {
			std::cerr << "open failed, only superspeed (usb3) supported!" << std::endl;
			return -1;
		}

		blade_check(bladerf_set_tuning_mode, dev, bladerf_tuning_mode::BLADERF_TUNING_MODE_FPGA);

		bool is_locked;
		blade_check(bladerf_set_pll_enable, dev, true);
		uint64_t refclock = 10000000UL;
		blade_check(bladerf_set_pll_refclk, dev, refclock);
		for (int i = 0; i < 20; i++) {
			usleep(50 * 1000);
			bladerf_get_pll_lock_state(dev, &is_locked);

			if (is_locked)
				break;
		}
		if (!is_locked) {
			std::cerr << "unable to lock refclk!" << std::endl;
			return -1;
		}

		blade_check(bladerf_set_rational_sample_rate, dev, BLADERF_CHANNEL_RX(0), &rate, &actual);
		blade_check(bladerf_set_rational_sample_rate, dev, BLADERF_CHANNEL_TX(0), &rate, &actual);

		blade_check(bladerf_set_frequency, dev, BLADERF_CHANNEL_RX(0), (bladerf_frequency)cfg.rx_freq);
		blade_check(bladerf_set_frequency, dev, BLADERF_CHANNEL_TX(0), (bladerf_frequency)cfg.tx_freq);

		blade_check(bladerf_set_bandwidth, dev, BLADERF_CHANNEL_RX(0), (bladerf_bandwidth)cfg.bandwidth,
			    (bladerf_bandwidth *)NULL);
		blade_check(bladerf_set_bandwidth, dev, BLADERF_CHANNEL_TX(0), (bladerf_bandwidth)cfg.bandwidth,
			    (bladerf_bandwidth *)NULL);

		blade_check(bladerf_set_gain_mode, dev, BLADERF_CHANNEL_RX(0),
			    use_agc ? BLADERF_GAIN_AUTOMATIC : BLADERF_GAIN_MGC);
		setRxGain(cfg.rxgain, 0);
		setTxGain(cfg.txgain, 0);
		usleep(1000);

		bladerf_set_stream_timeout(dev, BLADERF_TX, 10);
		bladerf_set_stream_timeout(dev, BLADERF_RX, 10);

		blade_check(bladerf_init_stream, &rx_stream, dev, getrxcb(rxh), &buf_mgmt.rx_samples, BLADE_NUM_BUFFERS,
			    BLADERF_FORMAT_SC16_Q11_META, BLADE_BUFFER_SIZE, NUM_TRANSFERS, (void *)this);

		blade_check(bladerf_init_stream, &tx_stream, dev, gettxcb(txh), &buf_mgmt.tx_samples, BLADE_NUM_BUFFERS,
			    BLADERF_FORMAT_SC16_Q11_META, BLADE_BUFFER_SIZE, NUM_TRANSFERS, (void *)this);

		for (unsigned int i = 0; i < BLADE_NUM_BUFFERS; i++) {
			auto cur_buffer = reinterpret_cast<tx_buf_q_type::elem_t *>(buf_mgmt.tx_samples);
			buf_mgmt.bufptrqueue.spsc_push(&cur_buffer[i]);
		}

		return 0;
	}

	void actually_enable_streams()
	{
		blade_check(bladerf_enable_module, dev, BLADERF_MODULE_RX, true);
		usleep(1000);
		blade_check(bladerf_enable_module, dev, BLADERF_MODULE_TX, true);
	}

	bool tuneTx(double freq, size_t chan = 0)
	{
		if (txfreq_cache == freq)
			return true;
		msleep(15);
		blade_check(bladerf_set_frequency, dev, BLADERF_CHANNEL_TX(0), (bladerf_frequency)freq);
		txfreq_cache = freq;
		msleep(15);
		return true;
	};
	bool tuneRx(double freq, size_t chan = 0)
	{
		if (rxfreq_cache == freq)
			return true;
		msleep(15);
		blade_check(bladerf_set_frequency, dev, BLADERF_CHANNEL_RX(0), (bladerf_frequency)freq);
		rxfreq_cache = freq;
		msleep(15);
		return true;
	};
	bool tuneRxOffset(double offset, size_t chan = 0)
	{
		return true;
	};

	double setRxGain(double dB, size_t chan = 0)
	{
		cfg.rxgain = dB;
		msleep(15);
		blade_check(bladerf_set_gain, dev, BLADERF_CHANNEL_RX(0), (bladerf_gain)dB);
		msleep(15);
		return dB;
	};
	double setTxGain(double dB, size_t chan = 0)
	{
		cfg.txgain = dB;
		msleep(15);
		blade_check(bladerf_set_gain, dev, BLADERF_CHANNEL_TX(0), (bladerf_gain)dB);
		msleep(15);
		return dB;
	};
	int setPowerAttenuation(int atten, size_t chan = 0)
	{
		return atten;
	};

	static void check_timestamp(dev_buf_t *rcd)
	{
		static bool first = true;
		static uint64_t last_ts;
		if (first) {
			first = false;
			last_ts = rcd->m[0].ts;
		} else if (last_ts + rcd->actual_samples_per_buffer() != rcd->m[0].ts) {
			std::cerr << "RX Overrun!" << last_ts << " " << rcd->actual_samples_per_buffer() << " "
				  << last_ts + rcd->actual_samples_per_buffer() << " " << rcd->m[0].ts << std::endl;
			last_ts = rcd->m[0].ts;
		} else {
			last_ts = rcd->m[0].ts;
		}
	}

	bladerf_stream_cb getrxcb(bh_fn_t rxbh)
	{
		// C cb -> no capture!
		static auto rxbhfn = rxbh;
		return [](struct bladerf *dev, struct bladerf_stream *stream, struct bladerf_metadata *meta,
			  void *samples, size_t num_samples, void *user_data) -> void * {
			// struct blade_hw *trx = (struct blade_hw *)user_data;
			static int to_skip = 0;
			dev_buf_t *rcd = (dev_buf_t *)samples;

			if (stop_lower_threads_flag)
				return BLADERF_STREAM_SHUTDOWN;

			if (to_skip < 120) // prevents weird overflows on startup
				to_skip++;
			else {
				check_timestamp(rcd);
				rxbhfn(rcd);
			}

			return samples;
		};
	}
	bladerf_stream_cb gettxcb(bh_fn_t txbh)
	{
		// C cb -> no capture!
		static auto txbhfn = txbh;
		return [](struct bladerf *dev, struct bladerf_stream *stream, struct bladerf_metadata *meta,
			  void *samples, size_t num_samples, void *user_data) -> void * {
			struct blade_hw *trx = (struct blade_hw *)user_data;
			auto ptr = reinterpret_cast<tx_buf_q_type::elem_t>(samples);

			if (samples) // put buffer address back into queue, ready to be reused
				trx->buf_mgmt.bufptrqueue.spsc_push(&ptr);

			if (stop_lower_threads_flag)
				return BLADERF_STREAM_SHUTDOWN;

			return BLADERF_STREAM_NO_DATA;
		};
	}

	auto get_rx_burst_handler_fn(bh_fn_t burst_handler)
	{
		using thist = decltype(this);
		auto fn = [](void *args) -> void * {
			thist t = reinterpret_cast<thist>(args);
			int status = 0;
			if (!stop_lower_threads_flag)
				status = bladerf_stream(t->rx_stream, BLADERF_RX_X1);
			if (status < 0)
				std::cerr << "rx stream error! " << bladerf_strerror(status) << std::endl;

			return 0;
		};
		return fn;
	}
	auto get_tx_burst_handler_fn(bh_fn_t burst_handler)
	{
		using thist = decltype(this);
		auto fn = [](void *args) -> void * {
			thist t = reinterpret_cast<thist>(args);
			int status = 0;
			if (!stop_lower_threads_flag)
				status = bladerf_stream(t->tx_stream, BLADERF_TX_X1);
			if (status < 0)
				std::cerr << "rx stream error! " << bladerf_strerror(status) << std::endl;

			return 0;
		};
		return fn;
	}

	void submit_burst_ts(blade_sample_type *buffer, int len, uint64_t ts)
	{
		tx_buf_q_type::elem_t rcd;

		// exit by submitting a dummy buffer to assure the libbladerf stream mutex is happy (thread!)
		if (!buffer) {
			bladerf_submit_stream_buffer(tx_stream, (void *)BLADERF_STREAM_SHUTDOWN, 1000);
			return;
		}

		//get empty bufer from list
		while (!buf_mgmt.bufptrqueue.spsc_pop(&rcd))
			buf_mgmt.bufptrqueue.spsc_prep_pop();
		assert(rcd != nullptr);

		rcd->write_n_burst(buffer, len, ts + rxtxdelay); // blade xa4 specific delay!
		blade_check(bladerf_submit_stream_buffer_nb, tx_stream, (void *)rcd);
	}
};
