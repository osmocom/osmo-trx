/*
 * Device support for Ettus Research UHD driver
 *
 * Copyright 2010,2011 Free Software Foundation, Inc.
 * Copyright (C) 2015 Ettus Research LLC
 * Copyright 2019 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 *
 * Author: Tom Tsou <tom.tsou@ettus.com>
 *
 * SPDX-License-Identifier: AGPL-3.0+
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 */

#pragma once

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "radioDevice.h"
#include "smpl_buf.h"

#include <uhd/version.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/usrp/multi_usrp.hpp>

extern "C" {
#include <osmocom/gsm/gsm_utils.h>
}


enum uhd_dev_type {
	USRP1,
	USRP2,
	B100,
	B200,
	B210,
	B2XX_MCBTS,
	E1XX,
	E3XX,
	X3XX,
	UMTRX,
	LIMESDR,
};

struct dev_band_desc {
	/* Maximum UHD Tx Gain which can be set/used without distorting the
	   output signal, and the resulting real output power measured when that
	   gain is used. Correct measured values only provided for B210 so far. */
	double nom_uhd_tx_gain;  /* dB */
	double nom_out_tx_power; /* dBm */
	/* Factor used to infer base real RSSI offset on the Rx path based on current
	   configured RxGain. The resulting rssiOffset is added to the per burst
	   calculated energy in upper layers. These values were empirically
	   found and may change based on multiple factors, see OS#4468.
	   rssiOffset = rxGain + rxgain2rssioffset_rel;
	*/
	double rxgain2rssioffset_rel; /* dB */
};

/*
    uhd_device - UHD implementation of the Device interface. Timestamped samples
                are sent to and received from the device. An intermediate buffer
                on the receive side collects and aligns packets of samples.
                Events and errors such as underruns are reported asynchronously
                by the device and received in a separate thread.
*/
class uhd_device : public RadioDevice {
public:
	uhd_device(size_t tx_sps, size_t rx_sps, InterfaceType type,
		   size_t chan_num, double offset,
		   const std::vector<std::string>& tx_paths,
		   const std::vector<std::string>& rx_paths);
	~uhd_device();

	int open(const std::string &args, int ref, bool swap_channels);
	bool start();
	bool stop();
	bool restart();
	enum TxWindowType getWindowType() { return tx_window; }

	int readSamples(std::vector<short *> &bufs, int len, bool *overrun,
			TIMESTAMP timestamp, bool *underrun);

	int writeSamples(std::vector<short *> &bufs, int len, bool *underrun,
			 TIMESTAMP timestamp);

	bool updateAlignment(TIMESTAMP timestamp);

	bool setTxFreq(double wFreq, size_t chan);
	bool setRxFreq(double wFreq, size_t chan);

	TIMESTAMP initialWriteTimestamp();
	TIMESTAMP initialReadTimestamp();

	double fullScaleInputValue();
	double fullScaleOutputValue();

	double setRxGain(double db, size_t chan);
	double getRxGain(size_t chan);
	double maxRxGain(void) { return rx_gain_max; }
	double minRxGain(void) { return rx_gain_min; }
	double rssiOffset(size_t chan);

	double setPowerAttenuation(int atten, size_t chan);
	double getPowerAttenuation(size_t chan = 0);

	int getNominalTxPower(size_t chan = 0);

	double getTxFreq(size_t chan);
	double getRxFreq(size_t chan);
	double getRxFreq();

	bool setRxAntenna(const std::string &ant, size_t chan);
	std::string getRxAntenna(size_t chan);
	bool setTxAntenna(const std::string &ant, size_t chan);
	std::string getTxAntenna(size_t chan);

	bool requiresRadioAlign();

	GSM::Time minLatency();

	inline double getSampleRate() { return tx_rate; }

	/** Receive and process asynchronous message
	    @return true if message received or false on timeout or error
	*/
	bool recv_async_msg();

	enum err_code {
		ERROR_TIMING = -1,
		ERROR_TIMEOUT = -2,
		ERROR_UNRECOVERABLE = -3,
		ERROR_UNHANDLED = -4,
	};

protected:
	uhd::usrp::multi_usrp::sptr usrp_dev;
	uhd::tx_streamer::sptr tx_stream;
	uhd::rx_streamer::sptr rx_stream;
	enum TxWindowType tx_window;
	enum uhd_dev_type dev_type;

	double tx_rate, rx_rate;

	double rx_gain_min, rx_gain_max;

	std::vector<double> tx_gains, rx_gains;
	std::vector<double> tx_freqs, rx_freqs;
	bool band_ass_curr_sess; /* true if  "band" was set after last POWEROFF */
	enum gsm_band band;
	struct dev_band_desc band_desc;
	size_t tx_spp, rx_spp;

	bool started;
	bool aligned;

	size_t drop_cnt;
	uhd::time_spec_t prev_ts;

	TIMESTAMP ts_initial, ts_offset;
	std::vector<smpl_buf *> rx_buffers;
	/* Sample buffers used to receive samples from UHD: */
	std::vector<std::vector<short> > pkt_bufs;
	/* Used to call UHD API: Buffer pointer of each elem in pkt_ptrs will
	   point to corresponding buffer of vector pkt_bufs. */
	std::vector<short *> pkt_ptrs;

	void init_gains();
	void set_channels(bool swap);
	void set_rates();
	bool parse_dev_type();
	bool flush_recv(size_t num_pkts);
	int check_rx_md_err(uhd::rx_metadata_t &md, ssize_t num_smpls);

	std::string str_code(uhd::rx_metadata_t metadata);
	std::string str_code(uhd::async_metadata_t metadata);

	uhd::tune_request_t select_freq(double wFreq, size_t chan, bool tx);
	bool set_freq(double freq, size_t chan, bool tx);
	void get_dev_band_desc(dev_band_desc& desc);
	bool set_band(enum gsm_band req_band);
	void assign_band_desc(enum gsm_band req_band);

	Thread *async_event_thrd;
	Mutex tune_lock;
};
