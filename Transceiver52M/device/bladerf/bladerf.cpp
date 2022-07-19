/*
 * Copyright 2022 sysmocom - s.f.m.c. GmbH
 *
 * Author: Eric Wild <ewild@sysmocom.de>
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

#include <map>
#include <libbladeRF.h>
#include "radioDevice.h"
#include "bladerf.h"
#include "Threads.h"
#include "Logger.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

extern "C" {
#include <osmocom/core/utils.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/vty/cpu_sched_vty.h>
}

#define USRP_TX_AMPL     0.3
#define UMTRX_TX_AMPL    0.7
#define LIMESDR_TX_AMPL  0.3
#define SAMPLE_BUF_SZ    (1 << 20)

/*
 * UHD timeout value on streaming (re)start
 *
 * Allow some time for streaming to commence after the start command is issued,
 * but consider a wait beyond one second to be a definite error condition.
 */
#define UHD_RESTART_TIMEOUT     1.0

/*
 * UmTRX specific settings
 */
#define UMTRX_VGA1_DEF   -18

/*
 * USRP version dependent device timings
 */

#define B2XX_TIMING_1SPS	1.7153e-4
#define B2XX_TIMING_4SPS	1.1696e-4
#define B2XX_TIMING_4_4SPS	6.18462e-5
#define B2XX_TIMING_MCBTS	7e-5

#define CHKRET() { \
	if(status !=0) \
	fprintf(stderr, "%s:%s:%d %s\n", __FILE__,__FUNCTION__, __LINE__, bladerf_strerror(status)); \
	}

/*
 * Tx / Rx sample offset values. In a perfect world, there is no group delay
 * though analog components, and behaviour through digital filters exactly
 * matches calculated values. In reality, there are unaccounted factors,
 * which are captured in these empirically measured (using a loopback test)
 * timing correction values.
 *
 * Notes:
 *   USRP1 with timestamps is not supported by UHD.
 */

/* Device Type, Tx-SPS, Rx-SPS */
typedef std::tuple<blade_dev_type, int, int> dev_key;

/* Device parameter descriptor */
struct dev_desc {
	unsigned channels;
	double mcr;
	double rate;
	double offset;
	std::string str;
};

static const std::map<dev_key, dev_desc> dev_param_map {
	{ std::make_tuple(blade_dev_type::BLADE2,  1, 1), { 1, 26e6, GSMRATE, B2XX_TIMING_1SPS, "B200 1 SPS"   } },
	{ std::make_tuple(blade_dev_type::BLADE2,  4, 1), { 1, 26e6, GSMRATE, B2XX_TIMING_4SPS, "B200 4/1 Tx/Rx SPS" } },
	{ std::make_tuple(blade_dev_type::BLADE2,  4, 4), { 1, 26e6, GSMRATE, B2XX_TIMING_4_4SPS, "B200 4 SPS" } },
};

typedef std::tuple<blade_dev_type, enum gsm_band> dev_band_key;
typedef std::map<dev_band_key, dev_band_desc>::const_iterator dev_band_map_it;
static const std::map<dev_band_key, dev_band_desc> dev_band_nom_power_param_map {
	{ std::make_tuple(blade_dev_type::BLADE2, GSM_BAND_850),	{ 89.75, 13.3, -7.5  } },
	{ std::make_tuple(blade_dev_type::BLADE2, GSM_BAND_900),	{ 89.75, 13.3, -7.5  } },
	{ std::make_tuple(blade_dev_type::BLADE2, GSM_BAND_1800),	{ 89.75, 7.5,  -11.0 } },
	{ std::make_tuple(blade_dev_type::BLADE2, GSM_BAND_1900),	{ 89.75, 7.7,  -11.0 } },
};


/* So far measurements done for B210 show really close to linear relationship
 * between gain and real output power, so we simply adjust the measured offset
 */
static double TxGain2TxPower(const dev_band_desc &desc, double tx_gain_db)
{
	return desc.nom_out_tx_power - (desc.nom_uhd_tx_gain - tx_gain_db);
}
static double TxPower2TxGain(const dev_band_desc &desc, double tx_power_dbm)
{
	return desc.nom_uhd_tx_gain - (desc.nom_out_tx_power - tx_power_dbm);
}

blade_device::blade_device(size_t tx_sps, size_t rx_sps,
		       InterfaceType iface, size_t chan_num, double lo_offset,
		       const std::vector<std::string>& tx_paths,
		       const std::vector<std::string>& rx_paths)
	: RadioDevice(tx_sps, rx_sps, iface, chan_num, lo_offset, tx_paths, rx_paths),
	  dev(nullptr), rx_gain_min(0.0), rx_gain_max(0.0),
	  band_ass_curr_sess(false), band((enum gsm_band)0), tx_spp(0),
	  rx_spp(0), started(false), aligned(false),
	  drop_cnt(0), prev_ts(0), ts_initial(0), ts_offset(0), async_event_thrd(NULL)
{
}

blade_device::~blade_device()
{
	if(dev) {
		bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), false);
		bladerf_enable_module(dev, BLADERF_CHANNEL_TX(0), false);
	}

	stop();

	for (size_t i = 0; i < rx_buffers.size(); i++)
		delete rx_buffers[i];
}

void blade_device::assign_band_desc(enum gsm_band req_band)
{
	dev_band_map_it it;

	it = dev_band_nom_power_param_map.find(dev_band_key(dev_type, req_band));
	if (it == dev_band_nom_power_param_map.end()) {
		dev_desc desc = dev_param_map.at(dev_key(dev_type, tx_sps, rx_sps));
		LOGC(DDEV, ERROR) << "No Power parameters exist for device "
				    << desc.str << " on band " << gsm_band_name(req_band)
				    << ", using B210 ones as fallback";
		it = dev_band_nom_power_param_map.find(dev_band_key(blade_dev_type::BLADE2, req_band));
	}
	OSMO_ASSERT(it != dev_band_nom_power_param_map.end())
	band_desc = it->second;
}

bool blade_device::set_band(enum gsm_band req_band)
{
	if (band_ass_curr_sess && req_band != band) {
		LOGC(DDEV, ALERT) << "Requesting band " << gsm_band_name(req_band)
				  << " different from previous band " << gsm_band_name(band);
		return false;
	}

	if (req_band != band) {
		band = req_band;
		assign_band_desc(band);
	}
	band_ass_curr_sess = true;
	return true;
}

void blade_device::get_dev_band_desc(dev_band_desc& desc)
{
	if (band == 0) {
		LOGC(DDEV, ERROR) << "Power parameters requested before Tx Frequency was set! Providing band 900 by default...";
		assign_band_desc(GSM_BAND_900);
	}
	desc = band_desc;
}

void blade_device::init_gains()
{
	double tx_gain_min, tx_gain_max;
	int status;

	
	const struct bladerf_range* r;
	bladerf_get_gain_range(dev, BLADERF_RX, &r);


	rx_gain_min = r->min;
	rx_gain_max = r->max;
	LOGC(DDEV, INFO) << "Supported Rx gain range [" << rx_gain_min << "; " << rx_gain_max << "]";

	for (size_t i = 0; i < rx_gains.size(); i++) {
		double gain = (rx_gain_min + rx_gain_max) / 2;
		status = bladerf_set_gain_mode(dev, BLADERF_CHANNEL_RX(i), BLADERF_GAIN_MGC );
		CHKRET()
		bladerf_gain_mode m;
		bladerf_get_gain_mode(dev, BLADERF_CHANNEL_RX(i), &m);
		LOGC(DDEV, INFO) << (m == BLADERF_GAIN_MANUAL ? "gain manual" : "gain AUTO") ;

		status = bladerf_set_gain(dev, BLADERF_CHANNEL_RX(i), 0);//gain);
		CHKRET()
		int actual_gain;
		status = bladerf_get_gain(dev, BLADERF_CHANNEL_RX(i), &actual_gain);
		CHKRET()
		LOGC(DDEV, INFO) << "Default setting Rx gain for channel " << i << " to " << gain << " scale " << r->scale << " actual " << actual_gain;
		rx_gains[i] = actual_gain;

		status = bladerf_set_gain(dev, BLADERF_CHANNEL_RX(i), 0);//gain);
		CHKRET()
		status = bladerf_get_gain(dev, BLADERF_CHANNEL_RX(i), &actual_gain);
		CHKRET()
		LOGC(DDEV, INFO) << "Default setting Rx gain for channel " << i << " to " << gain << " scale " << r->scale << " actual " << actual_gain;
		rx_gains[i] = actual_gain;
	}

	status = bladerf_get_gain_range(dev, BLADERF_TX, &r);
	CHKRET()
	tx_gain_min = r->min;
	tx_gain_max = r->max;
	LOGC(DDEV, INFO) << "Supported Tx gain range [" << tx_gain_min << "; " << tx_gain_max << "]";

	for (size_t i = 0; i < tx_gains.size(); i++) {
		double gain = (tx_gain_min + tx_gain_max) / 2;
		status = bladerf_set_gain(dev, BLADERF_CHANNEL_TX(i), 30);//gain);
		CHKRET()
		int actual_gain;
		status = bladerf_get_gain(dev, BLADERF_CHANNEL_TX(i), &actual_gain);
		CHKRET()
		LOGC(DDEV, INFO) << "Default setting Tx gain for channel " << i << " to " << gain << " scale " << r->scale << " actual " << actual_gain;
		tx_gains[i] = actual_gain;
	}

	return;

}

void blade_device::set_rates()
{
	//dev_desc desc = dev_param_map.at(dev_key(dev_type, tx_sps, rx_sps));

	struct bladerf_rational_rate rate = {0, static_cast<uint64_t>((1625e3 * 4)), 6}, actual;
	auto status = bladerf_set_rational_sample_rate(dev, BLADERF_CHANNEL_RX(0), &rate, &actual);
	CHKRET()
	status = bladerf_set_rational_sample_rate(dev, BLADERF_CHANNEL_TX(0), &rate, &actual);
	CHKRET()


	tx_rate = rx_rate = (double)rate.num/(double)rate.den;

	LOGC(DDEV, INFO) << "Rates set to" << tx_rate << " / " << rx_rate;


    bladerf_set_bandwidth(dev, BLADERF_CHANNEL_RX(0), (bladerf_bandwidth)2e6, (bladerf_bandwidth*)NULL);
    bladerf_set_bandwidth(dev, BLADERF_CHANNEL_TX(0), (bladerf_bandwidth)2e6, (bladerf_bandwidth*)NULL);

	ts_offset = 60;//static_cast<TIMESTAMP>(desc.offset * rx_rate);
	//LOGC(DDEV, INFO) << "Rates configured for " << desc.str;
}

double blade_device::setRxGain(double db, size_t chan)
{
	if (chan >= rx_gains.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return 0.0f;
	}

	bladerf_set_gain(dev, BLADERF_CHANNEL_RX(chan), 30);//db);
	int actual_gain;
	bladerf_get_gain(dev, BLADERF_CHANNEL_RX(chan), &actual_gain);

	rx_gains[chan] = actual_gain;

	LOGC(DDEV, INFO) << "Set RX gain to " << rx_gains[chan] << "dB (asked for " << db << "dB)";

	return rx_gains[chan];
}

double blade_device::getRxGain(size_t chan)
{
	if (chan >= rx_gains.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return 0.0f;
	}

	return rx_gains[chan];
}

double blade_device::rssiOffset(size_t chan)
{
	double rssiOffset;
	dev_band_desc desc;

	if (chan >= rx_gains.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return 0.0f;
	}

	get_dev_band_desc(desc);
	rssiOffset = rx_gains[chan] + desc.rxgain2rssioffset_rel;
	return rssiOffset;
}

double blade_device::setPowerAttenuation(int atten, size_t chan) {
	double tx_power, db;
	dev_band_desc desc;

	if (chan >= tx_gains.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel" << chan;
		return 0.0f;
	}

	get_dev_band_desc(desc);
	tx_power = desc.nom_out_tx_power - atten;
	db = TxPower2TxGain(desc, tx_power);

	bladerf_set_gain(dev, BLADERF_CHANNEL_TX(chan), 30);//db);
	int actual_gain;
	bladerf_get_gain(dev, BLADERF_CHANNEL_RX(chan), &actual_gain);

	tx_gains[chan] = actual_gain;

	LOGC(DDEV, INFO) << "Set TX gain to " << tx_gains[chan] << "dB, ~"
			 <<  TxGain2TxPower(desc, tx_gains[chan]) << " dBm "
			 << "(asked for " << db << " dB, ~" << tx_power << " dBm)";

	return desc.nom_out_tx_power - TxGain2TxPower(desc, tx_gains[chan]);
}
double blade_device::getPowerAttenuation(size_t chan) {
	dev_band_desc desc;
	if (chan >= tx_gains.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return 0.0f;
	}

	get_dev_band_desc(desc);
	return desc.nom_out_tx_power - TxGain2TxPower(desc, tx_gains[chan]);
}

int blade_device::getNominalTxPower(size_t chan)
{
	dev_band_desc desc;
	get_dev_band_desc(desc);

	return desc.nom_out_tx_power;
}


int blade_device::open(const std::string &args, int ref, bool swap_channels)
{

	bladerf_log_set_verbosity(BLADERF_LOG_LEVEL_VERBOSE);
	bladerf_set_usb_reset_on_open(true);
	auto success = bladerf_open(&dev, args.c_str());
	if(success != 0) {
		struct bladerf_devinfo* info;
		auto num_devs = bladerf_get_device_list(&info);
		LOGC(DDEV, ALERT) << "No bladerf devices found with identifier '" << args << "'";
		if(num_devs) {
			for(int i=0; i < num_devs; i++)
				LOGC(DDEV, ALERT) << "Found device:" << info[i].product << " serial " << info[i].serial;
		}

		return -1;
	}
	if(strcmp("bladerf2", bladerf_get_board_name(dev))) {
		LOGC(DDEV, ALERT) << "Only BladeRF2 supported! found:" << bladerf_get_board_name(dev) ;
		return -1;
	}

	dev_type = blade_dev_type::BLADE2;
	tx_window = TX_WINDOW_FIXED;
	
	struct bladerf_devinfo info;
	bladerf_get_devinfo(dev, &info);
	// Use the first found device
	LOGC(DDEV, INFO) << "Using discovered bladerf device " << info.serial;


	tx_freqs.resize(chans);
	rx_freqs.resize(chans);
	tx_gains.resize(chans);
	rx_gains.resize(chans);
	rx_buffers.resize(chans);

	switch (ref) {
	case REF_INTERNAL:
	case REF_EXTERNAL:
		break;
	default:
		LOGC(DDEV, ALERT) << "Invalid reference type";
		return -1;
	}

	if(ref == REF_EXTERNAL) {
		bool is_locked;
		int status = bladerf_set_pll_enable(dev, true);
		CHKRET()
		status = bladerf_set_pll_refclk(dev, 10000000);
		CHKRET()
		for(int i=0; i < 20; i++) {
			usleep(50*1000);
			status = bladerf_get_pll_lock_state(dev, &is_locked);
			CHKRET()
			if(is_locked)
				break;
		}
		if(!is_locked) {
			LOGC(DDEV, ALERT) << "unable to lock refclk!";
			return -1;
		}
	}

	LOGC(DDEV, INFO) << "Selected clock source is " << ((ref == REF_INTERNAL) ? "internal" : "external 10Mhz");

	set_rates();

/*
	1ts = 3/5200s
	1024*2 = small gap(~180us) every 9.23ms = every 16 ts? -> every 2 frames
	1024*1 = large gap(~627us) every 9.23ms = every 16 ts? -> every 2 frames

	rif convertbuffer = 625*4 = 2500 -> 4 ts
	rif rxtxbuf = 4 * segment(625*4) = 10000 -> 16 ts
*/
    const unsigned int num_buffers   = 256;
    const unsigned int buffer_size   = 1024*4; /* Must be a multiple of 1024 */
    const unsigned int num_transfers = 32;
    const unsigned int timeout_ms    = 3500;


    bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11_META,
                                 num_buffers, buffer_size, num_transfers,
                                 timeout_ms);

    bladerf_sync_config(dev, BLADERF_TX_X1, BLADERF_FORMAT_SC16_Q11_META,
                                 num_buffers, buffer_size, num_transfers,
                                 timeout_ms);




	/* Number of samples per over-the-wire packet */
	tx_spp = rx_spp = buffer_size;

	// Create receive buffer
	size_t buf_len = SAMPLE_BUF_SZ / sizeof(uint32_t);
	for (size_t i = 0; i < rx_buffers.size(); i++)
		rx_buffers[i] = new smpl_buf(buf_len);

	// Create vector buffer
	pkt_bufs = std::vector<std::vector<short> >(chans, std::vector<short>(2 * rx_spp));
	for (size_t i = 0; i < pkt_bufs.size(); i++)
		pkt_ptrs.push_back(&pkt_bufs[i].front());

	// Initialize and shadow gain values
	init_gains();

	return NORMAL;
}


bool blade_device::restart()
{
	/* Allow 100 ms delay to align multi-channel streams */
	double delay = 0.2;
	int status;

	status = bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), true);
	CHKRET()
	status = bladerf_enable_module(dev, BLADERF_CHANNEL_TX(0), true);
	CHKRET()

	bladerf_timestamp now;
	status = bladerf_get_timestamp(dev, BLADERF_RX, &now);
	ts_initial = now + rx_rate * delay;
	LOGC(DDEV, INFO) << "Initial timestamp " << ts_initial << std::endl;

	return true;
}

bool blade_device::start()
{
	LOGC(DDEV, INFO) << "Starting USRP...";

	if (started) {
		LOGC(DDEV, ERROR) << "Device already started";
		return false;
	}

	// Start streaming
	if (!restart())
		return false;


	started = true;
	return true;
}

bool blade_device::stop()
{
	if (!started)
		return false;

	/* reset internal buffer timestamps */
	for (size_t i = 0; i < rx_buffers.size(); i++)
		rx_buffers[i]->reset();

	band_ass_curr_sess = false;

	started = false;
	return true;
}

int blade_device::readSamples(std::vector<short *> &bufs, int len, bool *overrun,
			    TIMESTAMP timestamp, bool *underrun)
{
	ssize_t rc;
	uint64_t ts;

	if (bufs.size() != chans) {
		LOGC(DDEV, ALERT) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	*overrun = false;
	*underrun = false;

	// Shift read time with respect to transmit clock
	timestamp += ts_offset;

	ts = timestamp;
	LOGC(DDEV, DEBUG) << "Requested timestamp = " << ts;

	// Check that timestamp is valid
	rc = rx_buffers[0]->avail_smpls(timestamp);
	if (rc < 0) {
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_code(rc);
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_status(timestamp);
		return 0;
	}

	
	struct bladerf_metadata meta = {};
	meta.timestamp = ts;
	//static bool first_rx = true;
	// meta.timestamp = (first_rx) ? ts : 0;
	// meta.flags = (!first_rx) ? 0:BLADERF_META_FLAG_RX_NOW;
	// if(first_rx)
	// 	first_rx = false;

	// Receive samples from the usrp until we have enough
	while (rx_buffers[0]->avail_smpls(timestamp) < len) {
		thread_enable_cancel(false);
		int status = bladerf_sync_rx(dev, pkt_ptrs[0], len, &meta, 200U);
		thread_enable_cancel(true);

		if(status != 0)
			std::cerr << "RX fucked: " << bladerf_strerror(status);
		if(meta.flags & BLADERF_META_STATUS_OVERRUN )
			std::cerr << "RX fucked OVER: " << bladerf_strerror(status);

		size_t num_smpls = meta.actual_count;
;		ts = meta.timestamp;

		for (size_t i = 0; i < rx_buffers.size(); i++) {
			rc = rx_buffers[i]->write((short *) &pkt_bufs[i].front(),
						  num_smpls,
						  ts);

			// Continue on local overrun, exit on other errors
			if ((rc < 0)) {
				LOGC(DDEV, ERROR) << rx_buffers[i]->str_code(rc);
				LOGC(DDEV, ERROR) << rx_buffers[i]->str_status(timestamp);
				if (rc != smpl_buf::ERROR_OVERFLOW)
					return 0;
			}
		}
		meta = {};
		meta.timestamp = ts+num_smpls;
	}

	// We have enough samples
	for (size_t i = 0; i < rx_buffers.size(); i++) {
		rc = rx_buffers[i]->read(bufs[i], len, timestamp);
		if ((rc < 0) || (rc != len)) {
			LOGC(DDEV, ERROR) << rx_buffers[i]->str_code(rc);
			LOGC(DDEV, ERROR) << rx_buffers[i]->str_status(timestamp);
			return 0;
		}
	}

	return len;
}

int blade_device::writeSamples(std::vector<short *> &bufs, int len, bool *underrun,
			     unsigned long long timestamp)
{
	*underrun = false;
	static bool first_tx = true;
	struct bladerf_metadata meta = {};
	if(first_tx) {
		meta.timestamp = timestamp;
		meta.flags = BLADERF_META_FLAG_TX_BURST_START;
		first_tx = false;
	}

	thread_enable_cancel(false);
	int status = bladerf_sync_tx(dev, (const void*)bufs[0], len, &meta, 200U);
	//size_t num_smpls = tx_stream->send(bufs, len, metadata);
	thread_enable_cancel(true);

	if(status != 0)
		std::cerr << "TX fucked: " << bladerf_strerror(status);

	// LOGCHAN(0, DDEV, INFO) << "tx " << timestamp << " " << len << " t+l: "<< timestamp+len << std::endl;

	return len;
}

bool blade_device::updateAlignment(TIMESTAMP timestamp)
{
	return true;
}

bool blade_device::set_freq(double freq, size_t chan, bool tx)
{

	if (tx) {
		bladerf_set_frequency(dev, BLADERF_CHANNEL_TX(chan), freq);
		bladerf_frequency f;
		bladerf_get_frequency(dev,BLADERF_CHANNEL_TX(chan), &f);
		tx_freqs[chan] = f;
	} else {
		bladerf_set_frequency(dev, BLADERF_CHANNEL_RX(chan), freq);
		bladerf_frequency f;
		bladerf_get_frequency(dev,BLADERF_CHANNEL_RX(chan), &f);
		rx_freqs[chan] = f;
	}
	LOGCHAN(chan, DDEV, INFO) << "set_freq(" << freq << ", " << (tx  ? "TX" : "RX") << "): "  << std::endl;


	return true;
}

bool blade_device::setTxFreq(double wFreq, size_t chan)
{
	uint16_t req_arfcn;
	enum gsm_band req_band;

	if (chan >= tx_freqs.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return false;
	}
	ScopedLock lock(tune_lock);

	req_arfcn = gsm_freq102arfcn(wFreq / 1000 / 100 , 0);
	if (req_arfcn == 0xffff) {
		LOGCHAN(chan, DDEV, ALERT) << "Unknown ARFCN for Tx Frequency " << wFreq / 1000 << " kHz";
		return false;
	}
	if (gsm_arfcn2band_rc(req_arfcn, &req_band) < 0) {
		LOGCHAN(chan, DDEV, ALERT) << "Unknown GSM band for Tx Frequency " << wFreq
					   << " Hz (ARFCN " << req_arfcn << " )";
		return false;
	}

	if (!set_band(req_band))
		return false;

	if (!set_freq(wFreq, chan, true))
		return false;

	return true;
}

bool blade_device::setRxFreq(double wFreq, size_t chan)
{
	uint16_t req_arfcn;
	enum gsm_band req_band;

	if (chan >= rx_freqs.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return false;
	}
	ScopedLock lock(tune_lock);

	req_arfcn = gsm_freq102arfcn(wFreq / 1000 / 100, 1);
	if (req_arfcn == 0xffff) {
		LOGCHAN(chan, DDEV, ALERT) << "Unknown ARFCN for Rx Frequency " << wFreq / 1000 << " kHz";
		return false;
	}
	if (gsm_arfcn2band_rc(req_arfcn, &req_band) < 0) {
		LOGCHAN(chan, DDEV, ALERT) << "Unknown GSM band for Rx Frequency " << wFreq
					   << " Hz (ARFCN " << req_arfcn << " )";
		return false;
	}

	if (!set_band(req_band))
		return false;

	return set_freq(wFreq, chan, false);
}

double blade_device::getTxFreq(size_t chan)
{
	if (chan >= tx_freqs.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return 0.0;
	}

	return tx_freqs[chan];
}

double blade_device::getRxFreq(size_t chan)
{
	if (chan >= rx_freqs.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return 0.0;
	}

	return rx_freqs[chan];
}

bool blade_device::requiresRadioAlign()
{
	return false;
}

GSM::Time blade_device::minLatency() {
	/* Empirical data from a handful of
	relatively recent machines shows that the B100 will underrun when
	the transmit threshold is reduced to a time of 6 and a half frames,
	so we set a minimum 7 frame threshold. */
	return GSM::Time(6,7);
}

TIMESTAMP blade_device::initialWriteTimestamp()
{
	return ts_initial;
}

TIMESTAMP blade_device::initialReadTimestamp()
{
	return ts_initial;
}

double blade_device::fullScaleInputValue()
{
		return (double) 2047;
}

double blade_device::fullScaleOutputValue()
{
	return (double) 2047;
}


#ifndef IPCMAGIC
RadioDevice *RadioDevice::make(size_t tx_sps, size_t rx_sps,
			       InterfaceType iface, size_t chans, double lo_offset,
			       const std::vector<std::string>& tx_paths,
			       const std::vector<std::string>& rx_paths)
{
	return new blade_device(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths);
}
#endif
