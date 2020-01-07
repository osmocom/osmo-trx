/*
* Copyright 2018 sysmocom - s.f.m.c. GmbH
*
* SPDX-License-Identifier: AGPL-3.0+
*
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "Logger.h"
#include "Threads.h"
#include "LMSDevice.h"
#include "Utils.h"

#include <lime/LimeSuite.h>

extern "C" {
#include "osmo_signal.h"
#include <osmocom/core/utils.h>
}

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

using namespace std;

#define MAX_ANTENNA_LIST_SIZE 10
#define GSM_CARRIER_BW 270000.0 /* 270kHz */
#define LMS_MIN_BW_SUPPORTED 2.5e6 /* 2.5mHz, minimum supported by LMS */
#define LMS_CALIBRATE_BW_HZ OSMO_MAX(GSM_CARRIER_BW, LMS_MIN_BW_SUPPORTED)
#define SAMPLE_BUF_SZ    (1 << 20) /* Size of Rx timestamp based Ring buffer, in bytes */

LMSDevice::LMSDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chans, double lo_offset,
		     const std::vector<std::string>& tx_paths,
		     const std::vector<std::string>& rx_paths):
	RadioDevice(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths),
	m_lms_dev(NULL), started(false)
{
	LOGC(DDEV, INFO) << "creating LMS device...";

	m_lms_stream_rx.resize(chans);
	m_lms_stream_tx.resize(chans);
	rx_gains.resize(chans);
	tx_gains.resize(chans);

	rx_buffers.resize(chans);
}

LMSDevice::~LMSDevice()
{
	unsigned int i;
	LOGC(DDEV, INFO) << "Closing LMS device";
	if (m_lms_dev) {
		/* disable all channels */
		for (i=0; i<chans; i++) {
			LMS_EnableChannel(m_lms_dev, LMS_CH_RX, i, false);
			LMS_EnableChannel(m_lms_dev, LMS_CH_TX, i, false);
		}
		LMS_Close(m_lms_dev);
		m_lms_dev = NULL;
	}

	for (size_t i = 0; i < rx_buffers.size(); i++)
		delete rx_buffers[i];
}

static void lms_log_callback(int lvl, const char *msg)
{
	/* map lime specific log levels */
	static const int lvl_map[5] = {
		[0] = LOGL_FATAL,
		[LMS_LOG_ERROR] = LOGL_ERROR,
		[LMS_LOG_WARNING] = LOGL_NOTICE,
		[LMS_LOG_INFO] = LOGL_INFO,
		[LMS_LOG_DEBUG] = LOGL_DEBUG,
	};
	/* protect against future higher log level values (lower importance) */
	if ((unsigned int) lvl >= ARRAY_SIZE(lvl_map))
		lvl = ARRAY_SIZE(lvl_map)-1;

	LOGLV(DDEVDRV, lvl_map[lvl]) << msg;
}

static void print_range(const char* name, lms_range_t *range)
{
	LOGC(DDEV, INFO) << name << ": Min=" << range->min << " Max=" << range->max
		   << " Step=" << range->step;
}

/*! Find the device string that matches all filters from \a args.
 *  \param[in] info_list device addresses found by LMS_GetDeviceList()
 *  \param[in] count length of info_list
 *  \param[in] args dev-args value from osmo-trx.cfg, containing comma separated key=value pairs
 *  \return index of first matching device or -1 (no match) */
int info_list_find(lms_info_str_t* info_list, unsigned int count, const std::string &args)
{
	unsigned int i, j;
	vector<string> filters;

	filters = comma_delimited_to_vector(args.c_str());

	/* iterate over device addresses */
	for (i=0; i < count; i++) {
		/* check if all filters match */
		bool match = true;
		for (j=0; j < filters.size(); j++) {
			if (!strstr(info_list[i], filters[j].c_str())) {
				match = false;
				break;
			}
		}

		if (match)
			return i;
	}
	return -1;
}

int LMSDevice::open(const std::string &args, int ref, bool swap_channels)
{
	lms_info_str_t* info_list;
	const lms_dev_info_t* device_info;
	lms_range_t range_sr;
	float_type sr_host, sr_rf;
	unsigned int i, n;
	int rc, dev_id;

	LOGC(DDEV, INFO) << "Opening LMS device..";

	LMS_RegisterLogHandler(&lms_log_callback);

	if ((n = LMS_GetDeviceList(NULL)) < 0)
		LOGC(DDEV, ERROR) << "LMS_GetDeviceList(NULL) failed";
	LOGC(DDEV, INFO) << "Devices found: " << n;
	if (n < 1)
	    return -1;

	info_list = new lms_info_str_t[n];

	if (LMS_GetDeviceList(info_list) < 0)
		LOGC(DDEV, ERROR) << "LMS_GetDeviceList(info_list) failed";

	for (i = 0; i < n; i++)
		LOGC(DDEV, INFO) << "Device [" << i << "]: " << info_list[i];

	dev_id = info_list_find(info_list, n, args);
	if (dev_id == -1) {
		LOGC(DDEV, ERROR) << "No LMS device found with address '" << args << "'";
		delete[] info_list;
		return -1;
	}

	LOGC(DDEV, INFO) << "Using device[" << dev_id << "]";
	rc = LMS_Open(&m_lms_dev, info_list[dev_id], NULL);
	if (rc != 0) {
		LOGC(DDEV, ERROR) << "LMS_GetDeviceList() failed)";
		delete [] info_list;
		return -1;
	}

	delete [] info_list;

	device_info = LMS_GetDeviceInfo(m_lms_dev);

	if ((ref != REF_EXTERNAL) && (ref != REF_INTERNAL)){
		LOGC(DDEV, ERROR) << "Invalid reference type";
		goto out_close;
	}

	/* if reference clock is external setup must happen _before_ calling LMS_Init */
	/* FIXME make external reference frequency configurable */
	if (ref == REF_EXTERNAL) {
		LOGC(DDEV, INFO) << "Setting External clock reference to 10MHz";
		/* Assume an external 10 MHz reference clock */
		if (LMS_SetClockFreq(m_lms_dev, LMS_CLOCK_EXTREF, 10000000.0) < 0)
			goto out_close;
	}

	LOGC(DDEV, INFO) << "Init LMS device";
	if (LMS_Init(m_lms_dev) != 0) {
		LOGC(DDEV, ERROR) << "LMS_Init() failed";
		goto out_close;
	}

	/* LimeSDR-Mini does not have switches but needs soldering to select external/internal clock */
	/* LimeNET-Micro also does not like selecting internal clock*/
	/* also set device specific maximum tx levels selected by phasenoise measurements*/
	if (strncmp(device_info->deviceName,"LimeSDR-USB",11) == 0){
		/* if reference clock is internal setup must happen _after_ calling LMS_Init */
		/* according to lms using LMS_CLOCK_EXTREF with a frequency <= 0 is the correct way to set clock to internal reference*/
		if (ref == REF_INTERNAL) {
			LOGC(DDEV, INFO) << "Setting Internal clock reference";
			if (LMS_SetClockFreq(m_lms_dev, LMS_CLOCK_EXTREF, -1) < 0)
				goto out_close;
		}
		maxTxGainClamp = 73.0;
	} else if (strncmp(device_info->deviceName,"LimeSDR-Mini",12) == 0)
		maxTxGainClamp = 66.0;
	else
		maxTxGainClamp = 71.0; /* "LimeNET-Micro", etc FIXME pciE based LMS boards?*/

	/* enable all used channels */
	for (i=0; i<chans; i++) {
		if (LMS_EnableChannel(m_lms_dev, LMS_CH_RX, i, true) < 0)
			goto out_close;
		if (LMS_EnableChannel(m_lms_dev, LMS_CH_TX, i, true) < 0)
			goto out_close;
	}

	/* set samplerate */
	if (LMS_GetSampleRateRange(m_lms_dev, LMS_CH_RX, &range_sr))
		goto out_close;
	print_range("Sample Rate", &range_sr);

	LOGC(DDEV, INFO) << "Setting sample rate to " << GSMRATE*tx_sps << " " << tx_sps;
	if (LMS_SetSampleRate(m_lms_dev, GSMRATE*tx_sps, 32) < 0)
		goto out_close;

	if (LMS_GetSampleRate(m_lms_dev, LMS_CH_RX, 0, &sr_host, &sr_rf))
		goto out_close;
	LOGC(DDEV, INFO) << "Sample Rate: Host=" << sr_host << " RF=" << sr_rf;

	/* FIXME: make this device/model dependent, like UHDDevice:dev_param_map! */
	ts_offset = static_cast<TIMESTAMP>(8.9e-5 * GSMRATE * tx_sps); /* time * sample_rate */

	/* configure antennas */
	if (!set_antennas()) {
		LOGC(DDEV, FATAL) << "LMS antenna setting failed";
		goto out_close;
	}

	/* Set up per-channel Rx timestamp based Ring buffers */
	for (size_t i = 0; i < rx_buffers.size(); i++)
		rx_buffers[i] = new smpl_buf(SAMPLE_BUF_SZ / sizeof(uint32_t));

	return NORMAL;

out_close:
	LOGC(DDEV, FATAL) << "Error in LMS open, closing: " << LMS_GetLastErrorMessage();
	LMS_Close(m_lms_dev);
	m_lms_dev = NULL;
	return -1;
}

bool LMSDevice::start()
{
	LOGC(DDEV, INFO) << "starting LMS...";

	unsigned int i;

	if (started) {
		LOGC(DDEV, ERR) << "Device already started";
		return false;
	}

	/* configure the channels/streams */
	for (i=0; i<chans; i++) {
		/* Set gains for calibration/filter setup */
		/* TX gain to maximum */
		setTxGain(maxTxGain(), i);
		/* RX gain to midpoint */
		setRxGain((minRxGain() + maxRxGain()) / 2, i);

		/* set up Rx and Tx filters */
		if (!do_filters(i))
			return false;
		/* Perform Rx and Tx calibration */
		if (!do_calib(i))
			return false;

		/* configure Streams */
		m_lms_stream_rx[i] = {};
		m_lms_stream_rx[i].isTx = false;
		m_lms_stream_rx[i].channel = i;
		m_lms_stream_rx[i].fifoSize = 1024 * 1024;
		m_lms_stream_rx[i].throughputVsLatency = 0.3;
		m_lms_stream_rx[i].dataFmt = lms_stream_t::LMS_FMT_I16;

		m_lms_stream_tx[i] = {};
		m_lms_stream_tx[i].isTx = true;
		m_lms_stream_tx[i].channel = i;
		m_lms_stream_tx[i].fifoSize = 1024 * 1024;
		m_lms_stream_tx[i].throughputVsLatency = 0.3;
		m_lms_stream_tx[i].dataFmt = lms_stream_t::LMS_FMT_I16;

		if (LMS_SetupStream(m_lms_dev, &m_lms_stream_rx[i]) < 0)
			return false;

		if (LMS_SetupStream(m_lms_dev, &m_lms_stream_tx[i]) < 0)
			return false;
	}

	/* now start the streams in a second loop, as we can no longer call
	 * LMS_SetupStream() after LMS_StartStream() of the first stream */
	for (i = 0; i < chans; i++) {
		if (LMS_StartStream(&m_lms_stream_rx[i]) < 0)
			return false;

		if (LMS_StartStream(&m_lms_stream_tx[i]) < 0)
			return false;
	}

	flush_recv(10);

	started = true;
	return true;
}

bool LMSDevice::stop()
{
	unsigned int i;

	if (!started)
		return true;

	for (i=0; i<chans; i++) {
		LMS_StopStream(&m_lms_stream_tx[i]);
		LMS_StopStream(&m_lms_stream_rx[i]);
	}

	for (i=0; i<chans; i++) {
		LMS_DestroyStream(m_lms_dev, &m_lms_stream_tx[i]);
		LMS_DestroyStream(m_lms_dev, &m_lms_stream_rx[i]);
	}

	started = false;
	return true;
}

/* do rx/tx calibration - depends on gain, freq and bw */
bool LMSDevice::do_calib(size_t chan)
{
	LOGCHAN(chan, DDEV, INFO) << "Calibrating";
	if (LMS_Calibrate(m_lms_dev, LMS_CH_RX, chan, LMS_CALIBRATE_BW_HZ, 0) < 0)
		return false;
	if (LMS_Calibrate(m_lms_dev, LMS_CH_TX, chan, LMS_CALIBRATE_BW_HZ, 0) < 0)
		return false;
	return true;
}

/* do rx/tx filter config - depends on bw only? */
bool LMSDevice::do_filters(size_t chan)
{
	lms_range_t range_lpfbw_rx, range_lpfbw_tx;
	float_type lpfbw_rx, lpfbw_tx;

	LOGCHAN(chan, DDEV, INFO) << "Setting filters";
	if (LMS_GetLPFBWRange(m_lms_dev, LMS_CH_RX, &range_lpfbw_rx))
		return false;
	print_range("LPFBWRange Rx", &range_lpfbw_rx);
	if (LMS_GetLPFBWRange(m_lms_dev, LMS_CH_RX, &range_lpfbw_tx))
		return false;
	print_range("LPFBWRange Tx", &range_lpfbw_tx);

	lpfbw_rx = OSMO_MIN(OSMO_MAX(1.4001e6, range_lpfbw_rx.min), range_lpfbw_rx.max);
	lpfbw_tx = OSMO_MIN(OSMO_MAX(5.2e6, range_lpfbw_tx.min), range_lpfbw_tx.max);

	LOGCHAN(chan, DDEV, INFO) << "LPFBW: Rx=" << lpfbw_rx << " Tx=" << lpfbw_tx;

	LOGCHAN(chan, DDEV, INFO) << "Setting LPFBW";
	if (LMS_SetLPFBW(m_lms_dev, LMS_CH_RX, chan, lpfbw_rx) < 0)
		return false;
	if (LMS_SetLPFBW(m_lms_dev, LMS_CH_TX, chan, lpfbw_tx) < 0)
		return false;
	return true;
}


double LMSDevice::maxTxGain()
{
	return maxTxGainClamp;
}

double LMSDevice::minTxGain()
{
	return 0.0;
}

double LMSDevice::maxRxGain()
{
	return 73.0;
}

double LMSDevice::minRxGain()
{
	return 0.0;
}

double LMSDevice::setTxGain(double dB, size_t chan)
{
	if (dB > maxTxGain())
		dB = maxTxGain();
	if (dB < minTxGain())
		dB = minTxGain();

	LOGCHAN(chan, DDEV, NOTICE) << "Setting TX gain to " << dB << " dB";

	if (LMS_SetGaindB(m_lms_dev, LMS_CH_TX, chan, dB) < 0)
		LOGCHAN(chan, DDEV, ERR) << "Error setting TX gain to " << dB << " dB";
	else
		tx_gains[chan] = dB;
	return tx_gains[chan];
}

double LMSDevice::setRxGain(double dB, size_t chan)
{
	if (dB > maxRxGain())
		dB = maxRxGain();
	if (dB < minRxGain())
		dB = minRxGain();

	LOGCHAN(chan, DDEV, NOTICE) << "Setting RX gain to " << dB << " dB";

	if (LMS_SetGaindB(m_lms_dev, LMS_CH_RX, chan, dB) < 0)
		LOGCHAN(chan, DDEV, ERR) << "Error setting RX gain to " << dB << " dB";
	else
		rx_gains[chan] = dB;
	return rx_gains[chan];
}

void LMSDevice::log_ant_list(bool dir_tx, size_t chan, std::ostringstream& os)
{
	lms_name_t name_list[MAX_ANTENNA_LIST_SIZE]; /* large enough list for antenna names. */
	int num_names;
	int i;

	num_names = LMS_GetAntennaList(m_lms_dev, dir_tx, chan, name_list);
	for (i = 0; i < num_names; i++) {
		if (i)
			os << ", ";
		os << "'" << name_list[i] << "'";
	}
}

int LMSDevice::get_ant_idx(const std::string & name, bool dir_tx, size_t chan)
{
	lms_name_t name_list[MAX_ANTENNA_LIST_SIZE]; /* large enough list for antenna names. */
	const char* c_name = name.c_str();
	int num_names;
	int i;

	num_names = LMS_GetAntennaList(m_lms_dev, dir_tx, chan, name_list);
	for (i = 0; i < num_names; i++) {
		if (!strcmp(c_name, name_list[i]))
			return i;
	}
	return -1;
}

bool LMSDevice::flush_recv(size_t num_pkts)
{
	#define CHUNK 625
	int len = CHUNK * tx_sps;
	short *buffer = (short*) alloca(sizeof(short) * len * 2);
	int rc;
	lms_stream_meta_t rx_metadata = {};
	rx_metadata.flushPartialPacket = false;
	rx_metadata.waitForTimestamp = false;

	ts_initial = 0;

	while (!ts_initial || (num_pkts-- > 0)) {
		rc = LMS_RecvStream(&m_lms_stream_rx[0], &buffer[0], len, &rx_metadata, 100);
		LOGC(DDEV, DEBUG) << "Flush: Recv buffer of len " << rc << " at " << std::hex << rx_metadata.timestamp;
		if (rc != len) {
			LOGC(DDEV, ERROR) << "Flush: Device receive timed out";
			return false;
		}

		ts_initial = rx_metadata.timestamp + len;
	}

	LOGC(DDEV, INFO) << "Initial timestamp " << ts_initial << std::endl;
	return true;
}

bool LMSDevice::setRxAntenna(const std::string & ant, size_t chan)
{
	int idx;

	if (chan >= rx_paths.size()) {
		LOGC(DDEV, ERROR) << "Requested non-existent channel " << chan;
		return false;
	}

	idx = get_ant_idx(ant, LMS_CH_RX, chan);
	if (idx < 0) {
		std::ostringstream os;
		LOGCHAN(chan, DDEV, ERROR) << "Invalid Rx Antenna: " << ant;
		log_ant_list(LMS_CH_RX, chan, os);
		LOGCHAN(chan, DDEV, NOTICE) << "Available Rx Antennas: " << os;
		return false;
	}

	if (LMS_SetAntenna(m_lms_dev, LMS_CH_RX, chan, idx) < 0) {
		LOGCHAN(chan, DDEV, ERROR) << "Unable to set Rx Antenna";
	}

	return true;
}

std::string LMSDevice::getRxAntenna(size_t chan)
{
	lms_name_t name_list[MAX_ANTENNA_LIST_SIZE]; /* large enough list for antenna names. */
	int idx;

	if (chan >= rx_paths.size()) {
		LOGC(DDEV, ERROR) << "Requested non-existent channel " << chan;
		return "";
	}

	idx = LMS_GetAntenna(m_lms_dev, LMS_CH_RX, chan);
	if (idx < 0) {
		LOGCHAN(chan, DDEV, ERROR) << "Error getting Rx Antenna";
		return "";
	}

	if (LMS_GetAntennaList(m_lms_dev, LMS_CH_RX, chan, name_list) < idx) {
		LOGCHAN(chan, DDEV, ERROR) << "Error getting Rx Antenna List";
		return "";
	}

	return name_list[idx];
}

bool LMSDevice::setTxAntenna(const std::string & ant, size_t chan)
{
	int idx;

	if (chan >= tx_paths.size()) {
		LOGC(DDEV, ERROR) << "Requested non-existent channel " << chan;
		return false;
	}

	idx = get_ant_idx(ant, LMS_CH_TX, chan);
	if (idx < 0) {
		std::ostringstream os;
		LOGCHAN(chan, DDEV, ERROR) << "Invalid Tx Antenna: " << ant;
		log_ant_list(LMS_CH_TX, chan, os);
		LOGCHAN(chan, DDEV, NOTICE) << "Available Tx Antennas: " << os;
		return false;
	}

	if (LMS_SetAntenna(m_lms_dev, LMS_CH_TX, chan, idx) < 0) {
		LOGCHAN(chan, DDEV, ERROR) << "Unable to set Rx Antenna";
	}

	return true;
}

std::string LMSDevice::getTxAntenna(size_t chan)
{
	lms_name_t name_list[MAX_ANTENNA_LIST_SIZE]; /* large enough list for antenna names. */
	int idx;

	if (chan >= tx_paths.size()) {
		LOGC(DDEV, ERROR) << "Requested non-existent channel " << chan;
		return "";
	}

	idx = LMS_GetAntenna(m_lms_dev, LMS_CH_TX, chan);
	if (idx < 0) {
		LOGCHAN(chan, DDEV, ERROR) << "Error getting Tx Antenna";
		return "";
	}

	if (LMS_GetAntennaList(m_lms_dev, LMS_CH_TX, chan, name_list) < idx) {
		LOGCHAN(chan, DDEV, ERROR) << "Error getting Tx Antenna List";
		return "";
	}

	return name_list[idx];
}

bool LMSDevice::requiresRadioAlign()
{
	return false;
}

GSM::Time LMSDevice::minLatency() {
	/* UNUSED on limesdr (only used on usrp1/2) */
	return GSM::Time(0,0);
}
/*!
 * Issue tracking description of several events: https://github.com/myriadrf/LimeSuite/issues/265
 */
void LMSDevice::update_stream_stats_rx(size_t chan, bool *overrun)
{
	lms_stream_status_t status;
	bool changed = false;

	if (LMS_GetStreamStatus(&m_lms_stream_rx[chan], &status) != 0) {
		LOGCHAN(chan, DDEV, ERROR) << "Rx LMS_GetStreamStatus failed";
		return;
	}

	/* FIFO overrun is counted when Rx FIFO is full but new data comes from
	   the board and oldest samples in FIFO are overwritte. Value count
	   since the last call to LMS_GetStreamStatus(stream). */
	if (status.overrun) {
		changed = true;
		*overrun = true;
		LOGCHAN(chan, DDEV, ERROR) << "Rx Overrun! ("
					   << m_ctr[chan].rx_overruns << " -> "
					   << status.overrun << ")";
	}
	m_ctr[chan].rx_overruns += status.overrun;

	/* Dropped packets in Rx are counted when gaps in Rx timestamps are
	   detected (likely because buffer overflow in hardware). Value count
	   since the last call to LMS_GetStreamStatus(stream). */
	if (status.droppedPackets) {
		changed = true;
		LOGCHAN(chan, DDEV, ERROR) << "Rx Dropped packets by HW! ("
					   << m_ctr[chan].rx_dropped_samples << " -> "
					   << m_ctr[chan].rx_dropped_samples +
					      status.droppedPackets
					   << ")";
		m_ctr[chan].rx_dropped_events++;
	}
	m_ctr[chan].rx_dropped_samples += status.droppedPackets;

	if (changed)
		osmo_signal_dispatch(SS_DEVICE, S_DEVICE_COUNTER_CHANGE, &m_ctr[chan]);

}

// NOTE: Assumes sequential reads
int LMSDevice::readSamples(std::vector < short *>&bufs, int len, bool * overrun,
			   TIMESTAMP timestamp, bool * underrun, unsigned *RSSI)
{
	int rc, num_smpls, expect_smpls;
	ssize_t avail_smpls;
	TIMESTAMP expect_timestamp;
	unsigned int i;
	lms_stream_meta_t rx_metadata = {};
	rx_metadata.flushPartialPacket = false;
	rx_metadata.waitForTimestamp = false;
	rx_metadata.timestamp = 0;

	if (bufs.size() != chans) {
		LOGC(DDEV, ERROR) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	*overrun = false;
	*underrun = false;

	/* Check that timestamp is valid */
	rc = rx_buffers[0]->avail_smpls(timestamp);
	if (rc < 0) {
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_code(rc);
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_status(timestamp);
		return 0;
	}

	for (i = 0; i<chans; i++) {
		/* Receive samples from HW until we have enough */
		while ((avail_smpls = rx_buffers[i]->avail_smpls(timestamp)) < len) {
			thread_enable_cancel(false);
			num_smpls = LMS_RecvStream(&m_lms_stream_rx[i], bufs[i], len - avail_smpls, &rx_metadata, 100);
			update_stream_stats_rx(i, overrun);
			thread_enable_cancel(true);
			if (num_smpls <= 0) {
				LOGCHAN(i, DDEV, ERROR) << "Device receive timed out (" << rc << " vs exp " << len << ").";
				return -1;
			}

			LOGCHAN(i, DDEV, DEBUG) "Received timestamp = " << (TIMESTAMP)rx_metadata.timestamp << " (" << num_smpls << ")";

			expect_smpls = len - avail_smpls;
			if (expect_smpls != num_smpls)
				LOGCHAN(i, DDEV, NOTICE) << "Unexpected recv buffer len: expect "
							 << expect_smpls << " got " << num_smpls
							 << ", diff=" << expect_smpls - num_smpls;

			expect_timestamp = timestamp + avail_smpls;
			if (expect_timestamp != (TIMESTAMP)rx_metadata.timestamp)
				LOGCHAN(i, DDEV, ERROR) << "Unexpected recv buffer timestamp: expect "
							<< expect_timestamp << " got " << (TIMESTAMP)rx_metadata.timestamp
							<< ", diff=" << rx_metadata.timestamp - expect_timestamp;

			rc = rx_buffers[i]->write(bufs[i], num_smpls, (TIMESTAMP)rx_metadata.timestamp);
			if (rc < 0) {
				LOGCHAN(i, DDEV, ERROR) << rx_buffers[i]->str_code(rc);
				LOGCHAN(i, DDEV, ERROR) << rx_buffers[i]->str_status(timestamp);
				if (rc != smpl_buf::ERROR_OVERFLOW)
					return 0;
			}
		}
	}

	/* We have enough samples */
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

void LMSDevice::update_stream_stats_tx(size_t chan, bool *underrun)
{
	lms_stream_status_t status;
	bool changed = false;

	if (LMS_GetStreamStatus(&m_lms_stream_tx[chan], &status) != 0) {
		LOGCHAN(chan, DDEV, ERROR) << "Tx LMS_GetStreamStatus failed";
		return;
	}

	/* FIFO underrun is counted when Tx is running but FIFO is empty for
	   >100 ms (500ms in older versions). Value count since the last call to
	   LMS_GetStreamStatus(stream). */
	if (status.underrun) {
		changed = true;
		*underrun = true;
		LOGCHAN(chan, DDEV, ERROR) << "Tx Underrun! ("
					   << m_ctr[chan].tx_underruns << " -> "
					   << status.underrun << ")";
	}
	m_ctr[chan].tx_underruns += status.underrun;

	/* Dropped packets in Tx are counted only when timestamps are enabled
	   and SDR drops packet because of late timestamp. Value count since the
	   last call to LMS_GetStreamStatus(stream). */
	if (status.droppedPackets) {
		changed = true;
		LOGCHAN(chan, DDEV, ERROR) << "Tx Dropped packets by HW! ("
					   << m_ctr[chan].tx_dropped_samples << " -> "
					   << m_ctr[chan].tx_dropped_samples +
					      status.droppedPackets
					   << ")";
		m_ctr[chan].tx_dropped_events++;
	}
	m_ctr[chan].tx_dropped_samples += status.droppedPackets;

	if (changed)
		osmo_signal_dispatch(SS_DEVICE, S_DEVICE_COUNTER_CHANGE, &m_ctr[chan]);

}

int LMSDevice::writeSamples(std::vector < short *>&bufs, int len,
			    bool * underrun, unsigned long long timestamp,
			    bool isControl)
{
	int rc = 0;
	unsigned int i;
	lms_stream_meta_t tx_metadata = {};
	tx_metadata.flushPartialPacket = false;
	tx_metadata.waitForTimestamp = true;
	tx_metadata.timestamp = timestamp - ts_offset;	/* Shift Tx time by offset */

	if (isControl) {
		LOGC(DDEV, ERROR) << "Control packets not supported";
		return 0;
	}

	if (bufs.size() != chans) {
		LOGC(DDEV, ERROR) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	*underrun = false;

	for (i = 0; i<chans; i++) {
		LOGCHAN(i, DDEV, DEBUG) << "send buffer of len " << len << " timestamp " << std::hex << tx_metadata.timestamp;
		thread_enable_cancel(false);
		rc = LMS_SendStream(&m_lms_stream_tx[i], bufs[i], len, &tx_metadata, 100);
		update_stream_stats_tx(i, underrun);
		thread_enable_cancel(true);
		if (rc != len) {
			LOGCHAN(i, DDEV, ERROR) << "LMS: Device Tx timed out (" << rc << " vs exp " << len << ").";
			return -1;
		}
	}

	return rc;
}

bool LMSDevice::updateAlignment(TIMESTAMP timestamp)
{
	return true;
}

bool LMSDevice::setTxFreq(double wFreq, size_t chan)
{
	LOGCHAN(chan, DDEV, NOTICE) << "Setting Tx Freq to " << wFreq << " Hz";

	if (LMS_SetLOFrequency(m_lms_dev, LMS_CH_TX, chan, wFreq) < 0) {
		LOGCHAN(chan, DDEV, ERROR) << "Error setting Tx Freq to " << wFreq << " Hz";
		return false;
	}

	return true;
}

bool LMSDevice::setRxFreq(double wFreq, size_t chan)
{
	LOGCHAN(chan, DDEV, NOTICE) << "Setting Rx Freq to " << wFreq << " Hz";

	if (LMS_SetLOFrequency(m_lms_dev, LMS_CH_RX, chan, wFreq) < 0) {
		LOGCHAN(chan, DDEV, ERROR) << "Error setting Rx Freq to " << wFreq << " Hz";
		return false;
	}

	return true;
}

RadioDevice *RadioDevice::make(size_t tx_sps, size_t rx_sps,
			       InterfaceType iface, size_t chans, double lo_offset,
			       const std::vector < std::string > &tx_paths,
			       const std::vector < std::string > &rx_paths)
{
	if (tx_sps != rx_sps) {
		LOGC(DDEV, ERROR) << "LMS Requires tx_sps == rx_sps";
		return NULL;
	}
	if (lo_offset != 0.0) {
		LOGC(DDEV, ERROR) << "LMS doesn't support lo_offset";
		return NULL;
	}
	return new LMSDevice(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths);
}
