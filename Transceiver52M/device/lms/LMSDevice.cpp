/*
* Copyright 2018 sysmocom - s.f.m.c. GmbH
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

#include <lime/LimeSuite.h>

#include <osmocom/core/utils.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

using namespace std;

constexpr double LMSDevice::masterClockRate;

#define MAX_ANTENNA_LIST_SIZE 10
#define LMS_SAMPLE_RATE GSMRATE*32
#define GSM_CARRIER_BW 270000.0 /* 270kHz */
#define LMS_MIN_BW_SUPPORTED 2.5e6 /* 2.5mHz, minimum supported by LMS */
#define LMS_CALIBRATE_BW_HZ OSMO_MAX(GSM_CARRIER_BW, LMS_MIN_BW_SUPPORTED)

LMSDevice::LMSDevice(size_t sps, size_t chans):
	m_lms_dev(NULL), sps(sps), chans(chans)
{
	LOG(INFO) << "creating LMS device...";

	m_lms_stream_rx.resize(chans);
	m_lms_stream_tx.resize(chans);

	m_last_rx_underruns.resize(chans, 0);
	m_last_rx_overruns.resize(chans, 0);
	m_last_tx_underruns.resize(chans, 0);
	m_last_tx_overruns.resize(chans, 0);
}

static void lms_log_callback(int lvl, const char *msg)
{
	/* map lime specific log levels */
	static const int lvl_map[5] = {
		[0] = LOGL_FATAL,
		[1] = LOGL_ERROR,
		[2] = LOGL_NOTICE,
		[3] = LOGL_INFO,
		[4] = LOGL_DEBUG,
	};
	/* protect against future higher log level values (lower importance) */
	if ((unsigned int) lvl >= ARRAY_SIZE(lvl_map))
		lvl = ARRAY_SIZE(lvl_map)-1;

	LOGLV(DLMS, lvl) << msg;
}

static void thread_enable_cancel(bool cancel)
{
	cancel ? pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL) :
		 pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
}

int LMSDevice::open(const std::string &args, int ref, bool swap_channels)
{
	//lms_info_str_t dev_str;
	lms_info_str_t* info_list;
	uint16_t dac_val;
	unsigned int i, n;
	int rc;

	LOG(INFO) << "Opening LMS device..";

	LMS_RegisterLogHandler(&lms_log_callback);

        if ((n = LMS_GetDeviceList(NULL)) < 0)
            LOG(ERROR) << "LMS_GetDeviceList(NULL) failed";
        LOG(DEBUG) << "Devices found: " << n;
        if (n < 1)
            return -1;

        info_list = new lms_info_str_t[n];

        if (LMS_GetDeviceList(info_list) < 0)                //Populate device list
            LOG(ERROR) << "LMS_GetDeviceList(info_list) failed";

        for (i = 0; i < n; i++)                     //print device list
           LOG(DEBUG) << "Device [" << i << "]: " << info_list[i];

	rc = LMS_Open(&m_lms_dev, info_list[0], NULL);
	if (rc != 0) {
		LOG(ERROR) << "LMS_GetDeviceList() failed)";
		delete [] info_list;
		return -1;
	}

	delete [] info_list;

	LOG(INFO) << "Init LMS device";
	if (LMS_Init(m_lms_dev) != 0) {
		LOG(ERROR) << "LMS_Init() failed";
		return -1;
	}

	lms_range_t range;
	if (LMS_GetSampleRateRange(m_lms_dev, LMS_CH_RX, &range))
		goto out_close;
	LOG(DEBUG) << "Sample Rate: Min=" << range.min << " Max=" << range.max << " Step=" << range.step;

	LOG(DEBUG) << "Setting sample rate to " << GSMRATE*sps << " " << sps;
	if (LMS_SetSampleRate(m_lms_dev, GSMRATE*sps, 32) < 0)
		goto out_close;
	float_type sr_host, sr_rf;
	if (LMS_GetSampleRate(m_lms_dev, LMS_CH_RX, 0, &sr_host, &sr_rf))
		goto out_close;
	LOG(DEBUG) << "Sample Rate: Host=" << sr_host << " RF=" << sr_rf;
	/* FIXME: make this device/model dependent, like UHDDevice:dev_param_map! */
	ts_offset = static_cast<TIMESTAMP>(8.9e-5 * GSMRATE);

	switch (ref) {
	case REF_INTERNAL:
		LOG(DEBUG) << "Setting Internal clock reference";
		/* Ugly API: Selecting clock source implicit by writing to VCTCXO DAC ?!? */
		if (LMS_VCTCXORead(m_lms_dev, &dac_val) < 0)
			goto out_close;
		LOG(DEBUG) << "Setting VCTCXO to " << dac_val;
		if (LMS_VCTCXOWrite(m_lms_dev, dac_val) < 0)
			goto out_close;
		break;
	case REF_EXTERNAL:
		LOG(DEBUG) << "Setting Internal clock reference to " << 10000000.0;
		/* Assume an external 10 MHz reference clock */
		if (LMS_SetClockFreq(m_lms_dev, LMS_CLOCK_EXTREF, 10000000.0) < 0)
			goto out_close;
		break;
	default:
		LOG(ALERT) << "Invalid reference type";
		goto out_close;
	}

	/* Perform Rx and Tx calibration */
	for (i=0; i<chans; i++) {
		LOG(INFO) << "Calibrating chan " << i;
		if (LMS_Calibrate(m_lms_dev, LMS_CH_RX, i, LMS_CALIBRATE_BW_HZ, 0) < 0)
			goto out_close;
		if (LMS_Calibrate(m_lms_dev, LMS_CH_TX, i, LMS_CALIBRATE_BW_HZ, 0) < 0)
			goto out_close;
	}

	samplesRead = 0;
	samplesWritten = 0;
	started = false;

	return NORMAL;

out_close:
	LOG(ALERT) << "Error in LMS open, closing: " << LMS_GetLastErrorMessage();
	LMS_Close(m_lms_dev);
	return -1;
}

bool LMSDevice::start()
{
	LOG(INFO) << "starting LMS...";

	unsigned int i;

	for (i=0; i<chans; i++) {
		if (LMS_EnableChannel(m_lms_dev, LMS_CH_RX, i, true) < 0)
			return false;

		if (LMS_EnableChannel(m_lms_dev, LMS_CH_TX, i, true) < 0)
			return false;

		// Set gains to midpoint
		setTxGain((minTxGain() + maxTxGain()) / 2, i);
		setRxGain((minRxGain() + maxRxGain()) / 2, i);

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

		LMS_EnableChannel(m_lms_dev, LMS_CH_RX, i, false);
		LMS_EnableChannel(m_lms_dev, LMS_CH_TX, i, false);
	}

	return true;
}

double LMSDevice::maxTxGain()
{
	return 60.0;
}

double LMSDevice::minTxGain()
{
	return 0.0;
}

double LMSDevice::maxRxGain()
{
	return 70.0;
}

double LMSDevice::minRxGain()
{
	return 0.0;
}

double LMSDevice::setTxGain(double dB, size_t chan)
{
	if (chan) {
		LOG(ALERT) << "Invalid channel " << chan;
		return 0.0;
	}

	if (dB > maxTxGain())
		dB = maxTxGain();
	if (dB < minTxGain())
		dB = minTxGain();

	LOG(NOTICE) << "Setting TX gain to " << dB << " dB.";

	if (LMS_SetGaindB(m_lms_dev, LMS_CH_TX, chan, dB) < 0)
		LOG(ERR) << "Error setting TX gain";

	return dB;
}

double LMSDevice::setRxGain(double dB, size_t chan)
{
	if (chan) {
		LOG(ALERT) << "Invalid channel " << chan;
		return 0.0;
	}

	dB = 47.0;

	if (dB > maxRxGain())
		dB = maxRxGain();
	if (dB < minRxGain())
		dB = minRxGain();

	LOG(NOTICE) << "Setting RX gain to " << dB << " dB.";

	if (LMS_SetGaindB(m_lms_dev, LMS_CH_RX, chan, dB) < 0)
		LOG(ERR) << "Error setting RX gain";

	return dB;
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
	int len = CHUNK * sps;
	short *buffer = new short[len * 2];
	int rc;
	lms_stream_meta_t rx_metadata = {};
	rx_metadata.flushPartialPacket = false;
	rx_metadata.waitForTimestamp = false;

	ts_initial = 0;

	while (!ts_initial || (num_pkts-- > 0)) {
		rc = LMS_RecvStream(&m_lms_stream_rx[0], &buffer[0], len, &rx_metadata, 100);
		LOG(DEBUG) << "Flush: Recv buffer of len " << rc << " at " << std::hex << rx_metadata.timestamp;
		if (rc != len) {
			LOG(ALERT) << "LMS: Device receive timed out";
			delete[] buffer;
			return false;
		}

		ts_initial = rx_metadata.timestamp;
	}

	LOG(INFO) << "Initial timestamp " << ts_initial << std::endl;
	delete[] buffer;
	return true;
}

bool LMSDevice::setRxAntenna(const std::string & ant, size_t chan)
{
	int idx;

	if (chan >= rx_paths.size()) {
		LOG(ALERT) << "Requested non-existent channel " << chan;
		return false;
	}

	idx = get_ant_idx(ant, LMS_CH_RX, chan);
	if (idx < 0) {
		LOG(ALERT) << "Invalid Rx Antenna";
		return false;
	}

	if (LMS_SetAntenna(m_lms_dev, LMS_CH_RX, chan, idx) < 0) {
		LOG(ALERT) << "Unable to set Rx Antenna";
	}

	return true;
}

std::string LMSDevice::getRxAntenna(size_t chan)
{
	lms_name_t name_list[MAX_ANTENNA_LIST_SIZE]; /* large enough list for antenna names. */
	int idx;

	if (chan >= rx_paths.size()) {
		LOG(ALERT) << "Requested non-existent channel " << chan;
		return "";
	}

	idx = LMS_GetAntenna(m_lms_dev, LMS_CH_RX, chan);
	if (idx < 0) {
		LOG(ALERT) << "Error getting Rx Antenna";
		return "";
	}

	if (LMS_GetAntennaList(m_lms_dev, LMS_CH_RX, chan, name_list) < idx) {
		LOG(ALERT) << "Error getting Rx Antenna List";
		return "";
	}

	return name_list[idx];
}

bool LMSDevice::setTxAntenna(const std::string & ant, size_t chan)
{
	int idx;

	if (chan >= tx_paths.size()) {
		LOG(ALERT) << "Requested non-existent channel " << chan;
		return false;
	}

	idx = get_ant_idx(ant, LMS_CH_TX, chan);
	if (idx < 0) {
		LOG(ALERT) << "Invalid Rx Antenna";
		return false;
	}

	if (LMS_SetAntenna(m_lms_dev, LMS_CH_TX, chan, idx) < 0) {
		LOG(ALERT) << "Unable to set Rx Antenna";
	}

	return true;
}

std::string LMSDevice::getTxAntenna(size_t chan)
{
	lms_name_t name_list[MAX_ANTENNA_LIST_SIZE]; /* large enough list for antenna names. */
	int idx;

	if (chan >= tx_paths.size()) {
		LOG(ALERT) << "Requested non-existent channel " << chan;
		return "";
	}

	idx = LMS_GetAntenna(m_lms_dev, LMS_CH_TX, chan);
	if (idx < 0) {
		LOG(ALERT) << "Error getting Tx Antenna";
		return "";
	}

	if (LMS_GetAntennaList(m_lms_dev, LMS_CH_TX, chan, name_list) < idx) {
		LOG(ALERT) << "Error getting Tx Antenna List";
		return "";
	}

	return name_list[idx];
}

bool LMSDevice::requiresRadioAlign()
{
	return false;
}

GSM::Time LMSDevice::minLatency() {
	/* Empirical data from a handful of
	relatively recent machines shows that the B100 will underrun when
	the transmit threshold is reduced to a time of 6 and a half frames,
	so we set a minimum 7 frame threshold. */
	return GSM::Time(6,7);
}

// NOTE: Assumes sequential reads
int LMSDevice::readSamples(std::vector < short *>&bufs, int len, bool * overrun,
			   TIMESTAMP timestamp, bool * underrun, unsigned *RSSI)
{
	int rc = 0;
	unsigned int i;
	lms_stream_status_t status;
	lms_stream_meta_t rx_metadata = {};
	rx_metadata.flushPartialPacket = false;
	rx_metadata.waitForTimestamp = false;
	/* Shift read time with respect to transmit clock */
	timestamp += ts_offset;
	rx_metadata.timestamp = 0;

	if (bufs.size() != chans) {
		LOG(ALERT) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	*overrun = false;
	*underrun = false;
	for (i = 0; i<chans; i++) {
		thread_enable_cancel(false);
		rc = LMS_RecvStream(&m_lms_stream_rx[i], bufs[i], len, &rx_metadata, 100);
		LOG(ALERT) << "chan "<< i << " recv buffer of len " << rc << " expect " << std::hex << timestamp << " got " << std::hex << (TIMESTAMP)rx_metadata.timestamp << " (" << std::hex << rx_metadata.timestamp <<") diff=" << rx_metadata.timestamp - timestamp;
		if (rc != len) {
			LOG(ALERT) << "LMS: Device receive timed out";
		}

		if (LMS_GetStreamStatus(&m_lms_stream_rx[i], &status) == 0) {
			if (status.underrun > m_last_rx_underruns[i])
				*underrun = true;
			m_last_rx_underruns[i] = status.underrun;

			if (status.overrun > m_last_rx_overruns[i])
				*overrun = true;
			m_last_rx_overruns[i] = status.overrun;
		}
		thread_enable_cancel(true);
	}

	samplesRead += rc;

	if (((TIMESTAMP) rx_metadata.timestamp) < timestamp)
		rc = 0;

	return rc;
}

int LMSDevice::writeSamples(std::vector < short *>&bufs, int len,
			    bool * underrun, unsigned long long timestamp,
			    bool isControl)
{
	int rc;
	unsigned int i;
	lms_stream_status_t status;
	lms_stream_meta_t tx_metadata = {};
	tx_metadata.flushPartialPacket = false;
	tx_metadata.waitForTimestamp = true;
	tx_metadata.timestamp = timestamp;

	if (isControl) {
		LOG(ERR) << "Control packets not supported";
		return 0;
	}

	if (bufs.size() != chans) {
		LOG(ALERT) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	*underrun = false;

	for (i = 0; i<chans; i++) {
		LOG(ALERT) << "chan "<< i << " send buffer of len " << len << " timestamp " << std::hex << tx_metadata.timestamp;
		thread_enable_cancel(false);
		rc = LMS_SendStream(&m_lms_stream_tx[i], bufs[i], len, &tx_metadata, 100);
		if (rc != len) {
			LOG(ALERT) << "LMS: Device send timed out";
		}

		if (LMS_GetStreamStatus(&m_lms_stream_tx[i], &status) == 0) {
			if (status.underrun > m_last_tx_underruns[i])
				*underrun = true;
			m_last_tx_underruns[i] = status.underrun;
		}
		thread_enable_cancel(true);
	}

	samplesWritten += rc;

	return rc;
}

bool LMSDevice::updateAlignment(TIMESTAMP timestamp)
{
	return true;
}

bool LMSDevice::setTxFreq(double wFreq, size_t chan)
{

	if (chan) {
		LOG(ALERT) << "Invalid channel " << chan;
		return false;
	}

	if (LMS_SetLOFrequency(m_lms_dev, LMS_CH_TX, chan, wFreq) < 0) {
		LOG(ALERT) << "set Tx: " << wFreq << " failed!";
		return false;
	}

	return true;
}

bool LMSDevice::setRxFreq(double wFreq, size_t chan)
{
	if (chan) {
		LOG(ALERT) << "Invalid channel " << chan;
		return false;
	}

	if (LMS_SetLOFrequency(m_lms_dev, LMS_CH_RX, chan, wFreq) < 0) {
		LOG(ALERT) << "set Rx: " << wFreq << " failed!";
		return false;
	}

	return true;
}

RadioDevice *RadioDevice::make(size_t tx_sps, size_t rx_sps,
			       InterfaceType iface, size_t chans, double offset,
			       const std::vector < std::string > &tx_paths,
			       const std::vector < std::string > &rx_paths)
{
	return new LMSDevice(tx_sps, chans);
}
