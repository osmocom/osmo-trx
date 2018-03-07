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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

using namespace std;

const double LMSDevice::masterClockRate = 52.0e6;

LMSDevice::LMSDevice(size_t sps)
{
	LOG(INFO) << "creating LMS device...";

	m_lms_device = NULL;
	this->sps = sps;
}

static void lms_log_callback(int lvl, const char *msg)
{
	/* map lime specific log levels */
	static const lvl_map[4] = {
		[0] = LOGL_FATAL,
		[1] = LOGL_ERROR,
		[2] = LOGL_NOTICE,
		[3] = LOGL_INFO,
		[4] = LOGL_DEBUG,
	};
	/* protect against future higher log level values (lower importance) */
	if (lvl >= ARRAY_SIZE(lvl_map))
		lvl = ARRAY_SIZE(lvl_map)-1;

	LOG(lvl) << msg;
}

int LMSDevice::open(const std::string &, int, bool)
{
	lms_info_str dev_str;
	uint16_t dac_val;

	LOG(INFO) << "opening LMS device..";

	LMS_RegisterLogHandler(&lms_log_callback);

	rc = LMS_Open(&m_lms_dev, NULL, NULL);
	if (rc != 0)
		return -1;

	if (LMS_SetSampleRate(m_lms_dev, GSMRATE, sps) < 0)
		goto out_close;
	/* FIXME: make this device/model dependent, like UHDDevice:dev_param_map! */
	ts_offset = static_caset<TIMESTAMP>(8.9e-5 * GSMRATE);

	switch (ref) {
	case REF_INTERNAL:
		/* Ugly API: Selecting clock source implicit by writing to VCTCXO DAC ?!? */
		if (LMS_VCTCXORead(m_lms_dev, &dac_val) < 0)
			goto out_close;

		if (LMS_VCTCXOWrite(m_lms_dev, dac_val) < 0)
			goto out_close;
		break;
	case REF_EXTENAL:
		/* Assume an external 10 MHz reference clock */
		if (LMS_SetClockFreq(m_lms_dev, LMS_CLOCK_EXTREF, 10000000.0) < 0)
			goto out_close;
		break;
	default:
		LOG(ALERT) << "Invalid reference type";
		goto out_close;
	}

	if (LMS_Init(m_lms_dev) < 0)
		goto out_close;

	/* Perform Rx and Tx calibration */
	if (LMS_Calibrate(m_lms_dev, LMS_CH_RX, chan, 270000.0, 0) < 0)
		goto out_close;
	if (LMS_Calibrate(m_lms_dev, LMS_CH_TX, chan, 270000.0, 0) < 0)
		goto out_close;

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

	if (LMS_EnableChannel(m_lms_dev, LMS_CH_RX, 0, true) < 0)
		return false;

	if (LMS_EnableChannel(m_lms_dev, LMS_CH_TX, 0, true) < 0)
		return false;

	// Set gains to midpoint
	setTxGain((minTxGain() + maxTxGain()) / 2);
	setRxGain((minRxGain() + maxRxGain()) / 2);

	m_lms_stream_rx = {
		.isTx = false,
		.channel = 0,
		.fifoSize = 1024 * 1024,
		.throughputVsLatency = 0.3,
		.dataFmt = LMS_FMT_I16,
	}
	m_lms_stream_tx = {
		.ixTx = true,
		.channel = 0,
		.fifoSize = 1024 * 1024,
		.throughputVsLatency = 0.3,
		.dataFmt = LMS_FMT_I16,
	}

	if (LMS_SetupStream(m_lms_dev, &m_lms_stream_rx) < 0)
		return false;

	if (LMS_SetupStream(m_lms_dev, &m_lms_stream_tx) < 0)
		return false;

	if (LMS_StartStream(&m_lms_stream_rx) < 0)
		return false;

	if (LMS_StartStream(&m_lms_stream_tx) < 0)
		return false;

	started = true;
	return true;
}

bool LMSDevice::stop()
{
	if (!started)
		return true;

	LMS_StopStream(&m_lms_stream_tx);
	LMS_StopStream(&m_lms_stream_rx);

	LMS_EnableChannel(m_lms_dev, LMS_CH_RX, 0, false);
	LMS_EnableChannel(m_lms_dev, LMS_CH_TX, 0, false);

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

int get_ant_idx(const char *name, bool dir_tx)
{
	lms_name_t name_list;
	int num_names;
	num_names = LMS_GetAntennaList(m_lms_dev, dir_tx, &name_list);
	for (i = 0; i < num_names; i++) {
		if (!strcmp(name, name_list[i]))
			return i;
	}
	return -1;
}

bool LMSDevice::setRxAntenna(const std::string & ant, size_t chan)
{
	int idx;

	if (chan >= rx_paths.size()) {
		LOG(ALERT) << "Requested non-existent channel " << chan;
		return false;
	}

	idx = get_ant_idx(ant, LMS_CH_RX);
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
	if (chan >= rx_paths.size()) {
		LOG(ALERT) << "Requested non-existent channel " << chan;
		return "";
	}

	idx = LMS_GetAntenna(m_lms_dev, LMS_CH_RX, chan);
	if (idx < 0) {
		LOG(ALERT) << "Error getting Rx Antenna";
		return "";
	}

	if (LMS_GetAntennaList(m_lms_dev, LMS_CH_RX, chan, &list) < idx) {
		LOG(ALERT) << "Error getting Rx Antenna List";
		return "";
	}

	return list[idx];
}

bool LMSDevice::setTxAntenna(const std::string & ant, size_t chan)
{
	int idx;

	if (chan >= tx_paths.size()) {
		LOG(ALERT) << "Requested non-existent channel " << chan;
		return false;
	}

	idx = get_ant_idx(ant, LMS_CH_TX);
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

	if (LMS_GetAntennaList(m_lms_dev, LMS_CH_TX, chan, &list) < idx) {
		LOG(ALERT) << "Error getting Tx Antenna List";
		return "";
	}

	return list[idx];
}

// NOTE: Assumes sequential reads
int LMSDevice::readSamples(std::vector < short *>&bufs, int len, bool * overrun,
			   TIMESTAMP timestamp, bool * underrun, unsigned *RSSI)
{
	lms_stream_meta_t rx_metadata = {
		.flushPartialPacket = false,
		.waitForTimestamp = false,
	};
	int rc;

	if (bufs.size != 1) {
		LOG(ALERT) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	/* Shift read time with respect to transmit clock */
	timestamp += ts_offset;

	rc = LMS_RecvStream(&m_lms_stream_rx, bufs[0], len, &rx_metadata, 100);

	*overrun = false;
	*underrun = false;

	if (LMS_GetStreamStatus(&m_lms_stream_rx, &status) == 0) {
		if (status.underrun > m_last_rx_underruns)
			*underrun = true;
		m_last_rx_underruns = status.underrun;

		if (status.overrun > m_last_rx_overruns)
			*overrun = true;
		m_last_rx_overruns = status.overrun;
	}

	samplesRead += rc;

	return rc;
}

int LMSDevice::writeSamples(std::vector < short *>&bufs, int len,
			    bool * underrun, unsigned long long timestamp,
			    bool isControl)
{
	lms_stream_status_t status;
	lms_stream_meta_t tx_metadata = {
		.flushPartialPacket = false,
		.waitForTimestamp = true,
		.timestamp = timestamp,
	};
	int rc;

	if (isControl) {
		LOG(ERR) << "Control packets not supported";
		return 0;
	}

	if (bufs.size() != 1) {
		LOG(ALERT) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	rc = LMS_Send_Stream(&m_lms_stream_tx, bufs[0], len, &tx_metadata, 100);
	if (rc != len) {
		LOG(ALERT) << "LMS: Device send timed out ";
	}

	*underrun = false;

	if (LMS_GetStreamStatus(&m_lms_stream_tx, &status) == 0) {
		if (status.underrun > m_last_tx_underruns)
			*underrun = true;
		m_last_tx_underruns = status.underrun;
	}

	samplesWritten += rc;

	return rc;
}

bool LMSDevice::updateAlignment(TIMESTAMP timestamp)
{
	short data[] = { 0x00, 0x02, 0x00, 0x00 };
	uint32_t *wordPtr = (uint32_t *) data;
	*wordPtr = host_to_usrp_u32(*wordPtr);
	bool tmpUnderrun;

	std::vector < short *>buf(1, data);
	if (writeSamples(buf, 1, &tmpUnderrun, timestamp & 0x0ffffffffll, true)) {
		pingTimestamp = timestamp;
		return true;
	}
	return false;
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
	return new LMSDevice(tx_sps);
}
