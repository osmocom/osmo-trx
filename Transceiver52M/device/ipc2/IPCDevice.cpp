/*
* Copyright 2020 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
* Author: Pau Espin Pedrol <pespin@sysmocom.de>
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

#include <sys/time.h>
#include <osmocom/core/timer_compat.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "Logger.h"
#include "Threads.h"
#include "IPCDevice.h"
#include "smpl_buf.h"

#define SAMPLE_BUF_SZ (1 << 20)
static const auto ONE_BIT_DURATION ((12./5200.)/(156.25*4.));
static const auto ONE_SAMPLE_DURATION_US ((ONE_BIT_DURATION/4.)*1000*1000);
using namespace std;

IPCDevice2::IPCDevice2(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chan_num, double lo_offset,
		       const std::vector<std::string> &tx_paths, const std::vector<std::string> &rx_paths)
	: RadioDevice(tx_sps, rx_sps, iface, chan_num, lo_offset, tx_paths, rx_paths), rx_buffers(chans),
	  started(false), tx_gains(chans), rx_gains(chans)
{
	LOGC(DDEV, INFO) << "creating IPC device...";

	if (!(tx_sps == 4) || !(rx_sps == 4)) {
		LOGC(DDEV, FATAL) << "IPC shm if create failed!";
		exit(0);
	}

	/* Set up per-channel Rx timestamp based Ring buffers */
	for (size_t i = 0; i < rx_buffers.size(); i++)
		rx_buffers[i] = new smpl_buf(SAMPLE_BUF_SZ / sizeof(uint32_t));

	if (!m.create()) {
		LOGC(DDEV, FATAL) << "IPC shm if create failed!";
		exit(0);
	}
}

IPCDevice2::~IPCDevice2()
{
	LOGC(DDEV, INFO) << "Closing IPC device";
	/* disable all channels */

	for (size_t i = 0; i < rx_buffers.size(); i++)
		delete rx_buffers[i];
}

int IPCDevice2::open(const std::string &args, int ref, bool swap_channels)
{
	std::string k, v;

	/* configure antennas */
	if (!set_antennas()) {
		LOGC(DDEV, FATAL) << "IPC antenna setting failed";
		goto out_close;
	}

	return iface == MULTI_ARFCN ? MULTI_ARFCN : NORMAL;

out_close:
	LOGC(DDEV, FATAL) << "Error in IPC open, closing";
	return -1;
}

bool IPCDevice2::start()
{
	LOGC(DDEV, INFO) << "starting IPC...";

	if (started) {
		LOGC(DDEV, ERR) << "Device already started";
		return true;
	}

	int max_bufs_to_flush = 120;
	flush_recv(max_bufs_to_flush);

	started = true;
	return true;
}

bool IPCDevice2::stop()
{
	if (!started)
		return true;

	LOGC(DDEV, NOTICE) << "All channels stopped, terminating...";

	/* reset internal buffer timestamps */
	for (size_t i = 0; i < rx_buffers.size(); i++)
		rx_buffers[i]->reset();

	started = false;
	return true;
}

double IPCDevice2::maxRxGain()
{
	return 70;
}

double IPCDevice2::minRxGain()
{
	return 0;
}

int IPCDevice2::getNominalTxPower(size_t chan)
{
	return 10;
}

double IPCDevice2::setPowerAttenuation(int atten, size_t chan)
{
	return atten;
}

double IPCDevice2::getPowerAttenuation(size_t chan)
{
	return 0;
}

double IPCDevice2::setRxGain(double dB, size_t chan)
{
	if (dB > maxRxGain())
		dB = maxRxGain();
	if (dB < minRxGain())
		dB = minRxGain();

	LOGCHAN(chan, DDEV, NOTICE) << "Setting RX gain to " << dB << " dB";

	return dB;
}

bool IPCDevice2::flush_recv(size_t num_pkts)
{
	ts_initial = 10000;

	LOGC(DDEV, INFO) << "Initial timestamp " << ts_initial << std::endl;
	return true;
}

bool IPCDevice2::setRxAntenna(const std::string &ant, size_t chan)
{
	return true;
}

std::string IPCDevice2::getRxAntenna(size_t chan)
{
	return "";
}

bool IPCDevice2::setTxAntenna(const std::string &ant, size_t chan)
{
	return true;
}

std::string IPCDevice2::getTxAntenna(size_t chan)
{
	return "";
}

bool IPCDevice2::requiresRadioAlign()
{
	return false;
}

GSM::Time IPCDevice2::minLatency()
{
	/* UNUSED */
	return GSM::Time(0, 0);
}

/** Returns the starting write Timestamp*/
TIMESTAMP IPCDevice2::initialWriteTimestamp(void)
{
	return ts_initial;
}

/** Returns the starting read Timestamp*/
TIMESTAMP IPCDevice2::initialReadTimestamp(void)
{
	return ts_initial;
}

static timespec readtime, writetime;
static void wait_for_sample_time(timespec* last, unsigned int len) {
	#if 1
	timespec ts, diff;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	timespecsub(&ts, last, &diff);
	auto elapsed_us = (diff.tv_sec * 1000000) + (diff.tv_nsec / 1000);
	auto max_wait_us = ONE_SAMPLE_DURATION_US * len;
	if(elapsed_us < max_wait_us)
		usleep(max_wait_us-elapsed_us);
	*last = ts;
	#else
	usleep(ONE_SAMPLE_DURATION_US * 625);
	#endif
}

// NOTE: Assumes sequential reads
int IPCDevice2::readSamples(std::vector<short *> &bufs, int len, bool *overrun, TIMESTAMP timestamp, bool *underrun)
{
	int rc, num_smpls; //, expect_smpls;
	ssize_t avail_smpls;
	unsigned int i = 0;

	*overrun = false;
	*underrun = false;

	timestamp += 0;

	/* Check that timestamp is valid */
	rc = rx_buffers[0]->avail_smpls(timestamp);
	if (rc < 0) {
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_code(rc);
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_status(timestamp);
		return 0;
	}

	/* Receive samples from HW until we have enough */
	while ((avail_smpls = rx_buffers[i]->avail_smpls(timestamp)) < len) {
		uint64_t recv_timestamp = timestamp;

		m.read_ul(len - avail_smpls, &recv_timestamp, reinterpret_cast<sample_t *>(bufs[0]));
		num_smpls = len - avail_smpls;
		wait_for_sample_time(&readtime, num_smpls);

		if (num_smpls == -ETIMEDOUT)
			continue;

		LOGCHAN(i, DDEV, DEBUG)
		"Received timestamp = " << (TIMESTAMP)recv_timestamp << " (" << num_smpls << ")";

		rc = rx_buffers[i]->write(bufs[i], num_smpls, (TIMESTAMP)recv_timestamp);
		if (rc < 0) {
			LOGCHAN(i, DDEV, ERROR)
				<< rx_buffers[i]->str_code(rc) << " num smpls: " << num_smpls << " chan: " << i;
			LOGCHAN(i, DDEV, ERROR) << rx_buffers[i]->str_status(timestamp);
			if (rc != smpl_buf::ERROR_OVERFLOW)
				return 0;
		}
	}

	/* We have enough samples */

	rc = rx_buffers[i]->read(bufs[i], len, timestamp);
	if ((rc < 0) || (rc != len)) {
		LOGCHAN(i, DDEV, ERROR) << rx_buffers[i]->str_code(rc) << ". " << rx_buffers[i]->str_status(timestamp)
					<< ", (len=" << len << ")";
		return 0;
	}

	return len;
}

int IPCDevice2::writeSamples(std::vector<short *> &bufs, int len, bool *underrun, unsigned long long timestamp)
{
	*underrun = false;

	LOGCHAN(0, DDEV, DEBUG) << "send buffer of len " << len << " timestamp " << std::hex << timestamp;

	// rc = ipc_shm_enqueue(shm_io_tx_streams[i], timestamp, len, (uint16_t *)bufs[i]);
	m.write_dl(len, timestamp, reinterpret_cast<sample_t *>(bufs[0]));
	wait_for_sample_time(&writetime, len);

	return len;
}

bool IPCDevice2::updateAlignment(TIMESTAMP timestamp)
{
	return true;
}

bool IPCDevice2::setTxFreq(double wFreq, size_t chan)
{
	return true;
}

bool IPCDevice2::setRxFreq(double wFreq, size_t chan)
{
	return true;
}

RadioDevice *RadioDevice::make(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chans, double lo_offset,
			       const std::vector<std::string> &tx_paths, const std::vector<std::string> &rx_paths)
{
	if (tx_sps != rx_sps) {
		LOGC(DDEV, ERROR) << "IPC Requires tx_sps == rx_sps";
		return NULL;
	}
	if (lo_offset != 0.0) {
		LOGC(DDEV, ERROR) << "IPC doesn't support lo_offset";
		return NULL;
	}
	return new IPCDevice2(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths);
}
