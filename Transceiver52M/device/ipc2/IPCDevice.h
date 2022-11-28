/*
* Copyright 2022 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
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

#ifndef _IPC_DEVICE_H_
#define _IPC_DEVICE_H_


#include <climits>
#include <string>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "radioDevice.h"
#include "ipcif.h"

class smpl_buf;

class IPCDevice2 : public RadioDevice {
	trxmsif m;
    protected:
	std::vector<smpl_buf *> rx_buffers;
	double actualSampleRate;

	bool started;

	TIMESTAMP ts_initial;

	std::vector<double> tx_gains, rx_gains;

	bool flush_recv(size_t num_pkts);
	void update_stream_stats_rx(size_t chan, bool *overrun);
	void update_stream_stats_tx(size_t chan, bool *underrun);

	bool send_chan_wait_rsp(uint32_t chan, struct msgb *msg_to_send, uint32_t expected_rsp_msg_id);
	bool send_all_chan_wait_rsp(uint32_t msgid_to_send, uint32_t msgid_to_expect);

    public:
	/** Object constructor */
	IPCDevice2(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chan_num, double lo_offset,
		   const std::vector<std::string> &tx_paths, const std::vector<std::string> &rx_paths);
	virtual ~IPCDevice2() override;

	/** Instantiate the IPC */
	virtual int open(const std::string &args, int ref, bool swap_channels) override;

	/** Start the IPC */
	virtual bool start() override;

	/** Stop the IPC */
	virtual bool stop() override;

	/* FIXME: any != USRP1 will do for now... */
	enum TxWindowType getWindowType() override
	{
		return TX_WINDOW_FIXED;
	}

	/**
	Read samples from the IPC.
	@param buf preallocated buf to contain read result
	@param len number of samples desired
	@param overrun Set if read buffer has been overrun, e.g. data not being read fast enough
	@param timestamp The timestamp of the first samples to be read
	@param underrun Set if IPC does not have data to transmit, e.g. data not being sent fast enough
	@return The number of samples actually read
	*/
	virtual int readSamples(std::vector<short *> &buf, int len, bool *overrun, TIMESTAMP timestamp = 0xffffffff,
				bool *underrun = NULL) override;
	/**
	Write samples to the IPC.
	@param buf Contains the data to be written.
	@param len number of samples to write.
	@param underrun Set if IPC does not have data to transmit, e.g. data not being sent fast enough
	@param timestamp The timestamp of the first sample of the data buffer.
	@return The number of samples actually written
	*/
	virtual int writeSamples(std::vector<short *> &bufs, int len, bool *underrun,
				 TIMESTAMP timestamp = 0xffffffff) override;

	/** Update the alignment between the read and write timestamps */
	virtual bool updateAlignment(TIMESTAMP timestamp) override;

	/** Set the transmitter frequency */
	virtual bool setTxFreq(double wFreq, size_t chan = 0) override;

	/** Set the receiver frequency */
	virtual bool setRxFreq(double wFreq, size_t chan = 0) override;

	/** Returns the starting write Timestamp*/
	virtual TIMESTAMP initialWriteTimestamp(void) override;

	/** Returns the starting read Timestamp*/
	virtual TIMESTAMP initialReadTimestamp(void) override;

	/** returns the full-scale transmit amplitude **/
	virtual double fullScaleInputValue() override
	{
		return (double)SHRT_MAX * 1;
	}

	/** returns the full-scale receive amplitude **/
	virtual double fullScaleOutputValue() override
	{
		return (double)SHRT_MAX * 1;
	}

	/** sets the receive chan gain, returns the gain setting **/
	virtual double setRxGain(double dB, size_t chan = 0) override;

	/** get the current receive gain */
	virtual double getRxGain(size_t chan = 0) override
	{
		return rx_gains[chan];
	}

	/** return maximum Rx Gain **/
	virtual double maxRxGain(void) override;

	/** return minimum Rx Gain **/
	virtual double minRxGain(void) override;

	/* FIXME: return rx_gains[chan] ? receive factor from IPC Driver? */
	double rssiOffset(size_t chan) override
	{
		return 0.0f;
	};

	double setPowerAttenuation(int atten, size_t chan) override;
	double getPowerAttenuation(size_t chan = 0) override;

	virtual int getNominalTxPower(size_t chan = 0) override;

	/** sets the RX path to use, returns true if successful and false otherwise */
	virtual bool setRxAntenna(const std::string &ant, size_t chan = 0) override;

	/* return the used RX path */
	virtual std::string getRxAntenna(size_t chan = 0) override;

	/** sets the RX path to use, returns true if successful and false otherwise */
	virtual bool setTxAntenna(const std::string &ant, size_t chan = 0) override;

	/* return the used RX path */
	virtual std::string getTxAntenna(size_t chan = 0) override;

	/** return whether user drives synchronization of Tx/Rx of USRP */
	virtual bool requiresRadioAlign() override;

	/** return whether user drives synchronization of Tx/Rx of USRP */
	virtual GSM::Time minLatency() override;

	/** Return internal status values */
	virtual inline double getTxFreq(size_t chan = 0) override
	{
		return 0;
	}
	virtual inline double getRxFreq(size_t chan = 0) override
	{
		return 0;
	}
	virtual inline double getSampleRate() override
	{
		return actualSampleRate;
	}
};

#endif // _IPC_DEVICE_H_
