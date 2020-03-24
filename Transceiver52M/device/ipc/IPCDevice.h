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

#ifndef _IPC_DEVICE_H_
#define _IPC_DEVICE_H_

#include <cstdint>
#include <cstddef>
#include <climits>
#include <string>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

extern "C" {
#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include "shm.h"
}

#include "radioDevice.h"

class smpl_buf;

#define IPC_MAX_NUM_TRX 8

struct ipc_per_trx_sock_state {
	struct osmo_fd conn_bfd; /* fd for connection to the BTS */
	struct osmo_timer_list timer; /* socket connect retry timer */
	struct llist_head upqueue; /* queue for sending messages */
	uint32_t messages_processed_mask; // (=| IPC_IF_MSG_xxx-IPC_IF_CHAN_MSG_OFFSET) bitmask
	ipc_per_trx_sock_state() : conn_bfd(), timer(), upqueue(), messages_processed_mask()
	{
		conn_bfd.fd = -1;
	}
};

class IPCDevice : public RadioDevice {
    protected:
	struct ipc_per_trx_sock_state master_sk_state;

	std::vector<struct ipc_per_trx_sock_state> sk_chan_state;

	uint32_t tx_attenuation[IPC_MAX_NUM_TRX];
	uint8_t tmp_state;
	char shm_name[SHM_NAME_MAX];
	int ipc_shm_connect(const char *shm_name);
	void *shm;
	struct ipc_shm_region *shm_dec;

	std::vector<smpl_buf *> rx_buffers;
	double actualSampleRate;

	bool started;

	TIMESTAMP ts_initial, ts_offset;

	std::vector<double> tx_gains, rx_gains;

	struct ipc_sk_if_info_req current_info_req;
	struct ipc_sk_if_info_cnf current_info_cnf;
	struct ipc_sk_if_open_cnf current_open_cnf;

	std::vector<struct ipc_shm_io *> shm_io_rx_streams;
	std::vector<struct ipc_shm_io *> shm_io_tx_streams;

	bool flush_recv(size_t num_pkts);
	void update_stream_stats_rx(size_t chan, bool *overrun);
	void update_stream_stats_tx(size_t chan, bool *underrun);
	void manually_poll_sock_fds();

	void ipc_sock_close(ipc_per_trx_sock_state *state);

	int ipc_rx(uint8_t msg_type, struct ipc_sk_if *ipc_prim);
	int ipc_rx_greeting_cnf(const struct ipc_sk_if_greeting *greeting_cnf);
	int ipc_rx_info_cnf(const struct ipc_sk_if_info_cnf *info_cnf);
	int ipc_rx_open_cnf(const struct ipc_sk_if_open_cnf *open_cnf);
	int ipc_tx_open_req(struct ipc_per_trx_sock_state *state, uint32_t num_chans, uint32_t ref);

	int ipc_chan_rx(uint8_t msg_type, ipc_sk_chan_if *ipc_prim, uint8_t chan_nr);
	int ipc_rx_chan_start_cnf(ipc_sk_chan_if_op_rc *ret, uint8_t chan_nr);
	int ipc_rx_chan_stop_cnf(ipc_sk_chan_if_op_rc *ret, uint8_t chan_nr);
	int ipc_rx_chan_setgain_cnf(ipc_sk_chan_if_gain *ret, uint8_t chan_nr);
	int ipc_rx_chan_setfreq_cnf(ipc_sk_chan_if_freq_cnf *ret, uint8_t chan_nr);
	int ipc_rx_chan_notify_underflow(ipc_sk_chan_if_notfiy *ret, uint8_t chan_nr);
	int ipc_rx_chan_notify_overflow(ipc_sk_chan_if_notfiy *ret, uint8_t chan_nr);
	int ipc_rx_chan_settxattn_cnf(ipc_sk_chan_if_tx_attenuation *ret, uint8_t chan_nr);

	bool send_chan_wait_rsp(uint32_t chan, struct msgb *msg_to_send, uint32_t expected_rsp_msg_id);
	bool send_all_chan_wait_rsp(uint32_t msgid_to_send, uint32_t msgid_to_expect);

    public:
	int ipc_sock_read(struct osmo_fd *bfd);
	int ipc_sock_write(struct osmo_fd *bfd);
	int ipc_chan_sock_read(osmo_fd *bfd);
	int ipc_chan_sock_write(osmo_fd *bfd);

	/** Object constructor */
	IPCDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chan_num, double lo_offset,
		  const std::vector<std::string> &tx_paths, const std::vector<std::string> &rx_paths);
	virtual ~IPCDevice() override;

	/** Instantiate the IPC */
	virtual int open(const std::string &args, int ref, bool swap_channels) override;

	/** Start the IPC */
	virtual bool start() override;

	/** Stop the IPC */
	virtual bool stop() override;

	/* FIXME: any != USRP1 will do for now... */
	enum TxWindowType getWindowType() override
	{
		return TX_WINDOW_LMS1;
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
		return (double)SHRT_MAX * current_info_cnf.iq_scaling_val_rx;
	}

	/** returns the full-scale receive amplitude **/
	virtual double fullScaleOutputValue() override
	{
		return (double)SHRT_MAX * current_info_cnf.iq_scaling_val_tx;
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
