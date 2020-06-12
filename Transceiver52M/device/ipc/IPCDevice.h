/*
* Copyright 2020 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
* Author: Pau Espin Pedrol <pespin@sysmocom.de>
*
* SPDX-License-Identifier: AGPL-3.0+
*
* This software is distributed under multiple licenses; see the COPYING file in
* the main directory for licensing information for this specific distribution.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/

#ifndef _IPC_DEVICE_H_
#define _IPC_DEVICE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

extern "C" {
#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include "shm.h"
#include "ipc_shm.h"
}

#include "radioDevice.h"
#include "smpl_buf.h"

#include <sys/time.h>
#include <math.h>
#include <limits.h>
#include <string>
#include <iostream>

struct ipc_sock_state {
	struct osmo_fd conn_bfd; /* fd for connection to the BTS */
	struct osmo_timer_list timer; /* socket connect retry timer */
	struct llist_head upqueue; /* queue for sending messages */
};
#define IPC_MAX_NUM_TRX 8



/** A class to handle a LimeSuite supported device */
class IPCDevice : public RadioDevice {
    protected:
	struct ipc_sock_state sk_state;
	/* FIXME: current limit IPC_MAX_NUM_TRX chans, make dynamic */
	struct ipc_sock_state sk_chan_state[IPC_MAX_NUM_TRX];
	bool trx_is_started[IPC_MAX_NUM_TRX];
	uint8_t tmp_state;
	char shm_name[SHM_NAME_MAX];
	int ipc_shm_connect(const char *shm_name);
	void *shm;
	struct ipc_shm_region *shm_dec;

	std::vector<smpl_buf *> rx_buffers;
	double actualSampleRate; ///< the actual USRP sampling rate

	bool started; ///< flag indicates LMS has started
	bool skipRx; ///< set if LMS is transmit-only.

	TIMESTAMP ts_initial, ts_offset;

	std::vector<double> tx_gains, rx_gains;

	struct ipc_sk_if_info_req current_info_req;
	struct ipc_sk_if_info_cnf current_info_cnf;
	struct ipc_sk_if_open_cnf current_open_cnf;

	std::vector<struct ipc_shm_io *> shm_io_rx_streams;
	std::vector<struct ipc_shm_io *> shm_io_tx_streams;

	bool do_calib(size_t chan);
	bool do_filters(size_t chan);
	int get_ant_idx(const std::string &name, bool dir_tx, size_t chan);
	virtual bool flush_recv(size_t num_pkts);
	void update_stream_stats_rx(size_t chan, bool *overrun);
	void update_stream_stats_tx(size_t chan, bool *underrun);
	bool do_clock_src_freq(enum ReferenceType ref, double freq);

    public:
	virtual void ipc_sock_close(ipc_sock_state *state);
	virtual int ipc_sock_read(struct osmo_fd *bfd);
	virtual int ipc_sock_write(struct osmo_fd *bfd);
	virtual int ipc_rx(uint8_t msg_type, struct ipc_sk_if *ipc_prim);
	virtual int ipc_rx_greeting_cnf(const struct ipc_sk_if_greeting *greeting_cnf);
	virtual int ipc_rx_info_cnf(const struct ipc_sk_if_info_cnf *info_cnf);
	virtual int ipc_rx_open_cnf(const struct ipc_sk_if_open_cnf *open_cnf);
	virtual int ipc_tx_open_req(struct ipc_sock_state *state, uint32_t num_chans, uint32_t ref);

	/** Object constructor */
	IPCDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chan_num, double lo_offset,
		  const std::vector<std::string> &tx_paths, const std::vector<std::string> &rx_paths);
	virtual ~IPCDevice() override;

	/** Instantiate the LMS */
	virtual int open(const std::string &args, int ref, bool swap_channels) override;

	/** Start the LMS */
	virtual bool start() override;

	/** Stop the LMS */
	virtual bool stop() override;

	enum TxWindowType getWindowType() override
	{
		return TX_WINDOW_LMS1;
	}

	/**
	Read samples from the LMS.
	@param buf preallocated buf to contain read result
	@param len number of samples desired
	@param overrun Set if read buffer has been overrun, e.g. data not being read fast enough
	@param timestamp The timestamp of the first samples to be read
	@param underrun Set if LMS does not have data to transmit, e.g. data not being sent fast enough
	@return The number of samples actually read
	*/
	virtual int readSamples(std::vector<short *> &buf, int len, bool *overrun, TIMESTAMP timestamp = 0xffffffff,
				bool *underrun = NULL) override;
	/**
	Write samples to the LMS.
	@param buf Contains the data to be written.
	@param len number of samples to write.
	@param underrun Set if LMS does not have data to transmit, e.g. data not being sent fast enough
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

	/** sets the transmit chan gain, returns the gain setting **/
	virtual double setTxGain(double dB, size_t chan = 0) override;

	/** get transmit gain */
	virtual double getTxGain(size_t chan = 0) override
	{
		return tx_gains[chan];
	}

	/** return maximum Tx Gain **/
	virtual double maxTxGain(void) override;

	/** return minimum Rx Gain **/
	virtual double minTxGain(void) override;

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
	int ipc_chan_sock_read(osmo_fd *bfd);
	int ipc_chan_sock_write(osmo_fd *bfd);
	int ipc_chan_rx(uint8_t msg_type, ipc_sk_chan_if *ipc_prim, uint8_t chan_nr);
	int ipc_rx_chan_start_cnf(ipc_sk_chan_if_op_rc *ret, uint8_t chan_nr);
	int ipc_rx_chan_stop_cnf(ipc_sk_chan_if_op_rc *ret, uint8_t chan_nr);
	int ipc_rx_chan_setgain_cnf(ipc_sk_chan_if_gain *ret, uint8_t chan_nr);
	int ipc_rx_chan_setfreq_cnf(ipc_sk_chan_if_freq_cnf *ret, uint8_t chan_nr);
	int ipc_rx_chan_notify_underflow(ipc_sk_chan_if_notfiy *ret, uint8_t chan_nr);
	int ipc_rx_chan_notify_overflow(ipc_sk_chan_if_notfiy *ret, uint8_t chan_nr);
};

#endif // _IPC_DEVICE_H_
