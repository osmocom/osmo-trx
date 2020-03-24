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
}

#include "radioDevice.h"
#include "smpl_buf.h"

#include <sys/time.h>
#include <math.h>
#include <limits.h>
#include <string>
#include <iostream>
#include <lime/LimeSuite.h>

struct ipc_sock_state {
	struct osmo_fd conn_bfd;	/* fd for connection to the BTS */
	struct osmo_timer_list timer;	/* socket connect retry timer */
	struct llist_head upqueue;	/* queue for sending messages */
};


/** A class to handle a LimeSuite supported device */
class IPCDevice:public RadioDevice {

private:
	struct ipc_sock_state sk_state;
	uint8_t tmp_state;
	char shm_name[SHM_NAME_MAX];
	int ipc_shm_connect(const char *shm_name);
	void *shm;
	struct ipc_shm_region *shm_dec;

	std::vector<smpl_buf *> rx_buffers;
	double actualSampleRate;	///< the actual USRP sampling rate

	bool started;		///< flag indicates LMS has started
	bool skipRx;		///< set if LMS is transmit-only.

	TIMESTAMP ts_initial, ts_offset;

	std::vector<double> tx_gains, rx_gains;

	bool do_calib(size_t chan);
	bool do_filters(size_t chan);
	void log_ant_list(bool dir_tx, size_t chan, std::ostringstream& os);
	int get_ant_idx(const std::string & name, bool dir_tx, size_t chan);
	bool flush_recv(size_t num_pkts);
	void update_stream_stats_rx(size_t chan, bool *overrun);
	void update_stream_stats_tx(size_t chan, bool *underrun);
	bool do_clock_src_freq(enum ReferenceType ref, double freq);

public:
	void ipc_sock_close();
	int ipc_sock_read(struct osmo_fd *bfd);
	int ipc_sock_write(struct osmo_fd *bfd);
	int ipc_rx(uint8_t msg_type, struct ipc_sk_if *ipc_prim);
	int ipc_rx_greeting_cnf(const struct ipc_sk_if_greeting *greeting_cnf);
	int ipc_rx_info_cnf(const struct ipc_sk_if_info_cnf *info_cnf);
	int ipc_rx_open_cnf(const struct ipc_sk_if_open_cnf *open_cnf);

	/** Object constructor */
	IPCDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chan_num, double lo_offset,
		  const std::vector<std::string>& tx_paths,
		  const std::vector<std::string>& rx_paths);
	~IPCDevice();

	/** Instantiate the LMS */
	int open(const std::string &args, int ref, bool swap_channels);

	/** Start the LMS */
	bool start();

	/** Stop the LMS */
	bool stop();

	enum TxWindowType getWindowType() {
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
	int readSamples(std::vector < short *>&buf, int len, bool * overrun,
			TIMESTAMP timestamp = 0xffffffff, bool * underrun =
			NULL);
	/**
	Write samples to the LMS.
	@param buf Contains the data to be written.
	@param len number of samples to write.
	@param underrun Set if LMS does not have data to transmit, e.g. data not being sent fast enough
	@param timestamp The timestamp of the first sample of the data buffer.
	@return The number of samples actually written
	*/
	int writeSamples(std::vector < short *>&bufs, int len, bool * underrun,
			 TIMESTAMP timestamp = 0xffffffff);

	/** Update the alignment between the read and write timestamps */
	bool updateAlignment(TIMESTAMP timestamp);

	/** Set the transmitter frequency */
	bool setTxFreq(double wFreq, size_t chan = 0);

	/** Set the receiver frequency */
	bool setRxFreq(double wFreq, size_t chan = 0);

	/** Returns the starting write Timestamp*/
	TIMESTAMP initialWriteTimestamp(void) {
		return ts_initial;
	}

	/** Returns the starting read Timestamp*/
	TIMESTAMP initialReadTimestamp(void) {
		return ts_initial;
	}

	/** returns the full-scale transmit amplitude **/
	double fullScaleInputValue() {
		#define LIMESDR_TX_AMPL  0.3
		return(double) SHRT_MAX * LIMESDR_TX_AMPL;
	}

	/** returns the full-scale receive amplitude **/
	double fullScaleOutputValue() {
		return (double) SHRT_MAX;
	}

	/** sets the receive chan gain, returns the gain setting **/
	double setRxGain(double dB, size_t chan = 0);

	/** get the current receive gain */
	double getRxGain(size_t chan = 0) {
		return rx_gains[chan];
	}

	/** return maximum Rx Gain **/
	double maxRxGain(void);

	/** return minimum Rx Gain **/
	double minRxGain(void);

	/** sets the transmit chan gain, returns the gain setting **/
	double setTxGain(double dB, size_t chan = 0);

	/** get transmit gain */
	double getTxGain(size_t chan = 0) {
		return tx_gains[chan];
	}

	/** return maximum Tx Gain **/
	double maxTxGain(void);

	/** return minimum Rx Gain **/
	double minTxGain(void);

	/** sets the RX path to use, returns true if successful and false otherwise */
	bool setRxAntenna(const std::string & ant, size_t chan = 0);

	/* return the used RX path */
	std::string getRxAntenna(size_t chan = 0);

	/** sets the RX path to use, returns true if successful and false otherwise */
	bool setTxAntenna(const std::string & ant, size_t chan = 0);

	/* return the used RX path */
	std::string getTxAntenna(size_t chan = 0);

	/** return whether user drives synchronization of Tx/Rx of USRP */
        bool requiresRadioAlign();

        /** return whether user drives synchronization of Tx/Rx of USRP */
        virtual GSM::Time minLatency();

	/** Return internal status values */
	inline double getTxFreq(size_t chan = 0) {
		return 0;
	}
	inline double getRxFreq(size_t chan = 0) {
		return 0;
	}
	inline double getSampleRate() {
		return actualSampleRate;
	}
};

#endif // _IPC_DEVICE_H_
