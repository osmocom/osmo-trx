/*
* Copyright 2018 sysmocom - s.f.m.c. GmbH
*
* This software is distributed under multiple licenses; see the COPYING file in the main directory for licensing information for this specific distribuion.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/

#ifndef _LMS_DEVICE_H_
#define _LMS_DEVICE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "radioDevice.h"

#include <sys/time.h>
#include <math.h>
#include <limits.h>
#include <string>
#include <iostream>
#include <lime/LimeSuite.h>

/* Definition of LIMESDR_TX_AMPL limits maximum amplitude of I and Q
 * channels separately. Hence LIMESDR_TX_AMPL value must be 1/sqrt(2) =
 * 0.7071.... to get an amplitude of 1 of the complex signal:
 * 	A^2 = I^2 + Q^2
 * 	A^2 = (1/sqrt(2))^2 + (1/sqrt(2))^2
 * 	A^2 = 1/2 + 1/2
 * 	A^2 = 1 */
#define LIMESDR_TX_AMPL  0.707

/** A class to handle a LimeSuite supported device */
class LMSDevice:public RadioDevice {

private:

	static constexpr double masterClockRate = 52.0e6;

	lms_device_t *m_lms_dev;
	std::vector<lms_stream_t> m_lms_stream_rx;
	std::vector<lms_stream_t> m_lms_stream_tx;

	std::vector<uint32_t> m_last_rx_underruns;
	std::vector<uint32_t> m_last_rx_overruns;
	std::vector<uint32_t> m_last_tx_underruns;
	std::vector<uint32_t> m_last_tx_overruns;

	double actualSampleRate;	///< the actual USRP sampling rate

	unsigned long long samplesRead;	///< number of samples read from LMS
	unsigned long long samplesWritten;	///< number of samples sent to LMS

	bool started;		///< flag indicates LMS has started
	bool skipRx;		///< set if LMS is transmit-only.

	TIMESTAMP ts_initial, ts_offset;

	double rxGain;

	int get_ant_idx(const std::string & name, bool dir_tx, size_t chan);
	bool flush_recv(size_t num_pkts);

public:

	/** Object constructor */
	LMSDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chans, double lo_offset,
		  const std::vector<std::string>& tx_paths,
		  const std::vector<std::string>& rx_paths);
	~LMSDevice();

	/** Instantiate the LMS */
	int open(const std::string &args, int ref, bool swap_channels);

	/** Start the LMS */
	bool start();

	/** Stop the LMS */
	bool stop();

	/** Set priority not supported */
	void setPriority(float prio = 0.5) {
	}

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
	@param RSSI The received signal strength of the read result
	@return The number of samples actually read
	*/
	int readSamples(std::vector < short *>&buf, int len, bool * overrun,
			TIMESTAMP timestamp = 0xffffffff, bool * underrun =
			NULL, unsigned *RSSI = NULL);
	/**
	Write samples to the LMS.
	@param buf Contains the data to be written.
	@param len number of samples to write.
	@param underrun Set if LMS does not have data to transmit, e.g. data not being sent fast enough
	@param timestamp The timestamp of the first sample of the data buffer.
	@param isControl Set if data is a control packet, e.g. a ping command
	@return The number of samples actually written
	*/
	int writeSamples(std::vector < short *>&bufs, int len, bool * underrun,
			 TIMESTAMP timestamp = 0xffffffff, bool isControl =
			 false);

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
		return rxGain;
	}

	/** return maximum Rx Gain **/
	double maxRxGain(void);

	/** return minimum Rx Gain **/
	double minRxGain(void);

	/** sets the transmit chan gain, returns the gain setting **/
	double setTxGain(double dB, size_t chan = 0);

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
	inline double numberRead() {
		return samplesRead;
	}
	inline double numberWritten() {
		return samplesWritten;
	}
};

#endif // _LMS_DEVICE_H_
