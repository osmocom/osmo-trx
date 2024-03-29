/*
* Copyright 2008 Free Software Foundation, Inc.
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

#ifndef _USRP_DEVICE_H_
#define _USRP_DEVICE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "radioDevice.h"

#include <usrp/usrp_standard.h>
#include <usrp/usrp_bytesex.h>
#include <usrp/usrp_prims.h>
#include <sys/time.h>
#include <math.h>
#include <string>
#include <iostream>

#include <boost/shared_ptr.hpp>
typedef boost::shared_ptr<usrp_standard_tx> usrp_standard_tx_sptr;
typedef boost::shared_ptr<usrp_standard_rx> usrp_standard_rx_sptr;

/** A class to handle a USRP rev 4, with a two RFX900 daughterboards */
class USRPDevice: public RadioDevice {

private:

  static const double masterClockRate; ///< the USRP clock rate
  double desiredSampleRate; 	///< the desired sampling rate
  usrp_standard_rx_sptr m_uRx;	///< the USRP receiver
  usrp_standard_tx_sptr m_uTx;	///< the USRP transmitter

  db_base_sptr m_dbRx;          ///< rx daughterboard
  db_base_sptr m_dbTx;          ///< tx daughterboard
  usrp_subdev_spec rxSubdevSpec;
  usrp_subdev_spec txSubdevSpec;

  double actualSampleRate;	///< the actual USRP sampling rate
  unsigned int decimRate;	///< the USRP decimation rate

  bool started;			///< flag indicates USRP has started

  static const unsigned int currDataSize_log2 = 21;
  static const unsigned long currDataSize = (1 << currDataSize_log2);
  short *data;
  unsigned long dataStart;
  unsigned long dataEnd;
  TIMESTAMP timeStart;
  TIMESTAMP timeEnd;
  bool isAligned;

  Mutex writeLock;

  short *currData;		///< internal data buffer when reading from USRP
  TIMESTAMP currTimestamp;	///< timestamp of internal data buffer
  unsigned currLen;		///< size of internal data buffer

  TIMESTAMP timestampOffset;       ///< timestamp offset b/w Tx and Rx blocks
  TIMESTAMP latestWriteTimestamp;  ///< timestamp of most recent ping command
  TIMESTAMP pingTimestamp;	   ///< timestamp of most recent ping response

  long long  pingOffset;
  unsigned long hi32Timestamp;
  unsigned long lastPktTimestamp;

  double rxGain;
  double txGain;

  int writeSamplesControl(std::vector<short *> &bufs, int len, bool *underrun,
                   TIMESTAMP timestamp = 0xffffffff, bool isControl = false);

  /** sets the transmit chan gain, returns the gain setting **/
  double setTxGain(double dB, size_t chan = 0);

  /** get transmit gain */
  double getTxGain(size_t chan = 0) { return txGain; }

#ifdef SWLOOPBACK
  short loopbackBuffer[1000000];
  int loopbackBufferSize;
  double samplePeriod;

  struct timeval startTime;
  struct timeval lastReadTime;
  bool   firstRead;
#endif

 public:

  /** Object constructor */
     USRPDevice(InterfaceType iface, const struct trx_cfg *cfg);

     /** Instantiate the USRP */
     int open();

     /** Start the USRP */
     bool start();

     /** Stop the USRP */
     bool stop();

     enum TxWindowType getWindowType()
     {
	     return TX_WINDOW_USRP1;
     }

  /**
	Read samples from the USRP.
	@param buf preallocated buf to contain read result
	@param len number of samples desired
	@param overrun Set if read buffer has been overrun, e.g. data not being read fast enough
	@param timestamp The timestamp of the first samples to be read
	@param underrun Set if USRP does not have data to transmit, e.g. data not being sent fast enough
	@return The number of samples actually read
  */
  int readSamples(std::vector<short *> &buf, int len, bool *overrun,
                  TIMESTAMP timestamp = 0xffffffff, bool *underrun = NULL);
  /**
        Write samples to the USRP.
        @param buf Contains the data to be written.
        @param len number of samples to write.
        @param underrun Set if USRP does not have data to transmit, e.g. data not being sent fast enough
        @param timestamp The timestamp of the first sample of the data buffer.
        @return The number of samples actually written
  */
  int writeSamples(std::vector<short *> &bufs, int len, bool *underrun,
                   TIMESTAMP timestamp = 0xffffffff);

  /** Update the alignment between the read and write timestamps */
  bool updateAlignment(TIMESTAMP timestamp);

  /** Set the transmitter frequency */
  bool setTxFreq(double wFreq, size_t chan = 0);

  /** Set the receiver frequency */
  bool setRxFreq(double wFreq, size_t chan = 0);

  /** Returns the starting write Timestamp*/
  TIMESTAMP initialWriteTimestamp(void) { return 20000;}

  /** Returns the starting read Timestamp*/
  TIMESTAMP initialReadTimestamp(void) { return 20000;}

  /** returns the full-scale transmit amplitude **/
  double fullScaleInputValue() {return 13500.0;}

  /** returns the full-scale receive amplitude **/
  double fullScaleOutputValue() {return 9450.0;}

  /** sets the receive chan gain, returns the gain setting **/
  double setRxGain(double dB, size_t chan = 0);

  /** get the current receive gain */
  double getRxGain(size_t chan = 0) { return rxGain; }

  /** return maximum Rx Gain **/
  double maxRxGain(void);

  /** return minimum Rx Gain **/
  double minRxGain(void);

  double rssiOffset(size_t chan) { return 0.0f;  } /* FIXME: not implemented */

  double setPowerAttenuation(int atten, size_t chan);
  double getPowerAttenuation(size_t chan=0);

  int getNominalTxPower(size_t chan = 0);

  /** sets the RX path to use, returns true if successful and false otherwise */
  bool setRxAntenna(const std::string &ant, size_t chan = 0);

  /* return the used RX path */
  std::string getRxAntenna(size_t chan = 0);

  /** sets the RX path to use, returns true if successful and false otherwise */
  bool setTxAntenna(const std::string &ant, size_t chan = 0);

  /* return the used RX path */
  std::string getTxAntenna(size_t chan = 0);

  /** return whether user drives synchronization of Tx/Rx of USRP */
  bool requiresRadioAlign();

  /** return whether user drives synchronization of Tx/Rx of USRP */
  virtual GSM::Time minLatency();

  /** Return internal status values */
  inline double getTxFreq(size_t chan = 0) { return 0; }
  inline double getRxFreq(size_t chan = 0) { return 0; }
  inline double getSampleRate() { return actualSampleRate; }
};

#endif // _USRP_DEVICE_H_
