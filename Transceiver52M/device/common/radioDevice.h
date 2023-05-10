/*
* Copyright 2008 Free Software Foundation, Inc.
*
* This software is distributed under multiple licenses; see the COPYING file in the main directory for licensing information for this specific distribution.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/

#ifndef __RADIO_DEVICE_H__
#define __RADIO_DEVICE_H__

#include <string>
#include <vector>

#include "GSMCommon.h"
#include "Logger.h"

extern "C" {
#include "config_defs.h"
#include "osmo_signal.h"
}

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define GSMRATE       (1625e3/6)
#define MCBTS_SPACING  800000.0

/** a 64-bit virtual timestamp for radio data */
typedef unsigned long long TIMESTAMP;

/** A class to handle a USRP rev 4, with a two RFX900 daughterboards */
class RadioDevice {

  public:
  /* Available transport bus types */
  enum TxWindowType { TX_WINDOW_USRP1, TX_WINDOW_FIXED, TX_WINDOW_LMS1 };

  /* Radio interface types */
  enum InterfaceType {
    NORMAL,
    RESAMP_64M,
    RESAMP_100M,
    MULTI_ARFCN,
  };

  static RadioDevice *make(InterfaceType type, const struct trx_cfg *cfg);

  /** Initialize the USRP */
  virtual int open() = 0;

  virtual ~RadioDevice() { }

  /** Start the USRP */
  virtual bool start()=0;

  /** Stop the USRP */
  virtual bool stop()=0;

  /** Get the Tx window type */
  virtual enum TxWindowType getWindowType()=0;

  /**
	Read samples from the radio.
	@param buf preallocated buf to contain read result
	@param len number of samples desired
	@param overrun Set if read buffer has been overrun, e.g. data not being read fast enough
	@param timestamp The timestamp of the first samples to be read
	@param underrun Set if radio does not have data to transmit, e.g. data not being sent fast enough
	@return The number of samples actually read
  */
  virtual int readSamples(std::vector<short *> &bufs, int len, bool *overrun,
                          TIMESTAMP timestamp = 0xffffffff, bool *underrun = 0) = 0;
  /**
        Write samples to the radio.
        @param buf Contains the data to be written.
        @param len number of samples to write.
        @param underrun Set if radio does not have data to transmit, e.g. data not being sent fast enough
        @param timestamp The timestamp of the first sample of the data buffer.
        @return The number of samples actually written
  */
  virtual int writeSamples(std::vector<short *> &bufs, int len, bool *underrun,
                           TIMESTAMP timestamp) = 0;

  /** Update the alignment between the read and write timestamps */
  virtual bool updateAlignment(TIMESTAMP timestamp)=0;

  /** Set the transmitter frequency */
  virtual bool setTxFreq(double wFreq, size_t chan = 0) = 0;

  /** Set the receiver frequency */
  virtual bool setRxFreq(double wFreq, size_t chan = 0) = 0;

  /** Returns the starting write Timestamp*/
  virtual TIMESTAMP initialWriteTimestamp(void)=0;

  /** Returns the starting read Timestamp*/
  virtual TIMESTAMP initialReadTimestamp(void)=0;

  /** returns the full-scale transmit amplitude **/
  virtual double fullScaleInputValue()=0;

  /** returns the full-scale receive amplitude **/
  virtual double fullScaleOutputValue()=0;

  /** sets the receive chan gain, returns the gain setting **/
  virtual double setRxGain(double dB, size_t chan = 0) = 0;

  /** gets the current receive gain **/
  virtual double getRxGain(size_t chan = 0) = 0;

  /** return maximum Rx Gain **/
  virtual double maxRxGain(void) = 0;

  /** return minimum Rx Gain **/
  virtual double minRxGain(void) = 0;

  /** return base RSSI offset to apply for received samples **/
  virtual double rssiOffset(size_t chan) = 0;

  /** returns the Nominal transmit output power of the transceiver in dBm, negative on error **/
  virtual int getNominalTxPower(size_t chan = 0) = 0;

  /** sets the RX path to use, returns true if successful and false otherwise */
  virtual bool setRxAntenna(const std::string &ant, size_t chan = 0) = 0;

  /** return the used RX path */
  virtual std::string getRxAntenna(size_t chan = 0) = 0;

  /** sets the RX path to use, returns true if successful and false otherwise */
  virtual bool setTxAntenna(const std::string &ant, size_t chan = 0) = 0;

  /** return the used RX path */
  virtual std::string getTxAntenna(size_t chan = 0) = 0;

  /** return whether user drives synchronization of Tx/Rx of USRP */
  virtual bool requiresRadioAlign() = 0;

  /** Minimum latency that the device can achieve */
  virtual GSM::Time minLatency() = 0;

  /** Return internal status values */
  virtual double getTxFreq(size_t chan = 0) = 0;
  virtual double getRxFreq(size_t chan = 0) = 0;
  virtual double getSampleRate()=0;

  virtual double setPowerAttenuation(int atten, size_t chan) = 0;
  virtual double getPowerAttenuation(size_t chan=0) = 0;

  protected:
  size_t tx_sps, rx_sps;
  InterfaceType iface;
  size_t chans;
  double lo_offset;
  std::vector<std::string> tx_paths, rx_paths;
  std::vector<struct device_counters> m_ctr;
  const struct trx_cfg *cfg;

#define charp2str(a) ((a) ? std::string(a) : std::string(""))

  RadioDevice(InterfaceType type, const struct trx_cfg *cfg)
	  : tx_sps(cfg->tx_sps), rx_sps(cfg->rx_sps), iface(type), chans(cfg->num_chans), lo_offset(cfg->offset),
	    m_ctr(chans), cfg(cfg)
  {
	  /* Generate vector of rx/tx_path: */
	  for (unsigned int i = 0; i < cfg->num_chans; i++) {
		  rx_paths.push_back(charp2str(cfg->chans[i].rx_path));
		  tx_paths.push_back(charp2str(cfg->chans[i].tx_path));
	  }

	  if (iface == MULTI_ARFCN) {
		  LOGC(DDEV, INFO) << "Multi-ARFCN: " << chans << " logical chans -> 1 physical chans";
		  chans = 1;
	  }

	  for (size_t i = 0; i < chans; i++) {
		  memset(&m_ctr[i], 0, sizeof(m_ctr[i]));
		  m_ctr[i].chan = i;
	  }
  }

  bool set_antennas() {
	unsigned int i;

	for (i = 0; i < tx_paths.size(); i++) {
		if (tx_paths[i] == "")
			continue;
		if (iface == MULTI_ARFCN && i > 0) {
			LOGCHAN(i, DDEV, NOTICE) << "Not setting Tx antenna "
						 << tx_paths[i]
						 << " for a logical channel";
			continue;
		}

		LOGCHAN(i, DDEV, DEBUG) << "Configuring Tx antenna " << tx_paths[i];
		if (!setTxAntenna(tx_paths[i], i)) {
			LOGCHAN(i, DDEV, ALERT) << "Failed configuring Tx antenna " << tx_paths[i];
			return false;
		}
	}

	for (i = 0; i < rx_paths.size(); i++) {
		if (rx_paths[i] == "")
			continue;
		if (iface == MULTI_ARFCN && i > 0) {
			LOGCHAN(i, DDEV, NOTICE) << "Not setting Rx antenna "
						 << rx_paths[i]
						 << " for a logical channel";
			continue;
		}

		LOGCHAN(i, DDEV, DEBUG) << "Configuring Rx antenna " << rx_paths[i];
		if (!setRxAntenna(rx_paths[i], i)) {
			LOGCHAN(i, DDEV, ALERT) << "Failed configuring Rx antenna " << rx_paths[i];
			return false;
		}
	}
	LOG(INFO) << "Antennas configured successfully";
	return true;
  }


};

#endif
