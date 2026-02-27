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

  /**
   * @brief Open the radio device and initialize it with the provided configuration
   * @return NORMAL == 0 if the radio device was successfully opened and initialized, -1 otherwise
   */
  virtual int open() = 0;

  virtual ~RadioDevice() { }

  /**
   * @brief Start the radio device
   * @return true if the radio was successfully started, false otherwise
   */
  virtual bool start()=0;

  /**
   * @brief Stop the radio device
   * @return true if the radio device was successfully stopped, false otherwise
   */
  virtual bool stop()=0;

  /**
   * @brief Get the type of the transmit window, which can be one of TX_WINDOW_USRP1, TX_WINDOW_FIXED,
   * or TX_WINDOW_LMS1.
   * The transmit window type determines how the radio device handles the timing of transmitted samples.
   * @return The type of the transmit window used by the radio device
   */
  virtual enum TxWindowType getWindowType()=0;

  /**
   * @brief Read samples from the radio device.
   * @param bufs preallocated buffers to contain read result
   * @param len number of samples desired
   * @param overrun Set if read buffer has been overrun, e.g. data not being read fast enough
   * @param timestamp The timestamp of the first samples to be read
   * @param underrun Set if radio device does not have data to transmit, e.g. data not being sent fast enough
   * @return The number of samples actually read
   */
  virtual int readSamples(std::vector<short *> &bufs, int len, bool *overrun,
                          TIMESTAMP timestamp = 0xffffffff, bool *underrun = 0) = 0;

  /**
   * @brief Write samples to the radio device.
   * @param bufs Contains the data to be written.
   * @param len number of samples to write.
   * @param underrun Set if radio device does not have data to transmit, e.g. data not being sent fast enough
   * @param timestamp The timestamp of the first sample of the data buffer.
   * @return The number of samples actually written
   */
  virtual int writeSamples(std::vector<short *> &bufs, int len, bool *underrun,
                           TIMESTAMP timestamp) = 0;

  /**
   * @brief Update the alignment between the read and write timestamps
   * @param timestamp The timestamp to use for alignment
   * @return true if the alignment was successfully updated, false otherwise
   */
  virtual bool updateAlignment(TIMESTAMP timestamp)=0;

  /**
   * @brief Set the transmitter frequency
   * @param wFreq The frequency to set
   * @param chan The channel to set the frequency for
   * @return true if the transmitter frequency was successfully set, false otherwise
   */
  virtual bool setTxFreq(double wFreq, size_t chan = 0) = 0;

  /**
   * @brief Set the receiver frequency
   * @param wFreq The frequency to set
   * @param chan The channel to set the frequency for
   * @return true if the receiver frequency was successfully set, false otherwise
   */
  virtual bool setRxFreq(double wFreq, size_t chan = 0) = 0;

  /**
   * @brief Get the initial write timestamp, which is the timestamp of the first sample to be transmitted
   * after starting the device.
   * @return The initial write timestamp
   */
  virtual TIMESTAMP initialWriteTimestamp(void)=0;

  /**
   * @brief Get the initial read timestamp, i.e. the timestamp of the first received sample
   * after starting the device.
   * @return The initial read timestamp
   */
  virtual TIMESTAMP initialReadTimestamp(void)=0;

  /**
   * @brief Returns the full-scale transmit amplitude
   * Usually is set to half the ADC range multiplied by 1/√2
   * (i.e. ADC_range/2 * 1/√2 ≈ ADC_range/2 * 0.70710678) to avoid clipping for complex samples I + jQ.
   * With |I|, |Q| <= 1/√2 the magnitude I^2 + Q^2 <= 1.
   * @return The full-scale transmit amplitude
   */
  virtual double fullScaleInputValue()=0;

  /**
   * @brief Returns the full-scale receive amplitude
   * Usually is set to half of ADC range, e.g. 32767 for a 16-bit ADC.
   * @return The full-scale receive amplitude
   */
  virtual double fullScaleOutputValue()=0;

  /**
   * @brief Set the receive channel gain
   * @param dB The gain value in dB
   * @param chan The channel to set the gain for
   * @return The actual gain setting after applying the change
   */
  virtual double setRxGain(double dB, size_t chan = 0) = 0;

  /**
   * @brief Get the current receive channel gain
   * @param chan The channel to get the gain for
   * @return The current gain setting
   */
  virtual double getRxGain(size_t chan = 0) = 0;

  /**
   * @brief Get the maximum Rx Gain
   * @return The maximum Rx Gain
   */
  virtual double maxRxGain(void) = 0;

  /**
   * @brief Get the minimum Rx Gain
   * @return The minimum Rx Gain
   */
  virtual double minRxGain(void) = 0;

  /**
   * @brief Get the RSSI offset for a given channel to apply for received samples
   * @param chan The channel to get the RSSI offset for
   * @return The RSSI offset for the given channel
   */
  virtual double rssiOffset(size_t chan) = 0;

  /**
   * @brief Get the nominal transmit output power for a given channel
   * @param chan The channel to get the nominal transmit output power for
   * @return The nominal transmit output power in dBm, negative on error
   */
  virtual int getNominalTxPower(size_t chan = 0) = 0;

  /**
   * @brief Sets the RX path to use
   * @param ant The antenna to set
   * @param chan The channel to set the antenna for
   * @return True if successful, false otherwise
   */
  virtual bool setRxAntenna(const std::string &ant, size_t chan = 0) = 0;

  /**
   * @brief Get the used RX path
   * @param chan The channel to get the antenna for
   * @return The current RX path
   */
  virtual std::string getRxAntenna(size_t chan = 0) = 0;

  /**
   * @brief Sets the TX path to use
   * @param ant The antenna to set
   * @param chan The channel to set the antenna for
   * @return True if successful, false otherwise
   */
  virtual bool setTxAntenna(const std::string &ant, size_t chan = 0) = 0;

  /**
   * @brief Get the used TX path
   * @param chan The channel to get the antenna for
   * @return The current TX path
   */
  virtual std::string getTxAntenna(size_t chan = 0) = 0;

  /**
   * @brief Return whether user drives synchronization of Tx/Rx
   * @return true if user drives synchronization of Tx/Rx, false otherwise
   */
  virtual bool requiresRadioAlign() = 0;

  /**
   * @brief Return the minimum latency the radio device can achieve
   * @return The minimum latency
   */
  virtual GSM::Time minLatency() = 0;

  /** Return internal status values */

  /**
   * @brief Get the transceiver frequency
   * @param chan The channel to get the frequency for
   * @return The current transceiver frequency
   */
  virtual double getTxFreq(size_t chan = 0) = 0;

  /**
   * @brief Get the receiver frequency
   * @param chan The channel to get the frequency for
   * @return The current receiver frequency
   */
  virtual double getRxFreq(size_t chan = 0) = 0;

  /**
   * @brief Return actual sample rate of the radio device
   * @return The current sample rate
   */
  virtual double getSampleRate()=0;

  /**
   * @brief Set the power attenuation for a given channel
   * @param atten The attenuation value in dB
   * @param chan The channel to set the attenuation for
   * @return The actual attenuation setting after applying the change
   */
  virtual double setPowerAttenuation(int atten, size_t chan) = 0;

  /**
   * @brief Get the power attenuation for a given channel
   * @param chan The channel to get the attenuation for
   * @return The current attenuation setting
   */
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
