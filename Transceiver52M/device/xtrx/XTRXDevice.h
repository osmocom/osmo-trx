#ifndef _XTRX_DEVICE_H_
#define _XTRX_DEVICE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "radioDevice.h"

#include <stdint.h>
#include <sys/time.h>
#include <string>
#include <iostream>

#include "Threads.h"
#include <xtrx_api.h>

class XTRXDevice: public RadioDevice {
private:
	int txsps;
	int rxsps;
	double actualTXSampleRate;	///< the actual XTRX sampling rate
	double actualRXSampleRate;	///< the actual XTRX sampling rate
	//unsigned int decimRate;	///< the XTRX decimation rate
	//unsigned int interRate;	///< the XTRX decimation rate

	unsigned long long samplesRead;	///< number of samples read from XTRX
	unsigned long long samplesWritten;	///< number of samples sent to XTRX

	bool started;			///< flag indicates XTRX has started

	short *data;
	unsigned long dataStart;
	unsigned long dataEnd;
	TIMESTAMP timeStart;
	TIMESTAMP timeEnd;

	TIMESTAMP timeRx;
	bool isAligned;

	Mutex writeLock;

	short *currData;		///< internal data buffer when reading from XTRX
	TIMESTAMP currTimestamp;	///< timestamp of internal data buffer
	unsigned currLen;		///< size of internal data buffer

	TIMESTAMP timestampOffset;       ///< timestamp offset b/w Tx and Rx blocks
	TIMESTAMP latestWriteTimestamp;  ///< timestamp of most recent ping command
	TIMESTAMP pingTimestamp;	   ///< timestamp of most recent ping response

	unsigned long hi32Timestamp;
	unsigned long lastPktTimestamp;

	double rxGain;
	double txGain;
	bool loopback;

	xtrx_dev* device;
public:

	/** Object constructor */
	XTRXDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chan_num, double lo_offset,
			   const std::vector<std::string>& tx_paths,
			   const std::vector<std::string>& rx_paths);

	~XTRXDevice();

	/** Instantiate the XTRX */
	int open(const std::string &args, int ref, bool swap_channels);

	/** Start the XTRX */
	bool start();

	/** Stop the XTRX */
	bool stop();

	/** Set priority not supported */
	void setPriority(float prio = 0.5) { }

	enum TxWindowType getWindowType() { return TX_WINDOW_FIXED; }

	/**
	Read samples from the XTRX.
	@param buf preallocated buf to contain read result
	@param len number of samples desired
	@param overrun Set if read buffer has been overrun, e.g. data not being read fast enough
	@param timestamp The timestamp of the first samples to be read
	@param underrun Set if XTRX does not have data to transmit, e.g. data not being sent fast enough
	@param RSSI The received signal strength of the read result
	@return The number of samples actually read
	*/
	int readSamples(std::vector<short *> &buf, int len, bool *overrun,
					TIMESTAMP timestamp = 0xffffffff, bool *underrun = NULL,
					unsigned *RSSI = NULL);
	/**
		Write samples to the XTRX.
		@param buf Contains the data to be written.
		@param len number of samples to write.
		@param underrun Set if XTRX does not have data to transmit, e.g. data not being sent fast enough
		@param timestamp The timestamp of the first sample of the data buffer.
		@param isControl Set if data is a control packet, e.g. a ping command
		@return The number of samples actually written
	*/
	int writeSamples(std::vector<short *> &bufs, int len, bool *underrun,
					 TIMESTAMP timestamp = 0xffffffff, bool isControl = false);

	/** Update the alignment between the read and write timestamps */
	bool updateAlignment(TIMESTAMP timestamp);

	/** Set the transmitter frequency */
	bool setTxFreq(double wFreq, size_t chan = 0);

	/** Set the receiver frequency */
	bool setRxFreq(double wFreq, size_t chan = 0);

	/** Returns the starting write Timestamp*/
	TIMESTAMP initialWriteTimestamp(void);

	/** Returns the starting read Timestamp*/
	TIMESTAMP initialReadTimestamp(void) { return 20000;}

	/** returns the full-scale transmit amplitude **/
	double fullScaleInputValue() {return (double) 32767*0.7;}

	/** returns the full-scale receive amplitude **/
	double fullScaleOutputValue() {return (double) 32767;}

	/** sets the receive chan gain, returns the gain setting **/
	double setRxGain(double dB, size_t chan = 0);

	/** get the current receive gain */
	double getRxGain(size_t chan = 0) { return rxGain; }

	/** return maximum Rx Gain **/
	double maxRxGain(void);

	/** return minimum Rx Gain **/
	double minRxGain(void);

	/** sets the transmit chan gain, returns the gain setting **/
	double setTxGain(double dB, size_t chan = 0);

	/** gets the current transmit gain **/
	double getTxGain(size_t chan = 0) { return txGain; }

	/** return maximum Tx Gain **/
	double maxTxGain(void);

	/** return minimum Rx Gain **/
	double minTxGain(void);

	/** sets the RX path to use, returns true if successful and false otherwise */
	bool setRxAntenna(const std::string & ant, size_t chan = 0);

	/** return the used RX path */
	std::string getRxAntenna(size_t chan = 0);

	/** sets the RX path to use, returns true if successful and false otherwise */
	bool setTxAntenna(const std::string & ant, size_t chan = 0);

	/** return the used RX path */
	std::string getTxAntenna(size_t chan = 0);

	/** return whether user drives synchronization of Tx/Rx of USRP */
	bool requiresRadioAlign();

	/** return whether user drives synchronization of Tx/Rx of USRP */
	virtual GSM::Time minLatency();

	/** Return internal status values */
	inline double getTxFreq(size_t chan = 0) { return 0; }
	inline double getRxFreq(size_t chan = 0) { return 0; }
	inline double getSampleRate() { return actualTXSampleRate; }
	inline double numberRead() { return samplesRead; }
	inline double numberWritten() { return samplesWritten; }

};

#endif // _XTRX_DEVICE_H_

