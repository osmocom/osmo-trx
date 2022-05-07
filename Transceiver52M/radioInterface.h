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



#include "sigProcLib.h"
#include "GSMCommon.h"
#include "LinkedLists.h"
#include "radioDevice.h"
#include "radioVector.h"
#include "radioClock.h"
#include "radioBuffer.h"
#include "Resampler.h"
#include "Channelizer.h"
#include "Synthesis.h"

static const unsigned gSlotLen = 148;      ///< number of symbols per slot, not counting guard periods

/** class to interface the transceiver with the USRP */
class RadioInterface {

protected:

  Thread mAlignRadioServiceLoopThread;	      ///< thread that synchronizes transmit and receive sections

  std::vector<VectorFIFO>  mReceiveFIFO;      ///< FIFO that holds receive  bursts

  RadioDevice *mDevice;			      ///< the USRP object

  size_t mSPSTx;
  size_t mSPSRx;
  size_t mChans;

  std::vector<RadioBuffer *> sendBuffer;
  std::vector<RadioBuffer *> recvBuffer;

  std::vector<short *> convertRecvBuffer;
  std::vector<short *> convertSendBuffer;
  std::vector<float> powerScaling;
  int underrun;			              ///< indicates writes to USRP are too slow
  bool overrun;				      ///< indicates reads from USRP are too slow
  TIMESTAMP writeTimestamp;		      ///< sample timestamp of next packet written to USRP
  TIMESTAMP readTimestamp;		      ///< sample timestamp of next packet read from USRP

  RadioClock mClock;                          ///< the basestation clock!

  int receiveOffset;                          ///< offset b/w transmit and receive GSM timestamps, in timeslots
  int shiftOffset;
  bool shiftUpdate;
  bool mOn;				      ///< indicates radio is on

private:

  /** format samples to USRP */
  int radioifyVector(signalVector &wVector, size_t chan, bool zero);

  /** format samples from USRP */
  int unRadioifyVector(signalVector *wVector, size_t chan);

  /** push GSM bursts into the transmit buffer */
  virtual bool pushBuffer(void);

  /** pull GSM bursts from the receive buffer */
  virtual int pullBuffer(void);

public:

  /** start the interface */
  bool start();
  bool stop();

  /** initialization */
  virtual bool init(int type);
  virtual void close();

  /** constructor */
  RadioInterface(RadioDevice* wDevice, size_t tx_sps, size_t rx_sps,
                 size_t chans = 1, int receiveOffset = 3,
                 GSM::Time wStartTime = GSM::Time(0));

  /** destructor */
  virtual ~RadioInterface();

  /** check for underrun, resets underrun value */
  bool isUnderrun();
  void applyOffset(int offset);

  /** return the receive FIFO */
  VectorFIFO* receiveFIFO(size_t chan = 0);

  /** return the basestation clock */
  RadioClock* getClock(void) { return &mClock;};

  /** apply an offset to the main clock */
  void adjustClock(GSM::Time &offset);

  /** set transmit frequency */
  virtual bool tuneTx(double freq, size_t chan = 0);

  /** set receive frequency */
  virtual bool tuneRx(double freq, size_t chan = 0);

  /** set frequency correction */
  virtual bool tuneRxOffset(double offset, size_t chan = 0);

  /** set receive gain */
  virtual double setRxGain(double dB, size_t chan = 0);

  /** return base RSSI offset to apply for received samples **/
  virtual double rssiOffset(size_t chan = 0);

  /** drive transmission of GSM bursts */
  void driveTransmitRadio(std::vector<signalVector *> &bursts,
                          std::vector<bool> &zeros);

  /** drive reception of GSM bursts. -1: Error. 0: Radio off. 1: Received something. */
  int driveReceiveRadio();

  /** set transmit power attenuation */
  virtual int setPowerAttenuation(int atten, size_t chan = 0);
  int getNominalTxPower(size_t chan = 0);

  /** returns the full-scale transmit amplitude **/
  double fullScaleInputValue();

  /** returns the full-scale receive amplitude **/
  double fullScaleOutputValue();

  /** get transport window type of attached device */
  enum RadioDevice::TxWindowType getWindowType() { return mDevice->getWindowType(); }

  /** Minimum latency that the device can achieve */
  GSM::Time minLatency()  { return mDevice->minLatency(); }

protected:
  /** drive synchronization of Tx/Rx of USRP */
  void alignRadio();

  friend void *AlignRadioServiceLoopAdapter(RadioInterface*);
};

class RadioInterfaceResamp : public RadioInterface {
private:
  signalVector *outerSendBuffer;
  signalVector *outerRecvBuffer;

  bool pushBuffer();
  int pullBuffer();

public:
  RadioInterfaceResamp(RadioDevice* wDevice, size_t tx_sps, size_t rx_sps);
  ~RadioInterfaceResamp();

  bool init(int type);
  void close();
};

struct freq_cfg_state {
  bool set;
  double freq_hz;
};

class RadioInterfaceMulti : public RadioInterface {
private:
  bool pushBuffer();
  int pullBuffer();
  bool verify_arfcn_consistency(double freq, size_t chan, bool tx);
  virtual int setPowerAttenuation(int atten, size_t chan = 0);

  signalVector *outerSendBuffer;
  signalVector *outerRecvBuffer;
  std::vector<signalVector *> history;
  std::vector<bool> active;
  std::vector<struct freq_cfg_state> rx_freq_state;
  std::vector<struct freq_cfg_state> tx_freq_state;

  Resampler *dnsampler;
  Resampler *upsampler;
  Channelizer *channelizer;
  Synthesis *synthesis;

public:
  RadioInterfaceMulti(RadioDevice* radio, size_t tx_sps,
                      size_t rx_sps, size_t chans = 1);
  ~RadioInterfaceMulti();

  bool init(int type);
  void close();

  bool tuneTx(double freq, size_t chan);
  bool tuneRx(double freq, size_t chan);
  virtual double setRxGain(double dB, size_t chan);
  virtual double rssiOffset(size_t chan = 0);
};
