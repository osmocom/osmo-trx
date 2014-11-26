/*
* Copyright 2008 Free Software Foundation, Inc.
*
* This software is distributed under multiple licenses; see the COPYING file in the main directory for licensing information for this specific distribuion.
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
#include "Resampler.h"

static const unsigned gSlotLen = 148;      ///< number of symbols per slot, not counting guard periods

/** class to interface the transceiver with the USRP */
class RadioInterface {

protected:

  Thread mAlignRadioServiceLoopThread;	      ///< thread that synchronizes transmit and receive sections

  std::vector<VectorFIFO>  mReceiveFIFO;      ///< FIFO that holds receive  bursts

  RadioDevice *mRadio;			      ///< the USRP object

  size_t mSPSTx;
  size_t mSPSRx;
  size_t mChans;
  size_t mMIMO;

  std::vector<signalVector *> sendBuffer;
  std::vector<signalVector *> recvBuffer;
  unsigned sendCursor;
  unsigned recvCursor;

  std::vector<short *> convertRecvBuffer;
  std::vector<short *> convertSendBuffer;
  std::vector<float> powerScaling;
  bool underrun;			      ///< indicates writes to USRP are too slow
  bool overrun;				      ///< indicates reads from USRP are too slow
  TIMESTAMP writeTimestamp;		      ///< sample timestamp of next packet written to USRP
  TIMESTAMP readTimestamp;		      ///< sample timestamp of next packet read from USRP

  RadioClock mClock;                          ///< the basestation clock!

  int receiveOffset;                          ///< offset b/w transmit and receive GSM timestamps, in timeslots

  bool mOn;				      ///< indicates radio is on

private:

  /** format samples to USRP */ 
  int radioifyVector(signalVector &wVector,
                     float *floatVector,
                     bool zero);

  /** format samples from USRP */
  int unRadioifyVector(float *floatVector, signalVector &wVector);

  /** push GSM bursts into the transmit buffer */
  virtual void pushBuffer(void);

  /** pull GSM bursts from the receive buffer */
  virtual void pullBuffer(void);

public:

  /** start the interface */
  bool start();
  bool stop();

  /** intialization */
  virtual bool init(int type);
  virtual void close();

  /** constructor */
  RadioInterface(RadioDevice* wRadio = NULL,
                 size_t sps = 4, size_t chans = 1, size_t diversity = 1,
                 int receiveOffset = 3, GSM::Time wStartTime = GSM::Time(0));

  /** destructor */
  virtual ~RadioInterface();

  /** check for underrun, resets underrun value */
  bool isUnderrun();

  /** return the receive FIFO */
  VectorFIFO* receiveFIFO(size_t chan = 0);

  /** return the basestation clock */
  RadioClock* getClock(void) { return &mClock;};

  /** set transmit frequency */
  bool tuneTx(double freq, size_t chan = 0);

  /** set receive frequency */
  virtual bool tuneRx(double freq, size_t chan = 0);

  /** set receive gain */
  double setRxGain(double dB, size_t chan = 0);

  /** get receive gain */
  double getRxGain(size_t chan = 0);

  /** drive transmission of GSM bursts */
  void driveTransmitRadio(std::vector<signalVector *> &bursts,
                          std::vector<bool> &zeros);

  /** drive reception of GSM bursts */
  bool driveReceiveRadio();

  int setPowerAttenuation(int atten, size_t chan = 0);

  /** returns the full-scale transmit amplitude **/
  double fullScaleInputValue();

  /** returns the full-scale receive amplitude **/
  double fullScaleOutputValue();

  /** set thread priority on current thread */
  void setPriority(float prio = 0.5) { mRadio->setPriority(prio); }

  /** get transport window type of attached device */ 
  enum RadioDevice::TxWindowType getWindowType() { return mRadio->getWindowType(); }

#if USRP1
protected:

  /** drive synchronization of Tx/Rx of USRP */
  void alignRadio();

  friend void *AlignRadioServiceLoopAdapter(RadioInterface*);
#endif
};

#if USRP1
/** synchronization thread loop */
void *AlignRadioServiceLoopAdapter(RadioInterface*);
#endif

class RadioInterfaceResamp : public RadioInterface {

private:
  signalVector *innerSendBuffer;
  signalVector *outerSendBuffer;
  signalVector *innerRecvBuffer;
  signalVector *outerRecvBuffer;

  void pushBuffer();
  void pullBuffer();

public:

  RadioInterfaceResamp(RadioDevice* wRadio, size_t wSPS = 4, size_t chans = 1);

  ~RadioInterfaceResamp();

  bool init(int type);
  void close();
};

class RadioInterfaceDiversity : public RadioInterface {
public:
  RadioInterfaceDiversity(RadioDevice* wRadio,
                          size_t sps = 4, size_t chans = 2);

  ~RadioInterfaceDiversity();

  bool init(int type);
  void close();
  bool tuneRx(double freq, size_t chan);

private:
  std::vector<Resampler *> dnsamplers;
  std::vector<float> phases;
  signalVector *outerRecvBuffer;

  bool mDiversity;
  double mFreqSpacing;

  bool setupDiversityChannels();
  void pullBuffer();
};
