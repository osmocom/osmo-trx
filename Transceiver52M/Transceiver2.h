/*
* Copyright 2008 Free Software Foundation, Inc.
*
* This software is distributed under the terms of the GNU Public License.
* See the COPYING file in the main directory for details.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "radioInterface.h"
#include "Interthread.h"
#include "GSMCommon.h"

#include <sys/types.h>
#include <sys/socket.h>

extern "C" {
#include <osmocom/core/signal.h>
#include <osmocom/core/select.h>
#include "config_defs.h"
}

class Transceiver2;

/** Channel descriptor for transceiver object and channel number pair */
struct TransceiverChannel {
  TransceiverChannel(Transceiver2 *trx, int num)
  {
    this->trx = trx;
    this->num = num;
  }

  ~TransceiverChannel()
  {
  }

  Transceiver2 *trx;
  size_t num;
};

/** Internal transceiver state variables */
struct TransceiverState {
  TransceiverState();
  ~TransceiverState();

  /* Initialize a multiframe slot in the filler table */
  void init(size_t slot, signalVector *burst, bool fill);

  int chanType[8];

  /* The filler table */
  signalVector *fillerTable[102][8];
  int fillerModulus[8];
  bool mRetrans;

  /* Received noise energy levels */
  noiseVector mFreqOffsets;

  /* Store pointers to previous frame */
  radioVector *prevFrame[8];

  /* Transceiver mode */
  int mode;
};

/** The Transceiver class, responsible for physical layer of basestation */
class Transceiver2 {
private:

  int rx_sps, tx_sps;
  std::string mAddr;
  GSM::Time mTransmitLatency;     ///< latency between basestation clock and transmit deadline clock
  GSM::Time mLatencyUpdateTime;   ///< last time latency was updated

  std::vector<VectorQueue> mTxPriorityQueues;   ///< priority queue of transmit bursts received from GSM core
  std::vector<VectorFIFO *>  mReceiveFIFO;      ///< radioInterface FIFO of receive bursts

  std::vector<Thread *> mRxServiceLoopThreads;  ///< thread to pull bursts into receive FIFO
  Thread *mLowerLoopThread;                   ///< thread to pull bursts into receive FIFO
  std::vector<Thread *> mControlServiceLoopThreads;         ///< thread to process control messages from GSM core
  std::vector<Thread *> mTxPriorityQueueServiceLoopThreads; ///< thread to process transmit bursts from GSM core

  GSM::Time mTransmitDeadlineClock;       ///< deadline for pushing bursts into transmit FIFO 
  GSM::Time mLastClockUpdateTime;         ///< last time clock update was sent up to core

  RadioInterface *mRadioInterface;	  ///< associated radioInterface object
  double txFullScale;                     ///< full scale input to radio
  double rxFullScale;                     ///< full scale output to radio

  /** modulate and add a burst to the transmit queue */
  void addRadioVector(size_t chan, BitVector &bits,
                      int RSSI, GSM::Time &wTime);

  /** Update filler table */
  void updateFillerTable(size_t chan, radioVector *burst);

  /** Push modulated burst into transmit FIFO corresponding to a particular timestamp */
  void pushRadioVector(GSM::Time &nowTime);

  /** Pull and demodulate a burst from the receive FIFO */
  SoftVector *pullRadioVector(GSM::Time &wTime, int &RSSI,
                              int &timingOffset, size_t chan = 0);

  /** Set modulus for specific timeslot */
  void setModulus(size_t timeslot, size_t chan);

  /** return the expected burst type for the specified timestamp */
  CorrType expectedCorrType(GSM::Time currTime, size_t chan);

  /** send messages over the clock socket */
  void writeClockInterface(void);


  bool detectSCH(TransceiverState *state,
                  signalVector &burst,
                  struct estim_burst_params *ebp);

  bool decodeSCH(SoftVector *burst, GSM::Time *time);
  bool correctFCCH(TransceiverState *state, signalVector *burst);

  size_t mChans;

  bool mOn;			       ///< flag to indicate that transceiver is powered on
  double mTxFreq;                      ///< the transmit frequency
  double mRxFreq;                      ///< the receive frequency
  int mPower;                          ///< the transmit power in dB
  unsigned mTSC;                       ///< the midamble sequence code
  unsigned mMaxExpectedDelay;          ///< maximum TOA offset in GSM symbols
  unsigned long long mRxSlotMask[8];   ///< MS - enabled multiframe slot mask
  int mBSIC;                           ///< MS - detected BSIC

  std::vector<TransceiverState> mStates;

public:

  /** Transceiver constructor 
      @param wBasePort base port number of UDP sockets
      @param TRXAddress IP address of the TRX manager, as a string
      @param wSPS number of samples per GSM symbol
      @param wTransmitLatency initial setting of transmit latency
      @param radioInterface associated radioInterface object
  */
  Transceiver2(int wBasePort,
	      const char *TRXAddress,
	      size_t wSPS, size_t chans,
	      GSM::Time wTransmitLatency,
	      RadioInterface *wRadioInterface);

  /** Destructor */
  ~Transceiver2();

  /** start the Transceiver */
  void start();
  bool init(bool filler);

  /** attach the radioInterface receive FIFO */
  bool receiveFIFO(VectorFIFO *wFIFO, size_t chan)
  {
    if (chan >= mReceiveFIFO.size())
      return false;

    mReceiveFIFO[chan] = wFIFO;
    return true;
  }

  /** accessor for number of channels */
  size_t numChans() const { return mChans; };

  /** Codes for channel combinations */
  typedef enum {
    FILL,               ///< Channel is transmitted, but unused
    I,                  ///< TCH/FS
    II,                 ///< TCH/HS, idle every other slot
    III,                ///< TCH/HS
    IV,                 ///< FCCH+SCH+CCCH+BCCH, uplink RACH
    V,                  ///< FCCH+SCH+CCCH+BCCH+SDCCH/4+SACCH/4, uplink RACH+SDCCH/4
    VI,                 ///< CCCH+BCCH, uplink RACH
    VII,                ///< SDCCH/8 + SACCH/8
    VIII,               ///< TCH/F + FACCH/F + SACCH/M
    IX,                 ///< TCH/F + SACCH/M
    X,                  ///< TCH/FD + SACCH/MD
    XI,                 ///< PBCCH+PCCCH+PDTCH+PACCH+PTCCH
    XII,                ///< PCCCH+PDTCH+PACCH+PTCCH
    XIII,               ///< PDTCH+PACCH+PTCCH
    NONE,               ///< Channel is inactive, default
    LOOPBACK            ///< similar go VII, used in loopback testing
  } ChannelCombination;

  enum {
    TRX_MODE_OFF,
    TRX_MODE_BTS,
    TRX_MODE_MS_ACQUIRE,
    TRX_MODE_MS_TRACK,
  };

  void commandhandler(char *buffer, char *response, int chan);
  void stop();
protected:
  /** drive lower receive I/O and burst generation */
  void driveReceiveRadio();

  /** drive demodulation of GSM bursts */
  void driveReceiveFIFO(size_t chan);

  /** drive transmission of GSM bursts */
  void driveTxFIFO();

  /** drive handling of control messages from GSM core */
  void driveControl(size_t chan);

  /**
    drive modulation and sorting of GSM bursts from GSM core
    @return true if a burst was transferred successfully
  */
  bool driveTxPriorityQueue(size_t chan);

  friend void *RxUpperLoopAdapter(TransceiverChannel *);

  friend void *TxUpperLoopAdapter(TransceiverChannel *);

  friend void *LowerLoopAdapter(Transceiver2 *);

  friend void *ControlServiceLoopAdapter(TransceiverChannel *);


  void reset();

  /** set priority on current thread */
  //void setPriority(float prio = 0.5) { mRadioInterface->setPriority(prio); }

};

void *RxUpperLoopAdapter(TransceiverChannel *);

/** Main drive threads */
void *LowerLoopAdapter(Transceiver2 *);

/** control message handler thread loop */
void *ControlServiceLoopAdapter(TransceiverChannel *);

/** transmit queueing thread loop */
void *TxUpperLoopAdapter(TransceiverChannel *);

