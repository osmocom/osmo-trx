/*
* Copyright 2008 Free Software Foundation, Inc.
*
* SPDX-License-Identifier: GPL-3.0+
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
#include "config_defs.h"
}

class Transceiver;

/** Channel descriptor for transceiver object and channel number pair */
struct TrxChanThParams {
	Transceiver *trx;
	size_t num;
};

/** Internal transceiver state variables */
struct TransceiverState {
  TransceiverState();
  ~TransceiverState();

  /* Initialize a multiframe slot in the filler table */
  bool init(FillerType filler, size_t sps, float scale, size_t rtsc, unsigned rach_delay);

  int chanType[8];

  /* Last timestamp of each timeslot's channel estimate */
  GSM::Time chanEstimateTime[8];

  /* The filler table */
  signalVector *fillerTable[102][8];
  int fillerModulus[8];
  bool mRetrans;

  /* Most recent channel estimate of all timeslots */
  signalVector *chanResponse[8];

  /* Most recent DFE feedback filter of all timeslots */
  signalVector *DFEForward[8];
  signalVector *DFEFeedback[8];

  /* Most recent SNR, timing, and channel amplitude estimates */
  float SNRestimate[8];
  float chanRespOffset[8];
  complex chanRespAmplitude[8];

  /* Received noise energy levels */
  float mNoiseLev;
  noiseVector mNoises;

  /* Shadowed downlink attenuation */
  int mPower;
};

/** The Transceiver class, responsible for physical layer of basestation */
class Transceiver {
public:
  /** Transceiver constructor
      @param wBasePort base port number of UDP sockets
      @param TRXAddress IP address of the TRX, as a string
      @param GSMcoreAddress IP address of the GSM core, as a string
      @param wSPS number of samples per GSM symbol
      @param wTransmitLatency initial setting of transmit latency
      @param radioInterface associated radioInterface object
  */
  Transceiver(int wBasePort,
              const char *TRXAddress,
              const char *GSMcoreAddress,
              size_t tx_sps, size_t rx_sps, size_t chans,
              GSM::Time wTransmitLatency,
              RadioInterface *wRadioInterface,
              double wRssiOffset, int stackSize);

  /** Destructor */
  ~Transceiver();

  /** Start the control loop */
  bool init(FillerType filler, size_t rtsc, unsigned rach_delay,
            bool edge, bool ext_rach);

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

private:
  int mBasePort;
  std::string mLocalAddr;
  std::string mRemoteAddr;

  std::vector<int> mDataSockets;  ///< socket for writing to/reading from GSM core
  std::vector<int> mCtrlSockets;  ///< socket for writing/reading control commands from GSM core
  int mClockSocket;               ///< socket for writing clock updates to GSM core

  std::vector<VectorQueue> mTxPriorityQueues;   ///< priority queue of transmit bursts received from GSM core
  std::vector<VectorFIFO *>  mReceiveFIFO;      ///< radioInterface FIFO of receive bursts

  std::vector<Thread *> mRxServiceLoopThreads;  ///< thread to pull bursts into receive FIFO
  Thread *mRxLowerLoopThread;                   ///< thread to pull bursts into receive FIFO
  Thread *mTxLowerLoopThread;                   ///< thread to push bursts into transmit FIFO
  std::vector<Thread *> mControlServiceLoopThreads;         ///< thread to process control messages from GSM core
  std::vector<Thread *> mTxPriorityQueueServiceLoopThreads; ///< thread to process transmit bursts from GSM core

  GSM::Time mTransmitLatency;             ///< latency between basestation clock and transmit deadline clock
  GSM::Time mLatencyUpdateTime;           ///< last time latency was updated
  GSM::Time mTransmitDeadlineClock;       ///< deadline for pushing bursts into transmit FIFO
  GSM::Time mLastClockUpdateTime;         ///< last time clock update was sent up to core

  RadioInterface *mRadioInterface;	  ///< associated radioInterface object
  double txFullScale;                     ///< full scale input to radio
  double rxFullScale;                     ///< full scale output to radio

  double rssiOffset;                      ///< RSSI to dBm conversion offset
  int stackSize;                      ///< stack size for threads, 0 = OS default

  /** modulate and add a burst to the transmit queue */
  void addRadioVector(size_t chan, BitVector &bits,
                      int RSSI, GSM::Time &wTime);

  /** Update filler table */
  void updateFillerTable(size_t chan, radioVector *burst);

  /** Push modulated burst into transmit FIFO corresponding to a particular timestamp */
  void pushRadioVector(GSM::Time &nowTime);

  /** Pull and demodulate a burst from the receive FIFO */
  int pullRadioVector(size_t chan, struct trx_ul_burst_ind *ind);

  /** Set modulus for specific timeslot */
  void setModulus(size_t timeslot, size_t chan);

  /** return the expected burst type for the specified timestamp */
  CorrType expectedCorrType(GSM::Time currTime, size_t chan);

  /** send messages over the clock socket */
  bool writeClockInterface(void);

  int mSPSTx;                          ///< number of samples per Tx symbol
  int mSPSRx;                          ///< number of samples per Rx symbol
  size_t mChans;

  bool mExtRACH;
  bool mEdge;
  bool mOn;	                           ///< flag to indicate that transceiver is powered on
  bool mForceClockInterface;           ///< flag to indicate whether IND CLOCK shall be sent unconditionally after transceiver is started
  bool mHandover[8][8];                ///< expect handover to the timeslot/subslot
  double mTxFreq;                      ///< the transmit frequency
  double mRxFreq;                      ///< the receive frequency
  unsigned mTSC;                       ///< the midamble sequence code
  unsigned mMaxExpectedDelayAB;        ///< maximum expected time-of-arrival offset in GSM symbols for Access Bursts (RACH)
  unsigned mMaxExpectedDelayNB;        ///< maximum expected time-of-arrival offset in GSM symbols for Normal Bursts
  unsigned mWriteBurstToDiskMask;      ///< debug: bitmask to indicate which timeslots to dump to disk

  std::vector<unsigned> mVersionTRXD;  ///< Format version to use for TRXD protocol communication, per channel
  std::vector<TransceiverState> mStates;

  /** Start and stop I/O threads through the control socket API */
  bool start();
  void stop();

  /** Protect destructor accessible stop call */
  Mutex mLock;

protected:
  /** drive lower receive I/O and burst generation */
  bool driveReceiveRadio();

  /** drive demodulation of GSM bursts */
  bool driveReceiveFIFO(size_t chan);

  /** drive transmission of GSM bursts */
  void driveTxFIFO();

  /** drive handling of control messages from GSM core */
  bool driveControl(size_t chan);

  /**
    drive modulation and sorting of GSM bursts from GSM core
    @return true if a burst was transferred successfully
  */
  bool driveTxPriorityQueue(size_t chan);

  friend void *RxUpperLoopAdapter(TrxChanThParams *params);
  friend void *TxUpperLoopAdapter(TrxChanThParams *params);
  friend void *RxLowerLoopAdapter(Transceiver *transceiver);
  friend void *TxLowerLoopAdapter(Transceiver *transceiver);
  friend void *ControlServiceLoopAdapter(TrxChanThParams *params);


  void reset();

  void logRxBurst(size_t chan, const struct trx_ul_burst_ind *bi);
};

void *RxUpperLoopAdapter(TrxChanThParams *params);

/** Main drive threads */
void *RxLowerLoopAdapter(Transceiver *transceiver);
void *TxLowerLoopAdapter(Transceiver *transceiver);

/** control message handler thread loop */
void *ControlServiceLoopAdapter(TrxChanThParams *params);

/** transmit queueing thread loop */
void *TxUpperLoopAdapter(TrxChanThParams *params);
