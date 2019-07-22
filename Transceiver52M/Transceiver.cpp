/*
* Copyright 2008, 2009, 2010 Free Software Foundation, Inc.
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

#include <stdio.h>
#include <iomanip>      // std::setprecision
#include <fstream>
#include "Transceiver.h"
#include <Logger.h>

extern "C" {
#include "osmo_signal.h"
#include "proto_trxd.h"

#include <osmocom/core/utils.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/bits.h>
}

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

using namespace GSM;

#define USB_LATENCY_INTRVL		10,0

/* Number of running values use in noise average */
#define NOISE_CNT			20

TransceiverState::TransceiverState()
  : mRetrans(false), mNoiseLev(0.0), mNoises(NOISE_CNT), mPower(0.0)
{
  for (int i = 0; i < 8; i++) {
    chanType[i] = Transceiver::NONE;
    fillerModulus[i] = 26;
    chanResponse[i] = NULL;
    DFEForward[i] = NULL;
    DFEFeedback[i] = NULL;

    for (int n = 0; n < 102; n++)
      fillerTable[n][i] = NULL;
  }
}

TransceiverState::~TransceiverState()
{
  for (int i = 0; i < 8; i++) {
    delete chanResponse[i];
    delete DFEForward[i];
    delete DFEFeedback[i];

    for (int n = 0; n < 102; n++)
      delete fillerTable[n][i];
  }
}

bool TransceiverState::init(FillerType filler, size_t sps, float scale, size_t rtsc, unsigned rach_delay)
{
  signalVector *burst;

  if ((sps != 1) && (sps != 4))
    return false;

  for (size_t n = 0; n < 8; n++) {
    for (size_t i = 0; i < 102; i++) {
      switch (filler) {
      case FILLER_DUMMY:
        burst = generateDummyBurst(sps, n);
        break;
      case FILLER_NORM_RAND:
        burst = genRandNormalBurst(rtsc, sps, n);
        break;
      case FILLER_EDGE_RAND:
        burst = generateEdgeBurst(rtsc);
        break;
      case FILLER_ACCESS_RAND:
        burst = genRandAccessBurst(rach_delay, sps, n);
        break;
      case FILLER_ZERO:
      default:
        burst = generateEmptyBurst(sps, n);
      }

      scaleVector(*burst, scale);
      fillerTable[i][n] = burst;
    }

    if ((filler == FILLER_NORM_RAND) ||
        (filler == FILLER_EDGE_RAND)) {
        chanType[n] = TSC;
    }
  }

  return false;
}

Transceiver::Transceiver(int wBasePort,
                         const char *TRXAddress,
                         const char *GSMcoreAddress,
                         size_t tx_sps, size_t rx_sps, size_t chans,
                         GSM::Time wTransmitLatency,
                         RadioInterface *wRadioInterface,
                         double wRssiOffset, int wStackSize)
  : mBasePort(wBasePort), mLocalAddr(TRXAddress), mRemoteAddr(GSMcoreAddress),
    mClockSocket(-1), mTransmitLatency(wTransmitLatency), mRadioInterface(wRadioInterface),
    rssiOffset(wRssiOffset), stackSize(wStackSize),
    mSPSTx(tx_sps), mSPSRx(rx_sps), mChans(chans), mExtRACH(false), mEdge(false),
    mOn(false), mForceClockInterface(false),
    mTxFreq(0.0), mRxFreq(0.0), mTSC(0), mMaxExpectedDelayAB(0), mMaxExpectedDelayNB(0),
    mWriteBurstToDiskMask(0)
{
  txFullScale = mRadioInterface->fullScaleInputValue();
  rxFullScale = mRadioInterface->fullScaleOutputValue();

  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++)
      mHandover[i][j] = false;
  }
}

Transceiver::~Transceiver()
{
  stop();

  sigProcLibDestroy();

  if (mClockSocket >= 0)
    close(mClockSocket);

  for (size_t i = 0; i < mChans; i++) {
    mControlServiceLoopThreads[i]->cancel();
    mControlServiceLoopThreads[i]->join();
    delete mControlServiceLoopThreads[i];

    mTxPriorityQueues[i].clear();
    if (mCtrlSockets[i] >= 0)
      close(mCtrlSockets[i]);
    if (mDataSockets[i] >= 0)
      close(mDataSockets[i]);
  }
}

/*
 * Initialize transceiver
 *
 * Start or restart the control loop. Any further control is handled through the
 * socket API. Randomize the central radio clock set the downlink burst
 * counters. Note that the clock will not update until the radio starts, but we
 * are still expected to report clock indications through control channel
 * activity.
 */
bool Transceiver::init(FillerType filler, size_t rtsc, unsigned rach_delay,
                       bool edge, bool ext_rach)
{
  int d_srcport, d_dstport, c_srcport, c_dstport;

  if (!mChans) {
    LOG(ALERT) << "No channels assigned";
    return false;
  }

  if (!sigProcLibSetup()) {
    LOG(ALERT) << "Failed to initialize signal processing library";
    return false;
  }

  mExtRACH = ext_rach;
  mEdge = edge;

  mDataSockets.resize(mChans, -1);
  mCtrlSockets.resize(mChans, -1);
  mControlServiceLoopThreads.resize(mChans);
  mTxPriorityQueueServiceLoopThreads.resize(mChans);
  mRxServiceLoopThreads.resize(mChans);

  mTxPriorityQueues.resize(mChans);
  mReceiveFIFO.resize(mChans);
  mStates.resize(mChans);
  mVersionTRXD.resize(mChans);

  /* Filler table retransmissions - support only on channel 0 */
  if (filler == FILLER_DUMMY)
    mStates[0].mRetrans = true;

  /* Setup sockets */
  mClockSocket = osmo_sock_init2(AF_UNSPEC, SOCK_DGRAM, IPPROTO_UDP,
				    mLocalAddr.c_str(), mBasePort,
				    mRemoteAddr.c_str(), mBasePort + 100,
				    OSMO_SOCK_F_BIND | OSMO_SOCK_F_CONNECT);

  for (size_t i = 0; i < mChans; i++) {
    c_srcport = mBasePort + 2 * i + 1;
    c_dstport = mBasePort + 2 * i + 101;
    d_srcport = mBasePort + 2 * i + 2;
    d_dstport = mBasePort + 2 * i + 102;

    mCtrlSockets[i] = osmo_sock_init2(AF_UNSPEC, SOCK_DGRAM, IPPROTO_UDP,
                                      mLocalAddr.c_str(), c_srcport,
                                      mRemoteAddr.c_str(), c_dstport,
				      OSMO_SOCK_F_BIND | OSMO_SOCK_F_CONNECT);
    if (mCtrlSockets[i] < 0)
      return false;

    mDataSockets[i] = osmo_sock_init2(AF_UNSPEC, SOCK_DGRAM, IPPROTO_UDP,
                                      mLocalAddr.c_str(), d_srcport,
                                      mRemoteAddr.c_str(), d_dstport,
				      OSMO_SOCK_F_BIND | OSMO_SOCK_F_CONNECT);
    if (mCtrlSockets[i] < 0)
      return false;
  }

  /* Randomize the central clock */
  GSM::Time startTime(random() % gHyperframe, 0);
  mRadioInterface->getClock()->set(startTime);
  mTransmitDeadlineClock = startTime;
  mLastClockUpdateTime = startTime;
  mLatencyUpdateTime = startTime;

  /* Start control threads */
  for (size_t i = 0; i < mChans; i++) {
    TransceiverChannel *chan = new TransceiverChannel(this, i);
    mControlServiceLoopThreads[i] = new Thread(stackSize);
    mControlServiceLoopThreads[i]->start((void * (*)(void*))
                                 ControlServiceLoopAdapter, (void*) chan);

    if (i && filler == FILLER_DUMMY)
      filler = FILLER_ZERO;

    mStates[i].init(filler, mSPSTx, txFullScale, rtsc, rach_delay);
  }

  return true;
}

/*
 * Start the transceiver
 *
 * Submit command(s) to the radio device to commence streaming samples and
 * launch threads to handle sample I/O. Re-synchronize the transmit burst
 * counters to the central radio clock here as well.
 */
bool Transceiver::start()
{
  ScopedLock lock(mLock);

  if (mOn) {
    LOG(ERR) << "Transceiver already running";
    return true;
  }

  LOG(NOTICE) << "Starting the transceiver";

  GSM::Time time = mRadioInterface->getClock()->get();
  mTransmitDeadlineClock = time;
  mLastClockUpdateTime = time;
  mLatencyUpdateTime = time;

  if (!mRadioInterface->start()) {
    LOG(ALERT) << "Device failed to start";
    return false;
  }

  /* Device is running - launch I/O threads */
  mRxLowerLoopThread = new Thread(stackSize);
  mTxLowerLoopThread = new Thread(stackSize);
  mTxLowerLoopThread->start((void * (*)(void*))
                            TxLowerLoopAdapter,(void*) this);
  mRxLowerLoopThread->start((void * (*)(void*))
                            RxLowerLoopAdapter,(void*) this);

  /* Launch uplink and downlink burst processing threads */
  for (size_t i = 0; i < mChans; i++) {
    TransceiverChannel *chan = new TransceiverChannel(this, i);
    mRxServiceLoopThreads[i] = new Thread(stackSize);
    mRxServiceLoopThreads[i]->start((void * (*)(void*))
                            RxUpperLoopAdapter, (void*) chan);

    chan = new TransceiverChannel(this, i);
    mTxPriorityQueueServiceLoopThreads[i] = new Thread(stackSize);
    mTxPriorityQueueServiceLoopThreads[i]->start((void * (*)(void*))
                            TxUpperLoopAdapter, (void*) chan);
  }

  mForceClockInterface = true;
  mOn = true;
  return true;
}

/*
 * Stop the transceiver
 *
 * Perform stopping by disabling receive streaming and issuing cancellation
 * requests to running threads. Most threads will timeout and terminate once
 * device is disabled, but the transmit loop may block waiting on the central
 * UMTS clock. Explicitly signal the clock to make sure that the transmit loop
 * makes it to the thread cancellation point.
 */
void Transceiver::stop()
{
  ScopedLock lock(mLock);

  if (!mOn)
    return;

  LOG(NOTICE) << "Stopping the transceiver";
  mTxLowerLoopThread->cancel();
  mRxLowerLoopThread->cancel();
  mTxLowerLoopThread->join();
  mRxLowerLoopThread->join();
  delete mTxLowerLoopThread;
  delete mRxLowerLoopThread;

  for (size_t i = 0; i < mChans; i++) {
    mRxServiceLoopThreads[i]->cancel();
    mTxPriorityQueueServiceLoopThreads[i]->cancel();
  }

  LOG(INFO) << "Stopping the device";
  mRadioInterface->stop();

  for (size_t i = 0; i < mChans; i++) {
    mRxServiceLoopThreads[i]->join();
    mTxPriorityQueueServiceLoopThreads[i]->join();
    delete mRxServiceLoopThreads[i];
    delete mTxPriorityQueueServiceLoopThreads[i];

    mTxPriorityQueues[i].clear();
  }

  mOn = false;
  LOG(NOTICE) << "Transceiver stopped";
}

void Transceiver::addRadioVector(size_t chan, BitVector &bits,
                                 int RSSI, GSM::Time &wTime)
{
  signalVector *burst;
  radioVector *radio_burst;

  if (chan >= mTxPriorityQueues.size()) {
    LOG(ALERT) << "Invalid channel " << chan;
    return;
  }

  if (wTime.TN() > 7) {
    LOG(ALERT) << "Received burst with invalid slot " << wTime.TN();
    return;
  }

  /* Use the number of bits as the EDGE burst indicator */
  if (bits.size() == EDGE_BURST_NBITS)
    burst = modulateEdgeBurst(bits, mSPSTx);
  else
    burst = modulateBurst(bits, 8 + (wTime.TN() % 4 == 0), mSPSTx);

  scaleVector(*burst, txFullScale * pow(10, -RSSI / 10));

  radio_burst = new radioVector(wTime, burst);

  mTxPriorityQueues[chan].write(radio_burst);
}

void Transceiver::updateFillerTable(size_t chan, radioVector *burst)
{
  int TN, modFN;
  TransceiverState *state = &mStates[chan];

  TN = burst->getTime().TN();
  modFN = burst->getTime().FN() % state->fillerModulus[TN];

  delete state->fillerTable[modFN][TN];
  state->fillerTable[modFN][TN] = burst->getVector();
  burst->setVector(NULL);
}

void Transceiver::pushRadioVector(GSM::Time &nowTime)
{
  int TN, modFN;
  radioVector *burst;
  TransceiverState *state;
  std::vector<signalVector *> bursts(mChans);
  std::vector<bool> zeros(mChans);
  std::vector<bool> filler(mChans, true);

  for (size_t i = 0; i < mChans; i ++) {
    state = &mStates[i];

    while ((burst = mTxPriorityQueues[i].getStaleBurst(nowTime))) {
      LOGCHAN(i, DMAIN, NOTICE) << "dumping STALE burst in TRX->SDR interface ("
                  << burst->getTime() <<" vs " << nowTime << "), retrans=" << state->mRetrans;
      if (state->mRetrans)
        updateFillerTable(i, burst);
      delete burst;
    }

    TN = nowTime.TN();
    modFN = nowTime.FN() % state->fillerModulus[TN];

    bursts[i] = state->fillerTable[modFN][TN];
    zeros[i] = state->chanType[TN] == NONE;

    if ((burst = mTxPriorityQueues[i].getCurrentBurst(nowTime))) {
      bursts[i] = burst->getVector();

      if (state->mRetrans) {
        updateFillerTable(i, burst);
      } else {
        burst->setVector(NULL);
        filler[i] = false;
      }

      delete burst;
    }
  }

  mRadioInterface->driveTransmitRadio(bursts, zeros);

  for (size_t i = 0; i < mChans; i++) {
    if (!filler[i])
      delete bursts[i];
  }
}

void Transceiver::setModulus(size_t timeslot, size_t chan)
{
  TransceiverState *state = &mStates[chan];

  switch (state->chanType[timeslot]) {
  case NONE:
  case I:
  case II:
  case III:
  case FILL:
    state->fillerModulus[timeslot] = 26;
    break;
  case IV:
  case VI:
  case V:
    state->fillerModulus[timeslot] = 51;
    break;
    //case V:
  case VII:
    state->fillerModulus[timeslot] = 102;
    break;
  case XIII:
    state->fillerModulus[timeslot] = 52;
    break;
  default:
    break;
  }
}


CorrType Transceiver::expectedCorrType(GSM::Time currTime,
                                       size_t chan)
{
  static int tchh_subslot[26] = { 0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,1 };
  static int sdcch4_subslot[102] = { 3,3,3,3,0,0,2,2,2,2,3,3,3,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,2,2,2,2,
                                     3,3,3,3,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,2,2,2,2 };
  static int sdcch8_subslot[102] = { 5,5,5,5,6,6,6,6,7,7,7,7,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,0,0,0,0,
                                     1,1,1,1,2,2,2,2,3,3,3,3,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,4,4,4,4 };
  TransceiverState *state = &mStates[chan];
  unsigned burstTN = currTime.TN();
  unsigned burstFN = currTime.FN();
  int subch;

  switch (state->chanType[burstTN]) {
  case NONE:
    return OFF;
    break;
  case FILL:
    return IDLE;
    break;
  case I:
    // TODO: Are we expecting RACH on an IDLE frame?
/*    if (burstFN % 26 == 25)
      return IDLE;*/
    if (mHandover[burstTN][0])
      return RACH;
    return TSC;
    break;
  case II:
    subch = tchh_subslot[burstFN % 26];
    if (subch == 1)
      return IDLE;
    if (mHandover[burstTN][0])
      return RACH;
    return TSC;
    break;
  case III:
    subch = tchh_subslot[burstFN % 26];
    if (mHandover[burstTN][subch])
      return RACH;
    return TSC;
    break;
  case IV:
  case VI:
    return mExtRACH ? EXT_RACH : RACH;
    break;
  case V: {
    int mod51 = burstFN % 51;
    if ((mod51 <= 36) && (mod51 >= 14))
      return mExtRACH ? EXT_RACH : RACH;
    else if ((mod51 == 4) || (mod51 == 5))
      return mExtRACH ? EXT_RACH : RACH;
    else if ((mod51 == 45) || (mod51 == 46))
      return mExtRACH ? EXT_RACH : RACH;
    else if (mHandover[burstTN][sdcch4_subslot[burstFN % 102]])
      return RACH;
    else
      return TSC;
    break;
  }
  case VII:
    if ((burstFN % 51 <= 14) && (burstFN % 51 >= 12))
      return IDLE;
    else if (mHandover[burstTN][sdcch8_subslot[burstFN % 102]])
      return RACH;
    else
      return TSC;
    break;
  case XIII: {
    int mod52 = burstFN % 52;
    if ((mod52 == 12) || (mod52 == 38))
      return mExtRACH ? EXT_RACH : RACH;
    else if ((mod52 == 25) || (mod52 == 51))
      return IDLE;
    else
      return TSC;
    break;
  }
  case LOOPBACK:
    if ((burstFN % 51 <= 50) && (burstFN % 51 >=48))
      return IDLE;
    else
      return TSC;
    break;
  default:
    return OFF;
    break;
  }
}

void writeToFile(radioVector *radio_burst, size_t chan)
{
  GSM::Time time = radio_burst->getTime();
  std::ostringstream fname;
  fname << chan << "_" << time.FN() << "_" << time.TN() << ".fc";
  std::ofstream outfile (fname.str().c_str(), std::ofstream::binary);
  outfile.write((char*)radio_burst->getVector()->begin(), radio_burst->getVector()->size() * 2 * sizeof(float));
  outfile.close();
}

/*
 * Pull bursts from the FIFO and handle according to the slot
 * and burst correlation type. Equalzation is currently disabled.
 */
bool Transceiver::pullRadioVector(size_t chan, struct trx_ul_burst_ind *bi)
{
  int rc;
  struct estim_burst_params ebp;
  float max = -1.0, avg = 0.0;
  unsigned max_toa;
  int max_i = -1;
  signalVector *burst;
  GSM::Time burstTime;
  SoftVector *rxBurst;
  TransceiverState *state = &mStates[chan];

  /* Blocking FIFO read */
  radioVector *radio_burst = mReceiveFIFO[chan]->read();
  if (!radio_burst)
    return false;

  /* Set time and determine correlation type */
  burstTime = radio_burst->getTime();
  CorrType type = expectedCorrType(burstTime, chan);

  /* Enable 8-PSK burst detection if EDGE is enabled */
  if (mEdge && (type == TSC))
    type = EDGE;

  /* Debug: dump bursts to disk */
  /* bits 0-7  - chan 0 timeslots
   * bits 8-15 - chan 1 timeslots */
  if (mWriteBurstToDiskMask & ((1<<bi->tn) << (8*chan)))
    writeToFile(radio_burst, chan);

  /* No processing if the timeslot is off.
   * Not even power level or noise calculation. */
  if (type == OFF) {
    delete radio_burst;
    return false;
  }

  /* Initialize struct bi */
  bi->nbits = 0;
  bi->fn = burstTime.FN();
  bi->tn = burstTime.TN();
  bi->rssi = 0.0;
  bi->toa = 0.0;
  bi->noise = 0.0;
  bi->idle = false;
  bi->modulation = MODULATION_GMSK;
  bi->tss = 0; /* TODO: we only support tss 0 right now */
  bi->tsc = 0;
  bi->ci = 0.0;

  /* Select the diversity channel with highest energy */
  for (size_t i = 0; i < radio_burst->chans(); i++) {
    float pow = energyDetect(*radio_burst->getVector(i), 20 * mSPSRx);
    if (pow > max) {
      max = pow;
      max_i = i;
    }
    avg += pow;
  }

  if (max_i < 0) {
    LOG(ALERT) << "Received empty burst";
    goto ret_idle;
  }

  /* Average noise on diversity paths and update global levels */
  burst = radio_burst->getVector(max_i);
  avg = sqrt(avg / radio_burst->chans());

  if (type == IDLE) {
    /* Update noise levels */
    state->mNoises.insert(avg);
    state->mNoiseLev = state->mNoises.avg();
  }

  bi->rssi = 20.0 * log10(rxFullScale / avg) + rssiOffset;
  bi->noise = 20.0 * log10(rxFullScale / state->mNoiseLev) + rssiOffset;

  if (type == IDLE)
    goto ret_idle;

  max_toa = (type == RACH || type == EXT_RACH) ?
            mMaxExpectedDelayAB : mMaxExpectedDelayNB;

  /* Detect normal or RACH bursts */
  rc = detectAnyBurst(*burst, mTSC, BURST_THRESH, mSPSRx, type, max_toa, &ebp);
  if (rc <= 0) {
    if (rc == -SIGERR_CLIP)
      LOG(WARNING) << "Clipping detected on received RACH or Normal Burst";
    else if (rc != SIGERR_NONE)
      LOG(WARNING) << "Unhandled RACH or Normal Burst detection error";
    goto ret_idle;
  }

  type = (CorrType) rc;
  bi->toa = ebp.toa;
  bi->tsc = ebp.tsc;
  bi->ci = ebp.ci;
  rxBurst = demodAnyBurst(*burst, mSPSRx, ebp.amp, ebp.toa, type);

  /* EDGE demodulator returns 444 (gSlotLen * 3) bits */
  if (rxBurst->size() == EDGE_BURST_NBITS) {
    bi->modulation = MODULATION_8PSK;
    bi->nbits = EDGE_BURST_NBITS;
  } else { /* size() here is actually gSlotLen + 8, due to guard periods */
    bi->modulation = MODULATION_GMSK;
    bi->nbits = gSlotLen;
  }

  // Convert -1..+1 soft bits to 0..1 soft bits
  vectorSlicer(bi->rx_burst, rxBurst->begin(), bi->nbits);

  delete rxBurst;
  delete radio_burst;
  return true;

ret_idle:
  bi->idle = true;
  delete radio_burst;
  return false;
}

void Transceiver::reset()
{
  for (size_t i = 0; i < mTxPriorityQueues.size(); i++)
    mTxPriorityQueues[i].clear();
}


#define MAX_PACKET_LENGTH 100

/**
 * Matches a buffer with a command.
 * @param  buf    a buffer to look command in
 * @param  cmd    a command to look in buffer
 * @param  params pointer to arguments, or NULL
 * @return        true if command matches, otherwise false
 */
static bool match_cmd(char *buf,
  const char *cmd, char **params)
{
  size_t cmd_len = strlen(cmd);

  /* Check a command itself */
  if (strncmp(buf, cmd, cmd_len))
    return false;

  /* A command has arguments */
  if (params != NULL) {
    /* Make sure there is a space */
    if (buf[cmd_len] != ' ')
      return false;

    /* Update external pointer */
    *params = buf + cmd_len + 1;
  }

  return true;
}

void Transceiver::driveControl(size_t chan)
{
  char buffer[MAX_PACKET_LENGTH + 1];
  char response[MAX_PACKET_LENGTH + 1];
  char *command, *params;
  int msgLen;

  /* Attempt to read from control socket */
  msgLen = read(mCtrlSockets[chan], buffer, MAX_PACKET_LENGTH);
  if (msgLen <= 0) {
    LOGCHAN(chan, DTRXCTRL, WARNING) << "mCtrlSockets read(" << mCtrlSockets[chan] << ") failed: " << msgLen;
    return;
  }

  /* Zero-terminate received string */
  buffer[msgLen] = '\0';

  /* Verify a command signature */
  if (strncmp(buffer, "CMD ", 4)) {
    LOGC(DTRXCTRL, WARNING) << "bogus message on control interface";
    return;
  }

  /* Set command pointer */
  command = buffer + 4;
  LOGCHAN(chan, DTRXCTRL, INFO) << "command is '" << command << "'";

  if (match_cmd(command, "POWEROFF", NULL)) {
    stop();
    sprintf(response,"RSP POWEROFF 0");
  } else if (match_cmd(command, "POWERON", NULL)) {
    if (!start()) {
      sprintf(response,"RSP POWERON 1");
    } else {
      sprintf(response,"RSP POWERON 0");
      for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++)
          mHandover[i][j] = false;
      }
    }
  } else if (match_cmd(command, "HANDOVER", &params)) {
    unsigned ts = 0, ss = 0;
    sscanf(params, "%u %u", &ts, &ss);
    if (ts > 7 || ss > 7) {
      sprintf(response, "RSP NOHANDOVER 1 %u %u", ts, ss);
    } else {
      mHandover[ts][ss] = true;
      sprintf(response, "RSP HANDOVER 0 %u %u", ts, ss);
    }
  } else if (match_cmd(command, "NOHANDOVER", &params)) {
    unsigned ts = 0, ss = 0;
    sscanf(params, "%u %u", &ts, &ss);
    if (ts > 7 || ss > 7) {
      sprintf(response, "RSP NOHANDOVER 1 %u %u", ts, ss);
    } else {
      mHandover[ts][ss] = false;
      sprintf(response, "RSP NOHANDOVER 0 %u %u", ts, ss);
    }
  } else if (match_cmd(command, "SETMAXDLY", &params)) {
    //set expected maximum time-of-arrival
    int maxDelay;
    sscanf(params, "%d", &maxDelay);
    mMaxExpectedDelayAB = maxDelay; // 1 GSM symbol is approx. 1 km
    sprintf(response,"RSP SETMAXDLY 0 %d",maxDelay);
  } else if (match_cmd(command, "SETMAXDLYNB", &params)) {
    //set expected maximum time-of-arrival
    int maxDelay;
    sscanf(params, "%d", &maxDelay);
    mMaxExpectedDelayNB = maxDelay; // 1 GSM symbol is approx. 1 km
    sprintf(response,"RSP SETMAXDLYNB 0 %d",maxDelay);
  } else if (match_cmd(command, "SETRXGAIN", &params)) {
    //set expected maximum time-of-arrival
    int newGain;
    sscanf(params, "%d", &newGain);
    newGain = mRadioInterface->setRxGain(newGain, chan);
    sprintf(response,"RSP SETRXGAIN 0 %d",newGain);
  } else if (match_cmd(command, "NOISELEV", NULL)) {
    if (mOn) {
      float lev = mStates[chan].mNoiseLev;
      sprintf(response,"RSP NOISELEV 0 %d",
              (int) round(20.0 * log10(rxFullScale / lev)));
    }
    else {
      sprintf(response,"RSP NOISELEV 1  0");
    }
  } else if (match_cmd(command, "SETPOWER", &params)) {
    int power;
    sscanf(params, "%d", &power);
    power = mRadioInterface->setPowerAttenuation(power, chan);
    mStates[chan].mPower = power;
    sprintf(response, "RSP SETPOWER 0 %d", power);
  } else if (match_cmd(command, "ADJPOWER", &params)) {
    int power, step;
    sscanf(params, "%d", &step);
    power = mStates[chan].mPower + step;
    power = mRadioInterface->setPowerAttenuation(power, chan);
    mStates[chan].mPower = power;
    sprintf(response, "RSP ADJPOWER 0 %d", power);
  } else if (match_cmd(command, "RXTUNE", &params)) {
    // tune receiver
    int freqKhz;
    sscanf(params, "%d", &freqKhz);
    mRxFreq = freqKhz * 1e3;
    if (!mRadioInterface->tuneRx(mRxFreq, chan)) {
       LOGC(DTRXCTRL, ALERT) << "RX failed to tune";
       sprintf(response,"RSP RXTUNE 1 %d",freqKhz);
    }
    else
       sprintf(response,"RSP RXTUNE 0 %d",freqKhz);
  } else if (match_cmd(command, "TXTUNE", &params)) {
    // tune txmtr
    int freqKhz;
    sscanf(params, "%d", &freqKhz);
    mTxFreq = freqKhz * 1e3;
    if (!mRadioInterface->tuneTx(mTxFreq, chan)) {
       LOGC(DTRXCTRL, ALERT) << "TX failed to tune";
       sprintf(response,"RSP TXTUNE 1 %d",freqKhz);
    }
    else
       sprintf(response,"RSP TXTUNE 0 %d",freqKhz);
  } else if (match_cmd(command, "SETTSC", &params)) {
    // set TSC
    unsigned TSC;
    sscanf(params, "%u", &TSC);
    if (TSC > 7) {
      sprintf(response, "RSP SETTSC 1 %d", TSC);
    } else {
      LOGC(DTRXCTRL, NOTICE) << "Changing TSC from " << mTSC << " to " << TSC;
      mTSC = TSC;
      sprintf(response,"RSP SETTSC 0 %d", TSC);
    }
  } else if (match_cmd(command, "SETSLOT", &params)) {
    // set slot type
    int  corrCode;
    int  timeslot;
    sscanf(params, "%d %d", &timeslot, &corrCode);
    if ((timeslot < 0) || (timeslot > 7)) {
      LOGC(DTRXCTRL, WARNING) << "bogus message on control interface";
      sprintf(response,"RSP SETSLOT 1 %d %d",timeslot,corrCode);
      return;
    }
    mStates[chan].chanType[timeslot] = (ChannelCombination) corrCode;
    setModulus(timeslot, chan);
    sprintf(response,"RSP SETSLOT 0 %d %d",timeslot,corrCode);
  } else if (match_cmd(command, "SETFORMAT", &params)) {
    // set TRXD protocol version
    unsigned version_recv;
    sscanf(params, "%u", &version_recv);
    LOGCHAN(chan, DTRXCTRL, INFO) << "BTS requests TRXD version switch: " << version_recv;
    if (version_recv > TRX_DATA_FORMAT_VER) {
      LOGCHAN(chan, DTRXCTRL, INFO) << "rejecting TRXD version " << version_recv
                                    << "in favor of " <<  TRX_DATA_FORMAT_VER;
      sprintf(response, "RSP SETFORMAT %u %u", TRX_DATA_FORMAT_VER, version_recv);
    } else {
      LOGCHAN(chan, DTRXCTRL, NOTICE) << "switching to TRXD version " << version_recv;
      mVersionTRXD[chan] = version_recv;
      sprintf(response, "RSP SETFORMAT %u %u", version_recv, version_recv);
    }
  } else if (match_cmd(command, "_SETBURSTTODISKMASK", &params)) {
    // debug command! may change or disapear without notice
    // set a mask which bursts to dump to disk
    int mask;
    sscanf(params, "%d", &mask);
    mWriteBurstToDiskMask = mask;
    sprintf(response,"RSP _SETBURSTTODISKMASK 0 %d",mask);
  } else {
    LOGC(DTRXCTRL, WARNING) << "bogus command " << command << " on control interface.";
    sprintf(response,"RSP ERR 1");
  }

  LOGCHAN(chan, DTRXCTRL, INFO) << "response is '" << response << "'";
  msgLen = write(mCtrlSockets[chan], response, strlen(response) + 1);
  if (msgLen <= 0)
    LOGCHAN(chan, DTRXCTRL, WARNING) << "mCtrlSockets write(" << mCtrlSockets[chan] << ") failed: " << msgLen;
}

bool Transceiver::driveTxPriorityQueue(size_t chan)
{
  int msgLen;
  int burstLen;
  char buffer[EDGE_BURST_NBITS + 50];
  struct trxd_hdr_common *chdr;
  uint32_t fn;

  // check data socket
  msgLen = read(mDataSockets[chan], buffer, sizeof(buffer));
  if (msgLen <= 0) {
    LOGCHAN(chan, DTRXCTRL, WARNING) << "mDataSockets read(" << mCtrlSockets[chan] << ") failed: " << msgLen;
    return false;
  }

  if (msgLen == gSlotLen + 1 + 4 + 1) {
    burstLen = gSlotLen;
  } else if (msgLen == EDGE_BURST_NBITS + 1 + 4 + 1) {
    if (mSPSTx != 4)
      return false;

    burstLen = EDGE_BURST_NBITS;
  } else {
    LOG(ERR) << "badly formatted packet on GSM->TRX interface";
    return false;
  }

  /* Common header part: HDR version, TDMA TN & FN */
  chdr = (struct trxd_hdr_common *) buffer;

  /* Convert TDMA FN to the host endianness */
  fn = osmo_load32be(&chdr->fn);

  /* Make sure we support the received header format */
  switch (chdr->version) {
  case 0:
  /* Version 1 has the same format */
  case 1:
    break;

  default:
    LOG(ERR) << "Rx TRXD message with unknown header version " << chdr->version;
    return false;
  }

  LOG(DEBUG) << "Rx TRXD message (hdr_ver=" << chdr->version << "): "
             << "fn=" << fn << ", tn=" << chdr->tn << ", "
             << "burst_len=" << burstLen;

  int RSSI = (int) buffer[5];
  BitVector newBurst(burstLen);
  BitVector::iterator itr = newBurst.begin();
  char *bufferItr = buffer+6;
  while (itr < newBurst.end())
    *itr++ = *bufferItr++;

  GSM::Time currTime = GSM::Time(fn, chdr->tn);

  addRadioVector(chan, newBurst, RSSI, currTime);

  return true;


}

void Transceiver::driveReceiveRadio()
{
  int rc = mRadioInterface->driveReceiveRadio();
  if (rc == 0) {
    usleep(100000);
  } else if (rc < 0) {
    LOG(FATAL) << "radio Interface receive failed, requesting stop.";
    osmo_signal_dispatch(SS_MAIN, S_MAIN_STOP_REQUIRED, NULL);
  } else if (mForceClockInterface || mTransmitDeadlineClock > mLastClockUpdateTime + GSM::Time(216,0)) {
    mForceClockInterface = false;
    writeClockInterface();
  }
}

void Transceiver::logRxBurst(size_t chan, const struct trx_ul_burst_ind *bi)
{
  std::ostringstream os;
  for (size_t i=0; i < bi->nbits; i++) {
    if (bi->rx_burst[i] > 0.5) os << "1";
    else if (bi->rx_burst[i] > 0.25) os << "|";
    else if (bi->rx_burst[i] > 0.0) os << "'";
    else os << "-";
  }

  LOG(DEBUG) << std::fixed << std::right
    << " chan: "   << chan
    << " time: "   << bi->tn << ":" << bi->fn
    << " RSSI: "   << std::setw(5) << std::setprecision(1) << (bi->rssi - rssiOffset)
                   << "dBFS/" << std::setw(6) << -bi->rssi << "dBm"
    << " noise: "  << std::setw(5) << std::setprecision(1) << (bi->noise - rssiOffset)
                   << "dBFS/" << std::setw(6) << -bi->noise << "dBm"
    << " TOA: "    << std::setw(5) << std::setprecision(2) << bi->toa
    << " C/I: "    << std::setw(5) << std::setprecision(2) << bi->ci << "dB"
    << " bits: "   << os;
}

void Transceiver::driveReceiveFIFO(size_t chan)
{
  struct trx_ul_burst_ind bi;

  if (!pullRadioVector(chan, &bi))
        return;
  if (!bi.idle)
       logRxBurst(chan, &bi);

  switch (mVersionTRXD[chan]) {
    case 0:
      trxd_send_burst_ind_v0(chan, mDataSockets[chan], &bi);
      break;
    case 1:
      trxd_send_burst_ind_v1(chan, mDataSockets[chan], &bi);
      break;
    default:
      OSMO_ASSERT(false);
  }
}

void Transceiver::driveTxFIFO()
{

  /**
      Features a carefully controlled latency mechanism, to
      assure that transmit packets arrive at the radio/USRP
      before they need to be transmitted.

      Deadline clock indicates the burst that needs to be
      pushed into the FIFO right NOW.  If transmit queue does
      not have a burst, stick in filler data.
  */


  RadioClock *radioClock = (mRadioInterface->getClock());

  if (mOn) {
    //radioClock->wait(); // wait until clock updates
    LOG(DEBUG) << "radio clock " << radioClock->get();
    while (radioClock->get() + mTransmitLatency > mTransmitDeadlineClock) {
      // if underrun, then we're not providing bursts to radio/USRP fast
      //   enough.  Need to increase latency by one GSM frame.
      if (mRadioInterface->getWindowType() == RadioDevice::TX_WINDOW_USRP1) {
        if (mRadioInterface->isUnderrun()) {
          // only update latency at the defined frame interval
          if (radioClock->get() > mLatencyUpdateTime + GSM::Time(USB_LATENCY_INTRVL)) {
            mTransmitLatency = mTransmitLatency + GSM::Time(1,0);
            LOG(INFO) << "new latency: " << mTransmitLatency << " (underrun "
                      << radioClock->get() << " vs " << mLatencyUpdateTime + GSM::Time(USB_LATENCY_INTRVL) << ")";
            mLatencyUpdateTime = radioClock->get();
          }
        }
        else {
          // if underrun hasn't occurred in the last sec (216 frames) drop
          //    transmit latency by a timeslot
          if (mTransmitLatency > mRadioInterface->minLatency()) {
              if (radioClock->get() > mLatencyUpdateTime + GSM::Time(216,0)) {
              mTransmitLatency.decTN();
              LOG(INFO) << "reduced latency: " << mTransmitLatency;
              mLatencyUpdateTime = radioClock->get();
            }
          }
        }
      }
      // time to push burst to transmit FIFO
      pushRadioVector(mTransmitDeadlineClock);
      mTransmitDeadlineClock.incTN();
    }
  }

  radioClock->wait();
}



void Transceiver::writeClockInterface()
{
  int msgLen;
  char command[50];
  // FIXME -- This should be adaptive.
  sprintf(command,"IND CLOCK %llu",(unsigned long long) (mTransmitDeadlineClock.FN()+2));

  LOG(INFO) << "ClockInterface: sending " << command;

  msgLen = write(mClockSocket, command, strlen(command) + 1);
  if (msgLen <= 0)
    LOG(WARNING) << "mClockSocket write(" << mClockSocket << ") failed: " << msgLen;

  mLastClockUpdateTime = mTransmitDeadlineClock;

}

void *RxUpperLoopAdapter(TransceiverChannel *chan)
{
  char thread_name[16];
  Transceiver *trx = chan->trx;
  size_t num = chan->num;

  delete chan;

  snprintf(thread_name, 16, "RxUpper%zu", num);
  set_selfthread_name(thread_name);

  trx->setPriority(0.42);

  while (1) {
    trx->driveReceiveFIFO(num);
    pthread_testcancel();
  }
  return NULL;
}

void *RxLowerLoopAdapter(Transceiver *transceiver)
{
  set_selfthread_name("RxLower");

  transceiver->setPriority(0.45);

  while (1) {
    transceiver->driveReceiveRadio();
    pthread_testcancel();
  }
  return NULL;
}

void *TxLowerLoopAdapter(Transceiver *transceiver)
{
  set_selfthread_name("TxLower");

  transceiver->setPriority(0.44);

  while (1) {
    transceiver->driveTxFIFO();
    pthread_testcancel();
  }
  return NULL;
}

void *ControlServiceLoopAdapter(TransceiverChannel *chan)
{
  char thread_name[16];
  Transceiver *trx = chan->trx;
  size_t num = chan->num;

  delete chan;

  snprintf(thread_name, 16, "CtrlService%zu", num);
  set_selfthread_name(thread_name);

  while (1) {
    trx->driveControl(num);
    pthread_testcancel();
  }
  return NULL;
}

void *TxUpperLoopAdapter(TransceiverChannel *chan)
{
  char thread_name[16];
  Transceiver *trx = chan->trx;
  size_t num = chan->num;

  delete chan;

  snprintf(thread_name, 16, "TxUpper%zu", num);
  set_selfthread_name(thread_name);

  trx->setPriority(0.40);

  while (1) {
    trx->driveTxPriorityQueue(num);
    pthread_testcancel();
  }
  return NULL;
}
