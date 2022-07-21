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
#include <netinet/in.h>
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
#include <osmocom/vty/cpu_sched_vty.h>
}

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

using namespace GSM;

Transceiver *transceiver;

#define USB_LATENCY_INTRVL		10,0

/* Number of running values use in noise average */
#define NOISE_CNT			20


static void dispatch_trx_rate_ctr_change(TransceiverState *state, unsigned int chan) {
        thread_enable_cancel(false);
        state->ctrs.chan = chan;
        osmo_signal_dispatch(SS_DEVICE, S_TRX_COUNTER_CHANGE, &state->ctrs);
        thread_enable_cancel(true);
}

TransceiverState::TransceiverState()
  : mFiller(FILLER_ZERO), mRetrans(false), mNoiseLev(0.0), mNoises(NOISE_CNT),
    mPower(0.0), mMuted(false), first_dl_fn_rcv()
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
  memset(&ctrs, 0, sizeof(struct trx_counters));
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

  mFiller = filler;

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

Transceiver::Transceiver(const struct trx_cfg *cfg,
                         GSM::Time wTransmitLatency,
                         RadioInterface *wRadioInterface)
  : mChans(cfg->num_chans), cfg(cfg),
    mCtrlSockets(mChans), mClockSocket(-1),
    mTxPriorityQueues(mChans), mReceiveFIFO(mChans),
    mRxServiceLoopThreads(mChans), mRxLowerLoopThread(nullptr), mTxLowerLoopThread(nullptr),
    mTxPriorityQueueServiceLoopThreads(mChans), mTransmitLatency(wTransmitLatency), mRadioInterface(wRadioInterface),
    mOn(false),mForceClockInterface(false), mTxFreq(0.0), mRxFreq(0.0), mTSC(0), mMaxExpectedDelayAB(0),
    mMaxExpectedDelayNB(0), mWriteBurstToDiskMask(0), mVersionTRXD(mChans), mStates(mChans)
{
  txFullScale = mRadioInterface->fullScaleInputValue();
  rxFullScale = mRadioInterface->fullScaleOutputValue();

  for (size_t i = 0; i < ARRAY_SIZE(mHandover); i++) {
    for (size_t j = 0; j < ARRAY_SIZE(mHandover[i]); j++)
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
    mTxPriorityQueues[i].clear();
    if (mDataSockets[i] >= 0)
      close(mDataSockets[i]);
  }
}

int Transceiver::ctrl_sock_cb(struct osmo_fd *bfd, unsigned int flags)
{
  int rc = 0;
  int chan = static_cast<int>(reinterpret_cast<uintptr_t>(bfd->data));

  if (flags & OSMO_FD_READ)
    rc = transceiver->ctrl_sock_handle_rx(chan);
  if (rc < 0)
    osmo_signal_dispatch(SS_MAIN, S_MAIN_STOP_REQUIRED, NULL);

  if (flags & OSMO_FD_WRITE)
    rc = transceiver->ctrl_sock_write(chan);
  if (rc < 0)
    osmo_signal_dispatch(SS_MAIN, S_MAIN_STOP_REQUIRED, NULL);

  return rc;
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
bool Transceiver::init()
{
  int d_srcport, d_dstport, c_srcport, c_dstport;
  if (!mChans) {
    LOG(FATAL) << "No channels assigned";
    return false;
  }

  if (!sigProcLibSetup()) {
    LOG(FATAL) << "Failed to initialize signal processing library";
    return false;
  }

  mDataSockets.resize(mChans, -1);
  

  /* Filler table retransmissions - support only on channel 0 */
  if (cfg->filler == FILLER_DUMMY)
    mStates[0].mRetrans = true;

  /* Setup sockets */
  mClockSocket = osmo_sock_init2(AF_UNSPEC, SOCK_DGRAM, IPPROTO_UDP,
				    cfg->bind_addr, cfg->base_port,
				    cfg->remote_addr, cfg->base_port + 100,
				    OSMO_SOCK_F_BIND | OSMO_SOCK_F_CONNECT);
  if (mClockSocket < 0)
    return false;

  for (size_t i = 0; i < mChans; i++) {
    int rv;
    FillerType filler = cfg->filler;
    c_srcport = cfg->base_port + 2 * i + 1;
    c_dstport = cfg->base_port + 2 * i + 101;
    d_srcport = cfg->base_port + 2 * i + 2;
    d_dstport = cfg->base_port + 2 * i + 102;

    rv = osmo_sock_init2_ofd(&mCtrlSockets[i].conn_bfd, AF_UNSPEC, SOCK_DGRAM, IPPROTO_UDP,
                                      cfg->bind_addr, c_srcport,
                                      cfg->remote_addr, c_dstport,
				      OSMO_SOCK_F_BIND | OSMO_SOCK_F_CONNECT);
    if (rv < 0)
      return false;

    mCtrlSockets[i].conn_bfd.cb = ctrl_sock_cb;
    mCtrlSockets[i].conn_bfd.data = reinterpret_cast<void*>(i);


    mDataSockets[i] = osmo_sock_init2(AF_UNSPEC, SOCK_DGRAM, IPPROTO_UDP,
                                      cfg->bind_addr, d_srcport,
                                      cfg->remote_addr, d_dstport,
				      OSMO_SOCK_F_BIND | OSMO_SOCK_F_CONNECT);
    if (mDataSockets[i] < 0)
      return false;

    if (i && filler == FILLER_DUMMY)
      filler = FILLER_ZERO;

    mStates[i].init(filler, cfg->tx_sps, txFullScale, cfg->rtsc, cfg->rach_delay);
  }

  /* Randomize the central clock */
  GSM::Time startTime(random() % gHyperframe, 0);
  mRadioInterface->getClock()->set(startTime);
  mTransmitDeadlineClock = startTime;
  mLastClockUpdateTime = startTime;
  mLatencyUpdateTime = startTime;

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
    LOG(FATAL) << "Device failed to start";
    return false;
  }

  /* Device is running - launch I/O threads */
  mRxLowerLoopThread = new Thread(cfg->stack_size);
  mTxLowerLoopThread = new Thread(cfg->stack_size);
  mTxLowerLoopThread->start((void * (*)(void*))
                            TxLowerLoopAdapter,(void*) this);
  mRxLowerLoopThread->start((void * (*)(void*))
                            RxLowerLoopAdapter,(void*) this);

  /* Launch uplink and downlink burst processing threads */
  for (size_t i = 0; i < mChans; i++) {
    TrxChanThParams *params = (TrxChanThParams *)malloc(sizeof(struct TrxChanThParams));
    params->trx = this;
    params->num = i;
    mRxServiceLoopThreads[i] = new Thread(cfg->stack_size);
    mRxServiceLoopThreads[i]->start((void * (*)(void*))
                            RxUpperLoopAdapter, (void*) params);

    params = (TrxChanThParams *)malloc(sizeof(struct TrxChanThParams));
    params->trx = this;
    params->num = i;
    mTxPriorityQueueServiceLoopThreads[i] = new Thread(cfg->stack_size);
    mTxPriorityQueueServiceLoopThreads[i]->start((void * (*)(void*))
                            TxUpperLoopAdapter, (void*) params);
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
    LOGCHAN(chan, DTRXDDL, FATAL) << "Invalid channel";
    return;
  }

  if (wTime.TN() > 7) {
    LOGCHAN(chan, DTRXDDL, FATAL) << "Received burst with invalid slot " << wTime.TN();
    return;
  }

  /* Use the number of bits as the EDGE burst indicator */
  if (bits.size() == EDGE_BURST_NBITS)
    burst = modulateEdgeBurst(bits, cfg->tx_sps);
  else
    burst = modulateBurst(bits, 8 + (wTime.TN() % 4 == 0), cfg->tx_sps);

  scaleVector(*burst, txFullScale * pow(10, (double) -RSSI / 20));

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
  bool ratectr_changed;

  TN = nowTime.TN();

  for (size_t i = 0; i < mChans; i ++) {
    state = &mStates[i];
    ratectr_changed = false;

    zeros[i] = state->chanType[TN] == NONE || state->mMuted;

    Mutex *mtx = mTxPriorityQueues[i].getMutex();
    mtx->lock();

    while ((burst = mTxPriorityQueues[i].getStaleBurst(nowTime))) {
      LOGCHAN(i, DTRXDDL, INFO) << "dumping STALE burst in TRX->SDR interface ("
                  << burst->getTime() <<" vs " << nowTime << "), retrans=" << state->mRetrans;
      state->ctrs.tx_stale_bursts++;
      ratectr_changed = true;
      if (state->mRetrans)
        updateFillerTable(i, burst);
      delete burst;
    }

    if ((burst = mTxPriorityQueues[i].getCurrentBurst(nowTime))) {
      bursts[i] = burst->getVector();

      if (state->mRetrans) {
        updateFillerTable(i, burst);
      } else {
        burst->setVector(NULL);
        filler[i] = false;
      }

      delete burst;
    } else {
      modFN = nowTime.FN() % state->fillerModulus[TN];
      bursts[i] = state->fillerTable[modFN][TN];
      if (i == 0 && state->mFiller == FILLER_ZERO) {
        LOGCHAN(i, DTRXDDL, INFO) << "No Tx burst available for " << nowTime
                                    << ", retrans=" << state->mRetrans;
        state->ctrs.tx_unavailable_bursts++;
        ratectr_changed = true;
      }
    }

    mtx->unlock();

    if (ratectr_changed)
      dispatch_trx_rate_ctr_change(state, i);
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
    return cfg->ext_rach ? EXT_RACH : RACH;
    break;
  case V: {
    int mod51 = burstFN % 51;
    if ((mod51 <= 36) && (mod51 >= 14))
      return cfg->ext_rach ? EXT_RACH : RACH;
    else if ((mod51 == 4) || (mod51 == 5))
      return cfg->ext_rach ? EXT_RACH : RACH;
    else if ((mod51 == 45) || (mod51 == 46))
      return cfg->ext_rach ? EXT_RACH : RACH;
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
      return cfg->ext_rach ? EXT_RACH : RACH;
    else if ((mod52 == 25) || (mod52 == 51))
      return IDLE;
    else /* Enable 8-PSK burst detection if EDGE is enabled */
      return cfg->egprs ? EDGE : TSC;
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

double Transceiver::rssiOffset(size_t chan)
{
  if (cfg->force_rssi_offset)
        return cfg->rssi_offset;
  return mRadioInterface->rssiOffset(chan) + cfg->rssi_offset;
}

/*
 * Pull bursts from the FIFO and handle according to the slot
 * and burst correlation type. Equalzation is currently disabled.
 * returns 0 on success (bi filled), negative on error (bi content undefined):
 *        -ENOENT: timeslot is off (fn and tn in bi are filled),
 *        -EIO: read error
 */
int Transceiver::pullRadioVector(size_t chan, struct trx_ul_burst_ind *bi)
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
  bool ctr_changed = false;
  double rssi_offset;

  /* Blocking FIFO read */
  radioVector *radio_burst = mReceiveFIFO[chan]->read();
  if (!radio_burst) {
    LOGCHAN(chan, DTRXDUL, ERROR) << "ReceiveFIFO->read() returned no burst";
    return -EIO;
  }

  /* Set time and determine correlation type */
  burstTime = radio_burst->getTime() + cfg->ul_fn_offset;
  CorrType type = expectedCorrType(burstTime, chan);

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

  /* Debug: dump bursts to disk */
  /* bits 0-7  - chan 0 timeslots
   * bits 8-15 - chan 1 timeslots */
  if (mWriteBurstToDiskMask & ((1<<bi->tn) << (8*chan)))
    writeToFile(radio_burst, chan);

  /* No processing if the timeslot is off.
   * Not even power level or noise calculation. */
  if (type == OFF) {
    delete radio_burst;
    return -ENOENT;
  }

  /* If TRX RF is locked/muted by BTS, send idle burst indications */
  if (state->mMuted)
    goto ret_idle;

  /* Select the diversity channel with highest energy */
  for (size_t i = 0; i < radio_burst->chans(); i++) {
    float pow = energyDetect(*radio_burst->getVector(i), 20 * cfg->rx_sps);
    if (pow > max) {
      max = pow;
      max_i = i;
    }
    avg += pow;
  }

  if (max_i < 0) {
    LOGCHAN(chan, DTRXDUL, INFO) << "Received empty burst";
    state->ctrs.rx_empty_burst++;
    ctr_changed = true;
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

  rssi_offset = rssiOffset(chan);
  bi->rssi = 20.0 * log10(rxFullScale / avg) + rssi_offset;
  bi->noise = 20.0 * log10(rxFullScale / state->mNoiseLev) + rssi_offset;

  if (type == IDLE)
    goto ret_idle;

  max_toa = (type == RACH || type == EXT_RACH) ?
            mMaxExpectedDelayAB : mMaxExpectedDelayNB;

  /* Detect normal or RACH bursts */
  rc = detectAnyBurst(*burst, mTSC, BURST_THRESH, cfg->rx_sps, type, max_toa, &ebp);
  if (rc <= 0) {
    if (rc == -SIGERR_CLIP) {
      LOGCHAN(chan, DTRXDUL, INFO) << "Clipping detected on received RACH or Normal Burst";
      state->ctrs.rx_clipping++;
      ctr_changed = true;
    } else if (rc != SIGERR_NONE) {
      LOGCHAN(chan, DTRXDUL, INFO) << "Unhandled RACH or Normal Burst detection error";
      state->ctrs.rx_no_burst_detected++;
      ctr_changed = true;
    }
    goto ret_idle;
  }

  rxBurst = demodAnyBurst(*burst, (CorrType) rc, cfg->rx_sps, &ebp);
  bi->toa = ebp.toa;
  bi->tsc = ebp.tsc;
  bi->ci = ebp.ci;

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
  return 0;

ret_idle:
  if (ctr_changed)
    dispatch_trx_rate_ctr_change(state, chan);
  bi->idle = true;
  delete radio_burst;
  return 0;
}

void Transceiver::reset()
{
  for (size_t i = 0; i < mTxPriorityQueues.size(); i++)
    mTxPriorityQueues[i].clear();
}


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

void Transceiver::ctrl_sock_send(ctrl_msg& m, int chan)
{
  ctrl_sock_state& s = mCtrlSockets[chan];
  struct osmo_fd *conn_bfd = &s.conn_bfd;

  s.txmsgqueue.push_back(m);
  osmo_fd_write_enable(conn_bfd);
}

int Transceiver::ctrl_sock_write(int chan)
{
  int rc;
  ctrl_sock_state& s = mCtrlSockets[chan];

  if (s.conn_bfd.fd < 0) {
      return -EIO;
  }

  while (s.txmsgqueue.size()) {
    const ctrl_msg m = s.txmsgqueue.front();

    osmo_fd_write_disable(&s.conn_bfd);

    /* try to send it over the socket */
    rc = write(s.conn_bfd.fd, m.data, strlen(m.data) + 1);
    if (rc == 0)
      goto close;
    if (rc < 0) {
      if (errno == EAGAIN) {
        osmo_fd_write_enable(&s.conn_bfd);
        break;
      }
      goto close;
    }

      s.txmsgqueue.pop_front();
  }
  return 0;

close:
  LOGCHAN(chan, DTRXCTRL, NOTICE) << "mCtrlSockets write(" << s.conn_bfd.fd << ") failed: " << rc;
  return -1;
}

int Transceiver::ctrl_sock_handle_rx(int chan)
{
  ctrl_msg cmd_received;
  ctrl_msg cmd_to_send;
  char *buffer = cmd_received.data;
  char *response = cmd_to_send.data;
  char *command, *params;
  int msgLen;
  ctrl_sock_state& s = mCtrlSockets[chan];

  /* Attempt to read from control socket */
  msgLen = read(s.conn_bfd.fd, buffer, sizeof(cmd_received.data)-1);
  if (msgLen < 0 && errno == EAGAIN)
      return 0; /* Try again later */
  if (msgLen <= 0) {
    LOGCHAN(chan, DTRXCTRL, NOTICE) << "mCtrlSockets read(" << s.conn_bfd.fd << ") failed: " << msgLen;
    return -EIO;
  }


  /* Zero-terminate received string */
  buffer[msgLen] = '\0';

  /* Verify a command signature */
  if (strncmp(buffer, "CMD ", 4)) {
    LOGCHAN(chan, DTRXCTRL, NOTICE) << "bogus message on control interface";
    return -EIO;
  }

  /* Set command pointer */
  command = buffer + 4;
  LOGCHAN(chan, DTRXCTRL, INFO) << "command is '" << command << "'";

  if (match_cmd(command, "POWEROFF", NULL)) {
    // stop();
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
      sprintf(response, "RSP HANDOVER 1 %u %u", ts, ss);
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
      sprintf(response,"RSP NOISELEV 1 0");
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
} else if (match_cmd(command, "NOMTXPOWER", NULL)) {
    int power = mRadioInterface->getNominalTxPower(chan);
    sprintf(response, "RSP NOMTXPOWER 0 %d", power);
  } else if (match_cmd(command, "RXTUNE", &params)) {
    // tune receiver
    int freqKhz;
    sscanf(params, "%d", &freqKhz);
    mRxFreq = (freqKhz + cfg->freq_offset_khz) * 1e3;
    if (!mRadioInterface->tuneRx(mRxFreq, chan)) {
       LOGCHAN(chan, DTRXCTRL, FATAL) << "RX failed to tune";
       sprintf(response,"RSP RXTUNE 1 %d",freqKhz);
    }
    else
       sprintf(response,"RSP RXTUNE 0 %d",freqKhz);
  } else if (match_cmd(command, "TXTUNE", &params)) {
    // tune txmtr
    int freqKhz;
    sscanf(params, "%d", &freqKhz);
    mTxFreq = (freqKhz + cfg->freq_offset_khz) * 1e3;
    if (!mRadioInterface->tuneTx(mTxFreq, chan)) {
       LOGCHAN(chan, DTRXCTRL, FATAL) << "TX failed to tune";
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
      LOGCHAN(chan, DTRXCTRL, NOTICE) << "bogus message on control interface";
      sprintf(response,"RSP SETSLOT 1 %d %d",timeslot,corrCode);
      return 0;
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
                                    << " in favor of " <<  TRX_DATA_FORMAT_VER;
      sprintf(response, "RSP SETFORMAT %u %u", TRX_DATA_FORMAT_VER, version_recv);
    } else {
      LOGCHAN(chan, DTRXCTRL, NOTICE) << "switching to TRXD version " << version_recv;
      mVersionTRXD[chan] = version_recv;
      sprintf(response, "RSP SETFORMAT %u %u", version_recv, version_recv);
    }
  } else if (match_cmd(command, "RFMUTE", &params)) {
    // (Un)mute RF TX and RX
    unsigned mute;
    sscanf(params, "%u", &mute);
    mStates[chan].mMuted = mute ? true : false;
    sprintf(response, "RSP RFMUTE 0 %u", mute);
  } else if (match_cmd(command, "_SETBURSTTODISKMASK", &params)) {
    // debug command! may change or disappear without notice
    // set a mask which bursts to dump to disk
    int mask;
    sscanf(params, "%d", &mask);
    mWriteBurstToDiskMask = mask;
    sprintf(response,"RSP _SETBURSTTODISKMASK 0 %d",mask);
  } else {
    LOGCHAN(chan, DTRXCTRL, NOTICE) << "bogus command " << command << " on control interface.";
    sprintf(response,"RSP ERR 1");
  }

  LOGCHAN(chan, DTRXCTRL, INFO) << "response is '" << response << "'";
  transceiver->ctrl_sock_send(cmd_to_send, chan);
  return 0;
}

bool Transceiver::driveTxPriorityQueue(size_t chan)
{
  int msgLen;
  int burstLen;
  struct trxd_hdr_v01_dl *dl;
  char buffer[sizeof(*dl) + EDGE_BURST_NBITS];
  uint32_t fn;
  uint8_t tn;

  // check data socket
  msgLen = read(mDataSockets[chan], buffer, sizeof(buffer));
  if (msgLen <= 0) {
    LOGCHAN(chan, DTRXDDL, NOTICE) << "mDataSockets read(" << mDataSockets[chan] << ") failed: " << msgLen;
    return false;
  }

  switch (msgLen) {
    case sizeof(*dl) + gSlotLen: /* GSM burst */
      burstLen = gSlotLen;
      break;
    case sizeof(*dl) + EDGE_BURST_NBITS: /* EDGE burst */
      if (cfg->tx_sps != 4) {
        LOGCHAN(chan, DTRXDDL, ERROR) << "EDGE burst received but SPS is set to " << cfg->tx_sps;
        return false;
      }
      burstLen = EDGE_BURST_NBITS;
      break;
    default:
      LOGCHAN(chan, DTRXDDL, ERROR) << "badly formatted packet on GSM->TRX interface (len="<< msgLen << ")";
      return false;
  }

  dl = (struct trxd_hdr_v01_dl *) buffer;

  /* Convert TDMA FN to the host endianness */
  fn = osmo_load32be(&dl->common.fn);
  tn = dl->common.tn;

  /* Make sure we support the received header format */
  switch (dl->common.version) {
  case 0:
  /* Version 1 has the same format */
  case 1:
    break;
  default:
    LOGCHAN(chan, DTRXDDL, ERROR) << "Rx TRXD message with unknown header version " << unsigned(dl->common.version);
    return false;
  }

  LOGCHAN(chan, DTRXDDL, DEBUG) << "Rx TRXD message (hdr_ver=" << unsigned(dl->common.version)
    << "): fn=" << fn << ", tn=" << unsigned(tn) << ", burst_len=" << burstLen;

  TransceiverState *state = &mStates[chan];
  GSM::Time currTime = GSM::Time(fn, tn);

  /* Verify proper FN order in DL stream */
  if (state->first_dl_fn_rcv[tn]) {
    int32_t delta = GSM::FNDelta(currTime.FN(), state->last_dl_time_rcv[tn].FN());
    if (delta == 1) {
        /* usual expected scenario, continue code flow */
    } else if (delta == 0) {
      LOGCHAN(chan, DTRXDDL, INFO) << "Rx TRXD msg with repeated FN " << currTime;
      state->ctrs.tx_trxd_fn_repeated++;
      dispatch_trx_rate_ctr_change(state, chan);
      return true;
    } else if (delta < 0) {
      LOGCHAN(chan, DTRXDDL, INFO) << "Rx TRXD msg with previous FN " << currTime
                                     << " vs last " << state->last_dl_time_rcv[tn];
       state->ctrs.tx_trxd_fn_outoforder++;
       dispatch_trx_rate_ctr_change(state, chan);
       /* Allow adding radio vector below, since it gets sorted in the queue */
    } else if (chan == 0 && state->mFiller == FILLER_ZERO) {
        /* delta > 1. Some FN was lost in the middle. We can only easily rely
         * on consecutive FNs in TRX0 since it must transmit continuously in all
         * setups. Also, osmo-trx supports optionally filling empty bursts on
         * its own. In that case bts-trx is not obliged to submit all bursts. */
      LOGCHAN(chan, DTRXDDL, INFO) << "Rx TRXD msg with future FN " << currTime
                                     << " vs last " << state->last_dl_time_rcv[tn]
                                     << ", " << delta - 1 << " FN lost";
      state->ctrs.tx_trxd_fn_skipped += delta - 1;
      dispatch_trx_rate_ctr_change(state, chan);
    }
    if (delta > 0)
      state->last_dl_time_rcv[tn] = currTime;
  } else { /* Initial check, simply store state */
    state->first_dl_fn_rcv[tn] = true;
    state->last_dl_time_rcv[tn] = currTime;
  }

  BitVector newBurst(burstLen);
  BitVector::iterator itr = newBurst.begin();
  uint8_t *bufferItr = dl->soft_bits;
  while (itr < newBurst.end())
    *itr++ = *bufferItr++;

  addRadioVector(chan, newBurst, dl->tx_att, currTime);

  return true;
}

bool Transceiver::driveReceiveRadio()
{
  int rc = mRadioInterface->driveReceiveRadio();
  if (rc == 0) {
    usleep(100000);
    return true;
  }
  if (rc < 0)
    return false;

  if (mForceClockInterface || mTransmitDeadlineClock > mLastClockUpdateTime + GSM::Time(216,0)) {
    if (mForceClockInterface)
      LOGC(DTRXCLK, NOTICE) << "Sending CLOCK indications";
    mForceClockInterface = false;
    return writeClockInterface();
  }
  return true;
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

  double rssi_offset = rssiOffset(chan);

  LOGCHAN(chan, DTRXDUL, DEBUG) << std::fixed << std::right
    << " time: "   << unsigned(bi->tn) << ":" << bi->fn
    << " RSSI: "   << std::setw(5) << std::setprecision(1) << (bi->rssi - rssi_offset)
                   << "dBFS/" << std::setw(6) << -bi->rssi << "dBm"
    << " noise: "  << std::setw(5) << std::setprecision(1) << (bi->noise - rssi_offset)
                   << "dBFS/" << std::setw(6) << -bi->noise << "dBm"
    << " TOA: "    << std::setw(5) << std::setprecision(2) << bi->toa
    << " C/I: "    << std::setw(5) << std::setprecision(2) << bi->ci << "dB"
    << " bits: "   << os;
}

bool Transceiver::driveReceiveFIFO(size_t chan)
{
  struct trx_ul_burst_ind bi;
  int rc;

  if ((rc = pullRadioVector(chan, &bi)) < 0) {
    if (rc == -ENOENT) { /* timeslot off, continue processing */
      LOGCHAN(chan, DTRXDUL, DEBUG) << unsigned(bi.tn) << ":" << bi.fn << " timeslot is off";
      return true;
    }
    return false; /* other errors: we want to stop the process */
  }

  if (!bi.idle && log_check_level(DTRXDUL, LOGL_DEBUG))
    logRxBurst(chan, &bi);

  switch (mVersionTRXD[chan]) {
    case 0:
      return trxd_send_burst_ind_v0(chan, mDataSockets[chan], &bi);
    case 1:
      return trxd_send_burst_ind_v1(chan, mDataSockets[chan], &bi);
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
    LOGC(DTRXCLK, DEBUG) << "radio clock " << radioClock->get();
    while (radioClock->get() + mTransmitLatency > mTransmitDeadlineClock) {
      // if underrun, then we're not providing bursts to radio/USRP fast
      //   enough.  Need to increase latency by one GSM frame.
      if (mRadioInterface->getWindowType() == RadioDevice::TX_WINDOW_USRP1) {
        if (mRadioInterface->isUnderrun()) {
          // only update latency at the defined frame interval
          if (radioClock->get() > mLatencyUpdateTime + GSM::Time(USB_LATENCY_INTRVL)) {
            mTransmitLatency = mTransmitLatency + GSM::Time(1,0);
            LOGC(DTRXCLK, INFO) << "new latency: " << mTransmitLatency << " (underrun "
                                << radioClock->get() << " vs "
                                << mLatencyUpdateTime + GSM::Time(USB_LATENCY_INTRVL) << ")";
            mLatencyUpdateTime = radioClock->get();
          }
        }
        else {
          // if underrun hasn't occurred in the last sec (216 frames) drop
          //    transmit latency by a timeslot
          if (mTransmitLatency > mRadioInterface->minLatency()) {
              if (radioClock->get() > mLatencyUpdateTime + GSM::Time(216,0)) {
              mTransmitLatency.decTN();
              LOGC(DTRXCLK, INFO) << "reduced latency: " << mTransmitLatency;
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



bool Transceiver::writeClockInterface()
{
  int msgLen;
  char command[50];
  // FIXME -- This should be adaptive.
  sprintf(command,"IND CLOCK %llu",(unsigned long long) (mTransmitDeadlineClock.FN()+2));

  LOGC(DTRXCLK, INFO) << "sending " << command;

  msgLen = write(mClockSocket, command, strlen(command) + 1);
  if (msgLen <= 0) {
    LOGC(DTRXCLK, ERROR) << "mClockSocket write(" << mClockSocket << ") failed: " << msgLen;
    return false;
  }

  mLastClockUpdateTime = mTransmitDeadlineClock;
  return true;
}

void *RxUpperLoopAdapter(TrxChanThParams *params)
{
  char thread_name[16];
  Transceiver *trx = params->trx;
  size_t num = params->num;

  free(params);

  snprintf(thread_name, 16, "RxUpper%zu", num);
  set_selfthread_name(thread_name);
  OSMO_ASSERT(osmo_cpu_sched_vty_apply_localthread() == 0);

  while (1) {
    if (!trx->driveReceiveFIFO(num)) {
      LOGCHAN(num, DTRXDUL, FATAL) << "Something went wrong in thread " << thread_name << ", requesting stop";
      osmo_signal_dispatch(SS_MAIN, S_MAIN_STOP_REQUIRED, NULL);
      break;
    }
    pthread_testcancel();
  }
  return NULL;
}

void *RxLowerLoopAdapter(Transceiver *transceiver)
{
  set_selfthread_name("RxLower");
  OSMO_ASSERT(osmo_cpu_sched_vty_apply_localthread() == 0);

  while (1) {
    if (!transceiver->driveReceiveRadio()) {
      LOGC(DTRXDUL, FATAL) << "Something went wrong in thread RxLower, requesting stop";
      osmo_signal_dispatch(SS_MAIN, S_MAIN_STOP_REQUIRED, NULL);
      break;
    }
    pthread_testcancel();
  }
  return NULL;
}

void *TxLowerLoopAdapter(Transceiver *transceiver)
{
  set_selfthread_name("TxLower");
  OSMO_ASSERT(osmo_cpu_sched_vty_apply_localthread() == 0);

  while (1) {
    transceiver->driveTxFIFO();
    pthread_testcancel();
  }
  return NULL;
}

void *TxUpperLoopAdapter(TrxChanThParams *params)
{
  char thread_name[16];
  Transceiver *trx = params->trx;
  size_t num = params->num;

  free(params);

  snprintf(thread_name, 16, "TxUpper%zu", num);
  set_selfthread_name(thread_name);
  OSMO_ASSERT(osmo_cpu_sched_vty_apply_localthread() == 0);

  while (1) {
    if (!trx->driveTxPriorityQueue(num)) {
      LOGCHAN(num, DTRXDDL, FATAL) << "Something went wrong in thread " << thread_name << ", requesting stop";
      osmo_signal_dispatch(SS_MAIN, S_MAIN_STOP_REQUIRED, NULL);
      break;
    }
    pthread_testcancel();
  }
  return NULL;
}
