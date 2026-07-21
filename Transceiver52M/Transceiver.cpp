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
#include <grgsm_vitac/grgsm_vitac.h>

extern "C" {
#include "osmo_signal.h"

#include <osmocom/core/utils.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/msgb.h>
#include <osmocom/vty/cpu_sched_vty.h>
#include <osmocom/trx/trxc.h>
#include <osmocom/trx/trxd.h>
}

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

using namespace GSM;

Transceiver *transceiver;

#define USB_LATENCY_INTRVL		10,0

/* Number of running values use in noise average */
#define NOISE_CNT			20

/* Worst-case size of a batched TRXDv2 datagram (either direction): up to 8
 * timeslots per frame, each sized as generously as a lone PDU
 * (OSMO_TRXD_MSG_BUF_SIZE already covers header + max burst payload for one
 * PDU; PDUs after the first omit the 4-byte FN, so this is a safe
 * over-estimate, not a tight bound). */
#define TRX_TRXD_BATCH_BUF_SIZE		(8 * OSMO_TRXD_MSG_BUF_SIZE)


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
    mTxPriorityQueueServiceLoopThreads(mChans),
    mBurstIndBatch(mChans), mBurstIndBatchFn(mChans),
    mTransmitLatency(wTransmitLatency), mRadioInterface(wRadioInterface),
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
    msgb_free(mBurstIndBatch[i]);
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

  initvita();

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

    /* Rx thread is joined, so no concurrent access to the batch anymore */
    flushBurstIndBatch(i);

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
      return RACH; /* RACH is always 8-bit on PTCCH/U */
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

static SoftVector *demodAnyBurst_va(const signalVector &burst, CorrType type, int sps, int rach_max_toa, int tsc)
{
	auto conved_beg = reinterpret_cast<const std::complex<float> *>(&burst.begin()[0]);
	std::complex<float> chan_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];
	float ncmax;
	const unsigned burst_len_bits = 148 + 8;
	sbit_t demodded_softbits[burst_len_bits];
	SoftVector *bits = new SoftVector(burst_len_bits);

	if (type == CorrType::TSC) {
		auto rach_burst_start = get_norm_chan_imp_resp(conved_beg, chan_imp_resp, &ncmax, tsc);
		rach_burst_start = std::max(rach_burst_start, 0);
		detect_burst_nb(conved_beg, chan_imp_resp, rach_burst_start, demodded_softbits);
	} else {
		auto normal_burst_start = get_access_imp_resp(conved_beg, chan_imp_resp, &ncmax, 0);
		normal_burst_start = std::max(normal_burst_start, 0);
		detect_burst_ab(conved_beg, chan_imp_resp, normal_burst_start, demodded_softbits, rach_max_toa);
	}

	float *s = &bits->begin()[0];
	for (unsigned int i = 0; i < 148; i++)
		s[i] = demodded_softbits[i] * -1;
	for (unsigned int i = 148; i < burst_len_bits; i++)
		s[i] = 0;
	return bits;
}

#define USE_VA

#ifdef USE_VA
// signalvector is owning despite claiming not to, but we can pretend, too..
static void dummy_free(void *wData){};
static void *dummy_alloc(size_t newSize)
{
	return 0;
};
#endif

/* Convert a soft bit normalized to 0..1 (1.0 = confident '1') into the
 * sbit_t domain (-127..127) used by osmo_trxd_burst_ind, matching the wire
 * encoding of osmo_trxd_burst_ind_build()/soft_bits_build() (byte = 127 -
 * sbit): a value of 1.0 rounds to a raw byte of 255, which the TRXD wire
 * format reserves as -127 (see soft_bits_parse()), so clamp accordingly. */
static inline sbit_t float_soft_bit_to_sbit(float x)
{
  int v = 127 - (int) lround(x * 255.0);
  if (v > 127)
    v = 127;
  if (v < -127)
    v = -127;
  return (sbit_t) v;
}

/*
 * Pull bursts from the FIFO and handle according to the slot
 * and burst correlation type. Equalzation is currently disabled.
 * returns 0 on success (bi filled), negative on error (bi content undefined):
 *        -ENOENT: timeslot is off (fn and tn in bi are filled),
 *        -EIO: read error
 */
int Transceiver::pullRadioVector(size_t chan, struct osmo_trxd_burst_ind *bi)
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
  float soft_bits[OSMO_TRXD_BURST_LEN_MAX];
  static complex burst_shift_buffer[625];
  static signalVector shift_vec(burst_shift_buffer, 0, 625, dummy_alloc, dummy_free);
  signalVector *shvec_ptr = &shift_vec;

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
  memset(bi, 0, sizeof(*bi));
  bi->flags = OSMO_TRXD_F_MOD_TYPE | OSMO_TRXD_F_TS_INFO | OSMO_TRXD_F_CI_CB;
  bi->fn = burstTime.FN();
  bi->tn = burstTime.TN();
  bi->mod = OSMO_TRXD_MOD_T_GMSK;
  bi->tsc_set = 0; /* TODO: we only support tsc_set 0 right now */

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
  bi->rssi = (int8_t) lround(-(20.0 * log10(rxFullScale / avg) + rssi_offset));

  if (type == IDLE)
    goto ret_idle;

  max_toa = (type == RACH || type == EXT_RACH) ?
            mMaxExpectedDelayAB : mMaxExpectedDelayNB;

  if (cfg->use_va) {
    // shifted burst copy to make the old demod and detection happy
    std::copy(burst->begin() + 20, burst->end() - 20, shift_vec.begin());
  } else {
    shvec_ptr = burst;
  }

  /* Detect normal or RACH bursts */
  rc = detectAnyBurst(*shvec_ptr, mTSC, BURST_THRESH, cfg->rx_sps, type, max_toa, &ebp);
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

  if (cfg->use_va) {
    scaleVector(*burst, { (1. / (float)((1 << 14) - 1)), 0 });
    rxBurst = demodAnyBurst_va(*burst, (CorrType)rc, cfg->rx_sps, max_toa, mTSC);
  } else {
    rxBurst = demodAnyBurst(*shvec_ptr, (CorrType)rc, cfg->rx_sps, &ebp);
  }

  bi->toa256 = (int16_t) lround(ebp.toa * 256.0);
  bi->tsc = ebp.tsc;
  bi->ci_cb = (int16_t) ((ebp.ci * 10) + 0.5);

  /* EDGE demodulator returns 444 (gSlotLen * 3) bits */
  if (rxBurst->size() == EDGE_BURST_NBITS) {
    bi->mod = OSMO_TRXD_MOD_T_8PSK;
    bi->burst_len = EDGE_BURST_NBITS;
  } else { /* size() here is actually gSlotLen + 8, due to guard periods */
    bi->mod = OSMO_TRXD_MOD_T_GMSK;
    bi->burst_len = gSlotLen;
  }

  // Convert -1..+1 soft bits to 0..1 soft bits, then to the sbit_t wire domain
  vectorSlicer(soft_bits, rxBurst->begin(), bi->burst_len);
  for (size_t i = 0; i < bi->burst_len; i++)
    bi->burst[i] = float_soft_bit_to_sbit(soft_bits[i]);

  delete rxBurst;
  delete radio_burst;
  return 0;

ret_idle:
  if (ctr_changed)
    dispatch_trx_rate_ctr_change(state, chan);
  bi->flags |= OSMO_TRXD_F_NOPE_IND;
  bi->burst_len = 0;
  delete radio_burst;
  return 0;
}

void Transceiver::reset()
{
  for (size_t i = 0; i < mTxPriorityQueues.size(); i++)
    mTxPriorityQueues[i].clear();
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
  struct osmo_trxc_msg cmd, rsp;
  ctrl_msg cmd_received;
  ctrl_msg cmd_to_send;
  int msgLen;
  int rc;
  ctrl_sock_state& s = mCtrlSockets[chan];

  /* Attempt to read from control socket */
  msgLen = read(s.conn_bfd.fd, cmd_received.data, sizeof(cmd_received.data) - 1);
  if (msgLen < 0 && errno == EAGAIN)
      return 0; /* Try again later */
  if (msgLen <= 0) {
    LOGCHAN(chan, DTRXCTRL, NOTICE) << "mCtrlSockets read(" << s.conn_bfd.fd << ") failed: " << msgLen;
    return -EIO;
  }

  rc = osmo_trxc_msg_parse(&cmd, cmd_received.data, msgLen);
  if (rc < 0 || cmd.type != OSMO_TRXC_MT_CMD) {
    LOGCHAN(chan, DTRXCTRL, NOTICE) << "bogus message on control interface";
    return -EIO;
  }

  memset(&rsp, 0, sizeof(rsp));
  rsp.type = OSMO_TRXC_MT_RSP;
  rsp.status = 1; /* default: NACK, cleared to 0 on the success path of each verb below */
  snprintf(rsp.cmd, sizeof(rsp.cmd), "%s", cmd.cmd);

  LOGCHAN(chan, DTRXCTRL, INFO) << "command is '" << osmo_trxc_msg_name(&cmd) << "'";

  if (!strcmp(cmd.cmd, "POWEROFF")) {
    stop();
    rsp.status = 0;
  } else if (!strcmp(cmd.cmd, "POWERON")) {
    if (start()) {
      rsp.status = 0;
      for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++)
          mHandover[i][j] = false;
      }
    }
  } else if (!strcmp(cmd.cmd, "HANDOVER")) {
    unsigned ts = 0, ss = 0;
    if (osmo_trxc_msg_params_scan(&cmd, "%u %u", &ts, &ss) == 2 && ts <= 7 && ss <= 7) {
      mHandover[ts][ss] = true;
      rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%u %u", ts, ss);
  } else if (!strcmp(cmd.cmd, "NOHANDOVER")) {
    unsigned ts = 0, ss = 0;
    if (osmo_trxc_msg_params_scan(&cmd, "%u %u", &ts, &ss) == 2 && ts <= 7 && ss <= 7) {
      mHandover[ts][ss] = false;
      rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%u %u", ts, ss);
  } else if (!strcmp(cmd.cmd, "SETMAXDLY")) {
    //set expected maximum time-of-arrival for Access Bursts
    int maxDelay = 0;
    if (osmo_trxc_msg_params_scan(&cmd, "%d", &maxDelay) == 1) {
      mMaxExpectedDelayAB = maxDelay; // 1 GSM symbol is approx. 1 km
      rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%d", maxDelay);
  } else if (!strcmp(cmd.cmd, "SETMAXDLYNB")) {
    //set expected maximum time-of-arrival for Normal Bursts
    int maxDelay = 0;
    if (osmo_trxc_msg_params_scan(&cmd, "%d", &maxDelay) == 1) {
      mMaxExpectedDelayNB = maxDelay; // 1 GSM symbol is approx. 1 km
      rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%d", maxDelay);
  } else if (!strcmp(cmd.cmd, "SETRXGAIN")) {
    int newGain = 0;
    if (osmo_trxc_msg_params_scan(&cmd, "%d", &newGain) == 1) {
      newGain = mRadioInterface->setRxGain(newGain, chan);
      rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%d", newGain);
  } else if (!strcmp(cmd.cmd, "NOISELEV")) {
    if (mOn) {
      float lev = mStates[chan].mNoiseLev;
      rsp.status = 0;
      snprintf(rsp.params, sizeof(rsp.params), "%d",
              (int) round(20.0 * log10(rxFullScale / lev)));
    } else {
      snprintf(rsp.params, sizeof(rsp.params), "0");
    }
  } else if (!strcmp(cmd.cmd, "SETPOWER")) {
    int power = 0;
    if (osmo_trxc_msg_params_scan(&cmd, "%d", &power) == 1) {
      power = mRadioInterface->setPowerAttenuation(power, chan);
      mStates[chan].mPower = power;
      rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%d", power);
  } else if (!strcmp(cmd.cmd, "ADJPOWER")) {
    int power = mStates[chan].mPower, step;
    if (osmo_trxc_msg_params_scan(&cmd, "%d", &step) == 1) {
      power = mStates[chan].mPower + step;
      power = mRadioInterface->setPowerAttenuation(power, chan);
      mStates[chan].mPower = power;
      rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%d", power);
  } else if (!strcmp(cmd.cmd, "NOMTXPOWER")) {
    int power = mRadioInterface->getNominalTxPower(chan);
    rsp.status = 0;
    snprintf(rsp.params, sizeof(rsp.params), "%d", power);
  } else if (!strcmp(cmd.cmd, "RXTUNE")) {
    // tune receiver
    int freqKhz = 0;
    if (osmo_trxc_msg_params_scan(&cmd, "%d", &freqKhz) == 1) {
      mRxFreq = (freqKhz + cfg->freq_offset_khz) * 1e3;
      if (!mRadioInterface->tuneRx(mRxFreq, chan))
        LOGCHAN(chan, DTRXCTRL, FATAL) << "RX failed to tune";
      else
        rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%d", freqKhz);
  } else if (!strcmp(cmd.cmd, "TXTUNE")) {
    // tune txmtr
    int freqKhz = 0;
    if (osmo_trxc_msg_params_scan(&cmd, "%d", &freqKhz) == 1) {
      mTxFreq = (freqKhz + cfg->freq_offset_khz) * 1e3;
      if (!mRadioInterface->tuneTx(mTxFreq, chan))
        LOGCHAN(chan, DTRXCTRL, FATAL) << "TX failed to tune";
      else
        rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%d", freqKhz);
  } else if (!strcmp(cmd.cmd, "SETTSC")) {
    // set TSC
    unsigned TSC = 0;
    if (osmo_trxc_msg_params_scan(&cmd, "%u", &TSC) == 1 && TSC <= 7) {
      LOGC(DTRXCTRL, NOTICE) << "Changing TSC from " << mTSC << " to " << TSC;
      mTSC = TSC;
      rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%u", TSC);
  } else if (!strcmp(cmd.cmd, "SETSLOT")) {
    // set slot type
    struct osmo_trxc_setslot ss;
    if (osmo_trxc_setslot_parse(&ss, &cmd) < 0) {
      LOGCHAN(chan, DTRXCTRL, NOTICE) << "bogus message on control interface";
    } else if (ss.vamos) {
      LOGCHAN(chan, DTRXCTRL, NOTICE) << "VAMOS channel combinations are not supported";
    } else {
      mStates[chan].chanType[ss.tn] = (ChannelCombination) ss.chan_comb;
      setModulus(ss.tn, chan);
      rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%s", cmd.params);
  } else if (!strcmp(cmd.cmd, "SETFORMAT")) {
    // set TRXD protocol version
    unsigned version_recv = 0;
    osmo_trxc_msg_params_scan(&cmd, "%u", &version_recv);
    LOGCHAN(chan, DTRXCTRL, INFO) << "BTS requests TRXD version switch: " << version_recv;
    if (version_recv > TRX_DATA_FORMAT_VER) {
      LOGCHAN(chan, DTRXCTRL, INFO) << "rejecting TRXD version " << version_recv
                                    << " in favor of " <<  TRX_DATA_FORMAT_VER;
      rsp.status = TRX_DATA_FORMAT_VER;
    } else {
      LOGCHAN(chan, DTRXCTRL, NOTICE) << "switching to TRXD version " << version_recv;
      mVersionTRXD[chan] = version_recv;
      rsp.status = version_recv;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%u", version_recv);
  } else if (!strcmp(cmd.cmd, "RFMUTE")) {
    // (Un)mute RF TX and RX
    unsigned mute = 0;
    if (osmo_trxc_msg_params_scan(&cmd, "%u", &mute) == 1) {
      mStates[chan].mMuted = mute ? true : false;
      rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%u", mute);
  } else if (!strcmp(cmd.cmd, "_SETBURSTTODISKMASK")) {
    // debug command! may change or disappear without notice
    // set a mask which bursts to dump to disk
    int mask = 0;
    if (osmo_trxc_msg_params_scan(&cmd, "%d", &mask) == 1) {
      mWriteBurstToDiskMask = mask;
      rsp.status = 0;
    }
    snprintf(rsp.params, sizeof(rsp.params), "%d", mask);
  } else {
    LOGCHAN(chan, DTRXCTRL, NOTICE) << "bogus command " << cmd.cmd << " on control interface.";
    snprintf(rsp.cmd, sizeof(rsp.cmd), "%s", OSMO_TRXC_CMD_ERR);
  }

  LOGCHAN(chan, DTRXCTRL, INFO) << "response is '" << osmo_trxc_msg_name(&rsp) << "'";

  rc = osmo_trxc_msg_build(cmd_to_send.data, sizeof(cmd_to_send.data), &rsp);
  if (rc < 0) {
    LOGCHAN(chan, DTRXCTRL, ERROR) << "failed to build response (rc=" << rc << ")";
    return -EIO;
  }
  ctrl_sock_send(cmd_to_send, chan);
  return 0;
}

bool Transceiver::handleBurstReq(size_t chan, const struct osmo_trxd_burst_req *br)
{
  switch (br->burst_len) {
  case gSlotLen: /* GSM burst */
    break;
  case EDGE_BURST_NBITS: /* EDGE burst */
    if (cfg->tx_sps != 4) {
      LOGCHAN(chan, DTRXDDL, ERROR) << "EDGE burst received but SPS is set to " << cfg->tx_sps;
      return false;
    }
    break;
  default:
    LOGCHAN(chan, DTRXDDL, ERROR) << "badly formatted packet on GSM->TRX interface (burst_len="
                                  << br->burst_len << ")";
    return false;
  }

  LOGCHAN(chan, DTRXDDL, DEBUG) << "Rx TRXD message: fn=" << br->fn << ", tn=" << unsigned(br->tn)
                                << ", burst_len=" << br->burst_len;

  TransceiverState *state = &mStates[chan];
  GSM::Time currTime = GSM::Time(br->fn, br->tn);

  /* Verify proper FN order in DL stream */
  if (state->first_dl_fn_rcv[br->tn]) {
    int32_t delta = GSM::FNDelta(currTime.FN(), state->last_dl_time_rcv[br->tn].FN());
    if (delta == 1) {
        /* usual expected scenario, continue code flow */
    } else if (delta == 0) {
      LOGCHAN(chan, DTRXDDL, INFO) << "Rx TRXD msg with repeated FN " << currTime;
      state->ctrs.tx_trxd_fn_repeated++;
      dispatch_trx_rate_ctr_change(state, chan);
      return true;
    } else if (delta < 0) {
      LOGCHAN(chan, DTRXDDL, INFO) << "Rx TRXD msg with previous FN " << currTime
                                     << " vs last " << state->last_dl_time_rcv[br->tn];
       state->ctrs.tx_trxd_fn_outoforder++;
       dispatch_trx_rate_ctr_change(state, chan);
       /* Allow adding radio vector below, since it gets sorted in the queue */
    } else if (chan == 0 && state->mFiller == FILLER_ZERO) {
        /* delta > 1. Some FN was lost in the middle. We can only easily rely
         * on consecutive FNs in TRX0 since it must transmit continuously in all
         * setups. Also, osmo-trx supports optionally filling empty bursts on
         * its own. In that case bts-trx is not obliged to submit all bursts. */
      LOGCHAN(chan, DTRXDDL, INFO) << "Rx TRXD msg with future FN " << currTime
                                     << " vs last " << state->last_dl_time_rcv[br->tn]
                                     << ", " << delta - 1 << " FN lost";
      state->ctrs.tx_trxd_fn_skipped += delta - 1;
      dispatch_trx_rate_ctr_change(state, chan);
    }
    if (delta > 0)
      state->last_dl_time_rcv[br->tn] = currTime;
  } else { /* Initial check, simply store state */
    state->first_dl_fn_rcv[br->tn] = true;
    state->last_dl_time_rcv[br->tn] = currTime;
  }

  BitVector newBurst(br->burst_len);
  BitVector::iterator itr = newBurst.begin();
  const ubit_t *bufferItr = br->burst;
  while (itr < newBurst.end())
    *itr++ = *bufferItr++;

  addRadioVector(chan, newBurst, br->att, currTime);

  return true;
}

bool Transceiver::driveTxPriorityQueue(size_t chan)
{
  char buffer[TRX_TRXD_BATCH_BUF_SIZE];
  int msgLen;
  struct osmo_trxd_parse_state st;
  struct osmo_trxd_burst_req br;
  const uint8_t *pos;
  size_t remain;
  int rc;

  // check data socket
  msgLen = read(mDataSockets[chan], buffer, sizeof(buffer));
  if (msgLen <= 0) {
    LOGCHAN(chan, DTRXDDL, NOTICE) << "mDataSockets read(" << mDataSockets[chan] << ") failed: " << msgLen;
    return false;
  }

  /* Normally a single PDU per datagram (TRXDv0/v1, or unbatched TRXDv2),
   * but the BTS side may also send us a TRXDv2 batch (multiple PDUs for
   * the same FN, one per timeslot): keep parsing while BATCH.ind is set. */
  osmo_trxd_parse_state_init(&st);
  pos = (const uint8_t *) buffer;
  remain = (size_t) msgLen;

  do {
    rc = osmo_trxd_burst_req_parse(&st, &br, pos, remain);
    if (rc < 0) {
      LOGCHAN(chan, DTRXDDL, ERROR) << "failed to parse BURST.req (rc=" << rc << ")";
      return false;
    }

    if (!handleBurstReq(chan, &br))
      return false;

    pos += rc;
    remain -= rc;
  } while (br.flags & OSMO_TRXD_F_BATCH_IND);

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

void Transceiver::logRxBurst(size_t chan, const struct osmo_trxd_burst_ind *bi)
{
  std::ostringstream os;
  for (size_t i = 0; i < bi->burst_len; i++) {
    /* sbit_t polarity: <0 confident '1', 127 confident '0' (see float_soft_bit_to_sbit()) */
    if (bi->burst[i] < 0) os << "1";
    else if (bi->burst[i] < 64) os << "|";
    else if (bi->burst[i] < 127) os << "'";
    else os << "-";
  }

  double rssi_offset = rssiOffset(chan);
  double noise_dbfs = 20.0 * log10(rxFullScale / mStates[chan].mNoiseLev) + rssi_offset;
  double rssi_dbfs = -bi->rssi - rssi_offset;

  LOGCHAN(chan, DTRXDUL, DEBUG) << std::fixed << std::right
    << " time: "   << unsigned(bi->tn) << ":" << bi->fn
    << " RSSI: "   << std::setw(5) << std::setprecision(1) << rssi_dbfs
                   << "dBFS/" << std::setw(6) << int(bi->rssi) << "dBm"
    << " noise: "  << std::setw(5) << std::setprecision(1) << (noise_dbfs - rssi_offset)
                   << "dBFS/" << std::setw(6) << -noise_dbfs << "dBm"
    << " TOA: "    << std::setw(5) << std::setprecision(2) << (bi->toa256 / 256.0)
    << " C/I: "    << std::setw(5) << std::setprecision(2) << (bi->ci_cb / 10.0) << "dB"
    << " bits: "   << os;
}


bool Transceiver::sendBurstInd(size_t chan, const struct osmo_trxd_burst_ind *bi)
{
  struct msgb *msg;
  int rc;

  msg = msgb_alloc(OSMO_TRXD_MSG_BUF_SIZE, "trxd_burst_ind");
  if (!msg)
    return false;

  rc = osmo_trxd_burst_ind_build(msg, mVersionTRXD[chan], bi);
  if (rc < 0) {
    LOGCHAN(chan, DTRXDUL, ERROR) << "failed to build BURST.ind (rc=" << rc << ")";
    msgb_free(msg);
    return false;
  }
  osmo_trxd_build_fin(msg, mVersionTRXD[chan]);

  rc = write(mDataSockets[chan], msgb_data(msg), msgb_length(msg));
  msgb_free(msg);
  if (rc < 0) {
    LOGCHAN(chan, DTRXDUL, NOTICE) << "mDataSockets write(" << mDataSockets[chan] << ") failed: " << rc;
    return false;
  }

  return true;
}

bool Transceiver::flushBurstIndBatch(size_t chan)
{
  struct msgb *msg = mBurstIndBatch[chan];
  int rc;

  if (!msg || msgb_length(msg) == 0)
    return true; /* nothing accumulated */

  osmo_trxd_build_fin(msg, mVersionTRXD[chan]);

  rc = write(mDataSockets[chan], msgb_data(msg), msgb_length(msg));
  msgb_trim(msg, 0);
  if (rc < 0) {
    LOGCHAN(chan, DTRXDUL, NOTICE) << "mDataSockets write(" << mDataSockets[chan] << ") failed: " << rc;
    return false;
  }

  return true;
}

bool Transceiver::queueBurstIndBatched(size_t chan, const struct osmo_trxd_burst_ind *bi)
{
  struct msgb *msg = mBurstIndBatch[chan];
  int rc;

  if (msg && msgb_length(msg) > 0 && mBurstIndBatchFn[chan] != bi->fn) {
    /* previous frame's batch is complete (a PDU for a new FN showed up) */
    if (!flushBurstIndBatch(chan))
      return false;
  }

  if (!msg) {
    msg = msgb_alloc(TRX_TRXD_BATCH_BUF_SIZE, "trxd_burst_ind_batch");
    if (!msg)
      return false;
    mBurstIndBatch[chan] = msg;
  }

  if (msgb_length(msg) == 0)
    mBurstIndBatchFn[chan] = bi->fn;

  rc = osmo_trxd_burst_ind_build(msg, mVersionTRXD[chan], bi);
  if (rc < 0) {
    LOGCHAN(chan, DTRXDUL, ERROR) << "failed to build batched BURST.ind (rc=" << rc << ")";
    msgb_trim(msg, 0);
    return false;
  }

  return true;
}

bool Transceiver::driveReceiveFIFO(size_t chan)
{
  struct osmo_trxd_burst_ind bi;
  int rc;

  if ((rc = pullRadioVector(chan, &bi)) < 0) {
    if (rc == -ENOENT) { /* timeslot off, continue processing */
      LOGCHAN(chan, DTRXDUL, DEBUG) << unsigned(bi.tn) << ":" << bi.fn << " timeslot is off";
      return true;
    }
    return false; /* other errors: we want to stop the process */
  }

  if (!(bi.flags & OSMO_TRXD_F_NOPE_IND) && log_check_level(DTRXDUL, LOGL_DEBUG))
    logRxBurst(chan, &bi);

  /* TODO: make batching configurable via VTY, so it can be disabled even
   * when TRXDv2 is negotiated (e.g. to trade datagram count for latency) */
  if (mVersionTRXD[chan] < 2)
    return sendBurstInd(chan, &bi);

  return queueBurstIndBatched(chan, &bi);
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
  char command[64];
  int rc;
  // FIXME -- This should be adaptive.
  uint32_t fn = mTransmitDeadlineClock.FN() + 2;

  rc = osmo_trxc_clock_ind_build(command, sizeof(command), fn);
  if (rc < 0) {
    LOGC(DTRXCLK, ERROR) << "failed to build IND CLOCK " << fn;
    return false;
  }

  LOGC(DTRXCLK, INFO) << "sending " << command;

  rc = write(mClockSocket, command, strlen(command) + 1);
  if (rc <= 0) {
    LOGC(DTRXCLK, ERROR) << "mClockSocket write(" << mClockSocket << ") failed: " << rc;
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
