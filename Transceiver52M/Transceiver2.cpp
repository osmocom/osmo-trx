/*
* Copyright 2008, 2009, 2010 Free Software Foundation, Inc.
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

#include "BitVector.h"
#include "osmocom/core/bits.h"
#include <stdio.h>
#include <Logger.h>
#include "Transceiver2.h"
#include <grgsm_vitac/grgsm_vitac.h>

extern "C" {
#include "sch.h"
}

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "l1if.h"

using namespace GSM;

#define USB_LATENCY_INTRVL		10,0

#if USE_UHD
#  define USB_LATENCY_MIN		6,7
#else
#  define USB_LATENCY_MIN		1,1
#endif

/* Clock indication interval in frames */
#define CLK_IND_INTERVAL		100

/* Number of running values use in noise average */
#define NOISE_CNT			20
#define FREQ_CNT			20

TransceiverState::TransceiverState()
  : mRetrans(false), mFreqOffsets(FREQ_CNT), mode(Transceiver2::TRX_MODE_OFF)
{
  for (int i = 0; i < 8; i++) {
    chanType[i] = Transceiver2::NONE;
    fillerModulus[i] = 26;
    prevFrame[i] = NULL;

    for (int n = 0; n < 102; n++)
      fillerTable[n][i] = NULL;
  }
}

TransceiverState::~TransceiverState()
{
  for (int i = 0; i < 8; i++) {
    for (int n = 0; n < 102; n++)
      delete fillerTable[n][i];
  }
}

void TransceiverState::init(size_t slot, signalVector *burst, bool fill)
{
  signalVector *filler;

  for (int i = 0; i < 102; i++) {
    if (fill)
      filler = new signalVector(*burst);
    else
      filler = new signalVector(burst->size());

    fillerTable[i][slot] = filler;
  }
}
extern void initvita();
Transceiver2::Transceiver2(int wBasePort,
			 const char *TRXAddress,
			 size_t wSPS, size_t wChans,
			 GSM::Time wTransmitLatency,
			 RadioInterface *wRadioInterface)
  : mChans(wChans), rx_sps(4), tx_sps(4), mAddr(TRXAddress), mTransmitLatency(wTransmitLatency),
   mTxPriorityQueues(mChans), mReceiveFIFO(mChans), mRxServiceLoopThreads(mChans),
   mControlServiceLoopThreads(mChans), mTxPriorityQueueServiceLoopThreads(mChans), mRadioInterface(wRadioInterface),
   mOn(false), mTxFreq(0.0), mRxFreq(0.0), mPower(-10), mMaxExpectedDelay(0), mBSIC(-1), mStates(mChans)
{
  GSM::Time startTime(random() % gHyperframe,0);

  mLowerLoopThread = new Thread(32768);

  mTransmitDeadlineClock = startTime;
  mLastClockUpdateTime = startTime;
  mLatencyUpdateTime = startTime;
  mRadioInterface->getClock()->set(startTime);

  txFullScale = mRadioInterface->fullScaleInputValue();
  rxFullScale = mRadioInterface->fullScaleOutputValue();

  for (int i = 0; i < 8; i++)
    mRxSlotMask[i] = 0;

  initvita();
}

Transceiver2::~Transceiver2()
{
  stop();

  sigProcLibDestroy();

  for (size_t i = 0; i < mChans; i++) {
    mTxPriorityQueues[i].clear();
  }
}

bool Transceiver2::init(bool filler)
{

  if (!mChans) {
    LOG(ALERT) << "No channels assigned";
    return false;
  }

  if (!sigProcLibSetup()) {
    LOG(ALERT) << "Failed to initialize signal processing library";
    return false;
  }



  /* Filler table retransmissions - support only on channel 0 */
  if (filler)
    mStates[0].mRetrans = true;


  for (size_t i = 0; i < mChans; i++) {
    mControlServiceLoopThreads[i] = new Thread(32768);
    mTxPriorityQueueServiceLoopThreads[i] = new Thread(32768);
    mRxServiceLoopThreads[i] = new Thread(32768);

    for (size_t n = 0; n < 8; n++) {
      // burst = modulateBurst(gDummyBurst, 8 + (n % 4 == 0), tx_sps);
      // scaleVector(*burst, txFullScale);
      // mStates[i].init(n, burst, filler && !i);
      // delete burst;
    }
  }

  return true;
}

void Transceiver2::addRadioVector(size_t chan, BitVector &bits,
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

  if (mStates[0].mode != TRX_MODE_BTS)
    return;

  burst = modulateBurst(bits, 8 + (wTime.TN() % 4 == 0), tx_sps);
  scaleVector(*burst, txFullScale * pow(10, -RSSI / 10));

  radio_burst = new radioVector(wTime, burst);

  mTxPriorityQueues[chan].write(radio_burst);
}

void Transceiver2::updateFillerTable(size_t chan, radioVector *burst)
{
  int TN, modFN;
  TransceiverState *state = &mStates[chan];

  TN = burst->getTime().TN();
  modFN = burst->getTime().FN() % state->fillerModulus[TN];

  delete state->fillerTable[modFN][TN];
  state->fillerTable[modFN][TN] = burst->getVector();
  burst->setVector(NULL);
}

void Transceiver2::pushRadioVector(GSM::Time &nowTime)
{
  int TN, modFN;
  radioVector *burst;
  TransceiverState *state;
  std::vector<signalVector *> bursts(mChans);
  std::vector<bool> zeros(mChans, false);
  std::vector<bool> filler(mChans, true);

  for (size_t i = 0; i < mChans; i ++) {
    state = &mStates[i];

    while ((burst = mTxPriorityQueues[i].getStaleBurst(nowTime))) {
      LOG(NOTICE) << "dumping STALE burst in TRX->USRP interface";
      if (state->mRetrans)
        updateFillerTable(i, burst);
      delete burst;
    }

    TN = nowTime.TN();
    modFN = nowTime.FN() % state->fillerModulus[TN];

    bursts[i] = state->fillerTable[modFN][TN];
    if (state->mode == TRX_MODE_BTS)
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

void Transceiver2::setModulus(size_t timeslot, size_t chan)
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


CorrType Transceiver2::expectedCorrType(GSM::Time currTime,
                                                    size_t chan)
{
  TransceiverState *state = &mStates[chan];
  unsigned burstTN = currTime.TN();
  unsigned burstFN = currTime.FN();

  if (state->mode == TRX_MODE_MS_TRACK) {
    /* 102 modulus case currently unhandled */
    if (state->fillerModulus[burstTN] > 52)
      return OFF;

    int modFN = burstFN % state->fillerModulus[burstTN];
    unsigned long long reg = (unsigned long long) 1 << modFN;
    if (reg & mRxSlotMask[burstTN])
      return TSC;
    else
      return OFF;
  }

  switch (state->chanType[burstTN]) {
  case NONE:
    return OFF;
    break;
  case FILL:
    return IDLE;
    break;
  case I:
    return TSC;
    /*if (burstFN % 26 == 25) 
      return IDLE;
    else
      return TSC;*/
    break;
  case II:
    return TSC;
    break;
  case III:
    return TSC;
    break;
  case IV:
  case VI:
    return RACH;
    break;
  case V: {
    int mod51 = burstFN % 51;
    if ((mod51 <= 36) && (mod51 >= 14))
      return RACH;
    else if ((mod51 == 4) || (mod51 == 5))
      return RACH;
    else if ((mod51 == 45) || (mod51 == 46))
      return RACH;
    else
      return TSC;
    break;
  }
  case VII:
    if ((burstFN % 51 <= 14) && (burstFN % 51 >= 12))
      return IDLE;
    else
      return TSC;
    break;
  case XIII: {
    int mod52 = burstFN % 52;
    if ((mod52 == 12) || (mod52 == 38))
      return RACH;
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


/* Detect SCH synchronization sequence within a burst */
bool Transceiver2::detectSCH(TransceiverState *state,
                            signalVector &burst,
                            struct estim_burst_params *ebp)
{
  int shift;
  sch_detect_type full;
  float mag, threshold = 5.0;

  full = (state->mode == TRX_MODE_MS_TRACK) ?
	 sch_detect_type::SCH_DETECT_NARROW : sch_detect_type::SCH_DETECT_FULL;

  if (!detectSCHBurst(burst, threshold, rx_sps, full, ebp))
    return false;

  std::clog << "SCH : Timing offset     " << ebp->toa << " symbols" <<  std::endl;

  mag = fabsf(ebp->toa);
  if (mag < 1.0f)
    return true;

  shift = (int) (mag / 2.0f);
  if (!shift)
    shift++;

  shift = ebp->toa > 0 ? shift : -shift;
 std::clog << "SCH : shift ->     " << shift << " symbols" << std::endl;
  mRadioInterface->applyOffset(shift);
  return false;
}

#define SCH_BIT_SCALE	64

/* Decode SCH burst */
bool Transceiver2::decodeSCH(SoftVector *burst, GSM::Time *time)
{
  int fn;
  struct sch_info sch;
  ubit_t info[GSM_SCH_INFO_LEN];
  sbit_t data[GSM_SCH_CODED_LEN];

  if (burst->size() < 156) {
    std::clog << "Invalid SCH burst length" << std::endl;
    return false;
  }

  float_to_sbit(&(*burst)[3], &data[0], SCH_BIT_SCALE, 39);
  float_to_sbit(&(*burst)[106], &data[39], SCH_BIT_SCALE, 39);

  if (!gsm_sch_decode(info, data)) {
    gsm_sch_parse(info, &sch);

    mBSIC = sch.bsic;
    mTSC = mBSIC & 0x7;

//    std::clog << "SCH : Decoded values" << std::endl;
//    std::clog << "    BSIC: " << sch.bsic << std::endl;
//    std::clog << "    T1  : " << sch.t1 << std::endl;
//    std::clog << "    T2  : " << sch.t2 << std::endl;
//    std::clog << "    T3p : " << sch.t3p << std::endl;
//    std::clog << "    FN  : " << gsm_sch_to_fn(&sch) << std::endl;

    fn = gsm_sch_to_fn(&sch);
    if (fn < 0) {
      std::clog << "SCH : Failed to convert FN " << std::endl;
      return false;
    }

    time->FN(fn);
    time->TN(0);
  } else {
    std::clog << "Invalid SCH decode!!" << std::endl;
    return false;
  }

  return true;
}

#define FCCH_OFFSET_LIMIT	5e3
#define FCCH_ADJUST_LIMIT	20.0

/* Apply FCCH frequency correction */
bool Transceiver2::correctFCCH(TransceiverState *state, signalVector *burst)
{
  double offset;

  if (!burst)
    return false;

  offset = gsm_fcch_offset((float *) burst->begin(), burst->size());
  //std::cout << "XXXX FCCH: Frequency offset  " << offset << " Hz" << std::endl;
  if (offset > FCCH_OFFSET_LIMIT)
    return false;

  state->mFreqOffsets.insert(offset);

  if (state->mFreqOffsets.full()) {
    double avg = state->mFreqOffsets.avg();
    std::clog << "FCCH: Frequency offset  " << avg << " Hz" << std::endl;

    if (fabs(avg) > FCCH_ADJUST_LIMIT) {
      mRadioInterface->tuneRxOffset(-avg);
      state->mFreqOffsets.reset();
    }
  }

  return true;
}

extern int process_vita_burst(std::complex<float>* input, int tsc, unsigned char* output_binary);
/*
 * Pull bursts from the FIFO and handle according to the slot
 * and burst correlation type. Equalzation is currently disabled. 
 */
SoftVector *Transceiver2::pullRadioVector(GSM::Time &wTime, int &RSSI,
                                         int &timingOffset, size_t chan) __attribute__((optnone))
{
  struct estim_burst_params ebp;
  int rc;
  float pow, max = -1.0, avg = 1.0;
  int max_i = -1;
  signalVector *burst;
  SoftVector *bits = NULL;
  TransceiverState *state = &mStates[chan];
  bool printme = 0;
  GSM::Time sch_time, burst_time, diff_time;

  /* Blocking FIFO read */
  radioVector *radio_burst = mReceiveFIFO[chan]->read();
  if (!radio_burst)
    return NULL;

  /* Set time and determine correlation type */
  burst_time = radio_burst->getTime();

#if 0
  if (state->mode == TRX_MODE_MS_ACQUIRE) {
          switch(fbsb_acq_buf.s.load()) {
              case fbsb_par::fbsb_state::IDLE:
              case fbsb_par::fbsb_state::INIT:
                  fbsb_acq_buf.s.store(fbsb_par::fbsb_state::ACQ);
              case fbsb_par::fbsb_state::ACQ:
                  fbsb_acq_buf.take(radio_burst->getVector()->begin(), radio_burst->getVector()->size(), burst_time);

                  if(!fbsb_acq_buf.done()) {
                          delete radio_burst;
                      return nullptr;
                      }
                   fbsb_acq_buf.s.store(fbsb_par::fbsb_state::ACQ_COMPL);
//                  break;
              case fbsb_par::fbsb_state::ACQ_COMPL:


                  {
                  complex famp = 0;
                  int found_index;
                  auto foundat = fbsb_acq_buf.fcch(&famp, &found_index, false);
                  std::cerr << "@ " << found_index << std::endl;
                  foundat = found_index;

                  auto framelen = (3 + 142 + 3 + 8.25); // 1sps!;
                  int searchbegin = 8*framelen-20;
                  int searchend = 10*framelen+20;

                  if(famp.abs() < 2000 || foundat+searchend > fbsb_acq_buf.sz()) {
                          mReceiveFIFO[chan]->clear();
                          delete radio_burst;
                          fbsb_acq_buf.reset();
                          fbsb_acq_buf.s.store(fbsb_par::fbsb_state::ACQ);
                          return nullptr;
                      }
                  correctFCCH_raw(state, &fbsb_acq_buf.fbsb_buf[foundat], 98);

                  burst = new signalVector(searchend-searchbegin, GSM::gRACHSynchSequence.size());
//                  burst = new signalVector(fbsb_acq_buf.fbsb_buf, foundat+searchbegin, searchend-searchbegin);
                  memcpy(burst->begin(), &fbsb_acq_buf.fbsb_buf[foundat+searchbegin], (searchend-searchbegin) * sizeof(complex));
                  success = detectSCH(state, *burst, amp, toa, 0); // will "fail" if sample adjustment is required
                  std::cerr << "###detected sch: " << success << "at toa " << toa << std::endl;
                  mReceiveFIFO[chan]->clear();
                  delete radio_burst;

                  if(toa > 0) {
                   int how_many_ts = (found_index+searchbegin)/framelen;
//                   int how_many_fn = how_many_ts/8;
                   auto t = fbsb_acq_buf.rcvClock[how_many_ts];

                  diff_time = GSM::Time(sch_time.FN() - t.FN(),-t.TN());
                  mRadioInterface->adjustClock(diff_time);
                  mTransmitDeadlineClock = RadioClock::adjust(
                            mTransmitDeadlineClock,
                            diff_time);
                  }
                  if(!success) {
                  fbsb_acq_buf.reset();
                  fbsb_acq_buf.s.store(fbsb_par::fbsb_state::ACQ);
                  }
                  else
                    fbsb_acq_buf.s.store(fbsb_par::fbsb_state::DONE);
                  return nullptr;
                  }
              case fbsb_par::fbsb_state::DONE:
                  break;
              default:
                  //                  fbsb_acq_buf.s = fbsb_par::fbsb_state::WAIT;
                  return nullptr;
                  break;
                  /* no-op */
              }

      }
#endif
  CorrType type = expectedCorrType(burst_time, chan);

  switch (state->mode) {
  case TRX_MODE_MS_ACQUIRE:
    type = SCH;
    break;
  case TRX_MODE_MS_TRACK:
    if (gsm_sch_check_fn(burst_time.FN()) && burst_time.TN() == 0)
      type = SCH;
    else if(burst_time.TN() == 0 && !gsm_fcch_check_fn(burst_time.FN())) // all ts0, but not fcch or sch..
        type = TSC;
    else if (type == OFF)
      goto release;
    break;
  case TRX_MODE_BTS:
    if ((type == TSC) || (type == RACH))
      break;
  case TRX_MODE_OFF:
  default:
    goto release;
  }

  /* Select the diversity channel with highest energy */
  for (size_t i = 0; i < radio_burst->chans(); i++) {
    float pow = energyDetect(*radio_burst->getVector(i), 20 * rx_sps);
    if (pow > max) {
      max = pow;
      max_i = i;
    }
    avg += pow;
  }

  if (max_i < 0) {
    LOG(ALERT) << "Received empty burst";
    goto release;
  }

  /* Average noise on diversity paths and update global levels */
  burst = radio_burst->getVector(max_i);
  avg = sqrt(avg / radio_burst->chans());


  /* Detect normal or RACH bursts */
  if (type == SCH) {
    rc = detectSCH(state, *burst, &ebp);
    rc = rc > 0 ? rc : -1;
  } else {
    rc = detectAnyBurst(*burst, mTSC, BURST_THRESH, rx_sps, type, mMaxExpectedDelay, &ebp);
    if(rc > 0) {
      type = (CorrType)rc;
    }
  }


  if (rc < 0)
    goto release;

#if 0
  if(type == SCH){
    unsigned char outbin[148] = {0};

    bits = demodAnyBurst(*burst, type, rx_sps, &ebp);


    auto start = reinterpret_cast<float*>(burst->begin());
    for(int i=0; i < 625*2; i++)
      start[i] *= 1./2047.;

    auto ss = reinterpret_cast<std::complex<float>*>(burst->begin());

 //   int ret = process_vita_sc_burst(ss, mTSC, outbin, 0);

    {
      std::vector<gr_complex> channel_imp_resp(CHAN_IMP_RESP_LENGTH* d_OSR);
          /* Get channel impulse response */
      auto d_c0_burst_start = get_sch_chan_imp_resp(ss, &channel_imp_resp[0], 0, (SYNC_POS + SYNC_SEARCH_RANGE) * d_OSR + SYNC_POS * d_OSR );

      if(d_c0_burst_start < 0) {
        std::cerr << " fck! offset <0! " << ebp.toa << std::endl;
        d_c0_burst_start = 0;
      } else {
        std::cerr << " ## offset " << d_c0_burst_start << " vs " << ebp.toa << std::endl;
      }
      /* Perform MLSE detection */
      detect_burst(ss, &channel_imp_resp[0],
      d_c0_burst_start, outbin);
    }

    // auto bits2 = new SoftVector();
    // bits2->resize(156);
    // for(int i=0; i < 148; i++)
    //   (*bits2)[i] = outbin[i] < 1 ? -1 : 1;

    

    printme =  true;

    if(printme) {
      std::cerr << std::endl << "vita:" << std::endl;
      for(auto i : outbin)
        std::cerr << (int) i;
      std::cerr << std::endl << "org:" << std::endl;
      for(int i=0; i < 148; i++)
        std::cerr << (int) (bits->operator[](i) > 0 ? 1 : 0);
    }

  } else
#endif
  if (type == TSC) {
	  unsigned char outbin[148];

	  // bits = demodAnyBurst(*burst, type, rx_sps, &ebp);

	  {
		  auto start = reinterpret_cast<float *>(burst->begin());
		  for (int i = 0; i < 625 * 2; i++)
			  start[i] *= 1. / 2047.;

		  auto ss = reinterpret_cast<std::complex<float> *>(burst->begin());
		  // int ret = process_vita_burst(reinterpret_cast<std::complex<float>*>(burst->begin()), mTSC, outbin);

		  float ncmax, dcmax;
		  std::vector<gr_complex> channel_imp_resp(CHAN_IMP_RESP_LENGTH * d_OSR),
			  channel_imp_resp2(CHAN_IMP_RESP_LENGTH * d_OSR);
		  auto normal_burst_start = get_norm_chan_imp_resp(ss, &channel_imp_resp[0], &ncmax, mTSC);
		  auto dummy_burst_start = get_norm_chan_imp_resp(ss, &channel_imp_resp2[0], &dcmax, TS_DUMMY);
		  auto is_nb = ncmax > dcmax;

		  std::cerr << " ## " << is_nb << " o nb " << normal_burst_start << " db " << dummy_burst_start
			    << " vs " << ebp.toa << std::endl;

		  if (is_nb)
			  detect_burst(ss, &channel_imp_resp[0], normal_burst_start, outbin);
		  else
			  detect_burst(ss, &channel_imp_resp2[0], dummy_burst_start, outbin);
		  ;
	  }

	  bits = new SoftVector();
	  bits->resize(148);
	  for (int i = 0; i < 148; i++)
		  (*bits)[i] = outbin[i] < 1 ? -1 : 1;

	  // printme = ret >= 0 ? true : false;

	  // if(printme) {
	  //   std::cerr << std::endl << "vita:" << std::endl;
	  //   for(auto i : outbin)
	  //     std::cerr << (int) i;
	  //   std::cerr << std::endl << "org:" << std::endl;
	  // }
  } else {
	  /* Ignore noise threshold on MS mode for now */
	  //if ((type == SCH) || (avg - state->mNoiseLev > 0.0))
	  bits = demodAnyBurst(*burst, type, rx_sps, &ebp);

	  // if(printme)
	  //   for(int i=0; i < 148; i++)
	  //     std::cerr << (int) (bits->operator[](i) > 0 ? 1 : 0);
  }

  /* MS: Decode SCH and adjust GSM clock */
  if ((type != TSC) &&
       ((state->mode == TRX_MODE_MS_ACQUIRE) ||
       (state->mode == TRX_MODE_MS_TRACK))) {
    correctFCCH(state, state->prevFrame[burst_time.TN()]->getVector());

    if (decodeSCH(bits, &sch_time)) {
      if (state->mode == TRX_MODE_MS_ACQUIRE) {
          diff_time = GSM::Time(sch_time.FN() - burst_time.FN(),
                                -burst_time.TN());
          mRadioInterface->adjustClock(diff_time);
          mTransmitDeadlineClock = RadioClock::adjust(
					mTransmitDeadlineClock,
					diff_time);
          state->mode = TRX_MODE_MS_TRACK;

          burst_time = sch_time; //sch bits passed up might have a different time, so upper layers complain
          std::clog << "SCH : Locking GSM clock " << std::endl;
      } else {
        //std::clog << "SCH : Read SCH at FN " << burst_time.FN() << " FN51 " << burst_time.FN() % 51 << std::endl;
        wTime = burst_time;
        RSSI = (int) floor(20.0 * log10(rxFullScale / avg));
        timingOffset = (int) round(ebp.toa * 256.0 / rx_sps);
        return bits;
      }
    }
    else
        std::clog << "SCH : FAIL!!!!! SCH at FN " << burst_time.FN() << std::endl;

    goto release;
  }

  wTime = burst_time;
  RSSI = (int) floor(20.0 * log10(rxFullScale / avg));
  timingOffset = (int) round(ebp.toa * 256.0 / rx_sps);

  delete state->prevFrame[burst_time.TN()];
  state->prevFrame[burst_time.TN()] = radio_burst;

  return bits;

release:
  delete state->prevFrame[burst_time.TN()];
  state->prevFrame[burst_time.TN()] = radio_burst;
  delete bits;
  return NULL;
}

void Transceiver2::start()
{
  TransceiverChannel *chan;

  for (size_t i = 0; i < mControlServiceLoopThreads.size(); i++) {
    chan = new TransceiverChannel(this, i);
    mControlServiceLoopThreads[i]->start((void * (*)(void*))
                                 ControlServiceLoopAdapter, (void*) chan);
  }
}

void Transceiver2::stop()
{

  if (!mOn)
    return;

  LOG(NOTICE) << "Stopping the transceiver";
  mLowerLoopThread->cancel();
  mLowerLoopThread->join();
  delete mLowerLoopThread;


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


void Transceiver2::reset()
{
  for (size_t i = 0; i < mTxPriorityQueues.size(); i++)
    mTxPriorityQueues[i].clear();
}

  
void Transceiver2::driveControl(size_t chan)
{
  //char response[MAX_PACKET_LENGTH];

  // check control socket
//  char buffer[MAX_PACKET_LENGTH];
//  int msgLen = -1;
//  buffer[0] = '\0';


////  msgLen = mCtrlSockets[chan]->read(buffer);

//  if (msgLen < 1) {
//    return;
//  }

    auto m = pop_c();
    if(!m)
        return;

auto response = (TRX_C*)malloc(sizeof(TRX_C));
response->cmd[0] = '\0';
commandhandler(m->cmd, response->cmd, chan);
free(m);
std::clog << "response is " << response->cmd << std::endl;
push_c(response);
  //mCtrlSockets[chan]->write(response, strlen(response) + 1);
}

void Transceiver2::commandhandler(char* buffer, char* response, int chan)
{
  int MAX_PACKET_LENGTH = TRXC_BUF_SIZE;

  char cmdcheck[4];
  char command[MAX_PACKET_LENGTH];


  sscanf(buffer,"%3s %s",cmdcheck,command);

  if (!chan)
    writeClockInterface();

  if (strcmp(cmdcheck,"CMD")!=0) {
    LOG(WARNING) << "bogus message on control interface";
    return;
  }
  std::clog << "command is " << buffer << std::endl << std::flush;

 if (strcmp(command,"MEASURE")==0) {
         msleep(100);
    int freq;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&freq);
    sprintf(response,"RSP MEASURE 0 %d -80",freq);
  }
  else if (strcmp(command,"ECHO")==0) {
    msleep(100);
    sprintf(response,"RSP ECHO 0");
  }
  else if (strcmp(command,"POWEROFF")==0) {
    // turn off transmitter/demod
    sprintf(response,"RSP POWEROFF 0"); 
  }
  else if (strcmp(command,"POWERON")==0) {
    // turn on transmitter/demod
    if (!mTxFreq || !mRxFreq) 
      sprintf(response,"RSP POWERON 1");
    else {
      sprintf(response,"RSP POWERON 0");
      if (!chan && !mOn) {
        // Prepare for thread start
        mPower = -20;
        mRadioInterface->start();

        // Start radio interface threads.
        mLowerLoopThread->start((void * (*)(void*))
                                LowerLoopAdapter,(void*) this);

        for (size_t i = 0; i < mChans; i++) {
          TransceiverChannel *chan = new TransceiverChannel(this, i);
          mRxServiceLoopThreads[i]->start((void * (*)(void*))
                                  RxUpperLoopAdapter, (void*) chan);

          // chan = new TransceiverChannel(this, i);
          // mTxPriorityQueueServiceLoopThreads[i]->start((void * (*)(void*))
          //                         TxUpperLoopAdapter, (void*) chan);
        }

        writeClockInterface();
        mOn = true;
      }
    }
  }
  else if (strcmp(command,"SETMAXDLY")==0) {
    //set expected maximum time-of-arrival
    int maxDelay;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&maxDelay);
    mMaxExpectedDelay = maxDelay; // 1 GSM symbol is approx. 1 km
    sprintf(response,"RSP SETMAXDLY 0 %d",maxDelay);
  }
  else if (strcmp(command,"SETRXGAIN")==0) {
    //set expected maximum time-of-arrival
    int newGain;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&newGain);
    newGain = mRadioInterface->setRxGain(newGain, chan);
    sprintf(response,"RSP SETRXGAIN 0 %d",newGain);
  }
  else if (strcmp(command,"NOISELEV")==0) {
    if (mOn) {
      float lev = 0;//mStates[chan].mNoiseLev;
      sprintf(response,"RSP NOISELEV 0 %d",
              (int) round(20.0 * log10(rxFullScale / lev)));
    }
    else {
      sprintf(response,"RSP NOISELEV 1  0");
    }
  }
  else if (!strcmp(command, "SETPOWER")) {
    // set output power in dB
    int dbPwr;
    sscanf(buffer, "%3s %s %d", cmdcheck, command, &dbPwr);
    if (!mOn)
      sprintf(response, "RSP SETPOWER 1 %d", dbPwr);
    else {
      mPower = dbPwr;
      mRadioInterface->setPowerAttenuation(mPower, chan);
      sprintf(response, "RSP SETPOWER 0 %d", dbPwr);
    }
  }
  else if (!strcmp(command,"ADJPOWER")) {
    // adjust power in dB steps
    int dbStep;
    sscanf(buffer, "%3s %s %d", cmdcheck, command, &dbStep);
    if (!mOn)
      sprintf(response, "RSP ADJPOWER 1 %d", mPower);
    else {
      mPower += dbStep;
      mRadioInterface->setPowerAttenuation(mPower, chan);
      sprintf(response, "RSP ADJPOWER 0 %d", mPower);
    }
  }
  else if (strcmp(command,"RXTUNE")==0) {
    // tune receiver
    int freqKhz;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&freqKhz);
    mRxFreq = freqKhz * 1e3;
    if (!mRadioInterface->tuneRx(mRxFreq, chan)) {
       LOG(ALERT) << "RX failed to tune";
       sprintf(response,"RSP RXTUNE 1 %d",freqKhz);
    }
    else
       sprintf(response,"RSP RXTUNE 0 %d",freqKhz);
  }
  else if (strcmp(command,"TXTUNE")==0) {
    // tune txmtr
    int freqKhz;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&freqKhz);
    mTxFreq = freqKhz * 1e3;
    if (!mRadioInterface->tuneTx(mTxFreq, chan)) {
       LOG(ALERT) << "TX failed to tune";
       sprintf(response,"RSP TXTUNE 1 %d",freqKhz);
    }
    else
       sprintf(response,"RSP TXTUNE 0 %d",freqKhz);
  }
  else if (!strcmp(command,"SETTSC")) {
    // set TSC
    unsigned TSC;
    sscanf(buffer, "%3s %s %d", cmdcheck, command, &TSC);
    if (mOn)
      sprintf(response, "RSP SETTSC 1 %d", TSC);
    else if (chan && (TSC != mTSC))
      sprintf(response, "RSP SETTSC 1 %d", TSC);
    else {
      mTSC = TSC;
      //generateMidamble(rx_sps, TSC);
      sprintf(response,"RSP SETTSC 0 %d", TSC);
    }
  }
  else if (!strcmp(command,"GETBSIC")) {
    if (mBSIC < 0)
      sprintf(response, "RSP GETBSIC 1");
    else
      sprintf(response, "RSP GETBSIC 0 %d", mBSIC);
  }
  else if (strcmp(command,"SETSLOT")==0) {
    // set TSC 
    int  corrCode;
    int  timeslot;
    sscanf(buffer,"%3s %s %d %d",cmdcheck,command,&timeslot,&corrCode);
    if ((timeslot < 0) || (timeslot > 7)) {
      LOG(WARNING) << "bogus message on control interface";
      sprintf(response,"RSP SETSLOT 1 %d %d",timeslot,corrCode);
      return;
    }     
    mStates[chan].chanType[timeslot] = (ChannelCombination) corrCode;
    setModulus(timeslot, chan);
    sprintf(response,"RSP SETSLOT 0 %d %d",timeslot,corrCode);
  }
  else if (!strcmp(command,"SETRXMASK")) {
    int slot;
    unsigned long long mask;
    sscanf(buffer,"%3s %s %d 0x%llx", cmdcheck, command, &slot, &mask);
    if ((slot < 0) || (slot > 7)) {
      sprintf(response, "RSP SETRXMASK 1");
    } else {
      mRxSlotMask[slot] = mask;
      sprintf(response, "RSP SETRXMASK 0 %d 0x%llx", slot, mask);
    }
  }
  else if (!strcmp(command, "SYNC")) {
    msleep(10);
    mStates[0].mode = TRX_MODE_MS_ACQUIRE;
    sprintf(response,"RSP SYNC 0");
    mMaxExpectedDelay = 10;
    mRadioInterface->setRxGain(30, 0);
    msleep(10);
  }
  else {
    LOG(WARNING) << "bogus command " << command << " on control interface.";
  }

  //mCtrlSockets[chan]->write(response, strlen(response) + 1);
}

bool Transceiver2::driveTxPriorityQueue(size_t chan)
{
    auto burst = pop_d();
    if(!burst)
        return true;

    auto currTime = GSM::Time(burst->fn,burst->ts);
    int RSSI = (int) burst->txlev;

    static BitVector newBurst(gSlotLen);
    BitVector::iterator itr = newBurst.begin();
    auto *bufferItr = burst->symbols;
    while (itr < newBurst.end())
      *itr++ = *bufferItr++;

    addRadioVector(chan, newBurst, RSSI, currTime);
    free(burst);
    return true;



//  char buffer[gSlotLen+50];

//  // check data socket
//  size_t msgLen = mDataSockets[chan]->read(buffer);

//  if (msgLen!=gSlotLen+1+4+1) {
//    LOG(ERR) << "badly formatted packet on GSM->TRX interface";
//    return false;
//  }

//  int timeSlot = (int) buffer[0];
//  uint64_t frameNum = 0;
//  for (int i = 0; i < 4; i++)
//    frameNum = (frameNum << 8) | (0x0ff & buffer[i+1]);

//  LOG(DEBUG) << "rcvd. burst at: " << GSM::Time(frameNum,timeSlot);
  
//  int RSSI = (int) buffer[5];
//  static BitVector newBurst(gSlotLen);
//  BitVector::iterator itr = newBurst.begin();
//  char *bufferItr = buffer+6;
//  while (itr < newBurst.end())
//    *itr++ = *bufferItr++;
  
//  GSM::Time currTime = GSM::Time(frameNum,timeSlot);

//  addRadioVector(chan, newBurst, RSSI, currTime);

  return true;


}

void Transceiver2::driveReceiveRadio()
{
  if (!mRadioInterface->driveReceiveRadio())
    usleep(100000);
}

void Transceiver2::driveReceiveFIFO(size_t chan)
{
    SoftVector *rxBurst = NULL;
    int RSSI;
    int TOA;  // in 1/256 of a symbol
    GSM::Time burstTime;

    rxBurst = pullRadioVector(burstTime, RSSI, TOA, chan);

    if (rxBurst) {
            auto response = (trxd_from_trx*)malloc(sizeof(trxd_from_trx));
#if 0
            if( !gsm_fcch_check_fn(burstTime.FN()) && !gsm_sch_check_fn(burstTime.FN()))
                std::cerr << "burst parameters: "
                           << " time: " << burstTime
                           << " RSSI: " << RSSI
                           << " TOA: "  << TOA
                           << " bits: " << *rxBurst << std::endl;
#endif
            response->ts = burstTime.TN();
            response->fn = burstTime.FN();
            response->rssi = RSSI;
            response->toa = TOA;

            SoftVector::const_iterator burstItr = rxBurst->begin();
            if(gsm_sch_check_fn(burstTime.FN())) {
              for (unsigned int i = 0; i < gSlotLen; i++)
                  ((int8_t*)response->symbols)[i] = round(((*burstItr++)-0.5) * 64.0);
            } else {
              // invert and fix to +-127 sbits
              for (int i = 0; i < 148; i++)
		              ((int8_t*)response->symbols)[i] = *burstItr++ > 0.0f ? -127 : 127;
            }

            delete rxBurst;
            push_d(response);

        }
}

void Transceiver2::driveTxFIFO()
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
            LOG(INFO) << "new latency: " << mTransmitLatency;
            mLatencyUpdateTime = radioClock->get();
          }
        }
        else {
          // if underrun hasn't occurred in the last sec (216 frames) drop
          //    transmit latency by a timeslot
          if (mTransmitLatency > GSM::Time(USB_LATENCY_MIN)) {
              if (radioClock->get() > mLatencyUpdateTime + GSM::Time(216,0)) {
              mTransmitLatency.decTN();
              //LOG(INFO) << "reduced latency: " << mTransmitLatency;
              mLatencyUpdateTime = radioClock->get();
            }
          }
        }
      }
      // time to push burst to transmit FIFO
      pushRadioVector(mTransmitDeadlineClock);

      mTransmitDeadlineClock.incTN();

      if (!mTransmitDeadlineClock.TN() &&
          !(mTransmitDeadlineClock.FN() % CLK_IND_INTERVAL)) {
        writeClockInterface();
      }
    }
  }

  radioClock->wait();
}



void Transceiver2::writeClockInterface()
{
  mLastClockUpdateTime = mTransmitDeadlineClock;
}

void *RxUpperLoopAdapter(TransceiverChannel *chan)
{
  Transceiver2 *trx = chan->trx;
  size_t num = chan->num;

  delete chan;

  // trx->setPriority(0.42);

  while (1) {
    trx->driveReceiveFIFO(num);
    pthread_testcancel();
  }
  return NULL;
}

void *LowerLoopAdapter(Transceiver2 *transceiver)
{
  // transceiver->setPriority(0.45);

  while (1) {
    transceiver->driveReceiveRadio();
    //transceiver->driveTxFIFO();
    pthread_testcancel();
  }
  return NULL;
}

void *ControlServiceLoopAdapter(TransceiverChannel *chan)
{
  Transceiver2 *trx = chan->trx;
  size_t num = chan->num;

  delete chan;

  while (1) {
    trx->driveControl(num);
    pthread_testcancel();
  }
  return NULL;
}

void *TxUpperLoopAdapter(TransceiverChannel *chan)
{
  Transceiver2 *trx = chan->trx;
  size_t num = chan->num;

  delete chan;

  // trx->setPriority(0.40);

  while (1) {
    bool stale = false;
    // Flush the UDP packets until a successful transfer.
    while (!trx->driveTxPriorityQueue(num)) {
      stale = true;
    }
    if (!num && stale) {
      // If a packet was stale, remind the GSM stack of the clock.
      trx->writeClockInterface();
    }
    pthread_testcancel();
  }
  return NULL;
}
