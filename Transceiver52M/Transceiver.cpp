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

#include <stdio.h>
#include "Transceiver.h"
#include <Logger.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

using namespace GSM;

#define USB_LATENCY_INTRVL		10,0

#if USE_UHD
#  define USB_LATENCY_MIN		6,7
#else
#  define USB_LATENCY_MIN		1,1
#endif

/* Number of running values use in noise average */
#define NOISE_CNT			20

TransceiverState::TransceiverState()
  : mRetrans(false), mNoiseLev(0.0), mNoises(NOISE_CNT),
    mode(Transceiver::TRX_MODE_OFF)
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

Transceiver::Transceiver(int wBasePort,
			 const char *TRXAddress,
			 size_t wSPS, size_t wChans,
			 GSM::Time wTransmitLatency,
			 RadioInterface *wRadioInterface)
  : mBasePort(wBasePort), mAddr(TRXAddress),
    mTransmitLatency(wTransmitLatency), mClockSocket(NULL),
    mRadioInterface(wRadioInterface), mSPSTx(wSPS), mSPSRx(1), mChans(wChans),
    mOn(false), mTxFreq(0.0), mRxFreq(0.0), mPower(-10), mMaxExpectedDelay(0)
{
  GSM::Time startTime(random() % gHyperframe,0);

  mRxLowerLoopThread = new Thread(32768);
  mTxLowerLoopThread = new Thread(32768);

  mTransmitDeadlineClock = startTime;
  mLastClockUpdateTime = startTime;
  mLatencyUpdateTime = startTime;
  mRadioInterface->getClock()->set(startTime);

  txFullScale = mRadioInterface->fullScaleInputValue();
  rxFullScale = mRadioInterface->fullScaleOutputValue();
}

Transceiver::~Transceiver()
{
  sigProcLibDestroy();

  delete mClockSocket;

  for (size_t i = 0; i < mChans; i++) {
    mTxPriorityQueues[i].clear();
    delete mCtrlSockets[i];
    delete mDataSockets[i];
  }
}

bool Transceiver::init(bool filler)
{
  int d_srcport, d_dstport, c_srcport, c_dstport;
  signalVector *burst;

  if (!mChans) {
    LOG(ALERT) << "No channels assigned";
    return false;
  }

  if (!sigProcLibSetup(mSPSTx)) {
    LOG(ALERT) << "Failed to initialize signal processing library";
    return false;
  }

  mDataSockets.resize(mChans);
  mCtrlSockets.resize(mChans);

  mControlServiceLoopThreads.resize(mChans);
  mTxPriorityQueueServiceLoopThreads.resize(mChans);
  mRxServiceLoopThreads.resize(mChans);

  mTxPriorityQueues.resize(mChans);
  mReceiveFIFO.resize(mChans);
  mStates.resize(mChans);

  /* Filler table retransmissions - support only on channel 0 */
  if (filler)
    mStates[0].mRetrans = true;

  mClockSocket = new UDPSocket(mBasePort, mAddr.c_str(), mBasePort + 100);

  for (size_t i = 0; i < mChans; i++) {
    c_srcport = mBasePort + 2 * i + 1;
    c_dstport = mBasePort + 2 * i + 101;
    d_srcport = mBasePort + 2 * i + 2;
    d_dstport = mBasePort + 2 * i + 102;

    mCtrlSockets[i] = new UDPSocket(c_srcport, mAddr.c_str(), c_dstport);
    mDataSockets[i] = new UDPSocket(d_srcport, mAddr.c_str(), d_dstport);
  }

  for (size_t i = 0; i < mChans; i++) {
    mControlServiceLoopThreads[i] = new Thread(32768);
    mTxPriorityQueueServiceLoopThreads[i] = new Thread(32768);
    mRxServiceLoopThreads[i] = new Thread(32768);

    for (size_t n = 0; n < 8; n++) {
      burst = modulateBurst(gDummyBurst, 8 + (n % 4 == 0), mSPSTx);
      scaleVector(*burst, txFullScale);
      mStates[i].init(n, burst, filler && !i);
      delete burst;
    }
  }

  return true;
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

  if (mStates[0].mode != TRX_MODE_BTS)
    return;

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
      LOG(NOTICE) << "dumping STALE burst in TRX->USRP interface";
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


Transceiver::CorrType Transceiver::expectedCorrType(GSM::Time currTime,
                                                    size_t chan)
{
  TransceiverState *state = &mStates[chan];
  unsigned burstTN = currTime.TN();
  unsigned burstFN = currTime.FN();

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

/* 
 * Detect RACH synchronization sequence within a burst. No equalization
 * is used or available on the RACH channel.
 */
bool Transceiver::detectRACH(TransceiverState *state,
                             signalVector &burst,
                             complex &amp, float &toa)
{
  float threshold = 6.0;

  return detectRACHBurst(burst, threshold, mSPSRx, &amp, &toa);
}

/* Detect SCH synchronization sequence within a burst */
bool Transceiver::detectSCH(TransceiverState *state,
                            signalVector &burst,
                            complex &amp, float &toa)
{
  int shift;
  float mag, threshold = 7.0;

  if (!detectSCHBurst(burst, threshold, mSPSRx, &amp, &toa))
    return false;

  std::cout << "SCH : Timing offset     " << toa << " symbols" << std::endl;

  mag = fabsf(toa);
  if (mag < 1.0f)
    return true;

  shift = (int) (mag / 2.0f);
  if (!shift)
    shift++;

  mRadioInterface->applyOffset(toa > 0 ? shift : -shift);
  return false;
}

/*
 * Detect normal burst training sequence midamble. Update equalization
 * state information and channel estimate if necessary. Equalization
 * is currently disabled.
 */
bool Transceiver::detectTSC(TransceiverState *state, signalVector &burst,
                            complex &amp, float &toa, GSM::Time &time)
{
  int tn = time.TN();
  float chanOffset, threshold = 5.0;
  bool noise, needDFE = false, estimateChan = false;
  double elapsed = time - state->chanEstimateTime[tn];
  signalVector *chanResp;

  /* Check equalization update state */
  if (needDFE && ((elapsed > 50) || (!state->chanResponse[tn]))) {
    delete state->DFEForward[tn];
    delete state->DFEFeedback[tn];
    state->DFEForward[tn] = NULL;
    state->DFEFeedback[tn] = NULL;

    estimateChan = true;
  }

  /* Detect normal burst midambles */
  if (!analyzeTrafficBurst(burst, mTSC, threshold, mSPSRx, &amp,
                           &toa, mMaxExpectedDelay, estimateChan,
                           &chanResp, &chanOffset)) {
    return false;
  }

  noise = state->mNoiseLev;
  state->SNRestimate[tn] = amp.norm2() / (noise * noise + 1.0);

  /* Set equalizer if unabled */
  if (needDFE && estimateChan) {
     state->chanResponse[tn] = chanResp;
     state->chanRespOffset[tn] = chanOffset;
     state->chanRespAmplitude[tn] = amp;

     scaleVector(*chanResp, complex(1.0, 0.0) / amp);

     designDFE(*chanResp, state->SNRestimate[tn],
               7, &state->DFEForward[tn], &state->DFEFeedback[tn]);

     state->chanEstimateTime[tn] = time;
  }

  return true;;
}

/*
 * Demodulate GMSK burst using equalization if requested. Otherwise
 * demodulate by direct rotation and soft slicing.
 */
SoftVector *Transceiver::demodulate(TransceiverState *state,
                                    signalVector &burst, complex amp,
                                    float toa, size_t tn, bool equalize)
{
  if (equalize) {
    scaleVector(burst, complex(1.0, 0.0) / amp);
    return equalizeBurst(burst,
                         toa - state->chanRespOffset[tn],
                         mSPSRx,
                         *state->DFEForward[tn],
                         *state->DFEFeedback[tn]);
  }

  return demodulateBurst(burst, mSPSRx, amp, toa);
}

/*
 * Pull bursts from the FIFO and handle according to the slot
 * and burst correlation type. Equalzation is currently disabled. 
 */
SoftVector *Transceiver::pullRadioVector(GSM::Time &wTime, int &RSSI,
                                         int &timingOffset, size_t chan)
{
  bool success, equalize = false;
  complex amp;
  float toa, pow, max = -1.0, avg = 0.0;
  int max_i = -1;
  signalVector *burst;
  SoftVector *bits = NULL;
  TransceiverState *state = &mStates[chan];

  /* Blocking FIFO read */
  radioVector *radio_burst = mReceiveFIFO[chan]->read();
  if (!radio_burst)
    return NULL;

  /* Set time and determine correlation type */
  GSM::Time time = radio_burst->getTime();
  CorrType type = expectedCorrType(time, chan);

  switch (state->mode) {
  case TRX_MODE_MS_ACQUIRE:
    type = SCH;
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
    energyDetect(*radio_burst->getVector(i), 20 * mSPSRx, 0.0, &pow);
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
  state->mNoiseLev = state->mNoises.avg();

  /* Detect normal or RACH bursts */
  if (type == TSC)
    success = detectTSC(state, *burst, amp, toa, time);
  else if (type == RACH)
    success = detectRACH(state, *burst, amp, toa);
  else if (type == SCH)
    success = detectSCH(state, *burst, amp, toa);
  else
    success = false;

  if (!success) {
    state->mNoises.insert(avg);
    goto release;
  }

  /* Demodulate and set output info */
  if (equalize && (type != TSC))
    equalize = false;

  if (avg - state->mNoiseLev > 0.0)
    bits = demodulate(state, *burst, amp, toa, time.TN(), equalize);

  wTime = time;
  RSSI = (int) floor(20.0 * log10(rxFullScale / avg));
  timingOffset = (int) round(toa * 256.0 / mSPSRx);

  delete radio_burst;

  return bits;

release:
  delete bits;
  return NULL;
}

void Transceiver::start()
{
  TransceiverChannel *chan;

  for (size_t i = 0; i < mControlServiceLoopThreads.size(); i++) {
    chan = new TransceiverChannel(this, i);
    mControlServiceLoopThreads[i]->start((void * (*)(void*))
                                 ControlServiceLoopAdapter, (void*) chan);
  }
}

void Transceiver::reset()
{
  for (size_t i = 0; i < mTxPriorityQueues.size(); i++)
    mTxPriorityQueues[i].clear();
}

  
void Transceiver::driveControl(size_t chan)
{
  int MAX_PACKET_LENGTH = 100;

  // check control socket
  char buffer[MAX_PACKET_LENGTH];
  int msgLen = -1;
  buffer[0] = '\0';

  msgLen = mCtrlSockets[chan]->read(buffer);

  if (msgLen < 1) {
    return;
  }

  char cmdcheck[4];
  char command[MAX_PACKET_LENGTH];
  char response[MAX_PACKET_LENGTH];

  sscanf(buffer,"%3s %s",cmdcheck,command);

  if (!chan)
    writeClockInterface();

  if (strcmp(cmdcheck,"CMD")!=0) {
    LOG(WARNING) << "bogus message on control interface";
    return;
  }
  LOG(INFO) << "command is " << buffer;

  if (strcmp(command,"POWEROFF")==0) {
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
        mTxLowerLoopThread->start((void * (*)(void*))
                                  TxLowerLoopAdapter,(void*) this);
        mRxLowerLoopThread->start((void * (*)(void*))
                                  RxLowerLoopAdapter,(void*) this);

        for (size_t i = 0; i < mChans; i++) {
          TransceiverChannel *chan = new TransceiverChannel(this, i);
          mRxServiceLoopThreads[i]->start((void * (*)(void*))
                                  RxUpperLoopAdapter, (void*) chan);

          chan = new TransceiverChannel(this, i);
          mTxPriorityQueueServiceLoopThreads[i]->start((void * (*)(void*))
                                  TxUpperLoopAdapter, (void*) chan);
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
      float lev = mStates[chan].mNoiseLev;
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
      generateMidamble(mSPSRx, TSC);
      sprintf(response,"RSP SETTSC 0 %d", TSC);
    }
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
  else if (!strcmp(command, "SYNC")) {
    mStates[0].mode = TRX_MODE_MS_ACQUIRE;
    sprintf(response,"RSP SYNC 0");
  }
  else {
    LOG(WARNING) << "bogus command " << command << " on control interface.";
  }

  mCtrlSockets[chan]->write(response, strlen(response) + 1);
}

bool Transceiver::driveTxPriorityQueue(size_t chan)
{
  char buffer[gSlotLen+50];

  // check data socket
  size_t msgLen = mDataSockets[chan]->read(buffer);

  if (msgLen!=gSlotLen+1+4+1) {
    LOG(ERR) << "badly formatted packet on GSM->TRX interface";
    return false;
  }

  int timeSlot = (int) buffer[0];
  uint64_t frameNum = 0;
  for (int i = 0; i < 4; i++)
    frameNum = (frameNum << 8) | (0x0ff & buffer[i+1]);

  // periodically update GSM core clock
  LOG(DEBUG) << "mTransmitDeadlineClock " << mTransmitDeadlineClock
		<< " mLastClockUpdateTime " << mLastClockUpdateTime;

  if (!chan) {
    if (mTransmitDeadlineClock > mLastClockUpdateTime + GSM::Time(216,0))
      writeClockInterface();
  }

  LOG(DEBUG) << "rcvd. burst at: " << GSM::Time(frameNum,timeSlot);
  
  int RSSI = (int) buffer[5];
  static BitVector newBurst(gSlotLen);
  BitVector::iterator itr = newBurst.begin();
  char *bufferItr = buffer+6;
  while (itr < newBurst.end()) 
    *itr++ = *bufferItr++;
  
  GSM::Time currTime = GSM::Time(frameNum,timeSlot);

  addRadioVector(chan, newBurst, RSSI, currTime);

  return true;


}

void Transceiver::driveReceiveRadio()
{
  if (!mRadioInterface->driveReceiveRadio())
    usleep(100000);
}

void Transceiver::driveReceiveFIFO(size_t chan)
{
  SoftVector *rxBurst = NULL;
  int RSSI;
  int TOA;  // in 1/256 of a symbol
  GSM::Time burstTime;

  rxBurst = pullRadioVector(burstTime, RSSI, TOA, chan);

  if (rxBurst) { 

    LOG(DEBUG) << "burst parameters: "
	  << " time: " << burstTime
	  << " RSSI: " << RSSI
	  << " TOA: "  << TOA
	  << " bits: " << *rxBurst;
    
    char burstString[gSlotLen+10];
    burstString[0] = burstTime.TN();
    for (int i = 0; i < 4; i++)
      burstString[1+i] = (burstTime.FN() >> ((3-i)*8)) & 0x0ff;
    burstString[5] = RSSI;
    burstString[6] = (TOA >> 8) & 0x0ff;
    burstString[7] = TOA & 0x0ff;
    SoftVector::iterator burstItr = rxBurst->begin();

    for (unsigned int i = 0; i < gSlotLen; i++) {
      burstString[8+i] =(char) round((*burstItr++)*255.0);
    }
    burstString[gSlotLen+9] = '\0';
    delete rxBurst;

    mDataSockets[chan]->write(burstString,gSlotLen+10);
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
              LOG(INFO) << "reduced latency: " << mTransmitLatency;
              mLatencyUpdateTime = radioClock->get();
            }
          }
        }
      }
      // time to push burst to transmit FIFO
      if (mStates[0].mode == TRX_MODE_BTS)
        pushRadioVector(mTransmitDeadlineClock);

      mTransmitDeadlineClock.incTN();
    }
  }

  radioClock->wait();
}



void Transceiver::writeClockInterface()
{
  char command[50];
  // FIXME -- This should be adaptive.
  sprintf(command,"IND CLOCK %llu",(unsigned long long) (mTransmitDeadlineClock.FN()+2));

  LOG(INFO) << "ClockInterface: sending " << command;

  mClockSocket->write(command, strlen(command) + 1);

  mLastClockUpdateTime = mTransmitDeadlineClock;

}

void *RxUpperLoopAdapter(TransceiverChannel *chan)
{
  Transceiver *trx = chan->trx;
  size_t num = chan->num;

  delete chan;

  trx->setPriority(0.42);

  while (1) {
    trx->driveReceiveFIFO(num);
    pthread_testcancel();
  }
  return NULL;
}

void *RxLowerLoopAdapter(Transceiver *transceiver)
{
  transceiver->setPriority(0.45);

  while (1) {
    transceiver->driveReceiveRadio();
    pthread_testcancel();
  }
  return NULL;
}

void *TxLowerLoopAdapter(Transceiver *transceiver)
{
  transceiver->setPriority(0.44);

  while (1) {
    transceiver->driveTxFIFO();
    pthread_testcancel();
  }
  return NULL;
}

void *ControlServiceLoopAdapter(TransceiverChannel *chan)
{
  Transceiver *trx = chan->trx;
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
  Transceiver *trx = chan->trx;
  size_t num = chan->num;

  delete chan;

  trx->setPriority(0.40);

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
