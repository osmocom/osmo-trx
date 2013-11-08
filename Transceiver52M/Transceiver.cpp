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

void TransceiverState::init(size_t slot, signalVector *burst)
{
  for (int i = 0; i < 102; i++)
    fillerTable[i][slot] = new signalVector(*burst);
}

Transceiver::Transceiver(int wBasePort,
			 const char *TRXAddress,
			 size_t wSPS, size_t wChans,
			 GSM::Time wTransmitLatency,
			 RadioInterface *wRadioInterface)
  : mBasePort(wBasePort), mAddr(TRXAddress),
    mTransmitLatency(wTransmitLatency), mClockSocket(NULL), mRadioInterface(wRadioInterface),
    mNoiseLev(0.0), mNoises(NOISE_CNT), mSPSTx(wSPS), mSPSRx(1), mChans(wChans),
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

bool Transceiver::init()
{
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

  mClockSocket = new UDPSocket(mBasePort, mAddr.c_str(), mBasePort + 100);

  for (size_t i = 0; i < mChans; i++) {
    mDataSockets[i] = new UDPSocket(mBasePort + 2 * i + 2, mAddr.c_str(),
                                    mBasePort + 2 * i + 102);
    mCtrlSockets[i] = new UDPSocket(mBasePort + 2 * i + 1, mAddr.c_str(),
                                    mBasePort + 2 * i + 101);
  }

  for (size_t i = 0; i < mChans; i++) {
    mControlServiceLoopThreads[i] = new Thread(32768);
    mTxPriorityQueueServiceLoopThreads[i] = new Thread(32768);
    mRxServiceLoopThreads[i] = new Thread(32768);

    for (size_t n = 0; n < 8; n++) {
      burst = modulateBurst(gDummyBurst, 8 + (n % 4 == 0), mSPSTx);
      scaleVector(*burst, txFullScale);
      mStates[i].init(n, burst);
      delete burst;
    }
  }

  return true;
}

void Transceiver::addRadioVector(size_t chan, BitVector &burst,
                                 int RSSI, GSM::Time &wTime)
{
  if (chan >= mTxPriorityQueues.size()) {
    LOG(ALERT) << "Invalid channel " << chan;
    return;
  }

  // modulate and stick into queue 
  signalVector* modBurst = modulateBurst(burst,
					 8 + (wTime.TN() % 4 == 0),
					 mSPSTx);
  scaleVector(*modBurst,txFullScale * pow(10,-RSSI/10));
  radioVector *newVec = new radioVector(*modBurst,wTime);
  mTxPriorityQueues[chan].write(newVec);

  delete modBurst;
}

void Transceiver::pushRadioVector(GSM::Time &nowTime)
{
  int TN, modFN;
  radioVector *burst;
  TransceiverState *state;
  std::vector<signalVector *> bursts(mChans);
  std::vector<bool> zeros(mChans);

  for (size_t i = 0; i < mChans; i ++) {
    state = &mStates[i];

    while ((burst = mTxPriorityQueues[i].getStaleBurst(nowTime))) {
      LOG(NOTICE) << "dumping STALE burst in TRX->USRP interface";

      TN = burst->getTime().TN();
      modFN = burst->getTime().FN() % state->fillerModulus[TN];

      delete state->fillerTable[modFN][TN];
      state->fillerTable[modFN][TN] = burst;
    }

    TN = nowTime.TN();
    modFN = nowTime.FN() % state->fillerModulus[TN];

    bursts[i] = state->fillerTable[modFN][TN];
    zeros[i] = state->chanType[TN] == NONE;

    if ((burst = mTxPriorityQueues[i].getCurrentBurst(nowTime))) {
      delete state->fillerTable[modFN][TN];
      state->fillerTable[modFN][TN] = burst;
      bursts[i] = burst;
    }
  }

  mRadioInterface->driveTransmitRadio(bursts, zeros);

  return;
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

SoftVector *Transceiver::pullRadioVector(GSM::Time &wTime, int &RSSI,
                                         int &timingOffset, size_t chan)
{
  bool needDFE = false;
  bool success = false;
  complex amp = 0.0;
  float TOA = 0.0, avg = 0.0;
  TransceiverState *state = &mStates[chan];

  radioVector *rxBurst = (radioVector *) mReceiveFIFO[chan]->read();

  if (!rxBurst) return NULL;

  int timeslot = rxBurst->getTime().TN();

  CorrType corrType = expectedCorrType(rxBurst->getTime(), chan);

  if ((corrType==OFF) || (corrType==IDLE)) {
    delete rxBurst;
    return NULL;
  }

  signalVector *vectorBurst = rxBurst;

  energyDetect(*vectorBurst, 20 * mSPSRx, 0.0, &avg);

  // Update noise level
  mNoiseLev = mNoises.avg();
  avg = sqrt(avg);

  // run the proper correlator
  if (corrType==TSC) {
    LOG(DEBUG) << "looking for TSC at time: " << rxBurst->getTime();
    signalVector *channelResp;
    double framesElapsed = rxBurst->getTime() - state->chanEstimateTime[timeslot];
    bool estimateChannel = false;
    if ((framesElapsed > 50) || (state->chanResponse[timeslot]==NULL)) {
	if (state->chanResponse[timeslot])
          delete state->chanResponse[timeslot];
        if (state->DFEForward[timeslot])
          delete state->DFEForward[timeslot];
        if (state->DFEFeedback[timeslot])
          delete state->DFEFeedback[timeslot];

        state->chanResponse[timeslot] = NULL;
        state->DFEForward[timeslot] = NULL;
        state->DFEFeedback[timeslot] = NULL;
	estimateChannel = true;
    }
    if (!needDFE) estimateChannel = false;
    float chanOffset;
    success = analyzeTrafficBurst(*vectorBurst,
                                  mTSC,
                                  5.0,
                                  mSPSRx,
                                  &amp,
                                  &TOA,
                                  mMaxExpectedDelay,
                                  estimateChannel,
                                  &channelResp,
                                  &chanOffset);
    if (success) {
      state->SNRestimate[timeslot] = amp.norm2() / (mNoiseLev * mNoiseLev + 1.0);

      if (estimateChannel) {
         state->chanResponse[timeslot] = channelResp;
         state->chanRespOffset[timeslot] = chanOffset;
         state->chanRespAmplitude[timeslot] = amp;
	 scaleVector(*channelResp, complex(1.0, 0.0) / amp);
         designDFE(*channelResp, state->SNRestimate[timeslot],
                   7, &state->DFEForward[timeslot],
                   &state->DFEFeedback[timeslot]);

         state->chanEstimateTime[timeslot] = rxBurst->getTime();
      }
    }
    else {
      state->chanResponse[timeslot] = NULL;
      mNoises.insert(avg);
    }
  }
  else {
    // RACH burst
    if ((success = detectRACHBurst(*vectorBurst, 6.0, mSPSRx, &amp, &TOA)))
      state->chanResponse[timeslot] = NULL;
    else
      mNoises.insert(avg);
  }

  // demodulate burst
  SoftVector *burst = NULL;
  if ((rxBurst) && (success)) {
    if ((corrType==RACH) || (!needDFE)) {
      burst = demodulateBurst(*vectorBurst, mSPSRx, amp, TOA);
    } else {
      scaleVector(*vectorBurst, complex(1.0, 0.0) / amp);
      burst = equalizeBurst(*vectorBurst,
			    TOA - state->chanRespOffset[timeslot],
			    mSPSRx,
			    *state->DFEForward[timeslot],
			    *state->DFEFeedback[timeslot]);
    }
    wTime = rxBurst->getTime();
    RSSI = (int) floor(20.0*log10(rxFullScale/avg));
    LOG(DEBUG) << "RSSI: " << RSSI;
    timingOffset = (int) round(TOA * 256.0 / mSPSRx);
  }

  delete rxBurst;

  return burst;
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
      sprintf(response,"RSP NOISELEV 0 %d",
              (int) round(20.0*log10(rxFullScale/mNoiseLev)));
    }
    else {
      sprintf(response,"RSP NOISELEV 1  0");
    }
  }   
  else if (strcmp(command,"SETPOWER")==0) {
    // set output power in dB
    int dbPwr;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&dbPwr);
    if (!mOn) 
      sprintf(response,"RSP SETPOWER 1 %d",dbPwr);
    else {
      mPower = dbPwr;
      mRadioInterface->setPowerAttenuation(dbPwr, chan);
      sprintf(response,"RSP SETPOWER 0 %d",dbPwr);
    }
  }
  else if (strcmp(command,"ADJPOWER")==0) {
    // adjust power in dB steps
    int dbStep;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&dbStep);
    if (!mOn) 
      sprintf(response,"RSP ADJPOWER 1 %d",mPower);
    else {
      mPower += dbStep;
      sprintf(response,"RSP ADJPOWER 0 %d",mPower);
    }
  }
#define FREQOFFSET 0//11.2e3
  else if (strcmp(command,"RXTUNE")==0) {
    // tune receiver
    int freqKhz;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&freqKhz);
    mRxFreq = freqKhz*1.0e3+FREQOFFSET;
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
    //freqKhz = 890e3;
    mTxFreq = freqKhz*1.0e3+FREQOFFSET;
    if (!mRadioInterface->tuneTx(mTxFreq, chan)) {
       LOG(ALERT) << "TX failed to tune";
       sprintf(response,"RSP TXTUNE 1 %d",freqKhz);
    }
    else
       sprintf(response,"RSP TXTUNE 0 %d",freqKhz);
  }
  else if (strcmp(command,"SETTSC")==0) {
    // set TSC
    int TSC;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&TSC);
    if (mOn)
      sprintf(response,"RSP SETTSC 1 %d",TSC);
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
  
  /*
  if (GSM::Time(frameNum,timeSlot) >  mTransmitDeadlineClock + GSM::Time(51,0)) {
    // stale burst
    //LOG(DEBUG) << "FAST! "<< GSM::Time(frameNum,timeSlot);
    //writeClockInterface();
    }*/

/*
  DAB -- Just let these go through the demod.
  if (GSM::Time(frameNum,timeSlot) < mTransmitDeadlineClock) {
    // stale burst from GSM core
    LOG(NOTICE) << "STALE packet on GSM->TRX interface at time "<< GSM::Time(frameNum,timeSlot);
    return false;
  }
*/
  
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
