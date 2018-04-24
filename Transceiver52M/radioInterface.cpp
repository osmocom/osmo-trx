/*
 * Radio device interface
 *
 * Copyright (C) 2008-2014 Free Software Foundation, Inc.
 * Copyright (C) 2015 Ettus Research LLC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 */

#include "radioInterface.h"
#include "Resampler.h"
#include <Logger.h>

extern "C" {
#include "convert.h"
}

#define CHUNK		625
#define NUMCHUNKS	4

RadioInterface::RadioInterface(RadioDevice *wRadio, size_t tx_sps,
                               size_t rx_sps, size_t chans,
                               int wReceiveOffset, GSM::Time wStartTime)
  : mRadio(wRadio), mSPSTx(tx_sps), mSPSRx(rx_sps), mChans(chans),
    underrun(false), overrun(false), receiveOffset(wReceiveOffset), mOn(false)
{
  mClock.set(wStartTime);
}

RadioInterface::~RadioInterface(void)
{
  close();
}

bool RadioInterface::init(int type)
{
  if ((type != RadioDevice::NORMAL) || !mChans) {
    LOG(ALERT) << "Invalid configuration";
    return false;
  }

  close();

  sendBuffer.resize(mChans);
  recvBuffer.resize(mChans);
  convertSendBuffer.resize(mChans);
  convertRecvBuffer.resize(mChans);
  mReceiveFIFO.resize(mChans);
  powerScaling.resize(mChans);

  for (size_t i = 0; i < mChans; i++) {
    sendBuffer[i] = new RadioBuffer(NUMCHUNKS, CHUNK * mSPSTx, 0, true);
    recvBuffer[i] = new RadioBuffer(NUMCHUNKS, CHUNK * mSPSRx, 0, false);

    convertSendBuffer[i] = new short[CHUNK * mSPSTx * 2];
    convertRecvBuffer[i] = new short[CHUNK * mSPSRx * 2];

    powerScaling[i] = 1.0;
  }

  return true;
}

void RadioInterface::close()
{
  sendBuffer.resize(0);
  recvBuffer.resize(0);
  convertSendBuffer.resize(0);
  convertRecvBuffer.resize(0);
}

double RadioInterface::fullScaleInputValue(void) {
  return mRadio->fullScaleInputValue();
}

double RadioInterface::fullScaleOutputValue(void) {
  return mRadio->fullScaleOutputValue();
}

int RadioInterface::setPowerAttenuation(int atten, size_t chan)
{
  double rfGain, digAtten;

  if (chan >= mChans) {
    LOG(ALERT) << "Invalid channel requested";
    return -1;
  }

  if (atten < 0.0)
    atten = 0.0;

  rfGain = mRadio->setTxGain(mRadio->maxTxGain() - (double) atten, chan);
  digAtten = (double) atten - mRadio->maxTxGain() + rfGain;

  if (digAtten < 1.0)
    powerScaling[chan] = 1.0;
  else
    powerScaling[chan] = 1.0 / sqrt(pow(10, digAtten / 10.0));

  return atten;
}

int RadioInterface::radioifyVector(signalVector &wVector,
                                   size_t chan, bool zero)
{
  if (zero)
    sendBuffer[chan]->zero(wVector.size());
  else
    sendBuffer[chan]->write((float *) wVector.begin(), wVector.size());

  return wVector.size();
}

int RadioInterface::unRadioifyVector(signalVector *newVector, size_t chan)
{
  if (newVector->size() > recvBuffer[chan]->getAvailSamples()) {
    LOG(ALERT) << "Insufficient number of samples in receive buffer";
    return -1;
  }

  recvBuffer[chan]->read((float *) newVector->begin(), newVector->size());

  return newVector->size();
}

bool RadioInterface::tuneTx(double freq, size_t chan)
{
  return mRadio->setTxFreq(freq, chan);
}

bool RadioInterface::tuneRx(double freq, size_t chan)
{
  return mRadio->setRxFreq(freq, chan);
}

/** synchronization thread loop */
void *AlignRadioServiceLoopAdapter(RadioInterface *radioInterface)
{
  while (1) {
    sleep(60);
    radioInterface->alignRadio();
    pthread_testcancel();
  }
  return NULL;
}

void RadioInterface::alignRadio() {
  mRadio->updateAlignment(writeTimestamp+ (TIMESTAMP) 10000);
}

bool RadioInterface::start()
{
  if (mOn)
    return true;

  LOG(INFO) << "Starting radio device";
  if (mRadio->requiresRadioAlign())
        mAlignRadioServiceLoopThread.start(
                                (void * (*)(void*))AlignRadioServiceLoopAdapter,
                                (void*)this);

  if (!mRadio->start())
    return false;

  for (size_t i = 0; i < mChans; i++) {
    sendBuffer[i]->reset();
    recvBuffer[i]->reset();
  }

  writeTimestamp = mRadio->initialWriteTimestamp();
  readTimestamp = mRadio->initialReadTimestamp();

  mRadio->updateAlignment(writeTimestamp-10000);
  mRadio->updateAlignment(writeTimestamp-10000);

  mOn = true;
  LOG(INFO) << "Radio started";
  return true;
}

/*
 * Stop the radio device
 *
 * This is a pass-through call to the device interface. Because the underlying
 * stop command issuance generally doesn't return confirmation on device status,
 * this call will only return false if the device is already stopped.
 */
bool RadioInterface::stop()
{
  if (!mOn || !mRadio->stop())
    return false;

  mOn = false;
  return true;
}

void RadioInterface::driveTransmitRadio(std::vector<signalVector *> &bursts,
                                        std::vector<bool> &zeros)
{
  if (!mOn)
    return;

  for (size_t i = 0; i < mChans; i++)
    radioifyVector(*bursts[i], i, zeros[i]);

  while (pushBuffer());
}

bool RadioInterface::driveReceiveRadio()
{
  radioVector *burst = NULL;

  if (!mOn)
    return false;

  pullBuffer();

  GSM::Time rcvClock = mClock.get();
  rcvClock.decTN(receiveOffset);
  unsigned tN = rcvClock.TN();
  int recvSz = recvBuffer[0]->getAvailSamples();
  const int symbolsPerSlot = gSlotLen + 8;
  int burstSize;

  if (mSPSRx == 4)
    burstSize = 625;
  else
    burstSize = symbolsPerSlot + (tN % 4 == 0);

  /* 
   * Pre-allocate head room for the largest correlation size
   * so we can later avoid a re-allocation and copy
   * */
  size_t head = GSM::gRACHSynchSequence.size();

  /*
   * Form receive bursts and pass up to transceiver. Use repeating
   * pattern of 157-156-156-156 symbols per timeslot
   */
  while (recvSz > burstSize) {
    for (size_t i = 0; i < mChans; i++) {
      burst = new radioVector(rcvClock, burstSize, head);
      unRadioifyVector(burst->getVector(), i);

      if (mReceiveFIFO[i].size() < 32)
        mReceiveFIFO[i].write(burst);
      else
        delete burst;
    }

    mClock.incTN();
    rcvClock.incTN();
    recvSz -= burstSize;

    tN = rcvClock.TN();

    if (mSPSRx != 4)
      burstSize = (symbolsPerSlot + (tN % 4 == 0)) * mSPSRx;
  }

  return true;
}

bool RadioInterface::isUnderrun()
{
  bool retVal = underrun;
  underrun = false;

  return retVal;
}

VectorFIFO* RadioInterface::receiveFIFO(size_t chan)
{
  if (chan >= mReceiveFIFO.size())
    return NULL;

  return &mReceiveFIFO[chan];
}

double RadioInterface::setRxGain(double dB, size_t chan)
{
  return mRadio->setRxGain(dB, chan);
}

double RadioInterface::getRxGain(size_t chan)
{
  return mRadio->getRxGain(chan);
}

/* Receive a timestamped chunk from the device */
void RadioInterface::pullBuffer()
{
  bool local_underrun;
  size_t numRecv, segmentLen = recvBuffer[0]->getSegmentLen();

  if (recvBuffer[0]->getFreeSegments() <= 0)
    return;

  /* Outer buffer access size is fixed */
  numRecv = mRadio->readSamples(convertRecvBuffer,
                                segmentLen,
                                &overrun,
                                readTimestamp,
                                &local_underrun);

  if (numRecv != segmentLen) {
          LOG(ALERT) << "Receive error " << numRecv;
          return;
  }

  for (size_t i = 0; i < mChans; i++) {
    convert_short_float(recvBuffer[i]->getWriteSegment(),
			convertRecvBuffer[i],
			segmentLen * 2);
  }

  underrun |= local_underrun;
  readTimestamp += numRecv;
}

/* Send timestamped chunk to the device with arbitrary size */
bool RadioInterface::pushBuffer()
{
  size_t numSent, segmentLen = sendBuffer[0]->getSegmentLen();

  if (sendBuffer[0]->getAvailSegments() < 1)
    return false;

  for (size_t i = 0; i < mChans; i++) {
    convert_float_short(convertSendBuffer[i],
                        (float *) sendBuffer[i]->getReadSegment(),
                        powerScaling[i],
                        segmentLen * 2);
  }

  /* Send the all samples in the send buffer */
  numSent = mRadio->writeSamples(convertSendBuffer,
                                 segmentLen,
                                 &underrun,
                                 writeTimestamp);
  writeTimestamp += numSent;

  return true;
}
