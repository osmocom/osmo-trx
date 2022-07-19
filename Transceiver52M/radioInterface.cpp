/*
 * Radio device interface
 *
 * Copyright (C) 2008-2014 Free Software Foundation, Inc.
 * Copyright (C) 2015 Ettus Research LLC
 *
 * SPDX-License-Identifier: AGPL-3.0+
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
#include <Threads.h>

extern "C" {
#include <osmocom/core/utils.h>
#include <osmocom/vty/cpu_sched_vty.h>

#include "convert.h"
}

#define CHUNK		625
#define NUMCHUNKS	4

RadioInterface::RadioInterface(RadioDevice *wDevice, size_t tx_sps,
                               size_t rx_sps, size_t chans,
                               int wReceiveOffset, GSM::Time wStartTime)
  : mSPSTx(tx_sps), mSPSRx(rx_sps), mChans(chans), mReceiveFIFO(mChans), mDevice(wDevice),
    sendBuffer(mChans), recvBuffer(mChans), convertRecvBuffer(mChans),
    convertSendBuffer(mChans), powerScaling(mChans), underrun(false), overrun(false),
    writeTimestamp(0), readTimestamp(0), receiveOffset(wReceiveOffset), mOn(false)
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
  for (std::vector<RadioBuffer*>::iterator it = sendBuffer.begin(); it != sendBuffer.end(); ++it)
          delete *it;
  for (std::vector<RadioBuffer*>::iterator it = recvBuffer.begin(); it != recvBuffer.end(); ++it)
          delete *it;
  for (std::vector<short*>::iterator it = convertSendBuffer.begin(); it != convertSendBuffer.end(); ++it)
          delete[] *it;
  for (std::vector<short*>::iterator it = convertRecvBuffer.begin(); it != convertRecvBuffer.end(); ++it)
          delete[] *it;
  sendBuffer.resize(0);
  recvBuffer.resize(0);
  convertSendBuffer.resize(0);
  convertRecvBuffer.resize(0);
}

double RadioInterface::fullScaleInputValue(void) {
  return mDevice->fullScaleInputValue();
}

double RadioInterface::fullScaleOutputValue(void) {
  return mDevice->fullScaleOutputValue();
}

int RadioInterface::setPowerAttenuation(int atten, size_t chan)
{
  double rfAtten, digAtten;

  if (chan >= mChans) {
    LOG(ALERT) << "Invalid channel requested";
    return -1;
  }

  if (atten < 0.0)
    atten = 0.0;

  rfAtten = mDevice->setPowerAttenuation((double) atten, chan);
  digAtten = (double) atten - rfAtten;

  if (digAtten < 1.0)
    powerScaling[chan] = 1.0;
  else
    powerScaling[chan] = 1.0 / sqrt(pow(10, digAtten / 10.0));

  return atten;
}

int RadioInterface::getNominalTxPower(size_t chan)
{
  if (chan >= mChans) {
    LOG(ALERT) << "Invalid channel requested";
    return -1;
  }

    return mDevice->getNominalTxPower(chan);
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
  return mDevice->setTxFreq(freq, chan);
}

bool RadioInterface::tuneRx(double freq, size_t chan)
{
  return mDevice->setRxFreq(freq, chan);
}

/** synchronization thread loop */
void *AlignRadioServiceLoopAdapter(RadioInterface *radioInterface)
{
  set_selfthread_name("AlignRadio");
  OSMO_ASSERT(osmo_cpu_sched_vty_apply_localthread() == 0);
  while (1) {
    sleep(60);
    radioInterface->alignRadio();
    pthread_testcancel();
  }
  return NULL;
}

void RadioInterface::alignRadio() {
  mDevice->updateAlignment(writeTimestamp+ (TIMESTAMP) 10000);
}

bool RadioInterface::start()
{
  if (mOn)
    return true;

  LOG(INFO) << "Starting radio device";
  if (mDevice->requiresRadioAlign())
        mAlignRadioServiceLoopThread.start(
                                (void * (*)(void*))AlignRadioServiceLoopAdapter,
                                (void*)this);

  if (!mDevice->start())
    return false;

  for (size_t i = 0; i < mChans; i++) {
    sendBuffer[i]->reset();
    recvBuffer[i]->reset();
  }

  writeTimestamp = mDevice->initialWriteTimestamp();
  readTimestamp = mDevice->initialReadTimestamp();

  mDevice->updateAlignment(writeTimestamp-10000);
  mDevice->updateAlignment(writeTimestamp-10000);

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
  if (!mOn || !mDevice->stop())
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

int RadioInterface::driveReceiveRadio()
{
  radioVector *burst = NULL;

  if (!mOn)
    return 0;

  if (pullBuffer() < 0)
    return  -1;

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
  size_t head = GSM::gRACHSynchSequenceTS0.size();

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

  return 1;
}

bool RadioInterface::isUnderrun()
{
  bool retVal;
  /* atomically get previous value of "underrun" and set the var to false */
  retVal = osmo_trx_sync_fetch_and_and(&underrun, false);
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
  return mDevice->setRxGain(dB, chan);
}

double RadioInterface::rssiOffset(size_t chan)
{
	return mDevice->rssiOffset(chan);
}

/* Receive a timestamped chunk from the device */
int RadioInterface::pullBuffer()
{
  bool local_underrun;
  int numRecv;
  size_t segmentLen = recvBuffer[0]->getSegmentLen();

  if (recvBuffer[0]->getFreeSegments() <= 0)
    return -1;

  /* Outer buffer access size is fixed */
  numRecv = mDevice->readSamples(convertRecvBuffer,
                                segmentLen,
                                &overrun,
                                readTimestamp,
                                &local_underrun);

  if ((size_t) numRecv != segmentLen) {
          LOG(ALERT) << "Receive error " << numRecv;
          return -1;
  }

  for (size_t i = 0; i < mChans; i++) {
    convert_short_float(recvBuffer[i]->getWriteSegment(),
			convertRecvBuffer[i],
			segmentLen * 2);
  }

  osmo_trx_sync_or_and_fetch(&underrun, local_underrun);
  readTimestamp += numRecv;
  return 0;
}

/* Send timestamped chunk to the device with arbitrary size */
bool RadioInterface::pushBuffer()
{
  bool local_underrun;
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
  numSent = mDevice->writeSamples(convertSendBuffer,
                                 segmentLen,
                                 &local_underrun,
                                 writeTimestamp);
  osmo_trx_sync_or_and_fetch(&underrun, local_underrun);
  writeTimestamp += numSent;

  return true;
}
