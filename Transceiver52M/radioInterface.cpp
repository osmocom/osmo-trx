/*
* Copyright 2008, 2009 Free Software Foundation, Inc.
*
* This software is distributed under the terms of the GNU Affero Public License.
* See the COPYING file in the main directory for details.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "radioInterface.h"
#include "Resampler.h"
#include <Logger.h>

extern "C" {
#include "convert.h"
}

#define CHUNK		625
#define NUMCHUNKS	4

RadioInterface::RadioInterface(RadioDevice *wRadio,
                               size_t sps, size_t chans, size_t diversity,
                               int wReceiveOffset, GSM::Time wStartTime)
  : mRadio(wRadio), mSPSTx(sps), mSPSRx(1), mChans(chans), mMIMO(diversity),
    sendCursor(0), recvCursor(0), underrun(false), overrun(false),
    receiveOffset(wReceiveOffset), mOn(false)
{
  mClock.set(wStartTime);
}

RadioInterface::~RadioInterface(void)
{
  close();
}

bool RadioInterface::init(int type)
{
  if ((type != RadioDevice::NORMAL) || (mMIMO > 1) || !mChans) {
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
    sendBuffer[i] = new signalVector(CHUNK * mSPSTx);
    recvBuffer[i] = new signalVector(NUMCHUNKS * CHUNK * mSPSRx);

    convertSendBuffer[i] = new short[sendBuffer[i]->size() * 2];
    convertRecvBuffer[i] = new short[recvBuffer[i]->size() * 2];
  }

  sendCursor = 0;
  recvCursor = 0;

  return true;
}

void RadioInterface::close()
{
  for (size_t i = 0; i < sendBuffer.size(); i++)
    delete sendBuffer[i];

  for (size_t i = 0; i < recvBuffer.size(); i++)
    delete recvBuffer[i];

  for (size_t i = 0; i < convertSendBuffer.size(); i++)
    delete convertSendBuffer[i];

  for (size_t i = 0; i < convertRecvBuffer.size(); i++)
    delete convertRecvBuffer[i];

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
				   float *retVector,
				   bool zero)
{
  if (zero) {
    memset(retVector, 0, wVector.size() * 2 * sizeof(float));
    return wVector.size();
  }

  memcpy(retVector, wVector.begin(), wVector.size() * 2 * sizeof(float));

  return wVector.size();
}

int RadioInterface::unRadioifyVector(float *floatVector,
				     signalVector& newVector)
{
  signalVector::iterator itr = newVector.begin();

  if (newVector.size() > recvCursor) {
    LOG(ALERT) << "Insufficient number of samples in receive buffer";
    return -1;
  }

  for (size_t i = 0; i < newVector.size(); i++) {
    *itr++ = Complex<float>(floatVector[2 * i + 0],
			    floatVector[2 * i + 1]);
  }

  return newVector.size();
}

bool RadioInterface::tuneTx(double freq, size_t chan)
{
  return mRadio->setTxFreq(freq, chan);
}

bool RadioInterface::tuneRx(double freq, size_t chan)
{
  return mRadio->setRxFreq(freq, chan);
}

bool RadioInterface::start()
{
  if (mOn)
    return true;

  LOG(INFO) << "Starting radio device";
#ifdef USRP1
  mAlignRadioServiceLoopThread.start((void * (*)(void*))AlignRadioServiceLoopAdapter,
                                     (void*)this);
#endif

  if (!mRadio->start())
    return false;

  recvCursor = 0;
  sendCursor = 0;

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

#ifdef USRP1
void *AlignRadioServiceLoopAdapter(RadioInterface *radioInterface)
{
  while (1) {
    radioInterface->alignRadio();
    pthread_testcancel();
  }
  return NULL;
}

void RadioInterface::alignRadio() {
  sleep(60);
  mRadio->updateAlignment(writeTimestamp+ (TIMESTAMP) 10000);
}
#endif

void RadioInterface::driveTransmitRadio(std::vector<signalVector *> &bursts,
                                        std::vector<bool> &zeros)
{
  if (!mOn)
    return;

  for (size_t i = 0; i < mChans; i++) {
    radioifyVector(*bursts[i],
                   (float *) (sendBuffer[i]->begin() + sendCursor), zeros[i]);
  }

  sendCursor += bursts[0]->size();

  pushBuffer();
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
  int recvSz = recvCursor;
  int readSz = 0;
  const int symbolsPerSlot = gSlotLen + 8;
  int burstSize = (symbolsPerSlot + (tN % 4 == 0)) * mSPSRx;

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
      burst = new radioVector(rcvClock, burstSize, head, mMIMO);

      for (size_t n = 0; n < mMIMO; n++) {
        unRadioifyVector((float *)
                         (recvBuffer[mMIMO * i + n]->begin() + readSz),
                         *burst->getVector(n));
      }

      if (mReceiveFIFO[i].size() < 32)
        mReceiveFIFO[i].write(burst);
      else
        delete burst;
    }

    mClock.incTN();
    rcvClock.incTN();
    readSz += burstSize;
    recvSz -= burstSize;

    tN = rcvClock.TN();

    burstSize = (symbolsPerSlot + (tN % 4 == 0)) * mSPSRx;
  }

  if (readSz > 0) {
    for (size_t i = 0; i < recvBuffer.size(); i++) {
      memmove(recvBuffer[i]->begin(),
              recvBuffer[i]->begin() + readSz,
              (recvCursor - readSz) * 2 * sizeof(float));
    }

    recvCursor -= readSz;
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
  if (mRadio)
    return mRadio->setRxGain(dB, chan);
  else
    return -1;
}

double RadioInterface::getRxGain(size_t chan)
{
  if (mRadio)
    return mRadio->getRxGain(chan);
  else
    return -1;
}

/* Receive a timestamped chunk from the device */
void RadioInterface::pullBuffer()
{
  bool local_underrun;
  int num_recv;
  float *output;

  if (recvCursor > recvBuffer[0]->size() - CHUNK)
    return;

  /* Outer buffer access size is fixed */
  num_recv = mRadio->readSamples(convertRecvBuffer,
                                 CHUNK,
                                 &overrun,
                                 readTimestamp,
                                 &local_underrun);
  if (num_recv != CHUNK) {
          LOG(ALERT) << "Receive error " << num_recv;
          return;
  }

  for (size_t i = 0; i < mChans; i++) {
    output = (float *) (recvBuffer[i]->begin() + recvCursor);
    convert_short_float(output, convertRecvBuffer[i], 2 * num_recv);
  }

  underrun |= local_underrun;

  readTimestamp += num_recv;
  recvCursor += num_recv;
}

/* Send timestamped chunk to the device with arbitrary size */
void RadioInterface::pushBuffer()
{
  int num_sent;

  if (sendCursor < CHUNK)
    return;

  if (sendCursor > sendBuffer[0]->size())
    LOG(ALERT) << "Send buffer overflow";

  for (size_t i = 0; i < mChans; i++) {
    convert_float_short(convertSendBuffer[i],
                        (float *) sendBuffer[i]->begin(),
                        powerScaling[i], 2 * sendCursor);
  }

  /* Send the all samples in the send buffer */ 
  num_sent = mRadio->writeSamples(convertSendBuffer,
                                  sendCursor,
                                  &underrun,
                                  writeTimestamp);
  writeTimestamp += num_sent;
  sendCursor = 0;
}
