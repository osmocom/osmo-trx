/*
* Copyright 2008, 2009 Free Software Foundation, Inc.
*
* SPDX-License-Identifier: AGPL-3.0+
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


/*
	Compilation Flags

	SWLOOPBACK	compile for software loopback testing
*/


#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "Logger.h"
#include "Threads.h"
#include "USRPDevice.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

using namespace std;

enum dboardConfigType {
  TXA_RXB,
  TXB_RXA,
  TXA_RXA,
  TXB_RXB
};

#ifdef SINGLEDB
const dboardConfigType dboardConfig = TXA_RXA;
#else
const dboardConfigType dboardConfig = TXA_RXB;
#endif

const double USRPDevice::masterClockRate = 52.0e6;

USRPDevice::USRPDevice(InterfaceType iface, const struct trx_cfg *cfg) : RadioDevice(iface, cfg)
{
  LOGC(DDEV, INFO) << "creating USRP device...";

  decimRate = (unsigned int) round(masterClockRate/((GSMRATE) * (double) tx_sps));
  actualSampleRate = masterClockRate/decimRate;
  rxGain = 0;
  txGain = 0;

  /*
   * Undetermined delay b/w ping response timestamp and true
   * receive timestamp. Values are empirically measured. With
   * split sample rate Tx/Rx - 4/1 sps we need to need to
   * compensate for advance rather than delay.
   */
  if (tx_sps == 1)
    pingOffset = 272;
  else if (tx_sps == 4)
    pingOffset = 269 - 7500;
  else
    pingOffset = 0;

#ifdef SWLOOPBACK
  samplePeriod = 1.0e6/actualSampleRate;
  loopbackBufferSize = 0;
  gettimeofday(&lastReadTime,NULL);
  firstRead = false;
#endif
}

int USRPDevice::open()
{
  writeLock.unlock();

  LOGC(DDEV, INFO) << "opening USRP device..";
#ifndef SWLOOPBACK
  string rbf = "std_inband.rbf";
  //string rbf = "inband_1rxhb_1tx.rbf";
  m_uRx.reset();
  try {
    m_uRx = usrp_standard_rx_sptr(usrp_standard_rx::make(
                                        0, decimRate * tx_sps, 1, -1,
                                        usrp_standard_rx::FPGA_MODE_NORMAL,
                                        1024, 16 * 8, rbf));
    m_uRx->set_fpga_master_clock_freq(masterClockRate);
  }

  catch(...) {
    LOGC(DDEV, ALERT) << "make failed on Rx";
    m_uRx.reset();
    return -1;
  }

  if (m_uRx->fpga_master_clock_freq() != masterClockRate)
  {
    LOGC(DDEV, ALERT) << "WRONG FPGA clock freq = " << m_uRx->fpga_master_clock_freq()
               << ", desired clock freq = " << masterClockRate;
    m_uRx.reset();
    return -1;
  }

  try {
    m_uTx = usrp_standard_tx_sptr(usrp_standard_tx::make(
                                        0, decimRate * 2, 1, -1,
                                        1024, 16 * 8, rbf));
    m_uTx->set_fpga_master_clock_freq(masterClockRate);
  }

  catch(...) {
    LOGC(DDEV, ALERT) << "make failed on Tx";
    m_uTx.reset();
    return -1;
  }

  if (m_uTx->fpga_master_clock_freq() != masterClockRate)
  {
    LOGC(DDEV, ALERT) << "WRONG FPGA clock freq = " << m_uTx->fpga_master_clock_freq()
               << ", desired clock freq = " << masterClockRate;
    m_uTx.reset();
    return -1;
  }

  m_uRx->stop();
  m_uTx->stop();

#endif

  switch (dboardConfig) {
  case TXA_RXB:
    txSubdevSpec = usrp_subdev_spec(0,0);
    rxSubdevSpec = usrp_subdev_spec(1,0);
    break;
  case TXB_RXA:
    txSubdevSpec = usrp_subdev_spec(1,0);
    rxSubdevSpec = usrp_subdev_spec(0,0);
    break;
  case TXA_RXA:
    txSubdevSpec = usrp_subdev_spec(0,0);
    rxSubdevSpec = usrp_subdev_spec(0,0);
    break;
  case TXB_RXB:
    txSubdevSpec = usrp_subdev_spec(1,0);
    rxSubdevSpec = usrp_subdev_spec(1,0);
    break;
  default:
    txSubdevSpec = usrp_subdev_spec(0,0);
    rxSubdevSpec = usrp_subdev_spec(1,0);
  }

  m_dbTx = m_uTx->selected_subdev(txSubdevSpec);
  m_dbRx = m_uRx->selected_subdev(rxSubdevSpec);

  started = false;

  return NORMAL;
}



bool USRPDevice::start()
{
  LOGC(DDEV, INFO) << "starting USRP...";
#ifndef SWLOOPBACK
  if (!m_uRx) return false;
  if (!m_uTx) return false;

  m_uRx->stop();
  m_uTx->stop();

  writeLock.lock();
  // power up and configure daughterboards
  m_dbTx->set_enable(true);
  m_uTx->set_mux(m_uTx->determine_tx_mux_value(txSubdevSpec));
  m_uRx->set_mux(m_uRx->determine_rx_mux_value(rxSubdevSpec));

  if (!m_dbRx->select_rx_antenna(1))
    m_dbRx->select_rx_antenna(0);

  writeLock.unlock();

  // Set gains to midpoint
  setTxGain((m_dbTx->gain_min() + m_dbTx->gain_max()) / 2);
  setRxGain((m_dbTx->gain_min() + m_dbTx->gain_max()) / 2);

  data = new short[currDataSize];
  dataStart = 0;
  dataEnd = 0;
  timeStart = 0;
  timeEnd = 0;
  timestampOffset = 0;
  latestWriteTimestamp = 0;
  lastPktTimestamp = 0;
  hi32Timestamp = 0;
  isAligned = false;


  started = (m_uRx->start() && m_uTx->start());
  return started;
#else
  gettimeofday(&lastReadTime,NULL);
  return true;
#endif
}

bool USRPDevice::stop()
{
#ifndef SWLOOPBACK
  if (!m_uRx) return false;
  if (!m_uTx) return false;

  delete[] currData;

  started = !(m_uRx->stop() && m_uTx->stop());
  return !started;
#else
  return true;
#endif
}

double USRPDevice::maxRxGain()
{
  return m_dbRx->gain_max();
}

double USRPDevice::minRxGain()
{
  return m_dbRx->gain_min();
}

double USRPDevice::setTxGain(double dB, size_t chan)
{
  if (chan) {
    LOGC(DDEV, ALERT) << "Invalid channel " << chan;
    return 0.0;
  }

  writeLock.lock();
  if (dB > m_dbTx->gain_max())
    dB = m_dbTx->gain_max();
  if (dB < m_dbTx->gain_min())
    dB = m_dbTx->gain_min();

  LOGC(DDEV, NOTICE) << "Setting TX gain to " << dB << " dB.";

  if (!m_dbTx->set_gain(dB))
    LOGC(DDEV, ERR) << "Error setting TX gain";
  else
    txGain = dB;
  writeLock.unlock();

  return txGain;
}


double USRPDevice::setRxGain(double dB, size_t chan)
{
  if (chan) {
    LOGC(DDEV, ALERT) << "Invalid channel " << chan;
    return 0.0;
  }

  dB = 47.0;

  writeLock.lock();
  if (dB > maxRxGain())
    dB = maxRxGain();
  if (dB < minRxGain())
    dB = minRxGain();

  LOGC(DDEV, NOTICE) << "Setting RX gain to " << dB << " dB.";

  if (!m_dbRx->set_gain(dB))
    LOGC(DDEV, ERR) << "Error setting RX gain";
  else
    rxGain = dB;
  writeLock.unlock();

  return rxGain;
}

double USRPDevice::setPowerAttenuation(int atten, size_t chan) {
      double rfGain;
      rfGain = setTxGain(m_dbTx->gain_max() - atten, chan);
      return m_dbTx->gain_max() - rfGain;
}
double USRPDevice::getPowerAttenuation(size_t chan) {
      return m_dbTx->gain_max() - getTxGain(chan);
}

int USRPDevice::getNominalTxPower(size_t chan)
{
	/* TODO: return value based on some experimentally generated table depending on
	 * band/arfcn, which is known here thanks to TXTUNE
	 */
	return 23;
}

bool USRPDevice::setRxAntenna(const std::string &ant, size_t chan)
{
	if (chan >= rx_paths.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return false;
	}
	LOGC(DDEV, ALERT) << "Not implemented";
	return true;
}

std::string USRPDevice::getRxAntenna(size_t chan)
{
	if (chan >= rx_paths.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return "";
	}
	LOGC(DDEV, ALERT) << "Not implemented";
	return "";
}

bool USRPDevice::setTxAntenna(const std::string &ant, size_t chan)
{
	if (chan >= tx_paths.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return false;
	}
	LOGC(DDEV, ALERT) << "Not implemented";
	return true;
}

std::string USRPDevice::getTxAntenna(size_t chan)
{
	if (chan >= tx_paths.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return "";
	}
	LOGC(DDEV, ALERT) << "Not implemented";
	return "";
}

bool USRPDevice::requiresRadioAlign()
{
	return true;
}

GSM::Time USRPDevice::minLatency() {
	return GSM::Time(1,1);
}

// NOTE: Assumes sequential reads
int USRPDevice::readSamples(std::vector<short *> &bufs, int len, bool *overrun,
                            TIMESTAMP timestamp, bool *underrun)
{
#ifndef SWLOOPBACK
  if (!m_uRx)
    return 0;

  short *buf = bufs[0];

  timestamp += timestampOffset;

  if (timestamp + len < timeStart) {
    memset(buf,0,len*2*sizeof(short));
    return len;
  }

  *underrun = false;

  uint32_t readBuf[2000];

  while (1) {
    //guestimate USB read size
    int readLen=0;
    {
      int numSamplesNeeded = timestamp + len - timeEnd;
      if (numSamplesNeeded <=0) break;
      readLen = 512 * ((int) ceil((float) numSamplesNeeded/126.0));
      if (readLen > 8000) readLen= (8000/512)*512;
    }

    // read USRP packets, parse and save A/D data as needed
    readLen = m_uRx->read((void *)readBuf,readLen,overrun);
    for (int pktNum = 0; pktNum < (readLen/512); pktNum++) {
      // tmpBuf points to start of a USB packet
      uint32_t* tmpBuf = (uint32_t *) (readBuf+pktNum*512/4);
      TIMESTAMP pktTimestamp = usrp_to_host_u32(tmpBuf[1]);
      uint32_t word0 = usrp_to_host_u32(tmpBuf[0]);
      uint32_t chan = (word0 >> 16) & 0x1f;
      unsigned payloadSz = word0 & 0x1ff;
      LOGC(DDEV, DEBUG) << "first two bytes: " << hex << word0 << " " << dec << pktTimestamp;

      bool incrementHi32 = ((lastPktTimestamp & 0x0ffffffffll) > pktTimestamp);
      if (incrementHi32 && (timeStart!=0)) {
           LOGC(DDEV, DEBUG) << "high 32 increment!!!";
           hi32Timestamp++;
      }
      pktTimestamp = (((TIMESTAMP) hi32Timestamp) << 32) | pktTimestamp;
      lastPktTimestamp = pktTimestamp;

      if (chan == 0x01f) {
	// control reply, check to see if its ping reply
        uint32_t word2 = usrp_to_host_u32(tmpBuf[2]);
	if ((word2 >> 16) == ((0x01 << 8) | 0x02)) {
          timestamp -= timestampOffset;
	  timestampOffset = pktTimestamp - pingTimestamp + pingOffset;
	  LOGC(DDEV, DEBUG) << "updating timestamp offset to: " << timestampOffset;
          timestamp += timestampOffset;
	  isAligned = true;
	}
	continue;
      }
      if (chan != 0) {
	LOGC(DDEV, DEBUG) << "chan: " << chan << ", timestamp: " << pktTimestamp << ", sz:" << payloadSz;
	continue;
      }
      if ((word0 >> 28) & 0x04) {
	*underrun = true;
	LOGC(DDEV, DEBUG) << "UNDERRUN in TRX->USRP interface";
      }
#if 0
      /* FIXME: Do something with this ? */
      unsigned RSSI = (word0 >> 21) & 0x3f;
#endif
      if (!isAligned) continue;

      unsigned cursorStart = pktTimestamp - timeStart + dataStart;
      while (cursorStart*2 > currDataSize) {
	cursorStart -= currDataSize/2;
      }
      if (cursorStart*2 + payloadSz/2 > currDataSize) {
	// need to circle around buffer
	memcpy(data+cursorStart*2,tmpBuf+2,(currDataSize-cursorStart*2)*sizeof(short));
	memcpy(data,tmpBuf+2+(currDataSize/2-cursorStart),payloadSz-(currDataSize-cursorStart*2)*sizeof(short));
      }
      else {
	memcpy(data+cursorStart*2,tmpBuf+2,payloadSz);
      }
      if (pktTimestamp + payloadSz/2/sizeof(short) > timeEnd)
	timeEnd = pktTimestamp+payloadSz/2/sizeof(short);

      LOGC(DDEV, DEBUG) << "timeStart: " << timeStart << ", timeEnd: " << timeEnd << ", pktTimestamp: " << pktTimestamp;

    }
  }

  // copy desired data to buf
  unsigned bufStart = dataStart+(timestamp-timeStart);
  if (bufStart + len < currDataSize/2) {
    LOGC(DDEV, DEBUG) << "bufStart: " << bufStart;
    memcpy(buf,data+bufStart*2,len*2*sizeof(short));
    memset(data+bufStart*2,0,len*2*sizeof(short));
  }
  else {
    LOGC(DDEV, DEBUG) << "len: " << len << ", currDataSize/2: " << currDataSize/2 << ", bufStart: " << bufStart;
    unsigned firstLength = (currDataSize/2-bufStart);
    LOGC(DDEV, DEBUG) << "firstLength: " << firstLength;
    memcpy(buf,data+bufStart*2,firstLength*2*sizeof(short));
    memset(data+bufStart*2,0,firstLength*2*sizeof(short));
    memcpy(buf+firstLength*2,data,(len-firstLength)*2*sizeof(short));
    memset(data,0,(len-firstLength)*2*sizeof(short));
  }
  dataStart = (bufStart + len) % (currDataSize/2);
  timeStart = timestamp + len;

  return len;

#else
  if (loopbackBufferSize < 2) return 0;
  int numSamples = 0;
  struct timeval currTime;
  gettimeofday(&currTime,NULL);
  double timeElapsed = (currTime.tv_sec - lastReadTime.tv_sec)*1.0e6 +
    (currTime.tv_usec - lastReadTime.tv_usec);
  if (timeElapsed < samplePeriod) {return 0;}
  int numSamplesToRead = (int) floor(timeElapsed/samplePeriod);
  if (numSamplesToRead < len) return 0;

  if (numSamplesToRead > len) numSamplesToRead = len;
  if (numSamplesToRead > loopbackBufferSize/2) {
    firstRead =false;
    numSamplesToRead = loopbackBufferSize/2;
  }
  memcpy(buf,loopbackBuffer,sizeof(short)*2*numSamplesToRead);
  loopbackBufferSize -= 2*numSamplesToRead;
  memcpy(loopbackBuffer,loopbackBuffer+2*numSamplesToRead,
	 sizeof(short)*loopbackBufferSize);
  numSamples = numSamplesToRead;
  if (firstRead) {
    int new_usec = lastReadTime.tv_usec + (int) round((double) numSamplesToRead * samplePeriod);
    lastReadTime.tv_sec = lastReadTime.tv_sec + new_usec/1000000;
    lastReadTime.tv_usec = new_usec % 1000000;
  }
  else {
    gettimeofday(&lastReadTime,NULL);
    firstRead = true;
  }

  return numSamples;
#endif
}

int USRPDevice::writeSamplesControl(std::vector<short *> &bufs, int len,
                             bool *underrun, unsigned long long timestamp, bool isControl)
{
  writeLock.lock();

#ifndef SWLOOPBACK
  if (!m_uTx)
    return 0;

  short *buf = bufs[0];

  static uint32_t outData[128*20];

  for (int i = 0; i < len*2; i++) {
	buf[i] = host_to_usrp_short(buf[i]);
  }

  int numWritten = 0;
  unsigned isStart = 1;
  unsigned RSSI = 0;
  unsigned CHAN = (isControl) ? 0x01f : 0x00;
  len = len*2*sizeof(short);
  int numPkts = (int) ceil((float)len/(float)504);
  unsigned isEnd = (numPkts < 2);
  uint32_t *outPkt = outData;
  int pktNum = 0;
  while (numWritten < len) {
    // pkt is pointer to start of a USB packet
    uint32_t *pkt = outPkt + pktNum*128;
    isEnd = (len - numWritten <= 504);
    unsigned payloadLen = ((len - numWritten) < 504) ? (len-numWritten) : 504;
    pkt[0] = (isStart << 12 | isEnd << 11 | (RSSI & 0x3f) << 5 | CHAN) << 16 | payloadLen;
    pkt[1] = timestamp & 0x0ffffffffll;
    memcpy(pkt+2,buf+(numWritten/sizeof(short)),payloadLen);
    numWritten += payloadLen;
    timestamp += payloadLen/2/sizeof(short);
    isStart = 0;
    pkt[0] = host_to_usrp_u32(pkt[0]);
    pkt[1] = host_to_usrp_u32(pkt[1]);
    pktNum++;
  }
  m_uTx->write((const void*) outPkt,sizeof(uint32_t)*128*numPkts,NULL);

  writeLock.unlock();

  return len/2/sizeof(short);
#else
  int retVal = len;
  memcpy(loopbackBuffer+loopbackBufferSize,buf,sizeof(short)*2*len);
  loopbackBufferSize += retVal*2;

  return retVal;
#endif
}

int USRPDevice::writeSamples(std::vector<short *> &bufs, int len,
                             bool *underrun, unsigned long long timestamp)
{
  return writeSamplesControl(bufs, len, underrun, timestamp, false);
}

bool USRPDevice::updateAlignment(TIMESTAMP timestamp)
{
#ifndef SWLOOPBACK
  short data[] = {0x00,0x02,0x00,0x00};
  /* FIXME: big endian */
  bool tmpUnderrun;

  std::vector<short *> buf(1, data);
  if (writeSamplesControl(buf, 1, &tmpUnderrun, timestamp & 0x0ffffffffll, true)) {
    pingTimestamp = timestamp;
    return true;
  }
  return false;
#else
  return true;
#endif
}

#ifndef SWLOOPBACK
bool USRPDevice::setTxFreq(double wFreq, size_t chan)
{
  usrp_tune_result result;

  if (chan) {
    LOGC(DDEV, ALERT) << "Invalid channel " << chan;
    return false;
  }

  if (m_uTx->tune(txSubdevSpec.side, m_dbTx, wFreq, &result)) {
    LOGC(DDEV, INFO) << "set TX: " << wFreq << std::endl
              << "    baseband freq: " << result.baseband_freq << std::endl
              << "    DDC freq:      " << result.dxc_freq << std::endl
              << "    residual freq: " << result.residual_freq;
    return true;
  }
  else {
    LOGC(DDEV, ALERT) << "set TX: " << wFreq << " failed" << std::endl
               << "    baseband freq: " << result.baseband_freq << std::endl
               << "    DDC freq:      " << result.dxc_freq << std::endl
               << "    residual freq: " << result.residual_freq;
    return false;
  }
}

bool USRPDevice::setRxFreq(double wFreq, size_t chan)
{
  usrp_tune_result result;

  if (chan) {
    LOGC(DDEV, ALERT) << "Invalid channel " << chan;
    return false;
  }

  if (m_uRx->tune(0, m_dbRx, wFreq, &result)) {
    LOGC(DDEV, INFO) << "set RX: " << wFreq << std::endl
              << "    baseband freq: " << result.baseband_freq << std::endl
              << "    DDC freq:      " << result.dxc_freq << std::endl
              << "    residual freq: " << result.residual_freq;
    return true;
  }
  else {
    LOGC(DDEV, ALERT) << "set RX: " << wFreq << " failed" << std::endl
               << "    baseband freq: " << result.baseband_freq << std::endl
               << "    DDC freq:      " << result.dxc_freq << std::endl
               << "    residual freq: " << result.residual_freq;
    return false;
  }

}

#else
bool USRPDevice::setTxFreq(double wFreq) { return true;};
bool USRPDevice::setRxFreq(double wFreq) { return true;};
#endif

RadioDevice *RadioDevice::make(InterfaceType type, const struct trx_cfg *cfg)
{
  if (cfg->tx_sps != cfg->rx_sps) {
    LOGC(DDEV, ERROR) << "USRP1 requires tx_sps == rx_sps";
    return NULL;
  }
  if (cfg->num_chans != 1) {
    LOGC(DDEV, ERROR) << "USRP1 supports only 1 channel";
    return NULL;
  }
  if (cfg->offset != 0.0) {
    LOGC(DDEV, ERROR) << "USRP1 doesn't support lo_offset";
    return NULL;
  }
  return new USRPDevice(type, cfg);
}
