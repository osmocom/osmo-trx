/*
 * Multi-carrier radio interface
 *
 * Copyright (C) 2016 Ettus Research LLC
 *
 * Author: Tom Tsou <tom.tsou@ettus.com>
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

#include <radioInterface.h>
#include <Logger.h>

#include "Resampler.h"

extern "C" {
#include "convert.h"
}

/* Resampling parameters for 64 MHz clocking */
#define RESAMP_INRATE			65
#define RESAMP_OUTRATE			(96 / 2)

/* Universal resampling parameters */
#define NUMCHUNKS				24

/* number of narrow-band virtual ARFCNs in this wide-band multi-ARFCN device */
#define MCHANS					4

RadioInterfaceMulti::RadioInterfaceMulti(RadioDevice *radio, size_t tx_sps,
					 size_t rx_sps, size_t chans)
	: RadioInterface(radio, tx_sps, rx_sps, chans),
	  outerSendBuffer(NULL), outerRecvBuffer(NULL),
	  dnsampler(NULL), upsampler(NULL), channelizer(NULL), synthesis(NULL)
{
}

RadioInterfaceMulti::~RadioInterfaceMulti()
{
	close();
}

void RadioInterfaceMulti::close()
{
	delete outerSendBuffer;
	delete outerRecvBuffer;
	delete dnsampler;
	delete upsampler;
	delete channelizer;
	delete synthesis;

	outerSendBuffer = NULL;
	outerRecvBuffer = NULL;
	dnsampler = NULL;
	upsampler = NULL;
	channelizer = NULL;
	synthesis = NULL;


	for (std::vector<signalVector*>::iterator it = history.begin(); it != history.end(); ++it)
		delete *it;

	mReceiveFIFO.resize(0);
	powerScaling.resize(0);
	history.resize(0);
	active.resize(0);
	rx_freq_state.resize(0);
	tx_freq_state.resize(0);

	RadioInterface::close();
}

/*! we re-map the physical channels from the filter bank to logical per-TRX channels
 *  \param[in] pchan physical channel number within the channelizer
 *  \param[in] chans total number of narrow-band ARFCN channels
 *  \returns logical (TRX) channel number, or -1 in case there is none */
static int getLogicalChan(size_t pchan, size_t chans)
{
	switch (chans) {
	case 1:
		if (pchan == 0)
			return 0;
		else
			return -1;
		break;
	case 2:
		if (pchan == 0)
			return 0;
		if (pchan == 3)
			return 1;
		else
			return -1;
		break;
	case 3:
		if (pchan == 1)
			return 0;
		if (pchan == 0)
			return 1;
		if (pchan == 3)
			return 2;
		else
			return -1;
		break;
	default:
		break;
	};

	return -1;
}

/*! do we need to frequency shift our spectrum or not?
 *  \param chans total number of channels
 *  \returns 1 if we need to shift; 0 if not; -1 on error */
static int getFreqShift(size_t chans)
{
	switch (chans) {
	case 1:
		return 0;
	case 2:
		return 0;
	case 3:
		return 1;
	default:
		break;
	};

	return -1;
}

/* Initialize I/O specific objects */
bool RadioInterfaceMulti::init(int type)
{
	float cutoff = 1.0f;
	size_t inchunk = 0, outchunk = 0;

	if (mChans > MCHANS - 1) {
		LOG(ALERT) << "Invalid channel configuration " << mChans;
		return false;
	}

	close();

	sendBuffer.resize(mChans);
	recvBuffer.resize(mChans);
	convertSendBuffer.resize(1);
	convertRecvBuffer.resize(1);

	mReceiveFIFO.resize(mChans);
	powerScaling.resize(mChans);
	history.resize(mChans);
	rx_freq_state.resize(mChans);
	tx_freq_state.resize(mChans);
	active.resize(MCHANS, false);

	/* 4 == sps */
	inchunk = RESAMP_INRATE * 4;
	outchunk = RESAMP_OUTRATE * 4;

	if (inchunk  * NUMCHUNKS < 625 * 2) {
		LOG(ALERT) << "Invalid inner chunk size " << inchunk;
		return false;
	}

	dnsampler = new Resampler(RESAMP_INRATE, RESAMP_OUTRATE);
	if (!dnsampler->init(1.0)) {
		LOG(ALERT) << "Rx resampler failed to initialize";
		return false;
	}

	upsampler = new Resampler(RESAMP_OUTRATE, RESAMP_INRATE);
	if (!upsampler->init(cutoff)) {
		LOG(ALERT) << "Tx resampler failed to initialize";
		return false;
	}

	channelizer = new Channelizer(MCHANS, outchunk);
	if (!channelizer->init()) {
		LOG(ALERT) << "Rx channelizer failed to initialize";
		return false;
	}

	synthesis = new Synthesis(MCHANS, outchunk);
	if (!synthesis->init()) {
		LOG(ALERT) << "Tx synthesis filter failed to initialize";
		return false;
	}

	/*
	 * Allocate high and low rate buffers. The high rate receive
	 * buffer and low rate transmit vectors feed into the resampler
	 * and requires headroom equivalent to the filter length. Low
	 * rate buffers are allocated in the main radio interface code.
	 */
	for (size_t i = 0; i < mChans; i++) {
		sendBuffer[i] = new RadioBuffer(NUMCHUNKS, inchunk,
					        upsampler->len(), true);
		recvBuffer[i] = new RadioBuffer(NUMCHUNKS, inchunk,
		                                0, false);
		history[i] = new signalVector(dnsampler->len());

		synthesis->resetBuffer(i);
	}

	outerSendBuffer = new signalVector(synthesis->outputLen());
	outerRecvBuffer = new signalVector(channelizer->inputLen());

	convertSendBuffer[0] = new short[2 * synthesis->outputLen()];
	convertRecvBuffer[0] = new short[2 * channelizer->inputLen()];

	/* Configure channels */
	switch (mChans) {
	case 1:
		active[0] = true;
		break;
	case 2:
		active[0] = true;
		active[3] = true;
		break;
	case 3:
		active[0] = true;
		active[1] = true;
		active[3] = true;
		break;
	default:
		LOG(ALERT) << "Unsupported channel combination";
		return false;
	}

	return true;
}

/* Receive a timestamped chunk from the device */
int RadioInterfaceMulti::pullBuffer()
{
	bool local_underrun;
	size_t num;
	float *buf;
	unsigned int i;

	if (recvBuffer[0]->getFreeSegments() <= 0)
		return -1;

	/* Outer buffer access size is fixed */
	num = mDevice->readSamples(convertRecvBuffer,
				  outerRecvBuffer->size(),
				  &overrun,
				  readTimestamp,
				  &local_underrun);
	if (num != channelizer->inputLen()) {
		LOG(ALERT) << "Receive error " << num << ", " << channelizer->inputLen();
		return -1;
	}

	convert_short_float((float *) outerRecvBuffer->begin(),
			    convertRecvBuffer[0], 2 * outerRecvBuffer->size());

	osmo_trx_sync_or_and_fetch(&underrun, local_underrun);
	readTimestamp += num;

	channelizer->rotate((float *) outerRecvBuffer->begin(),
			    outerRecvBuffer->size());

	for (size_t pchan = 0; pchan < MCHANS; pchan++) {
		if (!active[pchan])
			continue;

		int lchan = getLogicalChan(pchan, mChans);
		if (lchan < 0) {
			LOG(ALERT) << "Invalid logical channel " << pchan;
			continue;
		}

		/*
		 * Update history by writing into the head portion of the
		 * channelizer output buffer. For this to work, filter length of
		 * the polyphase channelizer partition filter should be equal to
		 * or larger than the resampling filter.
		 */
		buf = channelizer->outputBuffer(pchan);
		size_t cLen = channelizer->outputLen();
		size_t hLen = dnsampler->len();

		float *fdst = &buf[2 * -hLen];
		complex *src = history[lchan]->begin();
		for (i = 0; i < hLen; i++) {
			fdst[0] = src->real();
			fdst[1] = src->imag();
			src++;
			fdst += 2;
		}
		complex *dst = history[lchan]->begin();
		float *fsrc = &buf[2 * (cLen - hLen)];
		for (i = 0; i < hLen; i++) {
			*dst = complex(fsrc[0], fsrc[1]);
			fsrc += 2;
			dst++;
		}

		float *wr_segment = recvBuffer[lchan]->getWriteSegment();

		/* Write to the end of the inner receive buffer */
		if (!dnsampler->rotate(channelizer->outputBuffer(pchan),
				       channelizer->outputLen(),
				       wr_segment,
				       recvBuffer[lchan]->getSegmentLen())) {
			LOG(ALERT) << "Sample rate upsampling error";
		}
	}
	return 0;
}

/* Send a timestamped chunk to the device */
bool RadioInterfaceMulti::pushBuffer()
{
	bool local_underrun;
	if (sendBuffer[0]->getAvailSegments() <= 0)
		return false;

	for (size_t pchan = 0; pchan < MCHANS; pchan++) {
		if (!active[pchan]) {
			synthesis->resetBuffer(pchan);
			continue;
		}

		int lchan = getLogicalChan(pchan, mChans);
		if (lchan < 0) {
			LOG(ALERT) << "Invalid logical channel " << pchan;
			continue;
		}

		if (!upsampler->rotate(sendBuffer[lchan]->getReadSegment(),
				       sendBuffer[lchan]->getSegmentLen(),
				       synthesis->inputBuffer(pchan),
				       synthesis->inputLen())) {
			LOG(ALERT) << "Sample rate downsampling error";
		}
	}

	synthesis->rotate((float *) outerSendBuffer->begin(),
			  outerSendBuffer->size());

	convert_float_short(convertSendBuffer[0],
			    (float *) outerSendBuffer->begin(),
			    1.0 / (float) mChans, 2 * outerSendBuffer->size());

	size_t num = mDevice->writeSamples(convertSendBuffer,
					  outerSendBuffer->size(),
					  &local_underrun,
					  writeTimestamp);
	if (num != outerSendBuffer->size()) {
		LOG(ALERT) << "Transmit error " << num;
	}

	osmo_trx_sync_or_and_fetch(&underrun, local_underrun);
	writeTimestamp += num;

	return true;
}

/* Frequency comparison limit */
#define FREQ_DELTA_LIMIT		10.0

static bool fltcmp(double a, double b)
{
	return fabs(a - b) < FREQ_DELTA_LIMIT ? true : false;
}

bool RadioInterfaceMulti::verify_arfcn_consistency(double freq, size_t chan, bool tx)
{
	double freq_i;
	std::string str_dir = tx ? "Tx" : "Rx";
	std::vector<struct freq_cfg_state> &v = tx ? tx_freq_state : rx_freq_state;

	for (size_t i = 0; i < mChans; i++) {
		if (i == chan)
			continue;
		if (!v[i].set)
			continue;

		freq_i = v[i].freq_hz + (double) ((int)chan - (int)i) * MCBTS_SPACING;
		if (!fltcmp(freq, freq_i)) {
			LOGCHAN(chan, DMAIN, ERROR)
				<< "Setting " << str_dir << " frequency " << freq
				<< " is incompatible: already configured channel "
				<< i << " uses frequency " << v[i].freq_hz
				<< " (expected " << freq_i << ")";
			return false;
		}
	}
	v[chan].set = true;
	v[chan].freq_hz = freq;
	return true;
}

bool RadioInterfaceMulti::tuneTx(double freq, size_t chan)
{
	double shift;

	if (chan >= mChans)
		return false;

	if (!verify_arfcn_consistency(freq, chan, true))
		return false;

	if (chan == 0) {
		shift = (double) getFreqShift(mChans);
		return mDevice->setTxFreq(freq + shift * MCBTS_SPACING);
	}

	return true;
}

bool RadioInterfaceMulti::tuneRx(double freq, size_t chan)
{
	double shift;

	if (chan >= mChans)
		return false;

	if (!verify_arfcn_consistency(freq, chan, false))
		return false;

	if (chan == 0) {
		shift = (double) getFreqShift(mChans);
		return mDevice->setRxFreq(freq + shift * MCBTS_SPACING);
	}

	return true;
}

double RadioInterfaceMulti::setRxGain(double db, size_t chan)
{
	if (chan == 0)
		return mDevice->setRxGain(db);
	else
		return mDevice->getRxGain();
}

double RadioInterfaceMulti::rssiOffset(size_t chan)
{
	return mDevice->rssiOffset(0);
}

int RadioInterfaceMulti::setPowerAttenuation(int atten, size_t chan)
{
	return RadioInterface::setPowerAttenuation(atten, 0);
}
