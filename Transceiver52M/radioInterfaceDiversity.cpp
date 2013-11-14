/*
 * SSE Convolution
 * Copyright (C) 2013 Thomas Tsou <tom@tsou.cc>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <radioInterface.h>
#include <Logger.h>

#include "Resampler.h"

extern "C" {
#include "convert.h"
}

/* Resampling parameters for 64 MHz clocking */
#define RESAMP_64M_INRATE			20
#define RESAMP_64M_OUTRATE			80

/* Downlink block size */
#define CHUNK					625

/* Universal resampling parameters */
#define NUMCHUNKS				48

/*
 * Resampling filter bandwidth scaling factor
 *   This narrows the filter cutoff relative to the output bandwidth
 *   of the polyphase resampler. At 4 samples-per-symbol using the
 *   2 pulse Laurent GMSK approximation gives us below 0.5 degrees
 *   RMS phase error at the resampler output.
 */
#define RESAMP_TX4_FILTER		0.45

static size_t resamp_inrate = 0;
static size_t resamp_inchunk = 0;
static size_t resamp_outrate = 0;
static size_t resamp_outchunk = 0;

RadioInterfaceDiversity::RadioInterfaceDiversity(RadioDevice *wRadio,
						 size_t sps, size_t chans)
	: RadioInterface(wRadio, sps, chans, 2), outerRecvBuffer(NULL),
	  mDiversity(false), mFreqSpacing(0.0)
{
}

RadioInterfaceDiversity::~RadioInterfaceDiversity()
{
	close();
}

void RadioInterfaceDiversity::close()
{
	delete outerRecvBuffer;

	outerRecvBuffer = NULL;

	for (size_t i = 0; i < dnsamplers.size(); i++) {
		delete dnsamplers[i];
		dnsamplers[i] = NULL;
	}

	if (recvBuffer.size())
		recvBuffer[0] = NULL;

	RadioInterface::close();
}

bool RadioInterfaceDiversity::setupDiversityChannels()
{
	size_t inner_rx_len;

	/* Inner and outer rates */
	resamp_inrate = RESAMP_64M_INRATE;
	resamp_outrate = RESAMP_64M_OUTRATE;
	resamp_inchunk = resamp_inrate * 4;
	resamp_outchunk = resamp_outrate * 4;

	/* Buffer lengths */
	inner_rx_len = NUMCHUNKS * resamp_inchunk;

	/* Inside buffer must hold at least 2 bursts */
	if (inner_rx_len < 157 * mSPSRx * 2) {
		LOG(ALERT) << "Invalid inner buffer size " << inner_rx_len;
		return false;
	}

	/* One Receive buffer and downsampler per diversity channel */
	for (size_t i = 0; i < mMIMO * mChans; i++) {
		dnsamplers[i] = new Resampler(resamp_inrate, resamp_outrate);
		if (!dnsamplers[i]->init()) {
			LOG(ALERT) << "Rx resampler failed to initialize";
			return false;
		}

		recvBuffer[i] = new signalVector(inner_rx_len);
	}

	return true;
}

/* Initialize I/O specific objects */
bool RadioInterfaceDiversity::init(int type)
{
	int tx_len, outer_rx_len;

	if ((mMIMO != 2) || (mChans != 2)) {
		LOG(ALERT) << "Unsupported channel configuration " << mChans;
		return false;
	}

	/* Resize for channel combination */
	sendBuffer.resize(mChans);
	recvBuffer.resize(mChans * mMIMO);
	convertSendBuffer.resize(mChans);
	convertRecvBuffer.resize(mChans);
	mReceiveFIFO.resize(mChans);
	dnsamplers.resize(mChans * mMIMO);
	phases.resize(mChans);

	if (!setupDiversityChannels())
		return false;

	tx_len = CHUNK * mSPSTx;
	outer_rx_len = resamp_outchunk;

	for (size_t i = 0; i < mChans; i++) {
		/* Full rate float and integer outer receive buffers */
		convertRecvBuffer[i] = new short[outer_rx_len * 2];

		/* Send buffers (not-resampled) */
		sendBuffer[i] = new signalVector(tx_len);
		convertSendBuffer[i] = new short[tx_len * 2];
	}

	outerRecvBuffer = new signalVector(outer_rx_len, dnsamplers[0]->len());

	return true;
}

bool RadioInterfaceDiversity::tuneRx(double freq, size_t chan)
{
	double f0, f1;

	if (chan > 1)
		return false;

	if (!mRadio->setRxFreq(freq, chan))
		return false;

	f0 = mRadio->getRxFreq(0);
	f1 = mRadio->getRxFreq(1);

	mFreqSpacing = f1 - f0;

	if (abs(mFreqSpacing) <= 600e3)
		mDiversity = true;
	else
		mDiversity = false;

	return true;
}

/* Receive a timestamped chunk from the device */
void RadioInterfaceDiversity::pullBuffer()
{
	bool local_underrun;
	int rc, num, path0, path1;
	signalVector *shift, *base;
	float *in, *out, rate = -mFreqSpacing * 2.0 * M_PI / 1.08333333e6;

	if (recvCursor > recvBuffer[0]->size() - resamp_inchunk)
		return;

	/* Outer buffer access size is fixed */
	num = mRadio->readSamples(convertRecvBuffer,
				  resamp_outchunk,
				  &overrun,
				  readTimestamp,
				  &local_underrun);
	if ((size_t) num != resamp_outchunk) {
		LOG(ALERT) << "Receive error " << num;
		return;
	}

	for (size_t i = 0; i < mChans; i++) {
		convert_short_float((float *) outerRecvBuffer->begin(),
			    convertRecvBuffer[i], 2 * resamp_outchunk);

		if (!i) {
			path0 = 0;
			path1 = 2;
		} else {
			path0 = 3;
			path1 = 1;
		}

		/* Diversity path 1 */
		base = outerRecvBuffer;
		in = (float *) base->begin();
		out = (float *) (recvBuffer[path0]->begin() + recvCursor);

		rc = dnsamplers[2 * i + 0]->rotate(in, resamp_outchunk,
						   out, resamp_inchunk);
		if (rc < 0) {
			LOG(ALERT) << "Sample rate downsampling error";
		}

		/* Enable path 2 if Nyquist bandwidth is sufficient */
		if (!mDiversity)
			continue;

		/* Diversity path 2 */
		shift = new signalVector(base->size(), base->getStart());
		in = (float *) shift->begin();
		out = (float *) (recvBuffer[path1]->begin() + recvCursor);

		rate = i ? -rate : rate;
		if (!frequencyShift(shift, base, rate, phases[i], &phases[i])) {
			LOG(ALERT) << "Frequency shift failed";
		}

		rc = dnsamplers[2 * i + 1]->rotate(in, resamp_outchunk,
						   out, resamp_inchunk);
		if (rc < 0) {
			LOG(ALERT) << "Sample rate downsampling error";
		}

		delete shift;
	}

	underrun |= local_underrun;
	readTimestamp += (TIMESTAMP) resamp_outchunk;
	recvCursor += resamp_inchunk;
}
