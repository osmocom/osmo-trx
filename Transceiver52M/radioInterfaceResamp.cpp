/*
 * Radio device interface with sample rate conversion
 *
 * Copyright (C) 2011-2014 Free Software Foundation, Inc.
 * Copyright (C) 2015 Ettus Research LLC
 *
 * Author: Tom Tsou <tom@tsou.cc>
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
#define RESAMP_64M_INRATE			65
#define RESAMP_64M_OUTRATE			96

/* Resampling parameters for 100 MHz clocking */
#define RESAMP_100M_INRATE			52
#define RESAMP_100M_OUTRATE			75

/* Universal resampling parameters */
#define NUMCHUNKS				24

/*
 * Resampling filter bandwidth scaling factor
 *   This narrows the filter cutoff relative to the output bandwidth
 *   of the polyphase resampler. At 4 samples-per-symbol using the
 *   2 pulse Laurent GMSK approximation gives us below 0.5 degrees
 *   RMS phase error at the resampler output.
 */
#define RESAMP_TX4_FILTER		0.45

static Resampler *upsampler = NULL;
static Resampler *dnsampler = NULL;
static size_t resamp_inrate = 0;
static size_t resamp_inchunk = 0;
static size_t resamp_outrate = 0;
static size_t resamp_outchunk = 0;

RadioInterfaceResamp::RadioInterfaceResamp(RadioDevice *wRadio,
					   size_t tx_sps, size_t rx_sps)
	: RadioInterface(wRadio, tx_sps, rx_sps, 1),
	  outerSendBuffer(NULL), outerRecvBuffer(NULL)
{
}

RadioInterfaceResamp::~RadioInterfaceResamp()
{
	close();
}

void RadioInterfaceResamp::close()
{
	delete outerSendBuffer;
	delete outerRecvBuffer;

	delete upsampler;
	delete dnsampler;

	outerSendBuffer = NULL;
	outerRecvBuffer = NULL;

	upsampler = NULL;
	dnsampler = NULL;

	if (sendBuffer.size())
		sendBuffer[0] = NULL;
	if (recvBuffer.size())
		recvBuffer[0] = NULL;

	RadioInterface::close();
}

/* Initialize I/O specific objects */
bool RadioInterfaceResamp::init(int type)
{
	float cutoff = 1.0f;

	close();

	sendBuffer.resize(1);
	recvBuffer.resize(1);
	convertSendBuffer.resize(1);
	convertRecvBuffer.resize(1);
	mReceiveFIFO.resize(1);
	powerScaling.resize(1);

	switch (type) {
	case RadioDevice::RESAMP_64M:
		resamp_inrate = RESAMP_64M_INRATE;
		resamp_outrate = RESAMP_64M_OUTRATE;
		break;
	case RadioDevice::RESAMP_100M:
		resamp_inrate = RESAMP_100M_INRATE;
		resamp_outrate = RESAMP_100M_OUTRATE;
		break;
	case RadioDevice::NORMAL:
	default:
		LOG(ALERT) << "Invalid device configuration";
		return false;
	}

	resamp_inchunk = resamp_inrate * 4 * mSPSRx;
	resamp_outchunk = resamp_outrate * 4 * mSPSRx;

	if (mSPSTx == 4)
		cutoff = RESAMP_TX4_FILTER;

	dnsampler = new Resampler(resamp_inrate, resamp_outrate);
	if (!dnsampler->init()) {
		LOG(ALERT) << "Rx resampler failed to initialize";
		return false;
	}

	upsampler = new Resampler(resamp_outrate, resamp_inrate);
	if (!upsampler->init(cutoff)) {
		LOG(ALERT) << "Tx resampler failed to initialize";
		return false;
	}

	/*
	 * Allocate high and low rate buffers. The high rate receive
	 * buffer and low rate transmit vectors feed into the resampler
	 * and requires headroom equivalent to the filter length. Low
	 * rate buffers are allocated in the main radio interface code.
	 */
	sendBuffer[0] = new RadioBuffer(NUMCHUNKS, resamp_inchunk,
					  upsampler->len(), true);
	recvBuffer[0] = new RadioBuffer(NUMCHUNKS * 20, resamp_inchunk, 0, false);

	outerSendBuffer =
		new signalVector(NUMCHUNKS * resamp_outchunk);
	outerRecvBuffer =
		new signalVector(resamp_outchunk, dnsampler->len());

	convertSendBuffer[0] = new short[outerSendBuffer->size() * 2];
	convertRecvBuffer[0] = new short[outerRecvBuffer->size() * 2];

	return true;
}

/* Receive a timestamped chunk from the device */
int RadioInterfaceResamp::pullBuffer()
{
	bool local_underrun;
	int rc, num_recv;

	if (recvBuffer[0]->getFreeSegments() <= 0)
		return -1;

	/* Outer buffer access size is fixed */
	num_recv = mRadio->readSamples(convertRecvBuffer,
				       resamp_outchunk,
				       &overrun,
				       readTimestamp,
				       &local_underrun);
	if (num_recv != (int) resamp_outchunk) {
		LOG(ALERT) << "Receive error " << num_recv;
		return -1;
	}

	convert_short_float((float *) outerRecvBuffer->begin(),
			    convertRecvBuffer[0], 2 * resamp_outchunk);

	underrun |= local_underrun;
	readTimestamp += (TIMESTAMP) resamp_outchunk;

	/* Write to the end of the inner receive buffer */
	rc = dnsampler->rotate((float *) outerRecvBuffer->begin(),
			       resamp_outchunk,
			       recvBuffer[0]->getWriteSegment(),
			       resamp_inchunk);
	if (rc < 0) {
		LOG(ALERT) << "Sample rate upsampling error";
	}

	/* Set history for the next chunk */
	outerRecvBuffer->updateHistory();
	return 0;
}

/* Send a timestamped chunk to the device */
bool RadioInterfaceResamp::pushBuffer()
{
	int rc;
	size_t numSent;

	if (sendBuffer[0]->getAvailSegments() <= 0)
		return false;

	/* Always send from the beginning of the buffer */
	rc = upsampler->rotate(sendBuffer[0]->getReadSegment(),
			       resamp_inchunk,
			       (float *) outerSendBuffer->begin(),
			       resamp_outchunk);
	if (rc < 0) {
		LOG(ALERT) << "Sample rate downsampling error";
	}

	convert_float_short(convertSendBuffer[0],
			    (float *) outerSendBuffer->begin(),
			    powerScaling[0], 2 * resamp_outchunk);

	numSent = mRadio->writeSamples(convertSendBuffer,
				       resamp_outchunk,
				       &underrun,
				       writeTimestamp);
	if (numSent != resamp_outchunk) {
		LOG(ALERT) << "Transmit error " << numSent;
	}

	writeTimestamp += resamp_outchunk;

	return true;
}
