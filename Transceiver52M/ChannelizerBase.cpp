/*
 * Polyphase channelizer
 *
 * Copyright (C) 2012-2014 Tom Tsou <tom@tsou.cc>
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

#include <malloc.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <cstdio>

#include "Logger.h"
#include "ChannelizerBase.h"

extern "C" {
#include "fft.h"
}

static float sinc(float x)
{
	if (x == 0.0f)
		return 0.999999999999f;

	return sin(M_PI * x) / (M_PI * x);
}

/*
 * There are more efficient reversal algorithms, but we only reverse at
 * initialization so we don't care.
 */
static void reverse(float *buf, size_t len)
{
	float tmp[2 * len];
	memcpy(tmp, buf, 2 * len * sizeof(float));

	for (size_t i = 0; i < len; i++) {
		buf[2 * i + 0] = tmp[2 * (len - 1 - i) + 0];
		buf[2 * i + 1] = tmp[2 * (len - 1 - i) + 1];
	}
}

/*
 * Create polyphase filterbank
 *
 * Implementation based material found in,
 *
 * "harris, fred, Multirate Signal Processing, Upper Saddle River, NJ,
 *     Prentice Hall, 2006."
 */
bool ChannelizerBase::initFilters()
{
	size_t protoLen = m * hLen;
	float *proto;
	float sum = 0.0f, scale = 0.0f;
	float midpt = (float) (protoLen - 1.0) / 2.0;

	/*
	 * Allocate 'M' partition filters and the temporary prototype
	 * filter. Coefficients are real only and must be 16-byte memory
	 * aligned for SSE usage.
	 */
	proto = new float[protoLen];
	if (!proto)
		return false;

	subFilters = (float **) malloc(sizeof(float *) * m);
	if (!subFilters) {
		delete[] proto;
		return false;
	}

	for (size_t i = 0; i < m; i++) {
		subFilters[i] = (float *)
				memalign(16, hLen * 2 * sizeof(float));
	}

	/*
	 * Generate the prototype filter with a Blackman-harris window.
	 * Scale coefficients with DC filter gain set to unity divided
	 * by the number of channels.
	 */
	float a0 = 0.35875;
	float a1 = 0.48829;
	float a2 = 0.14128;
	float a3 = 0.01168;

	for (size_t i = 0; i < protoLen; i++) {
		proto[i] = sinc(((float) i - midpt) / (float) m);
		proto[i] *= a0 -
			    a1 * cos(2 * M_PI * i / (protoLen - 1)) +
			    a2 * cos(4 * M_PI * i / (protoLen - 1)) -
			    a3 * cos(6 * M_PI * i / (protoLen - 1));
		sum += proto[i];
	}
	scale = (float) m / sum;

	/*
	 * Populate partition filters and reverse the coefficients per
	 * convolution requirements.
	 */
	for (size_t i = 0; i < hLen; i++) {
		for (size_t n = 0; n < m; n++) {
			subFilters[n][2 * i + 0] = proto[i * m + n] * scale;
			subFilters[n][2 * i + 1] = 0.0f;
		}
	}

	for (size_t i = 0; i < m; i++)
		reverse(subFilters[i], hLen);

	delete[] proto;

	return true;
}

bool ChannelizerBase::initFFT()
{
	size_t size;

	if (fftInput || fftOutput || fftHandle)
		return false;

	size = blockLen * m * 2 * sizeof(float);
	fftInput = (float *) fft_malloc(size);
	memset(fftInput, 0, size);

	size = (blockLen + hLen) * m * 2 * sizeof(float);
	fftOutput = (float *) fft_malloc(size);
	memset(fftOutput, 0, size);

	if (!fftInput | !fftOutput) {
		LOG(ALERT) << "Memory allocation error";
		return false;
	}

	fftHandle = init_fft(0, m, blockLen, blockLen + hLen,
			     fftInput, fftOutput, hLen);
	return true;
}

bool ChannelizerBase::mapBuffers()
{
	if (!fftHandle) {
		LOG(ALERT) << "FFT buffers not initialized";
		return false;
	}

	hInputs = (float **) malloc(sizeof(float *) * m);
	hOutputs = (float **) malloc(sizeof(float *) * m);
	if (!hInputs | !hOutputs)
		return false;

	for (size_t i = 0; i < m; i++) {
		hInputs[i] = &fftOutput[2 * (i * (blockLen + hLen) + hLen)];
		hOutputs[i] = &fftInput[2 * (i * blockLen)];
	}

	return true;
}

/*
 * Setup filterbank internals
 */
bool ChannelizerBase::init()
{
	/*
	 * Filterbank coefficients, fft plan, history, and output sample
	 * rate conversion blocks
	 */
	if (!initFilters()) {
		LOG(ALERT) << "Failed to initialize channelizing filter";
		return false;
	}

	hist = (float **) malloc(sizeof(float *) * m);
	for (size_t i = 0; i < m; i++) {
		hist[i] = new float[2 * hLen];
		memset(hist[i], 0, 2 * hLen * sizeof(float));
	}

	if (!initFFT()) {
		LOG(ALERT) << "Failed to initialize FFT";
		return false;
	}

	mapBuffers();

	return true;
}

/* Check vector length validity */
bool ChannelizerBase::checkLen(size_t innerLen, size_t outerLen)
{
	if (outerLen != innerLen * m) {
		LOG(ALERT) << "Invalid outer length " << innerLen
			   <<  " is not multiple of " << blockLen;
		return false;
	}

	if (innerLen != blockLen) {
		LOG(ALERT) << "Invalid inner length " << outerLen
			   <<  " does not equal " << blockLen;
		return false;
	}

	return true;
}

/*
 * Setup channelizer parameters
 */
ChannelizerBase::ChannelizerBase(size_t m, size_t blockLen, size_t hLen)
	: subFilters(NULL), hInputs(NULL), hOutputs(NULL), hist(NULL),
	  fftInput(NULL), fftOutput(NULL), fftHandle(NULL)
{
	this->m = m;
	this->hLen = hLen;
	this->blockLen = blockLen;
}

ChannelizerBase::~ChannelizerBase()
{
	free_fft(fftHandle);

	for (size_t i = 0; i < m; i++) {
		free(subFilters[i]);
		delete[] hist[i];
	}
	free(subFilters);

	fft_free(fftInput);
	fft_free(fftOutput);

	free(hInputs);
	free(hOutputs);
	free(hist);
}
