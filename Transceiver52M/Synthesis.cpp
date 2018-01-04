/*
 * Polyphase synthesis filter
 * 
 * Copyright (C) 2012-2014 Tom Tsou <tom@tsou.cc>
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

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <cstdio>
#include <iostream>

#include "Synthesis.h"

extern "C" {
#include "common/fft.h"
#include "common/convolve.h"
}

static void interleave(float **in, size_t ilen,
		       float *out, size_t m)
{
	size_t i, n;

	for (i = 0; i < ilen; i++) {
		for (n = 0; n < m; n++) {
			out[2 * (i * m + n) + 0] = in[n][2 * i + 0];
			out[2 * (i * m + n) + 1] = in[n][2 * i + 1];
		}
	}
}

size_t Synthesis::inputLen() const
{
	return blockLen;
}

size_t Synthesis::outputLen() const
{
	return blockLen * m;
}

float *Synthesis::inputBuffer(size_t chan) const
{
	if (chan >= m)
		return NULL;

	return hOutputs[chan];
}

bool Synthesis::resetBuffer(size_t chan)
{
	if (chan >= m)
		return false;

	memset(hOutputs[chan], 0, blockLen * 2 * sizeof(float));

	return true;
}

/* 
 * Implementation based on material found in:
 *
 * "harris, fred, Multirate Signal Processing, Upper Saddle River, NJ,
 *     Prentice Hall, 2006."
 */
bool Synthesis::rotate(float *out, size_t len)
{
	size_t hSize = 2 * hLen * sizeof(float);

	if (!checkLen(blockLen, len)) {
		std::cout << "Length fail" << std::endl;
		exit(1);
		return false;
	}

	cxvec_fft(fftHandle);

	/* 
	 * Convolve through filterbank while applying and saving sample history 
	 */
	for (size_t i = 0; i < m; i++) {
		memcpy(&hInputs[i][2 * -hLen], hist[i], hSize);
		memcpy(hist[i], &hInputs[i][2 * (blockLen - hLen)], hSize);

		convolve_real(hInputs[i], blockLen,
			      subFilters[i], hLen,
			      hOutputs[i], blockLen,
			      0, blockLen, 1, 0);
	}

	/* Interleave into output vector */
	interleave(hOutputs, blockLen, out, m);

	return true;
}

Synthesis::Synthesis(size_t m, size_t blockLen, size_t hLen)
	: ChannelizerBase(m, blockLen, hLen)
{
}

Synthesis::~Synthesis()
{
}
