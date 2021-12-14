/*
 * Rational Sample Rate Conversion
 * Copyright (C) 2012, 2013  Thomas Tsou <tom@tsou.cc>
 *
 * SPDX-License-Identifier: LGPL-2.1+
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
 */

#ifndef _RESAMPLER_H_
#define _RESAMPLER_H_

#include <vector>
#include <complex>

class Resampler {
public:
	/* Constructor for rational sample rate conversion
	 *   @param p numerator of resampling ratio
	 *   @param q denominator of resampling ratio
	 *   @param filt_len length of each polyphase subfilter
	 */
	Resampler(size_t p, size_t q, size_t filt_len = 16);
	~Resampler();

	/* Initialize resampler filterbank.
	 *   @param bw bandwidth factor on filter generation (pre-window)
	 *   @return false on error, zero otherwise
	 *
	 * Automatic setting is to compute the filter to prevent aliasing with
	 * a Blackman-Harris window. Adjustment is made through a bandwidth
	 * factor to shift the cutoff and/or the constituent filter lengths.
	 * Calculation of specific rolloff factors or 3-dB cutoff points is
	 * left as an excersize for the reader.
	 */
	bool init(float bw = 1.0f);

	/* Rotate "commutator" and drive samples through filterbank
	 *   @param in continuous buffer of input complex float values
	 *   @param in_len input buffer length
	 *   @param out continuous buffer of output complex float values
	 *   @param out_len output buffer length
	 *   @return number of samples outputted, negative on error
         *
	 * Input and output vector lengths must of be equal multiples of the
	 * rational conversion rate denominator and numerator respectively.
	 */
	int rotate(const float *in, size_t in_len, float *out, size_t out_len);

	/* Get filter length
	 *   @return number of taps in each filter partition
	 */
	size_t len();

private:
	size_t p;
	size_t q;
	size_t filt_len;
	std::vector<size_t> in_index;
	std::vector<size_t> out_path;
	std::vector<std::complex<float> *> partitions;

	void initFilters(float bw);
};

#endif /* _RESAMPLER_H_ */
