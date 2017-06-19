/*
 * Rational Sample Rate Conversion
 * Copyright (C) 2012, 2013  Thomas Tsou <tom@tsou.cc>
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

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <malloc.h>
#include <iostream>
#include <algorithm>

#include "Resampler.h"

extern "C" {
#include "convolve.h"
}

#ifndef M_PI
#define M_PI			3.14159265358979323846264338327f
#endif

#define MAX_OUTPUT_LEN		4096

using namespace std;

static float sinc(float x)
{
	if (x == 0.0)
		return 0.9999999999;

	return sin(M_PI * x) / (M_PI * x);
}

void Resampler::initFilters(float bw)
{
	float cutoff;
	float sum = 0.0f, scale = 0.0f;

	/* 
	 * Allocate partition filters and the temporary prototype filter
	 * according to numerator of the rational rate. Coefficients are
	 * real only and must be 16-byte memory aligned for SSE usage.
	 */
	auto proto = vector<float>(p * filt_len);
	for (auto &part : partitions)
		part = (complex<float> *) memalign(16, filt_len * sizeof(complex<float>));

	/* 
	 * Generate the prototype filter with a Blackman-harris window.
	 * Scale coefficients with DC filter gain set to unity divided
	 * by the number of filter partitions. 
	 */
	float a0 = 0.35875;
	float a1 = 0.48829;
	float a2 = 0.14128;
	float a3 = 0.01168;

	if (p > q)
		cutoff = (float) p;
	else
		cutoff = (float) q;

	float midpt = (proto.size() - 1) / 2.0;
	for (size_t i = 0; i < proto.size(); i++) {
		proto[i] = sinc(((float) i - midpt) / cutoff * bw);
		proto[i] *= a0 -
			    a1 * cos(2 * M_PI * i / (proto.size() - 1)) +
			    a2 * cos(4 * M_PI * i / (proto.size() - 1)) -
			    a3 * cos(6 * M_PI * i / (proto.size() - 1));
		sum += proto[i];
	}
	scale = p / sum;

	/* Populate filter partitions from the prototype filter */
	for (size_t i = 0; i < filt_len; i++) {
		for (size_t n = 0; n < p; n++)
			partitions[n][i] = complex<float>(proto[i * p + n] * scale);
	}

	/* Store filter taps in reverse */
	for (auto &part : partitions)
		reverse(&part[0], &part[filt_len]);
}

static bool check_vec_len(int in_len, int out_len, int p, int q)
{
	if (in_len % q) {
		std::cerr << "Invalid input length " << in_len
			  <<  " is not multiple of " << q << std::endl;
		return false;
	}

	if (out_len % p) {
		std::cerr << "Invalid output length " << out_len
			  <<  " is not multiple of " << p << std::endl;
		return false;
	}

	if ((in_len / q) != (out_len / p)) {
		std::cerr << "Input/output block length mismatch" << std::endl;
		std::cerr << "P = " << p << ", Q = " << q << std::endl;
		std::cerr << "Input len: " << in_len << std::endl;
		std::cerr << "Output len: " << out_len << std::endl;
		return false;
	}

	if (out_len > MAX_OUTPUT_LEN) {
		std::cerr << "Block length of " << out_len
			  << " exceeds max of " << MAX_OUTPUT_LEN << std::endl;
		return false;
	}

	return true;
}

int Resampler::rotate(const float *in, size_t in_len, float *out, size_t out_len)
{
	int n, path;

	if (!check_vec_len(in_len, out_len, p, q))
		return -1;

	/* Generate output from precomputed input/output paths */
	for (size_t i = 0; i < out_len; i++) {
		n = in_index[i]; 
		path = out_path[i]; 

		convolve_real(in, in_len,
			      reinterpret_cast<float *>(partitions[path]),
			      filt_len, &out[2 * i], out_len - i,
			      n, 1, 1, 0);
	}

	return out_len;
}

bool Resampler::init(float bw)
{
	if (p == 0 || q == 0 || filt_len == 0) return false;

	/* Filterbank filter internals */
	initFilters(bw);

	/* Precompute filterbank paths */
	int i = 0;
	for (auto &index : in_index)
		index = (q * i++) / p;
	i = 0;
	for (auto &path : out_path)
		path = (q * i++) % p;

	return true;
}

size_t Resampler::len()
{
	return filt_len;
}

Resampler::Resampler(size_t p, size_t q, size_t filt_len)
	: in_index(MAX_OUTPUT_LEN), out_path(MAX_OUTPUT_LEN), partitions(p)
{
	this->p = p;
	this->q = q;
	this->filt_len = filt_len;
}

Resampler::~Resampler()
{
	for (auto &part : partitions)
		free(part);
}
