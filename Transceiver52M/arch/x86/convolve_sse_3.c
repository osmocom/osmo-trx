/*
 * SSE Convolution
 * Copyright (C) 2012, 2013 Thomas Tsou <tom@tsou.cc>
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

#include <malloc.h>
#include <string.h>
#include <stdio.h>
#include "convolve_sse_3.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_SSE3
#include <xmmintrin.h>
#include <pmmintrin.h>

/* 4-tap SSE complex-real convolution */
__attribute__((xray_never_instrument))
void sse_conv_real4(const float *x, int x_len,
		    const float *h, int h_len,
		    float *y, int y_len,
		    int start, int len)
{
	/* NOTE: The parameter list of this function has to match the parameter
	 * list of _base_convolve_real() in convolve_base.c. This specific
	 * implementation, ignores some of the parameters of
	 * _base_convolve_complex(), which are: x_len, y_len. */

	__m128 m0, m1, m2, m3, m4, m5, m6, m7;

	const float *_x = &x[2 * (-(h_len - 1) + start)];

	/* Load (aligned) filter taps */
	m0 = _mm_load_ps(&h[0]);
	m1 = _mm_load_ps(&h[4]);
	m7 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));

	for (int i = 0; i < len; i++) {
		/* Load (unaligned) input data */
		m0 = _mm_loadu_ps(&_x[2 * i + 0]);
		m1 = _mm_loadu_ps(&_x[2 * i + 4]);
		m2 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
		m3 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));

		/* Quad multiply */
		m4 = _mm_mul_ps(m2, m7);
		m5 = _mm_mul_ps(m3, m7);

		/* Sum and store */
		m6 = _mm_hadd_ps(m4, m5);
		m0 = _mm_hadd_ps(m6, m6);

		_mm_store_ss(&y[2 * i + 0], m0);
		m0 = _mm_shuffle_ps(m0, m0, _MM_SHUFFLE(0, 3, 2, 1));
		_mm_store_ss(&y[2 * i + 1], m0);
	}
}

/* 8-tap SSE complex-real convolution */
__attribute__((xray_never_instrument))
void sse_conv_real8(const float *x, int x_len,
		    const float *h, int h_len,
		    float *y, int y_len,
		    int start, int len)
{
	/* See NOTE in sse_conv_real4() */

	__m128 m0, m1, m2, m3, m4, m5, m6, m7, m8, m9;

	const float *_x = &x[2 * (-(h_len - 1) + start)];

	/* Load (aligned) filter taps */
	m0 = _mm_load_ps(&h[0]);
	m1 = _mm_load_ps(&h[4]);
	m2 = _mm_load_ps(&h[8]);
	m3 = _mm_load_ps(&h[12]);

	m4 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
	m5 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));

	for (int i = 0; i < len; i++) {
		/* Load (unaligned) input data */
		m0 = _mm_loadu_ps(&_x[2 * i + 0]);
		m1 = _mm_loadu_ps(&_x[2 * i + 4]);
		m2 = _mm_loadu_ps(&_x[2 * i + 8]);
		m3 = _mm_loadu_ps(&_x[2 * i + 12]);

		m6 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
		m7 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));
		m8 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));
		m9 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(1, 3, 1, 3));

		/* Quad multiply */
		m6 = _mm_mul_ps(m6, m4);
		m7 = _mm_mul_ps(m7, m4);
		m8 = _mm_mul_ps(m8, m5);
		m9 = _mm_mul_ps(m9, m5);

		/* Sum and store */
		m6 = _mm_add_ps(m6, m8);
		m7 = _mm_add_ps(m7, m9);
		m6 = _mm_hadd_ps(m6, m7);
		m6 = _mm_hadd_ps(m6, m6);

		_mm_store_ss(&y[2 * i + 0], m6);
		m6 = _mm_shuffle_ps(m6, m6, _MM_SHUFFLE(0, 3, 2, 1));
		_mm_store_ss(&y[2 * i + 1], m6);
	}
}

/* 12-tap SSE complex-real convolution */
__attribute__((xray_never_instrument))
void sse_conv_real12(const float *x, int x_len,
		     const float *h, int h_len,
		     float *y, int y_len,
		     int start, int len)
{
	/* See NOTE in sse_conv_real4() */

	__m128 m0, m1, m2, m3, m4, m5, m6, m7;
	__m128 m8, m9, m10, m11, m12, m13, m14;

	const float *_x = &x[2 * (-(h_len - 1) + start)];

	/* Load (aligned) filter taps */
	m0 = _mm_load_ps(&h[0]);
	m1 = _mm_load_ps(&h[4]);
	m2 = _mm_load_ps(&h[8]);
	m3 = _mm_load_ps(&h[12]);
	m4 = _mm_load_ps(&h[16]);
	m5 = _mm_load_ps(&h[20]);

	m12 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
	m13 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));
	m14 = _mm_shuffle_ps(m4, m5, _MM_SHUFFLE(0, 2, 0, 2));

	for (int i = 0; i < len; i++) {
		/* Load (unaligned) input data */
		m0 = _mm_loadu_ps(&_x[2 * i + 0]);
		m1 = _mm_loadu_ps(&_x[2 * i + 4]);
		m2 = _mm_loadu_ps(&_x[2 * i + 8]);
		m3 = _mm_loadu_ps(&_x[2 * i + 12]);

		m4 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
		m5 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));
		m6 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));
		m7 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(1, 3, 1, 3));

		m0 = _mm_loadu_ps(&_x[2 * i + 16]);
		m1 = _mm_loadu_ps(&_x[2 * i + 20]);

		m8 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
		m9 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));

		/* Quad multiply */
		m0 = _mm_mul_ps(m4, m12);
		m1 = _mm_mul_ps(m5, m12);
		m2 = _mm_mul_ps(m6, m13);
		m3 = _mm_mul_ps(m7, m13);
		m4 = _mm_mul_ps(m8, m14);
		m5 = _mm_mul_ps(m9, m14);

		/* Sum and store */
		m8 = _mm_add_ps(m0, m2);
		m9 = _mm_add_ps(m1, m3);
		m10 = _mm_add_ps(m8, m4);
		m11 = _mm_add_ps(m9, m5);

		m2 = _mm_hadd_ps(m10, m11);
		m3 = _mm_hadd_ps(m2, m2);

		_mm_store_ss(&y[2 * i + 0], m3);
		m3 = _mm_shuffle_ps(m3, m3, _MM_SHUFFLE(0, 3, 2, 1));
		_mm_store_ss(&y[2 * i + 1], m3);
	}
}

/* 16-tap SSE complex-real convolution */
__attribute__((xray_never_instrument))
void sse_conv_real16(const float *x, int x_len,
		     const float *h, int h_len,
		     float *y, int y_len,
		     int start, int len)
{
	/* See NOTE in sse_conv_real4() */

	__m128 m0, m1, m2, m3, m4, m5, m6, m7;
	__m128 m8, m9, m10, m11, m12, m13, m14, m15;

	const float *_x = &x[2 * (-(h_len - 1) + start)];

	/* Load (aligned) filter taps */
	m0 = _mm_load_ps(&h[0]);
	m1 = _mm_load_ps(&h[4]);
	m2 = _mm_load_ps(&h[8]);
	m3 = _mm_load_ps(&h[12]);

	m4 = _mm_load_ps(&h[16]);
	m5 = _mm_load_ps(&h[20]);
	m6 = _mm_load_ps(&h[24]);
	m7 = _mm_load_ps(&h[28]);

	m12 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
	m13 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));
	m14 = _mm_shuffle_ps(m4, m5, _MM_SHUFFLE(0, 2, 0, 2));
	m15 = _mm_shuffle_ps(m6, m7, _MM_SHUFFLE(0, 2, 0, 2));

	for (int i = 0; i < len; i++) {
		/* Load (unaligned) input data */
		m0 = _mm_loadu_ps(&_x[2 * i + 0]);
		m1 = _mm_loadu_ps(&_x[2 * i + 4]);
		m2 = _mm_loadu_ps(&_x[2 * i + 8]);
		m3 = _mm_loadu_ps(&_x[2 * i + 12]);

		m4 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
		m5 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));
		m6 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));
		m7 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(1, 3, 1, 3));

		m0 = _mm_loadu_ps(&_x[2 * i + 16]);
		m1 = _mm_loadu_ps(&_x[2 * i + 20]);
		m2 = _mm_loadu_ps(&_x[2 * i + 24]);
		m3 = _mm_loadu_ps(&_x[2 * i + 28]);

		m8 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
		m9 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));
		m10 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));
		m11 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(1, 3, 1, 3));

		/* Quad multiply */
		m0 = _mm_mul_ps(m4, m12);
		m1 = _mm_mul_ps(m5, m12);
		m2 = _mm_mul_ps(m6, m13);
		m3 = _mm_mul_ps(m7, m13);

		m4 = _mm_mul_ps(m8, m14);
		m5 = _mm_mul_ps(m9, m14);
		m6 = _mm_mul_ps(m10, m15);
		m7 = _mm_mul_ps(m11, m15);

		/* Sum and store */
		m8 = _mm_add_ps(m0, m2);
		m9 = _mm_add_ps(m1, m3);
		m10 = _mm_add_ps(m4, m6);
		m11 = _mm_add_ps(m5, m7);

		m0 = _mm_add_ps(m8, m10);
		m1 = _mm_add_ps(m9, m11);
		m2 = _mm_hadd_ps(m0, m1);
		m3 = _mm_hadd_ps(m2, m2);

		_mm_store_ss(&y[2 * i + 0], m3);
		m3 = _mm_shuffle_ps(m3, m3, _MM_SHUFFLE(0, 3, 2, 1));
		_mm_store_ss(&y[2 * i + 1], m3);
	}
}

/* 20-tap SSE complex-real convolution */
__attribute__((xray_never_instrument))
void sse_conv_real20(const float *x, int x_len,
		     const float *h, int h_len,
		     float *y, int y_len,
		     int start, int len)
{
	/* See NOTE in sse_conv_real4() */

	__m128 m0, m1, m2, m3, m4, m5, m6, m7;
	__m128 m8, m9, m11, m12, m13, m14, m15;

	const float *_x = &x[2 * (-(h_len - 1) + start)];

	/* Load (aligned) filter taps */
	m0 = _mm_load_ps(&h[0]);
	m1 = _mm_load_ps(&h[4]);
	m2 = _mm_load_ps(&h[8]);
	m3 = _mm_load_ps(&h[12]);
	m4 = _mm_load_ps(&h[16]);
	m5 = _mm_load_ps(&h[20]);
	m6 = _mm_load_ps(&h[24]);
	m7 = _mm_load_ps(&h[28]);
	m8 = _mm_load_ps(&h[32]);
	m9 = _mm_load_ps(&h[36]);

	m11 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
	m12 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));
	m13 = _mm_shuffle_ps(m4, m5, _MM_SHUFFLE(0, 2, 0, 2));
	m14 = _mm_shuffle_ps(m6, m7, _MM_SHUFFLE(0, 2, 0, 2));
	m15 = _mm_shuffle_ps(m8, m9, _MM_SHUFFLE(0, 2, 0, 2));

	for (int i = 0; i < len; i++) {
		/* Multiply-accumulate first 12 taps */
		m0 = _mm_loadu_ps(&_x[2 * i + 0]);
		m1 = _mm_loadu_ps(&_x[2 * i + 4]);
		m2 = _mm_loadu_ps(&_x[2 * i + 8]);
		m3 = _mm_loadu_ps(&_x[2 * i + 12]);
		m4 = _mm_loadu_ps(&_x[2 * i + 16]);
		m5 = _mm_loadu_ps(&_x[2 * i + 20]);

		m6 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
		m7 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));
		m8 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));
		m9 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(1, 3, 1, 3));
		m0 = _mm_shuffle_ps(m4, m5, _MM_SHUFFLE(0, 2, 0, 2));
		m1 = _mm_shuffle_ps(m4, m5, _MM_SHUFFLE(1, 3, 1, 3));

		m2 = _mm_mul_ps(m6, m11);
		m3 = _mm_mul_ps(m7, m11);
		m4 = _mm_mul_ps(m8, m12);
		m5 = _mm_mul_ps(m9, m12);
		m6 = _mm_mul_ps(m0, m13);
		m7 = _mm_mul_ps(m1, m13);

		m0 = _mm_add_ps(m2, m4);
		m1 = _mm_add_ps(m3, m5);
		m8 = _mm_add_ps(m0, m6);
		m9 = _mm_add_ps(m1, m7);

		/* Multiply-accumulate last 8 taps */
		m0 = _mm_loadu_ps(&_x[2 * i + 24]);
		m1 = _mm_loadu_ps(&_x[2 * i + 28]);
		m2 = _mm_loadu_ps(&_x[2 * i + 32]);
		m3 = _mm_loadu_ps(&_x[2 * i + 36]);

		m4 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
		m5 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));
		m6 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));
		m7 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(1, 3, 1, 3));

		m0 = _mm_mul_ps(m4, m14);
		m1 = _mm_mul_ps(m5, m14);
		m2 = _mm_mul_ps(m6, m15);
		m3 = _mm_mul_ps(m7, m15);

		m4 = _mm_add_ps(m0, m2);
		m5 = _mm_add_ps(m1, m3);

		/* Final sum and store */
		m0 = _mm_add_ps(m8, m4);
		m1 = _mm_add_ps(m9, m5);
		m2 = _mm_hadd_ps(m0, m1);
		m3 = _mm_hadd_ps(m2, m2);

		_mm_store_ss(&y[2 * i + 0], m3);
		m3 = _mm_shuffle_ps(m3, m3, _MM_SHUFFLE(0, 3, 2, 1));
		_mm_store_ss(&y[2 * i + 1], m3);
	}
}

/* 4*N-tap SSE complex-real convolution */
__attribute__((xray_never_instrument))
void sse_conv_real4n(const float *x, int x_len,
		     const float *h, int h_len,
		     float *y, int y_len,
		     int start, int len)
{
	/* See NOTE in sse_conv_real4() */

	__m128 m0, m1, m2, m4, m5, m6, m7;

	const float *_x = &x[2 * (-(h_len - 1) + start)];

	for (int i = 0; i < len; i++) {
		/* Zero */
		m6 = _mm_setzero_ps();
		m7 = _mm_setzero_ps();

		for (int n = 0; n < h_len / 4; n++) {
			/* Load (aligned) filter taps */
			m0 = _mm_load_ps(&h[8 * n + 0]);
			m1 = _mm_load_ps(&h[8 * n + 4]);
			m2 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));

			/* Load (unaligned) input data */
			m0 = _mm_loadu_ps(&_x[2 * i + 8 * n + 0]);
			m1 = _mm_loadu_ps(&_x[2 * i + 8 * n + 4]);
			m4 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
			m5 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));

			/* Quad multiply */
			m0 = _mm_mul_ps(m2, m4);
			m1 = _mm_mul_ps(m2, m5);

			/* Accumulate */
			m6 = _mm_add_ps(m6, m0);
			m7 = _mm_add_ps(m7, m1);
		}

		m0 = _mm_hadd_ps(m6, m7);
		m0 = _mm_hadd_ps(m0, m0);

		_mm_store_ss(&y[2 * i + 0], m0);
		m0 = _mm_shuffle_ps(m0, m0, _MM_SHUFFLE(0, 3, 2, 1));
		_mm_store_ss(&y[2 * i + 1], m0);
	}
}

/* 4*N-tap SSE complex-complex convolution */
__attribute__((xray_never_instrument))
void sse_conv_cmplx_4n(const float *x, int x_len,
		       const float *h, int h_len,
		       float *y, int y_len,
		       int start, int len)
{
	/* NOTE: The parameter list of this function has to match the parameter
	 * list of _base_convolve_complex() in convolve_base.c. This specific
	 * implementation, ignores some of the parameters of
	 * _base_convolve_complex(), which are: x_len, y_len. */

	__m128 m0, m1, m2, m3, m4, m5, m6, m7;

	const float *_x = &x[2 * (-(h_len - 1) + start)];

	for (int i = 0; i < len; i++) {
		/* Zero */
		m6 = _mm_setzero_ps();
		m7 = _mm_setzero_ps();

		for (int n = 0; n < h_len / 4; n++) {
			/* Load (aligned) filter taps */
			m0 = _mm_load_ps(&h[8 * n + 0]);
			m1 = _mm_load_ps(&h[8 * n + 4]);
			m2 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
			m3 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));

			/* Load (unaligned) input data */
			m0 = _mm_loadu_ps(&_x[2 * i + 8 * n + 0]);
			m1 = _mm_loadu_ps(&_x[2 * i + 8 * n + 4]);
			m4 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
			m5 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));

			/* Quad multiply */
			m0 = _mm_mul_ps(m2, m4);
			m1 = _mm_mul_ps(m3, m5);

			m2 = _mm_mul_ps(m2, m5);
			m3 = _mm_mul_ps(m3, m4);

			/* Sum */
			m0 = _mm_sub_ps(m0, m1);
			m2 = _mm_add_ps(m2, m3);

			/* Accumulate */
			m6 = _mm_add_ps(m6, m0);
			m7 = _mm_add_ps(m7, m2);
		}

		m0 = _mm_hadd_ps(m6, m7);
		m0 = _mm_hadd_ps(m0, m0);

		_mm_store_ss(&y[2 * i + 0], m0);
		m0 = _mm_shuffle_ps(m0, m0, _MM_SHUFFLE(0, 3, 2, 1));
		_mm_store_ss(&y[2 * i + 1], m0);
	}
}

/* 8*N-tap SSE complex-complex convolution */
__attribute__((xray_never_instrument))
void sse_conv_cmplx_8n(const float *x, int x_len,
		       const float *h, int h_len,
		       float *y, int y_len,
		       int start, int len)
{
	/* See NOTE in sse_conv_cmplx_4n() */

	__m128 m0, m1, m2, m3, m4, m5, m6, m7;
	__m128 m8, m9, m10, m11, m12, m13, m14, m15;

	const float *_x = &x[2 * (-(h_len - 1) + start)];

	for (int i = 0; i < len; i++) {
		/* Zero */
		m12 = _mm_setzero_ps();
		m13 = _mm_setzero_ps();
		m14 = _mm_setzero_ps();
		m15 = _mm_setzero_ps();

		for (int n = 0; n < h_len / 8; n++) {
			/* Load (aligned) filter taps */
			m0 = _mm_load_ps(&h[16 * n + 0]);
			m1 = _mm_load_ps(&h[16 * n + 4]);
			m2 = _mm_load_ps(&h[16 * n + 8]);
			m3 = _mm_load_ps(&h[16 * n + 12]);

			m4 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
			m5 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));
			m6 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));
			m7 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(1, 3, 1, 3));

			/* Load (unaligned) input data */
			m0 = _mm_loadu_ps(&_x[2 * i + 16 * n + 0]);
			m1 = _mm_loadu_ps(&_x[2 * i + 16 * n + 4]);
			m2 = _mm_loadu_ps(&_x[2 * i + 16 * n + 8]);
			m3 = _mm_loadu_ps(&_x[2 * i + 16 * n + 12]);

			m8 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(0, 2, 0, 2));
			m9 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(1, 3, 1, 3));
			m10 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(0, 2, 0, 2));
			m11 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(1, 3, 1, 3));

			/* Quad multiply */
			m0 = _mm_mul_ps(m4, m8);
			m1 = _mm_mul_ps(m5, m9);
			m2 = _mm_mul_ps(m6, m10);
			m3 = _mm_mul_ps(m7, m11);

			m4 = _mm_mul_ps(m4, m9);
			m5 = _mm_mul_ps(m5, m8);
			m6 = _mm_mul_ps(m6, m11);
			m7 = _mm_mul_ps(m7, m10);

			/* Sum */
			m0 = _mm_sub_ps(m0, m1);
			m2 = _mm_sub_ps(m2, m3);
			m4 = _mm_add_ps(m4, m5);
			m6 = _mm_add_ps(m6, m7);

			/* Accumulate */
			m12 = _mm_add_ps(m12, m0);
			m13 = _mm_add_ps(m13, m2);
			m14 = _mm_add_ps(m14, m4);
			m15 = _mm_add_ps(m15, m6);
		}

		m0 = _mm_add_ps(m12, m13);
		m1 = _mm_add_ps(m14, m15);
		m2 = _mm_hadd_ps(m0, m1);
		m2 = _mm_hadd_ps(m2, m2);

		_mm_store_ss(&y[2 * i + 0], m2);
		m2 = _mm_shuffle_ps(m2, m2, _MM_SHUFFLE(0, 3, 2, 1));
		_mm_store_ss(&y[2 * i + 1], m2);
	}
}
#endif
