/*
 * SSE type conversions
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

#include <malloc.h>
#include <string.h>
#include "convert_sse_4_1.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_SSE4_1
#include <smmintrin.h>

/* 16*N 16-bit signed integer converted to single precision floats */
void _sse_convert_si16_ps_16n(float *restrict out,
			      const short *restrict in, int len)
{
	__m128i m0, m1, m2, m3, m4, m5;
	__m128 m6, m7, m8, m9;

	for (int i = 0; i < len / 16; i++) {
		/* Load (unaligned) packed floats */
		m0 = _mm_loadu_si128((__m128i *) & in[16 * i + 0]);
		m1 = _mm_loadu_si128((__m128i *) & in[16 * i + 8]);

		/* Unpack */
		m2 = _mm_cvtepi16_epi32(m0);
		m4 = _mm_cvtepi16_epi32(m1);
		m0 = _mm_shuffle_epi32(m0, _MM_SHUFFLE(1, 0, 3, 2));
		m1 = _mm_shuffle_epi32(m1, _MM_SHUFFLE(1, 0, 3, 2));
		m3 = _mm_cvtepi16_epi32(m0);
		m5 = _mm_cvtepi16_epi32(m1);

		/* Convert */
		m6 = _mm_cvtepi32_ps(m2);
		m7 = _mm_cvtepi32_ps(m3);
		m8 = _mm_cvtepi32_ps(m4);
		m9 = _mm_cvtepi32_ps(m5);

		/* Store */
		_mm_storeu_ps(&out[16 * i + 0], m6);
		_mm_storeu_ps(&out[16 * i + 4], m7);
		_mm_storeu_ps(&out[16 * i + 8], m8);
		_mm_storeu_ps(&out[16 * i + 12], m9);
	}
}

/* 16*N 16-bit signed integer conversion with remainder */
void _sse_convert_si16_ps(float *restrict out,
			  const short *restrict in, int len)
{
	int start = len / 16 * 16;

	_sse_convert_si16_ps_16n(out, in, len);

	for (int i = 0; i < len % 16; i++)
		out[start + i] = in[start + i];
}

#endif
