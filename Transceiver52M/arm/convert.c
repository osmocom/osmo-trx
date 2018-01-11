/*
 * NEON type conversions
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
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <malloc.h>
#include <string.h>
#include "convert.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

void neon_convert_ps_si16_4n(short *, const float *, const float *, int);
void neon_convert_si16_ps_4n(float *, const short *, int);

/* 4*N 16-bit signed integer conversion with remainder */
static void neon_convert_si16_ps(float *out,
				 const short *in,
				 int len)
{
	int start = len / 4 * 4;

	neon_convert_si16_ps_4n(out, in, len >> 2);

	for (int i = 0; i < len % 4; i++)
		out[start + i] = (float) in[start + i];
}

/* 4*N 16-bit signed integer conversion with remainder */
static void neon_convert_ps_si16(short *out,
				 const float *in,
				 const float *scale,
				 int len)
{
	int start = len / 4 * 4;

	neon_convert_ps_si16_4n(out, in, scale, len >> 2);

	for (int i = 0; i < len % 4; i++)
		out[start + i] = (short) (in[start + i] * (*scale));
}

void convert_float_short(short *out, const float *in, float scale, int len)
{
#ifdef HAVE_NEON
        float q[4] = { scale, scale, scale, scale };

	if (len % 4)
		neon_convert_ps_si16(out, in, q, len);
	else
		neon_convert_ps_si16_4n(out, in, q, len >> 2);
#else
	base_convert_float_short(out, in, scale, len);
#endif
}

void convert_short_float(float *out, const short *in, int len)
{
#ifdef HAVE_NEON
	if (len % 4)
		neon_convert_si16_ps(out, in, len);
	else
		neon_convert_si16_ps_4n(out, in, len >> 2);
#else
	base_convert_short_float(out, in, len);
#endif
}
