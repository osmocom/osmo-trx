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
#include "convert.h"
#include "convert_sse_3.h"
#include "convert_sse_4_1.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* Architecture dependant function pointers */
struct convert_cpu_context {
	void (*convert_si16_ps_16n) (float *, const short *, int);
	void (*convert_si16_ps) (float *, const short *, int);
	void (*convert_scale_ps_si16_16n)(short *, const float *, float, int);
	void (*convert_scale_ps_si16_8n)(short *, const float *, float, int);
	void (*convert_scale_ps_si16)(short *, const float *, float, int);
};

static struct convert_cpu_context c;

void convert_init(void)
{
	c.convert_scale_ps_si16_16n = base_convert_float_short;
	c.convert_scale_ps_si16_8n = base_convert_float_short;
	c.convert_scale_ps_si16 = base_convert_float_short;
	c.convert_si16_ps_16n = base_convert_short_float;
	c.convert_si16_ps = base_convert_short_float;

#ifdef HAVE_SSE4_1
	if (__builtin_cpu_supports("sse4.1")) {
		c.convert_si16_ps_16n = &_sse_convert_si16_ps_16n;
		c.convert_si16_ps = &_sse_convert_si16_ps;
	}
#endif

#ifdef HAVE_SSE3
	if (__builtin_cpu_supports("sse3")) {
		c.convert_scale_ps_si16_16n = _sse_convert_scale_ps_si16_16n;
		c.convert_scale_ps_si16_8n = _sse_convert_scale_ps_si16_8n;
		c.convert_scale_ps_si16 = _sse_convert_scale_ps_si16;
	}
#endif
}

void convert_float_short(short *out, const float *in, float scale, int len)
{
	if (!(len % 16))
		c.convert_scale_ps_si16_16n(out, in, scale, len);
	else if (!(len % 8))
		c.convert_scale_ps_si16_8n(out, in, scale, len);
	else
		c.convert_scale_ps_si16(out, in, scale, len);
}

void convert_short_float(float *out, const short *in, int len)
{
	if (!(len % 16))
		c.convert_si16_ps_16n(out, in, len);
	else
		c.convert_si16_ps(out, in, len);
}
