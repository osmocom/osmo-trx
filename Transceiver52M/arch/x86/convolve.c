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
#include "convolve.h"
#include "convolve_sse_3.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* Architecture dependent function pointers */
struct convolve_cpu_context {
	void (*conv_cmplx_4n) (const float *, int, const float *, int, float *,
			       int, int, int);
	void (*conv_cmplx_8n) (const float *, int, const float *, int, float *,
			       int, int, int);
	void (*conv_cmplx) (const float *, int, const float *, int, float *,
			    int, int, int);
	void (*conv_real4) (const float *, int, const float *, int, float *,
			    int, int, int);
	void (*conv_real8) (const float *, int, const float *, int, float *,
			    int, int, int);
	void (*conv_real12) (const float *, int, const float *, int, float *,
			     int, int, int);
	void (*conv_real16) (const float *, int, const float *, int, float *,
			     int, int, int);
	void (*conv_real20) (const float *, int, const float *, int, float *,
			     int, int, int);
	void (*conv_real4n) (const float *, int, const float *, int, float *,
			     int, int, int);
	void (*conv_real) (const float *, int, const float *, int, float *, int,
			   int, int);
};
static struct convolve_cpu_context c;

/* Forward declarations from base implementation */
int _base_convolve_real(const float *x, int x_len,
			const float *h, int h_len,
			float *y, int y_len,
			int start, int len);

int _base_convolve_complex(const float *x, int x_len,
			   const float *h, int h_len,
			   float *y, int y_len,
			   int start, int len);

int bounds_check(int x_len, int h_len, int y_len,
		 int start, int len);

/* API: Initialize convolve module */
void convolve_init(void)
{
	c.conv_cmplx_4n = (void *)_base_convolve_complex;
	c.conv_cmplx_8n = (void *)_base_convolve_complex;
	c.conv_cmplx = (void *)_base_convolve_complex;
	c.conv_real4 = (void *)_base_convolve_real;
	c.conv_real8 = (void *)_base_convolve_real;
	c.conv_real12 = (void *)_base_convolve_real;
	c.conv_real16 = (void *)_base_convolve_real;
	c.conv_real20 = (void *)_base_convolve_real;
	c.conv_real4n = (void *)_base_convolve_real;
	c.conv_real = (void *)_base_convolve_real;

#if defined(HAVE_SSE3) && defined(HAVE___BUILTIN_CPU_SUPPORTS)
	if (__builtin_cpu_supports("sse3")) {
		c.conv_cmplx_4n = sse_conv_cmplx_4n;
		c.conv_cmplx_8n = sse_conv_cmplx_8n;
		c.conv_real4 = sse_conv_real4;
		c.conv_real8 = sse_conv_real8;
		c.conv_real12 = sse_conv_real12;
		c.conv_real16 = sse_conv_real16;
		c.conv_real20 = sse_conv_real20;
		c.conv_real4n = sse_conv_real4n;
	}
#endif
}

/* API: Aligned complex-real */
__attribute__((xray_never_instrument))
int convolve_real(const float *x, int x_len,
		  const float *h, int h_len,
		  float *y, int y_len, int start, int len)
{
#ifndef __OPTIMIZE__
	if (bounds_check(x_len, h_len, y_len, start, len) < 0)
		return -1;
#endif
	memset(y, 0, len * 2 * sizeof(float));

	switch (h_len) {
	case 4:
		c.conv_real4(x, x_len, h, h_len, y, y_len, start, len);
		break;
	case 8:
		c.conv_real8(x, x_len, h, h_len, y, y_len, start, len);
		break;
	case 12:
		c.conv_real12(x, x_len, h, h_len, y, y_len, start, len);
		break;
	case 16:
		c.conv_real16(x, x_len, h, h_len, y, y_len, start, len);
		break;
	case 20:
		c.conv_real20(x, x_len, h, h_len, y, y_len, start, len);
		break;
	default:
		if (!(h_len % 4))
			c.conv_real4n(x, x_len, h, h_len, y, y_len,
				      start, len);
		else
			c.conv_real(x, x_len, h, h_len, y, y_len, start,
				    len);
	}

	return len;
}

/* API: Aligned complex-complex */
__attribute__((xray_never_instrument))
int convolve_complex(const float *x, int x_len,
		     const float *h, int h_len,
		     float *y, int y_len,
		     int start, int len)
{
#ifndef __OPTIMIZE__
	if (bounds_check(x_len, h_len, y_len, start, len) < 0)
		return -1;
#endif
	memset(y, 0, len * 2 * sizeof(float));

	if (!(h_len % 8))
		c.conv_cmplx_8n(x, x_len, h, h_len, y, y_len, start, len);
	else if (!(h_len % 4))
		c.conv_cmplx_4n(x, x_len, h, h_len, y, y_len, start, len);
	else
		c.conv_cmplx(x, x_len, h, h_len, y, y_len, start, len);

	return len;
}
