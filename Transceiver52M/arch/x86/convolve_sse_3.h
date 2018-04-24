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
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once

/* 4-tap SSE complex-real convolution */
void sse_conv_real4(const float *x, int x_len,
		    const float *h, int h_len,
		    float *y, int y_len,
		    int start, int len, int step, int offset);

/* 8-tap SSE complex-real convolution */
void sse_conv_real8(const float *x, int x_len,
		    const float *h, int h_len,
		    float *y, int y_len,
		    int start, int len, int step, int offset);

/* 12-tap SSE complex-real convolution */
void sse_conv_real12(const float *x, int x_len,
		     const float *h, int h_len,
		     float *y, int y_len,
		     int start, int len, int step, int offset);

/* 16-tap SSE complex-real convolution */
void sse_conv_real16(const float *x, int x_len,
		     const float *h, int h_len,
		     float *y, int y_len,
		     int start, int len, int step, int offset);

/* 20-tap SSE complex-real convolution */
void sse_conv_real20(const float *x, int x_len,
		     const float *h, int h_len,
		     float *y, int y_len,
		     int start, int len, int step, int offset);

/* 4*N-tap SSE complex-real convolution */
void sse_conv_real4n(const float *x, int x_len,
		     const float *h, int h_len,
		     float *y, int y_len,
		     int start, int len, int step, int offset);

/* 4*N-tap SSE complex-complex convolution */
void sse_conv_cmplx_4n(const float *x, int x_len,
		       const float *h, int h_len,
		       float *y, int y_len,
		       int start, int len, int step, int offset);

/* 8*N-tap SSE complex-complex convolution */
void sse_conv_cmplx_8n(const float *x, int x_len,
		       const float *h, int h_len,
		       float *y, int y_len,
		       int start, int len, int step, int offset);
