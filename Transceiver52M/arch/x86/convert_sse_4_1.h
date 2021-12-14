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
 */

#pragma once

/* 16*N 16-bit signed integer converted to single precision floats */
void _sse_convert_si16_ps_16n(float *restrict out,
			      const short *restrict in, int len);

/* 16*N 16-bit signed integer conversion with remainder */
void _sse_convert_si16_ps(float *restrict out,
			  const short *restrict in, int len);
