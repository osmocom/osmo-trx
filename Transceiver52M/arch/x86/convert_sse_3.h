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

#pragma once

/* 8*N single precision floats scaled and converted to 16-bit signed integer */
void _sse_convert_scale_ps_si16_8n(short *restrict out,
				   const float *restrict in,
				   float scale, int len);

/* 8*N single precision floats scaled and converted with remainder */
void _sse_convert_scale_ps_si16(short *restrict out,
				const float *restrict in, float scale, int len);

/* 16*N single precision floats scaled and converted to 16-bit signed integer */
void _sse_convert_scale_ps_si16_16n(short *restrict out,
				    const float *restrict in,
				    float scale, int len);
