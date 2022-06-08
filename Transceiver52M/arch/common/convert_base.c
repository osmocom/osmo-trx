/*
 * Conversion
 * Copyright (C) 2012, 2013 Thomas Tsou <tom@tsou.cc>
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

#include "convert.h"

__attribute__((xray_never_instrument))
void base_convert_float_short(short *out, const float *in,
			      float scale, int len)
{
	for (int i = 0; i < len; i++)
		out[i] = in[i] * scale;
}

__attribute__((xray_never_instrument))
void base_convert_short_float(float *out, const short *in, int len)
{
	for (int i = 0; i < len; i++)
		out[i] = in[i];
}
