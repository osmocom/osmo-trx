/*
 * Copyright 2018 sysmocom - s.f.m.c. GmbH
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

#include <vector>
#include <string>
#include <sstream>
#include <cstring>

std::vector<std::string> comma_delimited_to_vector(const char* opt)
{
	std::string str = std::string(opt);
	std::vector<std::string> result;
	std::stringstream ss(str);

	while( ss.good() )
	{
	    std::string substr;
	    getline(ss, substr, ',');
	    result.push_back(std::move(substr));
	}
	return result;
}

char *strerror_buf(int err, char *buf, size_t buf_size)
{
	if (buf == nullptr || buf_size == 0)
		return nullptr;

	char *err_str = buf;
#if defined(__GLIBC__) && defined(_GNU_SOURCE)
	err_str = strerror_r(err, buf, buf_size);
#else
	if (!strerror_r(err, buf, buf_size)) {
		snprintf(buf, buf_size, "Unknown error %d", err);
	}
#endif
	return err_str;
}
