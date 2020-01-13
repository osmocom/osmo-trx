/*
 * Sample Buffer - Allows reading and writing of timed samples
 *
 * Copyright 2010,2011 Free Software Foundation, Inc.
 * Copyright (C) 2015 Ettus Research LLC
 * Copyright 2019 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 *
 * Author: Tom Tsou <tom.tsou@ettus.com>
 *
 * SPDX-License-Identifier: AGPL-3.0+
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 */

#pragma once

#include <unistd.h>

#include "radioDevice.h"

/*
    Sample Buffer - Allows reading and writing of timed samples using osmo-trx
                    timestamps. Time conversions are handled
                    internally or accessible through the static convert calls.
*/
class smpl_buf {
public:
	/** Sample buffer constructor
	    @param len number of 32-bit samples the buffer should hold
	    @param timestamp
	*/
	smpl_buf(size_t len);
	~smpl_buf();

	/** Query number of samples available for reading
	    @param timestamp time of first sample
	    @return number of available samples or error
	*/
	ssize_t avail_smpls(TIMESTAMP timestamp) const;

	/** Read and write
	    @param buf pointer to buffer
	    @param len number of samples desired to read or write
	    @param timestamp time of first stample
	    @return number of actual samples read or written or error
	*/
	ssize_t read(void *buf, size_t len, TIMESTAMP timestamp);
	ssize_t write(void *buf, size_t len, TIMESTAMP timestamp);

	/** Buffer status string
	    @return a formatted string describing internal buffer state
	*/
	std::string str_status(TIMESTAMP timestamp) const;

	/** Formatted error string
	    @param code an error code
	    @return a formatted error string
	*/
	static std::string str_code(int code);

	enum err_code {
		ERROR_TIMESTAMP = -1,
		ERROR_READ = -2,
		ERROR_WRITE = -3,
		ERROR_OVERFLOW = -4
	};

private:
	uint32_t *data;
	size_t buf_len;

	TIMESTAMP time_start;
	TIMESTAMP time_end;

	size_t data_start;
	size_t data_end;
};
