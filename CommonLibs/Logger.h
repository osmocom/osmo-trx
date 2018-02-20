/*
* Copyright 2009, 2010 Free Software Foundation, Inc.
* Copyright 2010 Kestrel Signal Processing, Inc.
*
* This software is distributed under the terms of the GNU Affero Public License.
* See the COPYING file in the main directory for details.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdio.h>
#include <sstream>
#include <string>

extern "C" {
#include <osmocom/core/logging.h>
#include "debug.h"
}

/* Translation for old log statements */
#ifndef LOGL_ALERT
#define LOGL_ALERT LOGL_FATAL
#endif
#ifndef LOGL_ERR
#define LOGL_ERR LOGL_ERROR
#endif
#ifndef LOGL_WARNING
#define LOGL_WARNING LOGL_NOTICE
#endif

#define LOG(level) \
	Log(DMAIN, LOGL_##level).get() <<  "[tid=" << pthread_self() << "] "

#define LOGC(category, level) \
	Log(category, LOGL_##level).get() <<  "[tid=" << pthread_self() << "] "

/**
	A C++ stream-based thread-safe logger.
	This object is NOT the global logger;
	every log record is an object of this class.
*/
class Log {

	public:

	protected:

	std::ostringstream mStream;	///< This is where we buffer up the log entry.
	int mCategory;			///< Priority of current report.
	int mPriority;			///< Category of current report.

	public:

	Log(int wCategory, int wPriority)
		: mCategory(wCategory), mPriority(wPriority)
	{ }

	// Most of the work is in the destructor.
	/** The destructor actually generates the log entry. */
	~Log();

	std::ostringstream& get();
};

std::ostream& operator<<(std::ostream& os, std::ostringstream& ss);

#endif

// vim: ts=4 sw=4
