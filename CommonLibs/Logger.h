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

// (pat) WARNING is stupidly defined in /usr/local/include/osipparser2/osip_const.h.
// This must be outside the #ifndef LOGGER_H to fix it as long as Logger.h included after the above file.
#ifdef WARNING
#undef WARNING
#endif

#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdio.h>
#include <sstream>
#include <string>

extern int config_log_level;

#define LOG_EMERG       0       /* system is unusable */
#define LOG_ALERT       1       /* action must be taken immediately */
#define LOG_CRIT        2       /* critical conditions */
#define LOG_ERR         3       /* error conditions */
#define LOG_WARNING     4       /* warning conditions */
#define LOG_NOTICE      5       /* normal but significant condition */
#define LOG_INFO        6       /* informational */
#define LOG_DEBUG       7       /* debug-level messages */

#define _LOG(level) \
	Log(LOG_##level).get() << pthread_self() \
	<< timestr() << " " __FILE__  ":"  << __LINE__ << ":" << __FUNCTION__ << ": "

#define IS_LOG_LEVEL(wLevel) (config_log_level>=LOG_##wLevel)

#ifdef NDEBUG
#define LOG(wLevel) \
	if (LOG_##wLevel!=LOG_DEBUG && IS_LOG_LEVEL(wLevel)) _LOG(wLevel)
#else
#define LOG(wLevel) \
	if (IS_LOG_LEVEL(wLevel)) _LOG(wLevel)
#endif

/**
	A C++ stream-based thread-safe logger.
	This object is NOT the global logger;
	every log record is an object of this class.
*/
class Log {

	public:

	protected:

	std::ostringstream mStream;		///< This is where we buffer up the log entry.
	int mPriority;					///< Priority of current report.

	public:

	Log(int wPriority)
		:mPriority(wPriority)
	{ }

	// Most of the work is in the destructor.
	/** The destructor actually generates the log entry. */
	~Log();

	std::ostringstream& get();
};

const std::string timestr();		// A timestamp to print in messages.
std::ostream& operator<<(std::ostream& os, std::ostringstream& ss);

/**@ Global control and initialization of the logging system. */
//@{
/** Initialize the global logging system. */
void gLogInit(const char* level=NULL, char* fn=NULL);
//@}


#endif

// vim: ts=4 sw=4
