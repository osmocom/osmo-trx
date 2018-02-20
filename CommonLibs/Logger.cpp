/*
* Copyright 2009, 2010 Free Software Foundation, Inc.
* Copyright 2010 Kestrel Signal Processing, Inc.
* Copyright 2011, 2012 Range Networks, Inc.
*
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

#include <string.h>
#include <cstdio>
#include <fstream>
#include <string>
#include <stdarg.h>
#include <sys/time.h>	// For gettimeofday

#include "Logger.h"
#include "Threads.h"	// pat added

using namespace std;

// Switches to enable/disable logging targets
bool gLogToConsole = true;
bool gLogToSyslog = false;
FILE *gLogToFile = NULL;
Mutex gLogToLock;

// Global log level threshold:
int config_log_level;

/** Names of the logging levels. */
const char *levelNames[] = {
	"EMERG", "ALERT", "CRIT", "ERR", "WARNING", "NOTICE", "INFO", "DEBUG"
};
int numLevels = 8;


int levelStringToInt(const string& name)
{
	// Reverse search, since the numerically larger levels are more common.
	for (int i=numLevels-1; i>=0; i--) {
		if (name == levelNames[i]) return i;
	}

	// Common substitutions.
	if (name=="INFORMATION") return 6;
	if (name=="WARN") return 4;
	if (name=="ERROR") return 3;
	if (name=="CRITICAL") return 2;
	if (name=="EMERGENCY") return 0;

	// Unknown level.
	return -1;
}

static std::string format(const char *fmt, ...)
{
	va_list ap;
	char buf[300];
	va_start(ap,fmt);
	int n = vsnprintf(buf,300,fmt,ap);
	va_end(ap);
	if (n >= (300-4)) { strcpy(&buf[(300-4)],"..."); }
	return std::string(buf);
}

const std::string timestr()
{
	struct timeval tv;
	struct tm tm;
	gettimeofday(&tv,NULL);
	localtime_r(&tv.tv_sec,&tm);
	unsigned tenths = tv.tv_usec / 100000;	// Rounding down is ok.
	return format(" %02d:%02d:%02d.%1d",tm.tm_hour,tm.tm_min,tm.tm_sec,tenths);
}

std::ostream& operator<<(std::ostream& os, std::ostringstream& ss)
{
	return os << ss.str();
}

Log::~Log()
{
	if (mDummyInit) return;
	// Anything at or above LOG_CRIT is an "alarm".
	if (mPriority <= LOG_ERR) {
		cerr << mStream.str() << endl;
	}
	// Current logging level was already checked by the macro. So just log.
	// Log to syslog
	if (gLogToSyslog) {
		syslog(mPriority, "%s", mStream.str().c_str());
	}
	// Log to file and console
	if (gLogToConsole||gLogToFile) {
		int mlen = mStream.str().size();
		int neednl = (mlen==0 || mStream.str()[mlen-1] != '\n');
		ScopedLock lock(gLogToLock);
		if (gLogToConsole) {
			// The COUT() macro prevents messages from stomping each other but adds uninteresting thread numbers,
			// so just use std::cout.
			std::cout << mStream.str();
			if (neednl) std::cout<<"\n";
		}
		if (gLogToFile) {
			fputs(mStream.str().c_str(),gLogToFile);
			if (neednl) {fputc('\n',gLogToFile);}
			fflush(gLogToFile);
		}
	}
}


Log::Log(const char* name, const char* level, int facility)
{
	mDummyInit = true;
	gLogInit(name, level, facility);
}


ostringstream& Log::get()
{
	assert(mPriority<numLevels);
	mStream << levelNames[mPriority] <<  ' ';
	return mStream;
}



void gLogInit(const char* name, const char* level, int facility, char* fn)
{
	// Set the level if one has been specified.
	if (level)
		config_log_level = levelStringToInt(level);

	// Both the transceiver and OpenBTS use this same facility, but only OpenBTS/OpenNodeB may use this log file:
	if (!gLogToFile && fn) {
		gLogToFile = fopen(fn,"w"); // New log file each time we start.
		if (gLogToFile) {
			time_t now;
			time(&now);
			fprintf(gLogToFile,"Starting at %s",ctime(&now));
			fflush(gLogToFile);
			std::cout << "Logging to file: " << fn << "\n";
		}
	}

	// Open the log connection.
	openlog(name,0,facility);
}

// vim: ts=4 sw=4
