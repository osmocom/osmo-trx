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

#include "Configuration.h"
#include "Logger.h"
#include "Threads.h"	// pat added

#define MAX_ALARMS 20

using namespace std;

// Switches to enable/disable logging targets
// MUST BE DEFINED BEFORE gConfig FOR gLogEarly() TO WORK CORRECTLY
bool gLogToConsole = true;
bool gLogToSyslog = false;
FILE *gLogToFile = NULL;
Mutex gLogToLock;


// Reference to a global config table, used all over the system.
extern ConfigurationTable gConfig;
// Global log level threshold:
int config_log_level;

/**@ The global alarms table. */
//@{
Mutex           alarmsLock;
list<string>    alarmsList;
void            addAlarm(const string&);
//@}



// (pat) If Log messages are printed before the classes in this module are inited
// (which happens when static classes have constructors that do work)
// the OpenBTS just crashes.
// Prevent that by setting sLoggerInited to true when this module is inited.
static bool sLoggerInited = 0;
static struct CheckLoggerInitStatus {
	CheckLoggerInitStatus() { sLoggerInited = 1; }
} sCheckloggerInitStatus;



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

// copies the alarm list and returns it. list supposed to be small.
list<string> gGetLoggerAlarms()
{
    alarmsLock.lock();
    list<string> ret;
    // excuse the "complexity", but to use std::copy with a list you need
    // an insert_iterator - copy technically overwrites, doesn't insert.
    insert_iterator< list<string> > ii(ret, ret.begin());
    copy(alarmsList.begin(), alarmsList.end(), ii);
    alarmsLock.unlock();
    return ret;
}

/** Add an alarm to the alarm list. */
void addAlarm(const string& s)
{
    alarmsLock.lock();
    alarmsList.push_back(s);
    while (alarmsList.size() > MAX_ALARMS) alarmsList.pop_front();
    alarmsLock.unlock();
}


Log::~Log()
{
	if (mDummyInit) return;
	// Anything at or above LOG_CRIT is an "alarm".
	// Save alarms in the local list and echo them to stderr.
	if (mPriority <= LOG_ERR) {
		if (sLoggerInited) addAlarm(mStream.str().c_str());
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


void gLogEarly(int level, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);

	if (gLogToSyslog) {
		va_list args_copy;
		va_copy(args_copy, args);
		vsyslog(level | LOG_USER, fmt, args_copy);
		va_end(args_copy);
	}

	if (gLogToConsole) {
		va_list args_copy;
		va_copy(args_copy, args);
		vprintf(fmt, args_copy);
		printf("\n");
		va_end(args_copy);
	}

	if (gLogToFile) {
		va_list args_copy;
		va_copy(args_copy, args);
		vfprintf(gLogToFile, fmt, args_copy);
		fprintf(gLogToFile, "\n");
		va_end(args_copy);
	}

	va_end(args);
}

// vim: ts=4 sw=4
