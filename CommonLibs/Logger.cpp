/*
* Copyright (C) 2018 sysmocom - s.f.m.c. GmbH
*
* SPDX-License-Identifier: AGPL-3.0+
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

std::ostream& operator<<(std::ostream& os, std::ostringstream& ss)
{
	return os << ss.str();
}

Log::~Log()
{
	int old_state;
	int mlen = mStream.str().size();
	int neednl = (mlen==0 || mStream.str()[mlen-1] != '\n');
	const char *fmt = neednl ? "%s\n" : "%s";

	log_mutex_lock_canceldisable(&old_state);
	LOGPSRC(mCategory, mPriority, filename, line, fmt, mStream.str().c_str());
	log_mutex_unlock_canceldisable(old_state);
}

ostringstream& Log::get()
{
	return mStream;
}

// vim: ts=4 sw=4
