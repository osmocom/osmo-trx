/*
* Copyright 2008 Free Software Foundation, Inc.
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



#include "Timeval.h"

extern "C" {
#include <osmocom/core/timer.h>
}

using namespace std;

void Timeval::now()
{
	osmo_clock_gettime(CLOCK_REALTIME, &mTimespec);
}

void Timeval::future(unsigned offset)
{
	now();
	unsigned sec = offset/1000;
	unsigned msec = offset%1000;
	mTimespec.tv_nsec += msec*1000*1000;
	mTimespec.tv_sec += sec;
	if (mTimespec.tv_nsec > 1000*1000*1000) {
		mTimespec.tv_nsec -= 1000*1000*1000;
		mTimespec.tv_sec += 1;
	}
}


struct timespec Timeval::timespec() const
{
	return mTimespec;
}


bool Timeval::passed() const
{
	Timeval nowTime;
	if (nowTime.mTimespec.tv_sec < mTimespec.tv_sec) return false;
	if (nowTime.mTimespec.tv_sec > mTimespec.tv_sec) return true;
	if (nowTime.mTimespec.tv_nsec >= mTimespec.tv_nsec) return true;
	return false;
}

double Timeval::seconds() const
{
	return ((double)mTimespec.tv_sec) + 1e-9*((double)mTimespec.tv_nsec);
}



long Timeval::delta(const Timeval& other) const
{
	// 2^31 milliseconds is just over 4 years.
	int32_t deltaS = other.sec() - sec();
	int32_t deltaNs = other.nsec() - nsec();
	return 1000*deltaS + deltaNs/1000000;
}
	



ostream& operator<<(ostream& os, const Timeval& tv)
{
	os.setf( ios::fixed, ios::floatfield );
	os << tv.seconds();
	return os;
}


ostream& operator<<(ostream& os, const struct timespec& ts)
{
	os << ts.tv_sec << "," << ts.tv_nsec/1000;
	return os;
}



// vim: ts=4 sw=4
