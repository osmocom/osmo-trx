/*
 * Written by Thomas Tsou <ttsou@vt.edu>
 * Based on code by Harvind S Samra <hssamra@kestrelsp.com>
 *
 * Copyright 2011 Free Software Foundation, Inc.
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

#include "radioClock.h"

void RadioClock::set(const GSM::Time& wTime)
{
	mLock.lock();
	mClock = wTime;
	updateSignal.signal();
	mLock.unlock();
}

void RadioClock::adjust(GSM::Time& wOffset)
{
	int tn_diff, fn_diff = 0;

	mLock.lock();

	/* Modulo TN adustment */
	tn_diff = mClock.TN() + wOffset.TN();
	if (tn_diff < 0) {
		tn_diff += 8;
		fn_diff--;
	} else if (tn_diff >= 8) {
		tn_diff -= 8;
		fn_diff++;
	}

	/* Modulo FN adjustment */
	fn_diff += mClock.FN() + wOffset.FN();
	if (fn_diff < 0)
		fn_diff += GSM::gHyperframe;
	else if ((unsigned) fn_diff >= GSM::gHyperframe)
		fn_diff = fn_diff - GSM::gHyperframe;

	mClock = GSM::Time(fn_diff, tn_diff);
	updateSignal.signal();

	mLock.unlock();
}

void RadioClock::incTN()
{
	mLock.lock();
	mClock.incTN();
	updateSignal.signal();
	mLock.unlock();
}

GSM::Time RadioClock::get()
{
	mLock.lock();
	GSM::Time retVal = mClock;
	mLock.unlock();

	return retVal;
}

void RadioClock::wait()
{
	mLock.lock();
	updateSignal.wait(mLock,1);
	mLock.unlock();
}
