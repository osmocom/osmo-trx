/*
 * Written by Thomas Tsou <ttsou@vt.edu>
 * Based on code by Harvind S Samra <hssamra@kestrelsp.com>
 *
 * Copyright 2011 Free Software Foundation, Inc.
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

#ifndef RADIOCLOCK_H
#define RADIOCLOCK_H

#include "GSMCommon.h"

class RadioClock {
public:
	static GSM::Time adjust(GSM::Time &base, GSM::Time &offset);

	void set(const GSM::Time& wTime);
	void adjust(GSM::Time &wOffset);
	void incTN();
	GSM::Time get();
	void wait();

private:
	GSM::Time mClock;
	Mutex mLock;
	Signal updateSignal;
};

#endif /* RADIOCLOCK_H */
