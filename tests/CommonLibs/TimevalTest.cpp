/*
* Copyright 2008 Free Software Foundation, Inc.
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




#include "Timeval.h"
#include <iostream>
#include <assert.h>
#include <sys/time.h>

extern "C" {
#include <osmocom/core/timer.h>
}

using namespace std;

int main(int argc, char *argv[])
{

	osmo_clock_override_enable(CLOCK_REALTIME, true);

	struct timespec *clk = osmo_clock_override_gettimespec(CLOCK_REALTIME);
	clk->tv_sec = 0;
	clk->tv_nsec = 1000;

	long last_remaining = 10000; /*10 sec */
	Timeval then(last_remaining);
	assert(then.elapsed() == -last_remaining);
	cerr << then << " elapsed: " << then.elapsed() << endl;

	/* Check that last_remaining parameter affects setting time in the future */
	osmo_clock_override_add(CLOCK_REALTIME, 0, 10*1000*1000);
	double increased_time_secs = Timeval().seconds();
	assert(increased_time_secs < then.seconds());

	struct timespec invariant_time  = then.timespec();
	int loops = 0;

	while (!then.passed()) {
		struct timespec tspecnow = then.timespec();
		cerr << "["<< loops << "] now: " << Timeval().seconds() << " then: " << then << " remaining: " << then.remaining() << endl;
		assert(last_remaining >= then.remaining());
		assert(tspecnow.tv_sec == invariant_time.tv_sec && tspecnow.tv_nsec == invariant_time.tv_nsec);
		osmo_clock_override_add(CLOCK_REALTIME, 0, 500000*1000);
		loops++;
	}
	cerr << "now: " << Timeval() << " then: " << then << " remaining: " << then.remaining() << endl;
	assert(then.remaining() == -10);
	assert(loops == 20);

	printf("Done\n");
}
