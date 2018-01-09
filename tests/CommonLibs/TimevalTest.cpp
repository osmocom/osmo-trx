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
#include <iostream>
#include <assert.h>

using namespace std;

int main(int argc, char *argv[])
{
	Timeval then(10000);
	assert(then.elapsed() == -10000);
	cerr << then << " elapsed: " << then.elapsed() << endl;
	double then_seconds = then.seconds();
	double last_now = Timeval().seconds();
	long last_remaining = 10000;
	int loops = 0;

	while (!then.passed()) {
		double tnow = Timeval().seconds();
		cerr << "now: " << tnow << " then: " << then << " remaining: " << then.remaining() << endl;
		assert(last_now <= tnow && last_remaining >= then.remaining());
		assert(then_seconds == then.seconds());
		usleep(500000);
		loops++;
	}
	cerr << "now: " << Timeval() << " then: " << then << " remaining: " << then.remaining() << endl;
	assert(then.remaining() <= 0);
	assert(loops >= 18);

	printf("Done\n");
}
