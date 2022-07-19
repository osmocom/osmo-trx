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



#include "Threads.h"
#include "Interthread.h"
#include <iostream>
#include <mutex>

std::mutex dbg_cout;

InterthreadQueue<int> gQ;
InterthreadMap<int,int> gMap;

int q_last_read_val = -1;
int q_last_write_val;
int m_last_read_val;
int m_last_write_val;

#define CERR(text) { dbg_cout.lock() ; std::cerr << text; dbg_cout.unlock(); }

void* qWriter(void*)
{
	int *p;
	for (int i=0; i<20; i++) {
		p = new int;
		*p = i;
		CERR("queue write " << *p);
		gQ.write(p);
		q_last_write_val = i;
		if (random()%2) sleep(1);
	}
	p = new int;
	*p = -1;
	gQ.write(p);
	return NULL;
}

void* qReader(void*)
{
	bool done = false;
	while (!done) {
		int *p = gQ.read();
		CERR("queue read " << *p);
		if (*p<0) {
			assert(q_last_read_val == 19 && *p == -1);
			done = true;
		} else {
			assert(q_last_read_val == *p - 1);
			q_last_read_val = *p;
		}
		delete p;
	}
	return NULL;
}


void* mapWriter(void*)
{
	int *p;
	for (int i=0; i<20; i++) {
		p = new int;
		*p = i;
		CERR("map write " << *p);
		gMap.write(i,p);
		m_last_write_val = i;
		if (random()%2) sleep(1);
	}
	return NULL;
}

void* mapReader(void*)
{
	for (int i=0; i<20; i++) {
		int *p = gMap.read(i);
		CERR("map read " << *p);
		assert(*p == i);
		m_last_read_val = *p;
		// InterthreadMap will delete the pointers
		// delete p;
	}
	return NULL;
}






int main(int argc, char *argv[])
{
	Thread qReaderThread;
	qReaderThread.start(qReader,NULL);
	Thread mapReaderThread;
	mapReaderThread.start(mapReader,NULL);

	Thread qWriterThread;
	qWriterThread.start(qWriter,NULL);
	Thread mapWriterThread;
	mapWriterThread.start(mapWriter,NULL);

	qReaderThread.join();
	qWriterThread.join();
	mapReaderThread.join();
	mapWriterThread.join();

	assert(q_last_write_val == 19);
	assert(q_last_read_val == 19);
	assert(m_last_write_val == 19);
	assert(m_last_read_val == 19);

	printf("Done\n");
}


// vim: ts=4 sw=4
