/*
* Copyright 2008 Free Software Foundation, Inc.
* Copyright 2013 Alexander Chemeris <Alexander.Chemeris@fairwaves.ru>
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



#include "Threads.h"
#include "Interthread.h"
#include "Configuration.h"
#include <iostream>

using namespace std;

ConfigurationTable gConfig;

InterthreadQueue<int> gQ;
InterthreadMap<int,int> gMap;

class QueueWriter : public Thread
{
public:
	QueueWriter() : Thread("QueueWriter") {}

protected:
	virtual void runThread()
	{
		int *p;
		for (int i=0; i<20; i++) {
			p = new int;
			*p = i;
			COUT("queue write " << *p);
			gQ.write(p);
			msleep(1);
		}
		p = new int;
		*p = -1;
		gQ.write(p);
	}
};

class QueueReader : public Thread
{
public:
	QueueReader() : Thread("QueueReader") {}

protected:
	virtual void runThread()
	{
		bool done = false;
		while (!done) {
			int *p = gQ.read();
			COUT("queue read " << *p);
			if (*p<0) done=true;
			delete p;
		}
	}
};


class MapWriter : public Thread
{
public:
	MapWriter() : Thread("MapWriter") {}

protected:
	virtual void runThread()
	{
		int *p;
		for (int i=0; i<20; i++) {
			p = new int;
			*p = i;
			COUT("map write " << *p);
			gMap.write(i,p);
			msleep(1);
		}
	}
};

class MapReader : public Thread
{
public:
	MapReader() : Thread("MapReader") {}

protected:
	virtual void runThread()
	{
		for (int i=0; i<20; i++) {
			int *p = gMap.read(i);
			COUT("map read " << *p);
			// InterthreadMap will delete the pointers
		}
	}
};






int main(int argc, char *argv[])
{
	COUT("TEST 1: InterthreadQueue")
	QueueReader qReaderThread;
	QueueWriter qWriterThread;
	qReaderThread.startThread();
	qWriterThread.startThread();
	// stopThread() will wait for a thread to stop for 5 seconds, which
	// is more than enough for this test to finish.
	qReaderThread.stopThread();
	qWriterThread.stopThread();

	COUT("TEST 2: InterthreadMap")
	MapReader mapReaderThread;
	mapReaderThread.startThread();
	MapWriter mapWriterThread;
	mapWriterThread.startThread();
	// stopThread() will wait for a thread to stop for 5 seconds, which
	// is more than enough for this test to finish.
	mapReaderThread.stopThread();
	mapWriterThread.stopThread();
}


// vim: ts=4 sw=4
