/*
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
#include "Timeval.h"
#include "Configuration.h"
#include <iostream>

ConfigurationTable gConfig;

class SimpleThreadTest : public Thread
{
public:
    SimpleThreadTest() : Thread("SimpleThreadTest") {}

    void runThread()
    {
        COUT(getThreadName() << ": Started thread");
        while (isThreadRunning()) {
            COUT(getThreadName() << ": Sleeping...");
            msleep(50);
        }
        COUT(getThreadName() << ": Stopped thread");
    }

};


void testSimpleStartStop()
{
    SimpleThreadTest simpleThreadTest;
    COUT("Main: Starting thread " << simpleThreadTest.getThreadName());
    simpleThreadTest.startThread();
    COUT("Main: Started  thread " << simpleThreadTest.getThreadName());
    msleep(30);
    COUT("Main: Stopping thread " << simpleThreadTest.getThreadName());
    simpleThreadTest.stopThread();
    COUT("Main: Stopped  thread " << simpleThreadTest.getThreadName());
}

void testDoubleRequestStop()
{
    SimpleThreadTest simpleThreadTest;
    COUT("Main: Starting thread " << simpleThreadTest.getThreadName());
    simpleThreadTest.startThread();
    COUT("Main: Started  thread " << simpleThreadTest.getThreadName());
    msleep(30);
    COUT("Main: Requesting stop for thread " << simpleThreadTest.getThreadName());
    simpleThreadTest.requestThreadStop();
    msleep(30);
    COUT("Main: Requesting stop for thread " << simpleThreadTest.getThreadName());
    simpleThreadTest.requestThreadStop();
    msleep(30);
    COUT("Main: Stopping thread " << simpleThreadTest.getThreadName());
    simpleThreadTest.stopThread();
    COUT("Main: Stopped  thread " << simpleThreadTest.getThreadName());
}


int main(int argc, char *argv[])
{
    std::cout<< std::endl << "Simple start/stop test" << std::endl << std::endl ;
    testSimpleStartStop();

    std::cout << std::endl << "Double requestThreadStop() test" << std::endl << std::endl ;
    testDoubleRequestStop();
}


// vim: ts=4 sw=4
