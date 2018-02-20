/*
* Copyright 2009 Free Software Foundation, Inc.
* Copyright 2010 Kestrel Signal Processing, Inc.
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

#include <iostream>
#include <iterator>

#include "Logger.h"

int main(int argc, char *argv[])
{
	gLogInit("LogTest","NOTICE",LOG_LOCAL7);

	Log(LOG_EMERG).get() << " testing the logger.";
	Log(LOG_ALERT).get() << " testing the logger.";
	Log(LOG_CRIT).get() << " testing the logger.";
	Log(LOG_ERR).get() << " testing the logger.";
	Log(LOG_WARNING).get() << " testing the logger.";
	Log(LOG_NOTICE).get() << " testing the logger.";
	Log(LOG_INFO).get() << " testing the logger.";
        Log(LOG_DEBUG).get() << " testing the logger.";
    std::cout << "----------- generating 20 alarms ----------" << std::endl;
    for (int i = 0 ; i < 20 ; ++i) {
        Log(LOG_ALERT).get() << i;
    }
}
