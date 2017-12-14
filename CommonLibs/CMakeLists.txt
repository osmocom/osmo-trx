#
# Copyright 2008, 2009 Free Software Foundation, Inc.
# Copyright 2011, 2012 Range Networks, Inc.
#
# This software is distributed under the terms of the GNU Public License.
# See the COPYING file in the main directory for details.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

set(EXTRA_DIST example.config README.common)

set(libcommon_files
	BitVector.cpp
	LinkedLists.cpp
	Sockets.cpp
	Threads.cpp
	Timeval.cpp
	Logger.cpp)

if(SQLITE_CONFIG)
	set(libcommon_files
		${libcommon_files}
		Configuration.cpp
		sqlite3util.cpp)
endif(SQLITE_CONFIG)

add_library(common ${libcommon_files})

add_executable(InterthreadTest InterthreadTest.cpp)
target_link_libraries(InterthreadTest common pthread)

add_executable(SocketsTest SocketsTest.cpp)
target_link_libraries(SocketsTest common pthread)

add_executable(TimevalTest TimevalTest.cpp)
target_link_libraries(TimevalTest common)


