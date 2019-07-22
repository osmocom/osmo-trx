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




#include "BitVector.h"
#include <iostream>
#include <cstdlib>

using namespace std;


int main(int argc, char *argv[])
{
	BitVector v5("000011110000");
	int r1 = v5.peekField(0,8);
	int r2 = v5.peekField(4,4);
	int r3 = v5.peekField(4,8);
	cout << r1 <<  ' ' << r2 << ' ' << r3 << endl;
	cout << v5 << endl;
	v5.fillField(0,0xa,4);
	int r4 = v5.peekField(0,8);
	cout << v5 << endl;
	cout << r4 << endl;

	v5.reverse8();
	cout << v5 << endl;


	unsigned char ts[9] = "abcdefgh";
	BitVector tp(70);
	cout << "ts=" << ts << endl;
	tp.unpack(ts);
	cout << "tp=" << tp << endl;
	tp.pack(ts);
	cout << "ts=" << ts << endl;
}
