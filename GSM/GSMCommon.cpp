/*
* Copyright 2008 Free Software Foundation, Inc.
* Copyright 2011 Range Networks, Inc.
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


#include "GSMCommon.h"

using namespace GSM;
using namespace std;


const BitVector GSM::gTrainingSequence[] = {
    BitVector("00100101110000100010010111"),
    BitVector("00101101110111100010110111"),
    BitVector("01000011101110100100001110"),
    BitVector("01000111101101000100011110"),
    BitVector("00011010111001000001101011"),
    BitVector("01001110101100000100111010"),
    BitVector("10100111110110001010011111"),
    BitVector("11101111000100101110111100"),
};

const BitVector GSM::gEdgeTrainingSequence[] = {
    BitVector("111111001111111001111001001001111111111111001111111111001111111001111001001001"),
    BitVector("111111001111001001111001001001111001001001001111111111001111001001111001001001"),
    BitVector("111001111111111111001001001111001001001111001111111001111111111111001001001111"),
    BitVector("111001111111111001001001001111001001111001111111111001111111111001001001001111"),
    BitVector("111111111001001111001111001001001111111001111111111111111001001111001111001001"),
    BitVector("111001111111001001001111001111001001111111111111111001111111001001001111001111"),
    BitVector("001111001111111001001001001001111001001111111111001111001111111001001001001001"),
    BitVector("001001001111001001001001111111111001111111001111001001001111001001001001111111"),
};

const BitVector GSM::gDummyBurst("0001111101101110110000010100100111000001001000100000001111100011100010111000101110001010111010010100011001100111001111010011111000100101111101010000");
const BitVector GSM::gDummyBurstTSC("01110001011100010111000101");

/* 3GPP TS 05.02, section 5.2.7 "Access burst (AB)", synch. sequence bits */
const BitVector GSM::gRACHSynchSequenceTS0("01001011011111111001100110101010001111000");  /* GSM, GMSK (default) */
const BitVector GSM::gRACHSynchSequenceTS1("01010100111110001000011000101111001001101");  /* EGPRS, 8-PSK */
const BitVector GSM::gRACHSynchSequenceTS2("11101111001001110101011000001101101110111");  /* EGPRS, GMSK */

const BitVector GSM::gSCHSynchSequence("1011100101100010000001000000111100101101010001010111011000011011");

//                               |-head-||---------midamble----------------------||--------------data----------------||t|
const BitVector GSM::gRACHBurst("0011101001001011011111111001100110101010001111000110111101111110000111001001010110011000");


int32_t GSM::FNDelta(int32_t v1, int32_t v2)
{
	static const int32_t halfModulus = gHyperframe/2;
	int32_t delta = v1-v2;
	if (delta>=halfModulus) delta -= gHyperframe;
	else if (delta<-halfModulus) delta += gHyperframe;
	return (int32_t) delta;
}

int GSM::FNCompare(int32_t v1, int32_t v2)
{
	int32_t delta = FNDelta(v1,v2);
	if (delta>0) return 1;
	if (delta<0) return -1;
	return 0;
}




ostream& GSM::operator<<(ostream& os, const Time& t)
{
	os << t.TN() << ":" << t.FN();
	return os;
}


// vim: ts=4 sw=4
