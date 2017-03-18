/*
* Copyright 2008, 2009 Free Software Foundation, Inc.
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
#include <stdio.h>
#include <sstream>

using namespace std;


/**
  Apply a Galois polymonial to a binary seqeunce.
  @param val The input sequence.
  @param poly The polynomial.
  @param order The order of the polynomial.
  @return Single-bit result.
*/
unsigned applyPoly(uint64_t val, uint64_t poly, unsigned order)
{
	uint64_t prod = val & poly;
	unsigned sum = prod;
	for (unsigned i=1; i<order; i++) sum ^= prod>>i;
	return sum & 0x01;
}






BitVector::BitVector(const char *valString)
	:Vector<char>(strlen(valString))
{
	uint32_t accum = 0;
	for (size_t i=0; i<size(); i++) {
		accum <<= 1;
		if (valString[i]=='1') accum |= 0x01;
		mStart[i] = accum;
	}
}





uint64_t BitVector::peekField(size_t readIndex, unsigned length) const
{
	uint64_t accum = 0;
	char *dp = mStart + readIndex;
	assert(dp+length <= mEnd);
	for (unsigned i=0; i<length; i++) {
		accum = (accum<<1) | ((*dp++) & 0x01);
	}
	return accum;
}




uint64_t BitVector::peekFieldReversed(size_t readIndex, unsigned length) const
{
	uint64_t accum = 0;
	char *dp = mStart + readIndex + length - 1;
	assert(dp<mEnd);
	for (int i=(length-1); i>=0; i--) {
		accum = (accum<<1) | ((*dp--) & 0x01);
	}
	return accum;
}




uint64_t BitVector::readField(size_t& readIndex, unsigned length) const
{
	const uint64_t retVal = peekField(readIndex,length);
	readIndex += length;
	return retVal;
}


uint64_t BitVector::readFieldReversed(size_t& readIndex, unsigned length) const
{
	const uint64_t retVal = peekFieldReversed(readIndex,length);
	readIndex += length;
	return retVal;
}





void BitVector::fillField(size_t writeIndex, uint64_t value, unsigned length)
{
	char *dpBase = mStart + writeIndex;
	char *dp = dpBase + length - 1;
	assert(dp < mEnd);
	while (dp>=dpBase) {
		*dp-- = value & 0x01;
		value >>= 1;
	}
}


void BitVector::fillFieldReversed(size_t writeIndex, uint64_t value, unsigned length)
{
	char *dp = mStart + writeIndex;
	char *dpEnd = dp + length - 1;
	assert(dpEnd < mEnd);
	while (dp<=dpEnd) {
		*dp++ = value & 0x01;
		value >>= 1;
	}
}




void BitVector::writeField(size_t& writeIndex, uint64_t value, unsigned length)
{
	fillField(writeIndex,value,length);
	writeIndex += length;
}


void BitVector::writeFieldReversed(size_t& writeIndex, uint64_t value, unsigned length)
{
	fillFieldReversed(writeIndex,value,length);
	writeIndex += length;
}


void BitVector::invert()
{
	for (size_t i=0; i<size(); i++) {
		mStart[i] = ~mStart[i];
	}
}




void BitVector::reverse8()
{
	assert(size()>=8);

	char tmp0 = mStart[0];
	mStart[0] = mStart[7];
	mStart[7] = tmp0;

	char tmp1 = mStart[1];
	mStart[1] = mStart[6];
	mStart[6] = tmp1;

	char tmp2 = mStart[2];
	mStart[2] = mStart[5];
	mStart[5] = tmp2;

	char tmp3 = mStart[3];
	mStart[3] = mStart[4];
	mStart[4] = tmp3;
}



void BitVector::LSB8MSB()
{
	if (size()<8) return;
	size_t size8 = 8*(size()/8);
	size_t iTop = size8 - 8;
	for (size_t i=0; i<=iTop; i+=8) segment(i,8).reverse8();
}



uint64_t BitVector::syndrome(Generator& gen) const
{
	gen.clear();
	const char *dp = mStart;
	while (dp<mEnd) gen.syndromeShift(*dp++);
	return gen.state();
}


uint64_t BitVector::parity(Generator& gen) const
{
	gen.clear();
	const char *dp = mStart;
	while (dp<mEnd) gen.encoderShift(*dp++);
	return gen.state();
}



unsigned BitVector::sum() const
{
	unsigned sum = 0;
	for (size_t i=0; i<size(); i++) sum += mStart[i] & 0x01;
	return sum;
}




void BitVector::map(const unsigned *map, size_t mapSize, BitVector& dest) const
{
	for (unsigned i=0; i<mapSize; i++) {
		dest.mStart[i] = mStart[map[i]];
	}
}




void BitVector::unmap(const unsigned *map, size_t mapSize, BitVector& dest) const
{
	for (unsigned i=0; i<mapSize; i++) {
		dest.mStart[map[i]] = mStart[i];
	}
}







ostream& operator<<(ostream& os, const BitVector& hv)
{
	for (size_t i=0; i<hv.size(); i++) {
		if (hv.bit(i)) os << '1';
		else os << '0';
	}
	return os;
}




SoftVector::SoftVector(const BitVector& source)
{
	resize(source.size());
	for (size_t i=0; i<size(); i++) {
		if (source.bit(i)) mStart[i]=1.0F;
		else mStart[i]=0.0F;
	}
}


BitVector SoftVector::sliced() const
{
	size_t sz = size();
	BitVector newSig(sz);
	for (size_t i=0; i<sz; i++) {
		if (mStart[i]>0.5F) newSig[i]=1;
		else newSig[i] = 0;
	}
	return newSig;
}


float SoftVector::getEnergy(float *plow) const
{
	const SoftVector &vec = *this;
	int len = vec.size();
	float avg = 0; float low = 1;
	for (int i = 0; i < len; i++) {
		float bit = vec[i];
		float energy = 2*((bit < 0.5) ? (0.5-bit) : (bit-0.5));
		if (energy < low) low = energy;
		avg += energy/len;
	}
	if (plow) { *plow = low; }
	return avg;
}


ostream& operator<<(ostream& os, const SoftVector& sv)
{
	for (size_t i=0; i<sv.size(); i++) {
		if (sv[i]<0.25) os << "0";
		else if (sv[i]<0.4) os << "o";
		else if (sv[i]<0.5) os << ".";
		else if (sv[i]>0.75) os << "1";
		else if (sv[i]>0.6) os << "|";
		else if (sv[i]>0.5) os << "'";
		else os << "-";
	}
	return os;
}



void BitVector::pack(unsigned char* targ) const
{
	// Assumes MSB-first packing.
	unsigned bytes = size()/8;
	for (unsigned i=0; i<bytes; i++) {
		targ[i] = peekField(i*8,8);
	}
	unsigned whole = bytes*8;
	unsigned rem = size() - whole;
	if (rem==0) return;
	targ[bytes] = peekField(whole,rem) << (8-rem);
}


void BitVector::unpack(const unsigned char* src)
{
	// Assumes MSB-first packing.
	unsigned bytes = size()/8;
	for (unsigned i=0; i<bytes; i++) {
		fillField(i*8,src[i],8);
	}
	unsigned whole = bytes*8;
	unsigned rem = size() - whole;
	if (rem==0) return;
        fillField(whole,src[bytes] >> (8-rem),rem);
}

void BitVector::hex(ostream& os) const
{
	os << std::hex;
	unsigned digits = size()/4;
	size_t wp=0;
	for (unsigned i=0; i<digits; i++) {
		os << readField(wp,4);
	}
	os << std::dec;
}

std::string BitVector::hexstr() const
{
	std::ostringstream ss;
	hex(ss);
	return ss.str();
}


bool BitVector::unhex(const char* src)
{
	// Assumes MSB-first packing.
	unsigned int val;
	unsigned digits = size()/4;
	for (unsigned i=0; i<digits; i++) {
		if (sscanf(src+i, "%1x", &val) < 1) {
			return false;
		}
		fillField(i*4,val,4);
	}
	unsigned whole = digits*4;
	unsigned rem = size() - whole;
	if (rem>0) {
		if (sscanf(src+digits, "%1x", &val) < 1) {
			return false;
		}
		fillField(whole,val,rem);
	}
	return true;
}

// vim: ts=4 sw=4
