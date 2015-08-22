/*
 * Segmented Ring Buffer
 *
 * Copyright (C) 2015 Ettus Research LLC
 *
 * Author: Tom Tsou <tom@tsou.cc>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 */

#include <string.h>
#include <iostream>
#include "radioBuffer.h"

RadioBuffer::RadioBuffer(size_t numSegments, size_t segmentLen,
			 size_t hLen, bool outDirection)
	: writeIndex(0), readIndex(0), availSamples(0)
{
	if (!outDirection)
		hLen = 0;

	buffer = new float[2 * (hLen + numSegments * segmentLen)];
	bufferLen = numSegments * segmentLen;

	segments.resize(numSegments);

	for (size_t i = 0; i < numSegments; i++)
		segments[i] = &buffer[2 * (hLen + i * segmentLen)];

	this->outDirection = outDirection;
	this->numSegments = numSegments;
	this->segmentLen = segmentLen;
	this->hLen = hLen;
}

RadioBuffer::~RadioBuffer()
{
	delete buffer;
}

void RadioBuffer::reset()
{
	writeIndex = 0;
	readIndex = 0;
	availSamples = 0;
}

/*
 * Output direction
 *
 * Return a pointer to the oldest segment or NULL if a complete segment is not
 * available.
 */
const float *RadioBuffer::getReadSegment()
{
	if (!outDirection) {
		std::cout << "Invalid direction" << std::endl;
		return NULL;
	}
	if (availSamples < segmentLen) {
		std::cout << "Not enough samples " << std::endl;
		std::cout << availSamples << " available per segment "
			  << segmentLen << std::endl;
		return NULL;
	}

	size_t num = readIndex / segmentLen;

	if (num >= numSegments) {
		std::cout << "Invalid segment" << std::endl;
		return NULL;
	} else if (!num) {
		memcpy(buffer,
		       &buffer[2 * bufferLen],
		       hLen * 2 * sizeof(float));
	}

	availSamples -= segmentLen;
	readIndex = (readIndex + segmentLen) % bufferLen;

	return segments[num];
}

/*
 * Output direction
 *
 * Write a non-segment length of samples to the buffer. 
 */
bool RadioBuffer::write(const float *wr, size_t len)
{
	if (!outDirection) {
		std::cout << "Invalid direction" << std::endl;
		return false;
	}
	if (availSamples + len > bufferLen) {
		std::cout << "Insufficient space" << std::endl;
		std::cout << bufferLen - availSamples << " available per write "
			  << len << std::endl;
		return false;
	}

	if (writeIndex + len <= bufferLen) {
		memcpy(&buffer[2 * (writeIndex + hLen)],
		       wr, len * 2 * sizeof(float));
	} else {
		size_t len0 = bufferLen - writeIndex;
		size_t len1 = len - len0;
		memcpy(&buffer[2 * (writeIndex + hLen)], wr, len0 * 2 * sizeof(float));
		memcpy(&buffer[2 * hLen], &wr[2 * len0], len1 * 2 * sizeof(float));
	}

	availSamples += len;
	writeIndex = (writeIndex + len) % bufferLen;

	return true;
}

bool RadioBuffer::zero(size_t len)
{
	if (!outDirection) {
		std::cout << "Invalid direction" << std::endl;
		return false;
	}
	if (availSamples + len > bufferLen) {
		std::cout << "Insufficient space" << std::endl;
		std::cout << bufferLen - availSamples << " available per zero "
			  << len << std::endl;
		return false;
	}

	if (writeIndex + len <= bufferLen) {
		memset(&buffer[2 * (writeIndex + hLen)],
		       0, len * 2 * sizeof(float));
	} else {
		size_t len0 = bufferLen - writeIndex;
		size_t len1 = len - len0;
		memset(&buffer[2 * (writeIndex + hLen)], 0, len0 * 2 * sizeof(float));
		memset(&buffer[2 * hLen], 0, len1 * 2 * sizeof(float));
	}

	availSamples += len;
	writeIndex = (writeIndex + len) % bufferLen;

	return true;
}

/*
 * Input direction
 */
float *RadioBuffer::getWriteSegment()
{
	if (outDirection) {
		std::cout << "Invalid direction" << std::endl;
		return NULL;
	}
	if (bufferLen - availSamples < segmentLen) {
		std::cout << "Insufficient samples" << std::endl;
		std::cout << bufferLen - availSamples
			  << " available for segment " << segmentLen
			  << std::endl;
		return NULL;
	}
	if (writeIndex % segmentLen) {
		std::cout << "Internal segment error" << std::endl;
		return NULL;
	}

	size_t num = writeIndex / segmentLen;

	if (num >= numSegments)
		return NULL;

	availSamples += segmentLen;
	writeIndex = (writeIndex + segmentLen) % bufferLen;

	return segments[num];
}

bool RadioBuffer::zeroWriteSegment()
{
	float *segment = getWriteSegment();
	if (!segment)
		return false;

	memset(segment, 0, segmentLen * 2 * sizeof(float));

	return true;
}

bool RadioBuffer::read(float *rd, size_t len)
{
	if (outDirection) {
		std::cout << "Invalid direction" << std::endl;
		return false;
	}
	if (availSamples < len) {
		std::cout << "Insufficient samples" << std::endl;
		std::cout << availSamples << " available for "
			  << len << std::endl;
		return false;
	}

	if (readIndex + len <= bufferLen) {
		memcpy(rd, &buffer[2 * readIndex], len * 2 * sizeof(float));
	} else {
		size_t len0 = bufferLen - readIndex;
		size_t len1 = len - len0;
		memcpy(rd, &buffer[2 * readIndex], len0 * 2 * sizeof(float));
		memcpy(&rd[2 * len0], buffer, len1 * 2 * sizeof(float));
	}

	availSamples -= len;
	readIndex = (readIndex + len) % bufferLen;

	return true;
}
