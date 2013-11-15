/*
 * Written by Thomas Tsou <ttsou@vt.edu>
 * Based on code by Harvind S Samra <hssamra@kestrelsp.com>
 *
 * Copyright 2011 Free Software Foundation, Inc.
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

#include "radioVector.h"

radioVector::radioVector(GSM::Time &time, size_t size,
			 size_t start, size_t chans)
	: vectors(chans), mTime(time)
{
	for (size_t i = 0; i < vectors.size(); i++)
		vectors[i] = new signalVector(size, start);
}

radioVector::radioVector(GSM::Time& wTime, signalVector *vector)
	: vectors(1), mTime(wTime)
{
	vectors[0] = vector;
}

radioVector::~radioVector()
{
	for (size_t i = 0; i < vectors.size(); i++)
		delete vectors[i];
}

GSM::Time radioVector::getTime() const
{
	return mTime;
}

void radioVector::setTime(const GSM::Time& wTime)
{
	mTime = wTime;
}

bool radioVector::operator>(const radioVector& other) const
{
	return mTime > other.mTime;
}

signalVector *radioVector::getVector(size_t chan) const
{
	if (chan >= vectors.size())
		return NULL;

	return vectors[chan];
}

bool radioVector::setVector(signalVector *vector, size_t chan)
{
	if (chan >= vectors.size())
		return false;

	vectors[chan] = vector;

	return true;
}

noiseVector::noiseVector(size_t size)
	: std::vector<float>(size), itr(0)
{
}

float noiseVector::avg() const
{
	float val = 0.0;

	for (size_t i = 0; i < size(); i++)
		val += (*this)[i];

	return val / (float) size();
}

bool noiseVector::insert(float val)
{
	if (!size())
		return false;

	if (itr >= this->size())
		itr = 0;

	(*this)[itr++] = val;

	return true;
}

GSM::Time VectorQueue::nextTime() const
{
	GSM::Time retVal;
	mLock.lock();

	while (mQ.size()==0)
		mWriteSignal.wait(mLock);

	retVal = mQ.top()->getTime();
	mLock.unlock();

	return retVal;
}

radioVector* VectorQueue::getStaleBurst(const GSM::Time& targTime)
{
	mLock.lock();
	if ((mQ.size()==0)) {
		mLock.unlock();
		return NULL;
	}

	if (mQ.top()->getTime() < targTime) {
		radioVector* retVal = mQ.top();
		mQ.pop();
		mLock.unlock();
		return retVal;
	}
	mLock.unlock();

	return NULL;
}

radioVector* VectorQueue::getCurrentBurst(const GSM::Time& targTime)
{
	mLock.lock();
	if ((mQ.size()==0)) {
		mLock.unlock();
		return NULL;
	}

	if (mQ.top()->getTime() == targTime) {
		radioVector* retVal = mQ.top();
		mQ.pop();
		mLock.unlock();
		return retVal;
	}
	mLock.unlock();

	return NULL;
}
