/*
 * Written by Thomas Tsou <ttsou@vt.edu>
 * Based on code by Harvind S Samra <hssamra@kestrelsp.com>
 *
 * Copyright 2011 Free Software Foundation, Inc.
 *
 * SPDX-License-Identifier: AGPL-3.0+
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

#ifndef RADIOVECTOR_H
#define RADIOVECTOR_H

#include "sigProcLib.h"
#include "GSMCommon.h"
#include "Interthread.h"

class radioVector {
public:
	radioVector(GSM::Time& wTime, size_t size = 0,
		    size_t start = 0, size_t chans = 1);

	radioVector(GSM::Time& wTime, signalVector *vector);
	~radioVector();

	GSM::Time getTime() const;
	void setTime(const GSM::Time& wTime);
	bool operator>(const radioVector& other) const;

	signalVector *getVector(size_t chan = 0) const;
	bool setVector(signalVector *vector, size_t chan = 0);
	size_t chans() const { return vectors.size(); }
private:
	std::vector<signalVector *> vectors;
	GSM::Time mTime;
};

class avgVector : std::vector<float> {
public:
	avgVector(size_t size = 0);
	bool insert(float val);
	float avg() const;

private:
	size_t itr;
};

class VectorFIFO : public InterthreadQueue<radioVector> { };

class VectorQueue : public InterthreadPriorityQueue<radioVector> {
public:
	GSM::Time nextTime() const;
	radioVector* getStaleBurst(const GSM::Time& targTime);
	radioVector* getCurrentBurst(const GSM::Time& targTime);
	Mutex *getMutex() const { return &mLock; };
};

#endif /* RADIOVECTOR_H */
