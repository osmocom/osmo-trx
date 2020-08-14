/**@file Simplified Vector template with aliases. */
/*
* Copyright 2008 Free Software Foundation, Inc.
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




#ifndef VECTOR_H
#define VECTOR_H

#include <string.h>
#include <iostream>
#include <assert.h>
#include <stdlib.h>

#ifndef __OPTIMIZE__
#define assert_no_opt(x) assert(x)
#else
#define assert_no_opt(x)
#endif
// We can't use Logger.h in this file...
extern int gVectorDebug;
#define BVDEBUG(msg) if (gVectorDebug) {std::cout << msg;}

typedef void (*vector_free_func)(void* wData);
typedef void *(*vector_alloc_func)(size_t newSize);

/**
	A simplified Vector template with aliases.
	Unlike std::vector, this class does not support dynamic resizing.
	Unlike std::vector, this class does support "aliases" and subvectors.
*/
template <class T> class Vector {

	// TODO -- Replace memcpy calls with for-loops.

	public:

	/**@name Iterator types. */
	//@{
	typedef T* iterator;
	typedef const T* const_iterator;
	//@}

	protected:

	T* mData;		///< allocated data block, if any
	T* mStart;		///< start of useful data
	T* mEnd;		///< end of useful data + 1
	vector_alloc_func mAllocFunc; ///< function used to alloc new mData during resize.
	vector_free_func mFreeFunc; ///< function used to free mData.

	public:

	/****
	char *inspect() {
		static char buf[100];
		sprintf(buf," mData=%p mStart=%p mEnd=%p ",mData,mStart,mEnd);
		return buf;
	}
	***/

	/** Return the size of the Vector. */
	size_t size() const
	{
		assert_no_opt(mStart>=mData);
		assert_no_opt(mEnd>=mStart);
		return mEnd - mStart;
	}

	/** Return size in bytes. */
	size_t bytes() const { return size()*sizeof(T); }

	/** Change the size of the Vector, discarding content. */
	void resize(size_t newSize)
	{
		if (mData!=NULL) {
			if (mFreeFunc)
				mFreeFunc(mData);
			else
				delete[] mData;
		}
		if (newSize==0) mData=NULL;
		else {
			if (mAllocFunc)
				mData = (T*) mAllocFunc(newSize);
			else
				mData = new T[newSize];
		}
		mStart = mData;
		mEnd = mStart + newSize;
	}

	/** Reduce addressable size of the Vector, keeping content. */
	void shrink(size_t newSize)
	{
		assert_no_opt(newSize <= mEnd - mStart);
		mEnd = mStart + newSize;
	}

	/** Release memory and clear pointers. */
	void clear() { resize(0); }


	/** Copy data from another vector. */
	void clone(const Vector<T>& other)
	{
		resize(other.size());
		other.copyTo(*this);
	}




	//@{

	/** Build an empty Vector of a given size. */
	Vector(size_t wSize=0, vector_alloc_func wAllocFunc=NULL, vector_free_func wFreeFunc=NULL)
		:mData(NULL), mAllocFunc(wAllocFunc), mFreeFunc(wFreeFunc)
	{ resize(wSize); }

	/** Build a Vector by moving another. */
	Vector(Vector<T>&& other)
		:mData(other.mData),mStart(other.mStart),mEnd(other.mEnd), mAllocFunc(other.mAllocFunc),  mFreeFunc(other.mFreeFunc)
	{ other.mData=NULL; }

	/** Build a Vector by copying another. */
	Vector(const Vector<T>& other):mData(NULL), mAllocFunc(other.mAllocFunc), mFreeFunc(other.mFreeFunc) { clone(other); }

	/** Build a Vector with explicit values. */
	Vector(T* wData, T* wStart, T* wEnd, vector_alloc_func wAllocFunc=NULL, vector_free_func wFreeFunc=NULL)
		:mData(wData),mStart(wStart),mEnd(wEnd), mAllocFunc(wAllocFunc), mFreeFunc(wFreeFunc)
	{ }

	/** Build a vector from an existing block, NOT to be deleted upon destruction. */
	Vector(T* wStart, size_t span, vector_alloc_func wAllocFunc=NULL, vector_free_func wFreeFunc=NULL)
		:mData(NULL),mStart(wStart),mEnd(wStart+span),mAllocFunc(wAllocFunc), mFreeFunc(wFreeFunc)
	{ }

	/** Build a Vector by concatenation. */
	Vector(const Vector<T>& other1, const Vector<T>& other2, vector_alloc_func wAllocFunc=NULL, vector_free_func wFreeFunc=NULL)
		:mData(NULL), mAllocFunc(wAllocFunc), mFreeFunc(wFreeFunc)
	{
		resize(other1.size()+other2.size());
		memcpy(mStart, other1.mStart, other1.bytes());
		memcpy(mStart+other1.size(), other2.mStart, other2.bytes());
	}

	//@}

	/** Destroy a Vector, deleting held memory. */
	~Vector() { clear(); }




	//@{

	/** Assign from another Vector, shifting ownership. */
	void operator=(Vector<T>& other)
	{
		clear();
		mData=other.mData;
		mStart=other.mStart;
		mEnd=other.mEnd;
		mAllocFunc=other.mAllocFunc;
		mFreeFunc=other.mFreeFunc;
		other.mData=NULL;
	}

	/** Assign from another Vector, copying. */
	void operator=(const Vector<T>& other) { clone(other); }

	//@}


	//@{

	/** Return an alias to a segment of this Vector. */
	Vector<T> segment(size_t start, size_t span)
	{
		T* wStart = mStart + start;
		T* wEnd = wStart + span;
		assert_no_opt(wEnd<=mEnd);
		return Vector<T>(NULL,wStart,wEnd);
	}

	/** Return an alias to a segment of this Vector. */
	const Vector<T> segment(size_t start, size_t span) const
	{
		T* wStart = mStart + start;
		T* wEnd = wStart + span;
		assert_no_opt(wEnd<=mEnd);
		return Vector<T>(NULL,wStart,wEnd);
	}

	Vector<T> head(size_t span) { return segment(0,span); }
	const Vector<T> head(size_t span) const { return segment(0,span); }
	Vector<T> tail(size_t start) { return segment(start,size()-start); }
	const Vector<T> tail(size_t start) const { return segment(start,size()-start); }

	/**
		Copy part of this Vector to a segment of another Vector.
		@param other The other vector.
		@param start The start point in the other vector.
		@param span The number of elements to copy.
	*/
	void copyToSegment(Vector<T>& other, size_t start, size_t span) const
	{
		unsigned int i;
		T* dst = other.mStart + start;
		T* src = mStart;
		assert_no_opt(dst+span<=other.mEnd);
		assert_no_opt(mStart+span<=mEnd);
		for (i = 0; i < span; i++, src++, dst++)
			*dst = *src;
		/*TODO if not non-trivially copiable type class, optimize:
		memcpy(dst,mStart,span*sizeof(T)); */
	}

	/** Copy all of this Vector to a segment of another Vector. */
	void copyToSegment(Vector<T>& other, size_t start=0) const { copyToSegment(other,start,size()); }

	void copyTo(Vector<T>& other) const { copyToSegment(other,0,size()); }

	/**
		Copy a segment of this vector into another.
		@param other The other vector (to copt into starting at 0.)
		@param start The start point in this vector.
		@param span The number of elements to copy.
	*/
	void segmentCopyTo(Vector<T>& other, size_t start, size_t span) const
	{
		const T* base = mStart + start;
		assert_no_opt(base+span<=mEnd);
		assert_no_opt(other.mStart+span<=other.mEnd);
		memcpy(other.mStart,base,span*sizeof(T));
	}

	/**
		Move (copy) a segment of this vector into a different position in the vector
		@param from Start point from which to copy.
		@param to   Start point to which to copy.
		@param span The number of elements to copy.
	*/
	void segmentMove(size_t from, size_t to, size_t span)
	{
		const T* baseFrom = mStart + from;
		T* baseTo = mStart + to;
		assert_no_opt(baseFrom+span<=mEnd);
		assert_no_opt(baseTo+span<=mEnd);
		memmove(baseTo,baseFrom,span*sizeof(T));
	}

	void fill(const T& val)
	{
		T* dp=mStart;
		while (dp<mEnd) *dp++=val;
	}

	void fill(const T& val, unsigned start, unsigned length)
	{
		T* dp=mStart+start;
		T* end=dp+length;
		assert_no_opt(end<=mEnd);
		while (dp<end) *dp++=val;
	}


	//@}


	//@{

	T& operator[](size_t index)
	{
		assert_no_opt(mStart+index<mEnd);
		return mStart[index];
	}

	const T& operator[](size_t index) const
	{
		assert_no_opt(mStart+index<mEnd);
		return mStart[index];
	}

	const T* begin() const { return mStart; }
	T* begin() { return mStart; }
	const T* end() const { return mEnd; }
	T* end() { return mEnd; }
	bool isOwner() { return !!mData; }	// Do we own any memory ourselves?
	//@}


};




/** Basic print operator for Vector objects. */
template <class T>
std::ostream& operator<<(std::ostream& os, const Vector<T>& v)
{
	for (unsigned i=0; i<v.size(); i++) os << v[i] << " ";
	return os;
}



#endif
// vim: ts=4 sw=4
