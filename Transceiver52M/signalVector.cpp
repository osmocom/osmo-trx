#include "signalVector.h"

signalVector::signalVector(size_t size)
	: Vector<complex>(size),
	  real(false), aligned(false), symmetry(NONE)
{
}

signalVector::signalVector(size_t size, size_t start)
	: Vector<complex>(size + start),
	  real(false), aligned(false), symmetry(NONE)
{
	mStart = mData + start;
}

signalVector::signalVector(complex *data, size_t start, size_t span)
	: Vector<complex>(NULL, data + start, data + start + span),
	  real(false), aligned(false), symmetry(NONE)
{
}

signalVector::signalVector(const signalVector &vector)
	: Vector<complex>(vector.size() + vector.getStart()), aligned(false)
{
	mStart = mData + vector.getStart();
	vector.copyTo(*this);
	symmetry = vector.getSymmetry();
	real = vector.isReal();
};

signalVector::signalVector(const signalVector &vector,
			   size_t start, size_t tail)
	: Vector<complex>(start + vector.size() + tail), aligned(false)
{
	mStart = mData + start;
	vector.copyTo(*this);
	symmetry = vector.getSymmetry();
	real = vector.isReal();
};

void signalVector::operator=(const signalVector& vector)
{
	resize(vector.size() + vector.getStart());

	unsigned int i;
	complex *dst = mData;
	complex *src = vector.mData;
	for (i = 0; i < size(); i++, src++, dst++)
		*dst = *src;
	/* TODO: optimize for non non-trivially copyable types: */
	/*memcpy(mData, vector.mData, bytes()); */
	mStart = mData + vector.getStart();
}

signalVector signalVector::segment(size_t start, size_t span)
{
	return signalVector(mData, start, span);
}

size_t signalVector::getStart() const
{
	return mStart - mData;
}

size_t signalVector::updateHistory()
{
	size_t num = getStart();
	unsigned int i;
	complex *dst = mData;
	complex *src = mStart + this->size() - num;
	for (i = 0; i < num; i++, src++, dst++)
		*dst = *src;
	/* TODO: optimize for non non-trivially copyable types: */
	/*memmove(mData, mStart + this->size() - num, num * sizeof(complex)); */

	return num;
}

Symmetry signalVector::getSymmetry() const
{
	return symmetry;
}

void signalVector::setSymmetry(Symmetry symmetry)
{
	this->symmetry = symmetry;
}

bool signalVector::isReal() const
{
	return real;
}

void signalVector::isReal(bool wOnly)
{
	real = wOnly;
}

bool signalVector::isAligned() const
{
	return aligned;
}

void signalVector::setAligned(bool aligned)
{
	this->aligned = aligned;
}
