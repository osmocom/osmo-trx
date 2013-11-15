#ifndef _SIGNALVECTOR_H_
#define _SIGNALVECTOR_H_

#include <Vector.h>
#include <Complex.h>

/** Vector symmetry */
enum Symmetry {
  NONE = 0,
  ABSSYM = 1
};

class signalVector: public Vector<complex> {
public:
	/** Default constructor */
	signalVector(size_t size = 0);

	/** Construct with head room */
	signalVector(size_t size, size_t start);

	/** Construct from existing buffer data (buffer not managed) */
	signalVector(complex *data, size_t start, size_t span);

	/** Construct by from existing vector */
	signalVector(const signalVector &vector);

	/** Construct by from existing vector and append head-tail room */
	signalVector(const signalVector &vector, size_t start, size_t tail = 0);

	/** Override base assignment operator to include start offsets */
	void operator=(const signalVector& vector);

	/** Return head room */
	size_t getStart() const;

	Symmetry getSymmetry() const;
	void setSymmetry(Symmetry symmetry);

	bool isReal() const;
	void isReal(bool real);

	bool isAligned() const;
	void setAligned(bool aligned);

private:
	bool real;
	bool aligned;
	Symmetry symmetry;
};

#endif /* _SIGNALVECTOR_H_ */
