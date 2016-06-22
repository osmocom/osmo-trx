#ifndef _SYNTHESIS_H_
#define _SYNTHESIS_H_

#include "ChannelizerBase.h"

class Synthesis : public ChannelizerBase {
public:
	/** Constructor for synthesis filterbank
	    @param m number of physical channels
	    @param blockLen number of samples per output of each iteration
	    @param hLen number of taps in each constituent filter path
	*/
	Synthesis(size_t m, size_t blockLen, size_t hLen = 16);
	~Synthesis();

	/* Return required input and output buffer lengths */
	size_t inputLen() const;
	size_t outputLen() const;

	/** Rotate "output commutator" and drive samples through filterbank
	    @param out complex output vector 
	    @param oLen number of samples in buffer (must match block length * m)
	    @return false on error and true otherwise
	*/
	bool rotate(float *out, size_t oLen);

	/** Get buffer for an input path
	    @param chan channel number of filterbank
            @return NULL on error and pointer to buffer otherwise
	*/
	float *inputBuffer(size_t chan) const;
	bool resetBuffer(size_t chan);
};

#endif /* _SYNTHESIS_H_ */
