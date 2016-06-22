#ifndef _CHANNELIZER_RX_H_
#define _CHANNELIZER_RX_H_

#include "ChannelizerBase.h"

class Channelizer : public ChannelizerBase {
public:
	/** Constructor for channelizing filter bank
	    @param m number of physical channels
	    @param blockLen number of samples per output of each iteration
	    @param hLen number of taps in each constituent filter path
	*/
	Channelizer(size_t m, size_t blockLen, size_t hLen = 16);
	~Channelizer();

	/* Return required input and output buffer lengths */
	size_t inputLen() const;
	size_t outputLen() const;

	/** Rotate "input commutator" and drive samples through filterbank
	    @param in complex input vector 
	    @param iLen number of samples in buffer (must match block length)
	    @return false on error and true otherwise
	*/
	bool rotate(const float *in, size_t iLen);

	/** Get buffer for an output path
	    @param chan channel number of filterbank
            @return NULL on error and pointer to buffer otherwise
	*/
	float *outputBuffer(size_t chan) const;
};

#endif /* _CHANNELIZER_RX_H_ */
