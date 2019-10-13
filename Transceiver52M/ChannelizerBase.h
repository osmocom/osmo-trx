#ifndef _CHANNELIZER_BASE_H_
#define _CHANNELIZER_BASE_H_

class ChannelizerBase {
protected:
	ChannelizerBase(size_t m, size_t blockLen, size_t hLen);
	~ChannelizerBase();

	/* Channelizer parameters */
	size_t m;
	size_t hLen;
	size_t blockLen;

	/* Channelizer filterbank sub-filters */
	float **subFilters;

	/* Input/Output buffers */
	float **hInputs, **hOutputs, **hist;
	float *fftInput, *fftOutput;

	/* Pointer to opaque FFT instance */
	struct fft_hdl *fftHandle;

	/* Initializer internals */
	bool initFilters();
	bool initFFT();
	void releaseFilters();

	/* Map overlapped FFT and filter I/O buffers */
	bool mapBuffers();

	/* Buffer length validity checking */
	bool checkLen(size_t innerLen, size_t outerLen);
public:
	/* Initialize channelizer/synthesis filter internals */
	bool init();
};

#endif /* _CHANNELIZER_BASE_H_ */
