#ifndef _FFT_H_
#define _FFT_H_

struct fft_hdl;

struct fft_hdl *init_fft(int reverse, int m, int istride, int ostride,
			 float *in, float *out, int ooffset);
void *fft_malloc(size_t size);
void fft_free(void *ptr);
void free_fft(struct fft_hdl *hdl);
int cxvec_fft(struct fft_hdl *hdl);

#endif /* _FFT_H_ */
