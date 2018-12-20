#ifndef _CONVOLVE_H_
#define _CONVOLVE_H_

void *convolve_h_alloc(size_t num);

int convolve_real(const float *x, int x_len,
		  const float *h, int h_len,
		  float *y, int y_len,
		  int start, int len);

int convolve_complex(const float *x, int x_len,
		     const float *h, int h_len,
		     float *y, int y_len,
		     int start, int len);

int base_convolve_real(const float *x, int x_len,
		       const float *h, int h_len,
		       float *y, int y_len,
		       int start, int len);

int base_convolve_complex(const float *x, int x_len,
			  const float *h, int h_len,
			  float *y, int y_len,
			  int start, int len);

void convolve_init(void);

#endif /* _CONVOLVE_H_ */
