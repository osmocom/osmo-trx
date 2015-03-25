#ifndef _CONVOLVE_H_
#define _CONVOLVE_H_

void *convolve_h_alloc(int num);

int convolve_real(const float *x, int x_len,
		  const float *h, int h_len,
		  float *y, int y_len,
		  int start, int len,
		  int step, int offset);

int convolve_complex(const float *x, int x_len,
		     const float *h, int h_len,
		     float *y, int y_len,
		     int start, int len,
		     int step, int offset);

int base_convolve_real(const float *x, int x_len,
		       const float *h, int h_len,
		       float *y, int y_len,
		       int start, int len,
		       int step, int offset);

int base_convolve_complex(const float *x, int x_len,
			  const float *h, int h_len,
			  float *y, int y_len,
			  int start, int len,
			  int step, int offset);

#endif /* _CONVOLVE_H_ */
