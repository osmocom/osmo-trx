#ifndef _CONVERT_H_
#define _CONVERT_H_

void convert_float_short(short *out, const float *in, float scale, int len);

void convert_short_float(float *out, const short *in, int len);

void base_convert_float_short(short *out, const float *in,
			      float scale, int len);

void base_convert_short_float(float *out, const short *in, int len);

void convert_init(void);

#endif /* _CONVERT_H_ */
