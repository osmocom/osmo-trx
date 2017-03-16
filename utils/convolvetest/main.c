#include <stdio.h>
#include <string.h>
#include <osmocom/core/utils.h>
#include "../../Transceiver52M/common/convolve.h"

#define TESTVEC_LEN 1000
#define DO_INIT 1

float x_vect[TESTVEC_LEN];
float y_vect[TESTVEC_LEN];
float h_vect[TESTVEC_LEN];

float *x;
float *h;
float *y;

/* Generate some random values for testing */
void gen_floats(float *vect, int len)
{
	int i;
	for(i=0;i<len;i++) {
		vect[i] = (float)rand()/(float)(RAND_MAX);
	}
}

/* Reset testvectors */
static void reset_testvec(int seed)
{
	srand(seed);
	memset(x_vect,0,sizeof(x_vect));
	memset(y_vect,0,sizeof(y_vect));
	memset(h_vect,0,sizeof(h_vect));

	x=x_vect + TESTVEC_LEN/2;
	y=y_vect + TESTVEC_LEN/2;
	h=h_vect + TESTVEC_LEN/2;

	gen_floats(x_vect,TESTVEC_LEN);
	gen_floats(h_vect,TESTVEC_LEN);
}

/* Show float vector data cut and paste friendly */
static void dump_floats(float *vect, int len, char *name)
{
	int i;

	printf("float %s[] = {", name);
	for(i=0;i<len;i++) {

		printf("%f",vect[i]);

		if(i<len-1)
			printf(",");
	}
	printf("}\n");
}

/* Test complex convolution */
static void test_convolve_complex(int h_len)
{
	int x_len;
	int y_len;
	int start;
	int len;
	int step;
	int offset;

	x_len=34;
	y_len=26;
	start=8;
	len=26;
	step=1;
	offset=1;
	reset_testvec(0);
	dump_floats(x,x_len,"x");
	printf("\n");
	dump_floats(h,h_len,"h");
	printf("\n");
	convolve_complex(x, x_len, h, h_len, y, y_len, start, len, step, offset);
	dump_floats(y,y_len,"y");
	printf("\n");
}

/* Test real convolution */
static void test_convolve_real(int h_len)
{
	int x_len;
	int y_len;
	int start;
	int len;
	int step;
	int offset;

	x_len=34;
	y_len=26;
	start=8;
	len=26;
	step=1;
	offset=1;
	reset_testvec(0);
	dump_floats(x,x_len,"x");
	printf("\n");
	dump_floats(h,h_len,"h");
	printf("\n");
	convolve_real(x, x_len, h, h_len, y, y_len, start, len, step, offset);
	dump_floats(y,y_len,"y");
	printf("\n");
}

int main(void)
{
#if DO_INIT == 1
	convolve_init();
#endif

	printf("==== TEST COMPLEX BASE IMPLEMENTATION ====\n");
	test_convolve_complex(17);

	printf("==== TEST COMPLEX SSE3 IMPLEMENTATION: (h_len%%4=0) ====\n");
	test_convolve_complex(20);

	printf("==== TEST COMPLEX SSE3 IMPLEMENTATION: (h_len%%8=0) ====\n");
	test_convolve_complex(16);

	printf("\n");
	printf("\n");

	printf("==== TEST REAL BASE IMPLEMENTATION ====\n");
	test_convolve_real(17);

	printf("==== TEST REAL SSE3 IMPLEMENTATION (hlen=4) ====\n");
	test_convolve_real(4);

	printf("==== TEST REAL SSE3 IMPLEMENTATION (hlen=8) ====\n");
	test_convolve_real(8);

	printf("==== TEST REAL SSE3 IMPLEMENTATION (hlen=12) ====\n");
	test_convolve_real(12);

	printf("==== TEST REAL SSE3 IMPLEMENTATION (hlen=16) ====\n");
	test_convolve_real(16);

	printf("==== TEST REAL SSE3 IMPLEMENTATION (hlen=20) ====\n");
	test_convolve_real(20);

	printf("==== TEST REAL SSE3 IMPLEMENTATION (h_len%%4=0) ====\n");
	test_convolve_real(24);

	return 0;
}
