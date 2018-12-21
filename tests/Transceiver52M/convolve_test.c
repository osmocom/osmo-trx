#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "convolve.h"

struct test_vec
{
	float *x;
	float *h;
	float *y_ref;
	float *y_iut;

	int x_len;	/* Theses are in # of _floats_ ! */
	int h_len;	/* Theses are in # of _floats_ ! */
	int y_len;	/* Theses are in # of _floats_ ! */
};

/* Generate some random values for testing */
static void
gen_floats(float *vect, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		vect[i] = (float)rand()/(float)(RAND_MAX) - 0.5f;
	}
}

/* Show float vector data cut and paste friendly */
static void
dump_floats(float *vect, int len, char *name)
{
	int i;

	printf("float %s[] = {\n\t", name);
	for(i = 0; i < len; i++) {
		char *end;
		if (i == len-1)
			end = "\n";
		else if ((i&3) == 3)
			end = ",\n\t";
		else
			end = ", ";
		printf("%14.7e%s", vect[i], end);
	}
	printf("}\n");
}

/* Compare float with tolerance of delta (absolute) and epsilon (relative) */
static int
compare_floats(float *v0, float *v1, int len, float delta, float epsilon)
{
	int i;

	for (i=0; i<len; i++)
	{
		float a = v0[i];
		float b = v1[i];

		if (fabsf(a - b) < delta)
			continue;

		if (fabsf(1.0f - (a/b)) < epsilon)
			continue;

		return 1;
	}

	return 0;
}


/* Reset test vectors */
static void
test_vec_reset(struct test_vec *tv, int seed)
{
	srand(seed);

	memset(tv->x, 0, tv->x_len * sizeof(float));
	memset(tv->h, 0, tv->h_len * sizeof(float));
	memset(tv->y_ref, 0, tv->y_len * sizeof(float));
	memset(tv->y_iut, 0, tv->y_len * sizeof(float));

	gen_floats(tv->x, tv->x_len);
	gen_floats(tv->h, tv->h_len);
}

/* Allocate test vectors */
static struct test_vec *
test_vec_alloc(int x_len, int h_len)
{
	struct test_vec *tv;

	tv = calloc(1, sizeof(struct test_vec));
	if (!tv)
		return NULL;

	tv->x_len = x_len;
	tv->h_len = h_len;
	tv->y_len = x_len;	/* Results can never be longer than x */

	tv->x = convolve_h_alloc(x_len);
	tv->h = convolve_h_alloc(h_len);
	tv->y_ref = convolve_h_alloc(tv->y_len);
	tv->y_iut = convolve_h_alloc(tv->y_len);

	test_vec_reset(tv, 0);

	return tv;
}

/* Release test vectors */
static void
test_vec_release(struct test_vec *tv)
{
	if (!tv)
		return;

	free(tv->x);
	free(tv->h);
	free(tv->y_ref);
	free(tv->y_iut);

	free(tv);
}

/* Test complex convolution */
static void
test_convolve_complex(struct test_vec *tv, int h_len)
{
	int x_len;
	int start, len;
	int rv;

	test_vec_reset(tv, 0);

	/* Compute params that fit within our test vectors */
	x_len = tv->x_len / 2; /* float vs complex */
	start = h_len - 1;
	len   = x_len - start;

	/* Run both 'base/ref' implementation and the potentially optimized one */
	base_convolve_complex(
		tv->x, x_len,
		tv->h, h_len,
		tv->y_ref, tv->y_len,
		start, len
	);

	convolve_complex(
		tv->x, x_len,
		tv->h, h_len,
		tv->y_iut, tv->y_len,
		start, len
	);

	/* Print the 'ref' results. Those should be consistent across platforms */
	dump_floats(tv->y_ref, 2 * len, "y_ref");

	/* Compare to 'iut' ones with small tolerance for precision */
	rv = compare_floats(tv->y_ref, tv->y_iut, len * 2, 1e-5f, 1e-5f);
	printf("IUT: %s\n", rv ? "!!! FAIL !!!" : "PASS");

	if (rv)
		dump_floats(tv->y_iut, 2 * len, "y_iut");
}

/* Test real convolution */
static void
test_convolve_real(struct test_vec *tv, int h_len)
{
	int x_len;
	int start, len;
	int rv;

	test_vec_reset(tv, 0);

	/* Compute params that fit within our test vectors */
	x_len = tv->x_len / 2; /* float vs complex */
	start = h_len - 1;
	len   = x_len - start;

	/* Run both 'base/ref' implementation and the potentially optimized one */
	base_convolve_real(
		tv->x, x_len,
		tv->h, h_len,
		tv->y_ref, tv->y_len,
		start, len
	);

	convolve_real(
		tv->x, x_len,
		tv->h, h_len,
		tv->y_iut, tv->y_len,
		start, len
	);

	/* Print the 'ref' results. Those should be consistent across platforms */
	dump_floats(tv->y_ref, 2 * len, "y_ref");

	/* Compare to 'iut' ones with small tolerance for precision */
	rv = compare_floats(tv->y_ref, tv->y_iut, len * 2, 1e-5f, 1e-5f);
	printf("IUT: %s\n", rv ? "!!! FAIL !!!" : "PASS");

	if (rv)
		dump_floats(tv->y_iut, 2 * len, "y_iut");
}

int main(void)
{
	struct test_vec *tv;
	int i;

	convolve_init();

	/* Alloc test vectors */
		/* All *2 is to account for the facts all vectors are actually
		 * complex and need two floats */
	tv = test_vec_alloc(100*2, 25*2);

	/* Dump all input data to make sure we work off the same input data */
	printf("==== TEST INPUT DATA ====\n");
	dump_floats(tv->x, tv->x_len, "x");
	dump_floats(tv->h, tv->h_len, "h");
	printf("\n");
	printf("\n");

	/* Test complex */
	printf("==== TEST COMPLEX ====\n");

	for (i=4; i<=24; i+=4)
	{
		printf(" -- h_len = %d --\n", i);
		test_convolve_complex(tv, i);
		printf("\n");
	}

	printf(" -- h_len = %d --\n", 25);
	test_convolve_complex(tv, 25);

	printf("\n");
	printf("\n");

	/* Test real */
	printf("==== TEST REAL ====\n");

	for (i=4; i<=24; i+=4)
	{
		printf(" -- h_len = %d --\n", i);
		test_convolve_real(tv, i);
		printf("\n");
	}

	printf(" -- h_len = %d --\n", 25);
	test_convolve_real(tv, 25);

	printf("\n");
	printf("\n");

	/* All done ! */
	test_vec_release(tv);

	return 0;
}
