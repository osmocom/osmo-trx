#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "convolve.h"


// ---------------------------------------------------------------------------
// Misc utils
// ---------------------------------------------------------------------------

/* Generate some random values for testing */
static unsigned long rand_state = 0;

static void
rand_reset(void)
{
	rand_state = 0;
}

static unsigned long
rand_int(void)
{
	rand_state = (1103515245UL * rand_state + 12345UL) & 0x7fffffffUL;
	return rand_state;
}

static float
rand_float(void)
{
	union {
		uint32_t u;
		float f;
	} r;
	uint32_t u = rand_int();
	int e = 112 + ((u ^ (u>>8)) & 15);

	r.u  =  u & 0x007fffffUL;	// Mantissa
	r.u |= (u & 0x00800000UL) << 8;	// Sign
	r.u |= (e & 0xffUL) << 23;	// Exponent

	return r.f;
}

static void
gen_floats(float *vect, int len)
{
	int i;
	for (i = 0; i < len; i++)
		vect[i] = rand_float();
}

/* Show float vector data cut and paste friendly */
static void
dump_floats(float *vect, int len, char *name)
{
	int i;

	printf("static const float %s[] = {\n\t", name);
	for(i = 0; i < len; i++) {
		char *end;
		if (i == len-1)
			end = "\n";
		else if ((i&3) == 3)
			end = ",\n\t";
		else
			end = ", ";
		printf("%14.7ef%s", vect[i], end);
	}
	printf("};\n");
}

/* Compare float with tolerance of delta (absolute) and epsilon (relative) */
static int
compare_floats(const float *v0, const float *v1, int len, float delta, float epsilon)
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


// ---------------------------------------------------------------------------
// Golden reference results
// ---------------------------------------------------------------------------

#include "convolve_test_golden.h"

enum test_type {
	CONV_REAL_BASE		= 0,
	CONV_REAL_OPT		= 1,
	CONV_COMPLEX_BASE	= 2,
	CONV_COMPLEX_OPT	= 3
};

struct test_data {
	enum test_type type;
	int h_len;
	const  float *y_ref;
};

static const char *type_name[] = {
	"real_base", "real_opt", "complex_base", "complex_opt",
};

static const struct test_data tests[] = {
	{ CONV_REAL_BASE,  4, y_ref_real_base_4 },
	{ CONV_REAL_BASE,  8, y_ref_real_base_8 },
	{ CONV_REAL_BASE, 12, y_ref_real_base_12 },
	{ CONV_REAL_BASE, 16, y_ref_real_base_16 },
	{ CONV_REAL_BASE, 20, y_ref_real_base_20 },
	{ CONV_REAL_BASE, 24, y_ref_real_base_24 },
	{ CONV_COMPLEX_BASE,  4, y_ref_complex_base_4 },
	{ CONV_COMPLEX_BASE,  8, y_ref_complex_base_8 },
	{ CONV_COMPLEX_BASE, 12, y_ref_complex_base_12 },
	{ CONV_COMPLEX_BASE, 16, y_ref_complex_base_16 },
	{ CONV_COMPLEX_BASE, 20, y_ref_complex_base_20 },
	{ CONV_COMPLEX_BASE, 24, y_ref_complex_base_24 },
	{ 0, 0, NULL },
};


// ---------------------------------------------------------------------------
// Main testing logic
// ---------------------------------------------------------------------------

struct test_vec
{
	float *x;
	float *h;
	float *y;

	int x_len;	/* Theses are in # of _floats_ ! */
	int h_len;	/* Theses are in # of _floats_ ! */
	int y_len;	/* Theses are in # of _floats_ ! */
};

/* Reset test vectors */
static void
test_vec_reset(struct test_vec *tv, int seed)
{
	rand_reset();

	memset(tv->x, 0, tv->x_len * sizeof(float));
	memset(tv->h, 0, tv->h_len * sizeof(float));
	memset(tv->y, 0, tv->y_len * sizeof(float));

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
	tv->y = convolve_h_alloc(tv->y_len);

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
	free(tv->y);

	free(tv);
}

/* Run convolution */
static int
run_convolve(struct test_vec *tv, int h_len, enum test_type type)
{
	int x_len;
	int start, len;

	test_vec_reset(tv, 0);

	/* Compute params that fit within our test vectors */
	x_len = tv->x_len / 2; /* float vs complex */
	start = h_len - 1;
	len   = x_len - start;

	/* Run implementation */
	switch (type) {
	case CONV_REAL_BASE:
		base_convolve_real(
			tv->x, x_len,
			tv->h, h_len,
			tv->y, tv->y_len,
			start, len
		);
		break;

	case CONV_REAL_OPT:
		convolve_real(
			tv->x, x_len,
			tv->h, h_len,
			tv->y, tv->y_len,
			start, len
		);
		break;

	case CONV_COMPLEX_BASE:
		base_convolve_complex(
			tv->x, x_len,
			tv->h, h_len,
			tv->y, tv->y_len,
			start, len
		);
		break;

	case CONV_COMPLEX_OPT:
		convolve_complex(
			tv->x, x_len,
			tv->h, h_len,
			tv->y, tv->y_len,
			start, len
		);
		break;
	}

	return len * 2;
}


int main(int argc, char *argv[])
{
	struct test_vec *tv;
	int gen_ref_mode = 0;
	char name[80];
	int i, j, len;

	convolve_init();

	/* Mode */
	gen_ref_mode = (argc == 2) && !strcmp("genref", argv[1]);

	/* Alloc test vectors */
		/* All *2 is to account for the facts all vectors are actually
		 * complex and need two floats */
	tv = test_vec_alloc(100*2, 25*2);

	/* Dump all input data to make sure we work off the same input data */
	if (!gen_ref_mode) {
		printf("==== TEST INPUT DATA ====\n");
		dump_floats(tv->x, tv->x_len, "x");
		dump_floats(tv->h, tv->h_len, "h");
		printf("\n");
		printf("\n");
	}

	/* Run through all the tests */
	if (!gen_ref_mode)
		printf("==== TEST  ====\n");

	for (i=0; tests[i].h_len; i++)
	{
		for (j=0; j<(gen_ref_mode ? 1 : 2); j++)
		{
			len = run_convolve(tv, tests[i].h_len, tests[i].type + j);

			snprintf(name, sizeof(name)-1, "y_ref_%s_%d", type_name[tests[i].type + j], tests[i].h_len);

			if (gen_ref_mode)
			{
				/* If in generate mode, output data */
				dump_floats(tv->y, len, name);
			} else {
				/* If in test mode, compare with data */
				printf("%s: %s\n",
					name, 
					compare_floats(tests[i].y_ref, tv->y, len, 1e-5f, 1e-5f) ? "FAIL" : "PASS"
				);
			}
		}
	}

	if (!gen_ref_mode) {
		printf("\n");
		printf("\n");
	}

	/* All done ! */
	test_vec_release(tv);

	return 0;
}
