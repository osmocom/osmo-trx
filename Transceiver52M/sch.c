#include <complex.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/conv.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/crcgen.h>

#include "sch.h"

/* GSM 04.08, 9.1.30 Synchronization channel information */
struct sch_packed_info {
	ubit_t t1_hi[2];
	ubit_t bsic[6];
	ubit_t t1_md[8];
	ubit_t t3p_hi[2];
	ubit_t t2[5];
	ubit_t t1_lo[1];
	ubit_t t3p_lo[1];
} __attribute__((packed));

struct sch_burst {
	sbit_t tail0[3];
	sbit_t data0[39];
	sbit_t etsc[64];
	sbit_t data1[39];
	sbit_t tail1[3];
	sbit_t guard[8];
} __attribute__((packed));

static const uint8_t sch_next_output[][2] = {
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 3, 0 }, { 2, 1 }, { 3, 0 }, { 2, 1 },
	{ 0, 3 }, { 1, 2 }, { 0, 3 }, { 1, 2 },
};

static const uint8_t sch_next_state[][2] = {
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
	{  0,  1 }, {  2,  3 }, {  4,  5 }, {  6,  7 },
	{  8,  9 }, { 10, 11 }, { 12, 13 }, { 14, 15 },
};

static const struct osmo_conv_code gsm_conv_sch = {
	.N = 2,
	.K = 5,
	.len = GSM_SCH_UNCODED_LEN,
	.next_output = sch_next_output,
	.next_state  = sch_next_state,
};

const struct osmo_crc16gen_code gsm0503_sch_crc10 = {
	.bits = 10,
	.poly = 0x175,
	.init = 0x000,
	.remainder = 0x3ff,
};

#define GSM_MAX_BURST_LEN	157 * 4
#define GSM_SYM_RATE		(1625e3 / 6) * 4

/* Pre-generated FCCH measurement tone */
static complex float fcch_ref[GSM_MAX_BURST_LEN];

int float_to_sbit(const float *in, sbit_t *out, float scale, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		out[i] = (in[i] - 0.5f) * scale;
	}

	return 0;
}

/* Check if FN contains a FCCH burst */
int gsm_fcch_check_fn(int fn)
{
	int fn51 = fn % 51;

	switch (fn51) {
	case 0:
	case 10:
	case 20:
	case 30:
	case 40:
		return 1;
	}

	return 0;
}

/* Check if FN contains a SCH burst */
int gsm_sch_check_fn(int fn)
{
	int fn51 = fn % 51;

	switch (fn51) {
	case 1:
	case 11:
	case 21:
	case 31:
	case 41:
		return 1;
	}

	return 0;
}

/* SCH (T1, T2, T3p) to full FN value */
int gsm_sch_to_fn(struct sch_info *sch)
{
	int t1 = sch->t1;
	int t2 = sch->t2;
	int t3p = sch->t3p;

	if ((t1 < 0) || (t2 < 0) || (t3p < 0))
		return -1;
	int tt;
	int t3 = t3p * 10 + 1;

	if (t3 < t2)
		tt = (t3 + 26) - t2;
	else
		tt = (t3 - t2) % 26;

	return t1 * 51 * 26 + tt * 51 + t3;
}

/* Parse encoded SCH message */
int gsm_sch_parse(const uint8_t *info, struct sch_info *desc)
{
	struct sch_packed_info *p = (struct sch_packed_info *) info;

	desc->bsic = (p->bsic[0] << 0) | (p->bsic[1] << 1) |
		     (p->bsic[2] << 2) | (p->bsic[3] << 3) |
		     (p->bsic[4] << 4) | (p->bsic[5] << 5);

	desc->t1 = (p->t1_lo[0] << 0) | (p->t1_md[0] << 1) |
		   (p->t1_md[1] << 2) | (p->t1_md[2] << 3) |
		   (p->t1_md[3] << 4) | (p->t1_md[4] << 5) |
		   (p->t1_md[5] << 6) | (p->t1_md[6] << 7) |
		   (p->t1_md[7] << 8) | (p->t1_hi[0] << 9) |
		   (p->t1_hi[1] << 10);

	desc->t2 = (p->t2[0] << 0) | (p->t2[1] << 1) |
		   (p->t2[2] << 2) | (p->t2[3] << 3) |
		   (p->t2[4] << 4);

	desc->t3p = (p->t3p_lo[0] << 0) | (p->t3p_hi[0] << 1) |
		    (p->t3p_hi[1] << 2);

	return 0;
}

/* From osmo-bts */
int gsm_sch_decode(uint8_t *info, sbit_t *data)
{
	int rc;
	ubit_t uncoded[GSM_SCH_UNCODED_LEN];

	osmo_conv_decode(&gsm_conv_sch, data, uncoded);

	rc = osmo_crc16gen_check_bits(&gsm0503_sch_crc10,
				      uncoded, GSM_SCH_INFO_LEN,
				      uncoded + GSM_SCH_INFO_LEN);
	if (rc)
		return -1;

	memcpy(info, uncoded, GSM_SCH_INFO_LEN * sizeof(ubit_t));

	return 0;
}

#define FCCH_TAIL_BITS_LEN	3*4
#define FCCH_DATA_LEN	100*4//	142
#if 1
/* Compute FCCH frequency offset */
double org_gsm_fcch_offset(float *burst, int len)
{
	int i, start, end;
	float a, b, c, d, ang, avg = 0.0f;
	double freq;

	if (len > GSM_MAX_BURST_LEN)
		len = GSM_MAX_BURST_LEN;

	for (i = 0; i < len; i++) {
		a = burst[2 * i + 0];
		b = burst[2 * i + 1];
		c = crealf(fcch_ref[i]);
		d = cimagf(fcch_ref[i]);

		burst[2 * i + 0] = a * c - b * d;
		burst[2 * i + 1] = a * d + b * c;
	}

	start = FCCH_TAIL_BITS_LEN;
	end = start + FCCH_DATA_LEN;

	for (i = start; i < end; i++) {
		a = cargf(burst[2 * (i - 1) + 0] +
			  burst[2 * (i - 1) + 1] * I);
		b = cargf(burst[2 * i + 0] +
			  burst[2 * i + 1] * I);

		ang = b - a;

		if (ang > M_PI)
			ang -= 2 * M_PI;
		else if (ang < -M_PI)
			ang += 2 * M_PI;

		avg += ang;
	}

	avg /= (float) (end - start);
	freq = avg / (2 * M_PI) * GSM_SYM_RATE;

	return freq;
}


static const int L1 = 3;
static const int L2 = 32;
static const int N1 = 92;
static const int N2 = 92;

static struct { int8_t r; int8_t s; } P_inv_table[3+32];

void pinv(int P, int8_t* r, int8_t* s, int L1, int L2) {
	for (int i = 0; i < L1; i++)
		for (int j = 0; j < L2; j++)
			if (P == L2 * i - L1 * j) {
				*r = i;
				*s = j;
				return;
			}
}


float ac_sum_with_lag( complex float* in, int lag, int offset, int N) {
	complex float v = 0 + 0*I;
	int total_offset = offset + lag;
	for (int s = 0; s < N; s++)
		v += in[s + total_offset] * conjf(in[s + total_offset - lag]);
	return cargf(v);
}


double gsm_fcch_offset(float *burst, int len)
{
	int start;

	const float fs = 13. / 48. * 1e6 * 4;
	const float expected_fcch_val = ((2 * M_PI) / (fs)) * 67700;

	if (len > GSM_MAX_BURST_LEN)
		len = GSM_MAX_BURST_LEN;

	start = FCCH_TAIL_BITS_LEN+10 * 4;
	float alpha_one = ac_sum_with_lag((complex float*)burst, L1, start, N1);
	float alpha_two = ac_sum_with_lag((complex float*)burst, L2, start, N2);

	float P_unrounded = (L1 * alpha_two - L2 * alpha_one) / (2 * M_PI);
	int P = roundf(P_unrounded);

	int8_t r = 0, s = 0;
	pinv(P, &r, &s, L1, L2);

	float omegal1 = (alpha_one + 2 * M_PI * r) / L1;
	float omegal2 = (alpha_two + 2 * M_PI * s) / L2;

	float rv = org_gsm_fcch_offset(burst, len);
	//return rv;

	float reval = GSM_SYM_RATE / (2 * M_PI) * (expected_fcch_val - (omegal1+omegal2)/2);
	//fprintf(stderr, "XX rv %f %f %f %f\n", rv, reval, omegal1 / (2 * M_PI) * fs, omegal2 / (2 * M_PI) * fs);

	//fprintf(stderr, "XX rv %f %f\n", rv, reval);

	return -reval;
}
#endif
/* Generate FCCH measurement tone */
static __attribute__((constructor)) void init()
{
	int i;
	double freq = 0.25;

	for (i = 0; i < GSM_MAX_BURST_LEN; i++) {
		fcch_ref[i] = sin(2 * M_PI * freq * (double) i) +
			      cos(2 * M_PI * freq * (double) i) * I;
	}

}
