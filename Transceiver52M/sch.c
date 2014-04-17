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

#define GSM_MAX_BURST_LEN	157
#define GSM_SYM_RATE		(1625e3 / 6)

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
		     (p->bsic[4] << 4);

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

#define FCCH_TAIL_BITS_LEN	3
#define FCCH_DATA_LEN		142

/* Compute FCCH frequency offset */
double gsm_fcch_offset(float *burst, int len)
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
