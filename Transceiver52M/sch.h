#ifndef _SCH_H_
#define _SCH_H_

#include <osmocom/core/bits.h>

struct sch_info  {
	int bsic;
	int t1;
	int t2;
	int t3p;
};

#define GSM_SCH_INFO_LEN		25
#define GSM_SCH_UNCODED_LEN		35
#define GSM_SCH_CODED_LEN		78

int gsm_sch_decode(uint8_t *sb_info, sbit_t *burst);
int gsm_sch_parse(const uint8_t *sb_info, struct sch_info *desc);
int gsm_sch_to_fn(struct sch_info *sch);
int gsm_sch_check_fn(int fn);

int float_to_sbit(const float *in, sbit_t *out, float scale, int len);

#endif /* _SCH_H_ */
