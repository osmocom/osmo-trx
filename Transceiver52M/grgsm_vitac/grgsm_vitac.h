
#pragma once

#include <vector>
#include "constants.h"

#if defined(__has_attribute)
#if __has_attribute(target_clones) && defined(__x86_64) && false
#define MULTI_VER_TARGET_ATTR __attribute__((target_clones("avx", "sse4.2", "sse3", "sse2", "sse", "default")))
#else
#define MULTI_VER_TARGET_ATTR
#endif
#endif

#define SYNC_SEARCH_RANGE 30
const int d_OSR(4);

void initvita();

int process_vita_burst(gr_complex *input, int tsc, unsigned char *output_binary);
int process_vita_sc_burst(gr_complex *input, int tsc, unsigned char *output_binary, int *offset);

MULTI_VER_TARGET_ATTR
void detect_burst(const gr_complex *input, gr_complex *chan_imp_resp, int burst_start, unsigned char *output_binary);
void gmsk_mapper(const unsigned char *input, int nitems, gr_complex *gmsk_output, gr_complex start_point);
gr_complex correlate_sequence(const gr_complex *sequence, int length, const gr_complex *input);
inline void autocorrelation(const gr_complex *input, gr_complex *out, int nitems);
inline void mafi(const gr_complex *input, int nitems, gr_complex *filter, int filter_length, gr_complex *output);
int get_sch_chan_imp_resp(const gr_complex *input, gr_complex *chan_imp_resp);
int get_norm_chan_imp_resp(const gr_complex *input, gr_complex *chan_imp_resp, float *corr_max, int bcc);
int get_sch_buffer_chan_imp_resp(const gr_complex *input, gr_complex *chan_imp_resp, unsigned int len, float *corr_max);

enum class btype { NB, SCH };
struct fdata {
	btype t;
	unsigned int fn;
	int tn;
	int bcc;
	std::string fpath;
	std::vector<gr_complex> data;
	unsigned int data_start_offset;
};