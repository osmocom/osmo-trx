#pragma once
/* -*- c++ -*- */
/*
 * @file
 * @author (C) 2009-2017  by Piotr Krysik <ptrkrysik@gmail.com>
 * @section LICENSE
 *
 * Gr-gsm is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Gr-gsm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gr-gsm; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <vector>
#include "constants.h"

/* may only be used for for the DEFINITIONS!
* see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=91664
*/
#if defined(__has_attribute)
#if __has_attribute(target_clones) && defined(__x86_64) && true
#define MULTI_VER_TARGET_ATTR __attribute__((target_clones("avx", "sse4.2", "sse3", "sse2", "sse", "default")))
#else
#define MULTI_VER_TARGET_ATTR
#endif
#endif

/* ... but apparently clang disagrees... */
#if defined(__clang__)
#define MULTI_VER_TARGET_ATTR_CLANGONLY MULTI_VER_TARGET_ATTR
#else
#define MULTI_VER_TARGET_ATTR_CLANGONLY
#endif

#if defined(__has_attribute)
#if __has_attribute(no_sanitize)
#define NO_UBSAN __attribute__((no_sanitize("undefined")))
#endif
#else
#define NO_UBSAN
#endif

#define SYNC_SEARCH_RANGE 30
const int d_OSR(4);

void initvita();

int process_vita_burst(gr_complex *input, int tsc, unsigned char *output_binary);
int process_vita_sc_burst(gr_complex *input, int tsc, unsigned char *output_binary, int *offset);

MULTI_VER_TARGET_ATTR_CLANGONLY
void detect_burst(const gr_complex *input, gr_complex *chan_imp_resp, int burst_start, char *output_binary);
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