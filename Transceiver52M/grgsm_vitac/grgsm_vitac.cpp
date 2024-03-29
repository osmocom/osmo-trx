/* -*- c++ -*- */
/*
 * @file
 * @author (C) 2009-2017  by Piotr Krysik <ptrkrysik@gmail.com>
 * @author Contributions by sysmocom - s.f.m.c. GmbH / Eric Wild <ewild@sysmocom.de>
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

#include "constants.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <complex>


#include <algorithm>
#include <string.h>
#include <iostream>
#include <numeric>
#include <vector>
#include <fstream>

#include "viterbi_detector.h"
#include "grgsm_vitac.h"

gr_complex d_acc_training_seq[N_ACCESS_BITS]; ///<encoded training sequence of a RACH burst
gr_complex d_sch_training_seq[N_SYNC_BITS]; ///<encoded training sequence of a SCH burst
gr_complex d_norm_training_seq[TRAIN_SEQ_NUM][N_TRAIN_BITS]; ///<encoded training sequences of a normal and dummy burst
const int d_chan_imp_length = CHAN_IMP_RESP_LENGTH;

void initvita()
{
	/**
	 * Prepare SCH sequence bits
	 *
	 * (TS_BITS + 2 * GUARD_PERIOD)
	 * Burst and two guard periods
	 * (one guard period is an arbitrary overlap)
	 */
	gmsk_mapper(SYNC_BITS, N_SYNC_BITS, d_sch_training_seq, gr_complex(0.0, -1.0));
	for (auto &i : d_sch_training_seq)
		i = conj(i);

	/* ab */
	gmsk_mapper(ACCESS_BITS, N_ACCESS_BITS, d_acc_training_seq, gr_complex(0.0, -1.0));
	for (auto &i : d_acc_training_seq)
		i = conj(i);

	/* Prepare bits of training sequences */
	for (int i = 0; i < TRAIN_SEQ_NUM; i++) {
		/**
		 * If first bit of the sequence is 0
		 * => first symbol is 1, else -1
		 */
		gr_complex startpoint = train_seq[i][0] == 0 ? gr_complex(1.0, 0.0) : gr_complex(-1.0, 0.0);
		gmsk_mapper(train_seq[i], N_TRAIN_BITS, d_norm_training_seq[i], startpoint);
		for (auto &i : d_norm_training_seq[i])
			i = conj(i);
	}
}

template <unsigned int burst_size>
NO_UBSAN static void detect_burst_generic(const gr_complex *input, gr_complex *chan_imp_resp, int burst_start,
					  char *output_binary, int ss)
{
	std::vector<gr_complex> rhh_temp(CHAN_IMP_RESP_LENGTH * d_OSR);
	unsigned int stop_states[2] = { 4, 12 };
	gr_complex filtered_burst[burst_size];
	gr_complex rhh[CHAN_IMP_RESP_LENGTH];
	float output[burst_size];
	int start_state = ss;

	autocorrelation(chan_imp_resp, &rhh_temp[0], d_chan_imp_length * d_OSR);
	for (int ii = 0; ii < d_chan_imp_length; ii++)
		rhh[ii] = conj(rhh_temp[ii * d_OSR]);

	mafi(&input[burst_start], burst_size, chan_imp_resp, d_chan_imp_length * d_OSR, filtered_burst);

	viterbi_detector(filtered_burst, burst_size, rhh, start_state, stop_states, 2, output);

	for (unsigned int i = 0; i < burst_size; i++)
		output_binary[i] = output[i] > 0 ? -127 : 127; // pre flip bits!
}

NO_UBSAN void detect_burst_nb(const gr_complex *input, gr_complex *chan_imp_resp, int burst_start, char *output_binary,
			      int ss)
{
	return detect_burst_generic<BURST_SIZE>(input, chan_imp_resp, burst_start, output_binary, ss);
}
NO_UBSAN void detect_burst_ab(const gr_complex *input, gr_complex *chan_imp_resp, int burst_start, char *output_binary,
			      int ss)
{
	return detect_burst_generic<8 + 41 + 36 + 3>(input, chan_imp_resp, burst_start, output_binary, ss);
}

NO_UBSAN void detect_burst_nb(const gr_complex *input, gr_complex *chan_imp_resp, int burst_start, char *output_binary)
{
	return detect_burst_nb(input, chan_imp_resp, burst_start, output_binary, 3);
}
NO_UBSAN void detect_burst_ab(const gr_complex *input, gr_complex *chan_imp_resp, int burst_start, char *output_binary)
{
	return detect_burst_ab(input, chan_imp_resp, burst_start, output_binary, 3);
}

void gmsk_mapper(const unsigned char *input, int nitems, gr_complex *gmsk_output, gr_complex start_point)
{
	gr_complex j = gr_complex(0.0, 1.0);
	gmsk_output[0] = start_point;

	int previous_symbol = 2 * input[0] - 1;
	int current_symbol;
	int encoded_symbol;

	for (int i = 1; i < nitems; i++) {
		/* Change bits representation to NRZ */
		current_symbol = 2 * input[i] - 1;

		/* Differentially encode */
		encoded_symbol = current_symbol * previous_symbol;

		/* And do GMSK mapping */
		gmsk_output[i] = j * gr_complex(encoded_symbol, 0.0) * gmsk_output[i - 1];

		previous_symbol = current_symbol;
	}
}

gr_complex correlate_sequence(const gr_complex *sequence, int length, const gr_complex *input)
{
	gr_complex result(0.0, 0.0);

	for (int ii = 0; ii < length; ii++)
		result += sequence[ii] * input[ii * d_OSR];

	return conj(result) / gr_complex(length, 0);
}

/* Computes autocorrelation for positive arguments */
inline void autocorrelation(const gr_complex *input, gr_complex *out, int nitems)
{
	for (int k = nitems - 1; k >= 0; k--) {
		out[k] = gr_complex(0, 0);
		for (int i = k; i < nitems; i++)
			out[k] += input[i] * conj(input[i - k]);
	}
}

inline void mafi(const gr_complex *input, int nitems, gr_complex *filter, int filter_length, gr_complex *output)
{
	for (int n = 0; n < nitems; n++) {
		int a = n * d_OSR;
		output[n] = 0;

		for (int ii = 0; ii < filter_length; ii++) {
			if ((a + ii) >= nitems * d_OSR)
				break;

			output[n] += input[a + ii] * filter[ii];
		}
	}
}

int get_chan_imp_resp(const gr_complex *input, gr_complex *chan_imp_resp, int search_start_pos, int search_stop_pos,
		      gr_complex *tseq, int tseqlen, float *corr_max)
{
	const int num_search_windows = search_stop_pos - search_start_pos;
	const int power_search_window_len = d_chan_imp_length * d_OSR;
	std::vector<float> window_energy_buffer;
	std::vector<float> power_buffer;
	std::vector<gr_complex> correlation_buffer;

	power_buffer.reserve(num_search_windows);
	correlation_buffer.reserve(num_search_windows);
	window_energy_buffer.reserve(num_search_windows);

	for (int ii = 0; ii < num_search_windows; ii++) {
		gr_complex correlation = correlate_sequence(tseq, tseqlen, &input[search_start_pos + ii]);
		correlation_buffer.push_back(correlation);
		power_buffer.push_back(std::pow(abs(correlation), 2));
	}

	/* Compute window energies */
	float windowSum = 0;

	// first window
	for (int i = 0; i < power_search_window_len; i++) {
		windowSum += power_buffer[i];
	}
	window_energy_buffer.push_back(windowSum);

	// slide windows
	for (int i = power_search_window_len; i < num_search_windows; i++) {
		windowSum += power_buffer[i] - power_buffer[i - power_search_window_len];
		window_energy_buffer.push_back(windowSum);
	}

	int strongest_window_nr = std::max_element(window_energy_buffer.begin(), window_energy_buffer.end()) -
				  window_energy_buffer.begin();

	float max_correlation = 0;
	for (int ii = 0; ii < power_search_window_len; ii++) {
		gr_complex correlation = correlation_buffer[strongest_window_nr + ii];
		if (abs(correlation) > max_correlation)
			max_correlation = abs(correlation);
		chan_imp_resp[ii] = correlation;
	}

	*corr_max = max_correlation;

	/**
	 * Compute first sample position, which corresponds
	 * to the first sample of the impulse response
	 */
	return search_start_pos + strongest_window_nr;
}

/*
8 ext tail bits
41 sync seq
36 encrypted bits
3 tail bits
68.25 extended tail bits (!)

center at 8+5 (actually known tb -> known isi, start at 8?) FIXME
*/
int get_access_imp_resp(const gr_complex *input, gr_complex *chan_imp_resp, float *corr_max, int max_delay)
{
	const int search_center = 8 + 5;
	const int search_start_pos = (search_center - 5) * d_OSR + 1;
	const int search_stop_pos = (search_center + 5 + d_chan_imp_length + max_delay) * d_OSR;
	const auto tseq = &d_acc_training_seq[TRAIN_BEGINNING];
	const auto tseqlen = N_ACCESS_BITS - (2 * TRAIN_BEGINNING);
	return get_chan_imp_resp(input, chan_imp_resp, search_start_pos, search_stop_pos, tseq, tseqlen, corr_max) -
	       search_center * d_OSR;
}

/*

3 + 57 + 1 + 26 + 1 + 57 + 3 + 8.25

search center = 3 + 57 + 1 + 5 (due to tsc 5+16+5 split)
this is +-5 samples around (+5 beginning) of truncated t16 tsc

*/
int get_norm_chan_imp_resp(const gr_complex *input, gr_complex *chan_imp_resp, float *corr_max, int bcc)
{
	const int search_center = TRAIN_POS;
	const int search_start_pos = (search_center - 5) * d_OSR + 1;
	const int search_stop_pos = (search_center + 5 + d_chan_imp_length) * d_OSR;
	const auto tseq = &d_norm_training_seq[bcc][TRAIN_BEGINNING];
	const auto tseqlen = N_TRAIN_BITS - (2 * TRAIN_BEGINNING);
	return get_chan_imp_resp(input, chan_imp_resp, search_start_pos, search_stop_pos, tseq, tseqlen, corr_max) -
	       search_center * d_OSR;
}

/*

3 tail | 39 data | 64 tsc | 39 data | 3 tail | 8.25 guard
start 3+39 - 10
end 3+39 + SYNC_SEARCH_RANGE

*/
int get_sch_chan_imp_resp(const gr_complex *input, gr_complex *chan_imp_resp)
{
	const int search_center = SYNC_POS + TRAIN_BEGINNING;
	const int search_start_pos = (search_center - 10) * d_OSR;
	const int search_stop_pos = (search_center + SYNC_SEARCH_RANGE) * d_OSR;
	const auto tseq = &d_sch_training_seq[TRAIN_BEGINNING];
	const auto tseqlen = N_SYNC_BITS - (2 * TRAIN_BEGINNING);

	// strongest_window_nr + chan_imp_resp_center + SYNC_POS *d_OSR - 48 * d_OSR - 2 * d_OSR + 2 ;
	float corr_max;
	return get_chan_imp_resp(input, chan_imp_resp, search_start_pos, search_stop_pos, tseq, tseqlen, &corr_max) -
	       search_center * d_OSR;
	;
}

int get_sch_buffer_chan_imp_resp(const gr_complex *input, gr_complex *chan_imp_resp, unsigned int len, float *corr_max)
{
	const auto tseqlen = N_SYNC_BITS - (2 * TRAIN_BEGINNING);
	const int search_center = SYNC_POS + TRAIN_BEGINNING;
	const int search_start_pos = 0;
	// FIXME: proper end offset
	const int search_stop_pos = len - (N_SYNC_BITS * 8);
	auto tseq = &d_sch_training_seq[TRAIN_BEGINNING];

	return get_chan_imp_resp(input, chan_imp_resp, search_start_pos, search_stop_pos, tseq, tseqlen, corr_max) -
	       search_center * d_OSR;
}