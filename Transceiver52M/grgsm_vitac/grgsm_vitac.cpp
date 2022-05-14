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

#include "grgsm_vitac/constants.h"
#define _CRT_SECURE_NO_WARNINGS

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



//signalVector mChanResp;
gr_complex d_sch_training_seq[N_SYNC_BITS]; ///<encoded training sequence of a SCH burst
gr_complex d_norm_training_seq[TRAIN_SEQ_NUM][N_TRAIN_BITS]; ///<encoded training sequences of a normal and dummy burst
int get_norm_chan_imp_resp(const gr_complex* input, gr_complex* chan_imp_resp, float* corr_max, int* corr_max_index);

#define SYNC_SEARCH_RANGE 30

const int d_OSR(4);
const int d_chan_imp_length(CHAN_IMP_RESP_LENGTH);
std::vector<gr_complex> channel_imp_resp(CHAN_IMP_RESP_LENGTH* d_OSR);

void initv();
void process();
int
get_sch_chan_imp_resp(const gr_complex* input,
	gr_complex* chan_imp_resp);
void
detect_burst(const gr_complex* input,
	gr_complex* chan_imp_resp, int burst_start,
	unsigned char* output_binary);
void
gmsk_mapper(const unsigned char* input,
	int nitems, gr_complex* gmsk_output, gr_complex start_point)
	;
gr_complex
correlate_sequence(const gr_complex* sequence,
	int length, const gr_complex* input)
	;
inline void
autocorrelation(const gr_complex* input,
	gr_complex* out, int nitems)
	;
inline void
mafi(const gr_complex* input, int nitems,
	gr_complex* filter, int filter_length, gr_complex* output)
	;
int
get_norm_chan_imp_resp(const gr_complex* input,
	gr_complex* chan_imp_resp, float* corr_max, int bcc)
	;


struct fdata {
	unsigned int fn;
	int tn;
	int bcc;
	std::string fpath;
	std::vector<gr_complex> data;
};


std::vector<fdata> files_to_process;

void initvita() {

	/**
	 * Prepare SCH sequence bits
	 *
	 * (TS_BITS + 2 * GUARD_PERIOD)
	 * Burst and two guard periods
	 * (one guard period is an arbitrary overlap)
	 */
	gmsk_mapper(SYNC_BITS, N_SYNC_BITS,
		d_sch_training_seq, gr_complex(0.0, -1.0));

	/* Prepare bits of training sequences */
	for (int i = 0; i < TRAIN_SEQ_NUM; i++) {
		/**
		 * If first bit of the sequence is 0
		 * => first symbol is 1, else -1
		 */
		gr_complex startpoint = train_seq[i][0] == 0 ?
			gr_complex(1.0, 0.0) : gr_complex(-1.0, 0.0);
		gmsk_mapper(train_seq[i], N_TRAIN_BITS,
			d_norm_training_seq[i], startpoint);
	}

}

int
get_sch_chan_imp_resp(const gr_complex* input,
	gr_complex* chan_imp_resp)
{
	std::vector<gr_complex> correlation_buffer;
	std::vector<float> window_energy_buffer;
	std::vector<float> power_buffer;

	int chan_imp_resp_center = 0;
	int strongest_window_nr;
	int burst_start;
	float energy = 0;

	int len = (SYNC_POS + SYNC_SEARCH_RANGE) * d_OSR;
	for (int ii = SYNC_POS * d_OSR; ii < len; ii++) {
		gr_complex correlation = correlate_sequence(&d_sch_training_seq[5],
			N_SYNC_BITS - 10, &input[ii]);
		correlation_buffer.push_back(correlation);
		power_buffer.push_back(std::pow(abs(correlation), 2));
	}

	/* Compute window energies */
	std::vector<float>::iterator iter = power_buffer.begin();
	while (iter != power_buffer.end()) {
		std::vector<float>::iterator iter_ii = iter;
		bool loop_end = false;
		energy = 0;

		for (int ii = 0; ii < (d_chan_imp_length)*d_OSR; ii++, iter_ii++) {
			if (iter_ii == power_buffer.end()) {
				loop_end = true;
				break;
			}

			energy += (*iter_ii);
		}

		if (loop_end)
			break;

		window_energy_buffer.push_back(energy);
		iter++;
	}

	strongest_window_nr = max_element(window_energy_buffer.begin(),
		window_energy_buffer.end()) - window_energy_buffer.begin();

#if 0
	d_channel_imp_resp.clear();
#endif

	float max_correlation = 0;
	for (int ii = 0; ii < (d_chan_imp_length)*d_OSR; ii++) {
		gr_complex correlation = correlation_buffer[strongest_window_nr + ii];
		if (abs(correlation) > max_correlation) {
			chan_imp_resp_center = ii;
			max_correlation = abs(correlation);
		}

#if 0
		d_channel_imp_resp.push_back(correlation);
#endif

		chan_imp_resp[ii] = correlation;
	}

	burst_start = strongest_window_nr + chan_imp_resp_center
		- 48 * d_OSR - 2 * d_OSR + 2 + SYNC_POS * d_OSR;
	return burst_start;
}


#if defined(__has_attribute)
  #if __has_attribute(target_clones)
    #if defined(__x86_64)
    #define MULTI_VER_TARGET_ATTR __attribute__((target_clones("avx","sse4.2","sse3","sse2","sse","default")))
    #endif
  #else
  #define MULTI_VER_TARGET_ATTR
  #endif
#endif

MULTI_VER_TARGET_ATTR
void
detect_burst(const gr_complex* input,
	gr_complex* chan_imp_resp, int burst_start,
	unsigned char* output_binary)
{
	std::vector<gr_complex> rhh_temp(CHAN_IMP_RESP_LENGTH * d_OSR);
	unsigned int stop_states[2] = { 4, 12 };
	gr_complex filtered_burst[BURST_SIZE];
	gr_complex rhh[CHAN_IMP_RESP_LENGTH];
	float output[BURST_SIZE];
	int start_state = 3;

	autocorrelation(chan_imp_resp, &rhh_temp[0], d_chan_imp_length * d_OSR);
	for (int ii = 0; ii < d_chan_imp_length; ii++)
		rhh[ii] = conj(rhh_temp[ii * d_OSR]);

	mafi(&input[burst_start], BURST_SIZE, chan_imp_resp,
		d_chan_imp_length * d_OSR, filtered_burst);

	viterbi_detector(filtered_burst, BURST_SIZE, rhh,
		start_state, stop_states, 2, output);

	for (int i = 0; i < BURST_SIZE; i++)
		output_binary[i] = output[i] > 0;
}



int d_c0_burst_start;

int process_vita_burst(gr_complex* input, int tsc, unsigned char* output_binary) {
	unsigned int normal_burst_start, dummy_burst_start;
	float dummy_corr_max, normal_corr_max;

	dummy_burst_start = get_norm_chan_imp_resp(input,
		&channel_imp_resp[0], &dummy_corr_max, TS_DUMMY);
	normal_burst_start = get_norm_chan_imp_resp(input,
		&channel_imp_resp[0], &normal_corr_max, tsc);

	if (normal_corr_max > dummy_corr_max) {
		d_c0_burst_start = normal_burst_start;

		/* Perform MLSE detection */
		detect_burst(input, &channel_imp_resp[0],
			normal_burst_start, output_binary);
		
		return 0;

	}
	else {
		d_c0_burst_start = dummy_burst_start;
		memcpy(output_binary, dummy_burst, 148);
		//std::cerr << std::endl << "#NOPE#" << dd.fpath << std::endl << std::endl;
		return -1;
	}

}

int process_vita_sc_burst(gr_complex* input, int tsc, unsigned char* output_binary, int* offset) {

	int ncc, bcc;
	int t1, t2, t3;
	int rc;

	/* Get channel impulse response */
	d_c0_burst_start = get_sch_chan_imp_resp(input,
	&channel_imp_resp[0]);
	
	/* Perform MLSE detection */
	detect_burst(input, &channel_imp_resp[0],
	d_c0_burst_start, output_binary);

	/**
	* Decoding was successful, now
	* compute offset from burst_start,
	* burst should start after a guard period.
	*/
	*offset = d_c0_burst_start - floor((GUARD_PERIOD) * d_OSR);

}

void
gmsk_mapper(const unsigned char* input,
	int nitems, gr_complex* gmsk_output, gr_complex start_point)
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
		gmsk_output[i] = j * gr_complex(encoded_symbol, 0.0)
			* gmsk_output[i - 1];

		previous_symbol = current_symbol;
	}
}

gr_complex
correlate_sequence(const gr_complex* sequence,
	int length, const gr_complex* input)
{
	gr_complex result(0.0, 0.0);

	for (int ii = 0; ii < length; ii++)
		result += sequence[ii] * conj(input[ii * d_OSR]);

	return result / gr_complex(length, 0);
}

/* Computes autocorrelation for positive arguments */
inline void
autocorrelation(const gr_complex* input,
	gr_complex* out, int nitems)
{
	for (int k = nitems - 1; k >= 0; k--) {
		out[k] = gr_complex(0, 0);
		for (int i = k; i < nitems; i++)
			out[k] += input[i] * conj(input[i - k]);
	}
}

inline void
mafi(const gr_complex* input, int nitems,
	gr_complex* filter, int filter_length, gr_complex* output)
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

/* Especially computations of strongest_window_nr */
int
get_norm_chan_imp_resp(const gr_complex* input,
	gr_complex* chan_imp_resp, float* corr_max, int bcc)
{
	std::vector<gr_complex> correlation_buffer;
	std::vector<float> window_energy_buffer;
	std::vector<float> power_buffer;

	int search_center = (int)(TRAIN_POS + 0) * d_OSR;
	int search_start_pos = search_center + 1 - 5 * d_OSR;
	int search_stop_pos = search_center
		+ d_chan_imp_length * d_OSR + 5 * d_OSR;

	for (int ii = search_start_pos; ii < search_stop_pos; ii++) {
		gr_complex correlation = correlate_sequence(
			&d_norm_training_seq[bcc][TRAIN_BEGINNING],
			N_TRAIN_BITS - 10, &input[ii]);
		correlation_buffer.push_back(correlation);
		power_buffer.push_back(std::pow(abs(correlation), 2));
	}

#if 0
	plot(power_buffer);
#endif

	/* Compute window energies */
	std::vector<float>::iterator iter = power_buffer.begin();
	while (iter != power_buffer.end()) {
		std::vector<float>::iterator iter_ii = iter;
		bool loop_end = false;
		float energy = 0;

		int len = d_chan_imp_length * d_OSR;
		for (int ii = 0; ii < len; ii++, iter_ii++) {
			if (iter_ii == power_buffer.end()) {
				loop_end = true;
				break;
			}

			energy += (*iter_ii);
		}

		if (loop_end)
			break;

		window_energy_buffer.push_back(energy);
		iter++;
	}

	/* Calculate the strongest window number */
	int strongest_window_nr = max_element(window_energy_buffer.begin(),
		window_energy_buffer.end() - d_chan_imp_length * d_OSR)
		- window_energy_buffer.begin();

	if (strongest_window_nr < 0)
		strongest_window_nr = 0;

	float max_correlation = 0;
	for (int ii = 0; ii < d_chan_imp_length * d_OSR; ii++) {
		gr_complex correlation = correlation_buffer[strongest_window_nr + ii];
		if (abs(correlation) > max_correlation)
			max_correlation = abs(correlation);

#if 0
		d_channel_imp_resp.push_back(correlation);
#endif

		chan_imp_resp[ii] = correlation;
	}

	*corr_max = max_correlation;

	/**
	 * Compute first sample position, which corresponds
	 * to the first sample of the impulse response
	 */
	return search_start_pos + strongest_window_nr - TRAIN_POS * d_OSR;
}



