/*
 * (C) 2023 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
 * All Rights Reserved
 *
 * Author: Eric Wild <ewild@sysmocom.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// this allows messing with the demod to check the detecton offset impact,
// not intended for actual automated tests.

#include "sigProcLib.h"

extern "C" {
#include "convert.h"
#include <convolve.h>
}

#define _CRT_SECURE_NO_WARNINGS
#include <algorithm>
#include <string.h>
#include <iomanip>
#include <numeric>
#include <memory>
#include <iostream>
#include <fstream>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <grgsm_vitac/grgsm_vitac.h>

#define DO_RACH

const int SAMPLE_SCALE_FACTOR = 1;

template <typename DST_T, typename SRC_T, typename ST>
void convert_and_scale(void *dst, void *src, unsigned int src_len, ST scale)
{
	for (unsigned int i = 0; i < src_len; i++)
		reinterpret_cast<DST_T *>(dst)[i] = static_cast<DST_T>((reinterpret_cast<SRC_T *>(src)[i]) * scale);
}
template <typename DST_T, typename SRC_T>
void convert_and_scale_default(void *dst, void *src, unsigned int src_len)
{
	return convert_and_scale<DST_T, SRC_T>(dst, src, src_len, SAMPLE_SCALE_FACTOR);
}

static const unsigned int txFullScale = (float)(1 << 14) - 1;
// static const unsigned int rxFullScale = (float)(1 << 14) - 1;

static const BitVector
	gRACHBurstx("0011101001001011011111111001100110101010001111000110111101111110000111001001010110011000");

static const BitVector gTrainingSequencex[] = {
	BitVector("00100101110000100010010111"), BitVector("00101101110111100010110111"),
	BitVector("01000011101110100100001110"), BitVector("01000111101101000100011110"),
	BitVector("00011010111001000001101011"), BitVector("01001110101100000100111010"),
	BitVector("10100111110110001010011111"), BitVector("11101111000100101110111100"),
};

struct mrv {
	std::vector<char> bits;
	signalVector *rvbuf;
	std::unique_ptr<std::vector<std::complex<float>>> convolved;
	// mrv(): bits(), demod_bits() {}
	CorrType ct;
};

static mrv genRandNormalBurstx(int tsc, int sps, int tn)
{
	mrv retstruct;
	int i = 0;
	BitVector bits(148);

	/* Tail bits */
	for (; i < 3; i++)
		bits[i] = 0;

	/* Random bits */
	for (int j = 0; i < 60; i++, j++)
		bits[i] = rand() % 2;

	/* Stealing bit */
	bits[i++] = 0;

	/* Training sequence */
	for (int n = 0; i < 87; i++, n++)
		bits[i] = gTrainingSequencex[tsc][n];

	/* Stealing bit */
	bits[i++] = 0;

	/* Random bits */
	for (; i < 145; i++)
		bits[i] = rand() % 2;

	/* Tail bits */
	for (; i < 148; i++)
		bits[i] = 0;

	int guard = 8 + !(tn % 4);
	auto r = modulateBurst(bits, guard, sps);

	retstruct.rvbuf = r;
	for (size_t i = 0; i < bits.size(); i++)
		retstruct.bits.push_back(bits.bit(i) ? 1 : 0);
	return retstruct;
}

static mrv genRandAccessBurstx(int delay, int sps, int tn)
{
	mrv retstruct;
	int i = 0;
	BitVector bits(88 + delay);

	/* delay */
	for (; i < delay; i++)
		bits[i] = 0;

	/* head and synch bits */
	for (int n = 0; i < 49 + delay; i++, n++)
		bits[i] = gRACHBurstx[n];

	/* Random bits */
	for (int j = 0; i < 85 + delay; i++, j++)
		bits[i] = rand() % 2;

	for (; i < 88 + delay; i++)
		bits[i] = 0;

	int guard = 68 - delay + !(tn % 4);
	auto r = modulateBurst(bits, guard, sps);

	retstruct.rvbuf = r;
	for (size_t i = 0; i < bits.size(); i++)
		retstruct.bits.push_back(bits.bit(i) ? 1 : 0);
	return retstruct;
}

extern gr_complex d_acc_training_seq[N_ACCESS_BITS]; ///<encoded training sequence of a SCH burst
extern gr_complex d_sch_training_seq[N_SYNC_BITS]; ///<encoded training sequence of a SCH burst
extern gr_complex d_norm_training_seq[TRAIN_SEQ_NUM]
				     [N_TRAIN_BITS]; ///<encoded training sequences of a normal and dummy burst

void sv_write_helper(signalVector *burst, std::string fname)
{
	auto start = burst->begin();
	auto n = burst->bytes();
	char *data = reinterpret_cast<char *>(start);

	const int len_in_real = burst->size() * 2;
	auto cvrtbuf_tx_a = new int16_t[len_in_real];
	convert_float_short(cvrtbuf_tx_a, (float *)burst->begin(), float(txFullScale), len_in_real);

	std::ofstream fout;
	fout.open(fname + ".cfile", std::ios::binary | std::ios::out);
	fout.write(data, n);
	fout.close();

	fout.open(fname + ".cs16", std::ios::binary | std::ios::out);
	fout.write((char *)cvrtbuf_tx_a, len_in_real * sizeof(uint16_t));
	fout.close();
	delete[] cvrtbuf_tx_a;
}

// borrowed from a real world burst..
static std::vector<std::complex<float>> chan_im_resp = {
	{ 4.1588e-05 + -0.000361925 },	{ 0.000112728 + -0.000289796 }, { 0.000162952 + -0.000169028 },
	{ 0.000174185 + -2.54575e-05 }, { 0.000142947 + 0.000105992 },	{ 8.65919e-05 + 0.000187041 },
	{ 4.15799e-05 + 0.000184346 },	{ 5.30207e-05 + 7.84921e-05 },	{ 0.000158877 + -0.000128058 },
	{ 0.000373956 + -0.000407954 }, { 0.000680606 + -0.000712065 }, { 0.00102929 + -0.000979604 },
	{ 0.00135049 + -0.00115333 },	{ 0.00157434 + -0.0011948 },	{ 0.00165098 + -0.00109534 },
	{ 0.00156519 + -0.000878794 },	{ 0.0013399 + -0.000594285 },	{ 0.00102788 + -0.00030189 },
	{ 0.000694684 + -5.58912e-05 }, { 0.000399328 + 0.000109463 }
};

// as above, downsampled to 1sps + just magnitude
static std::vector<float> chan_im_resp_trunc = { 1., 0.20513351, 0.10020305, 0.11490235 };

template <typename A, typename B>
auto conv(const std::vector<A> &a, const std::vector<B> &b) -> std::unique_ptr<std::vector<A>>
{
	int data_len = a.size();
	int conv_len = b.size();
	int conv_size = conv_len + data_len - 1;
	auto retv = std::make_unique<std::vector<A>>(conv_size);

	for (int i = 0; i < data_len; ++i) {
		for (int j = 0; j < conv_len; ++j) {
			(*retv)[i + j] += a[i] * b[j];
		}
	}
	return retv;
}

template <typename A>
static auto conv(const A *a, int len, std::vector<float> &b)
{
	std::vector<A> aa(len);
	std::copy_n(a, len, aa.begin());
	std::reverse(b.begin(), b.end());
	return conv(aa, b);
}
template <typename A>
static auto conv(const A *a, int len, std::vector<A> &b)
{
	std::vector<A> aa(len);
	std::copy_n(a, len, aa.begin());
	std::reverse(b.begin(), b.end());
	return conv(aa, b);
}

// signalvector is owning despite claiming not to, but we can pretend, too..
static void dummy_free(void *wData){};
static void *dummy_alloc(size_t newSize)
{
	return 0;
};

template <typename T>
size_t read_from_file(std::string path, std::vector<T> &outvec)
{
	std::ifstream infile;
	infile.open(path, std::ios::in | std::ios::binary);
	if (infile.fail()) {
		std::cout << " not found: " << path << std::endl;
		exit(0);
	}
	infile.seekg(0, std::ios_base::end);
	size_t fsize = infile.tellg();
	auto fsize_in_T = fsize / sizeof(T);
	infile.seekg(0, std::ios_base::beg);

	outvec.resize(fsize_in_T);
	infile.read(reinterpret_cast<char *>(&outvec[0]), fsize);
	infile.close();
	std::cout << "Read " << fsize << " from " << path << std::endl;
	return fsize;
}
void demod_real_burst(int num = 0)
{
	auto path = "./nb_chunk_tsc7.cfile";
	auto bitfile = "./demodbits_tsc7.s8";

	std::vector<std::complex<float>> burstdata;
	std::vector<char> bitdata;
	read_from_file(path, burstdata);
	read_from_file(bitfile, bitdata);

	// print "known good" burst bits
	std::cerr << "known bits:" << std::endl;
	std::cerr << std::setw(5) << 0 << " - ";
	for (auto i : bitdata)
		std::cout << (i > 0 ? "1" : "0");
	std::cerr << std::endl;
	std::cerr << "demod tests sigproclib:" << std::endl;

	auto ct = CorrType::TSC;
	auto delay = 0;
	auto tsc = 7;
	int offset = 0;
	auto cplx = reinterpret_cast<complex *>(&burstdata[offset]);
	auto stdcplx = reinterpret_cast<std::complex<float> *>(&burstdata[offset]);
	signalVector sv(&cplx[0], 0, burstdata.size() - offset, dummy_alloc, dummy_free);

	struct estim_burst_params ebp;
	auto rc = detectAnyBurst(sv, tsc, BURST_THRESH, 4, ct, 40, &ebp);

	auto rxBurst = std::unique_ptr<SoftVector>(demodAnyBurst(sv, (CorrType)rc, 4, &ebp));
	// print osmotrx sigproclib demod result
	std::cerr << std::setw(5) << int(ebp.toa) << " o ";
	for (ssize_t i = 0 + delay; i < 148 + delay; i++)
		std::cout << (rxBurst->bit(i) ? "1" : "0");
	std::cerr << std::endl;

	std::cerr << "demod test va:" << std::endl;
	std::complex<float> chan_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];
	float ncmax;
	char demodded_softbits[444];

	// demod at known offset
	{
		auto inp = &stdcplx[29]; // known offset
		auto normal_burst_startX = get_norm_chan_imp_resp(inp, &chan_imp_resp[0], &ncmax, tsc);
		detect_burst_nb(inp, &chan_imp_resp[0], normal_burst_startX, demodded_softbits);

		std::cerr << std::setw(5) << normal_burst_startX << " v ";
		for (size_t i = 0; i < 148; i++)
			std::cerr << (demodded_softbits[i] < 0 ? "1" : "0");
		std::cerr << std::endl;
	}
	{
		std::cerr << "-- va start offset loop --" << std::endl;
		std::cerr << "offset/det offset/#errors/known^demod bits" << std::endl;
		for (int i = 0; i < 34; i++) {
			auto inp = &stdcplx[i];
			auto conved_beg = inp;

			auto me = get_norm_chan_imp_resp(conved_beg, &chan_imp_resp[0], &ncmax, tsc);
			detect_burst_nb(conved_beg, &chan_imp_resp[0], me, demodded_softbits);
			auto bitdiffarr = std::make_unique<char[]>(148);
			for (size_t i = 0; i < 148; i++)
				bitdiffarr.get()[i] = (demodded_softbits[i] < 0 ? 1 : 0) ^ (bitdata[i] > 0 ? 1 : 0);
			auto ber = std::accumulate(bitdiffarr.get(), bitdiffarr.get() + 148, 0);

			std::cerr << std::setw(3) << i << ": " << std::setw(3) << me << " v " << std::setw(3) << ber
				  << " ";
			for (size_t i = 0; i < 148; i++)
				std::cerr << (bitdiffarr[i] ? "1" : "0");
			std::cerr << std::endl;
			// std::cerr << std::setw(4) << i << " (" << std::setw(4) << 29 - i << "):" << std::setw(4) << org
			// 	  << " " << std::setw(4) << me << " y " << std::endl;
		}
	}
}

auto gen_burst(CorrType t, int delay, int tsc)
{
	mrv rs;
	if (t == CorrType::RACH) {
		rs = genRandAccessBurstx(delay, 4, tsc);

	} else if (t == CorrType::TSC) {
		rs = genRandNormalBurstx(tsc, 4, 0);
	} else {
		std::cerr << "wtf?" << std::endl;
		exit(0);
	}
	rs.ct = t;

	signalVector *burst = rs.rvbuf;
	// sv_write_helper(burst, std::to_string(num));
	// scaleVector(*burst, {1, 0});
	const int len_in_real = burst->size() * 2;
	auto cvrtbuf_tx_a = std::make_unique<short[]>(len_in_real);
	auto cvrtbuf_rx_a = std::make_unique<float[]>(len_in_real);
	auto rx_cfloat = reinterpret_cast<std::complex<float> *>(&cvrtbuf_rx_a[0]);

	convert_float_short(cvrtbuf_tx_a.get(), (float *)burst->begin(), float(txFullScale), len_in_real);
	convert_short_float(cvrtbuf_rx_a.get(), cvrtbuf_tx_a.get(), len_in_real);
	for (int i = 0; i < len_in_real; i++) // scale properly!
		cvrtbuf_rx_a[i] *= 1. / txFullScale;
	auto conved = conv(rx_cfloat, burst->size(), chan_im_resp);

	std::cerr << "-- generated " << (t == CorrType::RACH ? "RACH" : "TSC") << " burst --" << std::endl;
	for (size_t i = 0; i < rs.bits.size(); i++)
		std::cerr << (rs.bits[i] ? "1" : "0");
	std::cerr << std::endl;
	delete burst;
	rs.convolved = std::move(conved);
	return rs;
}

void demod_generated_burst(CorrType t)
{
	int tsc = 0;
	int delay = 0;
	auto rs = gen_burst(t, delay, tsc);
	auto conved_beg = &(*rs.convolved)[0];

	if (rs.ct == CorrType::RACH) {
		std::complex<float> chan_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];
		float ncmax;
		char demodded_softbits[444];
		int normal_burst_start = 0;
		normal_burst_start = get_access_imp_resp(conved_beg, &chan_imp_resp[0], &ncmax, 0);
		normal_burst_start = std::max(normal_burst_start, 0);
		for (int j = 0; j < 4; j++) {
			for (int start_val = 0; start_val < 16; start_val++) {
				auto bitdiffarr = std::make_unique<char[]>(rs.bits.size());
				detect_burst_ab(conved_beg, &chan_imp_resp[0], normal_burst_start + j,
						demodded_softbits, start_val);

				for (size_t i = 0; i < rs.bits.size(); i++)
					bitdiffarr.get()[i] = (demodded_softbits[i] < 0 ? 1 : 0) ^ rs.bits[i];
				auto ber = std::accumulate(bitdiffarr.get(), bitdiffarr.get() + rs.bits.size(), 0);

				std::cerr << "ber " << std::setw(4) << ber << " bo:" << std::setw(4) << j
					  << " vas:" << std::setw(4) << start_val << " ";
				// for (size_t i = 0; i < rs.num_bits; i++)
				// 	std::cerr << (demodded_softbits[i] < 0 ? "1" : "0");
				// std::cerr << std::endl;
				// std::cerr << "d " << std::setw(4) << ber << " ";
				for (size_t i = 0; i < rs.bits.size(); i++)
					std::cerr << (bitdiffarr.get()[i] ? "1" : "0");
				std::cerr << std::endl;

				// std::cerr << "v " << std::setw(4) << j << std::setw(4) << start_val << " ";
				// for (size_t i = 0; i < rs.num_bits; i++)
				// 	std::cerr << (demodded_softbits[i] < 0 ? "1" : "0");
				// std::cerr << std::endl;
				// std::cerr << "d " << std::setw(4) << ber << " ";
				// for (size_t i = 0; i < rs.num_bits; i++)
				// 	std::cerr << (ptr.get()[i] ? "1" : "0");
				// std::cerr << std::endl;
			}
		}

	} else {
		std::complex<float> chan_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];
		float ncmax;
		char demodded_softbits[444];

		auto normal_burst_start = get_norm_chan_imp_resp(conved_beg, &chan_imp_resp[0], &ncmax, tsc);
		detect_burst_nb(conved_beg, &chan_imp_resp[0], normal_burst_start + 0, demodded_softbits);
		std::cerr << "toa " << std::setprecision(2) << normal_burst_start << std::endl;

		std::cerr << "vita ";
		for (size_t i = 0; i < rs.bits.size(); i++)
			std::cerr << (demodded_softbits[i] < 0 ? "1" : "0");
		std::cerr << std::endl;
		std::cerr << "diff ";
		for (size_t i = 0; i < rs.bits.size(); i++)
			std::cerr << ((demodded_softbits[i] < 0 ? 1 : 0) ^ rs.bits[i] ? "1" : "0");
		std::cerr << std::endl;
	}

	struct estim_burst_params ebp;
	char demodded_softbits[444];
	complex *rx_sigproc_cfloat = reinterpret_cast<complex *>(conved_beg);
	signalVector sv(rx_sigproc_cfloat, 0, rs.convolved->size(), dummy_alloc, dummy_free);

	auto rc = detectAnyBurst(sv, tsc, BURST_THRESH, 4, rs.ct, 40, &ebp);
	auto rxBurst = std::unique_ptr<SoftVector>(demodAnyBurst(sv, (CorrType)rc, 4, &ebp));

	std::cerr << "toa " << std::setprecision(2) << ebp.toa << std::endl;

	for (ssize_t i = 0; i < delay; i++) // maybe pad rach op?
		demodded_softbits[i] = 0;
	for (size_t i = 0 + delay; i < rs.bits.size() + delay; i++)
		demodded_softbits[i] = (rxBurst->bit(i) ? 1 : 0);

	std::cerr << "sigp ";
	for (size_t i = 0; i < rs.bits.size(); i++)
		std::cerr << (demodded_softbits[i] ? "1" : "0");
	std::cerr << std::endl;

	std::cerr << "diff ";
	for (size_t i = 0; i < rs.bits.size(); i++)
		std::cerr << (demodded_softbits[i] ^ rs.bits[i] ? "1" : "0");
	std::cerr << std::endl;
}

void demod_test_offsets()
{
	const int tsc = 0;
	const int delaybuffer_realoffset = 100;

	{
		auto rs = gen_burst(CorrType::RACH, 0, tsc);
		typeof(*rs.convolved) delay_buffer(rs.convolved->size() * 2); // plenty of space..

		for (int delay = -10; delay < 60; delay++) {
			std::fill(delay_buffer.begin(), delay_buffer.end(), 0);
			std::copy(rs.convolved->begin(), rs.convolved->end(),
				  delay_buffer.begin() + delaybuffer_realoffset + delay);

			auto conved_beg = &delay_buffer[delaybuffer_realoffset];

			std::complex<float> chan_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];
			float ncmax;
			auto va_burst_start = get_access_imp_resp(conved_beg, &chan_imp_resp[0], &ncmax, 60);

			complex *rx_sigproc_cfloat = reinterpret_cast<complex *>(conved_beg);
			struct estim_burst_params ebp;
			signalVector sv(rx_sigproc_cfloat, 0, rs.convolved->size(), dummy_alloc, dummy_free);
			detectAnyBurst(sv, tsc, BURST_THRESH, 4, rs.ct, 60, &ebp);
			std::cerr << "delay:" << std::setw(3) << std::setprecision(2) << delay;
			std::cerr << " va: " << std::setw(3) << std::setprecision(2) << va_burst_start;
			std::cerr << " sg: " << std::setw(3) << std::setprecision(2) << ebp.toa;
			std::cerr << " d: " << std::setw(3) << std::setprecision(2) << (ebp.toa * 4) - va_burst_start;
			std::cerr << " ! " << float(va_burst_start + 13) / 4 << std::endl;
		}
	}
	{
		auto rs = gen_burst(CorrType::TSC, 0, tsc);
		typeof(*rs.convolved) delay_buffer(rs.convolved->size() * 2); // plenty of space..

		for (int delay = -10; delay < 10; delay++) {
			std::fill(delay_buffer.begin(), delay_buffer.end(), 0);
			std::copy(rs.convolved->begin(), rs.convolved->end(),
				  delay_buffer.begin() + delaybuffer_realoffset + delay);

			auto conved_beg = &delay_buffer[delaybuffer_realoffset];

			std::complex<float> chan_imp_resp[CHAN_IMP_RESP_LENGTH * d_OSR];
			float ncmax;
			auto va_burst_start = get_norm_chan_imp_resp(conved_beg, &chan_imp_resp[0], &ncmax, tsc);

			complex *rx_sigproc_cfloat = reinterpret_cast<complex *>(conved_beg);
			struct estim_burst_params ebp;
			signalVector sv(rx_sigproc_cfloat, 0, rs.convolved->size(), dummy_alloc, dummy_free);
			detectAnyBurst(sv, tsc, BURST_THRESH, 4, rs.ct, 60, &ebp);
			std::cerr << "delay:" << std::setw(3) << std::setprecision(2) << delay;
			std::cerr << " va: " << std::setw(3) << std::setprecision(2) << va_burst_start;
			std::cerr << " sg: " << std::setw(3) << std::setprecision(2) << ebp.toa;
			std::cerr << " d: " << std::setw(3) << std::setprecision(2) << (ebp.toa * 4) - va_burst_start;
			std::cerr << " ! " << float(va_burst_start + 19) / 4 << std::endl;
		}
	}
}

int main()
{
	convolve_init();
	convert_init();
	sigProcLibSetup();
	initvita();

	for (int i = 0; i < 1; i++) {
		demod_real_burst(i);
		demod_generated_burst(CorrType::RACH);
		demod_generated_burst(CorrType::TSC);
		demod_test_offsets();
	}
}
