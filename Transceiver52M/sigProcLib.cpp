/*
* Copyright 2008, 2011 Free Software Foundation, Inc.
*
* SPDX-License-Identifier: AGPL-3.0+
*
* This software is distributed under the terms of the GNU Affero Public License.
* See the COPYING file in the main directory for details.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "sigProcLib.h"
#include "GSMCommon.h"
#include "Logger.h"
#include "Resampler.h"

extern "C" {
#include <osmocom/core/panic.h>
#include "convolve.h"
#include "scale.h"
#include "mult.h"
}

using namespace GSM;

#define TABLESIZE		1024
#define DELAYFILTS		64

/* Clipping detection threshold */
#define CLIP_THRESH		30000.0f

/** Lookup tables for trigonometric approximation */
static float sincTable[TABLESIZE+1]; // add 1 element for wrap around

/** Constants */
static const float M_PI_F = (float)M_PI;

/* Precomputed rotation vectors */
static signalVector *GMSKRotation4 = NULL;
static signalVector *GMSKReverseRotation4 = NULL;
static signalVector *GMSKRotation1 = NULL;
static signalVector *GMSKReverseRotation1 = NULL;

/* Precomputed fractional delay filters */
static signalVector *delayFilters[DELAYFILTS];

static const Complex<float> psk8_table[8] = {
   Complex<float>(-0.70710678,  0.70710678),
   Complex<float>( 0.0, -1.0),
   Complex<float>( 0.0,  1.0),
   Complex<float>( 0.70710678, -0.70710678),
   Complex<float>(-1.0,  0.0),
   Complex<float>(-0.70710678, -0.70710678),
   Complex<float>( 0.70710678,  0.70710678),
   Complex<float>( 1.0,  0.0),
};

/* Downsampling filterbank - 4 SPS to 1 SPS */
#define DOWNSAMPLE_IN_LEN	624
#define DOWNSAMPLE_OUT_LEN	156

static Resampler *dnsampler = NULL;

/*
 * RACH and midamble correlation waveforms. Store the buffer separately
 * because we need to allocate it explicitly outside of the signal vector
 * constructor. This is because C++ (prior to C++11) is unable to natively
 * perform 16-byte memory alignment required by many SSE instructions.
 */
struct CorrelationSequence {
  CorrelationSequence() : sequence(NULL), buffer(NULL), toa(0.0)
  {
  }

  ~CorrelationSequence()
  {
    delete sequence;
  }

  signalVector *sequence;
  void         *buffer;
  float        toa;
  complex      gain;
};

/*
 * Gaussian and empty modulation pulses. Like the correlation sequences,
 * store the runtime (Gaussian) buffer separately because of needed alignment
 * for SSE instructions.
 */
struct PulseSequence {
  PulseSequence() : c0(NULL), c1(NULL), c0_inv(NULL), empty(NULL)
  {
  }

  ~PulseSequence()
  {
    delete c0;
    delete c1;
    delete c0_inv;
    delete empty;
  }

  signalVector *c0;
  signalVector *c1;
  signalVector *c0_inv;
  signalVector *empty;
};

static CorrelationSequence *gMidambles[] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
static CorrelationSequence *gEdgeMidambles[] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
static CorrelationSequence *gRACHSequences[] = {NULL,NULL,NULL};
static PulseSequence *GSMPulse1 = NULL;
static PulseSequence *GSMPulse4 = NULL;

void sigProcLibDestroy()
{
  for (int i = 0; i < 8; i++) {
    delete gMidambles[i];
    delete gEdgeMidambles[i];
    gMidambles[i] = NULL;
    gEdgeMidambles[i] = NULL;
  }

  for (int i = 0; i < DELAYFILTS; i++) {
    delete delayFilters[i];
    delayFilters[i] = NULL;
  }

  for (int i = 0; i < 3; i++) {
    delete gRACHSequences[i];
    gRACHSequences[i] = NULL;
  }

  delete GMSKRotation1;
  delete GMSKReverseRotation1;
  delete GMSKRotation4;
  delete GMSKReverseRotation4;
  delete GSMPulse1;
  delete GSMPulse4;
  delete dnsampler;

  GMSKRotation1 = NULL;
  GMSKRotation4 = NULL;
  GMSKReverseRotation4 = NULL;
  GMSKReverseRotation1 = NULL;
  GSMPulse1 = NULL;
  GSMPulse4 = NULL;
}

static float vectorNorm2(const signalVector &x)
{
  signalVector::const_iterator xPtr = x.begin();
  float Energy = 0.0;
  for (;xPtr != x.end();xPtr++) {
	Energy += xPtr->norm2();
  }
  return Energy;
}

/*
 * Initialize 4 sps and 1 sps rotation tables
 */
static void initGMSKRotationTables()
{
  size_t len1 = 157, len4 = 625;

  GMSKRotation4 = new signalVector(len4);
  GMSKReverseRotation4 = new signalVector(len4);
  signalVector::iterator rotPtr = GMSKRotation4->begin();
  signalVector::iterator revPtr = GMSKReverseRotation4->begin();
  auto phase = 0.0;
  while (rotPtr != GMSKRotation4->end()) {
    *rotPtr++ = complex(cos(phase), sin(phase));
    *revPtr++ = complex(cos(-phase), sin(-phase));
    phase += M_PI / 2.0 / 4.0;
  }

  GMSKRotation1 = new signalVector(len1);
  GMSKReverseRotation1 = new signalVector(len1);
  rotPtr = GMSKRotation1->begin();
  revPtr = GMSKReverseRotation1->begin();
  phase = 0.0;
  while (rotPtr != GMSKRotation1->end()) {
    *rotPtr++ = complex(cos(phase), sin(phase));
    *revPtr++ = complex(cos(-phase), sin(-phase));
    phase += M_PI / 2.0;
  }
}

static void GMSKRotate(signalVector &x, int sps)
{
#if HAVE_NEON
  size_t len;
  signalVector *a, *b, *out;

  a = &x;
  out = &x;
  len = out->size();

  if (len == 157)
    len--;

  if (sps == 1)
    b = GMSKRotation1;
  else
    b = GMSKRotation4;

  mul_complex((float *) out->begin(),
              (float *) a->begin(),
              (float *) b->begin(), len);
#else
  signalVector::iterator rotPtr, xPtr = x.begin();

  if (sps == 1)
    rotPtr = GMSKRotation1->begin();
  else
    rotPtr = GMSKRotation4->begin();

  if (x.isReal()) {
    while (xPtr < x.end()) {
      *xPtr = *rotPtr++ * (xPtr->real());
      xPtr++;
    }
  }
  else {
    while (xPtr < x.end()) {
      *xPtr = *rotPtr++ * (*xPtr);
      xPtr++;
    }
  }
#endif
}

static bool GMSKReverseRotate(signalVector &x, int sps)
{
  signalVector::iterator rotPtr, xPtr= x.begin();

  if (sps == 1)
    rotPtr = GMSKReverseRotation1->begin();
  else if (sps == 4)
    rotPtr = GMSKReverseRotation4->begin();
  else
    return false;

  if (x.isReal()) {
    while (xPtr < x.end()) {
      *xPtr = *rotPtr++ * (xPtr->real());
      xPtr++;
    }
  }
  else {
    while (xPtr < x.end()) {
      *xPtr = *rotPtr++ * (*xPtr);
      xPtr++;
    }
  }

  return true;
}

/** Convolution type indicator */
enum ConvType {
  START_ONLY,
  NO_DELAY,
  CUSTOM,
  UNDEFINED,
};

static signalVector *convolve(const signalVector *x, const signalVector *h,
                              signalVector *y, ConvType spanType,
                              size_t start = 0, size_t len = 0)
{
  int rc;
  size_t head = 0, tail = 0;
  bool alloc = false, append = false;
  const signalVector *_x = NULL;

  if (!x || !h)
    return NULL;

  switch (spanType) {
  case START_ONLY:
    start = 0;
    head = h->size() - 1;
    len = x->size();

    if (x->getStart() < head)
      append = true;
    break;
  case NO_DELAY:
    start = h->size() / 2;
    head = start;
    tail = start;
    len = x->size();
    append = true;
    break;
  case CUSTOM:
    if (start < h->size() - 1) {
      head = h->size() - start;
      append = true;
    }
    if (start + len > x->size()) {
      tail = start + len - x->size();
      append = true;
    }
    break;
  default:
    return NULL;
  }

  /*
   * Error if the output vector is too small. Create the output vector
   * if the pointer is NULL.
   */
  if (y && (len > y->size()))
    return NULL;
  if (!y) {
    y = new signalVector(len, convolve_h_alloc, free);
    alloc = true;
  }

  /* Prepend or post-pend the input vector if the parameters require it */
  if (append)
    _x = new signalVector(*x, head, tail);
  else
    _x = x;

  /*
   * Four convolve types:
   *   1. Complex-Real (aligned)
   *   2. Complex-Complex (aligned)
   *   3. Complex-Real (!aligned)
   *   4. Complex-Complex (!aligned)
   */
  if (h->isReal() && h->isAligned()) {
    rc = convolve_real((float *) _x->begin(), _x->size(),
                       (float *) h->begin(), h->size(),
                       (float *) y->begin(), y->size(),
                       start, len);
  } else if (!h->isReal() && h->isAligned()) {
    rc = convolve_complex((float *) _x->begin(), _x->size(),
                          (float *) h->begin(), h->size(),
                          (float *) y->begin(), y->size(),
                          start, len);
  } else if (h->isReal() && !h->isAligned()) {
    rc = base_convolve_real((float *) _x->begin(), _x->size(),
                            (float *) h->begin(), h->size(),
                            (float *) y->begin(), y->size(),
                            start, len);
  } else if (!h->isReal() && !h->isAligned()) {
    rc = base_convolve_complex((float *) _x->begin(), _x->size(),
                               (float *) h->begin(), h->size(),
                               (float *) y->begin(), y->size(),
                               start, len);
  } else {
    rc = -1;
  }

  if (append)
    delete _x;

  if (rc < 0) {
    if (alloc)
      delete y;
    return NULL;
  }

  return y;
}

/*
 * Generate static EDGE linear equalizer. This equalizer is not adaptive.
 * Filter taps are generated from the inverted 1 SPS impulse response of
 * the EDGE pulse shape captured after the downsampling filter.
 */
static bool generateInvertC0Pulse(PulseSequence *pulse)
{
  if (!pulse)
    return false;

  pulse->c0_inv = new signalVector((complex *) convolve_h_alloc(5), 0, 5, convolve_h_alloc, free);
  pulse->c0_inv->isReal(true);
  pulse->c0_inv->setAligned(false);

  signalVector::iterator xP = pulse->c0_inv->begin();
  *xP++ = 0.15884;
  *xP++ = -0.43176;
  *xP++ = 1.00000;
  *xP++ = -0.42608;
  *xP++ = 0.14882;

  return true;
}

static bool generateC1Pulse(int sps, PulseSequence *pulse)
{
  int len;

  if (!pulse)
    return false;

  switch (sps) {
  case 4:
    len = 8;
    break;
  default:
    return false;
  }

  pulse->c1 = new signalVector((complex *) convolve_h_alloc(len), 0, len, convolve_h_alloc, free);
  pulse->c1->isReal(true);

  /* Enable alignment for SSE usage */
  pulse->c1->setAligned(true);

  signalVector::iterator xP = pulse->c1->begin();

  switch (sps) {
  case 4:
    /* BT = 0.30 */
    *xP++ = 0.0;
    *xP++ = 8.16373112e-03;
    *xP++ = 2.84385729e-02;
    *xP++ = 5.64158904e-02;
    *xP++ = 7.05463553e-02;
    *xP++ = 5.64158904e-02;
    *xP++ = 2.84385729e-02;
    *xP++ = 8.16373112e-03;
  }

  return true;
}

static PulseSequence *generateGSMPulse(int sps)
{
  int len;
  float arg, avg, center;
  PulseSequence *pulse;

  if ((sps != 1) && (sps != 4))
    return NULL;

  /* Store a single tap filter used for correlation sequence generation */
  pulse = new PulseSequence();
  pulse->empty = new signalVector(1);
  pulse->empty->isReal(true);
  *(pulse->empty->begin()) = 1.0f;

  /*
   * For 4 samples-per-symbol use a precomputed single pulse Laurent
   * approximation. This should yields below 2 degrees of phase error at
   * the modulator output. Use the existing pulse approximation for all
   * other oversampling factors.
   */
  switch (sps) {
  case 4:
    len = 16;
    break;
  case 1:
  default:
    len = 4;
  }

  pulse->c0 = new signalVector((complex *) convolve_h_alloc(len), 0, len, convolve_h_alloc, free);
  pulse->c0->isReal(true);

  /* Enable alingnment for SSE usage */
  pulse->c0->setAligned(true);

  signalVector::iterator xP = pulse->c0->begin();

  if (sps == 4) {
    *xP++ = 0.0;
    *xP++ = 4.46348606e-03;
    *xP++ = 2.84385729e-02;
    *xP++ = 1.03184855e-01;
    *xP++ = 2.56065552e-01;
    *xP++ = 4.76375085e-01;
    *xP++ = 7.05961177e-01;
    *xP++ = 8.71291644e-01;
    *xP++ = 9.29453645e-01;
    *xP++ = 8.71291644e-01;
    *xP++ = 7.05961177e-01;
    *xP++ = 4.76375085e-01;
    *xP++ = 2.56065552e-01;
    *xP++ = 1.03184855e-01;
    *xP++ = 2.84385729e-02;
    *xP++ = 4.46348606e-03;
    generateC1Pulse(sps, pulse);
  } else {
    center = (float) (len - 1.0) / 2.0;

    /* GSM pulse approximation */
    for (int i = 0; i < len; i++) {
      arg = ((float) i - center) / (float) sps;
      *xP++ = 0.96 * exp(-1.1380 * arg * arg -
			 0.527 * arg * arg * arg * arg);
    }

    avg = sqrtf(vectorNorm2(*pulse->c0) / sps);
    xP = pulse->c0->begin();
    for (int i = 0; i < len; i++)
      *xP++ /= avg;
  }

  /*
   * Current form of the EDGE equalization filter non-realizable at 4 SPS.
   * Load the onto both 1 SPS and 4 SPS objects for convenience. Note that
   * the EDGE demodulator downsamples to 1 SPS prior to equalization.
   */
  generateInvertC0Pulse(pulse);

  return pulse;
}

/* Convert -1..+1 soft bits to 0..1 soft bits */
void vectorSlicer(float *dest, const float *src, size_t len)
{
	size_t i;
	for (i = 0; i < len; i++) {
		dest[i] = 0.5 * (src[i] + 1.0f);
		if (dest[i] > 1.0)
			dest[i] = 1.0;
		else if (dest[i] < 0.0)
			dest[i] = 0.0;
	}
}

static signalVector *rotateBurst(const BitVector &wBurst,
                                 int guardPeriodLength, int sps)
{
  int burst_len;
  signalVector *pulse, rotated;
  signalVector::iterator itr;

  pulse = GSMPulse1->empty;
  burst_len = sps * (wBurst.size() + guardPeriodLength);
  rotated = signalVector(burst_len);
  itr = rotated.begin();

  for (unsigned i = 0; i < wBurst.size(); i++) {
    *itr = 2.0 * (wBurst[i] & 0x01) - 1.0;
    itr += sps;
  }

  GMSKRotate(rotated, sps);
  rotated.isReal(false);

  /* Dummy filter operation */
  return convolve(&rotated, pulse, NULL, START_ONLY);
}

static void rotateBurst2(signalVector &burst, double phase)
{
  Complex<float> rot = Complex<float>(cos(phase), sin(phase));

  for (size_t i = 0; i < burst.size(); i++)
    burst[i] = burst[i] * rot;
}

/*
 * Ignore the guard length argument in the GMSK modulator interface
 * because it results in 624/628 sized bursts instead of the preferred
 * burst length of 625. Only 4 SPS is supported.
 */
static signalVector *modulateBurstLaurent(const BitVector &bits)
{
  int burst_len, sps = 4;
  float phase;
  signalVector *c0_pulse, *c1_pulse, *c0_shaped, *c1_shaped;
  signalVector::iterator c0_itr, c1_itr;

  c0_pulse = GSMPulse4->c0;
  c1_pulse = GSMPulse4->c1;

  if (bits.size() > 156)
    return NULL;

  burst_len = 625;

  signalVector c0_burst(burst_len, c0_pulse->size());
  c0_burst.isReal(true);
  c0_itr = c0_burst.begin();

  signalVector c1_burst(burst_len, c1_pulse->size());
  c1_itr = c1_burst.begin();

  /* Padded differential tail bits */
  *c0_itr = 2.0 * (0x00 & 0x01) - 1.0;
  c0_itr += sps;

  /* Main burst bits */
  for (unsigned i = 0; i < bits.size(); i++) {
    *c0_itr = 2.0 * (bits[i] & 0x01) - 1.0;
    c0_itr += sps;
  }

  /* Padded differential tail bits */
  *c0_itr = 2.0 * (0x00 & 0x01) - 1.0;

  /* Generate C0 phase coefficients */
  GMSKRotate(c0_burst, sps);
  c0_burst.isReal(false);

  c0_itr = c0_burst.begin();
  c0_itr += sps * 2;
  c1_itr += sps * 2;

  /* Start magic */
  phase = 2.0 * ((0x01 & 0x01) ^ (0x01 & 0x01)) - 1.0;
  *c1_itr = *c0_itr * Complex<float>(0, phase);
  c0_itr += sps;
  c1_itr += sps;

  /* Generate C1 phase coefficients */
  for (unsigned i = 2; i < bits.size(); i++) {
    phase = 2.0 * ((bits[i - 1] & 0x01) ^ (bits[i - 2] & 0x01)) - 1.0;
    *c1_itr = *c0_itr * Complex<float>(0, phase);

    c0_itr += sps;
    c1_itr += sps;
  }

  /* End magic */
  int i = bits.size();
  phase = 2.0 * ((bits[i-1] & 0x01) ^ (bits[i-2] & 0x01)) - 1.0;
  *c1_itr = *c0_itr * Complex<float>(0, phase);

  /* Primary (C0) and secondary (C1) pulse shaping */
  c0_shaped = convolve(&c0_burst, c0_pulse, NULL, START_ONLY);
  c1_shaped = convolve(&c1_burst, c1_pulse, NULL, START_ONLY);

  /* Sum shaped outputs into C0 */
  c0_itr = c0_shaped->begin();
  c1_itr = c1_shaped->begin();
  for (unsigned i = 0; i < c0_shaped->size(); i++ )
    *c0_itr++ += *c1_itr++;

  delete c1_shaped;
  return c0_shaped;
}

static signalVector *rotateEdgeBurst(const signalVector &symbols, int sps)
{
  signalVector *burst;
  signalVector::iterator burst_itr;

  burst = new signalVector(symbols.size() * sps);
  burst_itr = burst->begin();

  for (size_t i = 0; i < symbols.size(); i++) {
    float phase = i * 3.0f * M_PI / 8.0f;
    Complex<float> rot = Complex<float>(cos(phase), sin(phase));

    *burst_itr = symbols[i] * rot;
    burst_itr += sps;
  }

  return burst;
}

static signalVector *derotateEdgeBurst(const signalVector &symbols, int sps)
{
  signalVector *burst;
  signalVector::iterator burst_itr;

  if (symbols.size() % sps)
    return NULL;

  burst = new signalVector(symbols.size() / sps);
  burst_itr = burst->begin();

  for (size_t i = 0; i < burst->size(); i++) {
    float phase = (float) (i % 16) * 3.0f * M_PI / 8.0f;
    Complex<float> rot = Complex<float>(cosf(phase), -sinf(phase));

    *burst_itr = symbols[sps * i] * rot;
    burst_itr++;
  }

  return burst;
}

static signalVector *mapEdgeSymbols(const BitVector &bits)
{
  if (bits.size() % 3)
    return NULL;

  signalVector *symbols = new signalVector(bits.size() / 3);

  for (size_t i = 0; i < symbols->size(); i++) {
    unsigned index = (((unsigned) bits[3 * i + 0] & 0x01) << 0) |
                     (((unsigned) bits[3 * i + 1] & 0x01) << 1) |
                     (((unsigned) bits[3 * i + 2] & 0x01) << 2);

    (*symbols)[i] = psk8_table[index];
  }

  return symbols;
}

/*
 * EDGE 8-PSK rotate and pulse shape
 *
 * Delay the EDGE downlink bursts by one symbol in order to match GMSK pulse
 * shaping group delay. The difference in group delay arises from the dual
 * pulse filter combination of the GMSK Laurent representation whereas 8-PSK
 * uses a single pulse linear filter.
 */
static signalVector *shapeEdgeBurst(const signalVector &symbols)
{
  size_t nsyms, nsamps = 625, sps = 4;
  signalVector::iterator burst_itr;

  nsyms = symbols.size();

  if (nsyms * sps > nsamps)
    nsyms = 156;

  signalVector burst(nsamps, GSMPulse4->c0->size());

  /* Delay burst by 1 symbol */
  burst_itr = burst.begin() + sps;
  for (size_t i = 0; i < nsyms; i++) {
    float phase = i * 3.0f * M_PI / 8.0f;
    Complex<float> rot = Complex<float>(cos(phase), sin(phase));

    *burst_itr = symbols[i] * rot;
    burst_itr += sps;
  }

  /* Single Gaussian pulse approximation shaping */
  return convolve(&burst, GSMPulse4->c0, NULL, START_ONLY);
}

/*
 * Generate a random GSM normal burst.
 */
signalVector *genRandNormalBurst(int tsc, int sps, int tn)
{
  if ((tsc < 0) || (tsc > 7) || (tn < 0) || (tn > 7))
    return NULL;
  if ((sps != 1) && (sps != 4))
    return NULL;

  int i = 0;
  BitVector bits(148);

  /* Tail bits */
  for (; i < 3; i++)
    bits[i] = 0;

  /* Random bits */
  for (; i < 60; i++)
    bits[i] = rand() % 2;

  /* Stealing bit */
  bits[i++] = 0;

  /* Training sequence */
  for (int n = 0; i < 87; i++, n++)
    bits[i] = gTrainingSequence[tsc][n];

  /* Stealing bit */
  bits[i++] = 0;

  /* Random bits */
  for (; i < 145; i++)
    bits[i] = rand() % 2;

  /* Tail bits */
  for (; i < 148; i++)
    bits[i] = 0;

  int guard = 8 + !(tn % 4);
  return modulateBurst(bits, guard, sps);
}

/*
 * Generate a random GSM access burst.
 */
signalVector *genRandAccessBurst(int delay, int sps, int tn)
{
  if ((tn < 0) || (tn > 7))
    return NULL;
  if ((sps != 1) && (sps != 4))
    return NULL;
  if (delay > 68)
    return NULL;

  int i = 0;
  BitVector bits(88 + delay);

  /* delay */
  for (; i < delay; i++)
    bits[i] = 0;

  /* head and synch bits */
  for (int n = 0; i < 49+delay; i++, n++)
    bits[i] = gRACHBurst[n];

  /* Random bits */
  for (; i < 85+delay; i++)
    bits[i] = rand() % 2;

  /* Tail bits */
  for (; i < 88+delay; i++)
    bits[i] = 0;

  int guard = 68-delay + !(tn % 4);
  return modulateBurst(bits, guard, sps);
}

signalVector *generateEmptyBurst(int sps, int tn)
{
	if ((tn < 0) || (tn > 7))
		return NULL;

	if (sps == 4)
		return new signalVector(625);
	else if (sps == 1)
		return new signalVector(148 + 8 + !(tn % 4));
	else
		return NULL;
}

signalVector *generateDummyBurst(int sps, int tn)
{
	if (((sps != 1) && (sps != 4)) || (tn < 0) || (tn > 7))
		return NULL;

	return modulateBurst(gDummyBurst, 8 + !(tn % 4), sps);
}

/*
 * Generate a random 8-PSK EDGE burst. Only 4 SPS is supported with
 * the returned burst being 625 samples in length.
 */
signalVector *generateEdgeBurst(int tsc)
{
  int tail = 9 / 3;
  int data = 174 / 3;
  int train = 78 / 3;

  if ((tsc < 0) || (tsc > 7))
    return NULL;

  signalVector burst(148);
  const BitVector *midamble = &gEdgeTrainingSequence[tsc];

  /* Tail */
  int n, i = 0;
  for (; i < tail; i++)
    burst[i] = psk8_table[7];

  /* Body */
  for (; i < tail + data; i++)
    burst[i] = psk8_table[rand() % 8];

  /* TSC */
  for (n = 0; i < tail + data + train; i++, n++) {
    unsigned index = (((unsigned) (*midamble)[3 * n + 0] & 0x01) << 0) |
                     (((unsigned) (*midamble)[3 * n + 1] & 0x01) << 1) |
                     (((unsigned) (*midamble)[3 * n + 2] & 0x01) << 2);

    burst[i] = psk8_table[index];
  }

  /* Body */
  for (; i < tail + data + train + data; i++)
    burst[i] = psk8_table[rand() % 8];

  /* Tail */
  for (; i < tail + data + train + data + tail; i++)
    burst[i] = psk8_table[7];

  return shapeEdgeBurst(burst);
}

/*
 * Modulate 8-PSK burst. When empty pulse shaping (rotation only)
 * is enabled, the output vector length will be bit sequence length
 * times the SPS value. When pulse shaping is enabled, the output
 * vector length is fixed at 625 samples (156.25 symbols at 4 SPS).
 * Pulse shaped bit sequences that go beyond one burst are truncated.
 * Pulse shaping at anything but 4 SPS is not supported.
 */
signalVector *modulateEdgeBurst(const BitVector &bits,
                                int sps, bool empty)
{
  signalVector *shape, *burst;

  if ((sps != 4) && !empty)
    return NULL;

  burst = mapEdgeSymbols(bits);
  if (!burst)
    return NULL;

  if (empty)
    shape = rotateEdgeBurst(*burst, sps);
  else
    shape = shapeEdgeBurst(*burst);

  delete burst;
  return shape;
}

static signalVector *modulateBurstBasic(const BitVector &bits,
					int guard_len, int sps)
{
  int burst_len;
  signalVector *pulse;
  signalVector::iterator burst_itr;

  if (sps == 1)
    pulse = GSMPulse1->c0;
  else
    pulse = GSMPulse4->c0;

  burst_len = sps * (bits.size() + guard_len);

  signalVector burst(burst_len, pulse->size());
  burst.isReal(true);
  burst_itr = burst.begin();

  /* Raw bits are not differentially encoded */
  for (unsigned i = 0; i < bits.size(); i++) {
    *burst_itr = 2.0 * (bits[i] & 0x01) - 1.0;
    burst_itr += sps;
  }

  GMSKRotate(burst, sps);
  burst.isReal(false);

  /* Single Gaussian pulse approximation shaping */
  return convolve(&burst, pulse, NULL, START_ONLY);
}

/* Assume input bits are not differentially encoded */
signalVector *modulateBurst(const BitVector &wBurst, int guardPeriodLength,
			    int sps, bool emptyPulse)
{
  if (emptyPulse)
    return rotateBurst(wBurst, guardPeriodLength, sps);
  else if (sps == 4)
    return modulateBurstLaurent(wBurst);
  else
    return modulateBurstBasic(wBurst, guardPeriodLength, sps);
}

static void generateSincTable()
{
  for (int i = 0; i < TABLESIZE; i++) {
    auto x = (double) i / TABLESIZE * 8 * M_PI;
    auto y = sin(x) / x;
    sincTable[i] = std::isnan(y) ? 1.0 : y;
  }
}

static float sinc(float x)
{
  if (fabs(x) >= 8 * M_PI)
    return 0.0;

  int index = (int) floorf(fabs(x) / (8 * M_PI) * TABLESIZE);

  return sincTable[index];
}

/*
 * Create fractional delay filterbank with Blackman-harris windowed
 * sinc function generator. The number of filters generated is specified
 * by the DELAYFILTS value.
 */
static void generateDelayFilters()
{
  int h_len = 20;
  complex *data;
  signalVector *h;
  signalVector::iterator itr;

  float k, sum;
  float a0 = 0.35875;
  float a1 = 0.48829;
  float a2 = 0.14128;
  float a3 = 0.01168;

  for (int i = 0; i < DELAYFILTS; i++) {
    data = (complex *) convolve_h_alloc(h_len);
    h = new signalVector(data, 0, h_len, convolve_h_alloc, free);
    h->setAligned(true);
    h->isReal(true);

    sum = 0.0;
    itr = h->end();
    for (int n = 0; n < h_len; n++) {
      k = (float) n;
      *--itr = (complex) sinc(M_PI_F *
                         (k - (float) h_len / 2.0 - (float) i / DELAYFILTS));
      *itr *= a0 -
        a1 * cos(2 * M_PI * n / (h_len - 1)) +
        a2 * cos(4 * M_PI * n / (h_len - 1)) -
        a3 * cos(6 * M_PI * n / (h_len - 1));

      sum += itr->real();
    }

    itr = h->begin();
    for (int n = 0; n < h_len; n++)
      *itr++ /= sum;

    delayFilters[i] = h;
  }
}

signalVector *delayVector(const signalVector *in, signalVector *out, float delay)
{
  int whole, index;
  float frac;
  signalVector *h, *shift, *fshift = NULL;

  whole = floor(delay);
  frac = delay - whole;

  /* Sinc interpolated fractional shift (if allowable) */
  if (fabs(frac) > 1e-2) {
    index = floorf(frac * (float) DELAYFILTS);
    h = delayFilters[index];

    fshift = convolve(in, h, NULL, NO_DELAY);
    if (!fshift)
      return NULL;
  }

  if (!fshift)
    shift = new signalVector(*in);
  else
    shift = fshift;

  /* Integer sample shift */
  if (whole < 0) {
    whole = -whole;
    signalVector::iterator wBurstItr = shift->begin();
    signalVector::iterator shiftedItr = shift->begin() + whole;

    while (shiftedItr < shift->end())
      *wBurstItr++ = *shiftedItr++;

    while (wBurstItr < shift->end())
      *wBurstItr++ = 0.0;
  } else if (whole >= 0) {
    signalVector::iterator wBurstItr = shift->end() - 1;
    signalVector::iterator shiftedItr = shift->end() - 1 - whole;

    while (shiftedItr >= shift->begin())
      *wBurstItr-- = *shiftedItr--;

    while (wBurstItr >= shift->begin())
      *wBurstItr-- = 0.0;
  }

  if (!out)
    return shift;

  out->clone(*shift);
  delete shift;
  return out;
}

static complex interpolatePoint(const signalVector &inSig, float ix)
{
  int start = (int) (floor(ix) - 10);
  if (start < 0) start = 0;
  int end = (int) (floor(ix) + 11);
  if ((unsigned) end > inSig.size()-1) end = inSig.size()-1;

  complex pVal = 0.0;
  if (!inSig.isReal()) {
    for (int i = start; i < end; i++)
      pVal += inSig[i] * sinc(M_PI_F*(i-ix));
  }
  else {
    for (int i = start; i < end; i++)
      pVal += inSig[i].real() * sinc(M_PI_F*(i-ix));
  }

  return pVal;
}

static complex fastPeakDetect(const signalVector &rxBurst, float *index)
{
  float val, max = 0.0f;
  complex amp;
  int _index = -1;

  for (size_t i = 0; i < rxBurst.size(); i++) {
    val = rxBurst[i].norm2();
    if (val > max) {
      max = val;
      _index = i;
      amp = rxBurst[i];
    }
  }

  if (index)
    *index = (float) _index;

  return amp;
}

static complex peakDetect(const signalVector &rxBurst,
                          float *peakIndex, float *avgPwr)
{
  complex maxVal = 0.0;
  float maxIndex = -1;
  float sumPower = 0.0;

  for (unsigned int i = 0; i < rxBurst.size(); i++) {
    float samplePower = rxBurst[i].norm2();
    if (samplePower > maxVal.real()) {
      maxVal = samplePower;
      maxIndex = i;
    }
    sumPower += samplePower;
  }

  // interpolate around the peak
  // to save computation, we'll use early-late balancing
  float earlyIndex = maxIndex-1;
  float lateIndex = maxIndex+1;

  float incr = 0.5;
  while (incr > 1.0/1024.0) {
    complex earlyP = interpolatePoint(rxBurst,earlyIndex);
    complex lateP =  interpolatePoint(rxBurst,lateIndex);
    if (earlyP < lateP)
      earlyIndex += incr;
    else if (earlyP > lateP)
      earlyIndex -= incr;
    else break;
    incr /= 2.0;
    lateIndex = earlyIndex + 2.0;
  }

  maxIndex = earlyIndex + 1.0;
  maxVal = interpolatePoint(rxBurst,maxIndex);

  if (peakIndex!=NULL)
    *peakIndex = maxIndex;

  if (avgPwr!=NULL)
    *avgPwr = (sumPower-maxVal.norm2()) / (rxBurst.size()-1);

  return maxVal;

}

void scaleVector(signalVector &x,
		 complex scale)
{
#ifdef HAVE_NEON
  int len = x.size();

  scale_complex((float *) x.begin(),
                (float *) x.begin(),
                (float *) &scale, len);
#else
  signalVector::iterator xP = x.begin();
  signalVector::iterator xPEnd = x.end();
  if (!x.isReal()) {
    while (xP < xPEnd) {
      *xP = *xP * scale;
      xP++;
    }
  }
  else {
    while (xP < xPEnd) {
      *xP = xP->real() * scale;
      xP++;
    }
  }
#endif
}

/** in-place conjugation */
static void conjugateVector(signalVector &x)
{
  if (x.isReal()) return;
  signalVector::iterator xP = x.begin();
  signalVector::iterator xPEnd = x.end();
  while (xP < xPEnd) {
    *xP = xP->conj();
    xP++;
  }
}

static bool generateMidamble(int sps, int tsc)
{
  bool status = true;
  float toa;
  complex *data = NULL;
  signalVector *autocorr = NULL, *midamble = NULL;
  signalVector *midMidamble = NULL, *_midMidamble = NULL;

  if ((tsc < 0) || (tsc > 7))
    return false;

  delete gMidambles[tsc];

  /* Use middle 16 bits of each TSC. Correlation sequence is not pulse shaped */
  midMidamble = modulateBurst(gTrainingSequence[tsc].segment(5,16), 0, sps, true);
  if (!midMidamble)
    return false;

  /* Simulated receive sequence is pulse shaped */
  midamble = modulateBurst(gTrainingSequence[tsc], 0, sps, false);
  if (!midamble) {
    status = false;
    goto release;
  }

  // NOTE: Because ideal TSC 16-bit midamble is 66 symbols into burst,
  //       the ideal TSC has an + 180 degree phase shift,
  //       due to the pi/2 frequency shift, that
  //       needs to be accounted for.
  //       26-midamble is 61 symbols into burst, has +90 degree phase shift.
  scaleVector(*midMidamble, complex(-1.0, 0.0));
  scaleVector(*midamble, complex(0.0, 1.0));

  conjugateVector(*midMidamble);

  /* For SSE alignment, reallocate the midamble sequence on 16-byte boundary */
  data = (complex *) convolve_h_alloc(midMidamble->size());
  _midMidamble = new signalVector(data, 0, midMidamble->size(), convolve_h_alloc, free);
  _midMidamble->setAligned(true);
  midMidamble->copyTo(*_midMidamble);

  autocorr = convolve(midamble, _midMidamble, NULL, NO_DELAY);
  if (!autocorr) {
    status = false;
    goto release;
  }

  gMidambles[tsc] = new CorrelationSequence;
  gMidambles[tsc]->sequence = _midMidamble;
  gMidambles[tsc]->gain = peakDetect(*autocorr, &toa, NULL);

  /* For 1 sps only
   *     (Half of correlation length - 1) + midpoint of pulse shape + remainder
   *     13.5 = (16 / 2 - 1) + 1.5 + (26 - 10) / 2
   */
  if (sps == 1)
    gMidambles[tsc]->toa = toa - 13.5;
  else
    gMidambles[tsc]->toa = 0;

release:
  delete autocorr;
  delete midamble;
  delete midMidamble;

  if (!status) {
    delete _midMidamble;
    free(data);
    gMidambles[tsc] = NULL;
  }

  return status;
}

static CorrelationSequence *generateEdgeMidamble(int tsc)
{
  complex *data = NULL;
  signalVector *midamble = NULL, *_midamble = NULL;
  CorrelationSequence *seq;

  if ((tsc < 0) || (tsc > 7))
    return NULL;

  /* Use middle 48 bits of each TSC. Correlation sequence is not pulse shaped */
  const BitVector *bits = &gEdgeTrainingSequence[tsc];
  midamble = modulateEdgeBurst(bits->segment(15, 48), 1, true);
  if (!midamble)
    return NULL;

  conjugateVector(*midamble);

  data = (complex *) convolve_h_alloc(midamble->size());
  _midamble = new signalVector(data, 0, midamble->size(), convolve_h_alloc, free);
  _midamble->setAligned(true);
  midamble->copyTo(*_midamble);

  /* Channel gain is an empirically measured value */
  seq = new CorrelationSequence;
  seq->sequence = _midamble;
  seq->gain = Complex<float>(-19.6432, 19.5006) / 1.18;
  seq->toa = 0;

  delete midamble;

  return seq;
}

static bool generateRACHSequence(CorrelationSequence **seq, const BitVector &bv, int sps)
{
  bool status = true;
  float toa;
  complex *data = NULL;
  signalVector *autocorr = NULL;
  signalVector *seq0 = NULL, *seq1 = NULL, *_seq1 = NULL;

  if (*seq != NULL)
    delete *seq;

  seq0 = modulateBurst(bv, 0, sps, false);
  if (!seq0)
    return false;

  seq1 = modulateBurst(bv.segment(0, 40), 0, sps, true);
  if (!seq1) {
    status = false;
    goto release;
  }

  conjugateVector(*seq1);

  /* For SSE alignment, reallocate the midamble sequence on 16-byte boundary */
  data = (complex *) convolve_h_alloc(seq1->size());
  _seq1 = new signalVector(data, 0, seq1->size(), convolve_h_alloc, free);
  _seq1->setAligned(true);
  seq1->copyTo(*_seq1);

  autocorr = convolve(seq0, _seq1, autocorr, NO_DELAY);
  if (!autocorr) {
    status = false;
    goto release;
  }

  *seq = new CorrelationSequence;
  (*seq)->sequence = _seq1;
  (*seq)->gain = peakDetect(*autocorr, &toa, NULL);

  /* For 1 sps only
   *     (Half of correlation length - 1) + midpoint of pulse shaping filer
   *     20.5 = (40 / 2 - 1) + 1.5
   */
  if (sps == 1)
    (*seq)->toa = toa - 20.5;
  else
    (*seq)->toa = 0.0;

release:
  delete autocorr;
  delete seq0;
  delete seq1;

  if (!status) {
    delete _seq1;
    free(data);
    *seq = NULL;
  }

  return status;
}

/*
 * Peak-to-average computation +/- range from peak in symbols
 */
#define COMPUTE_PEAK_MIN     2
#define COMPUTE_PEAK_MAX     5

/*
 * Minimum number of values needed to compute peak-to-average
 */
#define COMPUTE_PEAK_CNT     5

static float computePeakRatio(signalVector *corr,
                              int sps, float toa, complex amp)
{
  int num = 0;
  complex *peak;
  float rms, avg = 0.0;

  /* Check for bogus results */
  if ((toa < 0.0) || (toa > corr->size()))
    return 0.0;

  peak = corr->begin() + (int) rint(toa);

  for (int i = COMPUTE_PEAK_MIN * sps; i <= COMPUTE_PEAK_MAX * sps; i++) {
    if (peak - i >= corr->begin()) {
      avg += (peak - i)->norm2();
      num++;
    }
    if (peak + i < corr->end()) {
      avg += (peak + i)->norm2();
      num++;
    }
  }

  if (num < COMPUTE_PEAK_CNT)
    return 0.0;

  rms = sqrtf(avg / (float) num) + 0.00001;

  return (amp.abs()) / rms;
}

float energyDetect(const signalVector &rxBurst, unsigned windowLength)
{

  signalVector::const_iterator windowItr = rxBurst.begin(); //+rxBurst.size()/2 - 5*windowLength/2;
  float energy = 0.0;
  if (windowLength == 0) return 0.0;
  if (windowLength > rxBurst.size()) windowLength = rxBurst.size();
  for (unsigned i = 0; i < windowLength; i++) {
    energy += windowItr->norm2();
    windowItr+=4;
  }
  return energy/windowLength;
}

static signalVector *downsampleBurst(const signalVector &burst)
{
  signalVector in(DOWNSAMPLE_IN_LEN, dnsampler->len());
  signalVector *out = new signalVector(DOWNSAMPLE_OUT_LEN);
  burst.copyToSegment(in, 0, DOWNSAMPLE_IN_LEN);

  if (dnsampler->rotate((float *) in.begin(), DOWNSAMPLE_IN_LEN,
                        (float *) out->begin(), DOWNSAMPLE_OUT_LEN) < 0) {
    delete out;
    out = NULL;
  }

  return out;
};

/*
 * Computes C/I (Carrier-to-Interference ratio) in dB (deciBels).
 * It is computed from the training sequence of each received burst,
 * by comparing the "ideal" training sequence with the actual one.
 */
static float computeCI(const signalVector *burst, const CorrelationSequence *sync,
                       float toa, int start, const complex &xcorr)
{
  const int N = sync->sequence->size();
  float S, C;
  /* Integer position where the sequence starts */
  const int ps = start + 1 - N + (int)roundf(toa);

  /* Estimate Signal power */
  S = 0.0f;
  for (int i=0, j=ps; i<(int)N; i++,j++)
    S += (*burst)[j].norm2();
  S /= N;

  /* Esimate Carrier power */
  C = xcorr.norm2() / ((N - 1) * sync->gain.abs());

  /* Interference = Signal - Carrier, so C/I = C / (S - C) */
  return 3.0103f * log2f(C / (S - C));
}

/*
 * Detect a burst based on correlation and peak-to-average ratio
 *
 * For one sampler-per-symbol, perform fast peak detection (no interpolation)
 * for initial gating. We do this because energy detection should be disabled.
 * For higher oversampling values, we assume the energy detector is in place
 * and we run full interpolating peak detection.
 */
static int detectBurst(const signalVector &burst,
                       signalVector &corr, const CorrelationSequence *sync,
                       float thresh, int sps, int start, int len,
                       struct estim_burst_params *ebp)
{
  const signalVector *corr_in;
  signalVector *dec = NULL;
  complex xcorr;
  int rc = 1;

  switch (sps) {
  case 1:
    corr_in = &burst;
    break;
  case 4:
    dec = downsampleBurst(burst);
     /* Running at the downsampled rate at this point: */
     corr_in = dec;
     sps = 1;
     break;
  default:
     osmo_panic("%s:%d SPS %d not supported! Only 1 or 4 supported", __FILE__, __LINE__, sps);
  }

  /* Correlate */
  if (!convolve(corr_in, sync->sequence, &corr,
                CUSTOM, start, len)) {
    rc = -1;
    goto del_ret;
  }

  /* Peak detection - place restrictions at correlation edges */
  ebp->amp = fastPeakDetect(corr, &ebp->toa);

  if ((ebp->toa < 3 * sps) || (ebp->toa > len - 3 * sps)) {
    rc = 0;
    goto del_ret;
  }

  /* Peak-to-average ratio */
  if (computePeakRatio(&corr, sps, ebp->toa, ebp->amp) < thresh) {
    rc = 0;
    goto del_ret;
  }

  /* Refine TOA and correlation value */
  xcorr = peakDetect(corr, &ebp->toa, NULL);

  /* Compute C/I */
  ebp->ci = computeCI(corr_in, sync, ebp->toa, start, xcorr);

  /* Normalize our channel gain */
  ebp->amp = xcorr / sync->gain;

  /* Compensate for residuate time lag */
  ebp->toa = ebp->toa - sync->toa;

del_ret:
  delete dec;
  return rc;
}

static float maxAmplitude(const signalVector &burst)
{
    float max = 0.0;
    for (size_t i = 0; i < burst.size(); i++) {
        if (fabs(burst[i].real()) > max)
            max = fabs(burst[i].real());
        if (fabs(burst[i].imag()) > max)
            max = fabs(burst[i].imag());
    }

    return max;
}

/*
 * RACH/Normal burst detection with clipping detection
 *
 * Correlation window parameters:
 *   target: Tail bits + burst length
 *   head: Search symbols before target
 *   tail: Search symbols after target
 */
static int detectGeneralBurst(const signalVector &rxBurst, float thresh, int sps,
                              int target, int head, int tail,
                              CorrelationSequence *sync,
                              struct estim_burst_params *ebp)
{
  int rc, start, len;
  bool clipping = false;

  if ((sps != 1) && (sps != 4))
    return -SIGERR_UNSUPPORTED;

  // Detect potential clipping
  // We still may be able to demod the burst, so we'll give it a try
  // and only report clipping if we can't demod.
  float maxAmpl = maxAmplitude(rxBurst);
  if (maxAmpl > CLIP_THRESH) {
    LOG(DEBUG) << "max burst amplitude: " << maxAmpl << " is above the clipping threshold: " << CLIP_THRESH << std::endl;
    clipping = true;
  }

  start = target - head - 1;
  len = head + tail;
  signalVector corr(len);

  rc = detectBurst(rxBurst, corr, sync,
                   thresh, sps, start, len, ebp);
  if (rc < 0) {
    return -SIGERR_INTERNAL;
  } else if (!rc) {
    ebp->amp = 0.0f;
    ebp->toa = 0.0f;
    ebp->ci = 0.0f;
    return clipping?-SIGERR_CLIP:SIGERR_NONE;
  }

  /* Subtract forward search bits from delay */
  ebp->toa -= head;

  return 1;
}


/*
 * RACH burst detection
 *
 * Correlation window parameters:
 *   target: Tail bits + RACH length (reduced from 41 to a multiple of 4)
 *   head: Search 8 symbols before target
 *   tail: Search 8 symbols + maximum expected delay
 */
static int detectRACHBurst(const signalVector &burst, float threshold, int sps,
                           unsigned max_toa, bool ext, struct estim_burst_params *ebp)
{
  int rc, target, head, tail;
  int i, num_seq;

  target = 8 + 40;
  head = 8;
  tail = 8 + max_toa;
  num_seq = ext ? 3 : 1;

  for (i = 0; i < num_seq; i++) {
    rc = detectGeneralBurst(burst, threshold, sps, target, head, tail,
                            gRACHSequences[i], ebp);
    if (rc > 0) {
      ebp->tsc = i;
      break;
    }
  }

  return rc;
}

/*
 * Normal burst detection
 *
 * Correlation window parameters:
 *   target: Tail + data + mid-midamble + 1/2 remaining midamblebits
 *   head: Search 6 symbols before target
 *   tail: Search 6 symbols + maximum expected delay
 */
static int analyzeTrafficBurst(const signalVector &burst, unsigned tsc, float threshold,
                               int sps, unsigned max_toa, struct estim_burst_params *ebp)
{
  int rc, target, head, tail;
  CorrelationSequence *sync;

  if (tsc > 7)
    return -SIGERR_UNSUPPORTED;

  target = 3 + 58 + 16 + 5;
  head = 6;
  tail = 6 + max_toa;
  sync = gMidambles[tsc];

  ebp->tsc = tsc;
  rc = detectGeneralBurst(burst, threshold, sps, target, head, tail, sync, ebp);
  return rc;
}

static int detectEdgeBurst(const signalVector &burst, unsigned tsc, float threshold,
                           int sps, unsigned max_toa, struct estim_burst_params *ebp)
{
  int rc, target, head, tail;
  CorrelationSequence *sync;

  if (tsc > 7)
    return -SIGERR_UNSUPPORTED;

  target = 3 + 58 + 16 + 5;
  head = 6;
  tail = 6 + max_toa;
  sync = gEdgeMidambles[tsc];

  ebp->tsc = tsc;
  rc = detectGeneralBurst(burst, threshold, sps,
                          target, head, tail, sync, ebp);
  return rc;
}

int detectAnyBurst(const signalVector &burst, unsigned tsc, float threshold,
                   int sps, CorrType type, unsigned max_toa,
                   struct estim_burst_params *ebp)
{
  int rc = 0;

  switch (type) {
  case EDGE:
    rc = detectEdgeBurst(burst, tsc, threshold, sps, max_toa, ebp);
    if (rc > 0)
      break;
    else
      type = TSC;
  case TSC:
    rc = analyzeTrafficBurst(burst, tsc, threshold, sps, max_toa, ebp);
    break;
  case EXT_RACH:
  case RACH:
    rc = detectRACHBurst(burst, threshold, sps, max_toa, type == EXT_RACH, ebp);
    break;
  default:
    LOG(ERR) << "Invalid correlation type";
  }

  if (rc > 0)
    return type;

  return rc;
}

/*
 * Soft 8-PSK decoding using Manhattan distance metric
 */
static SoftVector *softSliceEdgeBurst(signalVector &burst)
{
  size_t nsyms = 148;

  if (burst.size() < nsyms)
    return NULL;

  signalVector::iterator itr;
  SoftVector *bits = new SoftVector(nsyms * 3);

  /*
   * Bits 0 and 1 - First and second bits of the symbol respectively
   */
  rotateBurst2(burst, -M_PI / 8.0);
  itr = burst.begin();
  for (size_t i = 0; i < nsyms; i++) {
    (*bits)[3 * i + 0] = -itr->imag();
    (*bits)[3 * i + 1] = itr->real();
    itr++;
  }

  /*
   * Bit 2 - Collapse symbols into quadrant 0 (positive X and Y).
   * Decision area is then simplified to X=Y axis. Rotate again to
   * place decision boundary on X-axis.
   */
  itr = burst.begin();
  for (size_t i = 0; i < burst.size(); i++) {
    burst[i] = Complex<float>(fabs(itr->real()), fabs(itr->imag()));
    itr++;
  }

  rotateBurst2(burst, -M_PI / 4.0);
  itr = burst.begin();
  for (size_t i = 0; i < nsyms; i++) {
    (*bits)[3 * i + 2] = -itr->imag();
    itr++;
  }

  signalVector soft(bits->size());
  for (size_t i = 0; i < bits->size(); i++)
    soft[i] = (*bits)[i];

  return bits;
}

/*
 * Convert signalVector to SoftVector by taking real part of the signal.
 */
static SoftVector *signalToSoftVector(signalVector *dec)
{
  SoftVector *bits = new SoftVector(dec->size());

  SoftVector::iterator bit_itr = bits->begin();
  signalVector::iterator burst_itr = dec->begin();

  for (; burst_itr < dec->end(); burst_itr++)
    *bit_itr++ = burst_itr->real();

  return bits;
}

/*
 * Shared portion of GMSK and EDGE demodulators consisting of timing
 * recovery and single tap channel correction. For 4 SPS (if activated),
 * the output is downsampled prior to the 1 SPS modulation specific
 * stages.
 */
static signalVector *demodCommon(const signalVector &burst, int sps,
                                 const struct estim_burst_params *ebp)
{
  signalVector *delay, *dec;

  if ((sps != 1) && (sps != 4))
    return NULL;

  delay = delayVector(&burst, NULL, -ebp->toa * (float) sps);
  scaleVector(*delay, (complex) 1.0 / ebp->amp);

  if (sps == 1)
    return delay;

  dec = downsampleBurst(*delay);

  delete delay;
  return dec;
}

/*
 * Demodulate GSMK burst. Prior to symbol rotation, operate at
 * 4 SPS (if activated) to minimize distortion through the fractional
 * delay filters. Symbol rotation and after always operates at 1 SPS.
 */
static SoftVector *demodGmskBurst(const signalVector &rxBurst, int sps,
				  const struct estim_burst_params *ebp)
{
  SoftVector *bits;
  signalVector *dec;

  dec = demodCommon(rxBurst, sps, ebp);
  if (!dec)
    return NULL;

  /* Shift up by a quarter of a frequency */
  GMSKReverseRotate(*dec, 1);
  /* Take real part of the signal */
  bits = signalToSoftVector(dec);
  delete dec;

  return bits;
}

static float computeEdgeCI(const signalVector *rot)
{
  float err_pwr = 0.0f;
  float step = 2.0f * M_PI_F / 8.0f;

  for (size_t i = 8; i < rot->size() - 8; i++) {
    /* Compute the ideal symbol */
    complex sym = (*rot)[i];
    float phase = step * roundf(sym.arg() / step);
    complex ideal = complex(cos(phase), sin(phase));

    /* Compute the error vector */
    complex err = ideal - sym;

    /* Accumulate power */
    err_pwr += err.norm2();
  }

  return 3.0103f * log2f(1.0f * (rot->size() - 16) / err_pwr);
}

/*
 * Demodulate an 8-PSK burst. Prior to symbol rotation, operate at
 * 4 SPS (if activated) to minimize distortion through the fractional
 * delay filters. Symbol rotation and after always operates at 1 SPS.
 *
 * Allow 1 SPS demodulation here, but note that other parts of the
 * transceiver restrict EDGE operatoin to 4 SPS - 8-PSK distortion
 * through the fractional delay filters at 1 SPS renders signal
 * nearly unrecoverable.
 */
static SoftVector *demodEdgeBurst(const signalVector &burst, int sps,
                                  struct estim_burst_params *ebp)
{
  SoftVector *bits;
  signalVector *dec, *rot, *eq;

  dec = demodCommon(burst, sps, ebp);
  if (!dec)
    return NULL;

  /* Equalize and derotate */
  eq = convolve(dec, GSMPulse4->c0_inv, NULL, NO_DELAY);
  rot = derotateEdgeBurst(*eq, 1);
  ebp->ci = computeEdgeCI(rot);

  /* Soft slice and normalize */
  bits = softSliceEdgeBurst(*rot);

  delete dec;
  delete eq;
  delete rot;

  return bits;
}

SoftVector *demodAnyBurst(const signalVector &burst, CorrType type,
                          int sps, struct estim_burst_params *ebp)
{
  if (type == EDGE)
    return demodEdgeBurst(burst, sps, ebp);
  else
    return demodGmskBurst(burst, sps, ebp);
}

bool sigProcLibSetup()
{
  generateSincTable();
  initGMSKRotationTables();

  GSMPulse1 = generateGSMPulse(1);
  GSMPulse4 = generateGSMPulse(4);

  generateRACHSequence(&gRACHSequences[0], gRACHSynchSequenceTS0, 1);
  generateRACHSequence(&gRACHSequences[1], gRACHSynchSequenceTS1, 1);
  generateRACHSequence(&gRACHSequences[2], gRACHSynchSequenceTS2, 1);

  for (int tsc = 0; tsc < 8; tsc++) {
    generateMidamble(1, tsc);
    gEdgeMidambles[tsc] = generateEdgeMidamble(tsc);
  }

  generateDelayFilters();

  dnsampler = new Resampler(1, 4);
  if (!dnsampler->init()) {
    LOG(ALERT) << "Rx resampler failed to initialize";
    goto fail;
  }

  return true;

fail:
  sigProcLibDestroy();
  return false;
}
