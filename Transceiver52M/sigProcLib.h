/*
* Copyright 2008 Free Software Foundation, Inc.
*
* This software is distributed under multiple licenses; see the COPYING file in the main directory for licensing information for this specific distribuion.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/

#ifndef SIGPROCLIB_H
#define SIGPROCLIB_H

#include "Vector.h"
#include "Complex.h"
#include "BitVector.h"
#include "PRBS.h"
#include "signalVector.h"

/* Burst lengths */
#define NORMAL_BURST_NBITS    148
#define EDGE_BURST_NBITS      444
#define EDGE_BURST_NSYMS      (EDGE_BURST_NBITS / 3)

/** Convolution type indicator */
enum ConvType {
  START_ONLY,
  NO_DELAY,
  CUSTOM,
  UNDEFINED,
};

/** Codes for burst types of received bursts*/
enum CorrType{
  OFF,         ///< timeslot is off
  TSC,         ///< timeslot should contain a normal burst
  RACH,        ///< timeslot should contain an access burst
  EDGE,        ///< timeslot should contain an EDGE burst
  IDLE         ///< timeslot is an idle (or dummy) burst
};
std::string corrTypeToString(CorrType corr);
std::ostream& operator<<(std::ostream& os, CorrType corr);

enum SignalError {
  SIGERR_NONE,
  SIGERR_BOUNDS,
  SIGERR_CLIP,
  SIGERR_UNSUPPORTED,
  SIGERR_INTERNAL,
};

/*
 * Burst detection threshold
 *
 * Decision threshold value for burst gating on peak-to-average value of
 * correlated synchronization sequences. Lower values pass more bursts up
 * to upper layers but will increase the false detection rate.
 */
#define BURST_THRESH    4.0

/** Convert a linear number to a dB value */
float dB(float x);

/** Convert a dB value into a linear value */
float dBinv(float x);

/** Compute the energy of a vector */
float vectorNorm2(const signalVector &x);

/** Compute the average power of a vector */
float vectorPower(const signalVector &x);

/** Setup the signal processing library */
bool sigProcLibSetup();

/** Destroy the signal processing library */
void sigProcLibDestroy(void);

/**
        Convolve two vectors.
        @param a,b The vectors to be convolved.
        @param c, A preallocated vector to hold the convolution result.
        @param spanType The type/span of the convolution.
        @return The convolution result or NULL on error.
*/
signalVector *convolve(const signalVector *a, const signalVector *b,
                       signalVector *c, ConvType spanType,
                       size_t start = 0, size_t len = 0,
                       size_t step = 1, int offset = 0);

/**
        Frequency shift a vector.
        @param y The frequency shifted vector.
        @param x The vector to-be-shifted.
        @param freq The digital frequency shift
        @param startPhase The starting phase of the oscillator
        @param finalPhase The final phase of the oscillator
        @return The frequency shifted vector.
*/
signalVector* frequencyShift(signalVector *y,
                             signalVector *x,
                             float freq = 0.0,
                             float startPhase = 0.0,
                             float *finalPhase=NULL);

/**
        Correlate two vectors.
        @param a,b The vectors to be correlated.
        @param c, A preallocated vector to hold the correlation result.
        @param spanType The type/span of the correlation.
        @return The correlation result.
*/
signalVector* correlate(signalVector *a,
                        signalVector *b,
                        signalVector *c,
                        ConvType spanType,
                        bool bReversedConjugated = false,
                        unsigned startIx = 0,
                        unsigned len = 0);

/** Operate soft slicer on a soft-bit vector */
bool vectorSlicer(SoftVector *x);

/** GMSK modulate a GSM burst of bits */
signalVector *modulateBurst(const BitVector &wBurst,
                            int guardPeriodLength,
                            int sps, bool emptyPulse = false);

/** 8-PSK modulate a burst of bits */
signalVector *modulateEdgeBurst(const BitVector &bits,
                                int sps, bool emptyPulse = false);

/** Generate a EDGE burst with random payload - 4 SPS (625 samples) only */
signalVector *generateEdgeBurst(int tsc);

/** Generate an empty burst - 4 or 1 SPS */
signalVector *generateEmptyBurst(int sps, int tn);

/** Generate a normal GSM burst with random payload - 4 or 1 SPS */
signalVector *genRandNormalBurst(int tsc, int sps, int tn, PRBS &prbs);

/** Generate an access GSM burst with random payload - 4 or 1 SPS */
signalVector *genRandAccessBurst(int delay, int sps, int tn);

/** Generate a dummy GSM burst - 4 or 1 SPS */
signalVector *generateDummyBurst(int sps, int tn);

/** Sinc function */
float sinc(float x);

/** Delay a vector */
signalVector *delayVector(const signalVector *in, signalVector *out, float delay);

/** Add two vectors in-place */
bool addVector(signalVector &x,
               signalVector &y);

/** Multiply two vectors in-place*/
bool multVector(signalVector &x,
                signalVector &y);

/** Generate a vector of gaussian noise */
signalVector *gaussianNoise(int length,
                            float variance = 1.0,
                            complex mean = complex(0.0));

/**
        Given a non-integer index, interpolate a sample.
        @param inSig The signal from which to interpolate.
        @param ix The index.
        @return The interpolated signal value.
*/
complex interpolatePoint(const signalVector &inSig,
                         float ix);

/**
        Given a correlator output, locate the correlation peak.
        @param rxBurst The correlator result.
        @param peakIndex Pointer to value to receive interpolated peak index.
        @param avgPower Power to value to receive mean power.
        @return Peak value.
*/
complex peakDetect(const signalVector &rxBurst,
                   float *peakIndex,
                   float *avgPwr);

/**
        Apply a scalar to a vector.
        @param x The vector of interest.
        @param scale The scalar.
*/
void scaleVector(signalVector &x,
                 complex scale);

/**
        Rough energy estimator.
        @param rxBurst A GSM burst.
        @param windowLength The number of burst samples used to compute burst energy
        @return The average power of the received burst.
*/
float energyDetect(const signalVector &rxBurst,
                   unsigned windowLength);

/**
        RACH aka Access Burst correlator/detector.
        @param burst The received GSM burst of interest.
        @param threshold The threshold that the received burst's post-correlator SNR is compared against to determine validity.
        @param sps The number of samples per GSM symbol.
        @param amplitude The estimated amplitude of received RACH burst.
        @param toa The estimate time-of-arrival of received RACH burst.
        @param max_toa The maximum expected time-of-arrival
        @return 1 if threshold value is reached,
                negative value (-SignalError) on error,
                zero (SIGERR_NONE) if no burst is detected
*/
int detectRACHBurst(const signalVector &burst,
                    float threshold,
                    int sps,
                    complex &amplitude,
                    float &toa,
                    unsigned max_toa);

/**
        GMSK Normal Burst correlator/detector.
        @param rxBurst The received GSM burst of interest.
        @param tsc Midamble type (0..7) also known as TSC
        @param threshold The threshold that the received burst's post-correlator SNR is compared against to determine validity.
        @param sps The number of samples per GSM symbol.
        @param amplitude The estimated amplitude of received TSC burst.
        @param toa The estimate time-of-arrival of received TSC burst.
        @param max_toa The maximum expected time-of-arrival
        @return 1 if threshold value is reached,
                negative value (-SignalError) on error,
                zero (SIGERR_NONE) if no burst is detected
*/
int analyzeTrafficBurst(const signalVector &burst,
                        unsigned tsc,
                        float threshold,
                        int sps,
                        complex &amplitude,
                        float &toa,
                        unsigned max_toa);

/**
        EDGE/8-PSK Normal Burst correlator/detector
        @param burst The received GSM burst of interest
        @param tsc Midamble type (0..7) also known as TSC
        @param threshold The threshold that the received burst's post-correlator SNR is compared against to determine validity.
        @param sps The number of samples per GSM symbol.
        @param amplitude The estimated amplitude of received TSC burst.
        @param toa The estimate time-of-arrival of received TSC burst.
        @param max_toa The maximum expected time-of-arrival
        @return 1 if threshold value is reached,
                negative value (-SignalError) on error,
                zero (SIGERR_NONE) if no burst is detected
*/
int detectEdgeBurst(const signalVector &burst,
                    unsigned tsc,
                    float threshold,
                    int sps,
                    complex &amplitude,
                    float &toa,
                    unsigned max_toa);

/**
        8-PSK/GMSK/RACH burst detector
        @param burst The received GSM burst of interest
        @param tsc Midamble type (0..7) also known as TSC
        @param threshold The threshold that the received burst's post-correlator SNR is compared against to determine validity.
        @param sps The number of samples per GSM symbol.
        @param amplitude The estimated amplitude of received TSC burst.
        @param toa The estimate time-of-arrival of received TSC burst (in symbols).
        @param max_toa The maximum expected time-of-arrival (in symbols).
        @return positive value (CorrType) if threshold value is reached,
                negative value (-SignalError) on error,
                zero (SIGERR_NONE) if no burst is detected
*/
int detectAnyBurst(const signalVector &burst,
                   unsigned tsc,
                   float threshold,
                   int sps,
                   CorrType type,
                   complex &amp,
                   float &toa,
                   unsigned max_toa);

/**
        Downsample 4 SPS to 1 SPS using a polyphase filterbank
        @param burst Input burst of at least 624 symbols
        @return Decimated signal vector of 156 symbols
*/
signalVector *downsampleBurst(const signalVector &burst);

/**
        Decimate a vector.
        @param wVector The vector of interest.
        @param factor Decimation factor.
        @return The decimated signal vector.
*/
signalVector *decimateVector(signalVector &wVector, size_t factor);

/**
        Demodulates a GMSK burst using a soft-slicer.
        @param rxBurst The burst to be demodulated.
        @param gsmPulse The GSM pulse.
        @param sps The number of samples per GSM symbol.
        @param channel The amplitude estimate of the received burst.
        @param TOA The time-of-arrival of the received burst.
        @return The demodulated bit sequence.
*/
SoftVector *demodGmskBurst(const signalVector &rxBurst, int sps,
                           complex channel, float TOA);

/**
        Demodulate 8-PSK EDGE burst with soft symbol ooutput
        @param rxBurst The burst to be demodulated.
        @param sps The number of samples per GSM symbol.
        @param channel The amplitude estimate of the received burst.
        @param TOA The time-of-arrival of the received burst.
        @return The demodulated bit sequence.
*/
SoftVector *demodEdgeBurst(const signalVector &rxBurst, int sps,
                           complex channel, float TOA);

/** Demodulate burst basde on type and output soft bits */
SoftVector *demodAnyBurst(const signalVector &burst, int sps,
                          complex amp, float toa, CorrType type);

#endif /* SIGPROCLIB_H */
