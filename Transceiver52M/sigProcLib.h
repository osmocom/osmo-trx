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
#include "signalVector.h"

/* Burst lengths */
#define NORMAL_BURST_NBITS		148
#define EDGE_BURST_NBITS		444
#define EDGE_BURST_NSYMS		(EDGE_BURST_NBITS / 3)

/** Convolution type indicator */
enum ConvType {
  START_ONLY,
  NO_DELAY,
  CUSTOM,
  UNDEFINED,
};

enum signalError {
  SIGERR_NONE,
  SIGERR_BOUNDS,
  SIGERR_CLIP,
  SIGERR_UNSUPPORTED,
  SIGERR_INTERNAL,
};

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

/** Operate soft slicer on real-valued portion of vector */ 
bool vectorSlicer(signalVector *x);

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
signalVector *genRandNormalBurst(int tsc, int sps, int tn);

/** Generate an access GSM burst with random payload - 4 or 1 SPS */
signalVector *genRandAccessBurst(int delay, int sps, int tn);

/** Generate a dummy GSM burst - 4 or 1 SPS */
signalVector *generateDummyBurst(int sps, int tn);

/** Sinc function */
float sinc(float x);

/** Delay a vector */
signalVector *delayVector(signalVector *in, signalVector *out, float delay);

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
        Energy detector, checks to see if received burst energy is above a threshold.
        @param rxBurst The received GSM burst of interest.
        @param windowLength The number of burst samples used to compute burst energy
        @param detectThreshold The detection threshold, a linear value.
        @param avgPwr The average power of the received burst.
        @return True if burst energy is above threshold.
*/
bool energyDetect(signalVector &rxBurst,
		  unsigned windowLength,
                  float detectThreshold,
                  float *avgPwr = NULL);

/**
        RACH correlator/detector.
        @param rxBurst The received GSM burst of interest.
        @param detectThreshold The threshold that the received burst's post-correlator SNR is compared against to determine validity.
        @param sps The number of samples per GSM symbol.
        @param amplitude The estimated amplitude of received RACH burst.
        @param TOA The estimate time-of-arrival of received RACH burst.
        @param maxTOA The maximum expected time-of-arrival
        @return positive if threshold value is reached, negative on error, zero otherwise
*/
int detectRACHBurst(signalVector &rxBurst,
                    float detectThreshold,
                    int sps,
                    complex &amplitude,
                    float &TOA,
                    unsigned maxTOA);

/**
        Normal burst correlator, detector, channel estimator.
        @param rxBurst The received GSM burst of interest.
 
        @param detectThreshold The threshold that the received burst's post-correlator SNR is compared against to determine validity.
        @param sps The number of samples per GSM symbol.
        @param amplitude The estimated amplitude of received TSC burst.
        @param TOA The estimate time-of-arrival of received TSC burst.
        @param maxTOA The maximum expected time-of-arrival
        @param requestChannel Set to true if channel estimation is desired.
        @param channelResponse The estimated channel.
        @param channelResponseOffset The time offset b/w the first sample of the channel response and the reported TOA.
        @return positive if threshold value is reached, negative on error, zero otherwise
*/
int analyzeTrafficBurst(signalVector &rxBurst,
                        unsigned TSC,
                        float detectThreshold,
                        int sps,
                        complex &amplitude,
                        float &TOA,
                        unsigned maxTOA);

/**
        EDGE burst detector
        @param burst The received GSM burst of interest
 
        @param detectThreshold The threshold that the received burst's post-correlator SNR is compared against to determine validity.
        @param sps The number of samples per GSM symbol.
        @param amplitude The estimated amplitude of received TSC burst.
        @param TOA The estimate time-of-arrival of received TSC burst.
        @param maxTOA The maximum expected time-of-arrival
        @return positive if threshold value is reached, negative on error, zero otherwise
*/
int detectEdgeBurst(signalVector &burst,
                    unsigned TSC,
                    float detectThreshold,
                    int sps,
                    complex &amplitude,
                    float &TOA,
                    unsigned maxTOA);

/**
	Downsample 4 SPS to 1 SPS using a polyphase filterbank
        @param burst Input burst of at least 624 symbols
        @return Decimated signal vector of 156 symbols
*/

signalVector *downsampleBurst(signalVector &burst);

/**
	Decimate a vector.
        @param wVector The vector of interest.
        @param factor Decimation factor.
        @return The decimated signal vector.
*/
signalVector *decimateVector(signalVector &wVector, size_t factor);

/**
        Demodulates a received burst using a soft-slicer.
	@param rxBurst The burst to be demodulated.
        @param gsmPulse The GSM pulse.
        @param sps The number of samples per GSM symbol.
        @param channel The amplitude estimate of the received burst.
        @param TOA The time-of-arrival of the received burst.
        @return The demodulated bit sequence.
*/
SoftVector *demodulateBurst(signalVector &rxBurst, int sps,
                            complex channel, float TOA);

/**
        Demodulate 8-PSK EDGE burst with soft symbol ooutput
	@param rxBurst The burst to be demodulated.
        @param sps The number of samples per GSM symbol.
        @param channel The amplitude estimate of the received burst.
        @param TOA The time-of-arrival of the received burst.
        @return The demodulated bit sequence.
*/
SoftVector *demodEdgeBurst(signalVector &rxBurst, int sps,
                           complex channel, float TOA);

#endif /* SIGPROCLIB_H */
