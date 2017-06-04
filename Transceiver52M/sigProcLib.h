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
#define NORMAL_BURST_NBITS    148
#define EDGE_BURST_NBITS      444
#define EDGE_BURST_NSYMS      (EDGE_BURST_NBITS / 3)

/** Codes for burst types of received bursts*/
enum CorrType{
  OFF,         ///< timeslot is off
  TSC,         ///< timeslot should contain a normal burst
  RACH,        ///< timeslot should contain an access burst
  EDGE,        ///< timeslot should contain an EDGE burst
  IDLE         ///< timeslot is an idle (or dummy) burst
};

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

/** Setup the signal processing library */
bool sigProcLibSetup();

/** Destroy the signal processing library */
void sigProcLibDestroy(void);

/** Operate soft slicer on a soft-bit vector */
bool vectorSlicer(SoftVector *x);

/** GMSK modulate a GSM burst of bits */
signalVector *modulateBurst(const BitVector &wBurst,
                            int guardPeriodLength,
                            int sps, bool emptyPulse = false);

signalVector *modulateBurstLaurent4(const BitVector &wBurst);
signalVector *modulateBurstLaurent2(const BitVector &wBurst);
signalVector *modulateBurstLaurent1(const BitVector &wBurst);
signalVector *modulateBurstNCO(const BitVector &wBurst);

/** 8-PSK modulate a burst of bits */
signalVector *modulateEdgeBurst(const BitVector &bits,
                                int sps, bool emptyPulse = false);

/** Generate a EDGE burst with random payload - 4 SPS (625 samples) only */
signalVector *generateEdgeBurst(int tsc);
signalVector *shapeEdgeBurst(const signalVector &symbols);

/** Generate an empty burst - 4 or 1 SPS */
signalVector *generateEmptyBurst(int sps, int tn);

/** Generate a normal GSM burst with random payload - 4 or 1 SPS */
signalVector *genRandNormalBurst(int tsc, int sps, int tn);

/** Generate an access GSM burst with random payload - 4 or 1 SPS */
signalVector *genRandAccessBurst(int delay, int sps, int tn);

/** Generate a dummy GSM burst - 4 or 1 SPS */
signalVector *generateDummyBurst(int sps, int tn);

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

/** Demodulate burst basde on type and output soft bits */
SoftVector *demodAnyBurst(const signalVector &burst, int sps,
                          complex amp, float toa, CorrType type);

#endif /* SIGPROCLIB_H */
