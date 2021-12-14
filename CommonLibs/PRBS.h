/*
 * Copyright (C) 2017 Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
 *
 * SPDX-License-Identifier: LGPL-2.1+
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

#ifndef PRBS_H
#define PRBS_H

#include <stdint.h>
#include <assert.h>

/** Pseudo-random binary sequence (PRBS) generator (a Galois LFSR implementation). */
class PRBS {
public:

  PRBS(unsigned wLen, uint64_t wCoeff, uint64_t wState = 0x01)
    : mCoeff(wCoeff), mStartState(wState), mState(wState), mLen(wLen)
  { assert(wLen<=64); }

  /**@name Accessors */
  //@{
  uint64_t coeff() const { return mCoeff; }
  uint64_t state() const { return mState; }
  void state(uint64_t state) { mState = state & mask(); }
  unsigned size() const { return mLen; }
  //@}

  /**
    Calculate one bit of a PRBS
  */
  unsigned generateBit()
  {
    const unsigned result = mState & 0x01;
    processBit(result);
    return result;
  }

  /**
    Update the generator state by one bit.
    If you want to synchronize your PRBS to a known state, call this function
    size() times passing your PRBS to it bit by bit.
  */
  void processBit(unsigned inBit)
  {
    mState >>= 1;
    if (inBit) mState ^= mCoeff;
  }

  /** Return true when PRBS is wrapping through initial state */
  bool isFinished() const { return mStartState == mState; }

protected:

  uint64_t mCoeff;      ///< polynomial coefficients. LSB is zero exponent.
  uint64_t mStartState; ///< initial shift register state.
  uint64_t mState;      ///< shift register state.
  unsigned mLen;        ///< number of bits used in shift register

  /** Return mask for the state register */
  uint64_t mask() const { return (mLen==64)?0xFFFFFFFFFFFFFFFFUL:((1<<mLen)-1); }

};

/**
  A standard 9-bit based pseudorandom binary sequence (PRBS) generator.
  Polynomial: x^9 + x^5 + 1
*/
class PRBS9 : public PRBS {
  public:
  PRBS9(uint64_t wState = 0x01)
  : PRBS(9, 0x0110, wState)
  {}
};

/**
  A standard 15-bit based pseudorandom binary sequence (PRBS) generator.
  Polynomial: x^15 + x^14 + 1
*/
class PRBS15 : public PRBS {
public:
  PRBS15(uint64_t wState = 0x01)
  : PRBS(15, 0x6000, wState)
  {}
};

/**
  A standard 64-bit based pseudorandom binary sequence (PRBS) generator.
  Polynomial: x^64 + x^63 + x^61 + x^60 + 1
*/
class PRBS64 : public PRBS {
public:
  PRBS64(uint64_t wState = 0x01)
  : PRBS(64, 0xD800000000000000ULL, wState)
  {}
};

#endif // PRBS_H
