/*
 * Copyright (C) 2016-2017 Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
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
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <limits.h>
#include <fstream>
#include <iomanip>

#include "Logger.h"
#include "sigProcLib.h"
#include "signalVector.h"
#include "Transceiver.h"
#include "Configuration.h"

extern "C" {
#include "convolve.h"
#include "convert.h"
}

#define DEFAULT_RX_SPS        1
#define DEFAULT_SEARCH_WINDOW 30

// Tail + data + stealing + midamble + guard (without the last 0.25)
#define BURST_LEN_FULL        156
// Tail + data + stealing + midamble
#define BURST_LEN_ACTIVE      148
// Tail + data + stealing + midamble - 2*0.5
#define BURST_LEN_USEFUL      147

// Size of a sample in bytes as stores in a file
#define SAMPLE_SIZE_BYTES     (2 * sizeof(float))
// Burst length in bytes as stored in a file
#define BURST_LEN_BYTES       (BURST_LEN_FULL * SAMPLE_SIZE_BYTES)

ConfigurationTable gConfig;

struct trx_config {
  std::string log_level;
  unsigned sps;
  unsigned tsc;
  unsigned max_expected_delay_nb;
  unsigned max_expected_delay_ab;
  double full_scale;
  bool edge;
  CorrType type;
  std::string filename;
  unsigned ber_burst_avg;           ///< Average BER over this many bursts.
                                    ///< Set to 0 to average for the whole duration.
};

class NormalBurstSoftbitMask {
public:
  NormalBurstSoftbitMask(SoftVector &softBits)
  : mSoftBits(softBits)
  {
  }

  SoftVector &bits() { return mSoftBits; }
  SoftVector tailBitsL() { return mSoftBits.segment(0,3); }
  SoftVector dataBitsL() { return mSoftBits.segment(3,57); }
  SoftVector stealingBitsL() { return mSoftBits.segment(60, 1); }
  SoftVector midambleBits() { return mSoftBits.segment(61, 26); }
  SoftVector stealingBitsR() { return mSoftBits.segment(87, 1); }
  SoftVector dataBitsR() { return mSoftBits.segment(88,57); }
  SoftVector tailBitsR() { return mSoftBits.segment(145,3); }
  SoftVector guardBits() { return mSoftBits.segment(148,8); }

protected:
  SoftVector &mSoftBits;
};

class SoftBurst {
public:
  SoftBurst(SoftVector *softBits, double toa=0)
  : mSoftBits(softBits), mTOA(toa)
  {
    assert(mSoftBits != NULL);
  }

  ~SoftBurst()
  {
    delete mSoftBits;
  }

  void TOA(double TOA) { mTOA = TOA; }
  double TOA() { return mTOA; }

  NormalBurstSoftbitMask normalBurstMask() { return NormalBurstSoftbitMask(*mSoftBits); }

protected:
  SoftVector *mSoftBits;
  double mTOA;
};

class BEREstimator {
public:
  BEREstimator(const PRBS& prbs)
  : mPRBS(prbs), mTotalBits(0), mErrorBits(0), mSynchronized(false)
  {}

  unsigned synchronize(const BitVector &bits)
  {
    for (unsigned i=0; i<mPRBS.size(); i++) {
      mPRBS.processBit(bits[i]);
    }
    mSynchronized = true;
    return mPRBS.size();
  }

  void process(const BitVector &bits, size_t start_from = 0)
  {
    for (size_t i=start_from; i<bits.size(); i++) {
      mTotalBits++;
      if (mPRBS.generateBit() != bits.bit(i)) {
        mErrorBits++;
      }
    }
  }

  void sync_and_process(const BitVector &bits)
  {
    unsigned skip = 0;
    if (!mSynchronized) {
      skip = synchronize(bits);
    }

    process(bits, skip);
  }

  void skip(size_t num)
  {
    for (size_t i=0; i<num; i++) {
      mTotalBits++;
      mErrorBits++;
      mPRBS.generateBit();
    }
  }

  void reset()
  {
    mTotalBits = 0;
    mErrorBits = 0;
  }

  unsigned totalBits() const { return mTotalBits; }
  unsigned errorBits() const { return mErrorBits; }
  double BER() const { return mErrorBits/(double)mTotalBits; }

  bool isSynchronized() const {return mSynchronized; }

protected:
  PRBS mPRBS;
  unsigned mTotalBits;
  unsigned mErrorBits;
  bool mSynchronized;
};

double getBurstRSSI(const signalVector &burst, unsigned sps, double full_scale)
{
  /* Calculate average power of the burst */
  float avg = energyDetect(burst, 20 * sps);
  return 20.0 * log10(sqrt(avg) / full_scale);
}

void printDetectionResult(int rc)
{
  if (rc > 0) {
    std::cout << "Detected correlation type: " << (CorrType)rc << std::endl;
  } else {
    if (rc == -SIGERR_CLIP) {
      std::cout << "Clipping detected on received RACH or Normal Burst" << std::endl;
    } else if (rc != SIGERR_NONE) {
      std::cout << "Unhandled RACH or Normal Burst detection error" << std::endl;
    } else {
//      std::cout << "No burst detected" << std::endl;
    }
  }
}

SoftVector *demodulateBurst(const signalVector &burst,
                            CorrType expected_type,
                            unsigned sps, unsigned tsc,
                            unsigned max_expected_delay,
                            double &timingOffset)
{
  complex amp;
  float toa;
  int rc;
  CorrType detected_type;

  /* Detect normal or RACH bursts */
  rc = detectAnyBurst(burst, tsc, BURST_THRESH, sps, expected_type, amp, toa,
                      max_expected_delay);
  printDetectionResult(rc);
  if (rc <= 0) {
    return NULL;
  }

  // Convert samples to symbols
  timingOffset = toa / sps;
  // rc > 0 means it's a detected CorrType
  detected_type = (CorrType)rc;

  return demodAnyBurst(burst, sps, amp, toa, detected_type);
}

static bool processBurst(const trx_config &config, signalVector &burst,
                         unsigned max_expected_delay,
                         double &RSSI,
                         double &timingOffset,
                         BEREstimator &berEstimator)
{
  RSSI = getBurstRSSI(burst, config.sps, config.full_scale);
  SoftVector *softBits = demodulateBurst(burst, config.type, config.sps,config.tsc,
                                         max_expected_delay, timingOffset);

  /* Print burst information and content */
  if (softBits == NULL) {
    std::cout << "Skipped frame" << std::endl;
    // TODO: This is different for EDGE
    berEstimator.skip(57*2);
    return false;
  }

  SoftBurst softBurst(softBits, timingOffset);
  NormalBurstSoftbitMask nb = softBurst.normalBurstMask();

  berEstimator.sync_and_process(nb.dataBitsL().sliced());
  berEstimator.sync_and_process(nb.dataBitsR().sliced());

  std::cout << "TOA:  " << softBurst.TOA() << " symbols" << std::endl;
  // Exclude tail and guard bits from the energy calculation
  std::cout << "Energy:  " << softBits->segment(3,142).getEnergy() << std::endl;
  //std::cout << "Demodulated burst: " << *softBits << std::endl;
  std::cout << "                  tail|--------------------------data---------------------------|f|--------midamble----------|f|--------------------------data---------------------------|tai|-guard--" << std::endl;
  //           "                   000 010001011011110011101001100100000001010001011000100100010 0 11101111000100101110111100 0 011010111011101010011010111000101100001110101011011001011 000 1''..---"
  std::cout << "Demodulated burst:"
            << " " << nb.tailBitsL()
            << " " << nb.dataBitsL()
            << " " << nb.stealingBitsL()
            << " " << nb.midambleBits()
            << " " << nb.stealingBitsR()
            << " " << nb.dataBitsR()
            << " " << nb.tailBitsR()
            << " " << nb.guardBits()
            << std::endl;

  return true;
}

// Setup configuration values
static void print_config(struct trx_config *config)
{
  std::ostringstream ost("");
  ost << "Config Settings" << std::endl;
  ost << "   Source file name............. " << config->filename << std::endl;
  ost << "   Log Level.................... " << config->log_level << std::endl;
  ost << "   Rx Samples-per-Symbol........ " << config->sps << std::endl;
  ost << "   EDGE support................. " << (config->edge ? "Enabled" : "Disabled") << std::endl;
  ost << "   Burst type................... " << config->type << std::endl;
  ost << "   Burst TSC.................... " << config->tsc << std::endl;
  ost << "   Normal Burst search window... " << config->max_expected_delay_nb << std::endl;
  ost << "   Access Burst search window... " << config->max_expected_delay_ab << std::endl;
  ost << "   Signal full scale............ " << config->full_scale << std::endl;
  ost << "   BER average window (bursts).. " << config->ber_burst_avg << std::endl;
  std::cout << ost << std::endl;
}

static void print_help()
{
  fprintf(stdout, "Options:\n"
          "  -h          This text\n"
          "  -l LEVEL    Logging level (%s)\n"
          "  -e          Enable EDGE receiver\n"
          "  -s SPS      Samples-per-symbol (1 or 4, default: %d)\n"
          "  -t TSC      Burst training sequence (0 to 7, default: 0)\n"
          "  -f FILE     File to read\n"
          "  -w SYMBOLS  Normal Burst search window (0 to 156, default: %d)\n"
          "  -W SYMBOLS  Access Burst search window (0 to 156, default: %d)\n"
          "  -b BURSTS   BER average window. Set to 0 to average over the whole file (default: 1)\n",
          "EMERG, ALERT, CRT, ERR, WARNING, NOTICE, INFO, DEBUG",
          DEFAULT_RX_SPS,
          DEFAULT_SEARCH_WINDOW, DEFAULT_SEARCH_WINDOW);
}

static bool handle_options(int argc, char **argv, struct trx_config *config)
{
  int option;

  config->log_level = "NOTICE";
  config->sps = DEFAULT_RX_SPS;
  config->tsc = 0;
  config->max_expected_delay_nb = DEFAULT_SEARCH_WINDOW;
  config->max_expected_delay_ab = DEFAULT_SEARCH_WINDOW;
  config->full_scale = SHRT_MAX;
  config->edge = false;
  config->type = TSC;
  config->ber_burst_avg = 1;

  while ((option = getopt(argc, argv, "ls:et:f:w:W:b:h")) != -1) {
    switch (option) {
    case 'l':
      config->log_level = optarg;
      break;
    case 's':
      config->sps = atoi(optarg);
      break;
    case 'e':
      config->edge = true;
      break;
    case 't':
      config->tsc = atoi(optarg);
      break;
    case 'f':
      config->filename = optarg;
      break;
    case 'w':
      config->max_expected_delay_nb = atoi(optarg);
      break;
    case 'W':
      config->max_expected_delay_ab = atoi(optarg);
      break;
    case 'b':
      config->ber_burst_avg = atoi(optarg);
      break;
    case 'h':
    default:
      print_help();
      exit(0);
    }
  }

  if ((config->sps != 1) && (config->sps != 4)) {
    printf("ERROR: Unsupported samples-per-symbol %i\n\n", config->sps);
    return false;
  }

  if (config->edge && (config->sps != 4)) {
    printf("ERROR: EDGE only supported at 4 samples per symbol\n\n");
    return false;
  }

  if (config->tsc > 7) {
    printf("ERROR: Invalid training sequence %i\n\n", config->tsc);
    return false;
  }

  if (config->filename.length() == 0) {
    printf("ERROR: No input file specified\n\n");
    return false;
  }

  if (config->max_expected_delay_nb > 156 || config->max_expected_delay_nb < 0 ||
      config->max_expected_delay_ab > 156 || config->max_expected_delay_ab < 0) {
    printf("ERROR: Invalid search window size, must be withit [1..156] range\n\n");
    return false;
  }

  return true;
}

int main(int argc, char *argv[])
{
  struct trx_config config;

#ifdef HAVE_SSE3
  printf("Info: SSE3 support compiled in");
  if (__builtin_cpu_supports("sse3"))
    printf(" and supported by CPU\n");
  else
    printf(", but not supported by CPU\n");
#endif

#ifdef HAVE_SSE4_1
  printf("Info: SSE4.1 support compiled in");
  if (__builtin_cpu_supports("sse4.1"))
    printf(" and supported by CPU\n");
  else
    printf(", but not supported by CPU\n");
#endif

  convolve_init();
  convert_init();

  // Process command line options and print config to screen
  if (!handle_options(argc, argv, &config)) {
    print_help();
    exit(0);
  }
  print_config(&config);

  gLogInit("transceiver", config.log_level.c_str(), LOG_LOCAL7);

  if (!sigProcLibSetup()) {
    LOG(ALERT) << "Failed to initialize signal processing library";
    return -1;
  }

  double RSSI;
  double timingOffset, timingOffsetPrev = 0.0;
  signalVector burst(2*BURST_LEN_FULL);
  GSM::Time gsmTime;
  bool syncedTo157bits = false; // We should syncronize to 156-157 frame structure only once
  bool burst156_157 = false;    // Set to true to enable 156-156-156-157 frame
  int bitsReadExtra = 0; // set to 1 every 4 bursts and when TOA>1.0
  int bitsToSkip = 0;    // set to 1 when TOA<0.0
  unsigned berBurstsAveraged = 0;
  PRBS9 prbs;
  BEREstimator berEstimator(prbs);

  // Configure output stream
  std::cout << std::fixed;
  std::cout << std::setprecision(2);

  std::ifstream file (config.filename.c_str(), std::ifstream::binary);

  // Read the first burst, but do not process it, because we need at least two bursts
  // worth of data for reliable initial detection.
  file.read((char*)burst.begin(), config.sps * BURST_LEN_BYTES);
  {signalVector t = burst.segment(0, BURST_LEN_FULL); scaleVector(t, complex(SHRT_MAX)); }

#if 0
    /* Distort signal */
    {
      signalVector burst_read = burst.segment(85,156);
      std::ifstream file (config.filename.c_str(), std::ifstream::binary);
      file.read((char*)burst_read.begin(), burst_read.size() * 2 * sizeof(float));
      file.close();
    }
#endif
#if 1
  // Read more data and try burst detection until successful
  while(file.read((char*)(burst.begin()+config.sps*BURST_LEN_FULL), config.sps*BURST_LEN_BYTES))
  {
    {signalVector t = burst.segment(BURST_LEN_FULL, BURST_LEN_FULL); scaleVector(t, complex(SHRT_MAX)); }
    bool found = processBurst(config, burst, BURST_LEN_FULL, RSSI, timingOffset, berEstimator);
    std::cout << "RSSI: " << RSSI << " dBFS" << std::endl;
    if (found) {
      gsmTime.incTN();
      berBurstsAveraged++;
      break;
    }
    burst.segmentMove(config.sps*BURST_LEN_FULL, 0, config.sps*BURST_LEN_FULL);
  }

  // Align stream to burst
  int offsetInt = (int)timingOffset;
  burst.segmentMove(config.sps*(BURST_LEN_FULL+offsetInt), 0, config.sps*(BURST_LEN_FULL-offsetInt));
  {signalVector t = burst.segment(0, BURST_LEN_FULL-offsetInt); scaleVector(t, complex(1.0/SHRT_MAX)); }
  file.read((char*)(burst.begin()+config.sps*(BURST_LEN_FULL-offsetInt)), config.sps*offsetInt*SAMPLE_SIZE_BYTES);
#endif

  // Resize burst vector to hold only one burst, because demodulation code
  // always decode the full vector size.
  burst.shrink(BURST_LEN_FULL+1);

  // Process the rest of the stream
  do {
    {signalVector t = burst.segment(0, BURST_LEN_FULL); scaleVector(t, complex(SHRT_MAX)); }
    processBurst(config, burst, (config.type==RACH)?config.max_expected_delay_ab:config.max_expected_delay_ab,
                 RSSI, timingOffset, berEstimator);
    if (burst156_157 && !syncedTo157bits && timingOffset - timingOffsetPrev > .75) {
      std::cout << "TOA adjust: Found a 157-bit burst, reset TN to mark it" << std::endl;
      gsmTime.TN(2);
      timingOffset -= 1.0;
      // Make sure we do this adjustment only once.
      syncedTo157bits = true;
    } else {
      gsmTime.incTN();
    }
    bitsToSkip = 0;
    bitsReadExtra = 0;
    if (timingOffset < 0.0) {
      std::cout << "TOA adjust: skip a bit" << std::endl;
      burst[0] = 0;
      bitsToSkip = 1;
      bitsReadExtra--;
    }
    bitsReadExtra += (gsmTime.TN()%4 == 0);
    if (timingOffset > 1.1) {
      std::cout << "TOA adjust: add extra bit" << std::endl;
      bitsReadExtra++;
    }
    std::cout << "Clock: " << gsmTime;
    std::cout << " RSSI: " << RSSI << " dBFS";
    std::cout << " Error bits: " << berEstimator.errorBits() << " Total bits: " << berEstimator.totalBits()
              << " BER: " << 100.0*berEstimator.errorBits() / berEstimator.totalBits() << "%" << std::endl;
    berBurstsAveraged++;
    // Never reset if config.ber_burst_avg is 0
    if (config.ber_burst_avg > 0 && berBurstsAveraged >= config.ber_burst_avg) {
        berBurstsAveraged = 0;
        berEstimator.reset();
    }
    std::cout << "bitsReadExtra: " << bitsReadExtra << " bitsToSkip: " << bitsToSkip << std::endl;
    timingOffsetPrev = timingOffset;
  } while(file.read((char*)(burst.begin()+bitsToSkip), config.sps*(BURST_LEN_BYTES+SAMPLE_SIZE_BYTES*bitsReadExtra)));

  std::cout << "End of file reached" << std::endl;
  file.close();

  return 0;
}
