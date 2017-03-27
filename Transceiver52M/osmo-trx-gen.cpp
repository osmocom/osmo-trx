/*
 * Copyright (C) 2017 Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
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
#include <endian.h> // for byte order manipulation

#include "Logger.h"
#include "sigProcLib.h"
#include "GSMCommon.h"
#include "BitVector.h"
#include "Configuration.h"

extern "C" {
#include "convolve.h"
#include "convert.h"
}

#define DEFAULT_SPS           4
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

enum FileType {
  FLOAT_NORM_LE,  ///< Float -1..+1 Little Endian
  FLOAT16_LE,     ///< Float -32767..+32767 Little Endian
  SIGNED16_LE,    ///< Integer -32767..+32767 Little Endian
  SIGNED16_BE,    ///< Integer -32767..+32767 Big Endian (Keysight waveform format)
};

struct trx_config {
  std::string log_level;
  unsigned sps;
  unsigned tsc;
  double full_scale;
  bool edge;
  CorrType type;
  std::string filename;
  FileType file_type;
};

std::ostream& operator<<(std::ostream& os, FileType ftype)
{
  switch(ftype)
  {
  case FLOAT_NORM_LE:
    os << "float";
    break;
  case FLOAT16_LE:
    os << "float16";
    break;
  case SIGNED16_LE:
    os << "signed16";
    break;
  case SIGNED16_BE:
    os << "signed16be";
    break;
  default:
    assert(!"unknown file type");
  }
	return os;
}


void writeBurstFloatNorm(std::ofstream& os, const signalVector& v)
{
  os.write((char*)v.begin(), v.size() * 2 * sizeof(float));
}

void writeBurstFloat16LE(std::ofstream& os, const signalVector& v)
{
  const complex *c = v.begin();
  for (size_t i=0; i<v.size(); i++, c++) {
    float iq[2];
    iq[0] = c->real()*SHRT_MAX;
    iq[1] = c->imag()*SHRT_MAX;
    os.write((char*)&iq, 2*sizeof(float));
  }
}

void writeBurstSigned16LE(std::ofstream& os, const signalVector& v)
{
  const complex *c = v.begin();
  for (size_t i=0; i<v.size(); i++, c++) {
    int16_t iq[2];
    iq[0] = c->real()*SHRT_MAX;
    iq[1] = c->imag()*SHRT_MAX;
    iq[0] = htole16(iq[0]);
    iq[1] = htole16(iq[1]);
    os.write((char*)&iq, 2*sizeof(int16_t));
  }
}

void writeBurstSigned16BE(std::ofstream& os, const signalVector& v)
{
  const complex *c = v.begin();
  for (size_t i=0; i<v.size(); i++, c++) {
    int16_t iq[2];
    iq[0] = c->real()*SHRT_MAX;
    iq[1] = c->imag()*SHRT_MAX;
    iq[0] = htobe16(iq[0]);
    iq[1] = htobe16(iq[1]);
    os.write((char*)&iq, 2*sizeof(int16_t));
  }
}

void writeBurst(std::ofstream& os, const signalVector& v, FileType ftype)
{
  switch(ftype)
  {
  case FLOAT_NORM_LE:
    writeBurstFloatNorm(os, v);
    break;
  case FLOAT16_LE:
    writeBurstFloat16LE(os, v);
    break;
  case SIGNED16_LE:
    writeBurstSigned16LE(os, v);
    break;
  case SIGNED16_BE:
    writeBurstSigned16BE(os, v);
    break;
  default:
    assert(!"unknown file type");
  }
}

// Setup configuration values
static void print_config(struct trx_config *config)
{
  std::ostringstream ost("");
  ost << "Config Settings" << std::endl;
  ost << "   Destination file name........ " << config->filename << std::endl;
  ost << "   Destination file type........ " << config->file_type << std::endl;
  ost << "   Log Level.................... " << config->log_level << std::endl;
  ost << "   Tx Samples-per-Symbol........ " << config->sps << std::endl;
  ost << "   EDGE support................. " << (config->edge ? "Enabled" : "Disabled") << std::endl;
  ost << "   Burst type................... " << config->type << std::endl;
  ost << "   Burst TSC.................... " << config->tsc << std::endl;
  ost << "   Signal full scale............ " << config->full_scale << std::endl;
  std::cout << ost << std::endl;
}

static void print_help()
{
  fprintf(stdout,
          "This utility generates waveform files aka IQ binary files in a number of formats"
          "to use them as input to osmo-trx-dec or load them into signal generators.\n"
          "\n"
          "Options:\n"
          "  -h          This text\n"
          "  -l LEVEL    Logging level (%s)\n"
          "  -e          Enable EDGE receiver\n"
          "  -s SPS      Samples-per-symbol (1 or 4, default: %d)\n"
          "  -t TSC      Burst training sequence (0 to 7, default: 0)\n"
          "  -f FILE     File to write generated bursts to\n"
          "  -F FILETYPE Format of the file - float, float16, signed16, signed16be (default: f16)\n"
          "              Note: Keysight waveform format is signed16be. osmo-trx-dec accepts float16.\n",
          "EMERG, ALERT, CRT, ERR, WARNING, NOTICE, INFO, DEBUG",
          DEFAULT_SPS);
}

FileType option_to_file_type(const std::string &optarg)
{
  if (optarg == "float") {
    return FLOAT_NORM_LE;
  } else if (optarg == "float16") {
    return FLOAT16_LE;
  } else if (optarg == "signed16") {
    return SIGNED16_LE;
  } else if (optarg == "signed16be") {
    return SIGNED16_BE;
  } else {
    return (FileType)-1;
  }
}

static bool handle_options(int argc, char **argv, struct trx_config *config)
{
  int option;

  config->log_level = "NOTICE";
  config->sps = DEFAULT_SPS;
  config->tsc = 0;
  config->full_scale = SHRT_MAX;
  config->edge = false;
  config->type = TSC;
  config->file_type = FLOAT16_LE;

  while ((option = getopt(argc, argv, "ls:et:f:F:h")) != -1) {
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
    case 'F':
      config->file_type = option_to_file_type(optarg);
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
    printf("ERROR: No output file name specified\n\n");
    return false;
  }

  if (config->file_type < 0) {
    printf("ERROR: Wrong output file format\n\n");
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

  signalVector burst(2*BURST_LEN_FULL);
  GSM::Time gsmTime;
  PRBS9 prbs;

  // Configure output stream
  std::cout << std::fixed;
  std::cout << std::setprecision(2);

  std::ofstream file (config.filename.c_str(), std::ifstream::binary);

  for (int i=0; i<511; i++) {
    signalVector *signal = genRandNormalBurst(config.tsc, config.sps, gsmTime.TN(), prbs);
    writeBurst(file, *signal, config.file_type);
    gsmTime.incTN();
  }

  file.close();

  std::cout << "Done!" << std::endl;

  return 0;
}
