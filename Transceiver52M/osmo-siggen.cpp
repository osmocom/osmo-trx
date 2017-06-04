/*
 * GSM Signal Generator 
 *
 * Copyright (C) 2017 Ettus Research LLC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 *
 * Author: Tom Tsou <tom.tsou@ettus.com>
 */

#include <limits.h>
#include <unistd.h>
#include <getopt.h>
#include <algorithm>
#include <functional>
#include <memory>
#include <map>
#include <GSMCommon.h>
#include <Logger.h>
#include <Configuration.h>
#include <GSMCommon.h>
#include "sigProcLib.h"
#include "radioDevice.h"

extern "C" {
#include "convolve.h"
#include "convert.h"
}

ConfigurationTable gConfig;

#define DEFAULT_TX_SPS   4
#define DEFAULT_TX_AMPL  0.5
#define DEFAULT_TX_GAIN  50
#define DEFAULT_TX_FREQ  1e9
#define DEFAULT_OFFSET   0.0

using namespace std;

enum GsmModType {
  MOD_LAURENT4,
  MOD_LAURENT2,
  MOD_LAURENT1,
  MOD_NCO,
  NUM_MODS,
};

enum BurstType {
  BURST_NORMAL,
  BURST_ACCESS,
  BURST_FREQ,
  BURST_SYNC,
  BURST_EDGE,
  NUM_BURSTS,
};

enum BurstTSC {
  TSC0, TSC1, TSC2, TSC3, TSC4, TSC5, TSC6, TSC7,
};

struct Config {
  string args     = "";
  string logl     = "NOTICE";
  unsigned sps    = DEFAULT_TX_SPS;
  double offset   = DEFAULT_OFFSET;
  bool swap       = false;
  float ampl      = DEFAULT_TX_AMPL;
  double freq     = DEFAULT_TX_FREQ;
  double gain     = DEFAULT_TX_GAIN;
  BurstTSC tsc    = TSC0;
  GsmModType mod  = MOD_LAURENT2;
  BurstType burst = BURST_NORMAL;
  RadioDevice::ReferenceType ref = RadioDevice::REF_INTERNAL;
};

static shared_ptr<signalVector> modulateGMSK(BitVector &bits, GsmModType modType)
{
  switch (modType) {
  case MOD_LAURENT4: return shared_ptr<signalVector>(modulateBurstLaurent4(bits));
  case MOD_LAURENT2: return shared_ptr<signalVector>(modulateBurstLaurent2(bits));
  case MOD_LAURENT1: return shared_ptr<signalVector>(modulateBurstLaurent1(bits));
  case MOD_NCO:      return shared_ptr<signalVector>(modulateBurstNCO(bits));
  default:           return shared_ptr<signalVector>(modulateBurstLaurent2(bits));
  };
}

static shared_ptr<signalVector> generateNormalBurst(BurstTSC tsc, GsmModType modType)
{
  auto tail  = vector<char>(3, 0);
  auto data0 = vector<char>(57);
  auto data1 = vector<char>(57);
  auto steal = vector<char>(1, 0);
  auto train = vector<char>(26);

  auto ti = begin(GSM::gTrainingSequence[tsc]);
  for (auto &t : train) t = *ti++;
  for (auto &d : data0) d = rand() % 2;
  for (auto &d : data1) d = rand() % 2;

  auto bits = BitVector(NORMAL_BURST_NBITS);
  auto bi = bits.begin();

  for (auto t : tail)  *bi++ = t;
  for (auto d : data0) *bi++ = d;
  for (auto s : steal) *bi++ = s;
  for (auto t : train) *bi++ = t;
  for (auto s : steal) *bi++ = s;
  for (auto d : data1) *bi++ = d;
  for (auto t : tail)  *bi++ = t;

  return modulateGMSK(bits, modType);
}

static shared_ptr<signalVector> generateRABurst(GsmModType modType)
{
  auto tail0  = vector<char>(8, 0);
  auto train  = vector<char>(41);
  auto data   = vector<char>(36);
  auto tail1  = vector<char>(3, 0);

  auto ti = begin(GSM::gRACHBurst);
  for (auto &t : train) t = *ti++;
  for (auto &d : data)  d = rand() % 2;

  auto bits = BitVector(88);
  auto bi = bits.begin();

  for (auto t : tail0) *bi++ = t;
  for (auto t : train) *bi++ = t;
  for (auto d : data)  *bi++ = d;
  for (auto t : tail1) *bi++ = t;

  return modulateGMSK(bits, modType);
}

static shared_ptr<signalVector> generateFreqBurst(GsmModType modType)
{
  auto tail  = vector<char>(3, 0);
  auto fixed = vector<char>(142);

  auto bits = BitVector(148);
  auto bi = bits.begin();

  for (auto t : tail)  *bi++ = t;
  for (auto f : fixed) *bi++ = f;
  for (auto t : tail)  *bi++ = t;

  return modulateGMSK(bits, modType);
}

static shared_ptr<signalVector> generateSyncBurst(GsmModType modType)
{
  auto tail  = vector<char>(3, 0);
  auto data0 = vector<char>(39);
  auto data1 = vector<char>(39);

  /* 64 length synchronization sequence */
  vector<char> train {
    1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
    0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1,
    0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1,
  };
  for (auto &d : data0) d = rand() % 2;
  for (auto &d : data1) d = rand() % 2;

  auto bits = BitVector(148);
  auto bi = bits.begin();

  for (auto t : tail)  *bi++ = t;
  for (auto d : data0) *bi++ = d;
  for (auto t : train) *bi++ = t;
  for (auto d : data1) *bi++ = d;
  for (auto t : tail)  *bi++ = t;

  return modulateGMSK(bits, modType);
}

static shared_ptr<signalVector> generateEDGEBurst(BurstTSC tsc)
{
  auto tail =  vector<Complex<float>>(3);
  auto data0 = vector<Complex<float>>(58);
  auto train = vector<Complex<float>>(26);
  auto data1 = vector<Complex<float>>(58);

  extern const Complex<float> psk8_table[8];
  for (auto &t : tail)  t = psk8_table[0b111];
  for (auto &d : data0) d = psk8_table[rand() % 8];
  for (auto &d : data1) d = psk8_table[rand() % 8];

  auto ti = begin(GSM::gEdgeTrainingSequence[tsc]);
  for (auto &t : train) {
    unsigned i = (*(ti + 0) & 0b001) << 0 |
                 (*(ti + 1) & 0b001) << 1 |
                 (*(ti + 2) & 0b001) << 2;
    t = psk8_table[i];
    ti += 3;
  }

  /* NBITS refers to 148 symbols in this case */
  auto burst = signalVector(NORMAL_BURST_NBITS);
  auto bi = burst.begin();

  for (auto t : tail)  *bi++ = t;
  for (auto d : data0) *bi++ = d;
  for (auto t : train) *bi++ = t;
  for (auto d : data1) *bi++ = d;
  for (auto t : tail)  *bi++ = t;

  return shared_ptr<signalVector>(shapeEdgeBurst(burst));
}

/* Perform float-integer conversion and write to the device */
static void sendBurst(shared_ptr<RadioDevice> usrp, TIMESTAMP &ts,
                      shared_ptr<signalVector> sv, float ampl)
{
  auto buffer = vector<Complex<short>>(sv->size());

  transform(sv->begin(), sv->end(), buffer.begin(), [ampl](Complex<float> x) {
    const float scale = SHRT_MAX * ampl;
    return Complex<short>(x.real()*scale, x.imag()*scale);
  });

  auto buffers = vector<short *>(1, reinterpret_cast<short *>(&buffer.front()));
  ts += usrp->writeSamples(buffers, buffer.size(), nullptr, ts, true);
}

static void print_help()
{
  fprintf(stdout, "Options:\n"
    "  -h, --help     This text\n"
    "  -a, --args     UHD device args\n"
    "  -l  --log      Logging level (%s)\n"
    "  -b, --burst    Burst type (%s)\n"
    "  -r, --ref      Frequency reference (%s)\n"
    "  -f, --freq     Tx RF frequency\n"
    "  -g, --gain     Tx RF gain\n"
    "  -s, --sps      Tx samples-per-symbol (only 4 supported)\n"
    "  -m, --mod      GSMK modulator type (%s)\n"
    "  -p, --ampl     Tx amplitude (0.0 - 1.0)\n"
    "  -o, --offset   Baseband frequency offset\n"
    "  -t, --tsc      Normal and EDGE burst training sequence (0-7)\n"
    "  -S, --swap     Swap channels\n\n",
    "'err', 'warn', 'notice', 'info', 'debug'",
    "'normal', 'access', 'freq', 'sync', 'edge'",
    "'internal', 'external', 'gps'",
    "'laurent4', 'laurent2', 'laurent1', 'nco'"
  );
}

static void print_config(Config &config)
{
  const map<GsmModType, string> modMap = {
    { MOD_LAURENT4, "Laurent-4" },
    { MOD_LAURENT2, "Laurent-2" },
    { MOD_LAURENT1, "Laurent-1" },
    { MOD_NCO,      "NCO"       },
  };

  const map<BurstType, string> burstMap = {
    { BURST_NORMAL, "Normal"          },
    { BURST_ACCESS, "Access"          },
    { BURST_FREQ,   "Frequency"       },
    { BURST_SYNC,   "Synchronization" },
    { BURST_EDGE,   "EDGE"            },
  };

  const map<RadioDevice::ReferenceType, string> refMap = {
    { RadioDevice::REF_INTERNAL, "Internal" },
    { RadioDevice::REF_EXTERNAL, "External" },
    { RadioDevice::REF_GPS,      "GPS"      },
  };

  auto yesno = [](bool x) { return x ? "yes" : "no"; };

  ostringstream ost("");
  ost << "Config Settings" << endl;
  ost << "   Log level............... " << config.logl << std::endl;
  ost << "   Device args............. " << "\"" << config.args << "\"" << endl;
  ost << "   Samples-per-Symbol...... " << config.sps << endl;
  ost << "   RF frequency............ " << config.freq/1e9 << " GHz" << endl;
  ost << "   RF gain................. " << config.gain << " dB" << endl;
  ost << "   Reference............... " << refMap.at(config.ref) << endl;
  ost << "   Burst type.............. " << burstMap.at(config.burst) << endl;
  ost << "   Modulator type.......... " << modMap.at(config.mod) << endl;
  ost << "   Baseband offset......... " << config.offset/1e6 << " MHz" << endl;
  ost << "   Swap channels........... " << yesno(config.swap) << endl;
  cout << ost << endl;
}

static bool handle_options(int argc, char **argv, Config &config)
{
  int option;

  const struct option longopts[] = {
    { "help",     0, nullptr, 'h' },
    { "log",      1, nullptr, 'l' },
    { "args",     1, nullptr, 'a' },
    { "ref" ,     1, nullptr, 'r' },
    { "freq",     1, nullptr, 'f' },
    { "gain",     1, nullptr, 'g' },
    { "mod",      1, nullptr, 'm' },
    { "offset",   1, nullptr, 'o' },
    { "sps",      1, nullptr, 's' },
    { "ampl",     1, nullptr, 'p' },
    { "tsc",      1, nullptr, 'r' },
    { "burst",    1, nullptr, 'b' },
    { "swap",     1, nullptr, 'w' },
  };

  const map<string, string> logMap = {
    { "emerg",  "EMERG"   },
    { "EMERG",  "EMERG"   },
    { "alert",  "ALERT"   },
    { "ALERT",  "ALERT"   },
    { "err",    "ERR"     },
    { "ERR",    "ERR"     },
    { "warn",   "WARNING" },
    { "WARN",   "WARNING" },
    { "notice", "NOTICE"  },
    { "NOTICE", "NOTICE"  },
    { "info",   "INFO"    },
    { "INFO",   "INFO"    },
    { "debug",  "DEBUG"   },
    { "DEBUG",  "DEBUG"   },
  };

  const map<string, GsmModType> modMap = {
    { "laurent4", MOD_LAURENT4 },
    { "laurent2", MOD_LAURENT2 },
    { "laurent1", MOD_LAURENT1 },
    { "nco",      MOD_NCO      },
  };

  const map<string, BurstType> burstMap = {
    { "normal",   BURST_NORMAL },
    { "access",   BURST_ACCESS },
    { "freq",     BURST_FREQ   },
    { "sync",     BURST_SYNC   },
    { "edge",     BURST_EDGE   },
  };

  const map<string, RadioDevice::ReferenceType> refMap = {
    { "internal", RadioDevice::REF_INTERNAL },
    { "external", RadioDevice::REF_EXTERNAL },
    { "gpsdo",    RadioDevice::REF_GPS      },
    { "gps",      RadioDevice::REF_GPS      },
  };

  while ((option = getopt_long(argc, argv, "ha:l:r:f:g:m:o:s:p:t:b:w", longopts, nullptr)) != -1) {
    switch (option) {
    case 'a':
      config.args = optarg;
      break;
    case 'f':
      config.freq = atof(optarg);
      break;
    case 'g':
      config.gain = atof(optarg);
      break;
    case 'o':
      config.offset = atof(optarg);
      break;
    case 's':
      if (atoi(optarg) != 4) {
        printf("Unsupported SPS = %i\n", atoi(optarg));
        return false;
      }
      break;
    case 'p':
      config.ampl = atof(optarg);
      break;
    case 't':
      if (atoi(optarg) < TSC0 || atoi(optarg) > TSC7) {
        printf("Invalid training sequence %i", atoi(optarg));
        return false;
      }
      config.tsc = static_cast<BurstTSC>(atoi(optarg));
      break;
    case 'w':
      config.swap = true;
      break;
    case 'l':
      if (logMap.count(optarg) > 0) {
        config.logl = logMap.at(optarg);
      } else {
        printf("Invalid log parameter '%s'\n\n", optarg);
        return false;
      }
      break;
    case 'r':
      if (refMap.count(optarg) > 0) {
        config.ref = refMap.at(optarg);
      } else {
        printf("Invalid reference parameter '%s'\n\n", optarg);
        return false;
      }
      break;
    case 'm':
      if (modMap.count(optarg) > 0) {
        config.mod = modMap.at(optarg);
      } else {
        printf("Invalid modulation parameter '%s'\n\n", optarg);
        return false;
      }
      break;
    case 'b':
      if (burstMap.count(optarg) > 0) {
        config.burst = burstMap.at(optarg);
      } else {
        printf("Invalid burst type parameter '%s'\n\n", optarg);
        return false;
      }
      break;
    case 'h':
    default:
      return false;
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  Config config;
  if (!handle_options(argc, argv, config)) {
    print_help();
    return -EINVAL;
  }

  print_config(config);

  gLogInit("osmo-siggen", config.logl.c_str(), LOG_LOCAL7);

  convolve_init();
  convert_init();
  sigProcLibSetup();

  /* Device setup */
  shared_ptr<RadioDevice> usrp(RadioDevice::make(config.sps, config.sps, RadioDevice::NORMAL, 1, config.offset));
  usrp->open(config.args, config.ref, config.swap);
  usrp->setTxFreq(config.freq);
  usrp->setTxGain(config.gain);
  usrp->start(true);
  usrp->setPriority(0.5);

  /* Bind all burst-modulator configurations */
  auto makeBurstGenerator = [&config]()->function<shared_ptr<signalVector>()> {
    switch (config.burst) {
    case BURST_EDGE:   return bind(generateEDGEBurst, config.tsc);
    case BURST_ACCESS: return bind(generateRABurst, config.mod);
    case BURST_FREQ:   return bind(generateFreqBurst, config.mod);
    case BURST_SYNC:   return bind(generateSyncBurst, config.mod);
    case BURST_NORMAL:
    default:           return bind(generateNormalBurst, config.tsc, config.mod);
    }
  };

  auto burstGenerator = makeBurstGenerator();
  auto ts = usrp->initialWriteTimestamp();
  auto frameTrigger = []() {
    static int tn = 0;
    return ++tn % 8 == 0;
  };

  while (1) {
    try {
      if (frameTrigger()) usrp->triggerGPIO(ts);
      sendBurst(usrp, ts, burstGenerator(), config.ampl);
    } catch (const exception &e) {
      cout << e.what() << endl;
      break;
    }
  }

  sigProcLibDestroy();
}
