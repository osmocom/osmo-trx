#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "Threads.h"
#include "XTRXDevice.h"

#include <Logger.h>
#include <errno.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

using namespace std;

const double defaultRXBandwidth = 2e6;
const double defaultTXBandwidth = 3e6;

static int time_tx_corr = 60; //20+20+20+20+20;

XTRXDevice::XTRXDevice(size_t txsps, size_t rxsps)
{
  LOG(INFO) << "creating XTRX device...";

  this->txsps = txsps;
  this->rxsps = rxsps;

  rxGain = 0;

  loopback = false;
  device = NULL;
}

static int parse_config(const char* line, const char* argument, int default_value)
{
    const char* arg_found = strstr(line, argument);
    if (!arg_found)
        return default_value;

    const char* qe_pos = strchr(arg_found, '=');
    if (!qe_pos)
        return default_value;

    int res = strtol(qe_pos + 1, NULL, 10);
    if (res == 0 && errno) {
        return default_value;
    }

    return res;
}

int XTRXDevice::open(const std::string &args, int ref, bool swap_channels)
{
  LOG(INFO) << "opening XTRX device '"  << args << "'..";

  int loglevel = parse_config(args.c_str(), "loglevel", 3);
  int lb_param = parse_config(args.c_str(), "loopback", 0);
  time_tx_corr = parse_config(args.c_str(), "tcorr", time_tx_corr);
  int fref     = parse_config(args.c_str(), "refclk", 30720000);
  int rxdec    = parse_config(args.c_str(), "rxdec", 0);

  char xtrx_name[500];
  const char* lend = strchr(args.c_str(), ',');
  int len = (lend) ? (lend - args.c_str()) : sizeof(xtrx_name) - 1;
  strncpy(xtrx_name, args.c_str(), len);
  xtrx_name[len] = 0;

  if (lb_param) {
      LOG(ALERT) << "XTRX LOOPBACK mode is set!";
      loopback = true;
  }

  int res = xtrx_open(xtrx_name, loglevel, &device);
  if (res) {
    LOG(ALERT) << "XTRX creating failed, device " << xtrx_name << " code " << res;
    return -1;
  }
  double actualMasterClock = 0;

  if (fref > 0) {
	xtrx_set_ref_clk(device, fref, XTRX_CLKSRC_INT);
  }

  res = xtrx_set_samplerate(device,
							GSMRATE * (double) std::min(txsps, rxsps)  * 32 * 4 * ((rxdec) ? 2 : 1),
							GSMRATE * (double) rxsps,
							GSMRATE * (double) txsps,
							(rxdec) ? XTRX_SAMPLERATE_FORCE_RX_DECIM : 0,
                            &actualMasterClock,
                            &actualRXSampleRate,
                            &actualTXSampleRate);
  if (res) {
	  LOG(ALERT) << "XTRX failed to set samplerate RX: " << GSMRATE * (double) rxsps
				 << " TX: " << GSMRATE * (double) txsps
                 << " res: " << res;
      return -1;
  } else {
      LOG(INFO) << "XTRX set samplerate Master: " << actualMasterClock
                << " RX: " << actualRXSampleRate
                << " TX: " << actualTXSampleRate;
  }


  int i;
  double bw;
  double actualbw;

  actualbw = 0;
  bw = defaultRXBandwidth;
  for (i = 0, res = -1; res && (i < 4); i++, bw *= 1.5) {
      res = xtrx_tune_rx_bandwidth(device, XTRX_CH_AB, bw, &actualbw);
  }
  if (res) {
      LOG(ALERT) << "XTRX failed to set RX bandwidth: " << bw
                 << " res: " << res;
      return -1;
  } else {
      LOG(INFO) << "XTRX set RX bandwidth: " << actualbw;
  }

  actualbw = 0;
  bw = defaultTXBandwidth;
  for (i = 0, res = -1; res && (i < 4); i++, bw *= 1.1) {
      res = xtrx_tune_tx_bandwidth(device, XTRX_CH_AB, bw, &actualbw);
  }
  if (res) {
      LOG(ALERT) << "XTRX failed to set TX bandwidth: " << bw
                 << " res: " << res;
      return -1;
  } else {
      LOG(INFO) << "XTRX set TX bandwidth: " << actualbw;
  }

  samplesRead = 0;
  samplesWritten = 0;
  started = false;

  return NORMAL;
}

XTRXDevice::~XTRXDevice()
{
    if (device) {
        xtrx_close(device);
    }
}

bool XTRXDevice::start() 
{
  LOG(INFO) << "starting XTRX...";
  if (started) {
      return false;
  }

  dataStart = 0;
  dataEnd = 0;
  timeStart = 0;
  timeEnd = 0;
  timeRx = initialReadTimestamp();
  timestampOffset = 0;
  latestWriteTimestamp = 0;
  lastPktTimestamp = 0;
  hi32Timestamp = 0;
  isAligned = false;

  //xtrx_stop(device, XTRX_TX);
  //xtrx_stop(device, XTRX_RX);

  xtrx_set_antenna(device, XTRX_TX_L);
  xtrx_set_antenna(device, XTRX_RX_L);

  xtrx_run_params_t params;
  params.dir = XTRX_TRX;
  params.nflags = (loopback) ? XTRX_RUN_DIGLOOPBACK : 0;

  params.rx.chs = XTRX_CH_AB;
  params.rx.flags = XTRX_RSP_SISO_MODE;
  params.rx.hfmt = XTRX_IQ_INT16;
  params.rx.wfmt = XTRX_WF_16;
  params.rx.paketsize = 625 * rxsps;

  params.tx.chs = XTRX_CH_AB;
  params.tx.flags = XTRX_RSP_SISO_MODE;
  params.tx.hfmt = XTRX_IQ_INT16;
  params.tx.wfmt = XTRX_WF_16;
  params.tx.paketsize = 625 * txsps;

  if (loopback) {
	params.tx.flags |= XTRX_RSP_SWAP_AB | XTRX_RSP_SWAP_IQ;
  }

  params.tx_repeat_buf = NULL;
  params.rx_stream_start = initialReadTimestamp();

  int res = xtrx_run_ex(device, &params);
  if (res) {
	  LOG(ALERT) << "XTRX start failed res: " << res;
  } else {
	  LOG(INFO) << "XTRX started";
	  started = true;
  }
  return started;
}

bool XTRXDevice::stop() 
{
  if (started) {
      int res = xtrx_stop(device, XTRX_TRX);
      if (res) {
          LOG(ALERT) << "XTRX stop failed res: " << res;
      } else {
          LOG(INFO) << "XTRX stopped";
          started = false;
      }
  }
  return !started;
}

TIMESTAMP XTRXDevice::initialWriteTimestamp()
{
	if (/*(iface == MULTI_ARFCN) || */(rxsps == txsps))
		return initialReadTimestamp();
	else
		return initialReadTimestamp() * txsps;
}

double XTRXDevice::maxTxGain()
{
  return 30;
}

double XTRXDevice::minTxGain()
{
  return 0;
}

double XTRXDevice::maxRxGain()
{
  return 30;
} 

double XTRXDevice::minRxGain()
{
  return 0;
}

double XTRXDevice::setTxGain(double dB, size_t chan)
{
  if (chan) {
    LOG(ALERT) << "Invalid channel " << chan;
    return 0.0;
  }
  double actual = 0;
  LOG(NOTICE) << "Setting TX gain to " << dB << " dB.";

  int res = xtrx_set_gain(device, XTRX_CH_AB, XTRX_TX_PAD_GAIN, -10, &actual);
  if (res) {
    LOG(ERR) << "Error setting TX gain res: " << res;
  }

  return actual;
}


double XTRXDevice::setRxGain(double dB, size_t chan)
{
    if (chan) {
      LOG(ALERT) << "Invalid channel " << chan;
      return 0.0;
    }
    double actual = 0;
    LOG(NOTICE) << "Setting RX gain to " << dB << " dB.";

	int res = xtrx_set_gain(device, XTRX_CH_AB, XTRX_RX_LNA_GAIN, 25, &actual);
    if (res) {
      LOG(ERR) << "Error setting RX gain res: " << res;
    }

    return actual;
}

// NOTE: Assumes sequential reads
int XTRXDevice::readSamples(std::vector<short *> &bufs, int len, bool *overrun,
							TIMESTAMP timestamp, bool *underrun, unsigned *RSSI)
{
	if (!started)
		return -1;

	if (RSSI) {
		*RSSI = 10; // TODO
	}

	struct xtrx_recv_ex_info ri;
	ri.samples = len;
	ri.buffer_count = bufs.size();
	ri.buffers = (void* const*)&bufs[0];
	ri.flags = 0;

	int res = xtrx_recv_sync_ex(device, &ri);
	if (res) {
		LOG(ALERT) << "xtrx_recv_sync failed res " << res << " current TS " << timeRx << " req TS" << timestamp;
		return -1;
	}
	timeRx += len;

	// TODO: remove this
	int i;
	for (i = 0; i < len * 2; i++)
			bufs[0][i] <<= 4;

	if (underrun) {
		*underrun = (ri.out_events & RCVEX_EVENT_FILLED_ZERO);
	}
	return len;

}

int XTRXDevice::writeSamples(std::vector<short *> &bufs, int len,
                             bool *underrun, unsigned long long timestamp,
                             bool isControl)
{
	if (!started)
	  return 0;

	xtrx_send_ex_info_t nfo;
	nfo.buffers = (const void* const*)&bufs[0];
	nfo.buffer_count = bufs.size();
	nfo.flags = XTRX_TX_DONT_BUFFER;
	nfo.samples = len;
	nfo.ts = timestamp - time_tx_corr;

	int res = xtrx_send_sync_ex(device, &nfo);
	if (res != 0) {
		LOG(ALERT) << "xtrx_send_sync_ex returned " << res << " len=" << len << " ts=" << timestamp;
		return 0;
	}

	if (*underrun) {
		*underrun = (nfo.out_flags & XTRX_TX_DISCARDED_TO);
	}

	return len;
}

bool XTRXDevice::updateAlignment(TIMESTAMP timestamp) 
{
	LOG(ALERT) << "Update Aligment " << timestamp;
  return true;
}

bool XTRXDevice::setTxFreq(double wFreq, size_t chan)
{
    int res;
    double actual = 0;

    if (chan) {
      LOG(ALERT) << "Invalid channel " << chan;
      return false;
    }

    if ((res = xtrx_tune(device, XTRX_TUNE_TX_FDD, wFreq, &actual)) == 0) {
      LOG(INFO) << "set RX: " << wFreq << std::endl
                << "    actual freq: " << actual << std::endl;
      return true;
    }
    else {
      LOG(ALERT) << "set RX: " << wFreq << "failed (code: " << res << ")" << std::endl;
      return false;
    }
}

bool XTRXDevice::setRxFreq(double wFreq, size_t chan)
{
  int res;
  double actual = 0;

  if (chan) {
    LOG(ALERT) << "Invalid channel " << chan;
    return false;
  }

  if ((res = xtrx_tune(device, XTRX_TUNE_RX_FDD, wFreq, &actual)) == 0) {
    LOG(INFO) << "set RX: " << wFreq << std::endl
              << "    actual freq: " << actual << std::endl;
    return true;
  }
  else {
    LOG(ALERT) << "set RX: " << wFreq << "failed (code: " << res << ")" << std::endl;
    return false;
  }
}

RadioDevice *RadioDevice::make(size_t tx_sps, size_t rx_sps, InterfaceType type,
                               size_t chans, double offset)
{
    return new XTRXDevice(tx_sps, rx_sps);
}
