/*
 * Copyright (C) 2013 Thomas Tsou <tom@tsou.cc>
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

#include "Transceiver.h"
#include "radioDevice.h"

#include <time.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

#include <GSMCommon.h>
#include <Logger.h>

extern "C" {
#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/stats.h>
#include <osmocom/vty/logging.h>
#include <osmocom/vty/ports.h>
#include <osmocom/vty/misc.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/ctrl/control_vty.h>
#include <osmocom/ctrl/ports.h>
#include <osmocom/ctrl/control_if.h>
#include <osmocom/vty/stats.h>
#include "convolve.h"
#include "convert.h"
#include "trx_vty.h"
#include "debug.h"
}

/* Samples-per-symbol for downlink path
 *     4 - Uses precision modulator (more computation, less distortion)
 *     1 - Uses minimized modulator (less computation, more distortion)
 *
 *     Other values are invalid. Receive path (uplink) is always
 *     downsampled to 1 sps. Default to 4 sps for all cases.
 */
#define DEFAULT_TX_SPS		4

/*
 * Samples-per-symbol for uplink (receiver) path
 *     Do not modify this value. EDGE configures 4 sps automatically on
 *     B200/B210 devices only. Use of 4 sps on the receive path for other
 *     configurations is not supported.
 */
#define DEFAULT_RX_SPS		1

/* Default configuration parameters */
#define DEFAULT_TRX_PORT	5700
#define DEFAULT_TRX_IP		"127.0.0.1"
#define DEFAULT_CHANS		1
#define DEFAULT_CONFIG_FILE	"osmo-trx.cfg"

struct trx_config {
	std::string local_addr;
	std::string remote_addr;
	std::string dev_args;
	char* config_file;
	unsigned port;
	unsigned tx_sps;
	unsigned rx_sps;
	unsigned chans;
	unsigned rtsc;
	unsigned rach_delay;
	bool extref;
	bool gpsref;
	Transceiver::FillerType filler;
	bool mcbts;
	double offset;
	double rssi_offset;
	bool swap_channels;
	bool edge;
	int sched_rr;
	std::vector<std::string> rx_paths;
	std::vector<std::string> tx_paths;
};

volatile bool gshutdown = false;

static void *tall_trx_ctx;
static struct trx_ctx *g_trx_ctx;
static struct ctrl_handle *g_ctrlh;

static RadioDevice *usrp;
static RadioInterface *radio;
static Transceiver *transceiver;

/* Setup configuration values
 *     Don't query the existence of the Log.Level because it's a
 *     mandatory value. That is, if it doesn't exist, the configuration
 *     table will crash or will have already crashed. Everything else we
 *     can survive without and use default values if the database entries
 *     are empty.
 */
bool trx_setup_config(struct trx_config *config)
{
	std::string refstr, fillstr, divstr, mcstr, edgestr;
	std::vector<std::string>::const_iterator si;

	if (config->mcbts && config->chans > 5) {
		std::cout << "Unsupported number of channels" << std::endl;
		return false;
	}

	edgestr = config->edge ? "Enabled" : "Disabled";
	mcstr = config->mcbts ? "Enabled" : "Disabled";

	if (config->extref)
		refstr = "External";
	else if (config->gpsref)
		refstr = "GPS";
	else
		refstr = "Internal";

	switch (config->filler) {
	case Transceiver::FILLER_DUMMY:
		fillstr = "Dummy bursts";
		break;
	case Transceiver::FILLER_ZERO:
		fillstr = "Disabled";
		break;
	case Transceiver::FILLER_NORM_RAND:
		fillstr = "Normal busrts with random payload";
		break;
	case Transceiver::FILLER_EDGE_RAND:
		fillstr = "EDGE busrts with random payload";
		break;
	case Transceiver::FILLER_ACCESS_RAND:
		fillstr = "Access busrts with random payload";
		break;
	}

	std::ostringstream ost("");
	ost << "Config Settings" << std::endl;
	ost << "   Device args............. " << config->dev_args << std::endl;
	ost << "   TRX Base Port........... " << config->port << std::endl;
	ost << "   TRX Address............. " << config->local_addr << std::endl;
	ost << "   GSM Core Address........." << config->remote_addr << std::endl;
	ost << "   Channels................ " << config->chans << std::endl;
	ost << "   Tx Samples-per-Symbol... " << config->tx_sps << std::endl;
	ost << "   Rx Samples-per-Symbol... " << config->rx_sps << std::endl;
	ost << "   EDGE support............ " << edgestr << std::endl;
	ost << "   Reference............... " << refstr << std::endl;
	ost << "   C0 Filler Table......... " << fillstr << std::endl;
	ost << "   Multi-Carrier........... " << mcstr << std::endl;
	ost << "   Tuning offset........... " << config->offset << std::endl;
	ost << "   RSSI to dBm offset...... " << config->rssi_offset << std::endl;
	ost << "   Swap channels........... " << config->swap_channels << std::endl;
	ost << "   Tx Antennas.............";
		for (si = config->tx_paths.begin(); si != config->tx_paths.end(); ++si)
			ost << " '" << ((*si != "") ? *si : "<default>") <<  "'";
		ost << std::endl;
	ost << "   Rx Antennas.............";
		for (si = config->rx_paths.begin(); si != config->rx_paths.end(); ++si)
			ost << " '" << ((*si != "") ? *si : "<default>") <<  "'";
		ost << std::endl;

	std::cout << ost << std::endl;
	return true;
}

/* Create radio interface
 *     The interface consists of sample rate changes, frequency shifts,
 *     channel multiplexing, and other conversions. The transceiver core
 *     accepts input vectors sampled at multiples of the GSM symbol rate.
 *     The radio interface connects the main transceiver with the device
 *     object, which may be operating some other rate.
 */
RadioInterface *makeRadioInterface(struct trx_config *config,
                                   RadioDevice *usrp, int type)
{
	RadioInterface *radio = NULL;

	switch (type) {
	case RadioDevice::NORMAL:
		radio = new RadioInterface(usrp, config->tx_sps,
					   config->rx_sps, config->chans);
		break;
	case RadioDevice::RESAMP_64M:
	case RadioDevice::RESAMP_100M:
		radio = new RadioInterfaceResamp(usrp, config->tx_sps,
						 config->rx_sps);
		break;
	case RadioDevice::MULTI_ARFCN:
		radio = new RadioInterfaceMulti(usrp, config->tx_sps,
						config->rx_sps, config->chans);
		break;
	default:
		LOG(ALERT) << "Unsupported radio interface configuration";
		return NULL;
	}

	if (!radio->init(type)) {
		LOG(ALERT) << "Failed to initialize radio interface";
		return NULL;
	}

	return radio;
}

/* Create transceiver core
 *     The multi-threaded modem core operates at multiples of the GSM rate of
 *     270.8333 ksps and consists of GSM specific modulation, demodulation,
 *     and decoding schemes. Also included are the socket interfaces for
 *     connecting to the upper layer stack.
 */
int makeTransceiver(struct trx_config *config, RadioInterface *radio)
{
	VectorFIFO *fifo;

	transceiver = new Transceiver(config->port, config->local_addr.c_str(),
			      config->remote_addr.c_str(), config->tx_sps,
			      config->rx_sps, config->chans, GSM::Time(3,0),
			      radio, config->rssi_offset);
	if (!transceiver->init(config->filler, config->rtsc,
		       config->rach_delay, config->edge)) {
		LOG(ALERT) << "Failed to initialize transceiver";
		return -1;
	}

	for (size_t i = 0; i < config->chans; i++) {
		fifo = radio->receiveFIFO(i);
		if (fifo && transceiver->receiveFIFO(fifo, i))
			continue;

		LOG(ALERT) << "Could not attach FIFO to channel " << i;
		return -1;
	}

	return 0;
}

static void sig_handler(int signo)
{
	fprintf(stdout, "signal %d received\n", signo);
	switch (signo) {
	case SIGINT:
	case SIGTERM:
		fprintf(stdout, "shutting down\n");
		gshutdown = true;
		break;
	case SIGABRT:
	case SIGUSR1:
		talloc_report(tall_trx_ctx, stderr);
		talloc_report_full(tall_trx_ctx, stderr);
		break;
	case SIGUSR2:
		talloc_report_full(tall_trx_ctx, stderr);
		break;
	default:
		break;
	}
}

static void setup_signal_handlers()
{
	/* Handle keyboard interrupt SIGINT */
	signal(SIGINT, &sig_handler);
	signal(SIGTERM, &sig_handler);
	signal(SIGABRT, &sig_handler);
	signal(SIGUSR1, &sig_handler);
	signal(SIGUSR2, &sig_handler);
	osmo_init_ignore_signals();
}


static std::vector<std::string> comma_delimited_to_vector(char* opt) {
	std::string str = std::string(opt);
	std::vector<std::string> result;
	std::stringstream ss(str);

	while( ss.good() )
	{
	    std::string substr;
	    getline(ss, substr, ',');
	    result.push_back(substr);
	}
	return result;
}

static void print_help()
{
	fprintf(stdout, "Options:\n"
		"  -h    This text\n"
		"  -a    UHD device args\n"
		"  -i    IP address of GSM core\n"
		"  -j    IP address of osmo-trx\n"
		"  -p    Base port number\n"
		"  -e    Enable EDGE receiver\n"
		"  -m    Enable multi-ARFCN transceiver (default=disabled)\n"
		"  -x    Enable external 10 MHz reference\n"
		"  -g    Enable GPSDO reference\n"
		"  -s    Tx samples-per-symbol (1 or 4)\n"
		"  -b    Rx samples-per-symbol (1 or 4)\n"
		"  -c    Number of ARFCN channels (default=1)\n"
		"  -f    Enable C0 filler table\n"
		"  -o    Set baseband frequency offset (default=auto)\n"
		"  -r    Random Normal Burst test mode with TSC\n"
		"  -A    Random Access Burst test mode with delay\n"
		"  -R    RSSI to dBm offset in dB (default=0)\n"
		"  -S    Swap channels (UmTRX only)\n"
		"  -t    SCHED_RR real-time priority (1..32)\n"
		"  -y    comma-delimited list of Tx paths (num elements matches -c)\n"
		"  -z    comma-delimited list of Rx paths (num elements matches -c)\n"
		);
}

static void handle_options(int argc, char **argv, struct trx_config *config)
{
	int option;
	bool tx_path_set = false, rx_path_set = false;

	config->local_addr = DEFAULT_TRX_IP;
	config->remote_addr = DEFAULT_TRX_IP;
	config->config_file = (char *)DEFAULT_CONFIG_FILE;
	config->port = DEFAULT_TRX_PORT;
	config->tx_sps = DEFAULT_TX_SPS;
	config->rx_sps = DEFAULT_RX_SPS;
	config->chans = DEFAULT_CHANS;
	config->rtsc = 0;
	config->rach_delay = 0;
	config->extref = false;
	config->gpsref = false;
	config->filler = Transceiver::FILLER_ZERO;
	config->mcbts = false;
	config->offset = 0.0;
	config->rssi_offset = 0.0;
	config->swap_channels = false;
	config->edge = false;
	config->sched_rr = -1;
	config->tx_paths = std::vector<std::string>(DEFAULT_CHANS, "");
	config->rx_paths = std::vector<std::string>(DEFAULT_CHANS, "");

	while ((option = getopt(argc, argv, "ha:i:j:p:c:dmxgfo:s:b:r:A:R:Set:y:z:C:")) != -1) {
		switch (option) {
		case 'h':
			print_help();
			exit(0);
			break;
		case 'a':
			config->dev_args = optarg;
			break;
		case 'i':
			config->remote_addr = optarg;
			break;
		case 'j':
			config->local_addr = optarg;
			break;
		case 'p':
			config->port = atoi(optarg);
			break;
		case 'c':
			config->chans = atoi(optarg);
			break;
		case 'm':
			config->mcbts = true;
			break;
		case 'x':
			config->extref = true;
			break;
		case 'g':
			config->gpsref = true;
			break;
		case 'f':
			config->filler = Transceiver::FILLER_DUMMY;
			break;
		case 'o':
			config->offset = atof(optarg);
			break;
		case 's':
			config->tx_sps = atoi(optarg);
			break;
		case 'b':
			config->rx_sps = atoi(optarg);
			break;
		case 'r':
			config->rtsc = atoi(optarg);
			config->filler = Transceiver::FILLER_NORM_RAND;
			break;
		case 'A':
			config->rach_delay = atoi(optarg);
			config->filler = Transceiver::FILLER_ACCESS_RAND;
			break;
		case 'R':
			config->rssi_offset = atof(optarg);
			break;
		case 'S':
			config->swap_channels = true;
			break;
		case 'e':
			config->edge = true;
			break;
		case 't':
			config->sched_rr = atoi(optarg);
			break;
		case 'y':
			config->tx_paths = comma_delimited_to_vector(optarg);
			tx_path_set = true;
			break;
		case 'z':
			config->rx_paths = comma_delimited_to_vector(optarg);
			rx_path_set = true;
			break;
		case 'C':
			config->config_file = optarg;
			break;
		default:
			print_help();
			exit(0);
		}
	}

	/* Force 4 SPS for EDGE or multi-ARFCN configurations */
	if ((config->edge) || (config->mcbts)) {
		config->tx_sps = 4;
		config->rx_sps = 4;
	}

	if (config->gpsref && config->extref) {
		printf("External and GPSDO references unavailable at the same time\n\n");
		goto bad_config;
	}

	if (config->edge && (config->filler == Transceiver::FILLER_NORM_RAND))
		config->filler = Transceiver::FILLER_EDGE_RAND;

	if ((config->tx_sps != 1) && (config->tx_sps != 4) &&
	    (config->rx_sps != 1) && (config->rx_sps != 4)) {
		printf("Unsupported samples-per-symbol %i\n\n", config->tx_sps);
		goto bad_config;
	}

	if (config->rtsc > 7) {
		printf("Invalid training sequence %i\n\n", config->rtsc);
		goto bad_config;
	}

	if (config->rach_delay > 68) {
		printf("RACH delay is too big %i\n\n", config->rach_delay);
		goto bad_config;
	}

	if (!tx_path_set) {
		config->tx_paths = std::vector<std::string>(config->chans, "");
	} else if (config->tx_paths.size() != config->chans) {
		printf("Num of channels and num of Tx Antennas doesn't match\n\n");
		goto bad_config;
	}
	if (!rx_path_set) {
		config->rx_paths = std::vector<std::string>(config->chans, "");
	} else if (config->rx_paths.size() != config->chans) {
		printf("Num of channels and num of Rx Antennas doesn't match\n\n");
		goto bad_config;
	}

	return;

bad_config:
	print_help();
	exit(0);
}

static int set_sched_rr(int prio)
{
	struct sched_param param;
	int rc;
	memset(&param, 0, sizeof(param));
	param.sched_priority = prio;
	printf("Setting SCHED_RR priority(%d)\n", param.sched_priority);
	rc = sched_setscheduler(getpid(), SCHED_RR, &param);
	if (rc != 0) {
		std::cerr << "Config: Setting SCHED_RR failed" << std::endl;
		return -1;
	}
	return 0;
}

static void trx_stop()
{
	std::cout << "Shutting down transceiver..." << std::endl;

	delete transceiver;
	delete radio;
	delete usrp;
}

static int trx_start(struct trx_config *config)
{
	int type, chans, ref;
	RadioDevice::InterfaceType iface = RadioDevice::NORMAL;

	/* Create the low level device object */
	if (config->mcbts)
		iface = RadioDevice::MULTI_ARFCN;

	if (config->extref)
		ref = RadioDevice::REF_EXTERNAL;
	else if (config->gpsref)
		ref = RadioDevice::REF_GPS;
	else
		ref = RadioDevice::REF_INTERNAL;

	usrp = RadioDevice::make(config->tx_sps, config->rx_sps, iface,
				 config->chans, config->offset, config->tx_paths, config->rx_paths);
	type = usrp->open(config->dev_args, ref, config->swap_channels);
	if (type < 0) {
		LOG(ALERT) << "Failed to create radio device" << std::endl;
		goto shutdown;
	}

	/* Setup the appropriate device interface */
	radio = makeRadioInterface(config, usrp, type);
	if (!radio)
		goto shutdown;

	/* Create the transceiver core */
	if(makeTransceiver(config, radio) < 0)
		goto shutdown;

	chans = transceiver->numChans();
	std::cout << "-- Transceiver active with "
		  << chans << " channel(s)" << std::endl;

	return 0;

shutdown:
	trx_stop();
	return -1;
}

int main(int argc, char *argv[])
{
	struct trx_config config;
	int rc;

	tall_trx_ctx = talloc_named_const(NULL, 0, "OsmoTRX");
	msgb_talloc_ctx_init(tall_trx_ctx, 0);
	g_vty_info.tall_ctx = tall_trx_ctx;

	setup_signal_handlers();

	g_trx_ctx = talloc_zero(tall_trx_ctx, struct trx_ctx);

#ifdef HAVE_SSE3
	printf("Info: SSE3 support compiled in");
#ifdef HAVE___BUILTIN_CPU_SUPPORTS
	if (__builtin_cpu_supports("sse3"))
		printf(" and supported by CPU\n");
	else
		printf(", but not supported by CPU\n");
#else
	printf(", but runtime SIMD detection disabled\n");
#endif
#endif

#ifdef HAVE_SSE4_1
	printf("Info: SSE4.1 support compiled in");
#ifdef HAVE___BUILTIN_CPU_SUPPORTS
	if (__builtin_cpu_supports("sse4.1"))
		printf(" and supported by CPU\n");
	else
		printf(", but not supported by CPU\n");
#else
	printf(", but runtime SIMD detection disabled\n");
#endif
#endif

	convolve_init();
	convert_init();

	osmo_init_logging(&log_info);
	osmo_stats_init(tall_trx_ctx);
	vty_init(&g_vty_info);
	ctrl_vty_init(tall_trx_ctx);
	trx_vty_init(g_trx_ctx);

	logging_vty_add_cmds();
	osmo_talloc_vty_add_cmds();
	osmo_stats_vty_add_cmds();

	handle_options(argc, argv, &config);

	rate_ctr_init(tall_trx_ctx);

	rc = vty_read_config_file(config.config_file, NULL);
	if (rc < 0) {
		fprintf(stderr, "Failed to open config file: '%s'\n", config.config_file);
		exit(2);
	}

	rc = telnet_init_dynif(tall_trx_ctx, NULL, vty_get_bind_addr(), OSMO_VTY_PORT_TRX);
	if (rc < 0)
		exit(1);

	g_ctrlh = ctrl_interface_setup(NULL, OSMO_CTRL_PORT_TRX, NULL);
	if (!g_ctrlh) {
		fprintf(stderr, "Failed to create CTRL interface.\n");
		exit(1);
	}

	if (config.sched_rr != -1) {
		if (set_sched_rr(config.sched_rr) < 0)
			return EXIT_FAILURE;
	}

	/* Check database sanity */
	if (!trx_setup_config(&config)) {
		std::cerr << "Config: Database failure - exiting" << std::endl;
		return EXIT_FAILURE;
	}

	srandom(time(NULL));

	if(trx_start(&config) < 0)
		return EXIT_FAILURE;

	while (!gshutdown)
		osmo_select_main(0);

	trx_stop();

	return 0;
}
