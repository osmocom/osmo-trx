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

#include <GSMCommon.h>
#include <Logger.h>
#include <Configuration.h>

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
 *     4 - Provides better timing precision (avoids FPGA DDC timing ambiguity)
 *     1 - Reduces processor load, but increases timing ambiguity
 */
#define DEFAULT_RX_SPS		4

/* Default configuration parameters
 *     Note that these values are only used if the particular key does not
 *     exist in the configuration database. IP port and address values will
 *     typically be overwritten by the OpenBTS.db values. Other values will
 *     not be in the database by default.
 */
#define DEFAULT_TRX_PORT	5700
#define DEFAULT_TRX_IP		"127.0.0.1"
#define DEFAULT_EXTREF		false
#define DEFAULT_DIVERSITY	false
#define DEFAULT_CHANS		1

struct trx_config {
	std::string log_level;
	std::string addr;
	std::string dev_args;
	unsigned port;
	unsigned tx_sps;
	unsigned rx_sps;
	unsigned chans;
	unsigned rtsc;
	unsigned rach_delay;
	bool extref;
	bool gpsref;
	Transceiver::FillerType filler;
	bool diversity;
	bool mcbts;
	double offset;
	double rssi_offset;
	bool swap_channels;
	bool edge;
};

ConfigurationTable gConfig;

volatile bool gshutdown = false;

/* Run sanity check on configuration table
 *     The global table constructor cannot provide notification in the
 *     event of failure. Make sure that we can access the database,
 *     write to it, and that it contains the bare minimum required keys.
 */
bool testConfig()
{
	int val = 9999;
	std::string test = "asldfkjsaldkf";
	const char *key = "Log.Level";

	/* Attempt to query */
	try {
		gConfig.getStr(key);
	} catch (...) {
		std::cerr << std::endl;
		std::cerr << "Config: Failed query required key " << key
			  << std::endl;
		return false;
	}

	/* Attempt to set a test value in the global config */
	if (!gConfig.set(test, val)) {
		std::cerr << std::endl;
		std::cerr << "Config: Failed to set test key" << std::endl;
		return false;
	} else {
		gConfig.remove(test);
	}

	return true;
}


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

	if (!testConfig())
		return false;

	if (config->log_level == "")
		config->log_level = gConfig.getStr("Log.Level");

	if (!config->port) {
		if (gConfig.defines("TRX.Port"))
			config->port = gConfig.getNum("TRX.Port");
		else
			config->port = DEFAULT_TRX_PORT;
	}

	if (config->addr == "") {
		if (gConfig.defines("TRX.IP"))
			config->addr = gConfig.getStr("TRX.IP");
		else
			config->addr = DEFAULT_TRX_IP;
	}

	if (!config->extref) {
		if (gConfig.defines("TRX.Reference"))
			config->extref = gConfig.getNum("TRX.Reference");
		else
			config->extref = DEFAULT_EXTREF;
	}

	if (!config->diversity) {
		if (gConfig.defines("TRX.Diversity"))
			config->diversity = gConfig.getNum("TRX.Diversity");
		else
			config->diversity = DEFAULT_DIVERSITY;
	}

	if (!config->chans)
		config->chans = DEFAULT_CHANS;

	if (config->mcbts && ((config->chans < 0) || (config->chans > 5))) {
		std::cout << "Unsupported number of channels" << std::endl;
		return false;
	}

	edgestr = config->edge ? "Enabled" : "Disabled";
	divstr = config->diversity ? "Enabled" : "Disabled";
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
	ost << "   Log Level............... " << config->log_level << std::endl;
	ost << "   Device args............. " << config->dev_args << std::endl;
	ost << "   TRX Base Port........... " << config->port << std::endl;
	ost << "   TRX Address............. " << config->addr << std::endl;
	ost << "   Channels................ " << config->chans << std::endl;
	ost << "   Tx Samples-per-Symbol... " << config->tx_sps << std::endl;
	ost << "   Rx Samples-per-Symbol... " << config->rx_sps << std::endl;
	ost << "   EDGE support............ " << edgestr << std::endl;
	ost << "   Reference............... " << refstr << std::endl;
	ost << "   C0 Filler Table......... " << fillstr << std::endl;
	ost << "   Multi-Carrier........... " << mcstr << std::endl;
	ost << "   Diversity............... " << divstr << std::endl;
	ost << "   Tuning offset........... " << config->offset << std::endl;
	ost << "   RSSI to dBm offset...... " << config->rssi_offset << std::endl;
	ost << "   Swap channels........... " << config->swap_channels << std::endl;
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
	case RadioDevice::DIVERSITY:


		radio = new RadioInterfaceDiversity(usrp, config->tx_sps,
						    config->chans);
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
Transceiver *makeTransceiver(struct trx_config *config, RadioInterface *radio)
{
	Transceiver *trx;
	VectorFIFO *fifo;

	trx = new Transceiver(config->port, config->addr.c_str(),
			      config->tx_sps, config->rx_sps, config->chans,
			      GSM::Time(3,0), radio, config->rssi_offset);
	if (!trx->init(config->filler, config->rtsc,
		       config->rach_delay, config->edge)) {
		LOG(ALERT) << "Failed to initialize transceiver";
		delete trx;
		return NULL;
	}

	for (size_t i = 0; i < config->chans; i++) {
		fifo = radio->receiveFIFO(i);
		if (fifo && trx->receiveFIFO(fifo, i))
			continue;

		LOG(ALERT) << "Could not attach FIFO to channel " << i;
		delete trx;
		return NULL;
	}

	return trx;
}

static void sig_handler(int signo)
{
	fprintf(stdout, "Received shutdown signal");
	gshutdown = true;
}

static void setup_signal_handlers()
{
	if (signal(SIGINT, sig_handler) == SIG_ERR) {
		fprintf(stderr, "Failed to install SIGINT signal handler\n");
		exit(EXIT_FAILURE);
	}
	if (signal(SIGTERM, sig_handler) == SIG_ERR) {
		fprintf(stderr, "Couldn't install SIGTERM signal handler\n");
		exit( EXIT_FAILURE);
	}
}

static void print_help()
{
	fprintf(stdout, "Options:\n"
		"  -h    This text\n"
		"  -a    UHD device args\n"
		"  -l    Logging level (%s)\n"
		"  -i    IP address of GSM core\n"
		"  -p    Base port number\n"
		"  -e    Enable EDGE receiver\n"
		"  -d    Enable dual channel diversity receiver (deprecated)\n"
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
		"  -S    Swap channels (UmTRX only)\n",
		"EMERG, ALERT, CRT, ERR, WARNING, NOTICE, INFO, DEBUG");
}

static void handle_options(int argc, char **argv, struct trx_config *config)
{
	int option;

	config->port = 0;
	config->tx_sps = DEFAULT_TX_SPS;
	config->rx_sps = DEFAULT_RX_SPS;
	config->chans = DEFAULT_CHANS;
	config->rtsc = 0;
	config->rach_delay = 0;
	config->extref = false;
	config->gpsref = false;
	config->filler = Transceiver::FILLER_ZERO;
	config->mcbts = false;
	config->diversity = false;
	config->offset = 0.0;
	config->rssi_offset = 0.0;
	config->swap_channels = false;
	config->edge = false;

	while ((option = getopt(argc, argv, "ha:l:i:p:c:dmxgfo:s:b:r:A:R:Se")) != -1) {
		switch (option) {
		case 'h':
			print_help();
			exit(0);
			break;
		case 'a':
			config->dev_args = optarg;
			break;
		case 'l':
			config->log_level = optarg;
			break;
		case 'i':
			config->addr = optarg;
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
		case 'd':
			config->diversity = true;
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

	/* Special restrictions on (deprecated) diversity configuration */
	if (config->diversity) {
		if (config->mcbts || config->edge) {
			std::cout << "Multi-carrier/EDGE diversity unsupported" << std::endl;
			goto bad_config;
		}

		if (config->rx_sps != 1) {
			std::cout << "Diversity only supported with 1 SPS" << std::endl;
			goto bad_config;
		}

		if (config->chans != 2) {
			std::cout << "Diversity only supported with 2 channels" << std::endl;
			goto bad_config;
		}
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

	return;

bad_config:
	print_help();
	exit(0);
}

int main(int argc, char *argv[])
{
	int type, chans, ref;
	RadioDevice *usrp;
	RadioInterface *radio = NULL;
	Transceiver *trx = NULL;
	RadioDevice::InterfaceType iface = RadioDevice::NORMAL;
	struct trx_config config;

	handle_options(argc, argv, &config);

	setup_signal_handlers();

	/* Check database sanity */
	if (!trx_setup_config(&config)) {
		std::cerr << "Config: Database failure - exiting" << std::endl;
		return EXIT_FAILURE;
	}

	gLogInit("transceiver", config.log_level.c_str(), LOG_LOCAL7);

	srandom(time(NULL));

	/* Create the low level device object */
	if (config.mcbts)
		iface = RadioDevice::MULTI_ARFCN;

	if (config.extref)
		ref = RadioDevice::REF_EXTERNAL;
	else if (config.gpsref)
		ref = RadioDevice::REF_GPS;
	else
		ref = RadioDevice::REF_INTERNAL;

	usrp = RadioDevice::make(config.tx_sps, config.rx_sps, iface,
				 config.chans, config.offset);
	type = usrp->open(config.dev_args, ref, config.swap_channels);
	if (type < 0) {
		LOG(ALERT) << "Failed to create radio device" << std::endl;
		goto shutdown;
	}

	/* Setup the appropriate device interface */
	radio = makeRadioInterface(&config, usrp, type);
	if (!radio)
		goto shutdown;

	/* Create the transceiver core */
	trx = makeTransceiver(&config, radio);
	if (!trx)
		goto shutdown;

	chans = trx->numChans();
	std::cout << "-- Transceiver active with "
		  << chans << " channel(s)" << std::endl;

	while (!gshutdown)
		sleep(1);

shutdown:
	std::cout << "Shutting down transceiver..." << std::endl;

	delete trx;
	delete radio;
	delete usrp;

	return 0;
}
