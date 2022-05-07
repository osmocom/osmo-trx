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

#include "Transceiver2.h"
#include "radioDevice.h"

#include <time.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

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
#include <osmocom/vty/command.h>
#include <osmocom/vty/cpu_sched_vty.h>

#include "convolve.h"
#include "convert.h"
#include "trx_vty.h"
#include "debug.h"
#include "osmo_signal.h"
#include "trx_rate_ctr.h"
}


//#include <mempool.cpp>
//atomicstackpool<NUM_ALLOCS,ALLOC_SZ> ALLOCPRIO a0;

/* Samples-per-symbol for downlink path
 *     4 - Uses precision modulator (more computation, less distortion)
 *     1 - Uses minimized modulator (less computation, more distortion)
 *
 *     Other values are invalid. Receive path (uplink) is always
 *     downsampled to 1 sps. Default to 4 sps for all cases except for
 *     ARM and non-SIMD enabled architectures.
 */
#if defined(HAVE_NEON) || !defined(HAVE_SSE3)
#define DEFAULT_SPS		1
#else
#define DEFAULT_SPS		4
#endif

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
	unsigned sps;
	unsigned chans;
	bool extref;
	bool filler;
	bool diversity;
	bool ms;
	double offset;
};



extern "C" volatile bool gshutdown = false;



/* Setup configuration values
 *     Don't query the existence of the Log.Level because it's a
 *     mandatory value. That is, if it doesn't exist, the configuration
 *     table will crash or will have already crashed. Everything else we
 *     can survive without and use default values if the database entries
 *     are empty.
 */
bool trx_setup_config(struct trx_config *config)
{
	std::string refstr, fillstr, divstr, msstr;

	config->log_level = "foo";
	config->port = DEFAULT_TRX_PORT;
	config->addr = DEFAULT_TRX_IP;
	//config->extref = DEFAULT_EXTREF;
	config->diversity = DEFAULT_DIVERSITY;
	config->sps = 4;//DEFAULT_SPS;
	config->chans = DEFAULT_CHANS;

	/* Diversity only supported on 2 channels */
	if (config->diversity)
		config->chans = 2;

	refstr = config->extref ? "Enabled" : "Disabled";
	fillstr = config->filler ? "Enabled" : "Disabled";
	divstr = config->diversity ? "Enabled" : "Disabled";
	msstr = config->ms ? "Enabled" : "Disabled";

	std::ostringstream ost("");
	ost << "Config Settings" << std::endl;
	ost << "   Log Level............... " << config->log_level << std::endl;
	ost << "   Device args............. " << config->dev_args << std::endl;
	ost << "   TRX Base Port........... " << config->port << std::endl;
	ost << "   TRX Address............. " << config->addr << std::endl;
	ost << "   Channels................ " << config->chans << std::endl;
	ost << "   Samples-per-Symbol...... " << config->sps << std::endl;
	ost << "   External Reference...... " << refstr << std::endl;
	ost << "   C0 Filler Table......... " << fillstr << std::endl;
	ost << "   Diversity............... " << divstr << std::endl;
	ost << "   MS Mode................. " << msstr << std::endl;
	ost << "   Tuning offset........... " << config->offset << std::endl;
	std::cout << ost.str() << std::endl;

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
	size_t div = 1;
	int offset = 3;

	if (config->ms) {
		if (type != RadioDevice::NORMAL) {
			LOG(ALERT) << "Unsupported configuration";
			return NULL;
		}

		offset *= -1;
	}

	switch (type) {
	case RadioDevice::NORMAL:
		radio = new RadioInterface(usrp, config->sps, config->sps,
					   config->chans, div, offset);
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
Transceiver2 *makeTransceiver(struct trx_config *config, RadioInterface *radio)
{
	Transceiver2 *trx;
	VectorFIFO *fifo;

	trx = new Transceiver2(config->port, config->addr.c_str(), config->sps,
			      config->chans, GSM::Time(3,0), radio);
	if (!trx->init(config->filler)) {
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
		"  -d    Enable dual channel diversity receiver\n"
		"  -x    Enable external 10 MHz reference\n"
		"  -s    Samples-per-symbol (1 or 4)\n"
		"  -c    Number of ARFCN channels (default=1)\n"
		"  -f    Enable C0 filler table\n"
		"  -m    Enable MS mode\n"
		"  -o    Set baseband frequency offset (default=auto)\n",
		"EMERG, ALERT, CRT, ERR, WARNING, NOTICE, INFO, DEBUG");
}

static void handle_options(int argc, char **argv, struct trx_config *config)
{
	int option;

	optind=1;

	config->port = 0;
	config->sps = 0;
	config->chans = 0;
	config->extref = true;
	config->filler = false;
	config->diversity = false;
	config->ms = true;
	config->offset = 0.0;

	while ((option = getopt(argc, argv, "xo")) != -1) {
		switch (option) {
		case 'x':
			config->extref = true;
			break;
		case 'o':
			config->offset = atof(optarg);
			break;
		default:
			print_help();
			exit(0);
		}
	}
}

const char* commands[] {
"CMD POWEROFF",
//"CMD RXTUNE 1782000",
"CMD RXTUNE 931400",
//"CMD TXTUNE 1877000",
"CMD TXTUNE 931900",
"CMD SETTSC 7",
"CMD POWERON",
"CMD SETRXGAIN 30",
"CMD SETPOWER 0",
//"CMD SETSLOT 6 7",
//"CMD SETSLOT 7 13",
//"CMD NOHANDOVER 1 0",
"CMD SYNC",
"CMD GETBSIC",
};

const char* commands2[] {
"CMD GETBSIC",
};

RadioDevice *usrp;
RadioInterface *radio = NULL;
Transceiver2 *trx = NULL;

	void* tall_trx_ctx;
	struct trx_ctx* g_trx_ctx;

int trx_main(int argc, char *argv[])
{
	int type, chans;

	struct trx_config config;

	std::cerr << "starting.." << std::endl;

	convolve_init();
	convert_init();


	tall_trx_ctx = talloc_named_const(NULL, 0, "OsmoTRX");
	msgb_talloc_ctx_init(tall_trx_ctx, 0);
	g_vty_info.tall_ctx = tall_trx_ctx;

	//setup_signal_handlers();

	g_trx_ctx = vty_trx_ctx_alloc(tall_trx_ctx);

	osmo_init_logging2(tall_trx_ctx, &log_info);
	log_enable_multithread();
	//osmo_stats_init(tall_trx_ctx);
	vty_init(&g_vty_info);
	logging_vty_add_cmds();
	//ctrl_vty_init(tall_trx_ctx);
	osmo_cpu_sched_vty_init(tall_trx_ctx);
	trx_vty_init(g_trx_ctx);

	osmo_talloc_vty_add_cmds();
	//osmo_stats_vty_add_cmds();


	sched_param sch_params;
	sch_params.sched_priority = 19;
	pthread_setschedparam(pthread_self(), SCHED_RR, &sch_params);

	handle_options(argc, argv, &config);

	//setup_signal_handlers();

	/* Check database sanity */
	if (!trx_setup_config(&config)) {
		std::cerr << "Config: Database failure - exiting" << std::endl;
		return EXIT_FAILURE;
	}

	//gLogInit("transceiver", config.log_level.c_str(), LOG_LOCAL7);

	srandom(time(NULL));

	/* Create the low level device object */
	usrp = RadioDevice::make(config.sps, config.sps, RadioDevice::NORMAL, config.chans,
		config.offset);
	type = usrp->open(config.dev_args, config.extref, false);
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

	trx->start();

	chans = trx->numChans();
	std::cout << "-- Transceiver active with "
		  << chans << " channel(s)" << std::endl;


#if 0
    for(auto i: commands) {
    int MAX_PACKET_LENGTH = 100;
    char response[MAX_PACKET_LENGTH];
    char buffer[MAX_PACKET_LENGTH];
    memset(response, 0, sizeof(response));
    memset(buffer, 0, sizeof(buffer));
	sprintf(buffer, "%s", i);
	trx->commandhandler(buffer, response, 0);
	std::cerr << i << "\tr: " << response << " ##" << std::endl;
	}

    sleep(30);

    for(auto i: commands2) {
    int MAX_PACKET_LENGTH = 100;
    char response[MAX_PACKET_LENGTH];
    char buffer[MAX_PACKET_LENGTH];
    memset(response, 0, sizeof(response));
    memset(buffer, 0, sizeof(buffer));
	sprintf(buffer, "%s", i);
	trx->commandhandler(buffer, response, 0);
	std::cerr << i << "\tr: " << response << " ##" << std::endl;
	}
#endif

//	while (!gshutdown)
//		sleep(1);

shutdown:
//	std::cout << "Shutting down transceiver..." << std::endl;

//	delete trx;
//	delete radio;
//	delete usrp;

	return 0;
}

extern "C" void init_external_transceiver(int argc, char **argv) {
	trx_main(argc, argv);
}

extern "C" void stop_trx() {
	std::cout << "Shutting down transceiver..." << std::endl;

	delete trx;
	delete radio;
	delete usrp;

}
