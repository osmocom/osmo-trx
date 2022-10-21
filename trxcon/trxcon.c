/*
 * OsmocomBB <-> SDR connection bridge
 *
 * (C) 2016-2020 by Vadim Yanitskiy <axilirator@gmail.com>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>

#include <arpa/inet.h>

#include <osmocom/core/fsm.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/signal.h>
#include <osmocom/core/select.h>
#include <osmocom/core/application.h>
#include <osmocom/core/gsmtap_util.h>
#include <osmocom/core/gsmtap.h>

#include <osmocom/gsm/gsm_utils.h>

#include "trxcon.h"
#include "trx_if.h"
#include "logging.h"
#include "l1ctl.h"
#include "l1ctl_link.h"
#include "l1ctl_proto.h"
#include "scheduler.h"
#include "sched_trx.h"

#define COPYRIGHT \
	"Copyright (C) 2016-2020 by Vadim Yanitskiy <axilirator@gmail.com>\n" \
	"License GPLv2+: GNU GPL version 2 or later " \
	"<http://gnu.org/licenses/gpl.html>\n" \
	"This is free software: you are free to change and redistribute it.\n" \
	"There is NO WARRANTY, to the extent permitted by law.\n\n"

static struct {
	const char *debug_mask;
	int daemonize;
	int quit;

	/* L1CTL specific */
	struct l1ctl_link *l1l;
	const char *bind_socket;

	/* TRX specific */
	struct trx_instance *trx;
	const char *trx_bind_ip;
	const char *trx_remote_ip;
	uint16_t trx_base_port;
	uint32_t trx_fn_advance;
	const char *gsmtap_ip;
} app_data;

static void *tall_trxcon_ctx = NULL;
struct gsmtap_inst *gsmtap = NULL;
struct osmo_fsm_inst *trxcon_fsm;

static void trxcon_fsm_idle_action(struct osmo_fsm_inst *fi,
	uint32_t event, void *data)
{
	if (event == L1CTL_EVENT_CONNECT)
		osmo_fsm_inst_state_chg(trxcon_fsm, TRXCON_STATE_MANAGED, 0, 0);
}

static void trxcon_fsm_managed_action(struct osmo_fsm_inst *fi,
	uint32_t event, void *data)
{
	switch (event) {
	case L1CTL_EVENT_DISCONNECT:
		osmo_fsm_inst_state_chg(trxcon_fsm, TRXCON_STATE_IDLE, 0, 0);

		if (app_data.trx->fsm->state != TRX_STATE_OFFLINE) {
			/* Reset scheduler and clock counter */
			sched_trx_reset(app_data.trx, true);

			/* TODO: implement trx_if_reset() */
			trx_if_cmd_poweroff(app_data.trx);
			trx_if_cmd_echo(app_data.trx);
		}
		break;
	case TRX_EVENT_RSP_ERROR:
	case TRX_EVENT_OFFLINE:
		/* TODO: notify L2 & L3 about that */
		break;
	default:
		LOGPFSML(fi, LOGL_ERROR, "Unhandled event %u\n", event);
	}
}

static struct osmo_fsm_state trxcon_fsm_states[] = {
	[TRXCON_STATE_IDLE] = {
		.in_event_mask = GEN_MASK(L1CTL_EVENT_CONNECT),
		.out_state_mask = GEN_MASK(TRXCON_STATE_MANAGED),
		.name = "IDLE",
		.action = trxcon_fsm_idle_action,
	},
	[TRXCON_STATE_MANAGED] = {
		.in_event_mask = (
			GEN_MASK(L1CTL_EVENT_DISCONNECT) |
			GEN_MASK(TRX_EVENT_RSP_ERROR) |
			GEN_MASK(TRX_EVENT_OFFLINE)),
		.out_state_mask = GEN_MASK(TRXCON_STATE_IDLE),
		.name = "MANAGED",
		.action = trxcon_fsm_managed_action,
	},
};

static const struct value_string app_evt_names[] = {
	OSMO_VALUE_STRING(L1CTL_EVENT_CONNECT),
	OSMO_VALUE_STRING(L1CTL_EVENT_DISCONNECT),
	OSMO_VALUE_STRING(TRX_EVENT_OFFLINE),
	OSMO_VALUE_STRING(TRX_EVENT_RSP_ERROR),
	{ 0, NULL }
};

static struct osmo_fsm trxcon_fsm_def = {
	.name = "trxcon_app_fsm",
	.states = trxcon_fsm_states,
	.num_states = ARRAY_SIZE(trxcon_fsm_states),
	.log_subsys = DAPP,
	.event_names = app_evt_names,
};

static void print_usage(const char *app)
{
	printf("Usage: %s\n", app);
}

static void print_help(void)
{
	printf(" Some help...\n");
	printf("  -h --help         this text\n");
	printf("  -d --debug        Change debug flags. Default: %s\n", DEBUG_DEFAULT);
	printf("  -b --trx-bind     TRX bind IP address (default 0.0.0.0)\n");
	printf("  -i --trx-remote   TRX remote IP address (default 127.0.0.1)\n");
	printf("  -p --trx-port     Base port of TRX instance (default 6700)\n");
	printf("  -f --trx-advance  Uplink burst scheduling advance (default 3)\n");
	printf("  -s --socket       Listening socket for layer23 (default /tmp/osmocom_l2)\n");
	printf("  -g --gsmtap-ip    The destination IP used for GSMTAP (disabled by default)\n");
	printf("  -D --daemonize    Run as daemon\n");
}

static void handle_options(int argc, char **argv)
{
	while (1) {
		int option_index = 0, c;
		static struct option long_options[] = {
			{"help", 0, 0, 'h'},
			{"debug", 1, 0, 'd'},
			{"socket", 1, 0, 's'},
			{"trx-bind", 1, 0, 'b'},
			/* NOTE: 'trx-ip' is now an alias for 'trx-remote'
			 * due to backward compatibility reasons! */
			{"trx-ip", 1, 0, 'i'},
			{"trx-remote", 1, 0, 'i'},
			{"trx-port", 1, 0, 'p'},
			{"trx-advance", 1, 0, 'f'},
			{"gsmtap-ip", 1, 0, 'g'},
			{"daemonize", 0, 0, 'D'},
			{0, 0, 0, 0}
		};

		c = getopt_long(argc, argv, "d:b:i:p:f:s:g:Dh",
				long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_usage(argv[0]);
			print_help();
			exit(0);
			break;
		case 'd':
			app_data.debug_mask = optarg;
			break;
		case 'b':
			app_data.trx_bind_ip = optarg;
			break;
		case 'i':
			app_data.trx_remote_ip = optarg;
			break;
		case 'p':
			app_data.trx_base_port = atoi(optarg);
			break;
		case 'f':
			app_data.trx_fn_advance = atoi(optarg);
			break;
		case 's':
			app_data.bind_socket = optarg;
			break;
		case 'g':
			app_data.gsmtap_ip = optarg;
			break;
		case 'D':
			app_data.daemonize = 1;
			break;
		default:
			break;
		}
	}
}

static void init_defaults(void)
{
	app_data.bind_socket = "/tmp/osmocom_l2";
	app_data.trx_remote_ip = "127.0.0.1";
	app_data.trx_bind_ip = "0.0.0.0";
	app_data.trx_base_port = 6700;
	app_data.trx_fn_advance = 3;

	app_data.debug_mask = NULL;
	app_data.gsmtap_ip = NULL;
	app_data.daemonize = 0;
	app_data.quit = 0;
}

static void signal_handler(int signum)
{
	fprintf(stderr, "signal %u received\n", signum);

	switch (signum) {
	case SIGINT:
		app_data.quit++;
		break;
	case SIGABRT:
		/* in case of abort, we want to obtain a talloc report and
		 * then run default SIGABRT handler, who will generate coredump
		 * and abort the process. abort() should do this for us after we
		 * return, but program wouldn't exit if an external SIGABRT is
		 * received.
		 */
		talloc_report_full(tall_trxcon_ctx, stderr);
		signal(SIGABRT, SIG_DFL);
		raise(SIGABRT);
		break;
	case SIGUSR1:
	case SIGUSR2:
		talloc_report_full(tall_trxcon_ctx, stderr);
		break;
	default:
		break;
	}
}

extern void init_external_transceiver(int argc, char **argv);
extern void stop_trx();
extern volatile bool gshutdown;

int main(int argc, char **argv)
{
	int rc = 0;

	cpu_set_t cpuset;

	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

	int prio = sched_get_priority_max(SCHED_RR) - 5;
	struct sched_param param;
	param.sched_priority = prio;
	int rv = sched_setscheduler(0, SCHED_RR, &param);
	if (rv < 0) {
		LOGP(DAPP, LOGL_ERROR, "Failed to set sched!\n");
		exit(0);
	}

	printf("%s", COPYRIGHT);
	init_defaults();
	handle_options(argc, argv);

	/* Track the use of talloc NULL memory contexts */
	talloc_enable_null_tracking();

	/* Init talloc memory management system */
	tall_trxcon_ctx = talloc_init("trxcon context");
	msgb_talloc_ctx_init(tall_trxcon_ctx, 0);

	/* Setup signal handlers */
//	signal(SIGINT, &signal_handler);
//	signal(SIGABRT, &signal_handler);
//	signal(SIGUSR1, &signal_handler);
//	signal(SIGUSR2, &signal_handler);
	osmo_init_ignore_signals();

	/* Init logging system */
	trx_log_init(tall_trxcon_ctx, app_data.debug_mask);

	/* Configure pretty logging */
	log_set_print_extended_timestamp(osmo_stderr_target, 1);
	log_set_print_category_hex(osmo_stderr_target, 0);
	log_set_print_category(osmo_stderr_target, 1);
	log_set_print_level(osmo_stderr_target, 1);

	/* Optional GSMTAP  */
	if (app_data.gsmtap_ip != NULL) {
		gsmtap = gsmtap_source_init(app_data.gsmtap_ip, GSMTAP_UDP_PORT, 1);
		if (!gsmtap) {
			LOGP(DAPP, LOGL_ERROR, "Failed to init GSMTAP\n");
			goto exit;
		}
		/* Suppress ICMP "destination unreachable" errors */
		gsmtap_source_add_sink(gsmtap);
	}

	/* Allocate the application state machine */
	OSMO_ASSERT(osmo_fsm_register(&trxcon_fsm_def) == 0);
	trxcon_fsm = osmo_fsm_inst_alloc(&trxcon_fsm_def, tall_trxcon_ctx,
		NULL, LOGL_DEBUG, "main");

	/* Init L1CTL server */
	app_data.l1l = l1ctl_link_init(tall_trxcon_ctx,
		app_data.bind_socket);
	if (app_data.l1l == NULL)
		goto exit;

	/* Init transceiver interface */
	app_data.trx = trx_if_open(tall_trxcon_ctx,
		app_data.trx_bind_ip, app_data.trx_remote_ip,
		app_data.trx_base_port);
	if (!app_data.trx)
		goto exit;

	/* Bind L1CTL with TRX and vice versa */
	app_data.l1l->trx = app_data.trx;
	app_data.trx->l1l = app_data.l1l;

	/* Init scheduler */
	rc = sched_trx_init(app_data.trx, app_data.trx_fn_advance);
	if (rc)
		goto exit;

	LOGP(DAPP, LOGL_NOTICE, "Init complete\n");

	if (app_data.daemonize) {
		rc = osmo_daemonize();
		if (rc < 0) {
			perror("Error during daemonize");
			goto exit;
		}
	}

	/* Initialize pseudo-random generator */
	srand(time(NULL));

	init_external_transceiver(argc, argv);

	while (!app_data.quit)
		osmo_select_main(0);

	gshutdown = true;
	stop_trx();


exit:
	/* Close active connections */
	l1ctl_link_shutdown(app_data.l1l);
	sched_trx_shutdown(app_data.trx);
	trx_if_close(app_data.trx);

	/* Shutdown main state machine */
	osmo_fsm_inst_free(trxcon_fsm);

	/* Deinitialize logging */
	log_fini();

	/**
	 * Print report for the root talloc context in order
	 * to be able to find and fix potential memory leaks.
	 */
	talloc_report_full(tall_trxcon_ctx, stderr);
	talloc_free(tall_trxcon_ctx);

	/* Make both Valgrind and ASAN happy */
	talloc_report_full(NULL, stderr);
	talloc_disable_null_tracking();

	return rc;
}
