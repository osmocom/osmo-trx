/*! \file src/main.c
 * osmo-trx-proxy: main program entry point. */

/*
 * (C) 2026 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 * Author: Vadim Yanitskiy <vyanitskiy@sysmocom.de>
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <getopt.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/select.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/application.h>
#include <osmocom/vty/vty.h>
#include <osmocom/vty/command.h>
#include <osmocom/vty/cpu_sched_vty.h>
#include <osmocom/vty/telnet_interface.h>
#include <osmocom/vty/ports.h>

#include <osmocom/proxy/proxy.h>
#include <osmocom/proxy/vty.h>
#include <osmocom/proxy/trx.h>
#include <osmocom/proxy/logging.h>

void *g_talloc_ctx;
static char *g_config_file = "osmo-trx-proxy.cfg";
static volatile sig_atomic_t g_quit = 0;

static void print_help(void)
{
	printf("Usage: osmo-trx-proxy [options]\n");
	printf("  -h, --help                This text\n");
	printf("  -c, --config-file FILE    Specify the configuration file (default: %s)\n",
	       g_config_file);
}

static void handle_options(int argc, char **argv)
{
	while (1) {
		int option_index = 0;
		static const struct option long_options[] = {
			{ "help", 0, 0, 'h' },
			{ "config-file", 1, 0, 'c' },
			{ 0, 0, 0, 0 }
		};

		int c = getopt_long(argc, argv, "hc:", long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help();
			exit(0);
		case 'c':
			g_config_file = optarg;
			break;
		default:
			exit(2);
		}
	}
}

static void signal_handler(int signum)
{
	switch (signum) {
	case SIGINT:
	case SIGTERM:
		g_quit = 1;
		break;
	default:
		break;
	}
}

int main(int argc, char **argv)
{
	int rc;

	g_talloc_ctx = talloc_named_const(NULL, 0, "osmo-trx-proxy");
	msgb_talloc_ctx_init(g_talloc_ctx, 0);

	signal(SIGINT, &signal_handler);
	signal(SIGTERM, &signal_handler);
	osmo_init_ignore_signals();

	g_proxy_ctx = proxy_ctx_alloc(g_talloc_ctx);
	OSMO_ASSERT(g_proxy_ctx != NULL);

	osmo_init_logging2(g_talloc_ctx, &log_info);
	log_set_print_extended_timestamp(osmo_stderr_target, 1);
	log_set_print_category_hex(osmo_stderr_target, 0);
	log_set_print_category(osmo_stderr_target, 1);
	log_set_print_level(osmo_stderr_target, 1);
	log_set_print_filename2(osmo_stderr_target, LOG_FILENAME_BASENAME);
	log_set_print_filename_pos(osmo_stderr_target, LOG_FILENAME_POS_LINE_END);

	proxy_vty_init();

	handle_options(argc, argv);

	rc = vty_read_config_file(g_config_file, NULL);
	if (rc < 0) {
		fprintf(stderr, "Failed to open config file: '%s'\n", g_config_file);
		return 2;
	}

	rc = telnet_init_default(g_talloc_ctx, NULL, OSMO_VTY_PORT_TRX);
	if (rc < 0)
		return 1;

	osmo_cpu_sched_vty_apply_localthread();

	/* Only apply the built-in BTS/MS defaults if the config file did not
	 * declare any 'ep' stanza of its own. */
	rc = proxy_ctx_add_defaults_if_empty(g_proxy_ctx);
	if (rc < 0)
		return 1;

	LOGP(DPROXY, LOGL_NOTICE, "osmo-trx-proxy started\n");

	while (!g_quit)
		osmo_select_main(0);

	LOGP(DPROXY, LOGL_NOTICE, "Shutting down...\n");
	proxy_ctx_free(g_proxy_ctx);
	return 0;
}
