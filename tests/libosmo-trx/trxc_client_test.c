/*! \file tests/trxc_client_test.c
 * Regression test for the TRXC client command queue engine. */

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
#include <string.h>

#include <osmocom/core/application.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/timer_compat.h>
#include <osmocom/core/utils.h>

#include <osmocom/trx/trxc.h>
#include <osmocom/trx/trxc_client.h>

static void *test_ctx = NULL;

/* feed a response into the engine, printing what happens */
static void rx_rsp(struct osmo_trxc_client *client, const char *rsp)
{
	int rc;

	printf("rx_rsp: '%s'\n", rsp);
	rc = osmo_trxc_client_rx(client, rsp, strlen(rsp));
	if (rc != 0)
		printf("\trc=%d\n", rc);
}

/* advance the (overridden) time and fire expired timers */
static void fake_time_passes(time_t sec)
{
	printf("(time passes: %ld s)\n", (long)sec);
	osmo_gettimeofday_override_add(sec, 0);
	osmo_timers_prepare();
	osmo_timers_update();
}

static int tx_msg_cb(struct osmo_trxc_client *client, const struct osmo_trxc_msg *msg)
{
	printf("tx_msg: '%s'\n", osmo_trxc_msg_name(msg));
	return 0;
}

static void fatal_error_cb(struct osmo_trxc_client *client,
			   const struct osmo_trxc_msg *rsp)
{
	printf("fatal_error: '%s'\n", rsp ? osmo_trxc_msg_name(rsp) : "(null)");
}

static const struct osmo_trxc_client_ops test_ops = {
	.tx_msg = &tx_msg_cb,
	.fatal_error = &fatal_error_cb,
	.priv = "test-priv",
};

static struct osmo_trxc_client *client_alloc(void)
{
	struct osmo_trxc_client *client;

	client = osmo_trxc_client_alloc(test_ctx, &test_ops);
	OSMO_ASSERT(client != NULL);
	OSMO_ASSERT(strcmp(osmo_trxc_client_get_priv(client), "test-priv") == 0);
	osmo_trxc_client_set_name(client, "phy%u.trx%u", 0, 0);

	return client;
}

static int rsp_cb(struct osmo_trxc_client *client,
		  const struct osmo_trxc_msg *rsp, void *cb_data)
{
	printf("rsp_cb(%s): '%s'\n", (const char *)cb_data, osmo_trxc_msg_name(rsp));
	return 0;
}

static void test_basic(void)
{
	struct osmo_trxc_client *client = client_alloc();

	printf("=== %s ===\n", __func__);

	/* a command is transmitted immediately when the queue is empty */
	osmo_trxc_client_poweron(client, &rsp_cb, "poweron");
	rx_rsp(client, "RSP POWERON 0");

	osmo_trxc_client_free(client);
}

static void test_queueing(void)
{
	struct osmo_trxc_client *client = client_alloc();

	printf("=== %s ===\n", __func__);

	/* only the first command is transmitted... */
	osmo_trxc_client_rxtune(client, 890000, &rsp_cb, "rxtune");
	osmo_trxc_client_txtune(client, 935000, &rsp_cb, "txtune");
	osmo_trxc_client_send_cmd(client, 0, &rsp_cb, "setslot",
				  OSMO_TRXC_CMD_SETSLOT, "%u %u", 0, 1);
	/* ... consecutive duplicates are not enqueued at all */
	osmo_trxc_client_send_cmd(client, 0, &rsp_cb, "setslot",
				  OSMO_TRXC_CMD_SETSLOT, "%u %u", 0, 1);

	/* each response triggers transmission of the next command */
	rx_rsp(client, "RSP RXTUNE 0 890000");
	rx_rsp(client, "RSP TXTUNE 0 935000");
	rx_rsp(client, "RSP SETSLOT 0 0 1");

	osmo_trxc_client_free(client);
}

static void test_dup_rsp(void)
{
	struct osmo_trxc_client *client = client_alloc();

	printf("=== %s ===\n", __func__);

	osmo_trxc_client_poweron(client, &rsp_cb, "poweron");
	rx_rsp(client, "RSP POWERON 0");
	/* a duplicate response (e.g. caused by retransmission) is discarded */
	rx_rsp(client, "RSP POWERON 0");
	/* an unexpected response is reported */
	rx_rsp(client, "RSP POWEROFF 0");

	osmo_trxc_client_free(client);
}

static void test_retrans(void)
{
	struct osmo_trxc_client *client = client_alloc();

	printf("=== %s ===\n", __func__);

	osmo_trxc_client_poweron(client, &rsp_cb, "poweron");
	/* no response: the command is retransmitted (default: every 2 s) */
	fake_time_passes(2);
	fake_time_passes(2);
	rx_rsp(client, "RSP POWERON 0");
	/* no pending commands anymore, the timer shall be inactive */
	fake_time_passes(10);

	osmo_trxc_client_free(client);
}

static void test_max_retrans(void)
{
	struct osmo_trxc_client *client = client_alloc();

	printf("=== %s ===\n", __func__);

	/* give up after 3 retransmissions (like trxcon does) */
	osmo_trxc_client_set_max_retrans(client, 3);

	osmo_trxc_client_poweron(client, &rsp_cb, "poweron");
	/* no response: 3 retransmissions, then fatal_error escalation */
	for (unsigned int i = 0; i < 5; i++)
		fake_time_passes(2);

	osmo_trxc_client_free(client);

	/* a response resets the retransmission counter */
	client = client_alloc();
	osmo_trxc_client_set_max_retrans(client, 3);

	osmo_trxc_client_poweron(client, &rsp_cb, "poweron");
	fake_time_passes(2);
	fake_time_passes(2);
	rx_rsp(client, "RSP POWERON 0");
	osmo_trxc_client_rxtune(client, 890000, &rsp_cb, "rxtune");
	/* the previous 2 retransmissions shall not count for RXTUNE */
	fake_time_passes(2);
	fake_time_passes(2);
	rx_rsp(client, "RSP RXTUNE 0 890000");

	osmo_trxc_client_free(client);
}

static int rsp_retry_cb(struct osmo_trxc_client *client,
			const struct osmo_trxc_msg *rsp, void *cb_data)
{
	printf("rsp_retry_cb: '%s'\n", osmo_trxc_msg_name(rsp));

	/* POWERON failed: re-send it after 5 seconds */
	if (rsp->status != 0)
		return 5;
	return 0;
}

static void test_rsp_cb_retry(void)
{
	struct osmo_trxc_client *client = client_alloc();

	printf("=== %s ===\n", __func__);

	osmo_trxc_client_poweron(client, &rsp_retry_cb, NULL);
	/* transceiver is not ready yet, the call-back requests a retry */
	rx_rsp(client, "RSP POWERON 1");
	fake_time_passes(5);
	rx_rsp(client, "RSP POWERON 0");

	osmo_trxc_client_free(client);
}

static void test_fatal_error(void)
{
	struct osmo_trxc_client *client = client_alloc();

	printf("=== %s ===\n", __func__);

	/* no rsp_cb given: a NACKed critical command is escalated */
	osmo_trxc_client_rxtune(client, 890000, NULL, NULL);
	rx_rsp(client, "RSP RXTUNE 1 890000");

	osmo_trxc_client_free(client);
}

static void setformat_cb(struct osmo_trxc_client *client,
			 uint8_t ver_use, void *cb_data)
{
	printf("setformat_cb: ver_use=%u\n", ver_use);
}

static void test_negotiate_format(void)
{
	struct osmo_trxc_client *client = client_alloc();

	printf("=== %s ===\n", __func__);

	/* case a) the transceiver confirms the requested version */
	osmo_trxc_client_negotiate_format(client, 2, &setformat_cb, NULL);
	rx_rsp(client, "RSP SETFORMAT 2 2");

	/* case b) the transceiver indicates a lower version */
	osmo_trxc_client_negotiate_format(client, 2, &setformat_cb, NULL);
	rx_rsp(client, "RSP SETFORMAT 1 2");

	/* case c) an old transceiver rejects the command ('RSP ERR 1') */
	osmo_trxc_client_negotiate_format(client, 2, &setformat_cb, NULL);
	rx_rsp(client, "RSP ERR 1");

	/* case d) the transceiver indicates an out of range version */
	osmo_trxc_client_negotiate_format(client, 2, &setformat_cb, NULL);
	rx_rsp(client, "RSP SETFORMAT 5 2");

	osmo_trxc_client_free(client);
}

static void test_flush(void)
{
	struct osmo_trxc_client *client = client_alloc();

	printf("=== %s ===\n", __func__);

	osmo_trxc_client_rxtune(client, 890000, &rsp_cb, "rxtune");
	osmo_trxc_client_txtune(client, 935000, &rsp_cb, "txtune");
	osmo_trxc_client_flush(client);

	/* a late response finds no pending command */
	rx_rsp(client, "RSP RXTUNE 0 890000");
	/* the retransmit timer shall be inactive */
	fake_time_passes(10);

	osmo_trxc_client_free(client);
}

static int rsp_flush_cb(struct osmo_trxc_client *client,
			const struct osmo_trxc_msg *rsp, void *cb_data)
{
	printf("rsp_flush_cb: '%s', flushing the queue\n", osmo_trxc_msg_name(rsp));
	osmo_trxc_client_flush(client);
	return 0;
}

static void test_flush_in_rsp_cb(void)
{
	struct osmo_trxc_client *client = client_alloc();

	printf("=== %s ===\n", __func__);

	/* flushing the queue from within a response call-back */
	osmo_trxc_client_poweroff(client, &rsp_flush_cb, NULL);
	osmo_trxc_client_rxtune(client, 890000, &rsp_cb, "rxtune");
	rx_rsp(client, "RSP POWEROFF 0");
	fake_time_passes(10);

	osmo_trxc_client_free(client);
}

static void test_malformed(void)
{
	struct osmo_trxc_client *client = client_alloc();

	printf("=== %s ===\n", __func__);

	rx_rsp(client, "MALFORMED MESSAGE");
	rx_rsp(client, "IND CLOCK 1234"); /* not a RSP */

	osmo_trxc_client_free(client);
}

int main(int argc, char **argv)
{
	test_ctx = talloc_named_const(NULL, 0, "trxc_client_test");
	osmo_init_logging2(test_ctx, NULL);
	log_set_use_color(osmo_stderr_target, 0);
	log_set_print_timestamp(osmo_stderr_target, 0);
	log_set_print_filename2(osmo_stderr_target, LOG_FILENAME_NONE);
	log_set_print_category(osmo_stderr_target, 1);
	log_set_print_category_hex(osmo_stderr_target, 0);
	log_set_print_level(osmo_stderr_target, 1);
	log_set_category_filter(osmo_stderr_target, DLGLOBAL, 1, LOGL_DEBUG);

	/* take control over the clock */
	osmo_gettimeofday_override = true;
	osmo_gettimeofday_override_time = (struct timeval){ 1000000, 0 };

	test_basic();
	test_queueing();
	test_dup_rsp();
	test_retrans();
	test_max_retrans();
	test_rsp_cb_retry();
	test_fatal_error();
	test_negotiate_format();
	test_flush();
	test_flush_in_rsp_cb();
	test_malformed();

	printf("Done\n");
	return 0;
}
