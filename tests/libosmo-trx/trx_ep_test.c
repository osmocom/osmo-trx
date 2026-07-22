/*! \file tests/trx_ep_test.c
 * Regression test for the TRX endpoint module: two endpoints (L1 and TRX)
 * talking to each other over UDP sockets on localhost. */

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
#include <osmocom/core/select.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>

#include <osmocom/trx/ep.h>

#define TEST_BASE_PORT 16700

static void *test_ctx = NULL;

/* let the event loop deliver everything that is in flight */
static void flush_io(void)
{
	for (unsigned int i = 0; i < 16; i++)
		osmo_select_main(1);
}

static const char *ep_label(const struct osmo_trx_ep *ep)
{
	return (const char *)osmo_trx_ep_get_priv(ep);
}

/* RX handlers (strong symbols, overriding the library's weak stubs) */
void osmo_trx_ep_rx_clck_ind(struct osmo_trx_ep *ep, uint32_t fn)
{
	printf("%s: rx_clck_ind: fn=%u\n", ep_label(ep), fn);
}

void osmo_trx_ep_rx_ctrl_msg(struct osmo_trx_ep *ep, unsigned int chan,
			     const struct osmo_trxc_msg *msg)
{
	printf("%s: rx_ctrl_msg(chan=%u): '%s'\n",
	       ep_label(ep), chan, osmo_trxc_msg_name(msg));

	/* the TRX side acknowledges all commands */
	if (msg->type == OSMO_TRXC_MT_CMD) {
		struct osmo_trxc_msg rsp = {
			.type = OSMO_TRXC_MT_RSP,
			.status = 0,
		};

		OSMO_STRLCPY_ARRAY(rsp.cmd, msg->cmd);
		OSMO_STRLCPY_ARRAY(rsp.params, msg->params);
		osmo_trx_ep_send_ctrl_msg(ep, chan, &rsp);
	}
}

void osmo_trx_ep_rx_burst_ind(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxd_burst_ind *bi)
{
	printf("%s: rx_burst_ind(chan=%u): %s\n",
	       ep_label(ep), chan, osmo_trxd_burst_ind_name(bi));
}

void osmo_trx_ep_rx_burst_req(struct osmo_trx_ep *ep, unsigned int chan,
			      const struct osmo_trxd_burst_req *br)
{
	printf("%s: rx_burst_req(chan=%u): %s\n",
	       ep_label(ep), chan, osmo_trxd_burst_req_name(br));
}

static void fill_burst_req(struct osmo_trxd_burst_req *br, uint32_t fn)
{
	*br = (struct osmo_trxd_burst_req){
		.flags = OSMO_TRXD_F_MOD_TYPE | OSMO_TRXD_F_TS_INFO,
		.fn = fn,
		.tn = 3,
		.att = 10,
		.mod = OSMO_TRXD_MOD_T_GMSK,
		.tsc = 7,
		.burst_len = OSMO_TRXD_BURST_LEN_GMSK,
	};

	for (size_t i = 0; i < br->burst_len; i++)
		br->burst[i] = (i & 1);
}

static void fill_burst_ind(struct osmo_trxd_burst_ind *bi, uint32_t fn)
{
	*bi = (struct osmo_trxd_burst_ind){
		.flags = OSMO_TRXD_F_MOD_TYPE | OSMO_TRXD_F_TS_INFO | OSMO_TRXD_F_CI_CB,
		.fn = fn,
		.tn = 5,
		.toa256 = -512,
		.rssi = -63,
		.mod = OSMO_TRXD_MOD_T_GMSK,
		.tsc = 7,
		.ci_cb = -150,
		.burst_len = OSMO_TRXD_BURST_LEN_GMSK,
	};

	for (size_t i = 0; i < bi->burst_len; i++)
		bi->burst[i] = (i & 1) ? -100 : 100;
}

int main(int argc, char **argv)
{
	struct osmo_trx_ep *ep_l1, *ep_trx;
	struct osmo_trxd_burst_req br;
	struct osmo_trxd_burst_ind bi;
	int rc;

	test_ctx = talloc_named_const(NULL, 0, "trx_ep_test");
	osmo_init_logging2(test_ctx, NULL);
	log_set_use_color(osmo_stderr_target, 0);
	log_set_print_timestamp(osmo_stderr_target, 0);
	log_set_print_filename2(osmo_stderr_target, LOG_FILENAME_NONE);
	log_set_print_category(osmo_stderr_target, 1);
	log_set_print_category_hex(osmo_stderr_target, 0);
	log_set_print_level(osmo_stderr_target, 1);
	log_set_category_filter(osmo_stderr_target, DLGLOBAL, 1, LOGL_INFO);

	const struct osmo_trx_ep_cfg cfg_trx = {
		.mode = OSMO_TRX_EP_MODE_TRX,
		.laddr = "127.0.0.1",
		.raddr = "127.0.0.1",
		.base_port = TEST_BASE_PORT,
		.num_chans = 2,
		.clock_socket = true,
	};
	const struct osmo_trx_ep_cfg cfg_l1 = {
		.mode = OSMO_TRX_EP_MODE_L1,
		.laddr = "127.0.0.1",
		.raddr = "127.0.0.1",
		.base_port = TEST_BASE_PORT,
		.num_chans = 2,
		.clock_socket = true,
	};

	ep_trx = osmo_trx_ep_alloc(test_ctx, &cfg_trx, "trx");
	ep_l1 = osmo_trx_ep_alloc(test_ctx, &cfg_l1, "l1");
	OSMO_ASSERT(ep_trx != NULL && ep_l1 != NULL);
	osmo_trx_ep_set_name(ep_trx, "ep_%s", "trx");
	osmo_trx_ep_set_name(ep_l1, "ep_%s", "l1");

	OSMO_ASSERT(osmo_trx_ep_open(ep_trx) == 0);
	OSMO_ASSERT(osmo_trx_ep_open(ep_l1) == 0);

	printf("=== clock indication (TRX -> L1) ===\n");
	osmo_trx_ep_send_clck_ind(ep_trx, 402312);
	flush_io();

	printf("=== ctrl: CMD (L1 -> TRX), RSP (TRX -> L1) ===\n");
	osmo_trx_ep_send_ctrl_msg(ep_l1, 0, &(struct osmo_trxc_msg){
		.type = OSMO_TRXC_MT_CMD,
		.cmd = "POWERON",
	});
	flush_io();

	printf("=== burst req, TRXDv0 (L1 -> TRX) ===\n");
	fill_burst_req(&br, 100000);
	osmo_trx_ep_send_burst_req(ep_l1, 0, &br);
	flush_io();

	printf("=== burst ind, TRXDv0 (TRX -> L1), another channel ===\n");
	fill_burst_ind(&bi, 100005);
	osmo_trx_ep_send_burst_ind(ep_trx, 1, &bi);
	flush_io();

	printf("=== switch chan 0 to TRXDv2 ===\n");
	osmo_trx_ep_set_pdu_ver(ep_l1, 0, 2);
	osmo_trx_ep_set_pdu_ver(ep_trx, 0, 2);

	printf("=== burst req batch, TRXDv2 (L1 -> TRX) ===\n");
	/* an empty batch cannot be flushed */
	rc = osmo_trx_ep_send_burst_req(ep_l1, 0, NULL);
	printf("empty batch breaker: rc=%d\n", rc);
	/* accumulate two PDUs, then flush them with the breaker */
	fill_burst_req(&br, 200000);
	br.tn = 1;
	osmo_trx_ep_send_burst_req(ep_l1, 0, &br);
	br.tn = 2;
	osmo_trx_ep_send_burst_req(ep_l1, 0, &br);
	flush_io(); /* nothing shall be delivered yet */
	rc = osmo_trx_ep_send_burst_req(ep_l1, 0, NULL);
	printf("batch breaker: rc=%d\n", rc);
	flush_io();

	printf("=== burst ind, TRXDv2 with NOPE (TRX -> L1) ===\n");
	fill_burst_ind(&bi, 200005);
	bi.flags = OSMO_TRXD_F_NOPE_IND | OSMO_TRXD_F_CI_CB;
	bi.burst_len = 0;
	osmo_trx_ep_send_burst_ind(ep_trx, 0, &bi);
	flush_io();

	osmo_trx_ep_free(ep_l1);
	osmo_trx_ep_free(ep_trx);

	printf("Done\n");
	return 0;
}
