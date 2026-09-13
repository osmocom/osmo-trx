/*! \file tests/trx_ep_test.c
 * Regression test for the TRX endpoint module: two endpoints (TRX and BTS)
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

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <osmocom/core/application.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/select.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>

#include <osmocom/trx/ep.h>

#define TEST_BASE_PORT 16700
#define TEST_PROMISC_BASE_PORT 16720

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

static void ep_closed_cb(struct osmo_trx_ep *ep)
{
	printf("%s: closed_cb()\n", ep_label(ep));
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

/* Allocate and pre-configure an endpoint */
static struct osmo_trx_ep *ep_alloc(const char *label,
				    enum osmo_trx_ep_mode mode)
{
	struct osmo_trx_ep *ep = osmo_trx_ep_alloc(test_ctx, 1);

	OSMO_ASSERT(ep != NULL);
	OSMO_ASSERT(osmo_trx_ep_is_open(ep) == false);

	osmo_trx_ep_set_name(ep, "ep_%s", label);
	osmo_trx_ep_set_priv(ep, (void *)label);
	osmo_trx_ep_set_mode(ep, mode);
	osmo_trx_ep_set_laddr(ep, "127.0.0.1");
	osmo_trx_ep_set_raddr(ep, "127.0.0.1");
	osmo_trx_ep_set_base_port(ep, TEST_BASE_PORT);
	osmo_trx_ep_set_clock_socket(ep, true);

	return ep;
}

static void ep_set_num_chans(struct osmo_trx_ep *ep,
			     unsigned int num_chans)
{
	/* ep_alloc() allocates only one channel */
	OSMO_ASSERT(osmo_trx_ep_get_num_chans(ep) == 1);
	OSMO_ASSERT(osmo_trx_ep_set_num_chans(ep, num_chans) == 0);
	OSMO_ASSERT(osmo_trx_ep_set_num_chans(ep, 0) == -EINVAL);
	OSMO_ASSERT(osmo_trx_ep_get_num_chans(ep) == num_chans);
}

static void ep_open(struct osmo_trx_ep *ep)
{
	OSMO_ASSERT(osmo_trx_ep_is_open(ep) == false);
	OSMO_ASSERT(osmo_trx_ep_open(ep) == 0);
	OSMO_ASSERT(osmo_trx_ep_is_open(ep) == true);

	/* subsequent osmo_trx_ep_open() is expected to fail */
	OSMO_ASSERT(osmo_trx_ep_open(ep) == -EALREADY);
	/* osmo_trx_ep_set_num_chans() is expected to fail after open() */
	OSMO_ASSERT(osmo_trx_ep_set_num_chans(ep, 16) == -EBUSY);
}

static void ep_close_free(struct osmo_trx_ep *ep)
{
	osmo_trx_ep_close(ep);
	OSMO_ASSERT(osmo_trx_ep_is_open(ep) == false);
	OSMO_ASSERT(osmo_trx_ep_is_closing(ep) == false);
	osmo_trx_ep_free(ep);
}

static void test_clck_ctrl(void)
{
	struct osmo_trx_ep *ep_trx = ep_alloc("trx", OSMO_TRX_EP_MODE_TRX);
	struct osmo_trx_ep *ep_bts = ep_alloc("bts", OSMO_TRX_EP_MODE_L1);

	printf("=== %s(): starting testcase ===\n", __func__);

	ep_set_num_chans(ep_trx, 2);
	ep_set_num_chans(ep_bts, 2);

	ep_open(ep_trx);
	ep_open(ep_bts);

	printf("=== %s(): clock indication (TRX -> BTS) ===\n", __func__);
	osmo_trx_ep_send_clck_ind(ep_trx, 402312);
	flush_io();

	static const struct osmo_trxc_msg cmd_poweron = {
		.type = OSMO_TRXC_MT_CMD,
		.cmd = OSMO_TRXC_CMD_POWERON,
	};

	static const struct osmo_trxc_msg cmd_rfmute = {
		.type = OSMO_TRXC_MT_CMD,
		.cmd = OSMO_TRXC_CMD_RFMUTE,
		.params = "1",
	};

	static const struct osmo_trxc_msg cmd_poweroff = {
		.type = OSMO_TRXC_MT_CMD,
		.cmd = OSMO_TRXC_CMD_POWEROFF,
	};

	printf("=== %s(): CMD POWERON ===\n", __func__);
	osmo_trx_ep_send_ctrl_msg(ep_bts, 0, &cmd_poweron);
	flush_io();

	printf("=== %s(): CMD RFMUTE ===\n", __func__);
	osmo_trx_ep_send_ctrl_msg(ep_bts, 0, &cmd_rfmute);
	osmo_trx_ep_send_ctrl_msg(ep_bts, 1, &cmd_rfmute);
	flush_io();

	printf("=== %s(): CMD POWEROFF ===\n", __func__);
	osmo_trx_ep_send_ctrl_msg(ep_bts, 0, &cmd_poweroff);
	flush_io();

	ep_close_free(ep_trx);
	ep_close_free(ep_bts);
}

static void test_burst_req_ind(void)
{
	struct osmo_trx_ep *ep_trx = ep_alloc("trx", OSMO_TRX_EP_MODE_TRX);
	struct osmo_trx_ep *ep_bts = ep_alloc("bts", OSMO_TRX_EP_MODE_L1);
	struct osmo_trxd_burst_req br;
	struct osmo_trxd_burst_ind bi;

	printf("=== %s(): starting testcase ===\n", __func__);

	ep_set_num_chans(ep_trx, 2);
	ep_set_num_chans(ep_bts, 2);

	ep_open(ep_trx);
	ep_open(ep_bts);

	printf("=== %s(): BURST.req, TRXDv0 (BTS -> TRX) ===\n", __func__);
	fill_burst_req(&br, 100000);
	osmo_trx_ep_send_burst_req(ep_bts, 0, &br);
	flush_io();

	printf("=== %s(): BURST.ind, TRXDv0 (TRX -> BTS), another channel ===\n", __func__);
	fill_burst_ind(&bi, 100005);
	osmo_trx_ep_send_burst_ind(ep_trx, 1, &bi);
	flush_io();

	printf("=== %s(): switch chan 0 to TRXDv2 ===\n", __func__);
	OSMO_ASSERT(osmo_trx_ep_set_pdu_ver(ep_bts, 0, 2) == 0);
	OSMO_ASSERT(osmo_trx_ep_set_pdu_ver(ep_trx, 0, 2) == 0);

	printf("=== %s(): BURST.req batch, TRXDv2 (BTS -> TRX) ===\n", __func__);
	/* an empty batch cannot be flushed */
	OSMO_ASSERT(osmo_trx_ep_send_burst_fin(ep_bts, 0) == -ENOMSG);
	/* accumulate two PDUs, then flush them */
	fill_burst_req(&br, 200000);
	br.tn = 1;
	osmo_trx_ep_send_burst_req(ep_bts, 0, &br);
	br.tn = 2;
	osmo_trx_ep_send_burst_req(ep_bts, 0, &br);
	flush_io(); /* nothing shall be delivered yet */
	OSMO_ASSERT(osmo_trx_ep_send_burst_fin(ep_bts, 0) == 0);
	printf("BURST.req batch flush\n");
	flush_io();

	printf("=== %s(): BURST.ind batch, TRXDv2 with NOPE (TRX -> BTS) ===\n", __func__);
	/* an empty batch cannot be flushed */
	OSMO_ASSERT(osmo_trx_ep_send_burst_fin(ep_trx, 0) == -ENOMSG);
	/* accumulate two PDUs, then flush them */
	fill_burst_ind(&bi, 200005);
	bi.flags = OSMO_TRXD_F_NOPE_IND | OSMO_TRXD_F_CI_CB;
	bi.burst_len = 0;
	osmo_trx_ep_send_burst_ind(ep_trx, 0, &bi);
	fill_burst_ind(&bi, 200005);
	bi.tn = 6;
	osmo_trx_ep_send_burst_ind(ep_trx, 0, &bi);
	flush_io(); /* nothing shall be delivered yet */
	OSMO_ASSERT(osmo_trx_ep_send_burst_fin(ep_trx, 0) == 0);
	printf("BURST.ind batch flush\n");
	flush_io();

	ep_close_free(ep_trx);
	ep_close_free(ep_bts);
}

static void test_ctrl_close_flush(bool do_free)
{
	struct osmo_trx_ep *ep_trx = ep_alloc("trx", OSMO_TRX_EP_MODE_TRX);
	struct osmo_trx_ep *ep_bts = ep_alloc("bts", OSMO_TRX_EP_MODE_L1);

	printf("=== %s(do_free=%d): starting testcase ===\n", __func__, (int)do_free);

	ep_set_num_chans(ep_trx, 3);
	ep_set_num_chans(ep_bts, 3);

	ep_open(ep_trx);
	ep_open(ep_bts);

	static const struct osmo_trxc_msg cmd_rfmute = {
		.type = OSMO_TRXC_MT_CMD,
		.cmd = OSMO_TRXC_CMD_RFMUTE,
		.params = "1",
	};

	static const struct osmo_trxc_msg cmd_poweroff = {
		.type = OSMO_TRXC_MT_CMD,
		.cmd = OSMO_TRXC_CMD_POWEROFF,
	};

	printf("=== %s(): TRXC CMDs sent right before osmo_trx_ep_close() (BTS -> TRX) ===\n", __func__);
	osmo_trx_ep_send_ctrl_msg(ep_bts, 1, &cmd_rfmute);
	osmo_trx_ep_send_ctrl_msg(ep_bts, 0, &cmd_poweroff);

	osmo_trx_ep_set_closed_cb(ep_bts, ep_closed_cb);
	osmo_trx_ep_close(ep_bts);
	OSMO_ASSERT(osmo_trx_ep_is_open(ep_bts) == false);
	OSMO_ASSERT(osmo_trx_ep_is_closing(ep_bts) == true);

	/* osmo_trx_ep_open() is expected to fail while closing */
	OSMO_ASSERT(osmo_trx_ep_open(ep_bts) == -EBUSY);
	/* osmo_trx_ep_set_num_chans() is expected to fail too */
	OSMO_ASSERT(osmo_trx_ep_set_num_chans(ep_bts, 16) == -EBUSY);

	if (do_free) {
		OSMO_ASSERT(talloc_parent(ep_bts) == test_ctx);
		osmo_trx_ep_free(ep_bts); /* osmo_trx_ep_free() postpones the actual free() */
		OSMO_ASSERT(talloc_parent(ep_bts) == OTC_GLOBAL);
		OSMO_ASSERT(osmo_trx_ep_is_closing(ep_bts) == true);
		OSMO_ASSERT(osmo_trx_ep_is_open(ep_bts) == false);
		flush_io(); /* after flushing, the ep is finally free()ed! */
	} else {
		flush_io();
		OSMO_ASSERT(osmo_trx_ep_is_closing(ep_bts) == false);
		OSMO_ASSERT(osmo_trx_ep_is_open(ep_bts) == false);
		ep_close_free(ep_bts);
	}

	ep_close_free(ep_trx);
}

/* osmo_trx_ep_{get,set}_ctrl_promisc(): with promisc enabled, a channel's
 * ctrl socket is left unconnected (bind-only), so it accepts a CMD from any
 * peer, not just the configured raddr, and replies to whoever actually sent
 * it - e.g. a test tool injecting extra TRXC commands from its own socket. */
static void test_ctrl_promisc(void)
{
	struct sockaddr_in dst = { .sin_family = AF_INET };
	struct sockaddr_in from;
	socklen_t from_len;
	struct osmo_trx_ep *ep;
	int fd, rc, len;

	printf("=== %s(): starting testcase ===\n", __func__);

	ep = ep_alloc("promisc", OSMO_TRX_EP_MODE_TRX);
	osmo_trx_ep_set_base_port(ep, TEST_PROMISC_BASE_PORT);

	OSMO_ASSERT(osmo_trx_ep_get_ctrl_promisc(ep) == false);
	OSMO_ASSERT(osmo_trx_ep_set_ctrl_promisc(ep, true) == 0);
	OSMO_ASSERT(osmo_trx_ep_get_ctrl_promisc(ep) == true);

	ep_open(ep);

	/* config setters, including this one, are expected to fail once open */
	OSMO_ASSERT(osmo_trx_ep_set_ctrl_promisc(ep, false) == -EBUSY);

	/* chan 0 ctrl; source port of the foreign peer is left to the kernel
	 * (no bind() before sendto() below) */
	dst.sin_port = htons(TEST_PROMISC_BASE_PORT + 1);
	OSMO_ASSERT(inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr) == 1);

	fd = socket(AF_INET, SOCK_DGRAM, 0);
	OSMO_ASSERT(fd >= 0);

	char buf[OSMO_TRXC_MSG_BUF_SIZE];
	static const struct osmo_trxc_msg cmd_poweron = {
		.type = OSMO_TRXC_MT_CMD,
		.cmd = OSMO_TRXC_CMD_POWERON,
	};

	len = osmo_trxc_msg_build(buf, sizeof(buf), &cmd_poweron);
	OSMO_ASSERT(len > 0);
	rc = sendto(fd, buf, len, 0, (const struct sockaddr *)&dst, sizeof(dst));
	OSMO_ASSERT(rc == len);

	flush_io();

	/* the RSP must come back to us (the foreign sender), not to raddr */
	from_len = sizeof(from);
	rc = recvfrom(fd, buf, sizeof(buf) - 1, MSG_DONTWAIT,
		      (struct sockaddr *)&from, &from_len);
	OSMO_ASSERT(rc > 0);
	buf[rc] = '\0';
	OSMO_ASSERT(from.sin_addr.s_addr == dst.sin_addr.s_addr);
	printf("foreign rx: '%s'\n", buf);

	close(fd);
	ep_close_free(ep);
}

int main(int argc, char **argv)
{
	test_ctx = talloc_named_const(NULL, 0, "trx_ep_test");
	osmo_init_logging2(test_ctx, NULL);
	log_set_use_color(osmo_stderr_target, 0);
	log_set_print_timestamp(osmo_stderr_target, 0);
	log_set_print_filename2(osmo_stderr_target, LOG_FILENAME_NONE);
	log_set_print_category(osmo_stderr_target, 1);
	log_set_print_category_hex(osmo_stderr_target, 0);
	log_set_print_level(osmo_stderr_target, 1);
	log_set_category_filter(osmo_stderr_target, DLGLOBAL, 1, LOGL_DEBUG);

	test_clck_ctrl();
	test_burst_req_ind();
	test_ctrl_close_flush(false);
	test_ctrl_close_flush(true);

	test_ctrl_promisc();

	printf("Done\n");
	return 0;
}
