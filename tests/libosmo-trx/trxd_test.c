/*! \file tests/trxd_test.c
 * Regression test for the TRXD PDU codec. */

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
#include <errno.h>

#include <osmocom/core/msgb.h>
#include <osmocom/core/utils.h>

#include <osmocom/trx/trxd.h>
#include <osmocom/trx/trxd_int.h>

static void fill_burst_ind(struct osmo_trxd_burst_ind *bi, size_t burst_len)
{
	*bi = (struct osmo_trxd_burst_ind){
		.flags = OSMO_TRXD_F_MOD_TYPE
		       | OSMO_TRXD_F_TS_INFO
		       | OSMO_TRXD_F_CI_CB,
		.fn = 1234567,
		.tn = 5,
		.toa256 = -512,
		.rssi = -63,
		.mod = (burst_len == OSMO_TRXD_BURST_LEN_8PSK) ?
				OSMO_TRXD_MOD_T_8PSK : OSMO_TRXD_MOD_T_GMSK,
		.tsc_set = 1,
		.tsc = 7,
		.ci_cb = -150,
		.burst_len = burst_len,
	};

	for (size_t i = 0; i < burst_len; i++)
		bi->burst[i] = (i & 1) ? -100 : 100;
}

static void fill_burst_req(struct osmo_trxd_burst_req *br, size_t burst_len)
{
	*br = (struct osmo_trxd_burst_req){
		.flags = OSMO_TRXD_F_MOD_TYPE
		       | OSMO_TRXD_F_TS_INFO,
		.fn = 2654321,
		.tn = 2,
		.att = 10,
		.mod = (burst_len == OSMO_TRXD_BURST_LEN_8PSK) ?
				OSMO_TRXD_MOD_T_8PSK : OSMO_TRXD_MOD_T_GMSK,
		.tsc_set = 0,
		.tsc = 3,
		.burst_len = burst_len,
	};

	for (size_t i = 0; i < burst_len; i++)
		br->burst[i] = (i & 1);
}

static void test_burst_ind(uint8_t pdu_ver, size_t burst_len)
{
	struct osmo_trxd_parse_state st;
	struct osmo_trxd_burst_ind bi, bi2;
	struct msgb *msg = msgb_alloc(4096, "ind");
	int rc;

	printf("=== %s(v%u, burst_len=%zu) ===\n",
	       __func__, pdu_ver, burst_len);

	fill_burst_ind(&bi, burst_len);
	rc = osmo_trxd_burst_ind_build(msg, pdu_ver, &bi);
	OSMO_ASSERT(rc == 0);
	osmo_trxd_build_fin(msg, pdu_ver);

	printf("build: %s\n", osmo_trxd_burst_ind_name(&bi));
	printf("datagram (%u bytes): %s...\n", msgb_length(msg),
	       osmo_hexdump_nospc(msgb_data(msg), OSMO_MIN(16, msgb_length(msg))));

	osmo_trxd_parse_state_init(&st);
	rc = osmo_trxd_burst_ind_parse(&st, &bi2, msgb_data(msg), msgb_length(msg));
	OSMO_ASSERT(rc == (int)msgb_length(msg));
	printf("parse: %s\n", osmo_trxd_burst_ind_name(&bi2));

	OSMO_ASSERT(bi2.fn == bi.fn && bi2.tn == bi.tn);
	OSMO_ASSERT(bi2.rssi == bi.rssi && bi2.toa256 == bi.toa256);
	OSMO_ASSERT(bi2.burst_len == bi.burst_len);
	OSMO_ASSERT(memcmp(bi2.burst, bi.burst, bi.burst_len) == 0);
	if (pdu_ver >= 1) {
		OSMO_ASSERT(bi2.mod == bi.mod);
		OSMO_ASSERT(bi2.tsc_set == bi.tsc_set && bi2.tsc == bi.tsc);
		OSMO_ASSERT(bi2.ci_cb == bi.ci_cb);
	}

	msgb_free(msg);
}

static void test_burst_ind_nope(uint8_t pdu_ver)
{
	struct osmo_trxd_parse_state st;
	struct osmo_trxd_burst_ind bi, bi2;
	struct msgb *msg = msgb_alloc(4096, "nope");
	int rc;

	printf("=== %s(v%u) ===\n", __func__, pdu_ver);

	fill_burst_ind(&bi, 0);
	bi.flags = OSMO_TRXD_F_NOPE_IND | OSMO_TRXD_F_CI_CB;

	rc = osmo_trxd_burst_ind_build(msg, pdu_ver, &bi);
	OSMO_ASSERT(rc == 0);
	osmo_trxd_build_fin(msg, pdu_ver);
	printf("build: %s\n", osmo_trxd_burst_ind_name(&bi));

	osmo_trxd_parse_state_init(&st);
	rc = osmo_trxd_burst_ind_parse(&st, &bi2, msgb_data(msg), msgb_length(msg));
	OSMO_ASSERT(rc == (int)msgb_length(msg));
	printf("parse: %s\n", osmo_trxd_burst_ind_name(&bi2));

	OSMO_ASSERT(bi2.flags & OSMO_TRXD_F_NOPE_IND);
	OSMO_ASSERT(bi2.burst_len == 0);

	msgb_free(msg);
}

static void test_burst_ind_v0_legacy_padding(void)
{
	struct osmo_trxd_parse_state st;
	struct osmo_trxd_burst_ind bi, bi2;
	struct msgb *msg = msgb_alloc(4096, "legacy");
	int rc;

	printf("=== %s ===\n", __func__);

	fill_burst_ind(&bi, OSMO_TRXD_BURST_LEN_GMSK);
	rc = osmo_trxd_burst_ind_build(msg, 0, &bi);
	OSMO_ASSERT(rc == 0);

	/* a legacy transceiver may append two garbage bytes */
	msgb_put_u16(msg, 0xdead);

	osmo_trxd_parse_state_init(&st);
	rc = osmo_trxd_burst_ind_parse(&st, &bi2, msgb_data(msg), msgb_length(msg));
	printf("parse: rc=%d (%u bytes incl. padding)\n", rc, msgb_length(msg));
	OSMO_ASSERT(rc == (int)msgb_length(msg));
	OSMO_ASSERT(bi2.burst_len == OSMO_TRXD_BURST_LEN_GMSK);

	msgb_free(msg);
}

static void test_burst_ind_batch(uint8_t pdu_ver)
{
	struct osmo_trxd_parse_state st;
	struct osmo_trxd_burst_ind bi, bi2;
	struct msgb *msg;
	const uint8_t *buf;
	size_t buf_len;
	unsigned int i;
	int rc;

	/* PDU batching is only supported by TRXDv2 and higher */
	if (pdu_ver < 2)
		return;

	printf("=== %s(v%u) ===\n", __func__, pdu_ver);

	msg = msgb_alloc(4096, "batch");

	/* batch a normal burst + a NOPE.ind for another timeslot */
	fill_burst_ind(&bi, OSMO_TRXD_BURST_LEN_GMSK);
	bi.trx_num = 1;
	bi.flags |= OSMO_TRXD_F_TRX_NUM;
	rc = osmo_trxd_burst_ind_build(msg, pdu_ver, &bi);
	OSMO_ASSERT(rc == 0);

	bi.tn = 6;
	bi.flags = OSMO_TRXD_F_NOPE_IND | OSMO_TRXD_F_TRX_NUM;
	bi.burst_len = 0;
	rc = osmo_trxd_burst_ind_build(msg, pdu_ver, &bi);
	OSMO_ASSERT(rc == 0);

	osmo_trxd_build_fin(msg, pdu_ver);
	printf("datagram (%u bytes)\n", msgb_length(msg));

	osmo_trxd_parse_state_init(&st);
	buf = msgb_data(msg);
	buf_len = msgb_length(msg);
	for (i = 0; buf_len > 0; i++) {
		rc = osmo_trxd_burst_ind_parse(&st, &bi2, buf, buf_len);
		OSMO_ASSERT(rc > 0);
		printf("parse[%u]: %s\n", i, osmo_trxd_burst_ind_name(&bi2));
		/* the batched PDU inherits the TDMA fn of the first one */
		OSMO_ASSERT(bi2.fn == 1234567);
		buf += rc;
		buf_len -= rc;
		/* BATCH.ind shall be set on all PDUs but the last one */
		OSMO_ASSERT(!!(bi2.flags & OSMO_TRXD_F_BATCH_IND) == (buf_len > 0));
	}
	OSMO_ASSERT(i == 2);

	msgb_free(msg);
}

static void test_burst_req(uint8_t pdu_ver, size_t burst_len)
{
	struct osmo_trxd_parse_state st;
	struct osmo_trxd_burst_req br, br2;
	struct msgb *msg = msgb_alloc(4096, "req");
	int rc;

	printf("=== %s(v%u, burst_len=%zu) ===\n", __func__, pdu_ver, burst_len);

	fill_burst_req(&br, burst_len);
	rc = osmo_trxd_burst_req_build(msg, pdu_ver, &br);
	OSMO_ASSERT(rc == 0);
	osmo_trxd_build_fin(msg, pdu_ver);

	printf("build: %s\n", osmo_trxd_burst_req_name(&br));
	printf("datagram (%u bytes): %s...\n", msgb_length(msg),
	       osmo_hexdump_nospc(msgb_data(msg), OSMO_MIN(16, msgb_length(msg))));

	osmo_trxd_parse_state_init(&st);
	rc = osmo_trxd_burst_req_parse(&st, &br2, msgb_data(msg), msgb_length(msg));
	OSMO_ASSERT(rc == (int)msgb_length(msg));
	printf("parse: %s\n", osmo_trxd_burst_req_name(&br2));

	OSMO_ASSERT(br2.fn == br.fn && br2.tn == br.tn);
	OSMO_ASSERT(br2.att == br.att);
	OSMO_ASSERT(br2.burst_len == br.burst_len);
	OSMO_ASSERT(memcmp(br2.burst, br.burst, br.burst_len) == 0);
	if (pdu_ver >= 2) {
		OSMO_ASSERT(br2.mod == br.mod);
		OSMO_ASSERT(br2.tsc_set == br.tsc_set && br2.tsc == br.tsc);
	}

	msgb_free(msg);
}

static void test_burst_req_nope(uint8_t pdu_ver)
{
	struct osmo_trxd_parse_state st;
	struct osmo_trxd_burst_req br, br2;
	struct msgb *msg = msgb_alloc(4096, "nope");
	int rc;

	printf("=== %s(v%u) ===\n", __func__, pdu_ver);

	fill_burst_req(&br, 0);
	br.flags = OSMO_TRXD_F_NOPE_REQ;

	rc = osmo_trxd_burst_req_build(msg, pdu_ver, &br);
	OSMO_ASSERT(rc == 0);
	osmo_trxd_build_fin(msg, pdu_ver);
	printf("build: %s\n", osmo_trxd_burst_req_name(&br));

	osmo_trxd_parse_state_init(&st);
	rc = osmo_trxd_burst_req_parse(&st, &br2, msgb_data(msg), msgb_length(msg));
	OSMO_ASSERT(rc == (int)msgb_length(msg));
	printf("parse: %s\n", osmo_trxd_burst_req_name(&br2));

	OSMO_ASSERT(br2.flags & OSMO_TRXD_F_NOPE_REQ);
	OSMO_ASSERT(br2.burst_len == 0);

	msgb_free(msg);
}

static void test_burst_req_batch(uint8_t pdu_ver)
{
	struct osmo_trxd_parse_state st;
	struct osmo_trxd_burst_req br, br2;
	struct msgb *msg;
	const uint8_t *buf;
	size_t buf_len;
	unsigned int i;
	int rc;

	/* PDU batching is only supported by TRXDv2 and higher */
	if (pdu_ver < 2)
		return;

	printf("=== %s(v%u) ===\n", __func__, pdu_ver);

	msg = msgb_alloc(4096, "batch");

	for (i = 0; i < 3; i++) {
		fill_burst_req(&br, OSMO_TRXD_BURST_LEN_GMSK);
		br.tn = i;
		rc = osmo_trxd_burst_req_build(msg, pdu_ver, &br);
		OSMO_ASSERT(rc == 0);
	}
	osmo_trxd_build_fin(msg, pdu_ver);
	printf("datagram (%u bytes)\n", msgb_length(msg));

	osmo_trxd_parse_state_init(&st);
	buf = msgb_data(msg);
	buf_len = msgb_length(msg);
	for (i = 0; buf_len > 0; i++) {
		rc = osmo_trxd_burst_req_parse(&st, &br2, buf, buf_len);
		OSMO_ASSERT(rc > 0);
		printf("parse[%u]: %s\n", i, osmo_trxd_burst_req_name(&br2));
		OSMO_ASSERT(br2.tn == i && br2.fn == 2654321);
		buf += rc;
		buf_len -= rc;
		OSMO_ASSERT(!!(br2.flags & OSMO_TRXD_F_BATCH_IND) == (buf_len > 0));
	}
	OSMO_ASSERT(i == 3);

	msgb_free(msg);
}

static void test_mts(void)
{
	static const uint8_t mts_bytes[] = {
		0x80, /* NOPE.ind */
		0x07, /* GMSK, set 0, tsc 7 */
		0x1d, /* GMSK, set 3, tsc 5 */
		0x2b, /* 8-PSK, set 1, tsc 3 */
		0x31, /* GMSK, Access Burst, tsc 1 */
		0x66, /* AQPSK, set 0, tsc 6 */
		0x51, /* invalid */
	};

	printf("=== %s ===\n", __func__);

	for (unsigned int i = 0; i < ARRAY_SIZE(mts_bytes); i++) {
		struct osmo_trxd_burst_ind bi = { };
		uint8_t mts = mts_bytes[i];

		if (trxd_mts_parse_ind(&bi, mts) < 0) {
			printf("trxd_mts_parse_ind(0x%02x) failed\n", mts);
			continue;
		}
		printf("trxd_mts_parse_ind(0x%02x): flags=0x%02x mod=%s set=%u tsc=%u",
		       mts, bi.flags, osmo_trxd_mod_type_name(bi.mod),
		       bi.tsc_set, bi.tsc);

		/* re-encode and compare */
		OSMO_ASSERT(trxd_mts_build_ind(&mts, &bi) == 0);
		printf(" (re-encoded: 0x%02x)\n", mts);
		OSMO_ASSERT(mts == mts_bytes[i]);
	}
}

static void test_parse_errors(void)
{
	struct osmo_trxd_parse_state st;
	struct osmo_trxd_burst_ind bi;
	struct osmo_trxd_burst_req br;
	uint8_t buf[512];
	int rc;

	printf("=== %s ===\n", __func__);

	/* empty buffer */
	osmo_trxd_parse_state_init(&st);
	rc = osmo_trxd_burst_ind_parse(&st, &bi, buf, 0);
	printf("ind, empty buffer: rc=%d\n", rc);
	OSMO_ASSERT(rc < 0);

	/* unknown PDU version */
	memset(buf, 0, sizeof(buf));
	buf[0] = (0x0f << 4);
	osmo_trxd_parse_state_init(&st);
	rc = osmo_trxd_burst_ind_parse(&st, &bi, buf, sizeof(buf));
	printf("ind, PDU version 15: rc=%d\n", rc);
	OSMO_ASSERT(rc == -ENOTSUP);

	/* TRXDv0 with odd burst length */
	memset(buf, 0, sizeof(buf));
	osmo_trxd_parse_state_init(&st);
	rc = osmo_trxd_burst_ind_parse(&st, &bi, buf, 8 + 100);
	printf("ind, v0 with odd burst length: rc=%d\n", rc);
	OSMO_ASSERT(rc == -EINVAL);

	/* TRXDv1 header, but no burst bits */
	memset(buf, 0, sizeof(buf));
	buf[0] = (1 << 4);
	osmo_trxd_parse_state_init(&st);
	rc = osmo_trxd_burst_ind_parse(&st, &bi, buf, 11);
	printf("ind, v1 without burst bits: rc=%d\n", rc);
	OSMO_ASSERT(rc == -EINVAL);

	/* illegal TDMA fn */
	memset(buf, 0, sizeof(buf));
	osmo_store32be(0xffffffff, buf + 1);
	osmo_trxd_parse_state_init(&st);
	rc = osmo_trxd_burst_req_parse(&st, &br, buf, 6 + 148);
	printf("req, illegal TDMA fn: rc=%d\n", rc);
	OSMO_ASSERT(rc == -EINVAL);
}

int main(int argc, char **argv)
{
	for (uint8_t pdu_ver = 0; pdu_ver <= OSMO_TRXD_PDU_VER_MAX; pdu_ver++) {
		test_burst_req(pdu_ver, OSMO_TRXD_BURST_LEN_GMSK);
		test_burst_req(pdu_ver, OSMO_TRXD_BURST_LEN_8PSK);
		test_burst_req_nope(pdu_ver);
		test_burst_req_batch(pdu_ver);

		test_burst_ind(pdu_ver, OSMO_TRXD_BURST_LEN_GMSK);
		test_burst_ind(pdu_ver, OSMO_TRXD_BURST_LEN_8PSK);
		test_burst_ind_batch(pdu_ver);
		test_burst_ind_nope(pdu_ver);
	}

	test_burst_ind_v0_legacy_padding();

	test_mts();
	test_parse_errors();

	printf("Done\n");
	return 0;
}
