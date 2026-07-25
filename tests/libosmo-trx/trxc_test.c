/*! \file tests/trxc_test.c
 * Regression test for the TRXC message codec. */

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

#include <osmocom/core/utils.h>

#include <osmocom/trx/trxc.h>

static void test_msg_parse(void)
{
	static const char * const messages[] = {
		"CMD POWERON",
		"CMD POWEROFF",
		"CMD RXTUNE 890000",
		"CMD SETSLOT 0 7 C5/S1",
		"CMD ECHO",		/* MS dialect */
		"CMD FAKE_TOA 128 0",	/* fake_trx custom command */
		"RSP POWERON 0",
		"RSP SETSLOT 0 0 7",
		"RSP SETFORMAT 1 2",
		"RSP NOMTXPOWER 0 23",
		"RSP ERR 1",
		"RSP MEASURE 0 890000 -85",
		"IND CLOCK 402312",
		/* malformed messages */
		"NDI KCOLC 123456",
		"RSP NOSTATUS",
		"CMD ",
		"",
	};

	printf("=== %s ===\n", __func__);

	for (unsigned int i = 0; i < ARRAY_SIZE(messages); i++) {
		struct osmo_trxc_msg msg;
		int rc;

		rc = osmo_trxc_msg_parse(&msg, messages[i], strlen(messages[i]));
		if (rc < 0) {
			printf("'%s' -> rc=%d\n", messages[i], rc);
			continue;
		}
		printf("'%s' -> type=%d cmd='%s' status=%d params='%s'\n",
		       messages[i], msg.type, msg.cmd, msg.status, msg.params);
		/* re-encode and compare against the original */
		printf("\tre-encoded: '%s'\n", osmo_trxc_msg_name(&msg));
		OSMO_ASSERT(strcmp(osmo_trxc_msg_name(&msg), messages[i]) == 0);
	}
}

static void test_msg_build(void)
{
	static const struct osmo_trxc_msg messages[] = {
		{ .type = OSMO_TRXC_MT_CMD, .cmd = "POWERON" },
		{ .type = OSMO_TRXC_MT_CMD, .cmd = "SETSLOT", .params = "0 7" },
		{ .type = OSMO_TRXC_MT_RSP, .cmd = "POWERON", .status = 0 },
		{ .type = OSMO_TRXC_MT_RSP, .cmd = "SETSLOT", .status = 1, .params = "0 7" },
		{ .type = OSMO_TRXC_MT_IND, .cmd = "CLOCK", .params = "402312" },
	};
	char buf[OSMO_TRXC_MSG_BUF_SIZE];
	int rc;

	printf("=== %s ===\n", __func__);

	for (unsigned int i = 0; i < ARRAY_SIZE(messages); i++) {
		rc = osmo_trxc_msg_build(buf, sizeof(buf), &messages[i]);
		OSMO_ASSERT(rc > 0 && rc == (int)strlen(buf));
		printf("'%s' (rc=%d)\n", buf, rc);
	}

	/* buffer too small */
	rc = osmo_trxc_msg_build(buf, 8, &messages[1]);
	printf("build into a too small buffer: rc=%d\n", rc);
	OSMO_ASSERT(rc < 0);
}

static void test_params_scan(void)
{
	struct osmo_trxc_msg msg;
	unsigned int tn, ts_type;
	int rc;

	printf("=== %s ===\n", __func__);

	rc = osmo_trxc_msg_parse(&msg, "RSP SETSLOT 0 3 7", 17);
	OSMO_ASSERT(rc == 0);

	rc = osmo_trxc_msg_params_scan(&msg, "%u %u", &tn, &ts_type);
	printf("'%s' -> rc=%d tn=%u ts_type=%u\n", msg.params, rc, tn, ts_type);
	OSMO_ASSERT(rc == 2 && tn == 3 && ts_type == 7);
}

/* SETFH (trxcon dialect) carries the whole Mobile Allocation as pairs of
 * Rx/Tx frequencies in kHz, so its parameters can be over 1000 characters
 * long (up to 64 ARFCNs).  Ensure that such messages survive a round-trip. */
static void test_long_params(void)
{
	char buf[OSMO_TRXC_MSG_BUF_SIZE];
	struct osmo_trxc_msg msg = {
		.type = OSMO_TRXC_MT_CMD,
		.cmd = OSMO_TRXC_CMD_SETFH,
	};
	struct osmo_trxc_msg parsed;
	size_t len;
	int rc;

	printf("=== %s ===\n", __func__);

	/* HSN=32 MAIO=5, then 64 pairs of DCS1800 Rx/Tx frequencies */
	len = snprintf(msg.params, sizeof(msg.params), "32 5");
	for (unsigned int i = 0; i < 64; i++) {
		len += snprintf(msg.params + len, sizeof(msg.params) - len,
				" %u %u", 1805200 + i * 200, 1710200 + i * 200);
	}
	printf("SETFH params_len=%zu\n", len);
	OSMO_ASSERT(len < sizeof(msg.params));

	rc = osmo_trxc_msg_build(buf, sizeof(buf), &msg);
	printf("build: rc=%d\n", rc);
	OSMO_ASSERT(rc > 0);

	rc = osmo_trxc_msg_parse(&parsed, buf, rc);
	printf("parse: rc=%d cmd='%s' params_len=%zu\n",
	       rc, parsed.cmd, strlen(parsed.params));
	OSMO_ASSERT(rc == 0);
	OSMO_ASSERT(strcmp(parsed.params, msg.params) == 0);

	/* parameters longer than OSMO_TRXC_PARAMS_LEN_MAX shall be rejected */
	len = strlen(buf);
	memset(buf + len, '6', sizeof(buf) - len - 1);
	buf[sizeof(buf) - 1] = '\0';
	rc = osmo_trxc_msg_parse(&parsed, buf, strlen(buf));
	printf("parse oversized params: rc=%d\n", rc);
	OSMO_ASSERT(rc < 0);
}

static void test_clk_ind(void)
{
	static const char * const messages[] = {
		"IND CLOCK 402312",
		"IND CLOCK 0",
		"IND CLOCK 2715647",	/* GSM_TDMA_HYPERFRAME - 1 */
		"IND CLOCK 2715648",	/* GSM_TDMA_HYPERFRAME */
		"IND CLOCK",
		"IND KCOLC 123",
		"CMD CLOCK 123",
	};
	char buf[OSMO_TRXC_MSG_BUF_SIZE];
	uint32_t fn;
	int rc;

	printf("=== %s ===\n", __func__);

	for (unsigned int i = 0; i < ARRAY_SIZE(messages); i++) {
		rc = osmo_trxc_clock_ind_parse(&fn, messages[i], strlen(messages[i]));
		if (rc < 0) {
			printf("'%s' -> rc=%d\n", messages[i], rc);
			continue;
		}
		printf("'%s' -> fn=%u\n", messages[i], fn);
		/* re-encode and compare against the original */
		rc = osmo_trxc_clock_ind_build(buf, sizeof(buf), fn);
		OSMO_ASSERT(rc > 0);
		OSMO_ASSERT(strcmp(buf, messages[i]) == 0);
	}

	/* out of range TDMA fn */
	rc = osmo_trxc_clock_ind_build(buf, sizeof(buf), 2715648);
	printf("build with out of range fn: rc=%d\n", rc);
	OSMO_ASSERT(rc < 0);
}

static void test_setslot_parse_one(const char *params)
{
	struct osmo_trxc_msg msg = { .type = OSMO_TRXC_MT_CMD, .cmd = "SETSLOT" };
	struct osmo_trxc_setslot ss;
	char buf[OSMO_TRXC_MSG_BUF_SIZE];
	int rc;

	snprintf(msg.params, sizeof(msg.params), "%s", params);

	rc = osmo_trxc_setslot_parse(&ss, &msg);
	if (rc < 0) {
		printf("'%s' -> rc=%d\n", params, rc);
		return;
	}

	printf("'%s' -> tn=%u vamos=%d comb=%s num_tsc=%u",
	       params, ss.tn, ss.vamos,
	       ss.vamos ? osmo_trxc_vamos_comb_name(ss.vamos_comb) : "(numeric)",
	       ss.num_tsc);
	if (!ss.vamos)
		printf(" chan_comb=%d", ss.chan_comb);
	for (unsigned int i = 0; i < ss.num_tsc; i++)
		printf(" C%u/S%u", ss.tsc[i].tsc, ss.tsc[i].tsc_set);
	printf("\n");

	/* re-encode and compare against the original params */
	rc = osmo_trxc_setslot_build(buf, sizeof(buf), &ss);
	OSMO_ASSERT(rc > 0 && rc == (int)strlen(buf));
	printf("\tre-encoded: '%s'\n", buf);
	OSMO_ASSERT(strcmp(buf, params) == 0);
}

static void test_setslot(void)
{
	static const char * const good[] = {
		"0 0",
		"7 13",
		"4 1 C7/S1",		/* manual example */
		"0 VFF C0/S1 C0/S2",	/* manual example */
		"3 VHH C1/S3 C1/S4",	/* manual example */
		"1 VFH C2/S1 C2/S4",	/* manual example */
		"2 HVHH C0/S1 C0/S1 C0/S2", /* manual example */
	};
	static const char * const bad[] = {
		"8 0",			/* tn out of range */
		"0 14",			/* chan_comb out of range */
		"0 -1",			/* chan_comb out of range */
		"0",			/* missing chan_comb */
		"",			/* missing everything */
		"x 0",			/* tn not numeric */
		"0 x",			/* chan_comb not numeric, not a known VAMOS name */
		"0 0 bogus",		/* trailing token not a C<tsc>/S<tsc_set> */
		"0 HVHH C0/S1 C0/S1 C0/S2 C0/S3", /* too many TSC overrides */
	};
	struct osmo_trxc_msg msg = { .type = OSMO_TRXC_MT_CMD, .cmd = "SETSLOT" };
	struct osmo_trxc_setslot ss;
	int rc;

	printf("=== %s ===\n", __func__);

	for (unsigned int i = 0; i < ARRAY_SIZE(good); i++)
		test_setslot_parse_one(good[i]);
	for (unsigned int i = 0; i < ARRAY_SIZE(bad); i++)
		test_setslot_parse_one(bad[i]);

	/* build() range check */
	memset(&ss, 0, sizeof(ss));
	ss.tn = 8;
	rc = osmo_trxc_setslot_build(msg.params, sizeof(msg.params), &ss);
	printf("build with out of range tn: rc=%d\n", rc);
	OSMO_ASSERT(rc < 0);
}

int main(int argc, char **argv)
{
	test_msg_parse();
	test_msg_build();
	test_params_scan();
	test_long_params();
	test_clk_ind();
	test_setslot();

	printf("Done\n");
	return 0;
}
