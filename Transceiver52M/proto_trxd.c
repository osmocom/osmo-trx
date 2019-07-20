/*
 * Copyright (C) 2019 sysmocom - s.f.m.c. GmbH
 * All Rights Reserved
 *
 * SPDX-License-Identifier: AGPL-3.0+
 *
 * Author: Pau Espin Pedrol <pespin@sysmocom.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 */

#include "proto_trxd.h"

#include <osmocom/core/bits.h>

static void trxd_fill_common(struct trxd_hdr_common *common, const struct trx_ul_burst_ind *bi, uint8_t version)
{
	common->version = version & 0x07;
	common->reserved = 0;
	common->tn = bi->tn;
	osmo_store32be(bi->fn, &common->fn);
}

static void trxd_fill_v0_specific(struct trxd_hdr_v0_specific *v0, const struct trx_ul_burst_ind *bi)
{
	int toa_int;

	/* in 1/256 symbols, round to closest integer */
	toa_int = (int) (bi->toa * 256.0 + 0.5);
	v0->rssi = bi->rssi;
	osmo_store16be(toa_int, &v0->toa);
}

static void trxd_fill_v1_specific(struct trxd_hdr_v1_specific *v1, const struct trx_ul_burst_ind *bi)
{
	int16_t ci_int_cB;

	/* deciBels->centiBels, round to closest integer */
	ci_int_cB = (int16_t)((bi->ci * 10) + 0.5);

	v1->idle = !!bi->idle;
	v1->modulation = (bi->modulation == MODULATION_GMSK) ?
					TRXD_MODULATION_GMSK(bi->tss) :
					TRXD_MODULATION_8PSK(bi->tss);
	v1->tsc = bi->tsc;
	osmo_store16be(ci_int_cB, &v1->ci);
}

static void trxd_fill_burst_normalized255(uint8_t* soft_bits, const struct trx_ul_burst_ind *bi)
{
	unsigned i;
	for (i = 0; i < bi->nbits; i++)
		soft_bits[i] = (char) round(bi->rx_burst[i] * 255.0);
}

bool trxd_send_burst_ind_v0(size_t chan, int fd, const struct trx_ul_burst_ind *bi) {
	int rc;

	/* v0 doesn't support idle frames, they are simply dropped, not sent */
	if(bi->idle)
		return true;

	/* +2: Historically (OpenBTS times), two extra non-used bytes are sent appeneded to each burst */
	char buf[sizeof(struct trxd_hdr_v0) + bi->nbits + 2];
	struct trxd_hdr_v0* pkt = (struct trxd_hdr_v0*)buf;

	trxd_fill_common(&pkt->common, bi, 0);
	trxd_fill_v0_specific(&pkt->v0, bi);
	trxd_fill_burst_normalized255(&pkt->soft_bits[0], bi);

	/* +1: Historical reason. There's an uninitizalied byte in there: pkt->soft_bits[bi->nbits] */
	pkt->soft_bits[bi->nbits + 1] = '\0';

	rc = write(fd, buf, sizeof(struct trxd_hdr_v0) + bi->nbits + 2);
	if (rc <= 0) {
		CLOGCHAN(chan, DMAIN, LOGL_NOTICE, "mDataSockets write(%d) failed: %d\n", fd, rc);
		return false;
	}
	return true;
}

bool trxd_send_burst_ind_v1(size_t chan, int fd, const struct trx_ul_burst_ind *bi) {
	int rc;
	size_t buf_len;

	buf_len = sizeof(struct trxd_hdr_v1);
	if (!bi->idle)
		buf_len += bi->nbits;
	char buf[buf_len];

	struct trxd_hdr_v1* pkt = (struct trxd_hdr_v1*)buf;
	trxd_fill_common(&pkt->common, bi, 1);
	trxd_fill_v0_specific(&pkt->v0, bi);
	trxd_fill_v1_specific(&pkt->v1, bi);

	if (!bi->idle)
		trxd_fill_burst_normalized255(&pkt->soft_bits[0], bi);

	rc = write(fd, buf, buf_len);
	if (rc <= 0) {
		CLOGCHAN(chan, DMAIN, LOGL_NOTICE, "mDataSockets write(%d) failed: %d\n", fd, rc);
		return false;
	}
	return true;
}
