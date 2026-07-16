/*! \file src/trxd.c
 * TRXD (burst data) protocol: I/O-free PDU codec.
 * Based on the TRXD implementation in osmo-bts-trx and osmo-trx. */

/*
 * (C) 2013 Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2016-2017 Harald Welte <laforge@gnumonks.org>
 * (C) 2019 Vadim Yanitskiy <axilirator@gmail.com>
 * (C) 2021-2026 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
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
#include <string.h>
#include <stdint.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/utils.h>
#include <osmocom/gsm/gsm0502.h>

#include <osmocom/trx/trxd.h>
#include <osmocom/trx/trxd_int.h>

/* Uplink TRXDv0 header length: TDMA TN + FN + RSSI + ToA256 */
#define TRXD_IND_V0HDR_LEN	(1 + 4 + 1 + 2)
/* Uplink TRXDv1 header length: additional MTS + C/I */
#define TRXD_IND_V1HDR_LEN	(TRXD_IND_V0HDR_LEN + 1 + 2)
/* Uplink TRXDv2 header length: TDMA TN + TRXN + MTS + RSSI + ToA256 + C/I */
#define TRXD_IND_V2HDR_LEN	(1 + 1 + 1 + 1 + 2 + 2)

/* Downlink TRXDv0/v1 header length: TDMA TN + FN + Att */
#define TRXD_REQ_V01HDR_LEN	(1 + 4 + 1)
/* Downlink TRXDv2 header length: TDMA TN + TRXN + MTS + Att + SCPIR + spare3 */
#define TRXD_REQ_V2HDR_LEN	(1 + 1 + 1 + 1 + 1 + 3)

const struct value_string osmo_trxd_mod_type_names[] = {
	{ OSMO_TRXD_MOD_T_GMSK,		"GMSK" },
	{ OSMO_TRXD_MOD_T_8PSK,		"8-PSK" },
	{ OSMO_TRXD_MOD_T_AQPSK,	"AQPSK" },
	{ 0, NULL }
};

/*! Initialize a parser state; call before parsing each datagram.
 *  \param[out] st parser state to be initialized */
void osmo_trxd_parse_state_init(struct osmo_trxd_parse_state *st)
{
	memset(st, 0, sizeof(*st));
}

/* Expected burst length (in bits on the wire) for a given modulation */
static int burst_len_by_mod(enum osmo_trxd_mod_type mod)
{
	switch (mod) {
	case OSMO_TRXD_MOD_T_GMSK:
	case OSMO_TRXD_MOD_T_AQPSK:
		return OSMO_TRXD_BURST_LEN_GMSK;
	case OSMO_TRXD_MOD_T_8PSK:
		return OSMO_TRXD_BURST_LEN_8PSK;
	default:
		return -ENOTSUP;
	}
}

static int mts_parse(uint32_t *flags, enum osmo_trxd_mod_type *mod,
		     uint8_t *tsc_set, uint8_t *tsc, uint8_t mts)
{
	if (mts & (1 << 7)) {
		*flags |= OSMO_TRXD_F_NOPE_IND;
		return 0;
	}

	/* | 7 6 5 4 3 2 1 0 | Bitmask / description
	 * | . 0 0 X X . . . | GMSK, 4 TSC sets (0..3)
	 * | . 0 1 0 X . . . | 8-PSK, 2 TSC sets (0..1)
	 * | . 0 1 1 0 . . . | GMSK, Access Burst
	 * | . 1 1 X X . . . | AQPSK, 2 TSC sets + SCPIR */
	if ((mts >> 5) == 0x00) {
		*mod = OSMO_TRXD_MOD_T_GMSK;
		*tsc_set = (mts >> 3) & 0x03;
	} else if ((mts >> 4) == 0x02) {
		*mod = OSMO_TRXD_MOD_T_8PSK;
		*tsc_set = (mts >> 3) & 0x01;
	} else if ((mts >> 3) == 0x06) {
		*flags |= OSMO_TRXD_F_ACCESS_BURST;
		*mod = OSMO_TRXD_MOD_T_GMSK;
		*tsc_set = 0;
	} else if ((mts >> 5) == 0x03) {
		*mod = OSMO_TRXD_MOD_T_AQPSK;
		*tsc_set = (mts >> 3) & 0x01;
	} else {
		return -ENOTSUP;
	}

	*tsc = mts & 0x07;
	*flags |= (OSMO_TRXD_F_MOD_TYPE | OSMO_TRXD_F_TS_INFO);

	return 0;
}

int trxd_mts_parse_ind(struct osmo_trxd_burst_ind *bi, uint8_t mts)
{
	return mts_parse(&bi->flags, &bi->mod, &bi->tsc_set, &bi->tsc, mts);
}

int trxd_mts_parse_req(struct osmo_trxd_burst_req *br, uint8_t mts)
{
	return mts_parse(&br->flags, &br->mod, &br->tsc_set, &br->tsc, mts);
}

static int mts_build(uint8_t *mts, uint32_t flags,
		     enum osmo_trxd_mod_type mod,
		     uint8_t tsc_set, uint8_t tsc)
{
	if (flags & OSMO_TRXD_F_NOPE_IND) {
		*mts = (1 << 7);
		return 0;
	}

	if (flags & OSMO_TRXD_F_ACCESS_BURST) {
		*mts = (0x06 << 3) | (tsc & 0x07);
		return 0;
	}

	switch (mod) {
	case OSMO_TRXD_MOD_T_GMSK:
		*mts = ((tsc_set & 0x03) << 3);
		break;
	case OSMO_TRXD_MOD_T_8PSK:
		*mts = (0x02 << 4) | ((tsc_set & 0x01) << 3);
		break;
	case OSMO_TRXD_MOD_T_AQPSK:
		*mts = (0x03 << 5) | ((tsc_set & 0x01) << 3);
		break;
	default:
		return -ENOTSUP;
	}

	*mts |= (tsc & 0x07);
	return 0;
}

int trxd_mts_build_ind(uint8_t *mts, const struct osmo_trxd_burst_ind *bi)
{
	return mts_build(mts, bi->flags, bi->mod, bi->tsc_set, bi->tsc);
}

int trxd_mts_build_req(uint8_t *mts, const struct osmo_trxd_burst_req *br)
{
	return mts_build(mts, br->flags, br->mod, br->tsc_set, br->tsc);
}

/***********************************************************************
 * burst indication (TRX -> L1): soft bits + measurements
 ***********************************************************************/

/* Convert unsigned soft-bits [254..0] to soft-bits [-127..127] */
static void soft_bits_parse(sbit_t *out, const uint8_t *in, size_t len)
{
	size_t i;

	for (i = 0; i < len; i++) {
		if (in[i] == 255)
			out[i] = -127;
		else
			out[i] = 127 - in[i];
	}
}

/* Convert soft-bits [-127..127] to unsigned soft-bits [254..0] */
static void soft_bits_build(uint8_t *out, const sbit_t *in, size_t len)
{
	size_t i;

	for (i = 0; i < len; i++)
		out[i] = 127 - in[i];
}

/* Common header part shared by TRXDv0 and TRXDv1 */
static void trxd_burst_ind_parse_hdr_v0(struct osmo_trxd_burst_ind *bi,
					const uint8_t *buf)
{
	bi->tn = buf[0] & 0x07;
	bi->fn = osmo_load32be(buf + 1);
	bi->rssi = -(int8_t)buf[5];
	bi->toa256 = (int16_t)osmo_load16be(buf + 6);
}

/* TRXDv0: modulation is guessed by the burst length; a legacy transceiver
 * may append two garbage bytes, which we tolerate (and consume). */
static int trxd_burst_ind_parse_v0(struct osmo_trxd_burst_ind *bi,
				   const uint8_t *buf, size_t buf_len)
{
	size_t burst_len = buf_len - TRXD_IND_V0HDR_LEN;

	trxd_burst_ind_parse_hdr_v0(bi, buf);

	/* NOPE.ind: TRXDv0 has no MTS, the burst payload is simply omitted */
	if (burst_len == 0) {
		bi->flags |= OSMO_TRXD_F_NOPE_IND;
		return buf_len;
	}

	switch (burst_len) {
	case OSMO_TRXD_BURST_LEN_GMSK:
	case OSMO_TRXD_BURST_LEN_GMSK + 2:
		bi->mod = OSMO_TRXD_MOD_T_GMSK;
		bi->burst_len = OSMO_TRXD_BURST_LEN_GMSK;
		break;
	case OSMO_TRXD_BURST_LEN_8PSK:
	case OSMO_TRXD_BURST_LEN_8PSK + 2:
		bi->mod = OSMO_TRXD_MOD_T_8PSK;
		bi->burst_len = OSMO_TRXD_BURST_LEN_8PSK;
		break;
	default:
		return -EINVAL;
	}

	bi->flags |= OSMO_TRXD_F_MOD_TYPE;
	soft_bits_parse(&bi->burst[0], buf + TRXD_IND_V0HDR_LEN, bi->burst_len);

	/* consume everything, including the optional legacy padding */
	return buf_len;
}

static int trxd_burst_ind_parse_v1(struct osmo_trxd_burst_ind *bi,
				   const uint8_t *buf, size_t buf_len)
{
	int rc;

	trxd_burst_ind_parse_hdr_v0(bi, buf);

	/* MTS (Modulation and Training Sequence) */
	rc = trxd_mts_parse_ind(bi, buf[TRXD_IND_V0HDR_LEN + 0]);
	if (rc < 0)
		return rc;

	/* C/I: Carrier-to-Interference ratio (in centiBels) */
	bi->ci_cb = (int16_t)osmo_load16be(buf + TRXD_IND_V0HDR_LEN + 1);
	bi->flags |= OSMO_TRXD_F_CI_CB;

	return TRXD_IND_V1HDR_LEN;
}

static int trxd_burst_ind_parse_v2(struct osmo_trxd_parse_state *st,
				   struct osmo_trxd_burst_ind *bi,
				   const uint8_t *buf, size_t buf_len)
{
	int rc;

	/* TDMA timeslot number (other bits are RFU) */
	bi->tn = buf[0] & 0x07;

	if (buf[1] & (1 << 7)) /* BATCH.ind */
		bi->flags |= OSMO_TRXD_F_BATCH_IND;
	if (buf[1] & (1 << 6)) /* VAMOS.ind */
		bi->flags |= OSMO_TRXD_F_SHADOW_IND;

	/* TRX (RF channel) number */
	bi->trx_num = buf[1] & 0x3f;
	bi->flags |= OSMO_TRXD_F_TRX_NUM;

	/* MTS (Modulation and Training Sequence) */
	rc = trxd_mts_parse_ind(bi, buf[2]);
	if (rc < 0)
		return rc;

	bi->rssi = -(int8_t)buf[3];
	bi->toa256 = (int16_t)osmo_load16be(buf + 4);
	bi->ci_cb = (int16_t)osmo_load16be(buf + 6);
	bi->flags |= OSMO_TRXD_F_CI_CB;

	/* TDMA frame number is absent in batched PDUs */
	if (st->num_pdus == 0) {
		if (buf_len < TRXD_IND_V2HDR_LEN + sizeof(uint32_t))
			return -EINVAL;
		bi->fn = osmo_load32be(buf + TRXD_IND_V2HDR_LEN);
		st->fn = bi->fn;
		return TRXD_IND_V2HDR_LEN + sizeof(uint32_t);
	}

	bi->fn = st->fn;
	return TRXD_IND_V2HDR_LEN;
}

/*! Parse one burst indication (TRX -> L1) PDU from the given buffer.
 *
 *  The PDU version is parsed from the first PDU of a datagram and kept
 *  in \a st.  Starting from TRXDv2, a datagram may batch multiple PDUs:
 *  call this function repeatedly on the remaining buffer as long as the
 *  parsed PDU has OSMO_TRXD_F_BATCH_IND set in bi->flags.
 *
 *  \param[inout] st parser state (see osmo_trxd_parse_state_init())
 *  \param[out] bi parsed burst indication
 *  \param[in] buf buffer positioned at the beginning of a PDU
 *  \param[in] buf_len remaining length of the buffer
 *  \returns number of consumed bytes on success; negative on error */
int osmo_trxd_burst_ind_parse(struct osmo_trxd_parse_state *st,
			      struct osmo_trxd_burst_ind *bi,
			      const uint8_t *buf, size_t buf_len)
{
	int hdr_len;
	int burst_len;

	if (buf_len == 0)
		return -EINVAL;

	/* PDU version is parsed from the first PDU of a datagram */
	if (st->num_pdus == 0) {
		st->pdu_ver = buf[0] >> 4;
		if (st->pdu_ver > OSMO_TRXD_PDU_VER_MAX)
			return -ENOTSUP;
	}

	memset(bi, 0, sizeof(*bi));

	switch (st->pdu_ver) {
	case 0:
		if (buf_len < TRXD_IND_V0HDR_LEN)
			return -EINVAL;
		hdr_len = trxd_burst_ind_parse_v0(bi, buf, buf_len);
		break;
	case 1:
		if (buf_len < TRXD_IND_V1HDR_LEN)
			return -EINVAL;
		hdr_len = trxd_burst_ind_parse_v1(bi, buf, buf_len);
		break;
	case 2:
		if (buf_len < TRXD_IND_V2HDR_LEN)
			return -EINVAL;
		hdr_len = trxd_burst_ind_parse_v2(st, bi, buf, buf_len);
		break;
	default:
		return -ENOTSUP;
	}

	if (hdr_len < 0)
		return hdr_len;

	if (bi->fn >= GSM_TDMA_HYPERFRAME)
		return -EINVAL;

	st->num_pdus++;

	/* TRXDv0 consumes the whole datagram, incl. the burst bits */
	if (st->pdu_ver == 0)
		return hdr_len;

	/* NOPE.ind contains no burst */
	if (bi->flags & OSMO_TRXD_F_NOPE_IND) {
		bi->burst_len = 0;
		return hdr_len;
	}

	burst_len = burst_len_by_mod(bi->mod);
	if (burst_len < 0)
		return burst_len;
	if (buf_len < (size_t)(hdr_len + burst_len))
		return -EINVAL;

	bi->burst_len = burst_len;
	soft_bits_parse(&bi->burst[0], buf + hdr_len, bi->burst_len);

	return hdr_len + burst_len;
}

/*! Append one encoded burst indication (TRX -> L1) PDU to the given msgb.
 *
 *  For TRXDv2, PDU batching works by calling this function repeatedly on
 *  the same msgb; call osmo_trxd_build_fin() once the datagram is complete
 *  (it clears the BATCH.ind bit of the last PDU).
 *
 *  \param[inout] msg destination message buffer
 *  \param[in] pdu_ver TRXD PDU version to encode
 *  \param[in] bi burst indication to be encoded
 *  \returns 0 on success; negative on error.  Note that TRXDv0 has no
 *	     MTS field, so a NOPE.ind is encoded as a header-only PDU
 *	     with the burst payload omitted. */
int osmo_trxd_burst_ind_build(struct msgb *msg, uint8_t pdu_ver,
			      const struct osmo_trxd_burst_ind *bi)
{
	bool first = (msgb_length(msg) == 0);
	uint8_t *buf;
	int rc;

	switch (pdu_ver) {
	case 0:
		buf = msgb_put(msg, TRXD_IND_V0HDR_LEN);
		buf[0] = ((pdu_ver & 0x0f) << 4) | (bi->tn & 0x07);
		osmo_store32be(bi->fn, buf + 1);
		buf[5] = (uint8_t)(-bi->rssi);
		osmo_store16be(bi->toa256, buf + 6);
		break;
	case 1:
		buf = msgb_put(msg, TRXD_IND_V1HDR_LEN);
		buf[0] = ((pdu_ver & 0x0f) << 4) | (bi->tn & 0x07);
		osmo_store32be(bi->fn, buf + 1);
		buf[5] = (uint8_t)(-bi->rssi);
		osmo_store16be(bi->toa256, buf + 6);
		rc = trxd_mts_build_ind(&buf[8], bi);
		if (rc < 0)
			return rc;
		osmo_store16be(bi->ci_cb, buf + 9);
		break;
	case 2:
		/* l2h points to the last encoded PDU (for osmo_trxd_build_fin) */
		msg->l2h = msg->tail;
		buf = msgb_put(msg, TRXD_IND_V2HDR_LEN);
		buf[0] = bi->tn & 0x07;
		/* BATCH.ind; unset in the last PDU by osmo_trxd_build_fin() */
		buf[1] = (bi->trx_num & 0x3f) | (1 << 7);
		if (bi->flags & OSMO_TRXD_F_SHADOW_IND)
			buf[1] |= (1 << 6);
		rc = trxd_mts_build_ind(&buf[2], bi);
		if (rc < 0)
			return rc;
		buf[3] = (uint8_t)(-bi->rssi);
		osmo_store16be(bi->toa256, buf + 4);
		osmo_store16be(bi->ci_cb, buf + 6);
		/* Some fields are not present in batched PDUs */
		if (first) {
			buf[0] |= (pdu_ver & 0x0f) << 4;
			msgb_put_u32(msg, bi->fn);
		}
		break;
	default:
		return -ENOTSUP;
	}

	if (~bi->flags & OSMO_TRXD_F_NOPE_IND) {
		soft_bits_build(msgb_put(msg, bi->burst_len),
				&bi->burst[0], bi->burst_len);
	}

	return 0;
}

/***********************************************************************
 * burst transmit request (L1 -> TRX): hard bits
 ***********************************************************************/

static int trxd_burst_req_parse_v01(struct osmo_trxd_burst_req *br,
				    const uint8_t *buf, size_t buf_len)
{
	size_t burst_len = buf_len - TRXD_REQ_V01HDR_LEN;

	br->tn = buf[0] & 0x07;
	br->fn = osmo_load32be(&buf[1]);
	br->att = buf[5];

	/* NOPE.req: TRXDv0/v1 have no MTS, the burst payload is simply omitted */
	if (burst_len == 0) {
		br->flags |= OSMO_TRXD_F_NOPE_REQ;
		return buf_len;
	}

	switch (burst_len) {
	case OSMO_TRXD_BURST_LEN_8PSK:
		br->mod = OSMO_TRXD_MOD_T_8PSK;
		break;
	case OSMO_TRXD_BURST_LEN_GMSK:
		br->mod = OSMO_TRXD_MOD_T_GMSK;
		break;
	default:
		return -EINVAL;
	}

	br->flags |= OSMO_TRXD_F_MOD_TYPE;
	br->burst_len = burst_len;
	memcpy(&br->burst[0], buf + TRXD_REQ_V01HDR_LEN, burst_len);

	return buf_len;
}

static int trxd_burst_req_parse_v2(struct osmo_trxd_parse_state *st,
				   struct osmo_trxd_burst_req *br,
				   const uint8_t *buf, size_t buf_len)
{
	size_t hdr_len = TRXD_REQ_V2HDR_LEN;
	int burst_len;
	int rc;

	br->tn = buf[0] & 0x07;

	if (buf[1] & (1 << 7)) /* BATCH.ind */
		br->flags |= OSMO_TRXD_F_BATCH_IND;

	br->trx_num = buf[1] & 0x3f;
	br->flags |= OSMO_TRXD_F_TRX_NUM;

	rc = trxd_mts_parse_req(br, buf[2]);
	if (rc < 0)
		return rc;

	br->att = buf[3];
	br->scpir = (int8_t)buf[4];
	/* buf[5..7] is spare */

	/* TDMA frame number is absent in batched PDUs */
	if (st->num_pdus == 0) {
		if (buf_len < hdr_len + sizeof(uint32_t))
			return -EINVAL;
		br->fn = osmo_load32be(buf + hdr_len);
		st->fn = br->fn;
		hdr_len += sizeof(uint32_t);
	} else {
		br->fn = st->fn;
	}

	/* NOPE.req contains no burst */
	if (br->flags & OSMO_TRXD_F_NOPE_REQ) {
		br->burst_len = 0;
		return hdr_len;
	}

	burst_len = burst_len_by_mod(br->mod);
	if (burst_len < 0)
		return burst_len;
	if (buf_len < hdr_len + burst_len)
		return -EINVAL;

	br->burst_len = burst_len;
	memcpy(&br->burst[0], buf + hdr_len, burst_len);

	return hdr_len + burst_len;
}

/*! Parse one burst transmit request (L1 -> TRX) PDU from the given buffer.
 *
 *  The PDU version is parsed from the first PDU of a datagram and kept
 *  in \a st.  Starting from TRXDv2, a datagram may batch multiple PDUs:
 *  call this function repeatedly on the remaining buffer as long as the
 *  parsed PDU has OSMO_TRXD_F_BATCH_IND set in br->flags.
 *
 *  \param[inout] st parser state (see osmo_trxd_parse_state_init())
 *  \param[out] br parsed burst transmit request
 *  \param[in] buf buffer positioned at the beginning of a PDU
 *  \param[in] buf_len remaining length of the buffer
 *  \returns number of consumed bytes on success; negative on error */
int osmo_trxd_burst_req_parse(struct osmo_trxd_parse_state *st,
			      struct osmo_trxd_burst_req *br,
			      const uint8_t *buf, size_t buf_len)
{
	int pdu_len;

	if (buf_len == 0)
		return -EINVAL;

	if (st->num_pdus == 0) {
		st->pdu_ver = buf[0] >> 4;
		if (st->pdu_ver > OSMO_TRXD_PDU_VER_MAX)
			return -ENOTSUP;
	}

	memset(br, 0, sizeof(*br));

	switch (st->pdu_ver) {
	case 0:
	case 1:
		if (buf_len < TRXD_REQ_V01HDR_LEN)
			return -EINVAL;
		pdu_len = trxd_burst_req_parse_v01(br, buf, buf_len);
		break;
	case 2:
		if (buf_len < TRXD_REQ_V2HDR_LEN)
			return -EINVAL;
		pdu_len = trxd_burst_req_parse_v2(st, br, buf, buf_len);
		break;
	default:
		return -ENOTSUP;
	}

	if (pdu_len < 0)
		return pdu_len;

	if (br->fn >= GSM_TDMA_HYPERFRAME)
		return -EINVAL;

	st->num_pdus++;
	return pdu_len;
}

/*! Append one encoded burst transmit request (L1 -> TRX) PDU to the given msgb.
 *
 *  For TRXDv2, PDU batching works by calling this function repeatedly on
 *  the same msgb; call osmo_trxd_build_fin() once the datagram is complete
 *  (it clears the BATCH.ind bit of the last PDU).
 *
 *  \param[inout] msg destination message buffer
 *  \param[in] pdu_ver TRXD PDU version to encode
 *  \param[in] br burst transmit request to be encoded
 *  \returns 0 on success; negative on error */
int osmo_trxd_burst_req_build(struct msgb *msg, uint8_t pdu_ver,
			      const struct osmo_trxd_burst_req *br)
{
	bool first = (msgb_length(msg) == 0);
	uint8_t *buf;
	int rc;

	switch (pdu_ver) {
	/* Both versions have the same PDU format */
	case 0:
	case 1:
		buf = msgb_put(msg, TRXD_REQ_V01HDR_LEN);
		buf[0] = ((pdu_ver & 0x0f) << 4) | (br->tn & 0x07);
		osmo_store32be(br->fn, buf + 1);
		buf[5] = br->att;
		break;
	case 2:
		/* l2h points to the last encoded PDU (for osmo_trxd_build_fin) */
		msg->l2h = msg->tail;
		buf = msgb_put(msg, TRXD_REQ_V2HDR_LEN);
		buf[0] = br->tn & 0x07;
		/* BATCH.ind; unset in the last PDU by osmo_trxd_build_fin() */
		buf[1] = (br->trx_num & 0x3f) | (1 << 7);
		rc = trxd_mts_build_req(&buf[2], br);
		if (rc < 0)
			return rc;
		buf[3] = br->att;
		buf[4] = (uint8_t)br->scpir;
		buf[5] = buf[6] = buf[7] = 0x00; /* Spare */
		/* Some fields are not present in batched PDUs */
		if (first) {
			buf[0] |= (pdu_ver & 0x0f) << 4;
			msgb_put_u32(msg, br->fn);
		}
		break;
	default:
		return -ENOTSUP;
	}

	if (~br->flags & OSMO_TRXD_F_NOPE_REQ) {
		/* copy hard-bits {0,1} */
		memcpy(msgb_put(msg, br->burst_len),
		       &br->burst[0], br->burst_len);
	}

	return 0;
}

/*! Finalize a datagram built by osmo_trxd_burst_{ind,req}_build().
 *  \param[inout] msg message buffer holding the encoded PDU(s)
 *  \param[in] pdu_ver TRXD PDU version in use */
void osmo_trxd_build_fin(struct msgb *msg, uint8_t pdu_ver)
{
	/* TRXDv2: unset BATCH.ind in the last PDU */
	if (pdu_ver >= 2 && msg->l2h != NULL)
		msg->l2h[1] &= ~(1 << 7);
}

/***********************************************************************
 * logging helpers
 ***********************************************************************/

/*! Compose a human-readable representation of the given burst indication
 *  (for logging).  \returns pointer to a thread-local static buffer */
const char *osmo_trxd_burst_ind_name(const struct osmo_trxd_burst_ind *bi)
{
	static __thread char buf[256];
	struct osmo_strbuf sb = { .buf = buf, .len = sizeof(buf) };

	OSMO_STRBUF_PRINTF(sb, "%s tn=%u fn=%u",
			   (bi->flags & OSMO_TRXD_F_NOPE_IND) ? "NOPE.ind" : "BURST.ind",
			   bi->tn, bi->fn);
	if (bi->flags & OSMO_TRXD_F_TRX_NUM)
		OSMO_STRBUF_PRINTF(sb, " trx_num=%u", bi->trx_num);
	OSMO_STRBUF_PRINTF(sb, " rssi=%d toa256=%d", bi->rssi, bi->toa256);
	if (bi->flags & OSMO_TRXD_F_CI_CB)
		OSMO_STRBUF_PRINTF(sb, " C/I=%d cB", bi->ci_cb);
	if (bi->flags & OSMO_TRXD_F_NOPE_IND)
		return buf;
	if (bi->flags & OSMO_TRXD_F_MOD_TYPE)
		OSMO_STRBUF_PRINTF(sb, " mod=%s", osmo_trxd_mod_type_name(bi->mod));
	if (bi->flags & OSMO_TRXD_F_TS_INFO)
		OSMO_STRBUF_PRINTF(sb, " set=%u tsc=%u", bi->tsc_set, bi->tsc);
	OSMO_STRBUF_PRINTF(sb, " burst_len=%zu", bi->burst_len);

	return buf;
}

/*! Compose a human-readable representation of the given burst transmit
 *  request (for logging).  \returns pointer to a thread-local static buffer */
const char *osmo_trxd_burst_req_name(const struct osmo_trxd_burst_req *br)
{
	static __thread char buf[256];
	struct osmo_strbuf sb = { .buf = buf, .len = sizeof(buf) };

	OSMO_STRBUF_PRINTF(sb, "%s tn=%u fn=%u att=%u",
			   (br->flags & OSMO_TRXD_F_NOPE_REQ) ? "NOPE.req" : "BURST.req",
			   br->tn, br->fn, br->att);
	if (br->flags & OSMO_TRXD_F_TRX_NUM)
		OSMO_STRBUF_PRINTF(sb, " trx_num=%u", br->trx_num);
	if (br->flags & OSMO_TRXD_F_NOPE_REQ)
		return buf;
	if (br->flags & OSMO_TRXD_F_MOD_TYPE)
		OSMO_STRBUF_PRINTF(sb, " mod=%s", osmo_trxd_mod_type_name(br->mod));
	if (br->flags & OSMO_TRXD_F_TS_INFO)
		OSMO_STRBUF_PRINTF(sb, " set=%u tsc=%u", br->tsc_set, br->tsc);
	OSMO_STRBUF_PRINTF(sb, " burst_len=%zu", br->burst_len);

	return buf;
}
