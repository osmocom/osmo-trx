/*! \file src/burst_synch.c
 * osmo-trx-proxy: detect the training/synch. sequence of a burst. */

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

#include <stdint.h>
#include <string.h>

#include <osmocom/core/bits.h>
#include <osmocom/gsm/gsm0502.h>
#include <osmocom/trx/trxd.h>

#include <osmocom/proxy/burst_synch.h>

/* A BURST.req sourced from TRXDv0/v1 carries no MTS field at all, so the
 * proxy has no way to know the true modulation/TSC of a burst it forwards
 * onward as a TRXDv1/TRXDv2 BURST.ind (whose MTS field is mandatory).
 * Rather than asserting a fixed, possibly wrong TSC, detect it by
 * comparing known positions within the burst against the following
 * training/synch. sequences defined in 3GPP TS 45.002:
 *
 *   - Access Burst (RACH), table 5.2.7-3: TS0, TS1, TS2, TS4.
 *     Table 5.2.7-4 (EC operation variants) is not covered, as EC
 *     operation is not implemented by Osmocom.
 *   - Synchronization Burst (SCH), table 5.2.5-3: TS0..TS3.
 *   - Normal Burst (GMSK), tables 5.2.3a-d: all four TSC sets, TSC 0..7 each.
 *   - Normal Burst (8-PSK), tables 5.2.3f-g: both TSC sets, TSC 0..7 each.
 */

#define AB_SEQ_NUM		4

/* 3GPP TS 45.002, section 5.2.5, gives the Sync. Burst layout as
 * tail(3) + encrypted(39) + train_seq(64) + encrypted(39) + tail(3). */
#define SB_OFFSET		(GSM_NBITS_SB_GMSK_TAIL + GSM_NBITS_SB_GMSK_PAYLOAD / 2)
#define SB_SEQ_NUM		4

/* 3GPP TS 45.002, section 5.2.3, gives the Normal Burst (GMSK) layout as
 * tail(3) + encrypted(58) + train_seq(26) + encrypted(58) + tail(3). */
#define NB_OFFSET		(GSM_NBITS_NB_GMSK_TAIL + GSM_NBITS_NB_GMSK_PAYLOAD / 2)
#define NB_TSC_SET_NUM		4
#define NB_TSC_NUM		8

/* 3GPP TS 45.002, section 5.2.3.3, gives the Normal Burst (8-PSK) layout as
 * tail(9) + encrypted(174) + train_seq(78) + encrypted(174) + tail(9). */
#define NB8_OFFSET		(GSM_NBITS_NB_8PSK_TAIL + GSM_NBITS_NB_8PSK_PAYLOAD / 2)
#define NB8_TSC_SET_NUM		2
#define NB8_TSC_NUM		8

/* 3GPP TS 45.002, table 5.2.7-3: Access Burst synch. sequence bits.
 * Indexed by TSC (0, 1, 2, 4 -- 3, 5, 6, 7 are EC-only, see table 5.2.7-4). */
static const ubit_t ab_seq[AB_SEQ_NUM][GSM_NBITS_AB_GMSK_SYNCH_SEQ] = {
	{ /* TSC 0 */
		0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1,
		1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0,
	},
	{ /* TSC 1 */
		0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0,
		0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1,
	},
	{ /* TSC 2 */
		1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1,
		0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1,
	},
	{ /* TSC 4 */
		1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0,
	},
};
static const uint8_t ab_seq_tsc[AB_SEQ_NUM] = { 0, 1, 2, 4 };

/* 3GPP TS 45.002, table 5.2.5-3: Synchronization Burst extended training
 * sequence bits. Indexed by TSC (0..3). */
static const ubit_t sb_seq[SB_SEQ_NUM][GSM_NBITS_SB_GMSK_ETRAIN_SEQ] = {
	{ /* TSC 0 */
		1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1,
		0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1,
		1, 0, 1, 1,
	},
	{ /* TSC 1 */
		1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0,
		1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0,
		0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1,
		0, 1, 0, 1,
	},
	{ /* TSC 2 */
		1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1,
		0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0,
		1, 1, 1, 0,
	},
	{ /* TSC 3 */
		1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
		0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1,
		0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1,
		1, 0, 0, 0,
	},
};

/* 3GPP TS 45.002, tables 5.2.3a-d: Normal Burst training sequence bits.
 * Indexed by [TSC set 0..3][TSC 0..7]. */
static const ubit_t nb_seq[NB_TSC_SET_NUM][NB_TSC_NUM][GSM_NBITS_NB_GMSK_TRAIN_SEQ] = {
	{ /* TSC set 1 */
		{ 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1 }, /* TSC 0 */
		{ 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1 }, /* TSC 1 */
		{ 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0 }, /* TSC 2 */
		{ 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0 }, /* TSC 3 */
		{ 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1 }, /* TSC 4 */
		{ 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0 }, /* TSC 5 */
		{ 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1 }, /* TSC 6 */
		{ 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0 }, /* TSC 7 */
	},
	{ /* TSC set 2 */
		{ 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1 }, /* TSC 0 */
		{ 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1 }, /* TSC 1 */
		{ 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0 }, /* TSC 2 */
		{ 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0 }, /* TSC 3 */
		{ 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0 }, /* TSC 4 */
		{ 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1 }, /* TSC 5 */
		{ 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1 }, /* TSC 6 */
		{ 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1 }, /* TSC 7 */
	},
	{ /* TSC set 3 */
		{ 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0 }, /* TSC 0 */
		{ 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0 }, /* TSC 1 */
		{ 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0 }, /* TSC 2 */
		{ 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0 }, /* TSC 3 */
		{ 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0 }, /* TSC 4 */
		{ 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0 }, /* TSC 5 */
		{ 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0 }, /* TSC 6 */
		{ 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0 }, /* TSC 7 */
	},
	{ /* TSC set 4 */
		{ 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0 }, /* TSC 0 */
		{ 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0 }, /* TSC 1 */
		{ 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0 }, /* TSC 2 */
		{ 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0 }, /* TSC 3 */
		{ 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0 }, /* TSC 4 */
		{ 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0 }, /* TSC 5 */
		{ 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0 }, /* TSC 6 */
		{ 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0 }, /* TSC 7 */
	},
};

/* 3GPP TS 45.002, tables 5.2.3f-g: Normal Burst (8-PSK) training sequence
 * bits. Indexed by [TSC set 0..1][TSC 0..7]. */
static const ubit_t nb8_seq[NB8_TSC_SET_NUM][NB8_TSC_NUM][GSM_NBITS_NB_8PSK_TRAIN_SEQ] = {
	{ /* TSC set 1 */
		{ /* TSC 0 */
			1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1,
			1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1,
			1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
		},
		{ /* TSC 1 */
			1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1,
			1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0,
			0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1,
			0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
		},
		{ /* TSC 2 */
			1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
			1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1,
			1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1,
		},
		{ /* TSC 3 */
			1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0,
			1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0,
			0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1,
		},
		{ /* TSC 4 */
			1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0,
			1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0,
			0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1,
			0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1,
		},
		{ /* TSC 5 */
			1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0,
			1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
			0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1,
		},
		{ /* TSC 6 */
			0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0,
			1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1,
			1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
		},
		{ /* TSC 7 */
			0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0,
			1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1,
			1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1,
			0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
		},
	},
	{ /* TSC set 2 */
		{ /* TSC 0 */
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1,
			1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
			0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1,
			0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1,
		},
		{ /* TSC 1 */
			1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0,
			1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0,
			0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1,
			1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		},
		{ /* TSC 2 */
			0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0,
			1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0,
			0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1,
			1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
		},
		{ /* TSC 3 */
			1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0,
			1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0,
			0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1,
			1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
		},
		{ /* TSC 4 */
			1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0,
			1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
		},
		{ /* TSC 5 */
			1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0,
			1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1,
			1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
		},
		{ /* TSC 6 */
			0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0,
			1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1,
			0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
		},
		{ /* TSC 7 */
			0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0,
			1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0,
			0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1,
		},
	},
};

static void burst_synch_detect_gmsk(struct osmo_trxd_burst_ind *bi,
				    const struct osmo_trxd_burst_req *br)
{
	/* Normal Bursts are by far the most common: check them first */
	for (unsigned int j = 0; j < NB_TSC_SET_NUM; j++) {
		for (unsigned int k = 0; k < NB_TSC_NUM; k++) {
			if (memcmp(&br->burst[NB_OFFSET], nb_seq[j][k],
				   GSM_NBITS_NB_GMSK_TRAIN_SEQ) != 0)
				continue;
			bi->flags |= OSMO_TRXD_F_TS_INFO;
			bi->tsc_set = j;
			bi->tsc = k;
			return;
		}
	}

	for (unsigned int i = 0; i < AB_SEQ_NUM; i++) {
		if (memcmp(&br->burst[GSM_NBITS_AB_GMSK_ETAIL], ab_seq[i],
			   GSM_NBITS_AB_GMSK_SYNCH_SEQ) != 0)
			continue;
		bi->flags |= OSMO_TRXD_F_TS_INFO | OSMO_TRXD_F_ACCESS_BURST;
		bi->tsc_set = 0;
		bi->tsc = ab_seq_tsc[i];
		return;
	}

	/* SCH only ever occurs on TS0 */
	if (br->tn == 0) {
		for (unsigned int i = 0; i < SB_SEQ_NUM; i++) {
			if (memcmp(&br->burst[SB_OFFSET], sb_seq[i],
				   GSM_NBITS_SB_GMSK_ETRAIN_SEQ) != 0)
				continue;
			bi->flags |= OSMO_TRXD_F_TS_INFO;
			bi->tsc_set = 0;
			bi->tsc = i;
			return;
		}
	}
}

static void burst_synch_detect_8psk(struct osmo_trxd_burst_ind *bi,
				    const struct osmo_trxd_burst_req *br)
{
	for (unsigned int j = 0; j < NB8_TSC_SET_NUM; j++) {
		for (unsigned int k = 0; k < NB8_TSC_NUM; k++) {
			if (memcmp(&br->burst[NB8_OFFSET], nb8_seq[j][k],
				   GSM_NBITS_NB_8PSK_TRAIN_SEQ) != 0)
				continue;
			bi->flags |= OSMO_TRXD_F_TS_INFO;
			bi->tsc_set = j;
			bi->tsc = k;
			return;
		}
	}
}

/*! Detect the training/synch. sequence used by the hard-bits of br and
 * fill in bi->{mod,tsc_set,tsc} fields accordingly. */
void burst_synch_detect(struct osmo_trxd_burst_ind *bi,
			const struct osmo_trxd_burst_req *br)
{
	/* NOPE.req has no burst to correlate */
	if (br->flags & OSMO_TRXD_F_NOPE_REQ)
		return;

	/* libosmo-trx's trxd_burst_req_parse_v01() sets br->mod for us */
	if (br->mod == OSMO_TRXD_MOD_T_GMSK)
		burst_synch_detect_gmsk(bi, br);
	else if (br->mod == OSMO_TRXD_MOD_T_8PSK)
		burst_synch_detect_8psk(bi, br);
}
