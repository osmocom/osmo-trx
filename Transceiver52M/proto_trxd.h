#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>

#include <osmocom/core/endian.h>

#include "debug.h"

#define MAX_RX_BURST_BUF_SIZE 444 /* 444 = EDGE_BURST_NBITS */

struct trx_ul_burst_ind {
	float rx_burst[MAX_RX_BURST_BUF_SIZE]; /* soft bits normalized 0..1 */
	unsigned nbits; // number of symbols per slot in rxBurst, not counting guard periods
	uint32_t fn; // TDMA frame number
	uint8_t tn; // TDMA time-slot number
	double rssi; // in dBFS
	double toa;  // in symbols
	double noise; // noise level in dBFS
	bool idle; // true if no valid burst is included
};

bool trxd_send_burst_ind_v0(size_t chan, int fd, const struct trx_ul_burst_ind *bi);

/* The latest supported TRXD header format version */
#define TRX_DATA_FORMAT_VER    0

struct trxd_hdr_common {
#if OSMO_IS_LITTLE_ENDIAN
	uint8_t tn:3,
		reserved:1,
		version:4;
#elif OSMO_IS_BIG_ENDIAN
	uint8_t version:4,
		reserved:1,
		tn:3;
#endif
	uint32_t fn; /* big endian */
} __attribute__ ((packed));

struct trxd_hdr_v0_specific {
	uint8_t rssi;
	uint16_t toa; /* big endian */
} __attribute__ ((packed));

struct trxd_hdr_v0 {
	struct trxd_hdr_common common;
	struct trxd_hdr_v0_specific v0;
	uint8_t soft_bits[0];
} __attribute__ ((packed));
