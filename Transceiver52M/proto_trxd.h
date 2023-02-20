#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>

#include <osmocom/core/endian.h>

#include "debug.h"

#define MAX_RX_BURST_BUF_SIZE 444 /* 444 = EDGE_BURST_NBITS */

enum Modulation {
	MODULATION_GMSK,
	MODULATION_8PSK,
/* Not supported yet:
	MODULATION_AQPSK,
	MODULATION_16QAM,
	MODULATION_32QAM
*/
};

struct trx_ul_burst_ind {
	float rx_burst[MAX_RX_BURST_BUF_SIZE]; /* soft bits normalized 0..1 */
	unsigned nbits; // number of symbols per slot in rxBurst, not counting guard periods
	uint32_t fn; // TDMA frame number
	uint8_t tn; // TDMA time-slot number
	double rssi; // in dBFS
	double toa;  // in symbols
	double noise; // noise level in dBFS
	bool idle; // true if no valid burst is included
	enum Modulation modulation; // modulation type
	uint8_t tss; // training sequence set
	uint8_t tsc; // training sequence code
	float ci; // Carrier-to-Interference ratio, in dB
};

bool trxd_send_burst_ind_v0(size_t chan, int fd, const struct trx_ul_burst_ind *bi);
bool trxd_send_burst_ind_v1(size_t chan, int fd, const struct trx_ul_burst_ind *bi);

/* The latest supported TRXD header format version */
#define TRX_DATA_FORMAT_VER    1

struct trxd_hdr_common {
#if OSMO_IS_LITTLE_ENDIAN
	uint8_t tn:3,
		reserved:1,
		version:4;
#elif OSMO_IS_BIG_ENDIAN
/* auto-generated from the little endian part above (libosmocore/contrib/struct_endianness.py) */
	uint8_t version:4, reserved:1, tn:3;
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

/* Downlink burst (BTS->TRX), v0 anf v1 use same format */
struct trxd_hdr_v01_dl {
	struct trxd_hdr_common common;
	uint8_t tx_att; /* Tx Attentuation */
	uint8_t soft_bits[0];
} __attribute__ ((packed));


#define TRXD_MODULATION_GMSK(ts_set) (0b0000 | (ts_set & 0b0011))
#define TRXD_MODULATION_8PSK(ts_set) (0b0100 | (ts_set & 0b0001))
#define TRXD_MODULATION_AQPSK(ts_set) (0b0110 | (ts_set & 0b0001))
#define TRXD_MODULATION_16QAM(ts_set) (0b1000 | (ts_set & 0b0001))
#define TRXD_MODULATION_32QAM(ts_set) (0b1010 | (ts_set & 0b0001))

struct trxd_hdr_v1_specific {
#if OSMO_IS_LITTLE_ENDIAN
	uint8_t tsc:3,
		modulation:4,
		idle:1;
#elif OSMO_IS_BIG_ENDIAN
/* auto-generated from the little endian part above (libosmocore/contrib/struct_endianness.py) */
	uint8_t idle:1, modulation:4, tsc:3;
#endif
	int16_t ci;  /* big endian, in centiBels */
} __attribute__ ((packed));

struct trxd_hdr_v1 {
	struct trxd_hdr_common common;
	struct trxd_hdr_v0_specific v0;
	struct trxd_hdr_v1_specific v1;
	uint8_t soft_bits[0];
} __attribute__ ((packed));
