/*! \file osmocom/trx/trxd.h
 * TRXD (burst data) protocol: I/O-free PDU codec. */
#pragma once

#include <stdint.h>
#include <stddef.h>

#include <osmocom/core/bits.h>

struct msgb;

/*! The highest TRXD PDU version supported by this library */
#define OSMO_TRXD_PDU_VER_MAX		2

/*! GMSK modulated burst: 1 bit per symbol */
#define OSMO_TRXD_BURST_LEN_GMSK	148
/*! 8-PSK modulated burst: 3 bits per symbol */
#define OSMO_TRXD_BURST_LEN_8PSK	444
/*! Maximum burst length in (soft-)bits */
#define OSMO_TRXD_BURST_LEN_MAX		OSMO_TRXD_BURST_LEN_8PSK

/*! Maximum single-PDU header size, in bytes: the TRXDv2 header (largest of
 * all versions, either direction) plus the TDMA FN present on the first
 * PDU of a datagram. */
#define OSMO_TRXD_HDR_LEN_MAX		12
/*! Recommended Rx/Tx socket buffer size, in bytes, for a single
 * (non-batched) TRXD datagram: largest possible header plus the largest
 * possible burst payload. */
#define OSMO_TRXD_MSG_BUF_SIZE		(OSMO_TRXD_HDR_LEN_MAX + OSMO_TRXD_BURST_LEN_MAX)

/*! Modulation types (defined by the TRXDv2 MTS encoding, 3GPP TS 45.002) */
enum osmo_trxd_mod_type {
	OSMO_TRXD_MOD_T_GMSK,
	OSMO_TRXD_MOD_T_8PSK,
	OSMO_TRXD_MOD_T_AQPSK,
	/* room for 16QAM/32QAM */
};

/* Presence/meta flags for osmo_trxd_burst_{ind,req} */
#define OSMO_TRXD_F_NOPE_IND	(1 << 0) /*!< no burst detected / idle indication */
#define OSMO_TRXD_F_MOD_TYPE	(1 << 1) /*!< 'mod' is valid */
#define OSMO_TRXD_F_TS_INFO	(1 << 2) /*!< 'tsc_set'/'tsc' are valid */
#define OSMO_TRXD_F_CI_CB	(1 << 3) /*!< 'ci_cb' is valid */
#define OSMO_TRXD_F_TRX_NUM	(1 << 4) /*!< 'trx_num' is valid */
#define OSMO_TRXD_F_BATCH_IND	(1 << 5) /*!< more PDUs follow in this datagram */
#define OSMO_TRXD_F_SHADOW_IND	(1 << 6) /*!< burst received on a shadow (VAMOS) channel */
#define OSMO_TRXD_F_ACCESS_BURST (1 << 7) /*!< an Access Burst (RACH) */

/*! TRX -> L1: received burst indication (soft bits + measurements).
 *
 * ABI stability rule: mandatory fields come first, optional (flag-gated)
 * fields after them; fields introduced by future TRXD revisions shall be
 * appended at the end of the struct. */
struct osmo_trxd_burst_ind {
	uint32_t flags;		/*!< see OSMO_TRXD_F_* */

	/* Mandatory fields (all TRXD versions) */
	uint32_t fn;		/*!< TDMA frame number */
	uint8_t tn;		/*!< TDMA timeslot number */
	int16_t toa256;		/*!< Timing of Arrival in units of 1/256 of symbol */
	int8_t rssi;		/*!< Received Signal Strength Indication (dBm) */
	sbit_t burst[OSMO_TRXD_BURST_LEN_MAX]; /*!< soft-bits buffer */
	size_t burst_len;	/*!< number of soft-bits (0 for NOPE.ind) */

	/* Optional fields (presence indicated by flags) */
	enum osmo_trxd_mod_type mod; /*!< Modulation type */
	uint8_t tsc_set;	/*!< Training Sequence Set */
	uint8_t tsc;		/*!< Training Sequence Code */
	int16_t ci_cb;		/*!< Carrier-to-Interference ratio (centiBels) */
	uint8_t trx_num;	/*!< TRX (RF channel) number */
};

/*! L1 -> TRX: burst transmit request (hard bits).
 * Same ABI stability rule as for osmo_trxd_burst_ind. */
struct osmo_trxd_burst_req {
	uint32_t flags;		/*!< see OSMO_TRXD_F_* */

	/* Mandatory fields (all TRXD versions) */
	uint32_t fn;		/*!< TDMA frame number */
	uint8_t tn;		/*!< TDMA timeslot number */
	uint8_t att;		/*!< Tx power attenuation (BTS side) */
	ubit_t burst[OSMO_TRXD_BURST_LEN_MAX]; /*!< hard-bits buffer */
	size_t burst_len;	/*!< number of hard-bits */

	/* Optional fields (presence indicated by flags) */
	enum osmo_trxd_mod_type mod; /*!< Modulation type */
	uint8_t tsc_set;	/*!< Training Sequence Set */
	uint8_t tsc;		/*!< Training Sequence Code */
	int8_t scpir;		/*!< SCPIR (AQPSK only, TRXDv2) */
	uint8_t trx_num;	/*!< TRX (RF channel) number */
};

/*! Parser state, keeping cross-PDU context for TRXDv2 batches.
 * Initialize with osmo_trxd_parse_state_init() before parsing each datagram. */
struct osmo_trxd_parse_state {
	uint8_t pdu_ver;	/*!< PDU version (parsed from the first PDU) */
	unsigned int num_pdus;	/*!< number of PDUs parsed so far */
	uint32_t fn;		/*!< TDMA fn of the first PDU (v2 batches omit it later) */
};

void osmo_trxd_parse_state_init(struct osmo_trxd_parse_state *st);

int osmo_trxd_burst_ind_parse(struct osmo_trxd_parse_state *st,
			      struct osmo_trxd_burst_ind *bi,
			      const uint8_t *buf, size_t buf_len);
int osmo_trxd_burst_req_parse(struct osmo_trxd_parse_state *st,
			      struct osmo_trxd_burst_req *br,
			      const uint8_t *buf, size_t buf_len);

int osmo_trxd_burst_ind_build(struct msgb *msg, uint8_t pdu_ver,
			      const struct osmo_trxd_burst_ind *bi);
int osmo_trxd_burst_req_build(struct msgb *msg, uint8_t pdu_ver,
			      const struct osmo_trxd_burst_req *br);
void osmo_trxd_build_fin(struct msgb *msg, uint8_t pdu_ver);

const char *osmo_trxd_burst_ind_name(const struct osmo_trxd_burst_ind *bi);
const char *osmo_trxd_burst_req_name(const struct osmo_trxd_burst_req *br);
extern const struct value_string osmo_trxd_mod_type_names[];
static inline const char *osmo_trxd_mod_type_name(enum osmo_trxd_mod_type mod)
{
	return get_value_string(osmo_trxd_mod_type_names, mod);
}
