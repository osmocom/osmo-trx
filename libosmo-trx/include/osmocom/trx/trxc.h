/*! \file osmocom/trx/trxc.h
 * TRXC (control) protocol and clock indications: I/O-free message codec. */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <osmocom/core/utils.h>

/*! Maximum length of a command verb (incl. '\0') */
#define OSMO_TRXC_CMD_LEN_MAX		32
/*! Recommended TRXC socket read/send buffer size */
#define OSMO_TRXC_MSG_BUF_SIZE		1500
/*! Maximum length of the parameters string (incl. '\0').  Must be large
 * enough for SETFH, which carries the whole Mobile Allocation as pairs of
 * Rx/Tx frequencies in kHz (over 1000 characters for 64 ARFCNs).  Sized so
 * that any message ("RSP " + verb + status) still fits the buffer above. */
#define OSMO_TRXC_PARAMS_LEN_MAX	(OSMO_TRXC_MSG_BUF_SIZE - OSMO_TRXC_CMD_LEN_MAX - 32)

enum osmo_trxc_msg_type {
	OSMO_TRXC_MT_CMD,	/*!< "CMD <verb> [<params>]" (L1 -> TRX) */
	OSMO_TRXC_MT_RSP,	/*!< "RSP <verb> <status> [<params>]" (TRX -> L1) */
	OSMO_TRXC_MT_IND,	/*!< "IND <verb> <params>" (TRX -> L1) */
};

/*! A single de-/serialized TRXC message.
 * The command verb set is deliberately open: the codec is verb-agnostic,
 * so dialect specific (e.g. trxcon's ECHO/MEASURE/SETTA) and custom
 * (e.g. fake_trx's FAKE_*) commands need no library changes. */
struct osmo_trxc_msg {
	enum osmo_trxc_msg_type type;
	char cmd[OSMO_TRXC_CMD_LEN_MAX];	/*!< verb, e.g. "POWERON" */
	int status;				/*!< RSP only */
	char params[OSMO_TRXC_PARAMS_LEN_MAX];	/*!< raw parameters, may be "" */
};

int osmo_trxc_msg_parse(struct osmo_trxc_msg *msg, const char *buf, size_t len);
int osmo_trxc_msg_build(char *buf, size_t buf_size, const struct osmo_trxc_msg *msg);
int osmo_trxc_msg_params_scan(const struct osmo_trxc_msg *msg, const char *fmt, ...);
const char *osmo_trxc_msg_name(const struct osmo_trxc_msg *msg);

/* Well-known command verbs (the codec itself is verb-agnostic) */
#define OSMO_TRXC_CMD_POWERON		"POWERON"
#define OSMO_TRXC_CMD_POWEROFF		"POWEROFF"
#define OSMO_TRXC_CMD_RXTUNE		"RXTUNE"
#define OSMO_TRXC_CMD_TXTUNE		"TXTUNE"
#define OSMO_TRXC_CMD_SETSLOT		"SETSLOT"
#define OSMO_TRXC_CMD_SETTSC		"SETTSC"
#define OSMO_TRXC_CMD_SETBSIC		"SETBSIC"
#define OSMO_TRXC_CMD_SETPOWER		"SETPOWER"
#define OSMO_TRXC_CMD_ADJPOWER		"ADJPOWER"
#define OSMO_TRXC_CMD_NOMTXPOWER	"NOMTXPOWER"
#define OSMO_TRXC_CMD_SETRXGAIN		"SETRXGAIN"
#define OSMO_TRXC_CMD_SETMAXDLY		"SETMAXDLY"
#define OSMO_TRXC_CMD_SETMAXDLYNB	"SETMAXDLYNB"
#define OSMO_TRXC_CMD_SETFORMAT		"SETFORMAT"
#define OSMO_TRXC_CMD_HANDOVER		"HANDOVER"
#define OSMO_TRXC_CMD_NOHANDOVER	"NOHANDOVER"
#define OSMO_TRXC_CMD_RFMUTE		"RFMUTE"
#define OSMO_TRXC_CMD_SETFH		"SETFH"
#define OSMO_TRXC_CMD_ERR		"ERR" /*!< verb of a reject response */

/* Clock socket: "IND CLOCK <fn>" */
int osmo_trxc_clock_ind_parse(uint32_t *fn, const char *buf, size_t len);
int osmo_trxc_clock_ind_build(char *buf, size_t buf_size, uint32_t fn);

/* SETSLOT: "<tn> <chan_comb> [C<tsc>/S<tsc_set> ...]" */

/*! Classic (non-VAMOS) GSM TS 05.02 channel combinations, as used by the
 * <chan_comb> parameter of SETSLOT. Numeric values match the wire format. */
enum osmo_trxc_chan_comb {
	OSMO_TRXC_CHAN_COMB_UNUSED		= 0, /*!< Channel is transmitted, but unused */
	OSMO_TRXC_CHAN_COMB_TCHF		= 1,
	OSMO_TRXC_CHAN_COMB_TCHH_IDLE		= 2, /*!< TCH/HS, idle every other slot */
	OSMO_TRXC_CHAN_COMB_TCHH		= 3,
	OSMO_TRXC_CHAN_COMB_BCCH		= 4, /*!< DL: FCCH+SCH+CCCH+BCCH, UL: RACH */
	OSMO_TRXC_CHAN_COMB_BCCH_SDCCH4	= 5, /*!< DL: +SDCCH/4+SACCH/4, UL: +SDCCH/4 */
	OSMO_TRXC_CHAN_COMB_CCCH		= 6, /*!< DL: CCCH+BCCH, UL: RACH */
	OSMO_TRXC_CHAN_COMB_SDCCH8		= 7, /*!< SDCCH/8 + SACCH/8 */
	OSMO_TRXC_CHAN_COMB_TCHF_FACCH_SACCHM	= 8,
	OSMO_TRXC_CHAN_COMB_TCHF_SACCHM	= 9,
	OSMO_TRXC_CHAN_COMB_TCHFD_SACCHMD	= 10,
	OSMO_TRXC_CHAN_COMB_PBCCH		= 11, /*!< PBCCH+PCCCH+PDTCH+PACCH+PTCCH */
	OSMO_TRXC_CHAN_COMB_PCCCH		= 12, /*!< PCCCH+PDTCH+PACCH+PTCCH */
	OSMO_TRXC_CHAN_COMB_PDTCH		= 13, /*!< PDTCH+PACCH+PTCCH */
};

/*! VAMOS-enabled channel combinations: the <chan_comb> parameter of SETSLOT
 * is symbolic (not numeric) for these. */
enum osmo_trxc_vamos_comb {
	OSMO_TRXC_VAMOS_COMB_VFF = 1,	/*!< V0(TCH/F) & V1(TCH/F) */
	OSMO_TRXC_VAMOS_COMB_VHH,	/*!< V0(TCH/H0)&V1(TCH/H0) + V0(TCH/H1)&V1(TCH/H1) */
	OSMO_TRXC_VAMOS_COMB_VFH,	/*!< V0(TCH/F) & V1(TCH/H0) + V0(TCH/F) & V1(TCH/H1) */
	OSMO_TRXC_VAMOS_COMB_HVHH,	/*!< TCH/H0 + V0(TCH/H1) & V1(TCH/H1) (mixed) */
};

extern const struct value_string osmo_trxc_vamos_comb_names[];
static inline const char *osmo_trxc_vamos_comb_name(enum osmo_trxc_vamos_comb comb)
{
	return get_value_string(osmo_trxc_vamos_comb_names, comb);
}

#define OSMO_TRXC_SETSLOT_TSC_MAX	3 /*!< up to 3 sub-channels (VAMOS "HVHH") */

/*! One "C<tsc>/S<tsc_set>" override, as used by SETSLOT for (VAMOS)
 * sub-channels that don't use the endpoint-wide TSC (SETTSC). */
struct osmo_trxc_setslot_tsc {
	uint8_t tsc;
	uint8_t tsc_set;
};

/*! Parsed SETSLOT parameters. */
struct osmo_trxc_setslot {
	uint8_t tn;
	bool vamos;	/*!< false: chan_comb is valid, true: vamos_comb is valid */
	union {
		enum osmo_trxc_chan_comb chan_comb;
		enum osmo_trxc_vamos_comb vamos_comb;
	};
	unsigned int num_tsc;	/*!< number of valid entries in tsc[] */
	struct osmo_trxc_setslot_tsc tsc[OSMO_TRXC_SETSLOT_TSC_MAX];
};

int osmo_trxc_setslot_parse(struct osmo_trxc_setslot *ss, const struct osmo_trxc_msg *msg);
int osmo_trxc_setslot_build(char *buf, size_t buf_size, const struct osmo_trxc_setslot *ss);
