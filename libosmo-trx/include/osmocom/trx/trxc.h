/*! \file osmocom/trx/trxc.h
 * TRXC (control) protocol and clock indications: I/O-free message codec. */
#pragma once

#include <stdint.h>
#include <stddef.h>

/*! Maximum length of a command verb (incl. '\0') */
#define OSMO_TRXC_CMD_LEN_MAX		32
/*! Maximum length of the parameters string (incl. '\0') */
#define OSMO_TRXC_PARAMS_LEN_MAX	128
/*! Recommended TRXC socket read/send buffer size */
#define OSMO_TRXC_MSG_BUF_SIZE		1500

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
#define OSMO_TRXC_CMD_ERR		"ERR" /*!< verb of a reject response */

/* Clock socket: "IND CLOCK <fn>" */
int osmo_trxc_clock_ind_parse(uint32_t *fn, const char *buf, size_t len);
int osmo_trxc_clock_ind_build(char *buf, size_t buf_size, uint32_t fn);
