#pragma once

#include <osmocom/core/logging.h>

extern const struct log_info log_info;

enum {
	DPROXY,		/* main/vty/config */
	DTRXC,		/* TRXC (control) + clock handling */
	DTRXD,		/* TRXD (burst data) handling / forwarding */
};
