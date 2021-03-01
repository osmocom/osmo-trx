#pragma once

#include <stdbool.h>

#include <osmocom/core/logging.h>

extern const struct log_info log_info;

/* Debug Areas of the code */
enum {
	DMAIN,
	DTRXCLK,
	DTRXCTRL,
	DTRXDDL,
	DTRXDUL,
	DDEV,
	DDEVDRV,
	DCTR,
};

#define CLOGCHAN(chan, category, level, fmt, args...) do { \
	LOGP(category, level, "[chan=%zu] " fmt, chan, ##args);  \
} while(0)
