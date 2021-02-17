#pragma once

#include <stdbool.h>

#include <osmocom/core/logging.h>
#include <osmocom/core/thread.h>

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

#define CLOGC(category, level, fmt, args...) do { \
	LOGP(category, level, "[tid=%ld] " fmt, (long int) osmo_gettid(), ##args);  \
} while(0)

#define CLOGCHAN(chan, category, level, fmt, args...) do { \
	LOGP(category, level, "[tid=%ld][chan=%zu] " fmt, (long int) osmo_gettid(), chan, ##args);  \
} while(0)
