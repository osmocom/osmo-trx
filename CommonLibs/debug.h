#pragma once

#include <stdbool.h>
#include <sys/types.h>

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
};

pid_t my_gettid(void);

#define CLOGC(category, level, fmt, args...) do { \
	LOGP(category, level, "[tid=%ld] " fmt, (long int) my_gettid(), ##args);  \
} while(0)

#define CLOGCHAN(chan, category, level, fmt, args...) do { \
	LOGP(category, level, "[tid=%ld][chan=%zu] " fmt, (long int) my_gettid(), chan, ##args);  \
} while(0)
