#pragma once

#include <stdbool.h>
#include <pthread.h>

#include <osmocom/core/logging.h>

extern const struct log_info log_info;

/* Debug Areas of the code */
enum {
	DMAIN,
	DTRXCTRL,
	DDEV,
	DLMS,
};

#define CLOGC(category, level, fmt, args...) do { \
	LOGP(category, level, "[tid=%lu] " fmt, pthread_self(), ##args);  \
} while(0)

#define CLOGCHAN(chan, category, level, fmt, args...) do { \
	LOGP(category, level, "[tid=%lu][chan=%lu] " fmt, pthread_self(), chan, ##args);  \
} while(0)
