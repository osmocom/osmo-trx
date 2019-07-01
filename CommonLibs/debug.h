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


bool log_mutex_init();
void log_mutex_lock();
void log_mutex_unlock();
void log_mutex_lock_canceldisable(int *st);
void log_mutex_unlock_canceldisable(int st);

#define CLOGC(category, level, fmt, args...) do { \
	log_mutex_lock(); \
	LOGP(category, level, "[tid=%lu] " fmt, pthread_self(), ##args);  \
	log_mutex_unlock(); \
} while(0)

#define CLOGCHAN(chan, category, level, fmt, args...) do { \
	log_mutex_lock(); \
	LOGP(category, level, "[tid=%lu][chan=%lu] " fmt, pthread_self(), chan, ##args);  \
	log_mutex_unlock(); \
} while(0)
