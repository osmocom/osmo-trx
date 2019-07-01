#include <pthread.h>

#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>
#include "debug.h"

/* default categories */
static const struct log_info_cat default_categories[] = {
	[DMAIN] = {
		.name = "DMAIN",
		.description = "Main generic category",
		.color = NULL,
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DTRXCTRL] = {
			.name = "DTRXCTRL",
			.description = "TRX CTRL interface",
			.color = "\033[1;33m",
			.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DDEV] = {
		.name = "DDEV",
		.description = "Device/Driver specific code",
		.color = NULL,
		.enabled = 1, .loglevel = LOGL_INFO,
	},
	[DLMS] = {
		.name = "DLMS",
		.description = "Logging from within LimeSuite itself",
		.color = NULL,
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
};

const struct log_info log_info = {
	.cat = default_categories,
	.num_cat = ARRAY_SIZE(default_categories),
};

pthread_mutex_t log_mutex;

bool log_mutex_init() {
	int rc;
	pthread_mutexattr_t attr;

	if ((rc = pthread_mutexattr_init(&attr))) {
		fprintf(stderr, "pthread_mutexattr_init() failed: %d\n", rc);
		return false;
	}
	if ((rc = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE))) {
		fprintf(stderr, "pthread_mutexattr_settype() failed: %d\n", rc);
		return false;
	}
	if ((rc = pthread_mutex_init(&log_mutex, &attr))) {
		fprintf(stderr, "pthread_mutex_init() failed: %d\n", rc);
		return false;
	}
	if ((rc = pthread_mutexattr_destroy(&attr))) {
		fprintf(stderr, "pthread_mutexattr_destroy() failed: %d\n", rc);
		return false;
	}
	return true;
	/* FIXME: do we need to call pthread_mutex_destroy() during process exit? */
}

/* If called inside a C++ destructor, use log_mutex_(un)lock_canceldisable() APIs instead.
   See osmo-trx commit 86be40b4eb762d5c12e8e3f7388ca9f254e77b36 for more information */
void log_mutex_lock() {
	OSMO_ASSERT(!pthread_mutex_lock(&log_mutex));
}

void log_mutex_unlock() {
	OSMO_ASSERT(!pthread_mutex_unlock(&log_mutex));
}

void log_mutex_lock_canceldisable(int *st) {
	pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, st);
	log_mutex_lock();
}

void log_mutex_unlock_canceldisable(int st) {
	log_mutex_unlock();
	pthread_setcancelstate(st, NULL);
}
