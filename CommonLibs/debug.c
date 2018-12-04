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
