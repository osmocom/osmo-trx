#pragma once

#include <osmocom/vty/command.h>

extern struct vty_app_info g_vty_info;

struct trx_ctx {
	struct {
		char *bind_addr;
	} cfg;
};

int trx_vty_init(struct trx_ctx* trx);
