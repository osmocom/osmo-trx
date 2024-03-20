#pragma once
/*
 * (C) 2024 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
 * All Rights Reserved
 *
 * Author: Eric Wild <ewild@sysmocom.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

struct mssdr_cfg {
	struct {
		bool ul_freq_override;
		bool dl_freq_override;
		bool ul_gain_override;
		bool dl_gain_override;
		double ul_freq;
		double dl_freq;
		double ul_gain;
		double dl_gain;
	} overrides;
	bool use_va;
	bool use_agc;
};

struct mssdr_ctx {
	struct mssdr_cfg cfg;
};

struct mssdr_ctx *vty_mssdr_ctx_alloc(void *talloc_ctx);
int mssdr_vty_init(struct mssdr_ctx *trx);
extern struct vty_app_info g_mssdr_vty_info;
