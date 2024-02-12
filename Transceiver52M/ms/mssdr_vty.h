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
	// char *bind_addr;
	// char *remote_addr;
	// char *dev_args;
	// unsigned int base_port;
	// unsigned int tx_sps;
	// unsigned int rx_sps;
	// unsigned int rtsc;
	// unsigned int rach_delay;
	// enum ReferenceType clock_ref;
	// enum FillerType filler;
	// bool multi_arfcn;
	// double offset;
	// double freq_offset_khz;
	// double rssi_offset;
	// int ul_fn_offset;
	// bool force_rssi_offset; /* Force value set in VTY? */
	// bool swap_channels;
	// bool ext_rach;
	// bool egprs;
	// unsigned int sched_rr;
	// unsigned int stack_size;
	// unsigned int num_chans;
	// struct trx_chan chans[TRX_CHAN_MAX];
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