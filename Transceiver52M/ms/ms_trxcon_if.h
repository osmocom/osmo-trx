#pragma once
/*
 * (C) 2023 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
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

#include "ms.h"
extern "C" {
#include <osmocom/bb/trxcon/phyif.h>
}

extern struct trxcon_inst *g_trxcon;
struct internal_q_tx_buf {
	trxcon_phyif_burst_req r;
	uint8_t buf[148];
	internal_q_tx_buf() = default;
	internal_q_tx_buf(const internal_q_tx_buf &) = delete;
	internal_q_tx_buf &operator=(const internal_q_tx_buf &) = default;
	internal_q_tx_buf(const struct trxcon_phyif_burst_req *br) : r(*br)
	{
		memcpy(buf, (void *)br->burst, br->burst_len);
	}
};
using tx_queue_t = spsc_cond<8 * 1, internal_q_tx_buf, true, false>;
using cmd_queue_t = spsc_cond<8 * 1, trxcon_phyif_cmd, true, false>;
using cmdr_queue_t = spsc_cond<8 * 1, trxcon_phyif_rsp, false, false>;
