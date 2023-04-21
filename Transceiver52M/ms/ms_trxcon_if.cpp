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

#include <atomic>
#include "ms_trxcon_if.h"
extern "C" {
#include <osmocom/bb/trxcon/trxcon.h>
#include <osmocom/bb/trxcon/l1ctl_server.h>
#include <osmocom/core/panic.h>
}

extern tx_queue_t txq;
extern cmd_queue_t cmdq_to_phy;
extern cmdr_queue_t cmdq_from_phy;
extern std::atomic<bool> g_exit_flag;
// trxcon C call(back) if
extern "C" {
int trxcon_phyif_handle_burst_req(void *phyif, const struct trxcon_phyif_burst_req *br)
{
	if (br->burst_len == 0) // dummy/nope
		return 0;
	OSMO_ASSERT(br->burst != 0);

	internal_q_tx_buf b(br);
	if (!g_exit_flag)
		txq.spsc_push(&b);
	return 0;
}

int trxcon_phyif_handle_cmd(void *phyif, const struct trxcon_phyif_cmd *cmd)
{
#ifdef TXDEBUG
	DBGLG() << "TOP C: " << cmd2str(cmd->type) << std::endl;
#endif
	if (!g_exit_flag)
		cmdq_to_phy.spsc_push(cmd);
	// q for resp polling happens in main loop
	return 0;
}

void trxcon_phyif_close(void *phyif)
{
}

void trxcon_l1ctl_close(struct trxcon_inst *trxcon)
{
	/* Avoid use-after-free: both *fi and *trxcon are children of
	 * the L2IF (L1CTL connection), so we need to re-parent *fi
	 * to NULL before calling l1ctl_client_conn_close(). */
	talloc_steal(NULL, trxcon->fi);
	l1ctl_client_conn_close((struct l1ctl_client *)trxcon->l2if);
}

int trxcon_l1ctl_send(struct trxcon_inst *trxcon, struct msgb *msg)
{
	struct l1ctl_client *l1c = (struct l1ctl_client *)trxcon->l2if;

	return l1ctl_client_send(l1c, msg);
}
}
