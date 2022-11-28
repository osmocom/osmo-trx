/*
 * (C) 2022 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
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

extern "C" {
#include <osmocom/bb/trxcon/trxcon.h>
#include <osmocom/bb/trxcon/trxcon_fsm.h>
#include <osmocom/bb/trxcon/l1ctl_server.h>
}

static struct l1ctl_server_cfg server_cfg;
static struct l1ctl_server *server = NULL;
namespace trxcon
{
extern struct trxcon_inst *g_trxcon;
}

static int l1ctl_rx_cb(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct trxcon_inst *trxcon = (struct trxcon_inst *)l1c->priv;

	return trxcon_l1ctl_receive(trxcon, msg);
}

static void l1ctl_conn_accept_cb(struct l1ctl_client *l1c)
{
	l1c->log_prefix = talloc_strdup(l1c, trxcon::g_trxcon->log_prefix);
	l1c->priv = trxcon::g_trxcon;
	trxcon::g_trxcon->l2if = l1c;
}

static void l1ctl_conn_close_cb(struct l1ctl_client *l1c)
{
	struct trxcon_inst *trxcon = (struct trxcon_inst *)l1c->priv;

	if (trxcon == NULL || trxcon->fi == NULL)
		return;

	osmo_fsm_inst_dispatch(trxcon->fi, TRXCON_EV_L2IF_FAILURE, NULL);
}

namespace trxcon
{
void trxc_l1ctl_init(void *tallctx)
{
	/* Start the L1CTL server */
	server_cfg = (struct l1ctl_server_cfg){
		.sock_path = "/tmp/osmocom_l2",
		.num_clients_max = 1,
		.conn_read_cb = &l1ctl_rx_cb,
		.conn_accept_cb = &l1ctl_conn_accept_cb,
		.conn_close_cb = &l1ctl_conn_close_cb,
	};

	server = l1ctl_server_alloc(tallctx, &server_cfg);
	if (server == NULL) {
		return;
	}
}
} // namespace trxcon