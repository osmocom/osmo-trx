#pragma once

struct osmo_trxd_burst_ind;
struct osmo_trxd_burst_req;

void burst_synch_detect(struct osmo_trxd_burst_ind *bi,
			const struct osmo_trxd_burst_req *br);
