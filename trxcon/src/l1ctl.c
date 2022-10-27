/*
 * OsmocomBB <-> SDR connection bridge
 * GSM L1 control interface handlers
 *
 * (C) 2014 by Sylvain Munaut <tnt@246tNt.com>
 * (C) 2016-2022 by Vadim Yanitskiy <axilirator@gmail.com>
 * Contributions by sysmocom - s.f.m.c. GmbH
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <arpa/inet.h>

#include <osmocom/core/fsm.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/select.h>

#include <osmocom/gsm/gsm0502.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/protocol/gsm_08_58.h>

#include <osmocom/bb/trxcon/logging.h>
#include <osmocom/bb/trxcon/l1ctl_server.h>
#include <osmocom/bb/trxcon/l1ctl_proto.h>
#include <osmocom/bb/trxcon/trxcon.h>

static const char *arfcn2band_name(uint16_t arfcn)
{
	enum gsm_band band;

	if (gsm_arfcn2band_rc(arfcn, &band) < 0)
		return "(invalid)";

	return gsm_band_name(band);
}

static struct msgb *l1ctl_alloc_msg(uint8_t msg_type)
{
	struct l1ctl_hdr *l1h;
	struct msgb *msg;

	/**
	 * Each L1CTL message gets its own length pushed in front
	 * before sending. This is why we need this small headroom.
	 */
	msg = msgb_alloc_headroom(L1CTL_LENGTH + L1CTL_MSG_LEN_FIELD,
		L1CTL_MSG_LEN_FIELD, "l1ctl_tx_msg");
	if (!msg) {
		LOGP(DL1C, LOGL_ERROR, "Failed to allocate memory\n");
		return NULL;
	}

	msg->l1h = msgb_put(msg, sizeof(*l1h));
	l1h = (struct l1ctl_hdr *) msg->l1h;
	l1h->msg_type = msg_type;

	return msg;
}

int l1ctl_tx_pm_conf(struct l1ctl_client *l1c, uint16_t band_arfcn,
	int dbm, int last)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_pm_conf *pmc;
	struct msgb *msg;

	msg = l1ctl_alloc_msg(L1CTL_PM_CONF);
	if (!msg)
		return -ENOMEM;

	LOGPFSMSL(fi, DL1C, LOGL_DEBUG,
		  "Send PM Conf (%s %d = %d dBm)\n",
		  arfcn2band_name(band_arfcn),
		  band_arfcn & ~ARFCN_FLAG_MASK, dbm);

	pmc = (struct l1ctl_pm_conf *) msgb_put(msg, sizeof(*pmc));
	pmc->band_arfcn = htons(band_arfcn);
	pmc->pm[0] = dbm2rxlev(dbm);
	pmc->pm[1] = 0;

	if (last) {
		struct l1ctl_hdr *l1h = (struct l1ctl_hdr *) msg->l1h;
		l1h->flags |= L1CTL_F_DONE;
	}

	return l1ctl_client_send(l1c, msg);
}

int l1ctl_tx_reset_ind(struct l1ctl_client *l1c, uint8_t type)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct msgb *msg;
	struct l1ctl_reset *res;

	msg = l1ctl_alloc_msg(L1CTL_RESET_IND);
	if (!msg)
		return -ENOMEM;

	LOGPFSMSL(fi, DL1C, LOGL_DEBUG, "Send Reset Ind (%u)\n", type);

	res = (struct l1ctl_reset *) msgb_put(msg, sizeof(*res));
	res->type = type;

	return l1ctl_client_send(l1c, msg);
}

int l1ctl_tx_reset_conf(struct l1ctl_client *l1c, uint8_t type)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct msgb *msg;
	struct l1ctl_reset *res;

	msg = l1ctl_alloc_msg(L1CTL_RESET_CONF);
	if (!msg)
		return -ENOMEM;

	LOGPFSMSL(fi, DL1C, LOGL_DEBUG, "Send Reset Conf (%u)\n", type);
	res = (struct l1ctl_reset *) msgb_put(msg, sizeof(*res));
	res->type = type;

	return l1ctl_client_send(l1c, msg);
}

static struct l1ctl_info_dl *put_dl_info_hdr(struct msgb *msg,
					     const struct l1ctl_info_dl *dl_info)
{
	size_t len = sizeof(struct l1ctl_info_dl);
	struct l1ctl_info_dl *dl = (struct l1ctl_info_dl *) msgb_put(msg, len);

	if (dl_info) /* Copy DL info provided by handler */
		memcpy(dl, dl_info, len);
	else /* Init DL info header */
		memset(dl, 0x00, len);

	return dl;
}

/* Fill in FBSB payload: BSIC and sync result */
static struct l1ctl_fbsb_conf *fbsb_conf_make(struct msgb *msg, uint8_t result, uint8_t bsic)
{
	struct l1ctl_fbsb_conf *conf = (struct l1ctl_fbsb_conf *) msgb_put(msg, sizeof(*conf));

	conf->result = result;
	conf->bsic = bsic;

	return conf;
}

int l1ctl_tx_fbsb_fail(struct l1ctl_client *l1c, uint16_t band_arfcn)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_info_dl *dl;
	struct msgb *msg;

	msg = l1ctl_alloc_msg(L1CTL_FBSB_CONF);
	if (msg == NULL)
		return -ENOMEM;

	dl = put_dl_info_hdr(msg, NULL);

	/* Fill in current ARFCN */
	dl->band_arfcn = htons(band_arfcn);

	fbsb_conf_make(msg, 255, 0);

	LOGPFSMSL(fi, DL1C, LOGL_DEBUG, "Send FBSB Conf (timeout)\n");

	return l1ctl_client_send(l1c, msg);
}

int l1ctl_tx_fbsb_conf(struct l1ctl_client *l1c, uint16_t band_arfcn, uint8_t bsic)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_fbsb_conf *conf;
	struct l1ctl_info_dl *dl;
	struct msgb *msg;

	msg = l1ctl_alloc_msg(L1CTL_FBSB_CONF);
	if (msg == NULL)
		return -ENOMEM;

	dl = put_dl_info_hdr(msg, NULL);

	/* Fill in current ARFCN */
	dl->band_arfcn = htons(band_arfcn);

	conf = fbsb_conf_make(msg, 0, bsic);

	/* FIXME: set proper value */
	conf->initial_freq_err = 0;

	LOGPFSMSL(fi, DL1C, LOGL_DEBUG,
		  "Send FBSB Conf (result=%u, bsic=%u)\n",
		  conf->result, conf->bsic);

	return l1ctl_client_send(l1c, msg);
}

int l1ctl_tx_ccch_mode_conf(struct l1ctl_client *l1c, uint8_t mode)
{
	struct l1ctl_ccch_mode_conf *conf;
	struct msgb *msg;

	msg = l1ctl_alloc_msg(L1CTL_CCCH_MODE_CONF);
	if (msg == NULL)
		return -ENOMEM;

	conf = (struct l1ctl_ccch_mode_conf *) msgb_put(msg, sizeof(*conf));
	conf->ccch_mode = mode;

	return l1ctl_client_send(l1c, msg);
}

/**
 * Handles both L1CTL_DATA_IND and L1CTL_TRAFFIC_IND.
 */
int l1ctl_tx_dt_ind(struct l1ctl_client *l1c,
		    const struct l1ctl_info_dl *dl_info,
		    const uint8_t *l2, size_t l2_len,
		    bool traffic)
{
	struct msgb *msg;
	uint8_t *msg_l2;

	msg = l1ctl_alloc_msg(traffic ?
		L1CTL_TRAFFIC_IND : L1CTL_DATA_IND);
	if (msg == NULL)
		return -ENOMEM;

	put_dl_info_hdr(msg, dl_info);

	/* Copy the L2 payload if preset */
	if (l2 && l2_len > 0) {
		msg_l2 = (uint8_t *) msgb_put(msg, l2_len);
		memcpy(msg_l2, l2, l2_len);
	}

	/* Put message to upper layers */
	return l1ctl_client_send(l1c, msg);
}

int l1ctl_tx_rach_conf(struct l1ctl_client *l1c,
	uint16_t band_arfcn, uint32_t fn)
{
	struct l1ctl_info_dl *dl;
	struct msgb *msg;

	msg = l1ctl_alloc_msg(L1CTL_RACH_CONF);
	if (msg == NULL)
		return -ENOMEM;

	dl = put_dl_info_hdr(msg, NULL);
	memset(dl, 0x00, sizeof(*dl));

	dl->band_arfcn = htons(band_arfcn);
	dl->frame_nr = htonl(fn);

	return l1ctl_client_send(l1c, msg);
}


/**
 * Handles both L1CTL_DATA_CONF and L1CTL_TRAFFIC_CONF.
 */
int l1ctl_tx_dt_conf(struct l1ctl_client *l1c,
	struct l1ctl_info_dl *data, bool traffic)
{
	struct msgb *msg;

	msg = l1ctl_alloc_msg(traffic ?
		L1CTL_TRAFFIC_CONF : L1CTL_DATA_CONF);
	if (msg == NULL)
		return -ENOMEM;

	/* Copy DL frame header from source message */
	put_dl_info_hdr(msg, data);

	return l1ctl_client_send(l1c, msg);
}

static enum gsm_phys_chan_config l1ctl_ccch_mode2pchan_config(enum ccch_mode mode)
{
	switch (mode) {
	/* TODO: distinguish extended BCCH */
	case CCCH_MODE_NON_COMBINED:
	case CCCH_MODE_NONE:
		return GSM_PCHAN_CCCH;

	case CCCH_MODE_COMBINED:
		return GSM_PCHAN_CCCH_SDCCH4;
	case CCCH_MODE_COMBINED_CBCH:
		return GSM_PCHAN_CCCH_SDCCH4_CBCH;

	default:
		LOGP(DL1C, LOGL_NOTICE, "Undandled CCCH mode (%u), "
			"assuming non-combined configuration\n", mode);
		return GSM_PCHAN_CCCH;
	}
}

static int l1ctl_rx_fbsb_req(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_fbsb_req *fbsb;
	int rc = 0;

	fbsb = (struct l1ctl_fbsb_req *) msg->l1h;
	if (msgb_l1len(msg) < sizeof(*fbsb)) {
		LOGPFSMSL(fi, DL1C, LOGL_ERROR,
			  "MSG too short FBSB Req: %u\n",
			  msgb_l1len(msg));
		rc = -EINVAL;
		goto exit;
	}

	struct trxcon_param_fbsb_search_req req = {
		.pchan_config = l1ctl_ccch_mode2pchan_config(fbsb->ccch_mode),
		.timeout_ms = ntohs(fbsb->timeout) * GSM_TDMA_FN_DURATION_uS / 1000,
		.band_arfcn = ntohs(fbsb->band_arfcn),
	};

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
		  "Received FBSB request (%s %d, timeout %u ms)\n",
		  arfcn2band_name(req.band_arfcn),
		  req.band_arfcn & ~ARFCN_FLAG_MASK,
		  req.timeout_ms);

	osmo_fsm_inst_dispatch(fi, TRXCON_EV_FBSB_SEARCH_REQ, &req);

exit:
	msgb_free(msg);
	return rc;
}

static int l1ctl_rx_pm_req(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_pm_req *pmr;
	int rc = 0;

	pmr = (struct l1ctl_pm_req *) msg->l1h;
	if (msgb_l1len(msg) < sizeof(*pmr)) {
		LOGPFSMSL(fi, DL1C, LOGL_ERROR,
			  "MSG too short PM Req: %u\n",
			  msgb_l1len(msg));
		rc = -EINVAL;
		goto exit;
	}

	struct trxcon_param_full_power_scan_req req = {
		.band_arfcn_start = ntohs(pmr->range.band_arfcn_from),
		.band_arfcn_stop = ntohs(pmr->range.band_arfcn_to),
	};

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
		  "Received power measurement request (%s: %d -> %d)\n",
		  arfcn2band_name(req.band_arfcn_start),
		  req.band_arfcn_start & ~ARFCN_FLAG_MASK,
		  req.band_arfcn_stop & ~ARFCN_FLAG_MASK);

	osmo_fsm_inst_dispatch(fi, TRXCON_EV_FULL_POWER_SCAN_REQ, &req);

exit:
	msgb_free(msg);
	return rc;
}

static int l1ctl_rx_reset_req(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_reset *res;
	int rc = 0;

	res = (struct l1ctl_reset *) msg->l1h;
	if (msgb_l1len(msg) < sizeof(*res)) {
		LOGPFSMSL(fi, DL1C, LOGL_ERROR,
			  "MSG too short Reset Req: %u\n",
			  msgb_l1len(msg));
		rc = -EINVAL;
		goto exit;
	}

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
		  "Received reset request (%u)\n", res->type);

	switch (res->type) {
	case L1CTL_RES_T_FULL:
		osmo_fsm_inst_dispatch(fi, TRXCON_EV_RESET_FULL_REQ, NULL);
		break;
	case L1CTL_RES_T_SCHED:
		osmo_fsm_inst_dispatch(fi, TRXCON_EV_RESET_SCHED_REQ, NULL);
		break;
	default:
		LOGPFSMSL(fi, DL1C, LOGL_ERROR,
			  "Unknown L1CTL_RESET_REQ type\n");
		goto exit;
	}

	/* Confirm */
	rc = l1ctl_tx_reset_conf(l1c, res->type);

exit:
	msgb_free(msg);
	return rc;
}

static int l1ctl_rx_echo_req(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_hdr *l1h;

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE, "Recv Echo Req\n");
	LOGPFSMSL(fi, DL1C, LOGL_NOTICE, "Send Echo Conf\n");

	/* Nothing to do, just send it back */
	l1h = (struct l1ctl_hdr *) msg->l1h;
	l1h->msg_type = L1CTL_ECHO_CONF;
	msg->data = msg->l1h;

	return l1ctl_client_send(l1c, msg);
}

static int l1ctl_rx_ccch_mode_req(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_ccch_mode_req *mode_req;
	int rc;

	mode_req = (struct l1ctl_ccch_mode_req *)msg->l1h;
	if (msgb_l1len(msg) < sizeof(*mode_req)) {
		LOGPFSMSL(fi, DL1C, LOGL_ERROR,
			  "MSG too short Reset Req: %u\n",
			  msgb_l1len(msg));
		rc = -EINVAL;
		goto exit;
	}

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE, "Received CCCH mode request (%u)\n",
		  mode_req->ccch_mode); /* TODO: add value-string for ccch_mode */

	struct trxcon_param_set_ccch_tch_mode_req req = {
		/* Choose corresponding channel combination */
		.mode = l1ctl_ccch_mode2pchan_config(mode_req->ccch_mode),
	};

	rc = osmo_fsm_inst_dispatch(fi, TRXCON_EV_SET_CCCH_MODE_REQ, &req);
	if (rc == 0 && req.applied)
		l1ctl_tx_ccch_mode_conf(l1c, mode_req->ccch_mode);

exit:
	msgb_free(msg);
	return rc;
}

static int l1ctl_rx_rach_req(struct l1ctl_client *l1c, struct msgb *msg, bool is_11bit)
{
	struct trxcon_param_tx_access_burst_req req;
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_info_ul *ul;

	ul = (struct l1ctl_info_ul *) msg->l1h;

	if (is_11bit) {
		const struct l1ctl_ext_rach_req *rr = (void *)ul->payload;

		req = (struct trxcon_param_tx_access_burst_req) {
			.offset = ntohs(rr->offset),
			.synch_seq = rr->synch_seq,
			.ra = ntohs(rr->ra11),
			.is_11bit = true,
		};

		LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
			  "Received 11-bit RACH request "
			  "(offset=%u, synch_seq=%u, ra11=0x%02hx)\n",
			  req.offset, req.synch_seq, req.ra);
	} else {
		const struct l1ctl_rach_req *rr = (void *)ul->payload;

		req = (struct trxcon_param_tx_access_burst_req) {
			.offset = ntohs(rr->offset),
			.ra = rr->ra,
		};

		LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
			  "Received 8-bit RACH request "
			  "(offset=%u, ra=0x%02x)\n", req.offset, req.ra);
	}

	/* The controlling L1CTL side always does include the UL info header,
	 * but may leave it empty. We assume RACH is on TS0 in this case. */
	if (ul->chan_nr == 0x00) {
		LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
			  "The UL info header is empty, assuming RACH is on TS0\n");
		req.chan_nr = RSL_CHAN_RACH;
		req.link_id = 0x00;
	} else {
		req.chan_nr = ul->chan_nr;
		req.link_id = ul->link_id;
	}

	osmo_fsm_inst_dispatch(fi, TRXCON_EV_TX_ACCESS_BURST_REQ, &req);

	msgb_free(msg);
	return 0;
}

static int l1ctl_proc_est_req_h0(struct osmo_fsm_inst *fi,
				 struct trxcon_param_dedicated_establish_req *req,
				 const struct l1ctl_h0 *h)
{
	req->h0.band_arfcn = ntohs(h->band_arfcn);

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
		  "L1CTL_DM_EST_REQ indicates single ARFCN %s %u\n",
		  arfcn2band_name(req->h0.band_arfcn),
		  req->h0.band_arfcn & ~ARFCN_FLAG_MASK);

	return 0;
}

static int l1ctl_proc_est_req_h1(struct osmo_fsm_inst *fi,
				 struct trxcon_param_dedicated_establish_req *req,
				 const struct l1ctl_h1 *h)
{
	unsigned int i;

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
		  "L1CTL_DM_EST_REQ indicates a Frequency "
		  "Hopping (hsn=%u, maio=%u, chans=%u) channel\n",
		  h->hsn, h->maio, h->n);

	/* No channels?!? */
	if (!h->n) {
		LOGPFSMSL(fi, DL1C, LOGL_ERROR,
			  "No channels in mobile allocation?!?\n");
		return -EINVAL;
	} else if (h->n > ARRAY_SIZE(h->ma)) {
		LOGPFSMSL(fi, DL1C, LOGL_ERROR,
			  "More than 64 channels in mobile allocation?!?\n");
		return -EINVAL;
	}

	/* Convert from network to host byte order */
	for (i = 0; i < h->n; i++)
		req->h1.ma[i] = ntohs(h->ma[i]);
	req->h1.n = h->n;
	req->h1.hsn = h->hsn;
	req->h1.maio = h->maio;

	return 0;
}

static int l1ctl_rx_dm_est_req(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_dm_est_req *est_req;
	struct l1ctl_info_ul *ul;
	int rc;

	ul = (struct l1ctl_info_ul *) msg->l1h;
	est_req = (struct l1ctl_dm_est_req *) ul->payload;

	struct trxcon_param_dedicated_establish_req req = {
		.chan_nr = ul->chan_nr,
		.tch_mode = est_req->tch_mode,
		.tsc = est_req->tsc,
		.hopping = est_req->h,
	};

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
		  "Received L1CTL_DM_EST_REQ "
		  "(tn=%u, chan_nr=0x%02x, tsc=%u, tch_mode=0x%02x)\n",
		  req.chan_nr & 0x07, req.chan_nr, req.tsc, req.tch_mode);

	/* Frequency hopping? */
	if (est_req->h)
		rc = l1ctl_proc_est_req_h1(fi, &req, &est_req->h1);
	else /* Single ARFCN */
		rc = l1ctl_proc_est_req_h0(fi, &req, &est_req->h0);
	if (rc)
		goto exit;

	osmo_fsm_inst_dispatch(fi, TRXCON_EV_DEDICATED_ESTABLISH_REQ, &req);

exit:
	msgb_free(msg);
	return rc;
}

static int l1ctl_rx_dm_rel_req(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct osmo_fsm_inst *fi = l1c->priv;

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE, "Received L1CTL_DM_REL_REQ\n");

	osmo_fsm_inst_dispatch(fi, TRXCON_EV_DEDICATED_RELEASE_REQ, NULL);

	msgb_free(msg);
	return 0;
}

/**
 * Handles both L1CTL_DATA_REQ and L1CTL_TRAFFIC_REQ.
 */
static int l1ctl_rx_dt_req(struct l1ctl_client *l1c,
	struct msgb *msg, bool traffic)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_info_ul *ul;

	/* Extract UL frame header */
	ul = (struct l1ctl_info_ul *) msg->l1h;
	msg->l2h = ul->payload;

	struct trxcon_param_tx_traffic_data_req req = {
		.chan_nr = ul->chan_nr,
		.link_id = ul->link_id & 0x40,
		.data_len = msgb_l2len(msg),
		.data = ul->payload,
	};

	LOGPFSMSL(fi, DL1D, LOGL_DEBUG,
		  "Recv %s Req (chan_nr=0x%02x, link_id=0x%02x, len=%zu)\n",
		  traffic ? "TRAFFIC" : "DATA", req.chan_nr, req.link_id, req.data_len);

	switch (fi->state) {
	case TRXCON_ST_DEDICATED:
		if (traffic)
			osmo_fsm_inst_dispatch(fi, TRXCON_EV_TX_TRAFFIC_REQ, &req);
		else
			osmo_fsm_inst_dispatch(fi, TRXCON_EV_TX_DATA_REQ, &req);
		break;
	default:
		if (!traffic && req.link_id == 0x40) /* only for SACCH */
			osmo_fsm_inst_dispatch(fi, TRXCON_EV_UPDATE_SACCH_CACHE_REQ, &req);
		/* TODO: log an error about uhnandled DATA.req / TRAFFIC.req */
	}

	msgb_free(msg);
	return 0;
}

static int l1ctl_rx_param_req(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_par_req *par_req;
	struct l1ctl_info_ul *ul;

	ul = (struct l1ctl_info_ul *) msg->l1h;
	par_req = (struct l1ctl_par_req *) ul->payload;

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
		  "Received L1CTL_PARAM_REQ (ta=%d, tx_power=%u)\n",
		  par_req->ta, par_req->tx_power);

	struct trxcon_param_set_phy_config_req req = {
		.type = TRXCON_PHY_CFGT_TX_PARAMS,
		.tx_params = {
			.timing_advance = par_req->ta,
			.tx_power = par_req->tx_power,
		}
	};

	osmo_fsm_inst_dispatch(fi, TRXCON_EV_SET_PHY_CONFIG_REQ, &req);

	msgb_free(msg);
	return 0;
}

static int l1ctl_rx_tch_mode_req(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_tch_mode_req *mode_req;
	int rc;

	mode_req = (struct l1ctl_tch_mode_req *)msg->l1h;

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
		  "Received L1CTL_TCH_MODE_REQ (tch_mode=%u, audio_mode=%u)\n",
		  mode_req->tch_mode, mode_req->audio_mode);

	/* TODO: do we need to care about audio_mode? */

	struct trxcon_param_set_ccch_tch_mode_req req = {
		.mode = mode_req->tch_mode,
	};
	if (mode_req->tch_mode == GSM48_CMODE_SPEECH_AMR) {
		req.amr.start_codec = mode_req->amr.start_codec;
		req.amr.codecs_bitmask = mode_req->amr.codecs_bitmask;
	}

	rc = osmo_fsm_inst_dispatch(fi, TRXCON_EV_SET_TCH_MODE_REQ, &req);
	if (rc != 0 || !req.applied) {
		talloc_free(msg);
		return rc;
	}

	/* Re-use the original message as confirmation */
	struct l1ctl_hdr *l1h = (struct l1ctl_hdr *) msg->data;
	l1h->msg_type = L1CTL_TCH_MODE_CONF;

	return l1ctl_client_send(l1c, msg);
}

static int l1ctl_rx_crypto_req(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_crypto_req *cr;
	struct l1ctl_info_ul *ul;

	ul = (struct l1ctl_info_ul *) msg->l1h;
	cr = (struct l1ctl_crypto_req *) ul->payload;

	struct trxcon_param_crypto_req req = {
		.chan_nr = ul->chan_nr,
		.a5_algo = cr->algo,
		.key_len = cr->key_len,
		.key = cr->key,
	};

	LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
		  "L1CTL_CRYPTO_REQ (algo=A5/%u, key_len=%u)\n",
		  req.a5_algo, req.key_len);

	osmo_fsm_inst_dispatch(fi, TRXCON_EV_CRYPTO_REQ, &req);

	msgb_free(msg);
	return 0;
}

int l1ctl_rx_cb(struct l1ctl_client *l1c, struct msgb *msg)
{
	struct osmo_fsm_inst *fi = l1c->priv;
	struct l1ctl_hdr *l1h;

	l1h = (struct l1ctl_hdr *) msg->l1h;
	msg->l1h = l1h->data;

	switch (l1h->msg_type) {
	case L1CTL_FBSB_REQ:
		return l1ctl_rx_fbsb_req(l1c, msg);
	case L1CTL_PM_REQ:
		return l1ctl_rx_pm_req(l1c, msg);
	case L1CTL_RESET_REQ:
		return l1ctl_rx_reset_req(l1c, msg);
	case L1CTL_ECHO_REQ:
		return l1ctl_rx_echo_req(l1c, msg);
	case L1CTL_CCCH_MODE_REQ:
		return l1ctl_rx_ccch_mode_req(l1c, msg);
	case L1CTL_RACH_REQ:
		return l1ctl_rx_rach_req(l1c, msg, false);
	case L1CTL_EXT_RACH_REQ:
		return l1ctl_rx_rach_req(l1c, msg, true);
	case L1CTL_DM_EST_REQ:
		return l1ctl_rx_dm_est_req(l1c, msg);
	case L1CTL_DM_REL_REQ:
		return l1ctl_rx_dm_rel_req(l1c, msg);
	case L1CTL_DATA_REQ:
		return l1ctl_rx_dt_req(l1c, msg, false);
	case L1CTL_TRAFFIC_REQ:
		return l1ctl_rx_dt_req(l1c, msg, true);
	case L1CTL_PARAM_REQ:
		return l1ctl_rx_param_req(l1c, msg);
	case L1CTL_TCH_MODE_REQ:
		return l1ctl_rx_tch_mode_req(l1c, msg);
	case L1CTL_CRYPTO_REQ:
		return l1ctl_rx_crypto_req(l1c, msg);

	/* Not (yet) handled messages */
	case L1CTL_NEIGH_PM_REQ:
	case L1CTL_DATA_TBF_REQ:
	case L1CTL_TBF_CFG_REQ:
	case L1CTL_DM_FREQ_REQ:
	case L1CTL_SIM_REQ:
		LOGPFSMSL(fi, DL1C, LOGL_NOTICE,
			  "Ignoring unsupported message (type=%u)\n",
			  l1h->msg_type);
		msgb_free(msg);
		return -ENOTSUP;
	default:
		LOGPFSMSL(fi, DL1C, LOGL_ERROR, "Unknown MSG type %u: %s\n",
			  l1h->msg_type, osmo_hexdump(msgb_data(msg), msgb_length(msg)));
		msgb_free(msg);
		return -EINVAL;
	}
}
