/*
 * Copyright (C) 2018-2019 sysmocom - s.f.m.c. GmbH
 * All Rights Reserved
 *
 * SPDX-License-Identifier: AGPL-3.0+
 *
 * Author: Pau Espin Pedrol <pespin@sysmocom.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 */

#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/rate_ctr.h>

#include <osmocom/vty/command.h>
#include <osmocom/vty/logging.h>
#include <osmocom/vty/vty.h>
#include <osmocom/vty/misc.h>

#include "trx_rate_ctr.h"
#include "trx_vty.h"
#include "../config.h"

static struct trx_ctx* g_trx_ctx;

const struct value_string clock_ref_names[] = {
	{ REF_INTERNAL,	"internal" },
	{ REF_EXTERNAL,	"external" },
	{ REF_GPS,	"gpsdo" },
	{ 0,		NULL }
};

const struct value_string filler_names[] = {
	{ FILLER_DUMMY,		"Dummy bursts (C0 only)" },
	{ FILLER_ZERO,		"Empty bursts" },
	{ FILLER_NORM_RAND,	"GMSK Normal Bursts with random payload" },
	{ FILLER_EDGE_RAND,	"8-PSK Normal Bursts with random payload" },
	{ FILLER_ACCESS_RAND,	"Access Bursts with random payload" },
	{ 0,			NULL }
};

static const struct value_string filler_types[] = {
	{ FILLER_DUMMY,		"dummy" },
	{ FILLER_ZERO,		"zero" },
	{ FILLER_NORM_RAND,	"random-nb-gmsk" },
	{ FILLER_EDGE_RAND,	"random-nb-8psk" },
	{ FILLER_ACCESS_RAND,	"random-ab" },
	{ 0,			NULL }
};

static const struct value_string filler_docs[] = {
	{ FILLER_DUMMY,		"Send a Dummy Burst on C0 (TRX0) and empty burst on other channels" },
	{ FILLER_ZERO,		"Send an empty burst (default)" },
	{ FILLER_NORM_RAND,	"Send a GMSK modulated Normal Burst with random bits (spectrum mask testing)" },
	{ FILLER_EDGE_RAND,	"Send an 8-PSK modulated Normal Burst with random bits (spectrum mask testing)" },
	{ FILLER_ACCESS_RAND,	"Send an Access Burst with random bits (Rx/Tx alignment testing)" },
	{ 0,			NULL }
};


struct trx_ctx *trx_from_vty(struct vty *v)
{
        /* It can't hurt to force callers to continue to pass the vty instance
         * to this function, in case we'd like to retrieve the global
         * trx instance from the vty at some point in the future. But
         * until then, just return the global pointer, which should have been
         * initialized by trx_vty_init().
         */
        OSMO_ASSERT(g_trx_ctx);
        return g_trx_ctx;
}

enum trx_vty_node {
	TRX_NODE = _LAST_OSMOVTY_NODE + 1,
	CHAN_NODE,
};

static struct cmd_node trx_node = {
	TRX_NODE,
	"%s(config-trx)# ",
	1,
};

static struct cmd_node chan_node = {
	CHAN_NODE,
	"%s(config-trx-chan)# ",
	1,
};

DEFUN(cfg_trx, cfg_trx_cmd,
	"trx",
	"Configure the TRX\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	if (!trx)
		return CMD_WARNING;

	vty->node = TRX_NODE;

	return CMD_SUCCESS;
}

DEFUN(cfg_bind_ip, cfg_bind_ip_cmd,
	"bind-ip " VTY_IPV4_CMD,
	"Set the IP address for the local bind\n"
	"IPv4 Address\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	osmo_talloc_replace_string(trx, &trx->cfg.bind_addr, argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_remote_ip, cfg_remote_ip_cmd,
	"remote-ip " VTY_IPV4_CMD,
	"Set the IP address for the remote BTS\n"
	"IPv4 Address\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	osmo_talloc_replace_string(trx, &trx->cfg.remote_addr, argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_base_port, cfg_base_port_cmd,
	"base-port <1-65535>",
	"Set the TRX Base Port\n"
	"TRX Base Port\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.base_port = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_dev_args, cfg_dev_args_cmd,
	"dev-args DESC",
	"Set the device-specific arguments to pass to the device\n"
	"Device-specific arguments\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	osmo_talloc_replace_string(trx, &trx->cfg.dev_args, argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_tx_sps, cfg_tx_sps_cmd,
	"tx-sps (1|4)",
	"Set the Tx Samples-per-Symbol\n"
	"Tx Samples-per-Symbol\n"
	"1 Sample-per-Symbol\n"
	"4 Samples-per-Symbol\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.tx_sps = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_rx_sps, cfg_rx_sps_cmd,
	"rx-sps (1|4)",
	"Set the Rx Samples-per-Symbol\n"
	"Rx Samples-per-Symbol\n"
	"1 Sample-per-Symbol\n"
	"4 Samples-per-Symbol\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.rx_sps = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_clock_ref, cfg_clock_ref_cmd,
	"clock-ref (internal|external|gpsdo)",
	"Set the Reference Clock\n"
	"Enable internal reference (default)\n"
	"Enable external 10 MHz reference\n"
	"Enable GPSDO reference\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.clock_ref = get_string_value(clock_ref_names, argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_multi_arfcn, cfg_multi_arfcn_cmd,
	"multi-arfcn (disable|enable)",
	"Multi-ARFCN transceiver mode (default=disable)\n"
	"Enable multi-ARFCN mode\n" "Disable multi-ARFCN mode\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	if (strcmp("disable", argv[0]) == 0) {
		trx->cfg.multi_arfcn = false;
		return CMD_SUCCESS;
	}

	if (trx->cfg.num_chans > TRX_MCHAN_MAX) {
		vty_out(vty, "Up to %i channels are supported for multi-TRX mode%s",
			TRX_MCHAN_MAX, VTY_NEWLINE);
		return CMD_WARNING;
	}

	trx->cfg.multi_arfcn = true;
	return CMD_SUCCESS;
}

DEFUN(cfg_offset, cfg_offset_cmd,
	"offset FLOAT",
	"Set the baseband frequency offset (default=0, auto)\n"
	"Baseband Frequency Offset\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.offset = atof(argv[0]);

	return CMD_SUCCESS;
}

DEFUN_ATTR(cfg_freq_offset, cfg_freq_offset_cmd,
	   "freq-offset FLOAT",
	   "Apply an artificial offset to Rx/Tx carrier frequency\n"
	   "Frequency offset in kHz (e.g. -145300)\n",
	   CMD_ATTR_HIDDEN)
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.freq_offset_khz = atof(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_rssi_offset, cfg_rssi_offset_cmd,
	"rssi-offset FLOAT [relative]",
	"Set the RSSI to dBm offset in dB (default=0)\n"
	"RSSI to dBm offset in dB\n"
	"Add to the default rssi-offset value instead of completely replacing it\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.rssi_offset = atof(argv[0]);
	trx->cfg.force_rssi_offset = (argc == 1);

	return CMD_SUCCESS;
}


DEFUN_ATTR(cfg_ul_fn_offset, cfg_ul_fn_offset_cmd,
	"ul-fn-offset <-10-10>",
	"Adjusts the uplink frame FN by the specified amount\n"
	"Frame Number offset\n",
	CMD_ATTR_HIDDEN)
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.ul_fn_offset = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN_ATTR(cfg_ul_freq_override, cfg_ul_freq_override_cmd,
	   "ul-freq-override FLOAT",
	   "Overrides Rx carrier frequency\n"
	   "Frequency in Hz (e.g. 145300000)\n",
	   CMD_ATTR_HIDDEN)
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.overrides.ul_freq_override = true;
	trx->cfg.overrides.ul_freq = atof(argv[0]);

	return CMD_SUCCESS;
}
DEFUN_ATTR(cfg_dl_freq_override, cfg_dl_freq_override_cmd,
	   "dl-freq-override FLOAT",
	   "Overrides Tx carrier frequency\n"
	   "Frequency in Hz (e.g. 145300000)\n",
	   CMD_ATTR_HIDDEN)
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.overrides.dl_freq_override = true;
	trx->cfg.overrides.dl_freq = atof(argv[0]);

	return CMD_SUCCESS;
}

DEFUN_ATTR(cfg_ul_gain_override, cfg_ul_gain_override_cmd,
	   "ul-gain-override FLOAT",
	   "Overrides Rx gain\n"
	   "gain in dB\n",
	   CMD_ATTR_HIDDEN)
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.overrides.ul_gain_override = true;
	trx->cfg.overrides.ul_gain = atof(argv[0]);

	return CMD_SUCCESS;
}
DEFUN_ATTR(cfg_dl_gain_override, cfg_dl_gain_override_cmd,
	   "dl-gain-override FLOAT",
	   "Overrides Tx gain\n"
	   "gain in dB\n",
	   CMD_ATTR_HIDDEN)
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.overrides.dl_gain_override = true;
	trx->cfg.overrides.dl_gain = atof(argv[0]);

	return CMD_SUCCESS;
}

DEFUN_ATTR(cfg_use_viterbi, cfg_use_viterbi_cmd,
	"viterbi-eq (disable|enable)",
	"Use viterbi equalizer for gmsk (default=disable)\n"
	"Disable VA\n"
	"Enable VA\n",
	CMD_ATTR_HIDDEN)
{
	struct trx_ctx *trx = trx_from_vty(vty);

	if (strcmp("disable", argv[0]) == 0)
		trx->cfg.use_va = false;
	else if (strcmp("enable", argv[0]) == 0)
		trx->cfg.use_va = true;
	else
		return CMD_WARNING;

	return CMD_SUCCESS;
}

DEFUN(cfg_swap_channels, cfg_swap_channels_cmd,
	"swap-channels (disable|enable)",
	"Swap primary and secondary channels of the PHY (if any)\n"
	"Do not swap primary and secondary channels (default)\n"
	"Swap primary and secondary channels\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	if (strcmp("disable", argv[0]) == 0) {
		trx->cfg.swap_channels = false;
	} else if (strcmp("enable", argv[0]) == 0) {
		trx->cfg.swap_channels = true;
	} else {
		return CMD_WARNING;
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_egprs, cfg_egprs_cmd,
	"egprs (disable|enable)",
	"EGPRS (8-PSK demodulation) support (default=disable)\n"
	"Disable EGPRS (8-PSK demodulation) support\n"
	"Enable EGPRS (8-PSK demodulation) support\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	if (strcmp("disable", argv[0]) == 0) {
		trx->cfg.egprs = false;
	} else if (strcmp("enable", argv[0]) == 0) {
		trx->cfg.egprs = true;
	} else {
		return CMD_WARNING;
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_ext_rach, cfg_ext_rach_cmd,
	"ext-rach (disable|enable)",
	"11-bit Access Burst correlation support (default=disable)\n"
	"Disable 11-bit Access Burst (TS1 & TS2) correlation\n"
	"Enable 11-bit Access Burst (TS1 & TS2) correlation\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	if (strcmp("disable", argv[0]) == 0)
		trx->cfg.ext_rach = false;

	if (strcmp("enable", argv[0]) == 0)
		trx->cfg.ext_rach = true;

	return CMD_SUCCESS;
}

DEFUN_DEPRECATED(cfg_rt_prio, cfg_rt_prio_cmd,
	"rt-prio <1-32>",
	"Set the SCHED_RR real-time priority\n"
	"Real time priority\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.sched_rr = atoi(argv[0]);
	vty_out (vty, "%% 'rt-prio %u' is deprecated, use 'policy rr %u' under 'sched' node instead%s",
		 trx->cfg.sched_rr, trx->cfg.sched_rr, VTY_NEWLINE);

	return CMD_SUCCESS;
}

DEFUN(cfg_stack_size, cfg_stack_size_cmd,
	"stack-size <0-2147483647>",
	"Set the stack size per thread in BYTE, 0 = OS default\n"
	"Stack size per thread in BYTE\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.stack_size = atoi(argv[0]);

	return CMD_SUCCESS;
}

#define CFG_FILLER_DOC_STR \
	"Filler burst settings\n"

DEFUN(cfg_filler, cfg_filler_type_cmd,
      "AUTO-GENERATED", "AUTO-GENERATED")
{
	struct trx_ctx *trx = trx_from_vty(vty);
	// trx->cfg.filler is unsigned, so we need an interim int var to detect errors
	int type = get_string_value(filler_types, argv[0]);

	if (type < 0) {
		trx->cfg.filler = FILLER_ZERO;
		return CMD_WARNING;
	}
	trx->cfg.filler = type;

	return CMD_SUCCESS;
}

DEFUN(cfg_test_rtsc, cfg_filler_tsc_cmd,
	"filler tsc <0-7>",
	CFG_FILLER_DOC_STR
	"Set the TSC for GMSK/8-PSK Normal Burst random fillers. Used only with 'random-nb-gmsk' and"
	" 'random-nb-8psk' filler types. (default=0)\n"
	"TSC\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.rtsc = atoi(argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_test_rach_delay, cfg_filler_rach_delay_cmd,
	"filler access-burst-delay <0-68>",
	CFG_FILLER_DOC_STR
	"Set the delay for Access Burst random fillers. Used only with 'random-ab' filler type. (default=0)\n"
	"RACH delay in symbols\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx->cfg.rach_delay = atoi(argv[0]);

	return CMD_SUCCESS;
}

static int vty_ctr_name_2_id(const char* str) {
	size_t i;
	for (i = 0; trx_chan_ctr_names[i].str; i++) {
		if (strstr(trx_chan_ctr_names[i].str, str)) {
			return i;
		}
	}
	return -1;
}

static int vty_intv_name_2_id(const char* str) {
	size_t i;
	for (i = 0; rate_ctr_intv[i].str; i++) {
		if (strcmp(rate_ctr_intv[i].str, str) == 0) {
			return i;
		}
	}
	return -1;
}

#define THRESHOLD_ARGS "(rx_overruns|tx_underruns|rx_drop_events|rx_drop_samples|tx_drop_events|tx_drop_samples|tx_stale_bursts|tx_unavailable_bursts|tx_trxd_fn_repeated|tx_trxd_fn_outoforder|tx_trxd_fn_skipped)"
#define THRESHOLD_STR_VAL(s) "Set threshold value for rate_ctr device:" OSMO_STRINGIFY_VAL(s) "\n"
#define THRESHOLD_STRS \
	THRESHOLD_STR_VAL(rx_overruns) \
	THRESHOLD_STR_VAL(tx_underruns) \
	THRESHOLD_STR_VAL(rx_drop_events) \
	THRESHOLD_STR_VAL(rx_drop_samples) \
	THRESHOLD_STR_VAL(tx_drop_events) \
	THRESHOLD_STR_VAL(tx_drop_samples) \
	THRESHOLD_STR_VAL(tx_stale_bursts) \
	THRESHOLD_STR_VAL(tx_unavailable_bursts) \
	THRESHOLD_STR_VAL(tx_trxd_fn_repeated) \
	THRESHOLD_STR_VAL(tx_trxd_fn_outoforder) \
	THRESHOLD_STR_VAL(tx_trxd_fn_skipped) \
	""
#define INTV_ARGS "(per-second|per-minute|per-hour|per-day)"
#define INTV_STR_VAL(s) "Threshold value sampled " OSMO_STRINGIFY_VAL(s) "\n"
#define INTV_STRS \
	INTV_STR_VAL(per-second) \
	INTV_STR_VAL(per-minute) \
	INTV_STR_VAL(per-hour) \
	INTV_STR_VAL(per-day)

DEFUN_ATTR(cfg_ctr_error_threshold, cfg_ctr_error_threshold_cmd,
	   "ctr-error-threshold " THRESHOLD_ARGS " <0-65535> " INTV_ARGS,
	   "Threshold rate for error counter\n"
	   THRESHOLD_STRS
	   "Value to set for threshold\n"
	   INTV_STRS,
	   CMD_ATTR_IMMEDIATE)
{
	int rc;
	struct ctr_threshold ctr;

	rc = vty_ctr_name_2_id(argv[0]);
	if (rc < 0) {
		vty_out(vty, "No valid ctr_name found for ctr-error-threshold %s%s",
			argv[0], VTY_NEWLINE);
		return CMD_WARNING;
	}
	ctr.ctr_id = (enum TrxCtr)rc;
	ctr.val = atoi(argv[1]);
	rc = vty_intv_name_2_id(argv[2]);
	if (rc < 0) {
		vty_out(vty, "No valid time frame found for ctr-error-threshold %s %d %s%s",
			argv[0], ctr.val, argv[2], VTY_NEWLINE);
		return CMD_WARNING;
	}
	ctr.intv = (enum rate_ctr_intv) rc;
	trx_rate_ctr_threshold_add(&ctr);

	return CMD_SUCCESS;
}

DEFUN_ATTR(cfg_no_ctr_error_threshold, cfg_no_ctr_error_threshold_cmd,
	   "no ctr-error-threshold " THRESHOLD_ARGS " <0-65535> " INTV_ARGS,
	   NO_STR "Threshold rate for error counter\n"
	   THRESHOLD_STRS
	   "Value to set for threshold\n"
	   INTV_STRS,
	   CMD_ATTR_IMMEDIATE)
{
	int rc;
	struct ctr_threshold ctr;

	rc = vty_ctr_name_2_id(argv[0]);
	if (rc < 0) {
		vty_out(vty, "No valid ctr_name found for ctr-error-threshold %s%s",
			argv[0], VTY_NEWLINE);
		return CMD_WARNING;
	}
	ctr.ctr_id = (enum TrxCtr)rc;
	ctr.val = atoi(argv[1]);
	rc = vty_intv_name_2_id(argv[2]);
	if (rc < 0) {
		vty_out(vty, "No valid time frame found for ctr-error-threshold %s %d %s%s",
			argv[0], ctr.val, argv[2], VTY_NEWLINE);
		return CMD_WARNING;
	}
	ctr.intv = (enum rate_ctr_intv) rc;
	if (trx_rate_ctr_threshold_del(&ctr) < 0) {
		vty_out(vty, "no ctr-error-threshold: Entry to delete not found%s", VTY_NEWLINE);
		return CMD_WARNING;
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_chan, cfg_chan_cmd,
	"chan <0-100>",
	"Select a channel to configure\n"
	"Channel index\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);
	int idx = atoi(argv[0]);

	if (idx >= TRX_CHAN_MAX) {
		vty_out(vty, "Chan list full.%s", VTY_NEWLINE);
		return CMD_WARNING;
	} else if (trx->cfg.multi_arfcn && trx->cfg.num_chans >= TRX_MCHAN_MAX) {
		vty_out(vty, "Up to %i channels are supported for multi-TRX mode%s",
			TRX_MCHAN_MAX, VTY_NEWLINE);
		return CMD_WARNING;
	}

	if (trx->cfg.num_chans < idx) { /* Unexisting or creating non-consecutive */
		vty_out(vty, "Non-existent or non-consecutive chan %d.%s",
				idx, VTY_NEWLINE);
		return CMD_WARNING;
	} else if (trx->cfg.num_chans == idx)  { /* creating it */
		trx->cfg.num_chans++;
		trx->cfg.chans[idx].trx = trx;
		trx->cfg.chans[idx].idx = idx;
	}

	vty->node = CHAN_NODE;
	vty->index = &trx->cfg.chans[idx];

	return CMD_SUCCESS;
}

DEFUN(cfg_chan_rx_path, cfg_chan_rx_path_cmd,
	"rx-path NAME",
	"Set the Rx Path\n"
	"Rx Path name\n")
{
	struct trx_chan *chan = vty->index;

	if (chan->trx->cfg.multi_arfcn && chan->idx > 0) {
		vty_out(vty, "%% Setting 'rx-path' for chan %u in multi-ARFCN mode "
			     "does not make sense, because only chan 0 is used%s",
			chan->idx, VTY_NEWLINE);
	}

	osmo_talloc_replace_string(chan->trx, &chan->rx_path, argv[0]);

	return CMD_SUCCESS;
}

DEFUN(cfg_chan_tx_path, cfg_chan_tx_path_cmd,
	"tx-path NAME",
	"Set the Tx Path\n"
	"Tx Path name\n")
{
	struct trx_chan *chan = vty->index;

	if (chan->trx->cfg.multi_arfcn && chan->idx > 0) {
		vty_out(vty, "%% Setting 'tx-path' for chan %u in multi-ARFCN mode "
			     "does not make sense, because only chan 0 is used%s",
			chan->idx, VTY_NEWLINE);
	}

	osmo_talloc_replace_string(chan->trx, &chan->tx_path, argv[0]);

	return CMD_SUCCESS;
}

static int dummy_config_write(struct vty *v)
{
	return CMD_SUCCESS;
}

static int config_write_trx(struct vty *vty)
{
	struct trx_chan *chan;
	int i;
	struct trx_ctx *trx = trx_from_vty(vty);

	vty_out(vty, "trx%s", VTY_NEWLINE);
	if (trx->cfg.bind_addr)
		vty_out(vty, " bind-ip %s%s", trx->cfg.bind_addr, VTY_NEWLINE);
	if (trx->cfg.remote_addr)
		vty_out(vty, " remote-ip %s%s", trx->cfg.remote_addr, VTY_NEWLINE);
	if (trx->cfg.base_port != DEFAULT_TRX_PORT)
		vty_out(vty, " base-port %u%s", trx->cfg.base_port, VTY_NEWLINE);
	if (strlen(trx->cfg.dev_args))
		vty_out(vty, " dev-args %s%s", trx->cfg.dev_args, VTY_NEWLINE);
	if (trx->cfg.tx_sps != DEFAULT_TX_SPS)
		vty_out(vty, " tx-sps %u%s", trx->cfg.tx_sps, VTY_NEWLINE);
	if (trx->cfg.rx_sps != DEFAULT_RX_SPS)
		vty_out(vty, " rx-sps %u%s", trx->cfg.rx_sps, VTY_NEWLINE);
	if (trx->cfg.clock_ref != REF_INTERNAL)
		vty_out(vty, " clock-ref %s%s", get_value_string(clock_ref_names, trx->cfg.clock_ref), VTY_NEWLINE);
	vty_out(vty, " multi-arfcn %s%s", trx->cfg.multi_arfcn ? "enable" : "disable", VTY_NEWLINE);
	if (trx->cfg.offset != 0)
		vty_out(vty, " offset %f%s", trx->cfg.offset, VTY_NEWLINE);
	if (trx->cfg.freq_offset_khz != 0)
		vty_out(vty, " freq-offset %f%s", trx->cfg.freq_offset_khz, VTY_NEWLINE);
	if (!(trx->cfg.rssi_offset == 0 && !trx->cfg.force_rssi_offset))
		vty_out(vty, " rssi-offset %f%s%s", trx->cfg.rssi_offset,
			trx->cfg.force_rssi_offset ? " relative": "", VTY_NEWLINE);
	vty_out(vty, " swap-channels %s%s", trx->cfg.swap_channels ? "enable" : "disable", VTY_NEWLINE);
	vty_out(vty, " egprs %s%s", trx->cfg.egprs ? "enable" : "disable", VTY_NEWLINE);
	vty_out(vty, " ext-rach %s%s", trx->cfg.ext_rach ? "enable" : "disable", VTY_NEWLINE);
	if (trx->cfg.sched_rr != 0)
		vty_out(vty, " rt-prio %u%s", trx->cfg.sched_rr, VTY_NEWLINE);
	if (trx->cfg.filler != FILLER_ZERO)
		vty_out(vty, " filler type %s%s", get_value_string(filler_types, trx->cfg.filler), VTY_NEWLINE);
	if (trx->cfg.rtsc > 0)
		vty_out(vty, " filler tsc %u%s", trx->cfg.rtsc, VTY_NEWLINE);
	if (trx->cfg.rach_delay > 0)
		vty_out(vty, " filler access-burst-delay %u%s", trx->cfg.rach_delay, VTY_NEWLINE);
	if (trx->cfg.stack_size != 0)
		vty_out(vty, " stack-size %u%s", trx->cfg.stack_size, VTY_NEWLINE);
	if (trx->cfg.ul_fn_offset != 0)
		vty_out(vty, " ul-fn-offset %d%s", trx->cfg.ul_fn_offset, VTY_NEWLINE);
	if (trx->cfg.overrides.dl_freq_override)
		vty_out(vty, " dl-freq-override %f%s", trx->cfg.overrides.dl_freq, VTY_NEWLINE);
	if (trx->cfg.overrides.ul_freq_override)
		vty_out(vty, " ul-freq-override %f%s", trx->cfg.overrides.ul_freq, VTY_NEWLINE);
	if (trx->cfg.overrides.dl_gain_override)
		vty_out(vty, " dl-gain-override %f%s", trx->cfg.overrides.dl_gain, VTY_NEWLINE);
	if (trx->cfg.overrides.ul_gain_override)
		vty_out(vty, " ul-gain-override %f%s", trx->cfg.overrides.ul_gain, VTY_NEWLINE);
	if (trx->cfg.use_va)
		vty_out(vty, " viterbi-eq %s%s", trx->cfg.use_va ? "enable" : "disable", VTY_NEWLINE);
	trx_rate_ctr_threshold_write_config(vty, " ");

	for (i = 0; i < trx->cfg.num_chans; i++) {
		chan = &trx->cfg.chans[i];
		vty_out(vty, " chan %u%s", chan->idx, VTY_NEWLINE);
		if (chan->rx_path)
			vty_out(vty, "  rx-path %s%s", chan->rx_path, VTY_NEWLINE);
		if (chan->tx_path)
			vty_out(vty, "  tx-path %s%s", chan->tx_path, VTY_NEWLINE);
	}

	return CMD_SUCCESS;
}

static void trx_dump_vty(struct vty *vty, struct trx_ctx *trx)
{
	struct trx_chan *chan;
	int i;
	vty_out(vty, "TRX Config:%s", VTY_NEWLINE);
	vty_out(vty, " Local IP: %s%s", trx->cfg.bind_addr, VTY_NEWLINE);
	vty_out(vty, " Remote IP: %s%s", trx->cfg.remote_addr, VTY_NEWLINE);
	vty_out(vty, " TRX Base Port: %u%s", trx->cfg.base_port, VTY_NEWLINE);
	vty_out(vty, " Device args: %s%s", trx->cfg.dev_args, VTY_NEWLINE);
	vty_out(vty, " Tx Samples-per-Symbol: %u%s", trx->cfg.tx_sps, VTY_NEWLINE);
	vty_out(vty, " Rx Samples-per-Symbol: %u%s", trx->cfg.rx_sps, VTY_NEWLINE);
	vty_out(vty, " Filler Burst Type: %s%s", get_value_string(filler_names, trx->cfg.filler), VTY_NEWLINE);
	vty_out(vty, " Filler Burst TSC: %u%s", trx->cfg.rtsc, VTY_NEWLINE);
	vty_out(vty, " Filler Burst RACH Delay: %u%s", trx->cfg.rach_delay, VTY_NEWLINE);
	vty_out(vty, " Clock Reference: %s%s", get_value_string(clock_ref_names, trx->cfg.clock_ref), VTY_NEWLINE);
	vty_out(vty, " Multi-Carrier: %s%s", trx->cfg.multi_arfcn ? "Enabled" : "Disabled", VTY_NEWLINE);
	vty_out(vty, " Tuning offset: %f%s", trx->cfg.offset, VTY_NEWLINE);
	vty_out(vty, " RSSI to dBm offset: %f%s", trx->cfg.rssi_offset, VTY_NEWLINE);
	vty_out(vty, " Swap channels: %s%s", trx->cfg.swap_channels ? "Enabled" : "Disabled", VTY_NEWLINE);
	vty_out(vty, " EDGE support: %s%s", trx->cfg.egprs ? "Enabled" : "Disabled", VTY_NEWLINE);
	vty_out(vty, " Extended RACH support: %s%s", trx->cfg.ext_rach ? "Enabled" : "Disabled", VTY_NEWLINE);
	vty_out(vty, " Real Time Priority: %u (%s)%s", trx->cfg.sched_rr,
		trx->cfg.sched_rr ? "Enabled" : "Disabled", VTY_NEWLINE);
	vty_out(vty, " Stack size per Thread in BYTE (0 = OS default): %u%s", trx->cfg.stack_size, VTY_NEWLINE);
	vty_out(vty, " Channels: %u%s", trx->cfg.num_chans, VTY_NEWLINE);
	for (i = 0; i < trx->cfg.num_chans; i++) {
		chan = &trx->cfg.chans[i];
		vty_out(vty, "  Channel %u:%s", chan->idx, VTY_NEWLINE);
		if (chan->rx_path)
			vty_out(vty, "   Rx Path: %s%s", chan->rx_path, VTY_NEWLINE);
		if (chan->tx_path)
			vty_out(vty, "   Tx Path: %s%s", chan->tx_path, VTY_NEWLINE);
	}
}

DEFUN(show_trx, show_trx_cmd,
	"show trx",
	SHOW_STR "Display information on the TRX\n")
{
	struct trx_ctx *trx = trx_from_vty(vty);

	trx_dump_vty(vty, trx);

	return CMD_SUCCESS;
}

static int trx_vty_go_parent(struct vty *vty)
{
	switch (vty->node) {
	case TRX_NODE:
		vty->node = CONFIG_NODE;
		vty->index = NULL;
		vty->index_sub = NULL;
		break;
	case CHAN_NODE:
		vty->node = TRX_NODE;
		vty->index = NULL;
		vty->index_sub = NULL;
		break;
	default:
		vty->node = CONFIG_NODE;
		vty->index = NULL;
		vty->index_sub = NULL;
	}

	return vty->node;
}

static const char trx_copyright[] =
	"Copyright (C) 2007-2014 Free Software Foundation, Inc.\r\n"
	"Copyright (C) 2013 Thomas Tsou <tom@tsou.cc>\r\n"
	"Copyright (C) 2013-2019 Fairwaves, Inc.\r\n"
	"Copyright (C) 2015 Ettus Research LLC\r\n"
	"Copyright (C) 2017-2018 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>\r\n"
	"License AGPLv3+: GNU AGPL version 3 or later <http://gnu.org/licenses/agpl-3.0.html>\r\n"
	"This is free software: you are free to change and redistribute it.\r\n"
	"There is NO WARRANTY, to the extent permitted by law.\r\n";

struct vty_app_info g_vty_info = {
	.name		= "OsmoTRX",
	.version	= PACKAGE_VERSION,
	.copyright	= trx_copyright,
	.go_parent_cb	= trx_vty_go_parent,
};

struct trx_ctx *vty_trx_ctx_alloc(void *talloc_ctx)
{
	struct trx_ctx * trx = talloc_zero(talloc_ctx, struct trx_ctx);

	trx->cfg.bind_addr =  talloc_strdup(trx, DEFAULT_TRX_IP);
	trx->cfg.remote_addr = talloc_strdup(trx, DEFAULT_TRX_IP);
	trx->cfg.base_port = DEFAULT_TRX_PORT;
	trx->cfg.tx_sps = DEFAULT_TX_SPS;
	trx->cfg.rx_sps = DEFAULT_RX_SPS;
	trx->cfg.filler = FILLER_ZERO;
	trx->cfg.rssi_offset = 0.0f;
	trx->cfg.dev_args = talloc_strdup(trx, "");

	return trx;
}

int trx_vty_init(struct trx_ctx* trx)
{
	cfg_filler_type_cmd.string = vty_cmd_string_from_valstr(trx, filler_types,
		"filler type (", "|", ")", 0);
	cfg_filler_type_cmd.doc = vty_cmd_string_from_valstr(trx, filler_docs,
		CFG_FILLER_DOC_STR "What to do when there is nothing to send "
		"(filler type, default=zero)\n", "\n", "", 0);

	g_trx_ctx = trx;
	install_element_ve(&show_trx_cmd);

	install_element(CONFIG_NODE, &cfg_trx_cmd);

	install_node(&trx_node, config_write_trx);
	install_element(TRX_NODE, &cfg_bind_ip_cmd);
	install_element(TRX_NODE, &cfg_remote_ip_cmd);
	install_element(TRX_NODE, &cfg_base_port_cmd);
	install_element(TRX_NODE, &cfg_dev_args_cmd);
	install_element(TRX_NODE, &cfg_tx_sps_cmd);
	install_element(TRX_NODE, &cfg_rx_sps_cmd);
	install_element(TRX_NODE, &cfg_clock_ref_cmd);
	install_element(TRX_NODE, &cfg_multi_arfcn_cmd);
	install_element(TRX_NODE, &cfg_offset_cmd);
	install_element(TRX_NODE, &cfg_freq_offset_cmd);
	install_element(TRX_NODE, &cfg_rssi_offset_cmd);
	install_element(TRX_NODE, &cfg_swap_channels_cmd);
	install_element(TRX_NODE, &cfg_egprs_cmd);
	install_element(TRX_NODE, &cfg_ext_rach_cmd);
	install_element(TRX_NODE, &cfg_rt_prio_cmd);
	install_element(TRX_NODE, &cfg_filler_type_cmd);
	install_element(TRX_NODE, &cfg_filler_tsc_cmd);
	install_element(TRX_NODE, &cfg_filler_rach_delay_cmd);
	install_element(TRX_NODE, &cfg_ctr_error_threshold_cmd);
	install_element(TRX_NODE, &cfg_no_ctr_error_threshold_cmd);
	install_element(TRX_NODE, &cfg_stack_size_cmd);

	install_element(TRX_NODE, &cfg_chan_cmd);
	install_element(TRX_NODE, &cfg_ul_fn_offset_cmd);
	install_element(TRX_NODE, &cfg_ul_freq_override_cmd);
	install_element(TRX_NODE, &cfg_dl_freq_override_cmd);
	install_element(TRX_NODE, &cfg_ul_gain_override_cmd);
	install_element(TRX_NODE, &cfg_dl_gain_override_cmd);
	install_element(TRX_NODE, &cfg_use_viterbi_cmd);
	install_node(&chan_node, dummy_config_write);
	install_element(CHAN_NODE, &cfg_chan_rx_path_cmd);
	install_element(CHAN_NODE, &cfg_chan_tx_path_cmd);

	logging_vty_add_deprecated_subsys(g_trx_ctx, "lms");

	return 0;
}
