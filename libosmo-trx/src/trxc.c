/*! \file src/trxc.c
 * TRXC (control) protocol and clock indications: I/O-free message codec.
 * Based on the TRXC implementation in osmo-bts-trx and osmo-trx. */

/*
 * (C) 2013 Andreas Eversberg <jolly@eversberg.eu>
 * (C) 2016-2017 Harald Welte <laforge@gnumonks.org>
 * (C) 2019 Vadim Yanitskiy <axilirator@gmail.com>
 * (C) 2021-2026 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
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
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#include <osmocom/core/utils.h>
#include <osmocom/gsm/gsm0502.h>

#include <osmocom/trx/trxc.h>

static const struct value_string trxc_msg_type_names[] = {
	{ OSMO_TRXC_MT_CMD, "CMD" },
	{ OSMO_TRXC_MT_RSP, "RSP" },
	{ OSMO_TRXC_MT_IND, "IND" },
	{ 0, NULL }
};

/*! Parse a TRXC message ("CMD <verb> [<params>]", "RSP <verb> <status>
 *  [<params>]" or "IND <verb> <params>") from a zero-terminated buffer.
 *  \param[out] msg parsed message
 *  \param[in] buf message buffer (not necessarily zero-terminated)
 *  \param[in] len length of the message in the buffer
 *  \returns 0 on success; negative on error */
int osmo_trxc_msg_parse(struct osmo_trxc_msg *msg, const char *buf, size_t len)
{
	char tmp[OSMO_TRXC_MSG_BUF_SIZE];
	const char *p, *verb;
	size_t vlen;

	memset(msg, 0, sizeof(*msg));

	if (len >= sizeof(tmp))
		return -EMSGSIZE;
	/* work on a zero-terminated copy */
	memcpy(tmp, buf, len);
	tmp[len] = '\0';

	if (strncmp(tmp, "CMD ", 4) == 0)
		msg->type = OSMO_TRXC_MT_CMD;
	else if (strncmp(tmp, "RSP ", 4) == 0)
		msg->type = OSMO_TRXC_MT_RSP;
	else if (strncmp(tmp, "IND ", 4) == 0)
		msg->type = OSMO_TRXC_MT_IND;
	else
		return -EINVAL;

	/* the verb ends at the next space (or end of string) */
	verb = tmp + 4;
	p = strchr(verb, ' ');
	vlen = (p != NULL) ? (size_t)(p - verb) : strlen(verb);
	if (vlen == 0 || vlen >= sizeof(msg->cmd))
		return -EINVAL;
	memcpy(msg->cmd, verb, vlen);
	msg->cmd[vlen] = '\0';

	if (p == NULL) { /* no parameters at all */
		if (msg->type == OSMO_TRXC_MT_RSP)
			return -EINVAL; /* RSP requires a status code */
		return 0;
	}
	p++;

	/* RSP carries a status code before the parameters */
	if (msg->type == OSMO_TRXC_MT_RSP) {
		if (sscanf(p, "%d", &msg->status) != 1)
			return -EINVAL;
		p = strchr(p, ' ');
		if (p == NULL) /* no parameters after the status */
			return 0;
		p++;
	}

	if (strlen(p) >= sizeof(msg->params))
		return -EMSGSIZE;
	strcpy(msg->params, p);

	return 0;
}

/*! Serialize a TRXC message into the given buffer (zero-terminated).
 *  \returns length of the message (excl. '\0') on success; negative on error */
int osmo_trxc_msg_build(char *buf, size_t buf_size, const struct osmo_trxc_msg *msg)
{
	int rc;

	switch (msg->type) {
	case OSMO_TRXC_MT_CMD:
	case OSMO_TRXC_MT_IND:
		rc = snprintf(buf, buf_size, "%s %s%s%s",
			      get_value_string(trxc_msg_type_names, msg->type),
			      msg->cmd,
			      msg->params[0] != '\0' ? " " : "", msg->params);
		break;
	case OSMO_TRXC_MT_RSP:
		rc = snprintf(buf, buf_size, "RSP %s %d%s%s",
			      msg->cmd, msg->status,
			      msg->params[0] != '\0' ? " " : "", msg->params);
		break;
	default:
		return -EINVAL;
	}

	if (rc < 0 || (size_t)rc >= buf_size)
		return -EMSGSIZE;
	return rc;
}

/*! Parse the parameters string of a TRXC message, sscanf() style.
 *  \returns number of successfully matched items (see sscanf) */
int osmo_trxc_msg_params_scan(const struct osmo_trxc_msg *msg, const char *fmt, ...)
{
	va_list ap;
	int rc;

	va_start(ap, fmt);
	rc = vsscanf(msg->params, fmt, ap);
	va_end(ap);

	return rc;
}

/*! Compose a human-readable representation of the given message (for logging) */
const char *osmo_trxc_msg_name(const struct osmo_trxc_msg *msg)
{
	static __thread char buf[OSMO_TRXC_MSG_BUF_SIZE];

	if (osmo_trxc_msg_build(buf, sizeof(buf), msg) < 0)
		return "(invalid)";
	return buf;
}

/*! Parse a clock indication ("IND CLOCK <fn>").
 *  \param[out] fn TDMA frame number (validated to be < GSM_TDMA_HYPERFRAME)
 *  \returns 0 on success; negative on error */
int osmo_trxc_clock_ind_parse(uint32_t *fn, const char *buf, size_t len)
{
	struct osmo_trxc_msg msg;
	int rc;

	rc = osmo_trxc_msg_parse(&msg, buf, len);
	if (rc < 0)
		return rc;
	if (msg.type != OSMO_TRXC_MT_IND || strcmp(msg.cmd, "CLOCK") != 0)
		return -EINVAL;
	if (osmo_trxc_msg_params_scan(&msg, "%u", fn) != 1)
		return -EINVAL;
	if (*fn >= GSM_TDMA_HYPERFRAME)
		return -ERANGE;

	return 0;
}

/*! Serialize a clock indication ("IND CLOCK <fn>") into the given buffer.
 *  \returns length of the message (excl. '\0') on success; negative on error */
int osmo_trxc_clock_ind_build(char *buf, size_t buf_size, uint32_t fn)
{
	int rc;

	if (fn >= GSM_TDMA_HYPERFRAME)
		return -ERANGE;

	rc = snprintf(buf, buf_size, "IND CLOCK %u", fn);
	if (rc < 0 || (size_t)rc >= buf_size)
		return -EMSGSIZE;
	return rc;
}
