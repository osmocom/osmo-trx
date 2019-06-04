/* Generic signalling/notification infrastructure */
/* (C) 2018 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
 *
 * Author: Pau Espin Pedrol <pespin@sysmocom.de>
 *
 * All Rights Reserved
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

#pragma once

#include <osmocom/core/signal.h>

/* Signalling subsystems */
enum signal_subsystems {
	SS_MAIN,
	SS_DEVICE,
};

/* SS_MAIN signals */
enum SS_MAIN {
	S_MAIN_STOP_REQUIRED, /* TRX fatal error, it should be stopped */
};

/* SS_DEVICE signals */
enum SS_DEVICE {
	/* Device internal counters changed. Counters are provided as cb data
	   (struct device_counters). Must be sent with PTHREAD_CANCEL_DISABLE
	   to avoid deadlocks in case osmo-trx process is asked to exit. */
	S_DEVICE_COUNTER_CHANGE,
};

/* signal cb for signal <SS_DEVICE,S_DEVICE_COUNTER_CHANGE> */
struct device_counters {
	size_t chan;
	unsigned int rx_underruns;
	unsigned int rx_overruns;
	unsigned int tx_underruns;
	unsigned int rx_dropped_events;
	unsigned int rx_dropped_samples;
};
