
#pragma once

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
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "GSMCommon.h"
#include "radioClock.h"
#include "ms.h"

namespace trxcon
{
extern "C" {
#include <osmocom/bb/trxcon/phyif.h>
#include <osmocom/bb/trxcon/trx_if.h>
}
} // namespace trxcon
class upper_trx : public ms_trx {
	bool mOn;
	char demodded_softbits[444];

	// void driveControl();
	bool driveControl();
	void driveReceiveFIFO();
	void driveTx();

	bool pullRadioVector(GSM::Time &wTime, int &RSSI, int &timingOffset) __attribute__((optnone));

	std::thread thr_control, thr_rx, thr_tx;

    public:
	void start_threads();
	void start_lower_ms();

	upper_trx(){};
};
