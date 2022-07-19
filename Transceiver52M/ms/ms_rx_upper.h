
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
#include "syncthing.h"
#include "ms_state.h"

class upper_trx : public ms_trx {
	int rx_sps, tx_sps;

	ms_TransceiverState mStates;

	bool mOn; ///< flag to indicate that transceiver is powered on
	double mTxFreq; ///< the transmit frequency
	double mRxFreq; ///< the receive frequency
	int mPower; ///< the transmit power in dB
	unsigned mMaxExpectedDelay; ///< maximum TOA offset in GSM symbols
	unsigned long long mRxSlotMask[8]; ///< MS - enabled multiframe slot mask

	int mDataSockets;
	sockaddr_in datadest;
	sockaddr datasrc;
	int mCtrlSockets;
	sockaddr_in ctrldest;
	sockaddr ctrlsrc;

	void openudp(int *mSocketFD, unsigned short localPort, const char *wlocalIP)
	{
		*mSocketFD = socket(AF_INET, SOCK_DGRAM, 0);
		int on = 1;
		setsockopt(*mSocketFD, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

		struct sockaddr_in address;
		size_t length = sizeof(address);
		bzero(&address, length);
		address.sin_family = AF_INET;
		address.sin_addr.s_addr = inet_addr(wlocalIP);
		address.sin_port = htons(localPort);
		if (bind(*mSocketFD, (struct sockaddr *)&address, length) < 0) {
			std::cerr << "bind fail!" << std::endl;
			exit(0);
		}
	}

	bool resolveAddress(struct sockaddr_in *address, const char *host, unsigned short port)
	{
		struct hostent *hp;
		int h_errno_local;

		struct hostent hostData;
		char tmpBuffer[2048];

		auto rc = gethostbyname2_r(host, AF_INET, &hostData, tmpBuffer, sizeof(tmpBuffer), &hp, &h_errno_local);
		if (hp == NULL || hp->h_addrtype != AF_INET || rc != 0) {
			std::cerr << "WARNING -- gethostbyname() failed for " << host << ", "
				  << hstrerror(h_errno_local);
			exit(0);
			return false;
		}

		address->sin_family = hp->h_addrtype;
		assert(sizeof(address->sin_addr) == hp->h_length);
		memcpy(&(address->sin_addr), hp->h_addr_list[0], hp->h_length);
		address->sin_port = htons(port);
		return true;
	}

	void driveControl();
	void driveReceiveFIFO();
	void driveTx();
	void commandhandler(char *buffer, char *response);
	void writeClockInterface(){};

	SoftVector *pullRadioVector(GSM::Time &wTime, int &RSSI, int &timingOffset);

	bool detectSCH(ms_TransceiverState *state, signalVector &burst, struct estim_burst_params *ebp);

	std::thread thr_control, thr_rx, thr_tx;

    public:
	void start_threads();
	void start_ms();

	upper_trx() : rx_sps(4), tx_sps(4)
	{
		auto c_srcport = 6700 + 2 * 0 + 1;
		auto c_dstport = 6700 + 2 * 0 + 101;
		auto d_srcport = 6700 + 2 * 0 + 2;
		auto d_dstport = 6700 + 2 * 0 + 102;

		openudp(&mCtrlSockets, c_srcport, "127.0.0.1");
		openudp(&mDataSockets, d_srcport, "127.0.0.1");
		resolveAddress(&ctrldest, "127.0.0.1", c_dstport);
		resolveAddress(&datadest, "127.0.0.1", d_dstport);
	};
};
