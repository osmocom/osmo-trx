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

#include <radioInterface.h>
#include "l1if.h"
#include "ms_rx_upper.h"
#include "syncthing.h"
#include "ms_state.h"

void upper_trx::driveControl()
{
#ifdef IPCIF
	auto m = pop_c();
	if (!m)
		return;
#else
	TRX_C cmd;

	socklen_t addr_len = sizeof(ctrlsrc);
	int rdln = recvfrom(mCtrlSockets, (void *)cmd.cmd, sizeof(cmd) - 1, 0, &ctrlsrc, &addr_len);
	if (rdln < 0 && errno == EAGAIN) {
		std::cerr << "fuck, send ctrl?" << std::endl;
		exit(0);
	}

	TRX_C *m = &cmd;
#endif

	auto response = (TRX_C *)malloc(sizeof(TRX_C));
	response->cmd[0] = '\0';
	commandhandler(m->cmd, response->cmd);
#ifdef IPCIF
	free(m);
#endif
	std::clog << "response is " << response->cmd << std::endl;
#ifdef IPCIF
	push_c(response);
#else

	int rv = sendto(mCtrlSockets, response, strlen(response->cmd) + 1, 0, &ctrlsrc, sizeof(struct sockaddr_in));
	if (rv < 0) {
		std::cerr << "fuck, rcv ctrl?" << std::endl;
		exit(0);
	}
	free(response);

#endif
}

void upper_trx::commandhandler(char *buffer, char *response)
{
	int MAX_PACKET_LENGTH = TRXC_BUF_SIZE;

	char cmdcheck[4];
	char command[MAX_PACKET_LENGTH];

	sscanf(buffer, "%3s %s", cmdcheck, command);

	if (strcmp(cmdcheck, "CMD") != 0) {
		LOG(WARNING) << "bogus message on control interface";
		return;
	}
	std::clog << "command is " << buffer << std::endl << std::flush;

	if (strcmp(command, "MEASURE") == 0) {
		msleep(100);
		int freq;
		sscanf(buffer, "%3s %s %d", cmdcheck, command, &freq);
		sprintf(response, "RSP MEASURE 0 %d -80", freq);
	} else if (strcmp(command, "ECHO") == 0) {
		msleep(100);
		sprintf(response, "RSP ECHO 0");
	} else if (strcmp(command, "POWEROFF") == 0) {
		set_ta(0);
		// turn off transmitter/demod
		sprintf(response, "RSP POWEROFF 0");
	} else if (strcmp(command, "POWERON") == 0) {
		// turn on transmitter/demod
		if (!mTxFreq || !mRxFreq)
			sprintf(response, "RSP POWERON 1");
		else {
			sprintf(response, "RSP POWERON 0");
			if (!mOn) {
				// Prepare for thread start
				mPower = -20;
				start_ms();

				writeClockInterface();
				mOn = true;
			}
		}
	} else if (strcmp(command, "SETMAXDLY") == 0) {
		//set expected maximum time-of-arrival
		int maxDelay;
		sscanf(buffer, "%3s %s %d", cmdcheck, command, &maxDelay);
		mMaxExpectedDelay = maxDelay; // 1 GSM symbol is approx. 1 km
		sprintf(response, "RSP SETMAXDLY 0 %d", maxDelay);
	} else if (strcmp(command, "SETRXGAIN") == 0) {
		//set expected maximum time-of-arrival
		int newGain;
		sscanf(buffer, "%3s %s %d", cmdcheck, command, &newGain);
		newGain = setRxGain(newGain);
		sprintf(response, "RSP SETRXGAIN 0 %d", newGain);
	} else if (strcmp(command, "NOISELEV") == 0) {
		if (mOn) {
			float lev = 0; //mStates[chan].mNoiseLev;
			sprintf(response, "RSP NOISELEV 0 %d", (int)round(20.0 * log10(rxFullScale / lev)));
		} else {
			sprintf(response, "RSP NOISELEV 1  0");
		}
	} else if (!strcmp(command, "SETPOWER")) {
		// set output power in dB
		int dbPwr;
		sscanf(buffer, "%3s %s %d", cmdcheck, command, &dbPwr);
		if (!mOn)
			sprintf(response, "RSP SETPOWER 1 %d", dbPwr);
		else {
			mPower = dbPwr;
			setPowerAttenuation(mPower);
			sprintf(response, "RSP SETPOWER 0 %d", dbPwr);
		}
	} else if (!strcmp(command, "ADJPOWER")) {
		// adjust power in dB steps
		int dbStep;
		sscanf(buffer, "%3s %s %d", cmdcheck, command, &dbStep);
		if (!mOn)
			sprintf(response, "RSP ADJPOWER 1 %d", mPower);
		else {
			mPower += dbStep;
			setPowerAttenuation(mPower);
			sprintf(response, "RSP ADJPOWER 0 %d", mPower);
		}
	} else if (strcmp(command, "RXTUNE") == 0) {
		// tune receiver
		int freqKhz;
		sscanf(buffer, "%3s %s %d", cmdcheck, command, &freqKhz);
		mRxFreq = freqKhz * 1e3;
		if (!tuneRx(mRxFreq)) {
			LOG(ALERT) << "RX failed to tune";
			sprintf(response, "RSP RXTUNE 1 %d", freqKhz);
		} else
			sprintf(response, "RSP RXTUNE 0 %d", freqKhz);
	} else if (strcmp(command, "TXTUNE") == 0) {
		// tune txmtr
		int freqKhz;
		sscanf(buffer, "%3s %s %d", cmdcheck, command, &freqKhz);
		mTxFreq = freqKhz * 1e3;
		if (!tuneTx(mTxFreq)) {
			LOG(ALERT) << "TX failed to tune";
			sprintf(response, "RSP TXTUNE 1 %d", freqKhz);
		} else
			sprintf(response, "RSP TXTUNE 0 %d", freqKhz);
	} else if (!strcmp(command, "SETTSC")) {
		// set TSC
		unsigned TSC;
		sscanf(buffer, "%3s %s %d", cmdcheck, command, &TSC);
		if (mOn)
			sprintf(response, "RSP SETTSC 1 %d", TSC);
		// else if (chan && (TSC != mTSC))
		// 	sprintf(response, "RSP SETTSC 1 %d", TSC);
		else {
			mTSC = TSC;
			//generateMidamble(rx_sps, TSC);
			sprintf(response, "RSP SETTSC 0 %d", TSC);
		}
	} else if (!strcmp(command, "GETBSIC")) {
		if (mBSIC < 0)
			sprintf(response, "RSP GETBSIC 1");
		else
			sprintf(response, "RSP GETBSIC 0 %d", mBSIC);
	} else if (strcmp(command, "SETSLOT") == 0) {
		// set TSC
		int corrCode;
		int timeslot;
		sscanf(buffer, "%3s %s %d %d", cmdcheck, command, &timeslot, &corrCode);
		if ((timeslot < 0) || (timeslot > 7)) {
			LOG(WARNING) << "bogus message on control interface";
			sprintf(response, "RSP SETSLOT 1 %d %d", timeslot, corrCode);
			return;
		}
		mStates.chanType[timeslot] = (ChannelCombination)corrCode;
		mStates.setModulus(timeslot);
		sprintf(response, "RSP SETSLOT 0 %d %d", timeslot, corrCode);
	} else if (!strcmp(command, "SETRXMASK")) {
		int slot;
		unsigned long long mask;
		sscanf(buffer, "%3s %s %d 0x%llx", cmdcheck, command, &slot, &mask);
		if ((slot < 0) || (slot > 7)) {
			sprintf(response, "RSP SETRXMASK 1");
		} else {
			mRxSlotMask[slot] = mask;
			sprintf(response, "RSP SETRXMASK 0 %d 0x%llx", slot, mask);
		}
	} else if (!strcmp(command, "SYNC")) {
		// msleep(10);
		mStates.mode = trx_mode::TRX_MODE_MS_TRACK;
		sprintf(response, "RSP SYNC 0");
		mMaxExpectedDelay = 48;
		// setRxGain(30);
		// msleep(10);
	} else if (!strcmp(command, "SETTA")) {
		int ta;
		sscanf(buffer, "%3s %s %d", cmdcheck, command, &ta);
		set_ta(ta);
		sprintf(response, "RSP SETTA 0 %d", ta);
	} else {
		LOG(WARNING) << "bogus command " << command << " on control interface.";
	}

	//mCtrlSockets[chan]->write(response, strlen(response) + 1);
}
