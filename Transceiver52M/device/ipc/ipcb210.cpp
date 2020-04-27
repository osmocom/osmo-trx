/*
* Copyright 2020 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
* Author: Eric Wild <ewild@sysmocom.de>
*
* SPDX-License-Identifier: 0BSD
*
    Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.
    THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
    INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
    AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
    PERFORMANCE OF THIS SOFTWARE.
*/
extern "C" {
#include <osmocom/core/application.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/select.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>

#include "shm.h"
}
//#include "../uhd/UHDDevice.h"

#include "ipcb210.h"
//#include "uhdwrap.h"
//#include "trx_vty.h"
//#include "Logger.h"
//#include "Threads.h"
//#include "Utils.h"

#if 0
//#include "uhdwrap.h"
//extern uhd_wrap *uhd_wrap_dev;

//extern "C" void magicmain(int, char **);
//char *argv[] = {
//	"magic",
//};
//void magicthread()
//{
//	pthread_setname_np(pthread_self(), "magicthread");
//	magicmain(1, argv);
//}
#endif
template <typename... Args> IPC_b210::IPC_b210(Args... args) : IPCDevice(args...)
{
#if 0
	//	t = new std::thread(magicthread);
	//	// give the thread some time to start and set up
	//	std::this_thread::sleep_for(std::chrono::seconds(1));
#endif
}

IPC_b210::~IPC_b210()
{
	//gshutdown = 1;
	//t->join();
}

#if 0

bool IPC_b210::start()
{
	return uhd_wrap_dev->start();
}

/** Returns the starting write Timestamp*/
TIMESTAMP IPC_b210::initialWriteTimestamp(void)
{
	return uhd_wrap_dev->initialWriteTimestamp();
}

/** Returns the starting read Timestamp*/
TIMESTAMP IPC_b210::initialReadTimestamp(void)
{
	return uhd_wrap_dev->initialReadTimestamp();
}


//// NOTE: Assumes sequential reads
int IPC_b210::readSamples(std::vector<short *> &bufs, int len, bool *overrun, TIMESTAMP timestamp, bool *underrun)
{
#if 0
	return uhd_wrap_dev->readSamples(bufs, len, overrun, timestamp, underrun);
#else
	int rc, num_smpls, expect_smpls;
	ssize_t avail_smpls;
	TIMESTAMP expect_timestamp, actual_timestamp;
	unsigned int i;

	if (bufs.size() != chans) {
		LOGC(DDEV, ERROR) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	*overrun = false;
	*underrun = false;

	timestamp += uhd_wrap_dev->getTimingOffset();

	/* Check that timestamp is valid */
	rc = rx_buffers[0]->avail_smpls(timestamp);
	if (rc < 0) {
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_code(rc);
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_status(timestamp);
		return 0;
	}

	for (i = 0; i < chans; i++) {
		/* Receive samples from HW until we have enough */
		while ((avail_smpls = rx_buffers[i]->avail_smpls(timestamp)) < len) {
			uint64_t recv_timestamp = 0;

			num_smpls = uhd_wrap_dev->wrap_read((TIMESTAMP *)&recv_timestamp);
			//			memcpy(bufs[i], &uhd_wrap_dev->wrap_buffs[i].front(), num_smpls * 4);

			expect_timestamp = timestamp + avail_smpls;

			LOGCHAN(i, DDEV, DEBUG)
			"Received timestamp = " << (TIMESTAMP)recv_timestamp << " (" << num_smpls << ")";

			expect_smpls = len - avail_smpls;
			//			if (expect_smpls != num_smpls)
			//				LOGCHAN(i, DDEV, NOTICE)
			//					<< "Unexpected recv buffer len: expect " << expect_smpls << " got " << num_smpls
			//					<< ", diff=" << expect_smpls - num_smpls;

			//expect_timestamp = timestamp + avail_smpls;
			if (expect_timestamp != (TIMESTAMP)recv_timestamp)
				LOGCHAN(i, DDEV, ERROR)
					<< "Unexpected recv buffer timestamp: expect " << expect_timestamp << " got "
					<< recv_timestamp << ", diff=" << recv_timestamp - expect_timestamp;

			rc = rx_buffers[i]->write(&uhd_wrap_dev->wrap_buffs[i].front(), num_smpls,
						  (TIMESTAMP)recv_timestamp);
			if (rc < 0) {
				LOGCHAN(i, DDEV, ERROR) << rx_buffers[i]->str_code(rc);
				LOGCHAN(i, DDEV, ERROR) << rx_buffers[i]->str_status(timestamp);
				if (rc != smpl_buf::ERROR_OVERFLOW)
					return 0;
			}
		}
	}

	/* We have enough samples */
	for (size_t i = 0; i < rx_buffers.size(); i++) {
		rc = rx_buffers[i]->read(bufs[i], len, timestamp);
		if ((rc < 0) || (rc != len)) {
			LOGCHAN(i, DDEV, ERROR) << rx_buffers[i]->str_code(rc) << ". "
						<< rx_buffers[i]->str_status(timestamp) << ", (len=" << len << ")";
			return 0;
		}
	}

	return len;
#endif
}
#endif

#if 0
int IPC_b210::writeSamples(std::vector<short *> &bufs, int len, bool *underrun, unsigned long long timestamp)
{
	return uhd_wrap_dev->writeSamples(bufs, len, underrun, timestamp);
}
#endif
//bool IPC_b210::updateAlignment(TIMESTAMP timestamp)
//{
//	return drvtest::dev->updateAlignment(timestamp);
//}

//double IPC_b210::setTxGain(double dB, size_t chan)
//{
//	auto v = IPCDevice::setTxGain(dB, chan);
//	return uhd_wrap_dev->setTxGain(v, chan);
//}

//double IPC_b210::setRxGain(double dB, size_t chan)
//{
//	auto v = IPCDevice::setRxGain(dB, chan);
//	return uhd_wrap_dev->setRxGain(v, chan);
//}

//bool IPC_b210::stop()
//{
//	return uhd_wrap_dev->stop();
//}

//bool IPC_b210::flush_recv(size_t num_pkts)
//{
//	return true; //drvtest::dev->flush_recv(num_pkts);
//}

RadioDevice *RadioDevice::make(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chans, double lo_offset,
			       const std::vector<std::string> &tx_paths, const std::vector<std::string> &rx_paths)
{
	if (tx_sps != rx_sps) {
		LOGC(DDEV, ERROR) << "IPC Requires tx_sps == rx_sps";
		return NULL;
	}
	if (lo_offset != 0.0) {
		LOGC(DDEV, ERROR) << "IPC doesn't support lo_offset";
		return NULL;
	}
	return new IPC_b210(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths);
}
