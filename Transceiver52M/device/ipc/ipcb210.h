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
#ifndef IPC_B210_H
#define IPC_B210_H

#include <thread>
//#include "magicwrap.h"
#include "IPCDevice.h"
//#include "../uhd/UHDDevice.h"

class IPC_b210 : public IPCDevice {
	//	std::thread *t;
	//	bool flush_recv(size_t num_pkts) override;

    public:
	template <typename... Args> IPC_b210(Args... args);
	virtual ~IPC_b210();

	//    void ipc_sock_close() override {};
#if 0
	int readSamples(std::vector<short *> &bufs, int len, bool *overrun, TIMESTAMP timestamp,
			bool *underrun) override;
    TIMESTAMP initialWriteTimestamp() override;
	TIMESTAMP initialReadTimestamp() override;
    bool start() override;
#endif

#if 0
	int writeSamples(std::vector<short *> &bufs, int len, bool *underrun, unsigned long long timestamp) override;
	    bool updateAlignment(TIMESTAMP timestamp) override;
#endif
	//	double setTxGain(double dB, size_t chan) override;
	//	double setRxGain(double dB, size_t chan) override;

	//	bool stop() override;
};

#endif // IPC_B210_H
