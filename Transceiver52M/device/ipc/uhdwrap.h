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
#ifndef IPC_UHDWRAP_H
#define IPC_UHDWRAP_H

#ifdef __cplusplus
#include "../uhd/UHDDevice.h"

class uhd_wrap : public uhd_device {
    public:
	//	std::thread *t;
	size_t samps_per_buff_rx;
	size_t samps_per_buff_tx;
	int channel_count;

	std::vector<std::vector<short> > wrap_rx_buffs;
	std::vector<std::vector<short> > wrap_tx_buffs;
	std::vector<short *> wrap_rx_buf_ptrs;
	std::vector<short *> wrap_tx_buf_ptrs;

	template <typename... Args> uhd_wrap(Args... args) : uhd_device(args...)
	{
		//	t = new std::thread(magicthread);
		// give the thread some time to start and set up
		//	std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	virtual ~uhd_wrap();

	//    void ipc_sock_close() override {};
	int wrap_read(TIMESTAMP *timestamp);
	virtual int open(const std::string &args, int ref, bool swap_channels) override;

	//	bool start() override;
	//	bool stop() override;
	//	virtual TIMESTAMP initialWriteTimestamp() override;
	//	virtual TIMESTAMP initialReadTimestamp() override;

	int getTimingOffset()
	{
		return ts_offset;
	}
	size_t bufsizerx();
	size_t bufsizetx();
	int chancount();
};
#else
void *uhdwrap_open(struct ipc_sk_if_open_req *open_req);

int32_t uhdwrap_get_bufsizerx(void *dev);

int32_t uhdwrap_get_timingoffset(void *dev);

int32_t uhdwrap_read(void *dev, uint32_t num_chans);

int32_t uhdwrap_write(void *dev, uint32_t num_chans, bool *underrun);

double uhdwrap_set_freq(void *dev, double f, size_t chan, bool for_tx);

double uhdwrap_set_gain(void *dev, double f, size_t chan, bool for_tx);

int32_t uhdwrap_start(void *dev, int chan);

int32_t uhdwrap_stop(void *dev, int chan);

void uhdwrap_fill_info_cnf(struct ipc_sk_if *ipc_prim);
#endif

#endif // IPC_B210_H
