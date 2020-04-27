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
#include "ipc_shm.h"
#include "ipc-driver-test.h"
}
#include "../uhd/UHDDevice.h"
#include "uhdwrap.h"

#include "trx_vty.h"
#include "Logger.h"
#include "Threads.h"
#include "Utils.h"

int uhd_wrap::open(const std::string &args, int ref, bool swap_channels)
{
	int rv = uhd_device::open(args, ref, swap_channels);
	samps_per_buff_rx = rx_stream->get_max_num_samps();
	samps_per_buff_tx = tx_stream->get_max_num_samps();
	channel_count = usrp_dev->get_rx_num_channels();

	wrap_rx_buffs = std::vector<std::vector<short> >(channel_count, std::vector<short>(2 * samps_per_buff_rx));
	for (size_t i = 0; i < wrap_rx_buffs.size(); i++)
		wrap_rx_buf_ptrs.push_back(&wrap_rx_buffs[i].front());

	wrap_tx_buffs = std::vector<std::vector<short> >(channel_count, std::vector<short>(2 * 5000));
	for (size_t i = 0; i < wrap_tx_buffs.size(); i++)
		wrap_tx_buf_ptrs.push_back(&wrap_tx_buffs[i].front());

	return rv;
}

uhd_wrap::~uhd_wrap()
{
	//	drvtest::gshutdown = 1;
	//t->join();
}

size_t uhd_wrap::bufsizerx()
{
	return samps_per_buff_rx;
}

size_t uhd_wrap::bufsizetx()
{
	return samps_per_buff_tx;
}

int uhd_wrap::chancount()
{
	return channel_count;
}

int uhd_wrap::wrap_read(TIMESTAMP *timestamp)
{
	uhd::rx_metadata_t md;
	size_t num_rx_samps = rx_stream->recv(wrap_rx_buf_ptrs, samps_per_buff_rx, md, 0.1, true);
	*timestamp = md.time_spec.to_ticks(rx_rate);
	return num_rx_samps; //uhd_device::readSamples(bufs, len, overrun, timestamp, underrun);
}

extern "C" void *uhdwrap_open(struct ipc_sk_if_open_req *open_req)
{
	unsigned int rx_sps, tx_sps;

	/* FIXME: dev arg string* */
	/* FIXME: rx frontend bw? */
	/* FIXME: tx frontend bw? */
	ReferenceType cref;
	switch (open_req->clockref) {
	case FEATURE_MASK_CLOCKREF_EXTERNAL:
		cref = ReferenceType::REF_EXTERNAL;
		break;
	case FEATURE_MASK_CLOCKREF_INTERNAL:
	default:
		cref = ReferenceType::REF_INTERNAL;
		break;
	}

	std::vector<std::string> tx_paths;
	std::vector<std::string> rx_paths;
	for (unsigned int i = 0; i < open_req->num_chans; i++) {
		tx_paths.push_back(open_req->chan_info[i].tx_path);
		rx_paths.push_back(open_req->chan_info[i].rx_path);
	}

	rx_sps = open_req->rx_sample_freq_num / open_req->rx_sample_freq_den;
	tx_sps = open_req->tx_sample_freq_num / open_req->tx_sample_freq_den;
	uhd_wrap *uhd_wrap_dev =
		new uhd_wrap(tx_sps, rx_sps, RadioDevice::NORMAL, open_req->num_chans, 0.0, tx_paths, rx_paths);
	uhd_wrap_dev->open("", cref, false);

	return uhd_wrap_dev;
}
extern "C" int32_t uhdwrap_get_bufsizerx(void *dev)
{
	uhd_wrap *d = (uhd_wrap *)dev;
	return d->bufsizerx();
}

extern "C" int32_t uhdwrap_get_timingoffset(void *dev)
{
	uhd_wrap *d = (uhd_wrap *)dev;
	return d->getTimingOffset();
}

extern "C" int32_t uhdwrap_read(void *dev, uint32_t num_chans)
{
	TIMESTAMP t;
	uhd_wrap *d = (uhd_wrap *)dev;

	if (num_chans != d->wrap_rx_buf_ptrs.size()) {
		perror("omg chans?!");
	}

	int32_t read = d->wrap_read(&t);
	if (read < 0)
		return read;

	for (uint32_t i = 0; i < num_chans; i++) {
		ipc_shm_enqueue(ios_rx_from_device[i], t, read, (uint16_t *)&d->wrap_rx_buffs[i].front());
	}
	return read;
}

extern "C" int32_t uhdwrap_write(void *dev, uint32_t num_chans, bool *underrun)
{
	uhd_wrap *d = (uhd_wrap *)dev;

	uint64_t timestamp;
	int32_t len;
	for (uint32_t i = 0; i < num_chans; i++) {
		len = ipc_shm_read(ios_tx_to_device[i], (uint16_t *)&d->wrap_tx_buffs[i].front(), 5000, &timestamp, 1);
		if (len < 0)
			return 0;
	}

	return d->writeSamples(d->wrap_tx_buf_ptrs, len, underrun, timestamp);
}

extern "C" double uhdwrap_set_freq(void *dev, double f, size_t chan, bool for_tx)
{
	uhd_wrap *d = (uhd_wrap *)dev;
	if (for_tx)
		return d->setTxFreq(f, chan);
	else
		return d->setRxFreq(f, chan);
}

extern "C" double uhdwrap_set_gain(void *dev, double f, size_t chan, bool for_tx)
{
	uhd_wrap *d = (uhd_wrap *)dev;
	if (for_tx)
		return d->setTxGain(f, chan);
	else
		return d->setRxGain(f, chan);
}

extern "C" int32_t uhdwrap_start(void *dev, int chan)
{
	uhd_wrap *d = (uhd_wrap *)dev;
	return d->start();
}

extern "C" int32_t uhdwrap_stop(void *dev, int chan)
{
	uhd_wrap *d = (uhd_wrap *)dev;
	return d->stop();
}

extern "C" void uhdwrap_fill_info_cnf(struct ipc_sk_if *ipc_prim)
{
	struct ipc_sk_if_info_chan *chan_info;

	uhd::device_addr_t args("");
	uhd::device_addrs_t devs_found = uhd::device::find(args);
	if (devs_found.size() < 1) {
		std::cout << "\n No device found!";
		exit(0);
	}

	uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(devs_found[0]);
	auto rxchans = usrp->get_rx_num_channels();
	auto txchans = usrp->get_tx_num_channels();
	auto rx_range = usrp->get_rx_gain_range();
	auto tx_range = usrp->get_tx_gain_range();

	//auto nboards = usrp->get_num_mboards();
	auto refs = usrp->get_clock_sources(0);
	auto devname = usrp->get_mboard_name(0);

	ipc_prim->u.info_cnf.feature_mask = 0;
	if (std::find(refs.begin(), refs.end(), "internal") != refs.end())
		ipc_prim->u.info_cnf.feature_mask |= FEATURE_MASK_CLOCKREF_INTERNAL;
	if (std::find(refs.begin(), refs.end(), "external") != refs.end())
		ipc_prim->u.info_cnf.feature_mask |= FEATURE_MASK_CLOCKREF_EXTERNAL;

	// at least one duplex channel
	auto num_chans = rxchans == txchans ? txchans : 1;

	ipc_prim->u.info_cnf.min_rx_gain = rx_range.start();
	ipc_prim->u.info_cnf.max_rx_gain = rx_range.stop();
	ipc_prim->u.info_cnf.min_tx_gain = tx_range.start();
	ipc_prim->u.info_cnf.max_tx_gain = tx_range.stop();
	ipc_prim->u.info_cnf.iq_scaling_val_rx = 0.3;
	ipc_prim->u.info_cnf.iq_scaling_val_tx = 1;
	ipc_prim->u.info_cnf.max_num_chans = num_chans;
	OSMO_STRLCPY_ARRAY(ipc_prim->u.info_cnf.dev_desc, devname.c_str());
	chan_info = ipc_prim->u.info_cnf.chan_info;
	for (unsigned int i = 0; i < ipc_prim->u.info_cnf.max_num_chans; i++) {
		auto rxant = usrp->get_rx_antennas(i);
		auto txant = usrp->get_tx_antennas(i);
		for (unsigned int j = 0; j < txant.size(); j++) {
			OSMO_STRLCPY_ARRAY(chan_info->tx_path[j], txant[j].c_str());
		}
		for (unsigned int j = 0; j < rxant.size(); j++) {
			OSMO_STRLCPY_ARRAY(chan_info->rx_path[j], rxant[j].c_str());
		}
		chan_info++;
	}
}
