/*
* Copyright 2020 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
* Author: Pau Espin Pedrol <pespin@sysmocom.de>
*
* SPDX-License-Identifier: AGPL-3.0+
*
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */

#include <map>

#include "trx_vty.h"
#include "Logger.h"
#include "Threads.h"
#include "Utils.h"
#include "IPCDevice.h"

extern "C" {
#include "osmo_signal.h"
#include <osmocom/core/application.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/select.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
}

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define SAMPLE_BUF_SZ    (1 << 20)

using namespace std;

IPCDevice::IPCDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chan_num, double lo_offset,
		     const std::vector<std::string>& tx_paths,
		     const std::vector<std::string>& rx_paths):
	RadioDevice(tx_sps, rx_sps, iface, chan_num, lo_offset, tx_paths, rx_paths),
	tmp_state(IPC_IF_MSG_GREETING_REQ), shm(NULL), started(false)
{
	LOGC(DDEV, INFO) << "creating IPC device...";

	//m_IPC_stream_rx.resize(chans);
	//m_IPC_stream_tx.resize(chans);
	rx_gains.resize(chans);
	tx_gains.resize(chans);

	rx_buffers.resize(chans);

	/* Set up per-channel Rx timestamp based Ring buffers */
	for (size_t i = 0; i < rx_buffers.size(); i++)
		rx_buffers[i] = new smpl_buf(SAMPLE_BUF_SZ / sizeof(uint32_t));

}

IPCDevice::~IPCDevice()
{
	//unsigned int i;
	LOGC(DDEV, INFO) << "Closing IPC device";
	/* disable all channels */

	for (size_t i = 0; i < rx_buffers.size(); i++)
		delete rx_buffers[i];
}

int IPCDevice::ipc_shm_connect(const char *shm_name)
{
	int fd;
	size_t shm_len;
	int rc;

	LOGP(DDEV, LOGL_NOTICE, "Opening shm path %s\n", shm_name);
	if ((fd = shm_open(shm_name, O_CREAT|O_RDWR, S_IRUSR|S_IWUSR)) < 0) {
		LOGP(DDEV, LOGL_ERROR, "shm_open %d: %s\n", errno, strerror(errno));
                rc = -errno;
                goto err_shm_open;
        }

	// Get size of the allocated memory
	struct stat shm_stat;
	if (fstat(fd, &shm_stat) < 0) {
		LOGP(DDEV, LOGL_ERROR, "fstat %d: %s\n", errno, strerror(errno));
                rc = -errno;
                goto err_mmap;
	}

	shm_len = shm_stat.st_size;

	LOGP(DDEV, LOGL_NOTICE, "mmaping shared memory fd %d (size=%zu)\n", fd,shm_len);
        if ((shm = mmap(NULL, shm_len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0)) == MAP_FAILED) {
                LOGP(DDEV, LOGL_ERROR, "mmap %d: %s\n", errno, strerror(errno));
                rc = -errno;
                goto err_mmap;
        }
        LOGP(DDEV, LOGL_NOTICE, "mmap'ed shared memory at addr %p\n", shm);
	LOGP(DDEV, LOGL_NOTICE, "%s\n", osmo_hexdump((const unsigned char *) shm, 80));
        /* After a call to mmap(2) the file descriptor may be closed without affecting the memory mapping. */
        close(fd);
        return 0;
err_mmap:
        shm_unlink(shm_name);
        close(fd);
err_shm_open:
        return rc;
}

static int ipc_sock_send(struct ipc_sock_state* state, struct msgb *msg);

struct msgb *ipc_msgb_alloc(uint8_t msg_type)
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;

	msg = msgb_alloc(sizeof(struct ipc_sk_if) + 1000, "ipc_sock_tx");
	if (!msg)
		return NULL;
	msgb_put(msg, sizeof(struct ipc_sk_if) + 1000);
	ipc_prim = (struct ipc_sk_if *) msg->data;
	ipc_prim->msg_type = msg_type;

	return msg;
}

int ipc_tx_greeting_req(struct ipc_sock_state* state, uint8_t req_version)
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;

	LOGC(DDEV, NOTICE) << "Tx Greeting Req (" << IPC_IF_MSG_GREETING_REQ << ")\n";

	msg = ipc_msgb_alloc(IPC_IF_MSG_GREETING_REQ);
	if (!msg) {
		LOGC(DDEV, INFO) << "ipc_msgb_alloc() returns NULL!";
		return -ENOMEM;
	}
	ipc_prim = (struct ipc_sk_if *) msg->data;
	ipc_prim->u.greeting_req.req_version = req_version;

	return ipc_sock_send(state, msg);
}

int ipc_tx_info_req(struct ipc_sock_state* state)
{
	struct msgb *msg;
	//struct ipc_sk_if *ipc_prim;

	LOGC(DDEV, NOTICE) << "Tx INFO Req\n";

	msg = ipc_msgb_alloc(IPC_IF_MSG_INFO_REQ);
	if (!msg)
		return -ENOMEM;

	//ipc_prim = (struct ipc_sk_if *) msg->data;

	return ipc_sock_send(state, msg);
}

int ipc_tx_open_req(struct ipc_sock_state* state, uint32_t num_chans)
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;
	struct ipc_sk_if_open_req_chan *chan_info;

	LOGC(DDEV, NOTICE) << "Tx Open Req\n";

	msg = ipc_msgb_alloc(IPC_IF_MSG_OPEN_REQ);
	if (!msg) {
		return -ENOMEM;
	}
	ipc_prim = (struct ipc_sk_if *) msg->data;
	ipc_prim->u.open_req.num_chans = num_chans;
	chan_info = ipc_prim->u.open_req.chan_info;
	OSMO_STRLCPY_ARRAY(chan_info->rx_path, "RxAntenna1");
	OSMO_STRLCPY_ARRAY(chan_info->tx_path, "TxAntenna2");

	return ipc_sock_send(state, msg);
}

static void ipc_sock_timeout(void *_priv)
{
	LOGC(DDEV, INFO) << "UNIX SOCKET TIMEOUT!";
	exit(1);
}

int IPCDevice::ipc_rx_greeting_cnf(const struct ipc_sk_if_greeting *greeting_cnf)
{
        if (greeting_cnf->req_version == IPC_SOCK_API_VERSION) {
                LOGC(DDEV, NOTICE) << "Rx Greeting CNF: correct sock API version" << greeting_cnf->req_version;
		tmp_state = IPC_IF_MSG_GREETING_CNF;
        } else {
                LOGC(DDEV, ERROR) << "Wrong IPC SOCK API VERSION RECEIVED!" << greeting_cnf->req_version;
		exit(1);
	}
        return 0;
}

int IPCDevice::ipc_rx_info_cnf(const struct ipc_sk_if_info_cnf *info_cnf)
{
	/* Here:
	 * verify info_cnf->max_num_chans >= requested chans
	 * verify supports setting reflock as asked by user looking in info_cnf->feature_mask
	 * cache locally min/max tx/rxGain values from info_cnf
	 * do whatever validations or print info_cnf->dev_desc
	 * cache rx/tx paths per channel, and make sure it matches the one the user wants to set
	 */
	unsigned int i;
	LOGC(DDEV, NOTICE) << "Rx Info CNF:"
			  << " max_num_chans=" << info_cnf->max_num_chans
			  << " feature_mask=" << info_cnf->feature_mask
			  << " min_rx_gain=" << info_cnf->min_rx_gain
			  << " max_rx_gain=" << info_cnf->max_rx_gain;
	for (i = 0; i < info_cnf->max_num_chans; i++) {
		int j = 0;
		while (strcmp(info_cnf->chan_info[i].rx_path[j],"")!=0) {
			LOGC(DDEV, NOTICE) << "chan " << i << ": RxPath[" << j <<"]: " << info_cnf->chan_info[i].rx_path[j];
			j++;
		}
		j = 0;
		while (strcmp(info_cnf->chan_info[i].tx_path[j],"")!=0) {
			LOGC(DDEV, NOTICE) << "chan " << i << ": TxPath[" << j <<"]: " << info_cnf->chan_info[i].tx_path[j];
			j++;
		}
	}
        tmp_state = IPC_IF_MSG_INFO_CNF;
        return 0;
}

int IPCDevice::ipc_rx_open_cnf(const struct ipc_sk_if_open_cnf *open_cnf)
{
	unsigned int i;
	LOGC(DDEV, NOTICE) << "Rx Open CNF:"
			  << " return_code=" << (unsigned int)open_cnf->return_code
			  << " shm_name=" << open_cnf->shm_name;
	for (i = 0; i < chans; i++) {
		LOGC(DDEV, NOTICE) << "chan " << i << ": sk_path=" << open_cnf->chan_info[i].chan_ipc_sk_path;
	}

	OSMO_STRLCPY_ARRAY(shm_name, open_cnf->shm_name);
	if (ipc_shm_connect(shm_name) < 0)
		return -1;
	shm_dec = ipc_shm_decode_region(NULL, (ipc_shm_raw_region*)shm);
	LOGC(DDEV, NOTICE) << "shm: num_chans=" << shm_dec->num_chans;
	LOGC(DDEV, NOTICE) << "shm: chan0/dl: num_buffers=" << shm_dec->channels[0]->dl_stream->num_buffers;
	LOGC(DDEV, NOTICE) << "shm: chan0/dl: buffer_size=" << shm_dec->channels[0]->dl_stream->buffer_size;

        tmp_state = IPC_IF_MSG_OPEN_CNF;
        return 0;
}

int IPCDevice::ipc_rx(uint8_t msg_type, struct ipc_sk_if *ipc_prim)
{
	int rc = 0;

	switch (msg_type) {
	case IPC_IF_MSG_GREETING_CNF:
                rc = ipc_rx_greeting_cnf(&ipc_prim->u.greeting_cnf);
                break;
	case IPC_IF_MSG_INFO_CNF:
		rc = ipc_rx_info_cnf(&ipc_prim->u.info_cnf);
		break;
	case IPC_IF_MSG_OPEN_CNF:
		rc = ipc_rx_open_cnf(&ipc_prim->u.open_cnf);
		break;
	default:
		LOGP(DDEV, LOGL_ERROR, "Received unknown PCU msg type %d\n",
			msg_type);
		rc = -EINVAL;
	}

	return rc;
}

static int ipc_sock_send(struct ipc_sock_state* state, struct msgb *msg)
{
	struct osmo_fd *conn_bfd;
	//struct ipc_sk_if *ipc_prim = (struct ipc_sk_if *) msg->data;

	if (!state) {
		LOGP(DMAIN, LOGL_INFO, "PCU socket not created, "
			"dropping message\n");
		msgb_free(msg);
		return -EINVAL;
	}
	conn_bfd = &state->conn_bfd;
	if (conn_bfd->fd <= 0) {
		LOGP(DMAIN, LOGL_NOTICE, "PCU socket not connected, "
			"dropping message\n");
		msgb_free(msg);
		return -EIO;
	}
	msgb_enqueue(&state->upqueue, msg);
	conn_bfd->when |= BSC_FD_WRITE;

	return 0;
}

void IPCDevice::ipc_sock_close()
{
	struct osmo_fd *bfd = &sk_state.conn_bfd;

	LOGP(DDEV, LOGL_NOTICE, "PCU socket has LOST connection\n");

	close(bfd->fd);
	bfd->fd = -1;
	osmo_fd_unregister(bfd);

	/* flush the queue */
	while (!llist_empty(&sk_state.upqueue)) {
		struct msgb *msg = msgb_dequeue(&sk_state.upqueue);
		msgb_free(msg);
	}
}

int IPCDevice::ipc_sock_read(struct osmo_fd *bfd)
{
	struct ipc_sk_if *ipc_prim;
	struct msgb *msg;
	int rc;

	msg = msgb_alloc(sizeof(*ipc_prim) + 1000, "ipc_sock_rx");
	if (!msg)
		return -ENOMEM;

	ipc_prim = (struct ipc_sk_if *) msg->tail;

	rc = recv(bfd->fd, msg->tail, msgb_tailroom(msg), 0);
	if (rc == 0)
		goto close;

	if (rc < 0) {
		if (errno == EAGAIN) {
			msgb_free(msg);
			return 0;
		}
		goto close;
	}

	if ((size_t)rc < sizeof(*ipc_prim)) {
		LOGP(DDEV, LOGL_ERROR, "Received %d bytes on Unix Socket, but primitive size "
		     "is %zu, discarding\n", rc, sizeof(*ipc_prim));
		msgb_free(msg);
		return 0;
	}

	rc = ipc_rx(ipc_prim->msg_type, ipc_prim);

	/* as we always synchronously process the message in pcu_rx() and
	 * its callbacks, we can free the message here. */
	msgb_free(msg);

	return rc;

close:
	msgb_free(msg);
	ipc_sock_close();
	return -1;
}

int IPCDevice::ipc_sock_write(struct osmo_fd *bfd)
{
	int rc;

	while (!llist_empty(&sk_state.upqueue)) {
		struct msgb *msg, *msg2;
		struct ipc_sk_if *ipc_prim;

		/* peek at the beginning of the queue */
		msg = llist_entry(sk_state.upqueue.next, struct msgb, list);
		ipc_prim = (struct ipc_sk_if *)msg->data;

		bfd->when &= ~BSC_FD_WRITE;

		/* bug hunter 8-): maybe someone forgot msgb_put(...) ? */
		if (!msgb_length(msg)) {
			LOGP(DDEV, LOGL_ERROR, "message type (%d) with ZERO "
				"bytes!\n", ipc_prim->msg_type);
			goto dontsend;
		}

		/* try to send it over the socket */
		rc = write(bfd->fd, msgb_data(msg), msgb_length(msg));
		if (rc == 0)
			goto close;
		if (rc < 0) {
			if (errno == EAGAIN) {
				bfd->when |= BSC_FD_WRITE;
				break;
			}
			goto close;
		}

dontsend:
		/* _after_ we send it, we can deueue */
		msg2 = msgb_dequeue(&sk_state.upqueue);
		assert(msg == msg2);
		msgb_free(msg);
	}
	return 0;

close:
	ipc_sock_close();
	return -1;
}

static int ipc_sock_cb(struct osmo_fd *bfd, unsigned int flags)
{
	IPCDevice *device = (IPCDevice *)bfd->data;
	int rc = 0;

	if (flags & BSC_FD_READ)
		rc = device->ipc_sock_read(bfd);
	if (rc < 0)
		return rc;

	if (flags & BSC_FD_WRITE)
		rc = device->ipc_sock_write(bfd);

	return rc;
}

int IPCDevice::open(const std::string &args, int ref, bool swap_channels)
{
	//float_type sr_host, sr_rf;
	//unsigned int i, n;
	//int rc, dev_id;
	int rc;

	LOGC(DDEV, INFO) << "Opening IPC device..";

	memset(&sk_state, 0x00, sizeof(sk_state));
	INIT_LLIST_HEAD(&sk_state.upqueue);
	rc = osmo_sock_unix_init_ofd(&sk_state.conn_bfd, SOCK_SEQPACKET, 0,
				     IPC_SOCK_PATH, OSMO_SOCK_F_CONNECT);
	if (rc < 0) {
		LOGC(DDEV, ERROR) << "Failed to connect to the BTS (" << IPC_SOCK_PATH << "). " <<
					"Retrying...\n";
		osmo_timer_setup(&sk_state.timer, ipc_sock_timeout, NULL);
		osmo_timer_schedule(&sk_state.timer, 5, 0);
		return -1;
	}
	sk_state.conn_bfd.cb = ipc_sock_cb;
	sk_state.conn_bfd.data = this;

	ipc_tx_greeting_req(&sk_state, IPC_SOCK_API_VERSION);
	/* Wait until confirmation is recieved */
	while(tmp_state != IPC_IF_MSG_GREETING_CNF)
		osmo_select_main(0);

	ipc_tx_info_req(&sk_state);
	/* Wait until confirmation is recieved */
	while(tmp_state != IPC_IF_MSG_INFO_CNF)
		osmo_select_main(0);

	ipc_tx_open_req(&sk_state, chans);
	/* Wait until confirmation is recieved */
	while(tmp_state != IPC_IF_MSG_OPEN_CNF)
		osmo_select_main(0);
	LOGC(DDEV, NOTICE) << "Device driver opened successfuly!";

	/* configure antennas */
	if (!set_antennas()) {
		LOGC(DDEV, FATAL) << "IPC antenna setting failed";
		goto out_close;
	}

	return iface == MULTI_ARFCN ? MULTI_ARFCN : NORMAL;

out_close:
	LOGC(DDEV, FATAL) << "Error in IPC open, closing";
	return -1;
}

bool IPCDevice::start()
{
	LOGC(DDEV, INFO) << "starting IPC...";

	unsigned int i;

	if (started) {
		LOGC(DDEV, ERR) << "Device already started";
		return false;
	}

	/* configure the channels/streams */
	for (i=0; i<chans; i++) {
		/* Set gains for calibration/filter setup */
		/* TX gain to maximum */
		setTxGain(maxTxGain(), i);
		/* RX gain to midpoint */
		setRxGain((minRxGain() + maxRxGain()) / 2, i);

		/* set up Rx and Tx filters */
		if (!do_filters(i))
			return false;
		/* Perform Rx and Tx calibration */
		if (!do_calib(i))
			return false;

		/* configure Streams */
	}

	/* now start the streams in a second loop, as we can no longer call
	 * IPC_SetupStream() after IPC_StartStream() of the first stream */
	//for (i = 0; i < chans; i++) {
	//}

	flush_recv(10);

	started = true;
	return true;
}

bool IPCDevice::stop()
{
	//unsigned int i;

	if (!started)
		return true;

	/*
	for (i=0; i<chans; i++) {
		IPC_StopStream(&m_IPC_stream_tx[i]);
		IPC_StopStream(&m_IPC_stream_rx[i]);
	}

	for (i=0; i<chans; i++) {
		IPC_DestroyStream(m_IPC_dev, &m_IPC_stream_tx[i]);
		IPC_DestroyStream(m_IPC_dev, &m_IPC_stream_rx[i]);
	}*/

	started = false;
	return true;
}

/* do rx/tx calibration - depends on gain, freq and bw */
bool IPCDevice::do_calib(size_t chan)
{
	LOGCHAN(chan, DDEV, INFO) << "Calibrating";
	return true;
}

/* do rx/tx filter config - depends on bw only? */
bool IPCDevice::do_filters(size_t chan)
{
	LOGCHAN(chan, DDEV, INFO) << "Setting filters";
	return true;
}


double IPCDevice::maxTxGain()
{
	//return dev_param_map.at(m_dev_type).max_tx_gain;
	return 70.0;
}

double IPCDevice::minTxGain()
{
	return 0.0;
}

double IPCDevice::maxRxGain()
{
	return 73.0;
}

double IPCDevice::minRxGain()
{
	return 0.0;
}

double IPCDevice::setTxGain(double dB, size_t chan)
{
	if (dB > maxTxGain())
		dB = maxTxGain();
	if (dB < minTxGain())
		dB = minTxGain();

	LOGCHAN(chan, DDEV, NOTICE) << "Setting TX gain to " << dB << " dB";

	//if (IPC_SetGaindB(m_IPC_dev, IPC_CH_TX, chan, dB) < 0)
	//	LOGCHAN(chan, DDEV, ERR) << "Error setting TX gain to " << dB << " dB";
	//else
		tx_gains[chan] = dB;
	return tx_gains[chan];
}

double IPCDevice::setRxGain(double dB, size_t chan)
{
	if (dB > maxRxGain())
		dB = maxRxGain();
	if (dB < minRxGain())
		dB = minRxGain();

	LOGCHAN(chan, DDEV, NOTICE) << "Setting RX gain to " << dB << " dB";

	//if (IPC_SetGaindB(m_IPC_dev, IPC_CH_RX, chan, dB) < 0)
	//	LOGCHAN(chan, DDEV, ERR) << "Error setting RX gain to " << dB << " dB";
	//else
		rx_gains[chan] = dB;
	return rx_gains[chan];
}


void IPCDevice::log_ant_list(bool dir_tx, size_t chan, std::ostringstream& os)
{
	int num_names = 0;
	int i;
	for (i = 0; i < num_names; i++) {
		if (i)
			os << ", ";
		os << "''";
	}
}

bool IPCDevice::flush_recv(size_t num_pkts)
{/*
	#define CHUNK 625
	int len = CHUNK * tx_sps;
	short *buffer = (short*) alloca(sizeof(short) * len * 2);
	int rc;
	IPC_stream_meta_t rx_metadata = {};
	rx_metadata.flushPartialPacket = false;
	rx_metadata.waitForTimestamp = false;

	ts_initial = 0;

	while (!ts_initial || (num_pkts-- > 0)) {
		rc = IPC_RecvStream(&m_IPC_stream_rx[0], &buffer[0], len, &rx_metadata, 100);
		LOGC(DDEV, DEBUG) << "Flush: Recv buffer of len " << rc << " at " << std::hex << rx_metadata.timestamp;
		if (rc != len) {
			LOGC(DDEV, ERROR) << "Flush: Device receive timed out";
			return false;
		}

		ts_initial = rx_metadata.timestamp + len;
	}
*/
	LOGC(DDEV, INFO) << "Initial timestamp " << ts_initial << std::endl;
	return true;
}

bool IPCDevice::setRxAntenna(const std::string & ant, size_t chan)
{
	return true;
}

std::string IPCDevice::getRxAntenna(size_t chan)
{
	return "";

}

bool IPCDevice::setTxAntenna(const std::string & ant, size_t chan)
{
	return true;
}

std::string IPCDevice::getTxAntenna(size_t chan)
{
	return "";
}

bool IPCDevice::requiresRadioAlign()
{
	return false;
}

GSM::Time IPCDevice::minLatency() {
	/* UNUSED */
	return GSM::Time(0,0);
}

// NOTE: Assumes sequential reads
int IPCDevice::readSamples(std::vector < short *>&bufs, int len, bool * overrun,
			   TIMESTAMP timestamp, bool * underrun)
{
	return len;
}

int IPCDevice::writeSamples(std::vector < short *>&bufs, int len,
			    bool * underrun, unsigned long long timestamp)
{
	return len;
}

bool IPCDevice::updateAlignment(TIMESTAMP timestamp)
{
	return true;
}

bool IPCDevice::setTxFreq(double wFreq, size_t chan)
{
	LOGCHAN(chan, DDEV, NOTICE) << "Setting Tx Freq to " << wFreq << " Hz";

	return true;
}

bool IPCDevice::setRxFreq(double wFreq, size_t chan)
{
	LOGCHAN(chan, DDEV, NOTICE) << "Setting Rx Freq to " << wFreq << " Hz";

	return true;
}

RadioDevice *RadioDevice::make(size_t tx_sps, size_t rx_sps,
			       InterfaceType iface, size_t chans, double lo_offset,
			       const std::vector < std::string > &tx_paths,
			       const std::vector < std::string > &rx_paths)
{
	if (tx_sps != rx_sps) {
		LOGC(DDEV, ERROR) << "IPC Requires tx_sps == rx_sps";
		return NULL;
	}
	if (lo_offset != 0.0) {
		LOGC(DDEV, ERROR) << "IPC doesn't support lo_offset";
		return NULL;
	}
	return new IPCDevice(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths);
}
