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

#include <cstdint>
#include <cstring>
#include <cstdlib>


#include <map>

#include "trx_vty.h"
#include "Logger.h"
#include "Threads.h"
#include "Utils.h"
#include "IPCDevice.h"

extern "C" {
#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <fcntl.h> /* For O_* constants */

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

#define SAMPLE_BUF_SZ (1 << 20)

using namespace std;

static int ipc_chan_sock_cb(struct osmo_fd *bfd, unsigned int flags);

IPCDevice::IPCDevice(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chan_num, double lo_offset,
		     const std::vector<std::string> &tx_paths, const std::vector<std::string> &rx_paths)
	: RadioDevice(tx_sps, rx_sps, iface, chan_num, lo_offset, tx_paths, rx_paths),
	   tx_attenuation(), tmp_state(IPC_IF_MSG_GREETING_REQ), shm(NULL), shm_dec(0), started(false)
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

	memset(&sk_chan_state, 0, sizeof(sk_chan_state));
	memset(&trx_is_started, 0, sizeof(trx_is_started));
}

IPCDevice::~IPCDevice()
{
	//unsigned int i;
	LOGC(DDEV, INFO) << "Closing IPC device";
	/* disable all channels */

	for (size_t i = 0; i < rx_buffers.size(); i++)
		delete rx_buffers[i];

	ipc_sock_close(&master_sk_state);

	for (unsigned int i = 0; i < ARRAY_SIZE(sk_chan_state); i++)
		ipc_sock_close(&sk_chan_state[i]);

	for (auto i : shm_io_rx_streams)
		ipc_shm_close(i);
	for (auto i : shm_io_tx_streams)
		ipc_shm_close(i);

	if(shm_dec)
		talloc_free(shm_dec);
}

int IPCDevice::ipc_shm_connect(const char *shm_name)
{
	int fd;
	size_t shm_len;
	int rc;

	LOGP(DDEV, LOGL_NOTICE, "Opening shm path %s\n", shm_name);
	if ((fd = shm_open(shm_name, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR)) < 0) {
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

	LOGP(DDEV, LOGL_NOTICE, "mmaping shared memory fd %d (size=%zu)\n", fd, shm_len);
	if ((shm = mmap(NULL, shm_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) == MAP_FAILED) {
		LOGP(DDEV, LOGL_ERROR, "mmap %d: %s\n", errno, strerror(errno));
		rc = -errno;
		goto err_mmap;
	}
	LOGP(DDEV, LOGL_NOTICE, "mmap'ed shared memory at addr %p\n", shm);
//	LOGP(DDEV, LOGL_NOTICE, "%s\n", osmo_hexdump((const unsigned char *)shm, 80));
	/* After a call to mmap(2) the file descriptor may be closed without affecting the memory mapping. */
	close(fd);
	return 0;
err_mmap:
	shm_unlink(shm_name);
	close(fd);
err_shm_open:
	return rc;
}

static int ipc_sock_send(struct ipc_sock_state *state, struct msgb *msg);

static struct msgb *ipc_msgb_alloc(uint8_t msg_type)
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;

	msg = msgb_alloc(sizeof(struct ipc_sk_if) + 1000, "ipc_sock_tx");
	if (!msg)
		return NULL;
	msgb_put(msg, sizeof(struct ipc_sk_if) + 1000);
	ipc_prim = (struct ipc_sk_if *)msg->data;
	ipc_prim->msg_type = msg_type;

	return msg;
}

static int ipc_tx_greeting_req(struct ipc_sock_state *state, uint8_t req_version)
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;

	LOGC(DDEV, NOTICE) << "Tx Greeting Req (" << IPC_IF_MSG_GREETING_REQ << ")\n";

	msg = ipc_msgb_alloc(IPC_IF_MSG_GREETING_REQ);
	if (!msg) {
		LOGC(DDEV, INFO) << "ipc_msgb_alloc() returns NULL!";
		return -ENOMEM;
	}
	ipc_prim = (struct ipc_sk_if *)msg->data;
	ipc_prim->u.greeting_req.req_version = req_version;

	return ipc_sock_send(state, msg);
}

static int ipc_tx_info_req(struct ipc_sock_state *state)
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

int IPCDevice::ipc_tx_open_req(struct ipc_sock_state *state, uint32_t num_chans, uint32_t ref)
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;
	struct ipc_sk_if_open_req_chan *chan_info;

	LOGC(DDEV, NOTICE) << "Tx Open Req\n";

	msg = ipc_msgb_alloc(IPC_IF_MSG_OPEN_REQ);
	if (!msg) {
		return -ENOMEM;
	}
	ipc_prim = (struct ipc_sk_if *)msg->data;
	ipc_prim->u.open_req.num_chans = num_chans;

	/* FIXME: this is actually the sps value, not the sample rate!
	 * sample rate is looked up according to the sps rate by uhd backend */
	ipc_prim->u.open_req.rx_sample_freq_num = rx_sps;
	ipc_prim->u.open_req.rx_sample_freq_den = 1;
	ipc_prim->u.open_req.tx_sample_freq_num = tx_sps;
	ipc_prim->u.open_req.tx_sample_freq_den = 1;

	switch (ref) {
	case ReferenceType::REF_EXTERNAL:
		ipc_prim->u.open_req.clockref = FEATURE_MASK_CLOCKREF_EXTERNAL;
		break;
	case ReferenceType::REF_INTERNAL:
	case ReferenceType::REF_GPS:
		ipc_prim->u.open_req.clockref = FEATURE_MASK_CLOCKREF_INTERNAL;
		break;
	}

	/* FIXME: clock ref part of config, not open */
	ipc_prim->u.open_req.clockref = FEATURE_MASK_CLOCKREF_EXTERNAL;

	for (unsigned int i = 0; i < num_chans; i++) {
		chan_info = &ipc_prim->u.open_req.chan_info[i];
		OSMO_STRLCPY_ARRAY(chan_info->rx_path, rx_paths[i].c_str());
		OSMO_STRLCPY_ARRAY(chan_info->tx_path, tx_paths[i].c_str());
	}

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
	current_info_cnf = *info_cnf;
	unsigned int i;

	if (info_cnf->max_num_chans < chans) {
		LOGC(DDEV, ERROR) << "chan num mismatch:" << info_cnf->max_num_chans << " vs " << chans;
		return -1;
	}

	/* Here:
	 * verify info_cnf->max_num_chans >= requested chans
	 * verify supports setting reflock as asked by user looking in info_cnf->feature_mask
	 * cache locally min/max tx/rxGain values from info_cnf
	 * do whatever validations or print info_cnf->dev_desc
	 * cache rx/tx paths per channel, and make sure it matches the one the user wants to set
	 */

	LOGC(DDEV, NOTICE) << "Rx Info CNF:"
			   << " name=" << info_cnf->dev_desc << std::endl
			   << " max_num_chans=" << info_cnf->max_num_chans << " feature_mask=" << info_cnf->feature_mask;
	for (i = 0; i < info_cnf->max_num_chans; i++) {
		int j = 0;
		bool rx_found = false, tx_found = false;
		while (strcmp(info_cnf->chan_info[i].rx_path[j], "") != 0) {
			LOGC(DDEV, NOTICE)
				<< "chan " << i << ": RxPath[" << j << "]: " << info_cnf->chan_info[i].rx_path[j]
				<< " min_rx_gain=" << info_cnf->chan_info[i].min_rx_gain << " max_rx_gain=" << info_cnf->chan_info[i].max_rx_gain
				<< " min_tx_gain=" << info_cnf->chan_info[i].min_tx_gain << " max_tx_gain=" << info_cnf->chan_info[i].max_tx_gain;

			if (rx_paths.size() < (i + 1) ||
			    strcmp(rx_paths[i].c_str(), info_cnf->chan_info[i].rx_path[j]) == 0) {
				rx_found = true;
				break;
			}
			j++;
		}
		j = 0;
		while (strcmp(info_cnf->chan_info[i].tx_path[j], "") != 0) {
			LOGC(DDEV, NOTICE)
				<< "chan " << i << ": TxPath[" << j << "]: " << info_cnf->chan_info[i].tx_path[j];
			if (tx_paths.size() < (i + 1) ||
			    strcmp(tx_paths[i].c_str(), info_cnf->chan_info[i].tx_path[j]) == 0) {
				tx_found = true;
				break;
			}
			j++;
		}

		if (!rx_found) {
			LOGC(DDEV, ERROR) << "rx antenna not found: " << rx_paths[i];
			exit(0);
		}
		if (!tx_found) {
			LOGC(DDEV, ERROR) << "tx antenna not found: " << rx_paths[i];
			exit(0);
		}
	}
	tmp_state = IPC_IF_MSG_INFO_CNF;
	return 0;
}

int IPCDevice::ipc_rx_open_cnf(const struct ipc_sk_if_open_cnf *open_cnf)
{
	unsigned int i;
	current_open_cnf = *open_cnf;

	LOGC(DDEV, NOTICE)
		<< "Rx Open CNF:"
		<< " return_code=" << (unsigned int)open_cnf->return_code << " shm_name=" << open_cnf->shm_name;
	LOGC(DDEV, NOTICE) << "Rx Open CNF:"
			   << " ipc device path delay: " << (unsigned int)open_cnf->path_delay;
	for (i = 0; i < chans; i++) {
		int rc;
		LOGC(DDEV, NOTICE) << "chan " << i << ": sk_path=" << open_cnf->chan_info[i].chan_ipc_sk_path;

		/* FIXME: current limit IPC_MAX_NUM_TRX chans, make dynamic */
		if (i < IPC_MAX_NUM_TRX) {
			struct ipc_sock_state *state = &sk_chan_state[i];
			memset(state, 0x00, sizeof(*state));

			INIT_LLIST_HEAD(&state->upqueue);
			rc = osmo_sock_unix_init_ofd(&state->conn_bfd, SOCK_SEQPACKET, 0,
						     open_cnf->chan_info[i].chan_ipc_sk_path, OSMO_SOCK_F_CONNECT);
			if (rc < 0) {
				LOGC(DDEV, ERROR) << "Failed to connect to the BTS ("
						  << open_cnf->chan_info[i].chan_ipc_sk_path << "). "
						  << "Retrying...\n";
				osmo_timer_setup(&state->timer, ipc_sock_timeout, NULL);
				osmo_timer_schedule(&state->timer, 5, 0);
				return -1;
			}
			state->conn_bfd.cb = ipc_chan_sock_cb;
			state->conn_bfd.data = this;
			state->conn_bfd.priv_nr = i;
		}
	}

	OSMO_STRLCPY_ARRAY(shm_name, open_cnf->shm_name);
	if (ipc_shm_connect(shm_name) < 0)
		return -1;
	shm_dec = ipc_shm_decode_region(NULL, (ipc_shm_raw_region *)shm);
	LOGC(DDEV, NOTICE) << "shm: num_chans=" << shm_dec->num_chans;

	/* server inits both producers */
	for (unsigned int i = 0; i < shm_dec->num_chans; i++) {
		LOGC(DDEV, NOTICE)
			<< "shm: chan" << i << "/dl: num_buffers=" << shm_dec->channels[i]->dl_stream->num_buffers;
		LOGC(DDEV, NOTICE)
			<< "shm: chan" << i << "/dl: buffer_size=" << shm_dec->channels[i]->dl_stream->buffer_size;
		LOGC(DDEV, NOTICE)
			<< "shm: chan" << i << "/ul: num_buffers=" << shm_dec->channels[i]->ul_stream->num_buffers;
		LOGC(DDEV, NOTICE)
			<< "shm: chan" << i << "/ul: buffer_size=" << shm_dec->channels[i]->ul_stream->buffer_size;
		shm_io_rx_streams.push_back(ipc_shm_init_consumer(shm_dec->channels[i]->ul_stream));
		shm_io_tx_streams.push_back(ipc_shm_init_consumer(shm_dec->channels[i]->dl_stream));
		//		shm_io_tx_streams.push_back(ipc_shm_init_producer(shm_dec->channels[i]->dl_stream));
	}

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
		LOGP(DDEV, LOGL_ERROR, "Received unknown IPC msg type %d\n", msg_type);
		rc = -EINVAL;
	}

	return rc;
}

int IPCDevice::ipc_rx_chan_start_cnf(ipc_sk_chan_if_op_rc *ret, uint8_t chan_nr)
{
	if(chan_nr >= ARRAY_SIZE(trx_is_started)) {
		LOGC(DDEV, NOTICE) << "shm: illegal start response for chan #" << chan_nr << " ?!?";
		return 0;
	}

	trx_is_started[chan_nr] = true;
	return 0;
}
int IPCDevice::ipc_rx_chan_stop_cnf(ipc_sk_chan_if_op_rc *ret, uint8_t chan_nr)
{
	if(chan_nr >= ARRAY_SIZE(trx_is_started)) {
		LOGC(DDEV, NOTICE) << "shm: illegal stop response for chan #" << chan_nr << " ?!?";
		return 0;
	}

	trx_is_started[chan_nr] = false;
	return 0;
}
int IPCDevice::ipc_rx_chan_setgain_cnf(ipc_sk_chan_if_gain *ret, uint8_t chan_nr)
{
	if(chan_nr >= ARRAY_SIZE(trx_is_started)) {
		LOGC(DDEV, NOTICE) << "shm: illegal setgain response for chan #" << chan_nr << " ?!?";
		return 0;
	}

	ret->is_tx ? tx_gains[chan_nr] = ret->gain : rx_gains[chan_nr] = ret->gain;
	return 0;
}
int IPCDevice::ipc_rx_chan_settxattn_cnf(ipc_sk_chan_if_tx_attenuation *ret, uint8_t chan_nr)
{
	if(chan_nr >= ARRAY_SIZE(trx_is_started)) {
		LOGC(DDEV, NOTICE) << "shm: illegal tx attn response for chan #" << chan_nr << " ?!?";
		return 0;
	}

//	trx_is_started[chan_nr] = false;
	return 0;
}
int IPCDevice::ipc_rx_chan_setfreq_cnf(ipc_sk_chan_if_freq_cnf *ret, uint8_t chan_nr)
{
	if(chan_nr >= ARRAY_SIZE(trx_is_started)) {
		LOGC(DDEV, NOTICE) << "shm: illegal setfreq response for chan #" << chan_nr << " ?!?";
		return 0;
	}

	return 0;
}
int IPCDevice::ipc_rx_chan_notify_underflow(ipc_sk_chan_if_notfiy *ret, uint8_t chan_nr)
{
	if(chan_nr >= ARRAY_SIZE(trx_is_started)) {
		LOGC(DDEV, NOTICE) << "shm: illegal underfloww notification for chan #" << chan_nr << " ?!?";
		return 0;
	}

	m_ctr[chan_nr].tx_underruns += 1;
	osmo_signal_dispatch(SS_DEVICE, S_DEVICE_COUNTER_CHANGE, &m_ctr[chan_nr]);

	return 0;
}
int IPCDevice::ipc_rx_chan_notify_overflow(ipc_sk_chan_if_notfiy *ret, uint8_t chan_nr)
{
	if(chan_nr >= ARRAY_SIZE(trx_is_started)) {
		LOGC(DDEV, NOTICE) << "shm: illegal overflow notification for chan #" << chan_nr << " ?!?";
		return 0;
	}

	m_ctr[chan_nr].rx_overruns += 1;
	osmo_signal_dispatch(SS_DEVICE, S_DEVICE_COUNTER_CHANGE, &m_ctr[chan_nr]);
	return 0;
}

int IPCDevice::ipc_chan_rx(uint8_t msg_type, struct ipc_sk_chan_if *ipc_prim, uint8_t chan_nr)
{
	int rc = 0;

	switch (msg_type) {
	case IPC_IF_MSG_START_CNF:
		rc = ipc_rx_chan_start_cnf(&ipc_prim->u.start_cnf, chan_nr);
		break;
	case IPC_IF_MSG_STOP_CNF:
		rc = ipc_rx_chan_stop_cnf(&ipc_prim->u.stop_cnf, chan_nr);
		break;
	case IPC_IF_MSG_SETGAIN_CNF:
		rc = ipc_rx_chan_setgain_cnf(&ipc_prim->u.set_gain_cnf, chan_nr);
		break;
	case IPC_IF_MSG_SETFREQ_CNF:
		rc = ipc_rx_chan_setfreq_cnf(&ipc_prim->u.set_freq_cnf, chan_nr);
		break;
	case IPC_IF_NOTIFY_UNDERFLOW:
		rc = ipc_rx_chan_notify_underflow(&ipc_prim->u.notify, chan_nr);
		break;
	case IPC_IF_NOTIFY_OVERFLOW:
		rc = ipc_rx_chan_notify_overflow(&ipc_prim->u.notify, chan_nr);
		break;
	case IPC_IF_MSG_SETTXATTN_CNF:
		rc = ipc_rx_chan_settxattn_cnf(&ipc_prim->u.txatten_cnf, chan_nr);
		break;
	default:
		LOGP(DMAIN, LOGL_ERROR, "Received unknown IPC msg type %d\n", msg_type);
		rc = -EINVAL;
	}

	return rc;
}

static int ipc_sock_send(struct ipc_sock_state *state, struct msgb *msg)
{
	struct osmo_fd *conn_bfd;
	//struct ipc_sk_if *ipc_prim = (struct ipc_sk_if *) msg->data;

	if (!state) {
		LOGP(DMAIN, LOGL_INFO,
		     "IPC socket not created, "
		     "dropping message\n");
		msgb_free(msg);
		return -EINVAL;
	}
	conn_bfd = &state->conn_bfd;
	if (conn_bfd->fd <= 0) {
		LOGP(DMAIN, LOGL_NOTICE,
		     "IPC socket not connected, "
		     "dropping message\n");
		msgb_free(msg);
		return -EIO;
	}
	msgb_enqueue(&state->upqueue, msg);
	conn_bfd->when |= BSC_FD_WRITE;

	return 0;
}

void IPCDevice::ipc_sock_close(struct ipc_sock_state *state)
{
	if (state == 0)
		return;

	struct osmo_fd *bfd = &state->conn_bfd;

	if (bfd->fd <= 0)
		return;

	LOGP(DDEV, LOGL_NOTICE, "IPC socket has LOST connection\n");

	close(bfd->fd);
	bfd->fd = -1;
	osmo_fd_unregister(bfd);

	/* flush the queue */
	while (!llist_empty(&state->upqueue)) {
		struct msgb *msg = msgb_dequeue(&state->upqueue);
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

	ipc_prim = (struct ipc_sk_if *)msg->tail;

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
		LOGP(DDEV, LOGL_ERROR,
		     "Received %d bytes on Unix Socket, but primitive size "
		     "is %zu, discarding\n",
		     rc, sizeof(*ipc_prim));
		msgb_free(msg);
		return 0;
	}

	rc = ipc_rx(ipc_prim->msg_type, ipc_prim);

	/* as we always synchronously process the message in IPC_rx() and
	 * its callbacks, we can free the message here. */
	msgb_free(msg);

	return rc;

close:
	msgb_free(msg);
	ipc_sock_close(&master_sk_state);
	return -1;
}

int IPCDevice::ipc_chan_sock_read(struct osmo_fd *bfd)
{
	struct ipc_sk_chan_if *ipc_prim;
	struct msgb *msg;
	int rc;

	msg = msgb_alloc(sizeof(*ipc_prim) + 1000, "ipc_chan_sock_rx");
	if (!msg)
		return -ENOMEM;

	ipc_prim = (struct ipc_sk_chan_if *)msg->tail;

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
		LOGP(DDEV, LOGL_ERROR,
		     "Received %d bytes on Unix Socket, but primitive size "
		     "is %zu, discarding\n",
		     rc, sizeof(*ipc_prim));
		msgb_free(msg);
		return 0;
	}

	rc = ipc_chan_rx(ipc_prim->msg_type, ipc_prim, bfd->priv_nr);

	/* as we always synchronously process the message in IPC_rx() and
	 * its callbacks, we can free the message here. */
	msgb_free(msg);

	return rc;

close:
	msgb_free(msg);
	ipc_sock_close(&sk_chan_state[bfd->priv_nr]);
	return -1;
}

int IPCDevice::ipc_sock_write(struct osmo_fd *bfd)
{
	int rc;

	while (!llist_empty(&master_sk_state.upqueue)) {
		struct msgb *msg, *msg2;
		struct ipc_sk_if *ipc_prim;

		/* peek at the beginning of the queue */
		msg = llist_entry(master_sk_state.upqueue.next, struct msgb, list);
		ipc_prim = (struct ipc_sk_if *)msg->data;

		bfd->when &= ~BSC_FD_WRITE;

		/* bug hunter 8-): maybe someone forgot msgb_put(...) ? */
		if (!msgb_length(msg)) {
			LOGP(DDEV, LOGL_ERROR,
			     "message type (%d) with ZERO "
			     "bytes!\n",
			     ipc_prim->msg_type);
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
		msg2 = msgb_dequeue(&master_sk_state.upqueue);
		assert(msg == msg2);
		msgb_free(msg);
	}
	return 0;

close:
	ipc_sock_close(&master_sk_state);
	return -1;
}

int IPCDevice::ipc_chan_sock_write(struct osmo_fd *bfd)
{
	int rc;

	while (!llist_empty(&sk_chan_state[bfd->priv_nr].upqueue)) {
		struct msgb *msg, *msg2;
		struct ipc_sk_chan_if *ipc_prim;

		/* peek at the beginning of the queue */
		msg = llist_entry(sk_chan_state[bfd->priv_nr].upqueue.next, struct msgb, list);
		ipc_prim = (struct ipc_sk_chan_if *)msg->data;
		bfd->when &= ~BSC_FD_WRITE;
		/* bug hunter 8-): maybe someone forgot msgb_put(...) ? */
		if (!msgb_length(msg)) {
			LOGP(DDEV, LOGL_ERROR,
			     "message type (%d) with ZERO "
			     "bytes!\n",
			     ipc_prim->msg_type);
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
		msg2 = msgb_dequeue(&sk_chan_state[bfd->priv_nr].upqueue);
		assert(msg == msg2);
		msgb_free(msg);
	}
	return 0;

close:
	ipc_sock_close(&sk_chan_state[bfd->priv_nr]);
	return -1;
}

static int ipc_sock_cb(struct osmo_fd *bfd, unsigned int flags)
{
	IPCDevice *device = static_cast<IPCDevice *>(bfd->data);
	int rc = 0;

	if (flags & BSC_FD_READ)
		rc = device->ipc_sock_read(bfd);
	if (rc < 0)
		return rc;

	if (flags & BSC_FD_WRITE)
		rc = device->ipc_sock_write(bfd);

	return rc;
}

static int ipc_chan_sock_cb(struct osmo_fd *bfd, unsigned int flags)
{
	IPCDevice *device = static_cast<IPCDevice *>(bfd->data);
	int rc = 0;

	if (flags & BSC_FD_READ)
		rc = device->ipc_chan_sock_read(bfd);
	if (rc < 0)
		return rc;

	if (flags & BSC_FD_WRITE)
		rc = device->ipc_chan_sock_write(bfd);

	return rc;
}

int IPCDevice::open(const std::string &args, int ref, bool swap_channels)
{
	std::string k,v;
	std::string::size_type keyend;
	int rc;

	if ((keyend = args.find('=')) != std::string::npos) {
		k = args.substr(0, keyend++);
		v = args.substr(keyend);
	}
	if(k != "ipc_msock" || !v.length()) {
			LOGC(DDEV, ERROR) << "Invalid device args provided, expected  \"dev-args ipc_msock=/path/to/socket\"\n";
			return -1;
	}

	LOGC(DDEV, INFO) << "Opening IPC device" << v << "..";

	memset(&master_sk_state, 0x00, sizeof(master_sk_state));
	INIT_LLIST_HEAD(&master_sk_state.upqueue);
	rc = osmo_sock_unix_init_ofd(&master_sk_state.conn_bfd, SOCK_SEQPACKET, 0, v.c_str(), OSMO_SOCK_F_CONNECT);
	if (rc < 0) {
		LOGC(DDEV, ERROR) << "Failed to connect to the BTS (" << v << "). "
				  << "Retrying...\n";
		osmo_timer_setup(&master_sk_state.timer, ipc_sock_timeout, NULL);
		osmo_timer_schedule(&master_sk_state.timer, 5, 0);
		return -1;
	}
	master_sk_state.conn_bfd.cb = ipc_sock_cb;
	master_sk_state.conn_bfd.data = this;

	ipc_tx_greeting_req(&master_sk_state, IPC_SOCK_API_VERSION);
	/* Wait until confirmation is recieved */
	while (tmp_state != IPC_IF_MSG_GREETING_CNF)
		osmo_select_main(0);

	ipc_tx_info_req(&master_sk_state);
	/* Wait until confirmation is recieved */
	while (tmp_state != IPC_IF_MSG_INFO_CNF)
		osmo_select_main(0);

	ipc_tx_open_req(&master_sk_state, chans, ref);
	/* Wait until confirmation is recieved */
	while (tmp_state != IPC_IF_MSG_OPEN_CNF)
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

void IPCDevice::manually_poll_sock_fds() {
	struct timeval wait = {0, 100000};
	fd_set crfds, cwfds;
	int max_fd = 0;

	FD_ZERO(&crfds);
	FD_ZERO(&cwfds);
	for(int i = 0; i < chans; i++) {
		struct osmo_fd* curr_fd = &sk_chan_state[i].conn_bfd;
		max_fd = curr_fd->fd > max_fd ? curr_fd->fd : max_fd;

		if(curr_fd->when & OSMO_FD_READ)
		FD_SET(curr_fd->fd, &crfds);
		if(curr_fd->when & OSMO_FD_WRITE)
		FD_SET(curr_fd->fd, &cwfds);
	}

	select(max_fd+1, &crfds, &cwfds, 0, &wait);

	for(int i = 0; i < chans; i++) {
		int flags = 0;
		struct osmo_fd* ofd = &sk_chan_state[i].conn_bfd;

		if (FD_ISSET(ofd->fd, &crfds)) {
			flags |= OSMO_FD_READ;
			FD_CLR(ofd->fd, &crfds);
		}

		if (FD_ISSET(ofd->fd, &cwfds)) {
			flags |= OSMO_FD_WRITE;
			FD_CLR(ofd->fd, &cwfds);
		}
		if(flags)
		ipc_chan_sock_cb(ofd, flags);
	}
}
/* the call stack is rather difficult here, we're already in select:
>~"#0  IPCDevice::start (this=<optimized out>) at IPCDevice.cpp:789\n"
>~"#1  in RadioInterface::start (this=0x614000001640) at radioInterface.cpp:187\n"
>~"#2  in Transceiver::start (this=<optimized out>) at Transceiver.cpp:293\n"
>~"#3  in Transceiver::ctrl_sock_handle_rx (this=0x61600000b180, chan=0) at Transceiver.cpp:838\n"
>~"#4  in Transceiver::ctrl_sock_cb (bfd=<optimized out>, flags=1) at Transceiver.cpp:168\n"
>~"#5  in osmo_fd_disp_fds (_rset=<optimized out>, _wset=<optimized out>, _eset=<optimized out>) at select.c:227\n"
>~"#6  _osmo_select_main (polling=<optimized out>) at select.c:265\n"
>~"#7  in osmo_select_main (polling=128) at select.c:274\n"
>~"#8  in main (argc=<optimized out>, argv=<optimized out>) at osmo-trx.cpp:649\n"
 * */
bool IPCDevice::start()
{
	LOGC(DDEV, INFO) << "starting IPC...";

	if (started) {
		LOGC(DDEV, ERR) << "Device already started";
		return true;
	}

	struct msgb *msg;
	struct ipc_sk_chan_if *ipc_prim;
	struct timeval timer_now, timeout;

	for(int i = 0; i < chans; i++) {
		msg = ipc_msgb_alloc(IPC_IF_MSG_START_REQ);
		if (!msg)
			return -ENOMEM;
		ipc_prim = (struct ipc_sk_chan_if *)msg->data;
		ipc_prim->u.start_req.dummy = 0;

		ipc_sock_send(&sk_chan_state[i], msg);
	}

	gettimeofday(&timeout, 0);
	timeout.tv_sec += 2;

	int chan_started_count = 0;
	while (chan_started_count != chans) {
		chan_started_count = 0;

		/* just poll here, we're already in select, so there is no other way to drive
		 * the fds and "wait" for a response or retry */
		manually_poll_sock_fds();

		for(unsigned int i = 0; i < ARRAY_SIZE(trx_is_started); i++)
			if(trx_is_started[i] == true)
				chan_started_count++;

		gettimeofday(&timer_now, 0);
		if(timercmp(&timer_now, &timeout, >))
			return false;
	}


	int max_bufs_to_flush = 0;
	for(unsigned int i = 0; i < shm_dec->num_chans; i++) {
		int buf_per_chan = shm_dec->channels[i]->ul_stream->num_buffers;
		max_bufs_to_flush = max_bufs_to_flush < buf_per_chan ? buf_per_chan : max_bufs_to_flush;
	}
	flush_recv(max_bufs_to_flush);

	started = true;
	return true;
}

bool IPCDevice::stop()
{
	struct msgb *msg;
	struct ipc_sk_chan_if *ipc_prim;
	struct timeval timer_now, timeout;

	if (!started)
		return true;

	for(int i = 0; i < chans; i++) {
		if(trx_is_started[i] == true) {
			msg = ipc_msgb_alloc(IPC_IF_MSG_STOP_REQ);
			if (!msg)
				return -ENOMEM;
			ipc_prim = (struct ipc_sk_chan_if *)msg->data;
			ipc_prim->u.start_req.dummy = 0;

			ipc_sock_send(&sk_chan_state[i], msg);
		}
	}

	gettimeofday(&timeout, 0);
	timeout.tv_sec += 2;

	int chan_started_count = 0;
	do {
		chan_started_count = 0;

		/* just poll here, we're already in select, so there is no other way to drive
		 * the fds and "wait" for a response or retry */
		manually_poll_sock_fds();

		for(unsigned int i = 0; i < ARRAY_SIZE(trx_is_started); i++)
			if(trx_is_started[i] == true)
				chan_started_count++;

		gettimeofday(&timer_now, 0);
		if(timercmp(&timer_now, &timeout, >)) {
			LOGC(DDEV, ERR) << "No response to stop  msg received, terminating anyway...";
			break;
		}
	} while (chan_started_count > 0);

	LOGC(DDEV, NOTICE) << "All chanels stopped, terminating...";

	/* reset internal buffer timestamps */
	for (size_t i = 0; i < rx_buffers.size(); i++)
		rx_buffers[i]->reset();

	started = false;
	return true;
}

double IPCDevice::maxRxGain()
{
	return current_info_cnf.chan_info[0].max_rx_gain;
}

double IPCDevice::minRxGain()
{
	return current_info_cnf.chan_info[0].min_rx_gain;
}


int IPCDevice::getNominalTxPower(size_t chan)
{
	return current_info_cnf.chan_info[chan].nominal_tx_power;
}

double IPCDevice::setPowerAttenuation(int atten, size_t chan) {
	struct msgb *msg;
	struct ipc_sk_chan_if *ipc_prim;

	if(chan >= chans)
		return 0;

	LOGCHAN(chan, DDEV, NOTICE) << "Setting TX attenuation to " << atten << " dB" << " chan " << chan;

	msg = ipc_msgb_alloc(IPC_IF_MSG_SETTXATTN_REQ);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_chan_if *)msg->data;
	ipc_prim->u.txatten_req.attenuation = atten;

	ipc_sock_send(&sk_chan_state[chan], msg);

	tx_attenuation[chan] = atten;
	return atten;
}

double IPCDevice::getPowerAttenuation(size_t chan) {

	if(chan >= chans)
		return 0;

	return tx_attenuation[chan];
}

double IPCDevice::setRxGain(double dB, size_t chan)
{
	struct msgb *msg;
	struct ipc_sk_chan_if *ipc_prim;

	if (dB > maxRxGain())
		dB = maxRxGain();
	if (dB < minRxGain())
		dB = minRxGain();

	LOGCHAN(chan, DDEV, NOTICE) << "Setting RX gain to " << dB << " dB";

	msg = ipc_msgb_alloc(IPC_IF_MSG_SETGAIN_REQ);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_chan_if *)msg->data;
	ipc_prim->u.set_gain_req.is_tx = 0;
	ipc_prim->u.set_gain_req.gain = dB;

	ipc_sock_send(&sk_chan_state[chan], msg);

	rx_gains[chan] = dB;
	return rx_gains[chan];
}

bool IPCDevice::flush_recv(size_t num_pkts)
{
	std::vector<uint16_t> tmp(4096);
	uint64_t tmps;
	uint32_t read;

	for (uint32_t j = 0; j < num_pkts; j++) {
		for (unsigned int i = 0; i < chans; i++)
			read = ipc_shm_read(shm_io_rx_streams[i], (uint16_t *)&tmp.front(), 4096 / 2, &tmps, 3);
	}
	ts_initial = tmps + read;

	LOGC(DDEV, INFO) << "Initial timestamp " << ts_initial << std::endl;
	return true;
}

bool IPCDevice::setRxAntenna(const std::string &ant, size_t chan)
{
	return true;
}

std::string IPCDevice::getRxAntenna(size_t chan)
{
	return "";
}

bool IPCDevice::setTxAntenna(const std::string &ant, size_t chan)
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

GSM::Time IPCDevice::minLatency()
{
	/* UNUSED */
	return GSM::Time(0, 0);
}

/** Returns the starting write Timestamp*/
TIMESTAMP IPCDevice::initialWriteTimestamp(void)
{
	return ts_initial;
}

/** Returns the starting read Timestamp*/
TIMESTAMP IPCDevice::initialReadTimestamp(void)
{
	return ts_initial;
}

// NOTE: Assumes sequential reads
int IPCDevice::readSamples(std::vector<short *> &bufs, int len, bool *overrun, TIMESTAMP timestamp, bool *underrun)
{
	int rc, num_smpls, expect_smpls;
	ssize_t avail_smpls;
	TIMESTAMP expect_timestamp;
	unsigned int i;

	if (bufs.size() != chans) {
		LOGC(DDEV, ERROR) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	*overrun = false;
	*underrun = false;

	timestamp += current_open_cnf.path_delay;

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

			thread_enable_cancel(false);

			num_smpls = ipc_shm_read(shm_io_rx_streams[i], (uint16_t *)bufs[i], len - avail_smpls,
						 &recv_timestamp, 1);
			expect_timestamp = timestamp + avail_smpls;
			thread_enable_cancel(true);

			if(num_smpls == -ETIMEDOUT)
				continue;

			LOGCHAN(i, DDEV, DEBUG)
			"Received timestamp = " << (TIMESTAMP)recv_timestamp << " (" << num_smpls << ")";

			expect_smpls = len - avail_smpls;
			//			if (expect_smpls != num_smpls)
			//				LOGCHAN(i, DDEV, NOTICE)
			//					<< "Unexpected recv buffer len: expect " << expect_smpls << " got " << num_smpls
			//					<< ", diff=" << expect_smpls - num_smpls;

			//expect_timestamp = timestamp + avail_smpls;
			if (expect_timestamp != (TIMESTAMP)recv_timestamp)
				LOGCHAN(i, DDEV, ERROR) << "Unexpected recv buffer timestamp: expect "
							<< expect_timestamp << " got " << recv_timestamp << ", diff="
							<< ((uint64_t)recv_timestamp > expect_timestamp ?
								    (uint64_t)recv_timestamp - expect_timestamp :
								    expect_timestamp - recv_timestamp);

			rc = rx_buffers[i]->write(bufs[i], num_smpls, (TIMESTAMP)recv_timestamp);
			if (rc < 0) {
				LOGCHAN(i, DDEV, ERROR) << rx_buffers[i]->str_code(rc) << " num smpls: " << num_smpls << " chan: " << i;
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
}

int IPCDevice::writeSamples(std::vector<short *> &bufs, int len, bool *underrun, unsigned long long timestamp)
{
	int rc = 0;
	unsigned int i;

	if (bufs.size() != chans) {
		LOGC(DDEV, ERROR) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	*underrun = false;

	for (i = 0; i < chans; i++) {
		LOGCHAN(i, DDEV, DEBUG) << "send buffer of len " << len << " timestamp " << std::hex << timestamp;
		//		thread_enable_cancel(false);
		rc = ipc_shm_enqueue(shm_io_tx_streams[i], timestamp, len, (uint16_t *)bufs[i]);

		//		rc = LMS_SendStream(&m_lms_stream_tx[i], bufs[i], len, &tx_metadata, 100);
		//		update_stream_stats_tx(i, underrun);
		//		thread_enable_cancel(true);
		if (rc != len) {
			LOGCHAN(i, DDEV, ERROR) << "LMS: Device Tx timed out (" << rc << " vs exp " << len << ").";
			return -1;
		}
	}

	return rc;
}

bool IPCDevice::updateAlignment(TIMESTAMP timestamp)
{
	return true;
}

bool IPCDevice::setTxFreq(double wFreq, size_t chan)
{
	struct msgb *msg;
	struct ipc_sk_chan_if *ipc_prim;
	LOGCHAN(chan, DDEV, NOTICE) << "Setting Tx Freq to " << wFreq << " Hz";

	msg = ipc_msgb_alloc(IPC_IF_MSG_SETFREQ_REQ);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_chan_if *)msg->data;
	ipc_prim->u.set_freq_req.is_tx = 1;
	ipc_prim->u.set_freq_req.freq = wFreq;

	return ipc_sock_send(&sk_chan_state[chan], msg) < 0 ? false : true;
}

bool IPCDevice::setRxFreq(double wFreq, size_t chan)
{
	struct msgb *msg;
	struct ipc_sk_chan_if *ipc_prim;
	LOGCHAN(chan, DDEV, NOTICE) << "Setting Rx Freq to " << wFreq << " Hz";

	msg = ipc_msgb_alloc(IPC_IF_MSG_SETFREQ_REQ);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_chan_if *)msg->data;
	ipc_prim->u.set_freq_req.is_tx = 0;
	ipc_prim->u.set_freq_req.freq = wFreq;

	return ipc_sock_send(&sk_chan_state[chan], msg) < 0 ? false : true;
}

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
	return new IPCDevice(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths);
}
