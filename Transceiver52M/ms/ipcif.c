/*
* Copyright 2020 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
* Author: Pau Espin Pedrol <pespin@sysmocom.de>
*
* SPDX-License-Identifier: 0BSD
*
* Permission to use, copy, modify, and/or distribute this software for any purpose
* with or without fee is hereby granted.THE SOFTWARE IS PROVIDED "AS IS" AND THE
* AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
* BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
* WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF
* CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
* USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#define _GNU_SOURCE
#include <pthread.h>

#include <debug.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <inttypes.h>
#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <fcntl.h> /* For O_* constants */

#include <osmocom/core/application.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/select.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>

#include <shm.h>
#include <ipc_shm.h>
#include <ipc_chan.h>
#include <ipc_sock.h>

#define DEFAULT_SHM_NAME "/osmo-trx-ipc-driver-shm2"
#define IPC_SOCK_PATH_PREFIX "/tmp"

static void *tall_ctx;
struct ipc_sock_state *global_ipc_sock_state;

/* 8 channels are plenty */
struct ipc_sock_state *global_ctrl_socks[8];
struct ipc_shm_io *ios_tx_to_device[8];
struct ipc_shm_io *ios_rx_from_device[8];

void *shm;
// void *global_dev;

static pthread_mutex_t wait_open_lock;
static pthread_cond_t wait_open_cond;


static struct ipc_shm_region *decoded_region;

static struct {
	int msocknum;
	char *ud_prefix_dir;
} cmdline_cfg = { 1, IPC_SOCK_PATH_PREFIX };

static const struct log_info_cat default_categories[] = {
	[DMAIN] = {
		.name = "DMAIN",
		.color = NULL,
		.description = "Main generic category",
		.loglevel = LOGL_DEBUG,.enabled = 1,
	},
	[DDEV] = {
		.name = "DDEV",
		.description = "Device/Driver specific code",
		.color = NULL,
		.enabled = 1, .loglevel = LOGL_DEBUG,
	},
};

const struct log_info log_infox = {
	.cat = default_categories,
	.num_cat = ARRAY_SIZE(default_categories),
};

volatile int ipc_exit_requested = 0;

static int ipc_shm_setup(const char *shm_name, size_t shm_len)
{
	int fd;
	int rc;

	LOGP(DMAIN, LOGL_NOTICE, "Opening shm path %s\n", shm_name);
	if ((fd = shm_open(shm_name, O_CREAT | O_RDWR | O_TRUNC, S_IRUSR | S_IWUSR)) < 0) {
		LOGP(DMAIN, LOGL_ERROR, "shm_open %d: %s\n", errno, strerror(errno));
		rc = -errno;
		goto err_shm_open;
	}

	LOGP(DMAIN, LOGL_NOTICE, "Truncating %d to size %zu\n", fd, shm_len);
	if (ftruncate(fd, shm_len) < 0) {
		LOGP(DMAIN, LOGL_ERROR, "ftruncate %d: %s\n", errno, strerror(errno));
		rc = -errno;
		goto err_mmap;
	}

	LOGP(DMAIN, LOGL_NOTICE, "mmaping shared memory fd %d\n", fd);
	if ((shm = mmap(NULL, shm_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) == MAP_FAILED) {
		LOGP(DMAIN, LOGL_ERROR, "mmap %d: %s\n", errno, strerror(errno));
		rc = -errno;
		goto err_mmap;
	}

	LOGP(DMAIN, LOGL_NOTICE, "mmap'ed shared memory at addr %p\n", shm);
	/* After a call to mmap(2) the file descriptor may be closed without affecting the memory mapping. */
	close(fd);
	return 0;
err_mmap:
	shm_unlink(shm_name);
	close(fd);
err_shm_open:
	return rc;
}

struct msgb *ipc_msgb_alloc(uint8_t msg_type)
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

static int ipc_tx_info_cnf()
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;

	msg = ipc_msgb_alloc(IPC_IF_MSG_INFO_CNF);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_if *)msg->data;

	ipc_prim->u.info_cnf.feature_mask = FEATURE_MASK_CLOCKREF_EXTERNAL;
	ipc_prim->u.info_cnf.iq_scaling_val_rx = 1;
	ipc_prim->u.info_cnf.iq_scaling_val_tx = 1;
	ipc_prim->u.info_cnf.max_num_chans = 1;
	OSMO_STRLCPY_ARRAY(ipc_prim->u.info_cnf.dev_desc, "bernd");

	struct ipc_sk_if_info_chan *chan_info = ipc_prim->u.info_cnf.chan_info;
	OSMO_STRLCPY_ARRAY(chan_info->tx_path[0], "TX/RX");
	OSMO_STRLCPY_ARRAY(chan_info->rx_path[0], "RX2");
	chan_info->min_rx_gain = 0;
	chan_info->max_rx_gain = 60;
	chan_info->min_tx_gain = 0;
	chan_info->max_tx_gain = 60;
	chan_info->nominal_tx_power = 10;

	return ipc_sock_send(msg);
}

static int ipc_tx_open_cnf(int rc, uint32_t num_chans, int32_t timingoffset)
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;
	struct ipc_sk_if_open_cnf_chan *chan_info;
	unsigned int i;

	msg = ipc_msgb_alloc(IPC_IF_MSG_OPEN_CNF);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_if *)msg->data;
	ipc_prim->u.open_cnf.return_code = rc;
	ipc_prim->u.open_cnf.path_delay = timingoffset; // 6.18462e-5 * 1625e3 / 6;
	OSMO_STRLCPY_ARRAY(ipc_prim->u.open_cnf.shm_name, DEFAULT_SHM_NAME);

	chan_info = ipc_prim->u.open_cnf.chan_info;
	for (i = 0; i < num_chans; i++) {
		snprintf(chan_info->chan_ipc_sk_path, sizeof(chan_info->chan_ipc_sk_path), "%s/ipc_sock%d_%d",
			 cmdline_cfg.ud_prefix_dir, cmdline_cfg.msocknum, i);
		/* FIXME: dynamc chan limit, currently 8 */
		if (i < 8)
			ipc_sock_init(chan_info->chan_ipc_sk_path, &global_ctrl_socks[i], ipc_chan_sock_accept, i);
		chan_info++;
	}

	return ipc_sock_send(msg);
}

int ipc_rx_greeting_req(struct ipc_sk_if_greeting *greeting_req)
{
	struct msgb *msg;
	struct ipc_sk_if *ipc_prim;

	msg = ipc_msgb_alloc(IPC_IF_MSG_GREETING_CNF);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_if *)msg->data;
	ipc_prim->u.greeting_cnf.req_version =
		greeting_req->req_version == IPC_SOCK_API_VERSION ? IPC_SOCK_API_VERSION : 0;

	ipc_sock_send(msg);
	return 0;
}

int ipc_rx_info_req(struct ipc_sk_if_info_req *info_req)
{
	ipc_tx_info_cnf();
	return 0;
}

int ipc_rx_open_req(struct ipc_sk_if_open_req *open_req)
{
	/* calculate size needed */
	unsigned int len;
	unsigned int i;

	// global_dev = uhdwrap_open(open_req);

	/* b210 packet size is 2040, but our tx size is 2500, so just do *2 */
	int shmbuflen = 5000 * 2;

	len = ipc_shm_encode_region(NULL, open_req->num_chans, 4, shmbuflen);
	/* Here we verify num_chans, rx_path, tx_path, clockref, etc. */
	int rc = ipc_shm_setup(DEFAULT_SHM_NAME, len);
	len = ipc_shm_encode_region((struct ipc_shm_raw_region *)shm, open_req->num_chans, 4, shmbuflen);
	//	LOGP(DMAIN, LOGL_NOTICE, "%s\n", osmo_hexdump((const unsigned char *)shm, 80));

	/* set up our own copy of the decoded area, we have to do it here,
	* since the uhd wrapper does not allow starting single channels
	* additionally go for the producer init for both, so only we are responsible for the init, instead
	* of splitting it with the client and causing potential races if one side uses it too early */
	decoded_region = ipc_shm_decode_region(0, (struct ipc_shm_raw_region *)shm);
	for (i = 0; i < open_req->num_chans; i++) {
		//		ios_tx_to_device[i] = ipc_shm_init_consumer(decoded_region->channels[i]->dl_stream);
		ios_tx_to_device[i] = ipc_shm_init_producer(decoded_region->channels[i]->dl_stream);
		ios_rx_from_device[i] = ipc_shm_init_producer(decoded_region->channels[i]->ul_stream);
	}

	ipc_tx_open_cnf(-rc, open_req->num_chans, 0);
	return 0;
}

volatile bool ul_running = false;
volatile bool dl_running = false;

void *uplink_thread(void *x_void_ptr)
{
	uint32_t chann = decoded_region->num_chans;
	ul_running = true;
	pthread_setname_np(pthread_self(), "uplink_rx");

	while (!ipc_exit_requested) {
		// int32_t read = uhdwrap_read(global_dev, chann);
		// ipc_shm_enqueue(ios_rx_from_device[i], tstamp, num_rx_samps, (uint16_t *)&d->wrap_rx_buffs[i].front());
		if (read < 0)
			return 0;
	}
	return 0;
}

void *downlink_thread(void *x_void_ptr)
{
	int chann = decoded_region->num_chans;
	dl_running = true;
	pthread_setname_np(pthread_self(), "downlink_tx");

	while (!ipc_exit_requested) {
		bool underrun;
		// uhdwrap_write(global_dev, chann, &underrun);

		// len = ipc_shm_read(ios_tx_to_device[i], (uint16_t *)&d->wrap_tx_buffs[i].front(), 5000, &timestamp, 1);
		// return d->writeSamples(d->wrap_tx_buf_ptrs, len, underrun, timestamp);
	}
	return 0;
}

int ipc_rx_chan_start_req(struct ipc_sk_chan_if_op_void *req, uint8_t chan_nr)
{
	struct msgb *msg;
	struct ipc_sk_chan_if *ipc_prim;
	int rc = 0;

	// rc = uhdwrap_start(global_dev, chan_nr);

	pthread_cond_broadcast(&wait_open_cond);
	// /* no per-chan start/stop */
	// if (!dl_running || !ul_running) {
	// 	/* chan != first chan start will "fail", which is fine, usrp can't start/stop chans independently */
	// 	if (rc) {
	// 		LOGP(DMAIN, LOGL_INFO, "starting rx/tx threads.. req for chan:%d\n", chan_nr);
	// 		pthread_t rx, tx;
	// 		pthread_create(&rx, NULL, uplink_thread, 0);
	// 		pthread_create(&tx, NULL, downlink_thread, 0);
	// 	}
	// } else
	// 	LOGP(DMAIN, LOGL_INFO, "starting rx/tx threads request ignored.. req for chan:%d\n", chan_nr);

	msg = ipc_msgb_alloc(IPC_IF_MSG_START_CNF);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_chan_if *)msg->data;
	ipc_prim->u.start_cnf.return_code = rc ? 0 : -1;

	return ipc_chan_sock_send(msg, chan_nr);
}
int ipc_rx_chan_stop_req(struct ipc_sk_chan_if_op_void *req, uint8_t chan_nr)
{
	struct msgb *msg;
	struct ipc_sk_chan_if *ipc_prim;
	int rc = true;

	/* no per-chan start/stop */
	// rc = uhdwrap_stop(global_dev, chan_nr);

	msg = ipc_msgb_alloc(IPC_IF_MSG_STOP_CNF);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_chan_if *)msg->data;
	ipc_prim->u.stop_cnf.return_code = rc ? 0 : -1;

	return ipc_chan_sock_send(msg, chan_nr);
}
int ipc_rx_chan_setgain_req(struct ipc_sk_chan_if_gain *req, uint8_t chan_nr)
{
	struct msgb *msg;
	struct ipc_sk_chan_if *ipc_prim;
	double rv = req->gain;

	// rv = uhdwrap_set_gain(global_dev, req->gain, chan_nr, req->is_tx);

	msg = ipc_msgb_alloc(IPC_IF_MSG_SETGAIN_CNF);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_chan_if *)msg->data;
	ipc_prim->u.set_gain_cnf.is_tx = req->is_tx;
	ipc_prim->u.set_gain_cnf.gain = rv;

	return ipc_chan_sock_send(msg, chan_nr);
}

int ipc_rx_chan_setfreq_req(struct ipc_sk_chan_if_freq_req *req, uint8_t chan_nr)
{
	struct msgb *msg;
	struct ipc_sk_chan_if *ipc_prim;
	bool rv = true;

	// rv = uhdwrap_set_freq(global_dev, req->freq, chan_nr, req->is_tx);

	msg = ipc_msgb_alloc(IPC_IF_MSG_SETFREQ_CNF);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_chan_if *)msg->data;
	ipc_prim->u.set_freq_cnf.return_code = rv ? 0 : 1;

	return ipc_chan_sock_send(msg, chan_nr);
}

int ipc_rx_chan_settxatten_req(struct ipc_sk_chan_if_tx_attenuation *req, uint8_t chan_nr)
{
	struct msgb *msg;
	struct ipc_sk_chan_if *ipc_prim;
	double rv = req->attenuation;

	// rv = uhdwrap_set_txatt(global_dev, req->attenuation, chan_nr);

	msg = ipc_msgb_alloc(IPC_IF_MSG_SETTXATTN_CNF);
	if (!msg)
		return -ENOMEM;
	ipc_prim = (struct ipc_sk_chan_if *)msg->data;
	ipc_prim->u.txatten_cnf.attenuation = rv;

	return ipc_chan_sock_send(msg, chan_nr);
}

int ipc_sock_init(const char *path, struct ipc_sock_state **global_state_var,
		  int (*sock_callback_fn)(struct osmo_fd *fd, unsigned int what), int n)
{
	struct ipc_sock_state *state;
	struct osmo_fd *bfd;
	int rc;

	state = talloc_zero(NULL, struct ipc_sock_state);
	if (!state)
		return -ENOMEM;
	*global_state_var = state;

	INIT_LLIST_HEAD(&state->upqueue);
	state->conn_bfd.fd = -1;

	bfd = &state->listen_bfd;

	bfd->fd = osmo_sock_unix_init(SOCK_SEQPACKET, 0, path, OSMO_SOCK_F_BIND);
	if (bfd->fd < 0) {
		LOGP(DMAIN, LOGL_ERROR, "Could not create %s unix socket: %s\n", path, strerror(errno));
		talloc_free(state);
		return -1;
	}

	osmo_fd_setup(bfd, bfd->fd, OSMO_FD_READ, sock_callback_fn, state, n);

	rc = osmo_fd_register(bfd);
	if (rc < 0) {
		LOGP(DMAIN, LOGL_ERROR, "Could not register listen fd: %d\n", rc);
		close(bfd->fd);
		talloc_free(state);
		return rc;
	}

	LOGP(DMAIN, LOGL_INFO, "Started listening on IPC socket: %s\n", path);

	return 0;
}

int main_ipc()
{
	char *ipc_msock_path = "/tmp/ipc_sock1";
	tall_ctx = talloc_named_const(NULL, 0, "trx-ipc-ms");
	msgb_talloc_ctx_init(tall_ctx, 0);
	osmo_init_logging2(tall_ctx, &log_infox);
	log_enable_multithread();

	LOGP(DMAIN, LOGL_INFO, "Starting %s\n", "bernd");
	ipc_sock_init(ipc_msock_path, &global_ipc_sock_state, ipc_sock_accept, 0);
	// while (!ipc_exit_requested)
	// 	osmo_select_main(0);

	// if (global_dev) {
	// 	unsigned int i;
	// 	for (i = 0; i < decoded_region->num_chans; i++)
	// 		uhdwrap_stop(global_dev, i);
	// }

	// ipc_sock_close(global_ipc_sock_state);

	int rv;
	pthread_condattr_t t2;
	rv = pthread_condattr_setpshared(&t2, PTHREAD_PROCESS_SHARED);
	rv = pthread_cond_init(&wait_open_cond, &t2);


	return 0;
}

int wait_for_shm_open()
{
	struct timespec tv;
	int rv;
	clock_gettime(CLOCK_REALTIME, &tv);
	tv.tv_sec += 15;

	rv = pthread_mutex_timedlock(&wait_open_lock, &tv);
	if (rv != 0)
		return -rv;

	rv = pthread_cond_timedwait(&wait_open_cond, &wait_open_lock, &tv);
	return rv;
}