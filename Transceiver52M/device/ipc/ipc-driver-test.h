#pragma once

#include <osmocom/core/select.h>
#include "shm.h"

extern struct ipc_sock_state *global_ipc_sock_state;

/* 8 channels are plenty */
extern struct ipc_sock_state *global_ctrl_socks[8];
extern struct ipc_shm_io *ios_tx_to_device[8];
extern struct ipc_shm_io *ios_rx_from_device[8];

struct ipc_sock_state {
	struct osmo_fd listen_bfd; /* fd for listen socket */
	struct osmo_fd conn_bfd; /* fd for connection to lcr */
	struct llist_head upqueue; /* queue for sending messages */
};

int ipc_sock_init(const char *path, struct ipc_sock_state **global_state_var,
		  int (*sock_callback_fn)(struct osmo_fd *fd, unsigned int what), int n);

int ipc_rx_greeting_req(struct ipc_sk_if_greeting *greeting_req);
int ipc_rx_info_req(struct ipc_sk_if_info_req *info_req);
int ipc_rx_open_req(struct ipc_sk_if_open_req *open_req);

int ipc_rx_chan_start_req(struct ipc_sk_chan_if_op_void *req, uint8_t chan_nr);
int ipc_rx_chan_stop_req(struct ipc_sk_chan_if_op_void *req, uint8_t chan_nr);
int ipc_rx_chan_setgain_req(struct ipc_sk_chan_if_gain *req, uint8_t chan_nr);
int ipc_rx_chan_setfreq_req(struct ipc_sk_chan_if_freq_req *req, uint8_t chan_nr);
