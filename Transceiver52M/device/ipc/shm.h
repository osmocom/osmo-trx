/*
* Copyright 2020 sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
* Author: Pau Espin Pedrol <pespin@sysmocom.de>
*
* SPDX-License-Identifier: AGPL-3.0+
*
* This software is distributed under multiple licenses; see the COPYING file in
* the main directory for licensing information for this specific distribution.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/

#include <stdint.h>
#include <unistd.h>
#include <limits.h>

/* RAW structures */
struct ipc_shm_raw_smpl_buf {
        uint32_t timestamp;
        uint32_t data_len; /* In samples */
        uint16_t samples[0];
};

struct ipc_shm_raw_stream {
        uint32_t num_buffers;
        uint32_t buffer_size; /* In samples */
        uint32_t read_next;
        uint32_t write_next;
        uint32_t buffer_offset[0];
        //struct ipc_shm_smpl_buf buffers[0];
};

struct ipc_shm_raw_channel {
        uint32_t dl_buf_offset;
        uint32_t ul_buf_offset;
};

struct ipc_shm_raw_region {
        uint32_t num_chans;
        uint32_t chan_offset[0];
};


/* non-raw, Pointer converted structures */
struct ipc_shm_stream {
        uint32_t num_buffers;
        uint32_t buffer_size;
        struct ipc_shm_raw_stream *raw;
        struct ipc_shm_raw_smpl_buf *buffers[0];
};

struct ipc_shm_channel {
        struct ipc_shm_stream *dl_stream;
        struct ipc_shm_stream *ul_stream;
};

/* Pointer converted structures */
struct ipc_shm_region {
        uint32_t num_chans;
        struct ipc_shm_channel *channels[0];
};

unsigned int ipc_shm_encode_region(struct ipc_shm_raw_region *root_raw, uint32_t num_chans, uint32_t num_buffers, uint32_t buffer_size);
struct ipc_shm_region *ipc_shm_decode_region(void *tall_ctx, struct ipc_shm_raw_region *root_raw);
/****************************************/
/* UNIX SOCKET API                      */
/****************************************/

//////////////////
// Master socket
//////////////////

#define IPC_SOCK_PATH "/tmp/ipc_sock"
#define IPC_SOCK_API_VERSION 1

/* msg_type */
#define IPC_IF_MSG_GREETING_REQ	0x00
#define IPC_IF_MSG_GREETING_CNF	0x01
#define IPC_IF_MSG_INFO_REQ	0x02
#define IPC_IF_MSG_INFO_CNF	0x03
#define IPC_IF_MSG_OPEN_REQ	0x04
#define IPC_IF_MSG_OPEN_CNF	0x05

#define MAX_NUM_CHANS 30
#define RF_PATH_NAME_SIZE 25
#define MAX_NUM_RF_PATHS 10
#define SHM_NAME_MAX NAME_MAX /* 255 */

#define FEATURE_MASK_CLOCKREF_INTERNAL (0x1 << 0)
#define FEATURE_MASK_CLOCKREF_EXTERNAL (0x1 << 1)
struct ipc_sk_if_info_chan {
        char tx_path[MAX_NUM_RF_PATHS][RF_PATH_NAME_SIZE];
        char rx_path[MAX_NUM_RF_PATHS][RF_PATH_NAME_SIZE];
} __attribute__ ((packed));

struct ipc_sk_if_open_req_chan {
        char tx_path[RF_PATH_NAME_SIZE];
        char rx_path[RF_PATH_NAME_SIZE];
} __attribute__ ((packed));

struct ipc_sk_if_open_cnf_chan {
        char chan_ipc_sk_path[108];
} __attribute__ ((packed));

struct ipc_sk_if_greeting {
        uint8_t         req_version;
} __attribute__ ((packed));

struct ipc_sk_if_info_req {
        uint8_t         spare;
} __attribute__ ((packed));

struct ipc_sk_if_info_cnf {
        uint32_t        feature_mask;
        double          min_rx_gain;
        double          max_rx_gain;
        double          min_tx_gain;
        double          max_tx_gain;
        uint32_t        max_num_chans;
        char            dev_desc[200];
        struct ipc_sk_if_info_chan chan_info[0];
} __attribute__ ((packed));

struct ipc_sk_if_open_req {
        uint32_t        num_chans;
        uint32_t        clockref; /* One of FEATUER_MASK_CLOCKREF_* */
        uint32_t        rx_sps;
        uint32_t        tx_sps;
        uint32_t        bandwidth;
        struct ipc_sk_if_open_req_chan chan_info[0];
} __attribute__ ((packed));

struct ipc_sk_if_open_cnf {
        uint8_t return_code;
        char shm_name[SHM_NAME_MAX];
        struct ipc_sk_if_open_cnf_chan chan_info[0];
} __attribute__ ((packed));

struct ipc_sk_if {
	uint8_t		msg_type;	/* message type */
	uint8_t		spare[2];

	union {
		struct ipc_sk_if_greeting       greeting_req;
		struct ipc_sk_if_greeting       greeting_cnf;
		struct ipc_sk_if_info_req       info_req;
		struct ipc_sk_if_info_cnf       info_cnf;
		struct ipc_sk_if_open_req       open_req;
		struct ipc_sk_if_open_cnf       open_cnf;
	} u;
} __attribute__ ((packed));


//////////////////
// Channel socket
//////////////////
#define IPC_IF_MSG_START_REQ	0x00
#define IPC_IF_MSG_START_CNF	0x01
#define IPC_IF_MSG_STOP_REQ	0x02
#define IPC_IF_MSG_STOP_CNF	0x03
#define IPC_IF_MSG_SETGAIN_REQ	0x04
#define IPC_IF_MSG_SETGAIN_CNF	0x05
#define IPC_IF_MSG_SETFREQ_REQ	0x04
#define IPC_IF_MSG_SETFREQ_CNF	0x05

struct ipc_sk_chan_if_op_void {
} __attribute__ ((packed));

struct ipc_sk_chan_if_op_rc {
        uint8_t return_code;
} __attribute__ ((packed));

struct ipc_sk_chan_if_gain {
        double gain;
        uint8_t is_tx;
} __attribute__ ((packed));

struct ipc_sk_chan_if_freq_req {
        double freq;
        uint8_t is_tx;
} __attribute__ ((packed));

struct ipc_sk_chan_if_freq_cnf {
        uint8_t return_code;
} __attribute__ ((packed));

struct ipc_sk_chan_if {
	uint8_t		msg_type;	/* message type */
	uint8_t		spare[2];

	union {
		struct ipc_sk_chan_if_op_void        start_req;
		struct ipc_sk_chan_if_op_rc          start_cnf;
		struct ipc_sk_chan_if_op_void        stop_req;
		struct ipc_sk_chan_if_op_rc          stop_cnf;
		struct ipc_sk_chan_if_gain           set_gain_req;
		struct ipc_sk_chan_if_gain           set_gain_cnf;
		struct ipc_sk_chan_if_freq_req       set_freq_req;
		struct ipc_sk_chan_if_freq_cnf       set_freq_cnf;
	} u;
} __attribute__ ((packed));
