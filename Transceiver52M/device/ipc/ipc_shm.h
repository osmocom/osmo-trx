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
#ifndef IPC_SHM_H
#define IPC_SHM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <shm.h>
#include <stdint.h>

#ifdef __cplusplus
}
#endif

struct ipc_shm_io {
	volatile struct ipc_shm_raw_stream *this_stream; // plus num_buffers at end
	volatile struct ipc_shm_raw_smpl_buf **volatile buf_ptrs;
	uint32_t partial_read_begin_ptr;
};

struct ipc_shm_io *ipc_shm_init_consumer(struct ipc_shm_stream *s);
struct ipc_shm_io *ipc_shm_init_producer(struct ipc_shm_stream *s);
void ipc_shm_close(struct ipc_shm_io *r);
int32_t ipc_shm_enqueue(struct ipc_shm_io *r, uint64_t timestamp, uint32_t len_in_sps, uint16_t *data);
int32_t ipc_shm_tryenqueue(struct ipc_shm_io *r, uint64_t timestamp, uint32_t len_in_sps, uint16_t *data);
volatile struct ipc_shm_raw_smpl_buf *ipc_shm_dequeue(struct ipc_shm_io *r);
int32_t ipc_shm_read(struct ipc_shm_io *r, uint16_t *out_buf, uint32_t num_samples, uint64_t *timestamp,
		     uint32_t timeout_seconds);

#endif // IPC_SHM_H
