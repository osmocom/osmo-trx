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
#ifdef __cplusplus
extern "C" {
#endif

#include <shm.h>
#include "ipc_shm.h"
#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#ifdef __cplusplus
}
#endif

#define SAMPLE_SIZE_BYTE sizeof(uint16_t) * 2

struct ipc_shm_io *ipc_shm_init_consumer(struct ipc_shm_stream *s)
{
	unsigned int i;

	struct ipc_shm_io *r = (struct ipc_shm_io *)malloc(sizeof(struct ipc_shm_io));
	r->this_stream = s->raw;
	r->buf_ptrs =
		(volatile struct ipc_shm_raw_smpl_buf **)malloc(sizeof(struct ipc_shm_raw_smpl_buf *) * s->num_buffers);

	/* save actual ptrs */
	for (i = 0; i < s->num_buffers; i++)
		r->buf_ptrs[i] = s->buffers[i];

	r->partial_read_begin_ptr = 0;
	return r;
}

struct ipc_shm_io *ipc_shm_init_producer(struct ipc_shm_stream *s)
{
	int rv;
	pthread_mutexattr_t att;
	pthread_condattr_t t1, t2;
	struct ipc_shm_io *r = ipc_shm_init_consumer(s);
	rv = pthread_mutexattr_init(&att);
	if (rv != 0) {
		fprintf(stderr, "%s:%d rv:%d", __FILE__, __LINE__, rv);
		exit(EXIT_FAILURE);
	}

	rv = pthread_mutexattr_setrobust(&att, PTHREAD_MUTEX_ROBUST);
	if (rv != 0) {
		fprintf(stderr, "%s:%d rv:%d", __FILE__, __LINE__, rv);
		exit(EXIT_FAILURE);
	}

	rv = pthread_mutexattr_setpshared(&att, PTHREAD_PROCESS_SHARED);
	if (rv != 0) {
		fprintf(stderr, "%s:%d rv:%d", __FILE__, __LINE__, rv);
		exit(EXIT_FAILURE);
	}

	rv = pthread_mutex_init((pthread_mutex_t *)&r->this_stream->lock, &att);
	if (rv != 0) {
		fprintf(stderr, "%s:%d rv:%d", __FILE__, __LINE__, rv);
		exit(EXIT_FAILURE);
	}

	pthread_mutexattr_destroy(&att);

	rv = pthread_condattr_setpshared(&t1, PTHREAD_PROCESS_SHARED);
	if (rv != 0) {
		fprintf(stderr, "%s:%d rv:%d", __FILE__, __LINE__, rv);
		exit(EXIT_FAILURE);
	}

	rv = pthread_condattr_setpshared(&t2, PTHREAD_PROCESS_SHARED);
	if (rv != 0) {
		fprintf(stderr, "%s:%d rv:%d", __FILE__, __LINE__, rv);
		exit(EXIT_FAILURE);
	}

	rv = pthread_cond_init((pthread_cond_t *)&r->this_stream->cf, &t1);
	if (rv != 0) {
		fprintf(stderr, "%s:%d rv:%d", __FILE__, __LINE__, rv);
		exit(EXIT_FAILURE);
	}

	rv = pthread_cond_init((pthread_cond_t *)&r->this_stream->ce, &t2);
	if (rv != 0) {
		fprintf(stderr, "%s:%d rv:%d", __FILE__, __LINE__, rv);
		exit(EXIT_FAILURE);
	}

	pthread_condattr_destroy(&t1);
	pthread_condattr_destroy(&t2);

	r->this_stream->read_next = 0;
	r->this_stream->write_next = 0;
	return r;
}

void ipc_shm_close(struct ipc_shm_io *r)
{
	if (r) {
		if (r->buf_ptrs)
			free(r->buf_ptrs);
		free(r);
	}
}

int32_t ipc_shm_enqueue(struct ipc_shm_io *r, uint64_t timestamp, uint32_t len_in_sps, uint16_t *data)
{
	volatile struct ipc_shm_raw_smpl_buf *buf;
	int32_t rv;
	struct timespec tv;
	clock_gettime(CLOCK_REALTIME, &tv);
	tv.tv_sec += 1;

	rv = pthread_mutex_timedlock((pthread_mutex_t *)&r->this_stream->lock, &tv);
	if (rv != 0)
		return -rv;

	while (((r->this_stream->write_next + 1) & (r->this_stream->num_buffers - 1)) == r->this_stream->read_next &&
	       rv == 0)
		rv = pthread_cond_timedwait((pthread_cond_t *)&r->this_stream->ce,
					    (pthread_mutex_t *)&r->this_stream->lock, &tv);
	if (rv != 0)
		return -rv;

	buf = r->buf_ptrs[r->this_stream->write_next];
	buf->timestamp = timestamp;

	rv = len_in_sps <= r->this_stream->buffer_size ? len_in_sps : r->this_stream->buffer_size;

	memcpy((void *)buf->samples, data, SAMPLE_SIZE_BYTE * rv);
	buf->data_len = rv;

	r->this_stream->write_next = (r->this_stream->write_next + 1) & (r->this_stream->num_buffers - 1);

	pthread_cond_signal((pthread_cond_t *)&r->this_stream->cf);
	pthread_mutex_unlock((pthread_mutex_t *)&r->this_stream->lock);

	return rv;
}

int32_t ipc_shm_read(struct ipc_shm_io *r, uint16_t *out_buf, uint32_t num_samples, uint64_t *timestamp,
		     uint32_t timeout_seconds)
{
	volatile struct ipc_shm_raw_smpl_buf *buf;
	int32_t rv;
	uint8_t freeflag = 0;
	struct timespec tv;
	clock_gettime(CLOCK_REALTIME, &tv);
	tv.tv_sec += timeout_seconds;

	rv = pthread_mutex_timedlock((pthread_mutex_t *)&r->this_stream->lock, &tv);
	if (rv != 0)
		return -rv;

	while (r->this_stream->write_next == r->this_stream->read_next && rv == 0)
		rv = pthread_cond_timedwait((pthread_cond_t *)&r->this_stream->cf,
					    (pthread_mutex_t *)&r->this_stream->lock, &tv);
	if (rv != 0)
		return -rv;

	buf = r->buf_ptrs[r->this_stream->read_next];
	if (buf->data_len <= num_samples) {
		memcpy(out_buf, (void *)&buf->samples[r->partial_read_begin_ptr * 2], SAMPLE_SIZE_BYTE * buf->data_len);
		r->partial_read_begin_ptr = 0;
		rv = buf->data_len;
		buf->data_len = 0;
		r->this_stream->read_next = (r->this_stream->read_next + 1) & (r->this_stream->num_buffers - 1);
		freeflag = 1;

	} else /*if (buf->data_len > num_samples)*/ {
		memcpy(out_buf, (void *)&buf->samples[r->partial_read_begin_ptr * 2], SAMPLE_SIZE_BYTE * num_samples);
		r->partial_read_begin_ptr += num_samples;
		buf->data_len -= num_samples;
		rv = num_samples;
	}

	*timestamp = buf->timestamp;
	buf->timestamp += rv;

	if (freeflag)
		pthread_cond_signal((pthread_cond_t *)&r->this_stream->ce);

	pthread_mutex_unlock((pthread_mutex_t *)&r->this_stream->lock);

	return rv;
}
