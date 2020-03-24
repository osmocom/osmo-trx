#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <osmocom/core/talloc.h>

#include "shm.h"

/* Convert offsets to pointers */
struct ipc_shm_stream *ipc_shm_decode_stream(void *tall_ctx, struct ipc_shm_raw_region *root_raw, struct ipc_shm_raw_stream *stream_raw)
{
        int i;
        struct ipc_shm_stream *stream;
        stream = talloc_zero(tall_ctx, struct ipc_shm_stream);
        stream = talloc_zero_size(tall_ctx, sizeof(struct ipc_shm_stream) +
                                          sizeof(struct ipc_shm_raw_smpl_buf *) * stream_raw->num_buffers);
        if (!stream)
                return NULL;
        stream->num_buffers = stream_raw->num_buffers;
        stream->buffer_size = stream_raw->buffer_size;
        stream->raw = stream_raw;
        for (i = 0; i < stream->num_buffers; i++) {
                fprintf(stderr, "decode: smpl_buf %d at offset %u\n", i, stream_raw->buffer_offset[i]);
                stream->buffers[i] = (struct ipc_shm_raw_smpl_buf *)(((uint8_t*)root_raw) + stream_raw->buffer_offset[i]);
        }
        return stream;
}

struct ipc_shm_channel *ipc_shm_decode_channel(void *tall_ctx, struct ipc_shm_raw_region *root_raw, struct ipc_shm_raw_channel *chan_raw)
{
        struct ipc_shm_channel *chan;
        chan = talloc_zero(tall_ctx, struct ipc_shm_channel);
        if (!chan)
                return NULL;
        fprintf(stderr, "decode: streams at offset %u and %u\n", chan_raw->dl_buf_offset, chan_raw->ul_buf_offset);
        chan->dl_stream = ipc_shm_decode_stream(chan, root_raw, (struct ipc_shm_raw_stream *)(((uint8_t*)root_raw) + chan_raw->dl_buf_offset));
        chan->ul_stream = ipc_shm_decode_stream(chan, root_raw, (struct ipc_shm_raw_stream *)(((uint8_t*)root_raw) + chan_raw->ul_buf_offset));
        return chan;
}
struct ipc_shm_region *ipc_shm_decode_region(void *tall_ctx, struct ipc_shm_raw_region *root_raw)
{
        int i;
        struct ipc_shm_region *root;
        root = talloc_zero_size(tall_ctx, sizeof(struct ipc_shm_region) +
                                          sizeof(struct ipc_shm_channel*) * root_raw->num_chans);
	if (!root)
		return NULL;

        root->num_chans = root_raw->num_chans;
        for (i = 0; i < root->num_chans; i++) {
                fprintf(stderr, "decode: channel %d at offset %u\n", i, root_raw->chan_offset[i]);
                root->channels[i] = ipc_shm_decode_channel(root, root_raw, (struct ipc_shm_raw_channel *)(((uint8_t*)root_raw) + root_raw->chan_offset[i]));
        }
        return root;
}

unsigned int ipc_shm_encode_smpl_buf(struct ipc_shm_raw_region *root_raw, struct ipc_shm_raw_smpl_buf *smpl_buf_raw, uint32_t buffer_size)
{
        uint8_t* start = (uint8_t*)smpl_buf_raw;
        unsigned int offset = sizeof(struct ipc_shm_raw_smpl_buf);

        fprintf(stderr, "encode: smpl_buf at offset %lu\n", (start - (uint8_t*)root_raw));

        offset += buffer_size * sizeof(uint16_t); /* samples */
        return offset;
}

unsigned int ipc_shm_encode_stream(struct ipc_shm_raw_region *root_raw, struct ipc_shm_raw_stream *stream_raw, uint32_t num_buffers, uint32_t buffer_size)
{
        int i;
        uint8_t* start = (uint8_t*)stream_raw;
        unsigned int offset = sizeof(struct ipc_shm_raw_stream) + sizeof(uint32_t)*num_buffers;

        fprintf(stderr, "encode: stream at offset %lu\n", (start - (uint8_t*)root_raw));

        if (root_raw) {
                stream_raw->num_buffers = num_buffers;
                stream_raw->buffer_size = buffer_size;
                stream_raw->read_next = 0;
                stream_raw->write_next = 0;
        }
        for (i = 0; i < num_buffers; i++) {
                if (root_raw)
                        stream_raw->buffer_offset[i] = (start + offset - (uint8_t*)root_raw);
                offset += ipc_shm_encode_smpl_buf(root_raw, (struct ipc_shm_raw_smpl_buf *)(start + offset), buffer_size);
        }
        return offset;
}
unsigned int ipc_shm_encode_channel(struct ipc_shm_raw_region *root_raw, struct ipc_shm_raw_channel *chan_raw, uint32_t num_buffers, uint32_t buffer_size)
{
        uint8_t* start = (uint8_t*)chan_raw;
        unsigned int offset = sizeof(struct ipc_shm_raw_channel);

        fprintf(stderr, "encode: channel at offset %lu\n", (start - (uint8_t*)root_raw));

        if (root_raw)
                chan_raw->dl_buf_offset = (start + offset - (uint8_t*)root_raw);
        offset += ipc_shm_encode_stream(root_raw, (struct ipc_shm_raw_stream *)(start + offset), num_buffers, buffer_size);
        if (root_raw)
                chan_raw->ul_buf_offset = (start + offset - (uint8_t*)root_raw);
        offset += ipc_shm_encode_stream(root_raw, (struct ipc_shm_raw_stream *)(start + offset), num_buffers, buffer_size);
        return offset;
}
/* if root_raw is NULL, then do a dry run, aka only calculate final offset */
unsigned int ipc_shm_encode_region(struct ipc_shm_raw_region *root_raw, uint32_t num_chans, uint32_t num_buffers, uint32_t buffer_size)
{
        int i;
        uint8_t* start = (uint8_t*)root_raw;
        unsigned int offset = sizeof(struct ipc_shm_raw_region) + sizeof(uint32_t)*num_chans;

        if (root_raw)
                root_raw->num_chans = num_chans;
        for (i = 0; i < num_chans; i++) {
                if (root_raw) {
                        root_raw->chan_offset[i] = (start + offset - (uint8_t*)root_raw);
                        fprintf(stderr, "encode: channel %d chan_offset[i]=%u\n", i, root_raw->chan_offset[i]);
                        offset += ipc_shm_encode_channel(root_raw, (struct ipc_shm_raw_channel *)(start + offset), num_buffers, buffer_size);
                }
        }
        //TODO: pass maximum size and verify we didn't go through
        return offset;
}
