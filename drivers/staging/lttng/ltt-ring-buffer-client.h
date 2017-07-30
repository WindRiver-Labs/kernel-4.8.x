/*
 * ltt-ring-buffer-client.h
 *
 * Copyright (C) 2010 - Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
 *
 * LTTng lib ring buffer client template.
 *
 * Dual LGPL v2.1/GPL v2 license.
 */

#include <linux/module.h>
#include <linux/types.h>
#include "lib/bitfield.h"
#include "wrapper/vmalloc.h"	/* for wrapper_vmalloc_sync_all() */
#include "wrapper/trace-clock.h"
#include "ltt-events.h"
#include "ltt-tracer.h"
#include "wrapper/ringbuffer/frontend_types.h"

/*
 * Keep the natural field alignment for _each field_ within this structure if
 * you ever add/remove a field from this header. Packed attribute is not used
 * because gcc generates poor code on at least powerpc and mips. Don't ever
 * let gcc add padding between the structure elements.
 */

struct packet_header {
	/* Trace packet header */
	uint32_t magic;			/*
					 * Trace magic number.
					 * contains endianness information.
					 */
	uint8_t uuid[16];
	uint32_t stream_id;

	struct {
		/* Stream packet context */
		uint64_t timestamp_begin;	/* Cycle count at subbuffer start */
		uint64_t timestamp_end;		/* Cycle count at subbuffer end */
		uint32_t events_discarded;	/*
						 * Events lost in this subbuffer since
						 * the beginning of the trace.
						 * (may overflow)
						 */
		uint32_t content_size;		/* Size of data in subbuffer */
		uint32_t packet_size;		/* Subbuffer size (include padding) */
		uint32_t cpu_id;		/* CPU id associated with stream */
		uint8_t header_end;		/* End of header */
	} ctx;
};


static inline notrace u64 lib_ring_buffer_clock_read(struct channel *chan)
{
	return trace_clock_read64();
}

/*
 * record_header_size - Calculate the header size and padding necessary.
 * @config: ring buffer instance configuration
 * @chan: channel
 * @offset: offset in the write buffer
 * @data_size: size of the payload
 * @pre_header_padding: padding to add before the header (output)
 * @rflags: reservation flags
 * @ctx: reservation context
 *
 * Returns the event header size (including padding).
 *
 * The payload must itself determine its own alignment from the biggest type it
 * contains.
 */
static __inline__
unsigned char record_header_size(const struct lib_ring_buffer_config *config,
				 struct channel *chan, size_t offset,
				 size_t data_size, size_t *pre_header_padding,
				 unsigned int rflags,
				 struct lib_ring_buffer_ctx *ctx)
{
	struct ltt_channel *ltt_chan = channel_get_private(chan);
	size_t orig_offset = offset;
	size_t padding;

	switch (ltt_chan->header_type) {
	case 1:	/* compact */
		padding = lib_ring_buffer_align(offset, ltt_alignof(uint32_t));
		offset += padding;
		if (!(rflags & RING_BUFFER_RFLAG_FULL_TSC)) {
			offset += sizeof(uint32_t);	/* id and timestamp */
		} else {
			/* Minimum space taken by 5-bit id */
			offset += sizeof(uint8_t);
			/* Align extended struct on largest member */
			offset += lib_ring_buffer_align(offset, ltt_alignof(uint64_t));
			offset += sizeof(uint32_t);	/* id */
			offset += lib_ring_buffer_align(offset, ltt_alignof(uint64_t));
			offset += sizeof(uint64_t);	/* timestamp */
		}
		break;
	case 2:	/* large */
		padding = lib_ring_buffer_align(offset, ltt_alignof(uint16_t));
		offset += padding;
		offset += sizeof(uint16_t);
		if (!(rflags & RING_BUFFER_RFLAG_FULL_TSC)) {
			offset += lib_ring_buffer_align(offset, ltt_alignof(uint32_t));
			offset += sizeof(uint32_t);	/* timestamp */
		} else {
			/* Align extended struct on largest member */
			offset += lib_ring_buffer_align(offset, ltt_alignof(uint64_t));
			offset += sizeof(uint32_t);	/* id */
			offset += lib_ring_buffer_align(offset, ltt_alignof(uint64_t));
			offset += sizeof(uint64_t);	/* timestamp */
			
		}
		break;
	default:
		WARN_ON(1);
	}

	*pre_header_padding = padding;
	return offset - orig_offset;
}

#include "wrapper/ringbuffer/api.h"

extern
void ltt_write_event_header_slow(const struct lib_ring_buffer_config *config,
				 struct lib_ring_buffer_ctx *ctx,
				 u16 eID, u32 event_size);

/*
 * ltt_write_event_header
 *
 * Writes the event header to the offset (already aligned on 32-bits).
 *
 * @config: ring buffer instance configuration
 * @ctx: reservation context
 * @eID : event ID
 * @event_size : size of the event, excluding the event header.
 */
static __inline__
void ltt_write_event_header(const struct lib_ring_buffer_config *config,
			    struct lib_ring_buffer_ctx *ctx,
			    u16 eID, u32 event_size)
{
	struct ltt_channel *ltt_chan = channel_get_private(ctx->chan);

	if (unlikely(ctx->rflags))
		goto slow_path;

	switch (ltt_chan->header_type) {
	case 1:	/* compact */
	{
		uint32_t id_time = 0;

		bt_bitfield_write(&id_time, uint32_t, 0, 5, eID);
		bt_bitfield_write(&id_time, uint32_t, 5, 27, ctx->tsc);
		lib_ring_buffer_write(config, ctx, &id_time, sizeof(id_time));
		break;
	}
	case 2:	/* large */
	{
		uint16_t event_id = eID;
		uint32_t timestamp = (uint32_t) ctx->tsc;

		lib_ring_buffer_write(config, ctx, &event_id, sizeof(event_id));
		lib_ring_buffer_align_ctx(ctx, ltt_alignof(uint32_t));
		lib_ring_buffer_write(config, ctx, &timestamp, sizeof(timestamp));
		break;
	}
	default:
		WARN_ON(1);
	}
	return;

slow_path:
	ltt_write_event_header_slow(config, ctx, eID, event_size);
}

/*
 * TODO: For now, we only support 65536 event ids per channel.
 */
void ltt_write_event_header_slow(const struct lib_ring_buffer_config *config,
				   struct lib_ring_buffer_ctx *ctx,
				   u16 eID, u32 event_size)
{
	struct ltt_channel *ltt_chan = channel_get_private(ctx->chan);

	switch (ltt_chan->header_type) {
	case 1:	/* compact */
		if (!(ctx->rflags & RING_BUFFER_RFLAG_FULL_TSC)) {
			uint32_t id_time = 0;

			bt_bitfield_write(&id_time, uint32_t, 0, 5, eID);
			bt_bitfield_write(&id_time, uint32_t, 5, 27, ctx->tsc);
			lib_ring_buffer_write(config, ctx, &id_time, sizeof(id_time));
		} else {
			uint8_t id = 0;
			uint32_t event_id = (uint32_t) eID;
			uint64_t timestamp = ctx->tsc;

			bt_bitfield_write(&id, uint8_t, 0, 5, 31);
			lib_ring_buffer_write(config, ctx, &id, sizeof(id));
			/* Align extended struct on largest member */
			lib_ring_buffer_align_ctx(ctx, ltt_alignof(uint64_t));
			lib_ring_buffer_write(config, ctx, &event_id, sizeof(event_id));
			lib_ring_buffer_align_ctx(ctx, ltt_alignof(uint64_t));
			lib_ring_buffer_write(config, ctx, &timestamp, sizeof(timestamp));
		}
		break;
	case 2:	/* large */
	{
		if (!(ctx->rflags & RING_BUFFER_RFLAG_FULL_TSC)) {
			uint16_t event_id = eID;
			uint32_t timestamp = (uint32_t) ctx->tsc;

			lib_ring_buffer_write(config, ctx, &event_id, sizeof(event_id));
			lib_ring_buffer_align_ctx(ctx, ltt_alignof(uint32_t));
			lib_ring_buffer_write(config, ctx, &timestamp, sizeof(timestamp));
		} else {
			uint16_t event_id = 65535;
			uint32_t event_id_ext = (uint32_t) eID;
			uint64_t timestamp = ctx->tsc;

			lib_ring_buffer_write(config, ctx, &event_id, sizeof(event_id));
			/* Align extended struct on largest member */
			lib_ring_buffer_align_ctx(ctx, ltt_alignof(uint64_t));
			lib_ring_buffer_write(config, ctx, &event_id_ext, sizeof(event_id_ext));
			lib_ring_buffer_align_ctx(ctx, ltt_alignof(uint64_t));
			lib_ring_buffer_write(config, ctx, &timestamp, sizeof(timestamp));
		}
		break;
	}
	default:
		WARN_ON(1);
	}
}

static const struct lib_ring_buffer_config client_config;

static u64 client_ring_buffer_clock_read(struct channel *chan)
{
	return lib_ring_buffer_clock_read(chan);
}

static
size_t client_record_header_size(const struct lib_ring_buffer_config *config,
				 struct channel *chan, size_t offset,
				 size_t data_size,
				 size_t *pre_header_padding,
				 unsigned int rflags,
				 struct lib_ring_buffer_ctx *ctx)
{
	return record_header_size(config, chan, offset, data_size,
				  pre_header_padding, rflags, ctx);
}

/**
 * client_packet_header_size - called on buffer-switch to a new sub-buffer
 *
 * Return header size without padding after the structure. Don't use packed
 * structure because gcc generates inefficient code on some architectures
 * (powerpc, mips..)
 */
static size_t client_packet_header_size(void)
{
	return offsetof(struct packet_header, ctx.header_end);
}

static void client_buffer_begin(struct lib_ring_buffer *buf, u64 tsc,
				unsigned int subbuf_idx)
{
	struct channel *chan = buf->backend.chan;
	struct packet_header *header =
		(struct packet_header *)
			lib_ring_buffer_offset_address(&buf->backend,
				subbuf_idx * chan->backend.subbuf_size);
	struct ltt_channel *ltt_chan = channel_get_private(chan);
	struct ltt_session *session = ltt_chan->session;

	header->magic = CTF_MAGIC_NUMBER;
	memcpy(header->uuid, session->uuid.b, sizeof(session->uuid));
	header->stream_id = ltt_chan->id;
	header->ctx.timestamp_begin = tsc;
	header->ctx.timestamp_end = 0;
	header->ctx.events_discarded = 0;
	header->ctx.content_size = 0xFFFFFFFF; /* for debugging */
	header->ctx.packet_size = 0xFFFFFFFF;
	header->ctx.cpu_id = buf->backend.cpu;
}

/*
 * offset is assumed to never be 0 here : never deliver a completely empty
 * subbuffer. data_size is between 1 and subbuf_size.
 */
static void client_buffer_end(struct lib_ring_buffer *buf, u64 tsc,
			      unsigned int subbuf_idx, unsigned long data_size)
{
	struct channel *chan = buf->backend.chan;
	struct packet_header *header =
		(struct packet_header *)
			lib_ring_buffer_offset_address(&buf->backend,
				subbuf_idx * chan->backend.subbuf_size);
	unsigned long records_lost = 0;

	header->ctx.timestamp_end = tsc;
	header->ctx.content_size = data_size * CHAR_BIT; 	/* in bits */
	header->ctx.packet_size = PAGE_ALIGN(data_size) * CHAR_BIT; /* in bits */
	records_lost += lib_ring_buffer_get_records_lost_full(&client_config, buf);
	records_lost += lib_ring_buffer_get_records_lost_wrap(&client_config, buf);
	records_lost += lib_ring_buffer_get_records_lost_big(&client_config, buf);
	header->ctx.events_discarded = records_lost;
}

static int client_buffer_create(struct lib_ring_buffer *buf, void *priv,
				int cpu, const char *name)
{
	return 0;
}

static void client_buffer_finalize(struct lib_ring_buffer *buf, void *priv, int cpu)
{
}

static const struct lib_ring_buffer_config client_config = {
	.cb.ring_buffer_clock_read = client_ring_buffer_clock_read,
	.cb.record_header_size = client_record_header_size,
	.cb.subbuffer_header_size = client_packet_header_size,
	.cb.buffer_begin = client_buffer_begin,
	.cb.buffer_end = client_buffer_end,
	.cb.buffer_create = client_buffer_create,
	.cb.buffer_finalize = client_buffer_finalize,

	.tsc_bits = 32,
	.alloc = RING_BUFFER_ALLOC_PER_CPU,
	.sync = RING_BUFFER_SYNC_PER_CPU,
	.mode = RING_BUFFER_MODE_TEMPLATE,
	.backend = RING_BUFFER_PAGE,
	.output = RING_BUFFER_SPLICE,
	.oops = RING_BUFFER_OOPS_CONSISTENCY,
	.ipi = RING_BUFFER_IPI_BARRIER,
	.wakeup = RING_BUFFER_WAKEUP_BY_TIMER,
};

static
struct channel *_channel_create(const char *name,
				struct ltt_channel *ltt_chan, void *buf_addr,
				size_t subbuf_size, size_t num_subbuf,
				unsigned int switch_timer_interval,
				unsigned int read_timer_interval)
{
	return channel_create(&client_config, name, ltt_chan, buf_addr,
			      subbuf_size, num_subbuf, switch_timer_interval,
			      read_timer_interval);
}

static
void ltt_channel_destroy(struct channel *chan)
{
	channel_destroy(chan);
}

static
struct lib_ring_buffer *ltt_buffer_read_open(struct channel *chan)
{
	struct lib_ring_buffer *buf;
	int cpu;

	for_each_channel_cpu(cpu, chan) {
		buf = channel_get_ring_buffer(&client_config, chan, cpu);
		if (!lib_ring_buffer_open_read(buf))
			return buf;
	}
	return NULL;
}

static
void ltt_buffer_read_close(struct lib_ring_buffer *buf)
{
	lib_ring_buffer_release_read(buf);
	
}

static
int ltt_event_reserve(struct lib_ring_buffer_ctx *ctx)
{
	int ret, cpu;

	cpu = lib_ring_buffer_get_cpu(&client_config);
	if (cpu < 0)
		return -EPERM;
	ctx->cpu = cpu;

	ret = lib_ring_buffer_reserve(&client_config, ctx);
	if (ret)
		goto put;
	return ret;

put:
	lib_ring_buffer_put_cpu(&client_config);
	return ret;
}

static
void ltt_event_commit(struct lib_ring_buffer_ctx *ctx)
{
	lib_ring_buffer_commit(&client_config, ctx);
	lib_ring_buffer_put_cpu(&client_config);
}

static
void ltt_event_write(struct lib_ring_buffer_ctx *ctx, const void *src,
		     size_t len)
{
	lib_ring_buffer_write(&client_config, ctx, src, len);
}

static
wait_queue_head_t *ltt_get_reader_wait_queue(struct ltt_channel *chan)
{
	return &chan->chan->read_wait;
}

static struct ltt_transport ltt_relay_transport = {
	.name = "relay-" RING_BUFFER_MODE_TEMPLATE_STRING,
	.owner = THIS_MODULE,
	.ops = {
		.channel_create = _channel_create,
		.channel_destroy = ltt_channel_destroy,
		.buffer_read_open = ltt_buffer_read_open,
		.buffer_read_close = ltt_buffer_read_close,
		.event_reserve = ltt_event_reserve,
		.event_commit = ltt_event_commit,
		.event_write = ltt_event_write,
		.packet_avail_size = NULL,	/* Would be racy anyway */
		.get_reader_wait_queue = ltt_get_reader_wait_queue,
	},
};

static int __init ltt_ring_buffer_client_init(void)
{
	/*
	 * This vmalloc sync all also takes care of the lib ring buffer
	 * vmalloc'd module pages when it is built as a module into LTTng.
	 */
	wrapper_vmalloc_sync_all();
	printk(KERN_INFO "LTT : ltt ring buffer client init\n");
	ltt_transport_register(&ltt_relay_transport);
	return 0;
}

module_init(ltt_ring_buffer_client_init);

static void __exit ltt_ring_buffer_client_exit(void)
{
	printk(KERN_INFO "LTT : ltt ring buffer client exit\n");
	ltt_transport_unregister(&ltt_relay_transport);
}

module_exit(ltt_ring_buffer_client_exit);

MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("LTTng ring buffer " RING_BUFFER_MODE_TEMPLATE_STRING
		   " client");
