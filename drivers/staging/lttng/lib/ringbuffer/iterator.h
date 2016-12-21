#ifndef _LIB_RING_BUFFER_ITERATOR_H
#define _LIB_RING_BUFFER_ITERATOR_H

/*
 * lib/ringbuffer/iterator.h
 *
 * Ring buffer and channel iterators.
 *
 * Copyright (C) 2010-2012 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; only
 * version 2.1 of the License.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Author:
 *	Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
 */

#include "../../wrapper/ringbuffer/backend.h"
#include "../../wrapper/ringbuffer/frontend.h"
#include "../../wrapper/ringbuffer/vfs.h"

/*
 * lib_ring_buffer_get_next_record advances the buffer read position to the next
 * record. It returns either the size of the next record, -EAGAIN if there is
 * currently no data available, or -ENODATA if no data is available and buffer
 * is finalized.
 */
extern ssize_t lib_ring_buffer_get_next_record(struct channel *chan,
					       struct lib_ring_buffer *buf);

/*
 * channel_get_next_record advances the buffer read position to the next record.
 * It returns either the size of the next record, -EAGAIN if there is currently
 * no data available, or -ENODATA if no data is available and buffer is
 * finalized.
 * Returns the current buffer in ret_buf.
 */
extern ssize_t channel_get_next_record(struct channel *chan,
				       struct lib_ring_buffer **ret_buf);

/**
 * read_current_record - copy the buffer current record into dest.
 * @buf: ring buffer
 * @dest: destination where the record should be copied
 *
 * dest should be large enough to contain the record. Returns the number of
 * bytes copied.
 */
static inline size_t read_current_record(struct lib_ring_buffer *buf, void *dest)
{
	return lib_ring_buffer_read(&buf->backend, buf->iter.read_offset,
				    dest, buf->iter.payload_len);
}

extern int lib_ring_buffer_iterator_open(struct lib_ring_buffer *buf);
extern void lib_ring_buffer_iterator_release(struct lib_ring_buffer *buf);
extern int channel_iterator_open(struct channel *chan);
extern void channel_iterator_release(struct channel *chan);

extern const struct file_operations channel_payload_file_operations;
extern const struct file_operations lib_ring_buffer_payload_file_operations;

/*
 * Used internally.
 */
int channel_iterator_init(struct channel *chan);
void channel_iterator_unregister_notifiers(struct channel *chan);
void channel_iterator_free(struct channel *chan);
void channel_iterator_reset(struct channel *chan);
void lib_ring_buffer_iterator_reset(struct lib_ring_buffer *buf);

#endif /* _LIB_RING_BUFFER_ITERATOR_H */
