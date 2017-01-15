#ifndef _LIB_RING_BUFFER_VFS_H
#define _LIB_RING_BUFFER_VFS_H

/*
 * lib/ringbuffer/vfs.h
 *
 * Wait-free ring buffer VFS file operations.
 *
 * Copyright (C) 2005-2012 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
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

#include <linux/fs.h>
#include <linux/poll.h>

/* VFS API */

extern const struct file_operations lib_ring_buffer_file_operations;

/*
 * Internal file operations.
 */

int lib_ring_buffer_open(struct inode *inode, struct file *file);
int lib_ring_buffer_release(struct inode *inode, struct file *file);
unsigned int lib_ring_buffer_poll(struct file *filp, poll_table *wait);
ssize_t lib_ring_buffer_splice_read(struct file *in, loff_t *ppos,
				    struct pipe_inode_info *pipe, size_t len,
				    unsigned int flags);
int lib_ring_buffer_mmap(struct file *filp, struct vm_area_struct *vma);

/* Ring Buffer ioctl() and ioctl numbers */
long lib_ring_buffer_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#ifdef CONFIG_COMPAT
long lib_ring_buffer_compat_ioctl(struct file *filp, unsigned int cmd,
				  unsigned long arg);
#endif

/*
 * Use RING_BUFFER_GET_NEXT_SUBBUF / RING_BUFFER_PUT_NEXT_SUBBUF to read and
 * consume sub-buffers sequentially.
 *
 * Reading sub-buffers without consuming them can be performed with:
 *
 * RING_BUFFER_SNAPSHOT
 * RING_BUFFER_SNAPSHOT_GET_CONSUMED
 * RING_BUFFER_SNAPSHOT_GET_PRODUCED
 *
 * to get the offset range to consume, and then by passing each sub-buffer
 * offset to RING_BUFFER_GET_SUBBUF, read the sub-buffer, and then release it
 * with RING_BUFFER_PUT_SUBBUF.
 *
 * Note that the "snapshot" API can be used to read the sub-buffer in reverse
 * order, which is useful for flight recorder snapshots.
 */

/* Get a snapshot of the current ring buffer producer and consumer positions */
#define RING_BUFFER_SNAPSHOT			_IO(0xF6, 0x00)
#define RING_BUFFER_COMPAT_SNAPSHOT		RING_BUFFER_SNAPSHOT
/* Get the consumer position (iteration start) */
#define RING_BUFFER_SNAPSHOT_GET_CONSUMED	_IOR(0xF6, 0x01, unsigned long)
#define RING_BUFFER_COMPAT_SNAPSHOT_GET_CONSUMED \
	_IOR(0xF6, 0x01, compat_ulong_t)
/* Get the producer position (iteration end) */
#define RING_BUFFER_SNAPSHOT_GET_PRODUCED	_IOR(0xF6, 0x02, unsigned long)
#define RING_BUFFER_COMPAT_SNAPSHOT_GET_PRODUCED \
	_IOR(0xF6, 0x02, compat_ulong_t)
/* Get exclusive read access to the specified sub-buffer position */
#define RING_BUFFER_GET_SUBBUF			_IOW(0xF6, 0x03, unsigned long)
#define RING_BUFFER_COMPAT_GET_SUBBUF		_IOW(0xF6, 0x03, compat_ulong_t)
/* Release exclusive sub-buffer access */
#define RING_BUFFER_PUT_SUBBUF			_IO(0xF6, 0x04)
#define RING_BUFFER_COMPAT_PUT_SUBBUF		RING_BUFFER_PUT_SUBBUF

/* Get exclusive read access to the next sub-buffer that can be read. */
#define RING_BUFFER_GET_NEXT_SUBBUF		_IO(0xF6, 0x05)
#define RING_BUFFER_COMPAT_GET_NEXT_SUBBUF	RING_BUFFER_GET_NEXT_SUBBUF
/* Release exclusive sub-buffer access, move consumer forward. */
#define RING_BUFFER_PUT_NEXT_SUBBUF		_IO(0xF6, 0x06)
#define RING_BUFFER_COMPAT_PUT_NEXT_SUBBUF	RING_BUFFER_PUT_NEXT_SUBBUF
/* returns the size of the current sub-buffer, without padding (for mmap). */
#define RING_BUFFER_GET_SUBBUF_SIZE		_IOR(0xF6, 0x07, unsigned long)
#define RING_BUFFER_COMPAT_GET_SUBBUF_SIZE	_IOR(0xF6, 0x07, compat_ulong_t)
/* returns the size of the current sub-buffer, with padding (for splice). */
#define RING_BUFFER_GET_PADDED_SUBBUF_SIZE	_IOR(0xF6, 0x08, unsigned long)
#define RING_BUFFER_COMPAT_GET_PADDED_SUBBUF_SIZE \
	_IOR(0xF6, 0x08, compat_ulong_t)
/* returns the maximum size for sub-buffers. */
#define RING_BUFFER_GET_MAX_SUBBUF_SIZE		_IOR(0xF6, 0x09, unsigned long)
#define RING_BUFFER_COMPAT_GET_MAX_SUBBUF_SIZE	_IOR(0xF6, 0x09, compat_ulong_t)
/* returns the length to mmap. */
#define RING_BUFFER_GET_MMAP_LEN		_IOR(0xF6, 0x0A, unsigned long)
#define RING_BUFFER_COMPAT_GET_MMAP_LEN		_IOR(0xF6, 0x0A, compat_ulong_t)
/* returns the offset of the subbuffer belonging to the mmap reader. */
#define RING_BUFFER_GET_MMAP_READ_OFFSET	_IOR(0xF6, 0x0B, unsigned long)
#define RING_BUFFER_COMPAT_GET_MMAP_READ_OFFSET	_IOR(0xF6, 0x0B, compat_ulong_t)
/* flush the current sub-buffer */
#define RING_BUFFER_FLUSH			_IO(0xF6, 0x0C)
#define RING_BUFFER_COMPAT_FLUSH		RING_BUFFER_FLUSH

#endif /* _LIB_RING_BUFFER_VFS_H */
