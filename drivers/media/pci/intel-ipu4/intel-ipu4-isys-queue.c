/*
 * Copyright (c) 2013--2016 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/string.h>

#include <media/media-entity.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-ioctl.h>

#include "intel-ipu4.h"
#include "intel-ipu4-bus.h"
#include "intel-ipu4-buttress.h"
#include "intel-ipu4-isys.h"
#include "intel-ipu-isys-csi2-common.h"
#include "intel-ipu4-isys-video.h"

static int queue_setup(struct vb2_queue *q,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
		       const struct v4l2_format *__fmt,
#endif
		       unsigned int *num_buffers, unsigned int *num_planes,
		       unsigned int sizes[], void *alloc_ctxs[])
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(q);
	struct intel_ipu4_isys_video *av =
		intel_ipu4_isys_queue_to_video(aq);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	const struct v4l2_format *fmt = __fmt;
	const struct intel_ipu4_isys_pixelformat *pfmt;
	struct v4l2_pix_format_mplane mpix;
#else
	bool use_fmt = false;
#endif
	unsigned int i;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	if (fmt)
		mpix = fmt->fmt.pix_mp;
	else
		mpix = av->mpix;

	pfmt = av->try_fmt_vid_mplane(av, &mpix);

	*num_planes = mpix.num_planes;
#else
	/* num_planes == 0: we're being called through VIDIOC_REQBUFS */
	if (!*num_planes) {
		use_fmt = true;
		*num_planes = av->mpix.num_planes;
	}
#endif

	for (i = 0; i < *num_planes; i++) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
		sizes[i] = mpix.plane_fmt[i].sizeimage;
#else
		if (use_fmt)
			sizes[i] = av->mpix.plane_fmt[i].sizeimage;
#endif
		alloc_ctxs[i] = aq->ctx;
		dev_dbg(&av->isys->adev->dev,
			"%s: queue setup: plane %d size %u\n",
			av->vdev.name, i, sizes[i]);
	}

	return 0;
}

void intel_ipu4_isys_queue_lock(struct vb2_queue *q)
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(q);
	struct intel_ipu4_isys_video *av =
		intel_ipu4_isys_queue_to_video(aq);

	dev_dbg(&av->isys->adev->dev, "%s: queue lock\n", av->vdev.name);
	mutex_lock(&av->mutex);
}

void intel_ipu4_isys_queue_unlock(struct vb2_queue *q)
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(q);
	struct intel_ipu4_isys_video *av =
		intel_ipu4_isys_queue_to_video(aq);

	dev_dbg(&av->isys->adev->dev, "%s: queue unlock\n", av->vdev.name);
	mutex_unlock(&av->mutex);
}

static int buf_init(struct vb2_buffer *vb)
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(vb->vb2_queue);
	struct intel_ipu4_isys_video *av = intel_ipu4_isys_queue_to_video(aq);

	dev_dbg(&av->isys->adev->dev, "buffer: %s: buf_init\n", av->vdev.name);

	if (aq->buf_init)
		return aq->buf_init(vb);

	return 0;
}

int intel_ipu4_isys_buf_prepare(struct vb2_buffer *vb)
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(vb->vb2_queue);
	struct intel_ipu4_isys_video *av = intel_ipu4_isys_queue_to_video(aq);

	dev_dbg(&av->isys->adev->dev,
		"buffer: %s: configured size %u, buffer size %lu\n",
		av->vdev.name,
		av->mpix.plane_fmt[0].sizeimage, vb2_plane_size(vb, 0));

	if (av->mpix.plane_fmt[0].sizeimage > vb2_plane_size(vb, 0))
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, av->mpix.plane_fmt[0].bytesperline *
			      av->mpix.height);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	vb->v4l2_planes[0].data_offset = av->line_header_length / BITS_PER_BYTE;
#else
	vb->planes[0].data_offset = av->line_header_length / BITS_PER_BYTE;
#endif

	return 0;
}

static int buf_prepare(struct vb2_buffer *vb)
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(vb->vb2_queue);
	struct intel_ipu4_isys_video *av =
		intel_ipu4_isys_queue_to_video(aq);
	struct intel_ipu4_isys_buffer *ib =
		vb2_buffer_to_intel_ipu4_isys_buffer(vb);
	u32 request =
#ifdef MEDIA_IOC_REQUEST_CMD
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
		to_vb2_v4l2_buffer(vb)->request;
#else
		vb->v4l2_buf.request;
#endif
	struct media_device *mdev = &av->isys->media_dev;
	struct intel_ipu4_isys_request *ireq;
	u32 request_state;
	unsigned long flags;
#else /* ! MEDIA_IOC_REQUEST_CMD */
		0;
#endif /* ! MEDIA_IOC_REQUEST_CMD */
	int rval;

	if (av->isys->adev->isp->flr_done)
		return -EIO;

	if (request) {
		ib->req = media_device_request_find(&av->isys->media_dev,
						    request);
		if (!ib->req) {
			dev_dbg(&av->isys->adev->dev,
				"can't find request %u\n", request);
			return -ENOENT;
		}
	}

	rval = aq->buf_prepare(vb);
#ifdef MEDIA_IOC_REQUEST_CMD
	if (!request)
		return rval;
	if (rval)
		goto out_put_request;

	ireq = to_intel_ipu4_isys_request(ib->req);

	spin_lock_irqsave(&ireq->lock, flags);
	spin_lock(&mdev->req_lock);
	request_state = ib->req->state;
	if (request_state == MEDIA_DEVICE_REQUEST_STATE_IDLE)
		list_add(&ib->req_head, &ireq->buffers);
	spin_unlock(&mdev->req_lock);
	spin_unlock_irqrestore(&ireq->lock, flags);
	if (request_state != MEDIA_DEVICE_REQUEST_STATE_IDLE) {
		dev_dbg(&av->isys->adev->dev,
			"buf_prepare: request %u state %u\n", ib->req->id,
			request_state);
		rval = -EINVAL;
	} else {
		dev_dbg(&av->isys->adev->dev,
			"buf_prepare: request %u\n", ib->req->id);
	}

	if (!rval)
		return 0;

out_put_request:
	media_device_request_put(ib->req);
	ib->req = NULL;

#endif /* ! MEDIA_IOC_REQUEST_CMD */
	return rval;
}

static void buf_finish(struct vb2_buffer *vb)
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(vb->vb2_queue);
	struct intel_ipu4_isys_video *av =
		intel_ipu4_isys_queue_to_video(aq);
	struct intel_ipu4_isys_buffer *ib =
		vb2_buffer_to_intel_ipu4_isys_buffer(vb);

	dev_dbg(&av->isys->adev->dev, "buffer: %s: buf_finish\n",
		av->vdev.name);

	if (ib->req) {
		struct intel_ipu4_isys_request *ireq =
			to_intel_ipu4_isys_request(ib->req);
		unsigned long flags;
		bool done;

		spin_lock_irqsave(&ireq->lock, flags);
		list_del(&ib->req_head);
		done = list_empty(&ireq->buffers);
		spin_unlock_irqrestore(&ireq->lock, flags);
		dev_dbg(&av->isys->adev->dev, "request %u complete %s\n",
			ib->req->id, done ? "true" : "false");
		if (done) {
			media_device_request_complete(&av->isys->media_dev,
						      ib->req);
			mutex_lock(&av->isys->stream_mutex);
			list_del(&ireq->head);
			mutex_unlock(&av->isys->stream_mutex);
		}
		media_device_request_put(ib->req);
		ib->req = NULL;
	}
}

static void buf_cleanup(struct vb2_buffer *vb)
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(vb->vb2_queue);
	struct intel_ipu4_isys_video *av = intel_ipu4_isys_queue_to_video(aq);

	dev_dbg(&av->isys->adev->dev, "buffer: %s: buf_cleanup\n",
		av->vdev.name);

	if (aq->buf_cleanup)
		return aq->buf_cleanup(vb);
}

/*
 * Queue a buffer list back to incoming or active queues. The buffers
 * are removed from the buffer list.
 */
void intel_ipu4_isys_buffer_list_queue(struct intel_ipu4_isys_buffer_list *bl,
				unsigned long op_flags,
				enum vb2_buffer_state state)
{
	struct intel_ipu4_isys_buffer *ib, *ib_safe;
	unsigned long flags;
	bool first = true;

	if (!bl)
		return;

	WARN_ON(!bl->nbufs);
	WARN_ON(op_flags & INTEL_IPU4_ISYS_BUFFER_LIST_FL_ACTIVE &&
		op_flags & INTEL_IPU4_ISYS_BUFFER_LIST_FL_INCOMING);

	list_for_each_entry_safe(ib, ib_safe, &bl->head, head) {
		struct intel_ipu4_isys_video *av;

		if (ib->type == INTEL_IPU4_ISYS_VIDEO_BUFFER) {
			struct vb2_buffer *vb =
				intel_ipu4_isys_buffer_to_vb2_buffer(ib);
			struct intel_ipu4_isys_queue *aq =
				vb2_queue_to_intel_ipu4_isys_queue(
					vb->vb2_queue);

			av = intel_ipu4_isys_queue_to_video(aq);
			spin_lock_irqsave(&aq->lock, flags);
			list_del(&ib->head);
			if (op_flags & INTEL_IPU4_ISYS_BUFFER_LIST_FL_ACTIVE)
				list_add(&ib->head, &aq->active);
			else if (op_flags &
				INTEL_IPU4_ISYS_BUFFER_LIST_FL_INCOMING)
				list_add_tail(&ib->head, &aq->incoming);
			spin_unlock_irqrestore(&aq->lock, flags);

			if (op_flags & INTEL_IPU4_ISYS_BUFFER_LIST_FL_SET_STATE)
				vb2_buffer_done(vb, state);
		} else if (ib->type == INTEL_IPU4_ISYS_SHORT_PACKET_BUFFER) {
			struct intel_ipu4_isys_private_buffer *pb =
				intel_ipu4_isys_buffer_to_private_buffer(ib);
			struct intel_ipu4_isys_pipeline *ip = pb->ip;

			av = container_of(ip, struct intel_ipu4_isys_video, ip);
			spin_lock_irqsave(&ip->short_packet_queue_lock, flags);
			list_del(&ib->head);
			if (op_flags & INTEL_IPU4_ISYS_BUFFER_LIST_FL_ACTIVE)
				list_add(&ib->head, &ip->short_packet_active);
			else if (op_flags &
				INTEL_IPU4_ISYS_BUFFER_LIST_FL_INCOMING)
				list_add(&ib->head,
					&ip->short_packet_incoming);
			spin_unlock_irqrestore(&ip->short_packet_queue_lock,
				flags);
		} else {
			WARN_ON(1);
			return;
		}

		if (first) {
			dev_dbg(&av->isys->adev->dev,
				"queue buffer list %p op_flags %lx, \
				state %d, %d buffers\n",
				bl, op_flags, state, bl->nbufs);
			first = false;
		}

		bl->nbufs--;
	}

	WARN_ON(bl->nbufs);
}

/*
 * flush_firmware_streamon_fail() - Flush in cases where requests may
 * have been queued to firmware and the *firmware streamon fails for a
 * reason or another.
 */
static void flush_firmware_streamon_fail(struct intel_ipu4_isys_pipeline *ip)
{
	struct intel_ipu4_isys_video *pipe_av =
		container_of(ip, struct intel_ipu4_isys_video, ip);
	struct intel_ipu4_isys_queue *aq;
	unsigned long flags;

	lockdep_assert_held(&pipe_av->mutex);

	list_for_each_entry(aq, &ip->queues, node) {
		struct intel_ipu4_isys_video *av =
			intel_ipu4_isys_queue_to_video(aq);
		struct intel_ipu4_isys_buffer *ib, *ib_safe;

		spin_lock_irqsave(&aq->lock, flags);
		list_for_each_entry_safe(ib, ib_safe, &aq->active, head) {
			struct vb2_buffer *vb =
				intel_ipu4_isys_buffer_to_vb2_buffer(ib);

			list_del(&ib->head);
			if (av->streaming) {
				dev_dbg(&av->isys->adev->dev,
					"%s: queue buffer %u back to incoming\n",
					av->vdev.name,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
					vb->v4l2_buf.index);
#else
					vb->index);
#endif
				/* Queue already streaming, return to driver. */
				list_add(&ib->head, &aq->incoming);
				continue;
			}
			/* Queue not yet streaming, return to user. */
			dev_dbg(&av->isys->adev->dev,
				"%s: return %u back to videobuf2\n",
				av->vdev.name,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
				vb->v4l2_buf.index);
#else
				vb->index);
#endif
			vb2_buffer_done(
				intel_ipu4_isys_buffer_to_vb2_buffer(ib),
				VB2_BUF_STATE_QUEUED);
		}
		spin_unlock_irqrestore(&aq->lock, flags);
	}
}

/*
 * Attempt obtaining a buffer list from the incoming queues, a list of
 * buffers that contains one entry from each video buffer queue. If
 * all queues have no buffers, the buffers that were already dequeued
 * are returned to their queues.
 */
static int buffer_list_get(struct intel_ipu4_isys_pipeline *ip,
			struct intel_ipu4_isys_buffer_list *bl)
{
	struct intel_ipu4_isys_queue *aq;
	struct intel_ipu4_isys_buffer *ib;
	unsigned long flags;

	bl->nbufs = 0;
	INIT_LIST_HEAD(&bl->head);

	list_for_each_entry(aq, &ip->queues, node) {
		struct intel_ipu4_isys_buffer *ib;

		spin_lock_irqsave(&aq->lock, flags);
		if (list_empty(&aq->incoming)) {
			spin_unlock_irqrestore(&aq->lock, flags);
			goto failed_no_buffer;
		}

		ib = list_last_entry(&aq->incoming,
				     struct intel_ipu4_isys_buffer, head);
		if (ib->req) {
			spin_unlock_irqrestore(&aq->lock, flags);
			goto failed_no_buffer;
		}

		dev_dbg(&ip->isys->adev->dev, "buffer: %s: buffer %u\n",
			intel_ipu4_isys_queue_to_video(aq)->vdev.name,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
			intel_ipu4_isys_buffer_to_vb2_buffer(ib)->v4l2_buf.index

#else
			intel_ipu4_isys_buffer_to_vb2_buffer(ib)->index
#endif
			);
		list_del(&ib->head);
		list_add(&ib->head, &bl->head);
		spin_unlock_irqrestore(&aq->lock, flags);

		bl->nbufs++;
	}

	list_for_each_entry(ib, &bl->head, head) {
		struct vb2_buffer *vb =
			intel_ipu4_isys_buffer_to_vb2_buffer(ib);
		struct intel_ipu4_isys_queue *aq =
			vb2_queue_to_intel_ipu4_isys_queue(vb->vb2_queue);

		if (aq->prepare_frame_buff_set)
			aq->prepare_frame_buff_set(vb);
	}

	/* Get short packet buffer. */
	if (ip->interlaced && ip->isys->short_packet_source ==
	    INTEL_IPU_ISYS_SHORT_PACKET_FROM_RECEIVER) {
		spin_lock_irqsave(&ip->short_packet_queue_lock, flags);
		ib = intel_ipu_isys_csi2_get_short_packet_buffer(ip);
		if (!ib) {
			spin_unlock_irqrestore(&ip->short_packet_queue_lock,
					       flags);
			dev_err(&ip->isys->adev->dev,
				"No more short packet buffers. Driver bug?");
			WARN_ON(1);
			goto failed_no_buffer;
		}
		list_move(&ib->head, &bl->head);
		spin_unlock_irqrestore(&ip->short_packet_queue_lock, flags);
		bl->nbufs++;
	}

	dev_dbg(&ip->isys->adev->dev, "get buffer list %p, %u buffers\n", bl,
		bl->nbufs);
	return 0;

failed_no_buffer:
	if (!list_empty(&bl->head))
		intel_ipu4_isys_buffer_list_queue(bl,
			INTEL_IPU4_ISYS_BUFFER_LIST_FL_INCOMING, 0);
	return -ENODATA;
}

void intel_ipu4_isys_buffer_list_to_ipu_fw_isys_frame_buff_set_pin(
	struct vb2_buffer *vb, struct ipu_fw_isys_frame_buff_set_abi *set)
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(vb->vb2_queue);

	set->output_pins[aq->fw_output].addr =
		vb2_dma_contig_plane_dma_addr(vb, 0);
	set->output_pins[aq->fw_output].out_buf_id =
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
		vb->v4l2_buf.index + 1;
#else
		vb->index + 1;
#endif
}

/*
 * Convert a buffer list to a isys fw ABI framebuffer set. The
 * buffer list is not modified.
 */
void intel_ipu4_isys_buffer_list_to_ipu_fw_isys_frame_buff_set(
	struct ipu_fw_isys_frame_buff_set_abi *set,
	struct intel_ipu4_isys_pipeline *ip,
	struct intel_ipu4_isys_buffer_list *bl)
{
	struct intel_ipu4_isys_buffer *ib;

	WARN_ON(!bl->nbufs);

	set->send_irq_sof = 1;
	set->send_resp_sof = 1;

	list_for_each_entry(ib, &bl->head, head) {
		if (ib->type == INTEL_IPU4_ISYS_VIDEO_BUFFER) {
			struct vb2_buffer *vb =
				intel_ipu4_isys_buffer_to_vb2_buffer(ib);
			struct intel_ipu4_isys_queue *aq =
				vb2_queue_to_intel_ipu4_isys_queue(
					vb->vb2_queue);

			if (aq->fill_frame_buff_set_pin)
				aq->fill_frame_buff_set_pin(vb, set);
		} else if (ib->type == INTEL_IPU4_ISYS_SHORT_PACKET_BUFFER) {
			struct intel_ipu4_isys_private_buffer *pb =
				intel_ipu4_isys_buffer_to_private_buffer(ib);
			struct ipu_fw_isys_output_pin_payload_abi *output_pin =
				&set->output_pins[ip->short_packet_output_pin];

			output_pin->addr = pb->dma_addr;
			output_pin->out_buf_id = pb->index + 1;
		} else {
			WARN_ON(1);
		}
	}
}

static void intel_ipu4_isys_req_dispatch(
	struct media_device *mdev, struct intel_ipu4_isys_request *ireq,
	struct intel_ipu4_isys_pipeline *ip,
	struct ipu_fw_isys_frame_buff_set_abi *set,
	dma_addr_t dma_addr);

struct intel_ipu4_isys_request *
intel_ipu4_isys_next_queued_request(struct intel_ipu4_isys_pipeline *ip)
{
	struct intel_ipu4_isys *isys =
		container_of(ip, struct intel_ipu4_isys_video, ip)->isys;
	struct intel_ipu4_isys_request *ireq;
	struct intel_ipu4_isys_buffer *ib;
	unsigned long flags;

	lockdep_assert_held(&isys->stream_mutex);

	if (list_empty(&isys->requests)) {
		dev_dbg(&isys->adev->dev, "%s: no requests found\n",
			__func__);
		return NULL;
	}

	list_for_each_entry_reverse(ireq, &isys->requests, head) {
		/* Does the request belong to this pipeline? */
		bool is_ours = false;
		bool is_others = false;

		dev_dbg(&isys->adev->dev, "%s: checking request %u\n",
			__func__, ireq->req.id);

		spin_lock_irqsave(&ireq->lock, flags);
		list_for_each_entry(ib, &ireq->buffers, req_head) {
			struct vb2_buffer *vb =
				intel_ipu4_isys_buffer_to_vb2_buffer(ib);
			struct intel_ipu4_isys_queue *aq =
				vb2_queue_to_intel_ipu4_isys_queue(
					vb->vb2_queue);
			struct intel_ipu4_isys_video *av =
				intel_ipu4_isys_queue_to_video(aq);

			dev_dbg(&isys->adev->dev, "%s: buffer in vdev %s\n",
				__func__, av->vdev.name);

			if (media_entity_enum_test(&ip->entity_enum,
						   &av->vdev.entity))
				is_ours = true;
			else
				is_others = true;
		}
		spin_unlock_irqrestore(&ireq->lock, flags);

		dev_dbg(&isys->adev->dev, "%s: is%s ours, is%s others'\n",
			__func__, is_ours ? "" : "n't", is_others ? "" : "n't");

		if (!is_ours || WARN_ON(is_others))
			continue;

		list_del_init(&ireq->head);

		return ireq;
	}

	return NULL;
}

/* Start streaming for real. The buffer list must be available. */
static int intel_ipu4_isys_stream_start(struct intel_ipu4_isys_pipeline *ip,
					struct intel_ipu4_isys_buffer_list *bl,
					bool error)
{
	struct intel_ipu4_isys_video *pipe_av =
		container_of(ip, struct intel_ipu4_isys_video, ip);
	struct media_device *mdev = &pipe_av->isys->media_dev;
	struct intel_ipu4_isys_buffer_list __bl;
	struct intel_ipu4_isys_request *ireq;
	int rval;

	mutex_lock(&pipe_av->isys->stream_mutex);

	rval = intel_ipu4_isys_video_set_streaming(pipe_av, 1, bl);
	if (rval)
		goto out_requeue;

	ip->streaming = 1;

	dev_dbg(&pipe_av->isys->adev->dev, "dispatching queued requests\n");

	while ((ireq = intel_ipu4_isys_next_queued_request(ip))) {
		struct ipu_fw_isys_frame_buff_set_abi *set;
		struct isys_fw_msgs *msg;

		msg = intel_ipu4_get_fw_msg_buf(ip);
		if (!msg)
			/* TODO: A PROPER CLEAN UP */
			return -ENOMEM;

		set = to_frame_msg_buf(msg);

		intel_ipu4_isys_req_prepare(mdev, ireq, ip, set);
		fw_dump_isys_frame_buff_set(&pipe_av->isys->adev->dev, set,
						ip->nr_output_pins);
		intel_ipu4_isys_req_dispatch(mdev, ireq, ip, set,
					     to_dma_addr(msg));
	}

	dev_dbg(&pipe_av->isys->adev->dev,
		"done dispatching queued requests\n");

	mutex_unlock(&pipe_av->isys->stream_mutex);

	bl = &__bl;

	do {
		struct ipu_fw_isys_frame_buff_set_abi *buf = NULL;
		struct isys_fw_msgs *msg;

		rval = buffer_list_get(ip, bl);
		if (rval)
			break;

		msg = intel_ipu4_get_fw_msg_buf(ip);
		if (!msg)
			/* TODO: PROPER CLEANUP */
			return -ENOMEM;

		buf = to_frame_msg_buf(msg);

		intel_ipu4_isys_buffer_list_to_ipu_fw_isys_frame_buff_set(
			buf, ip, bl);

		intel_ipu4_isys_buffer_list_queue(
			bl, INTEL_IPU4_ISYS_BUFFER_LIST_FL_ACTIVE, 0);

		rval = pipe_av->isys->fwctrl->complex_cmd(
			pipe_av->isys,
			ip->stream_handle,
			buf,
			to_dma_addr(msg),
			sizeof(*buf),
			IPU_FW_ISYS_SEND_TYPE_STREAM_CAPTURE);
	} while (!WARN_ON(rval));

	return 0;

out_requeue:
	mutex_unlock(&pipe_av->isys->stream_mutex);
	if (bl && bl->nbufs)
		intel_ipu4_isys_buffer_list_queue(
			bl, INTEL_IPU4_ISYS_BUFFER_LIST_FL_INCOMING |
			(error ? INTEL_IPU4_ISYS_BUFFER_LIST_FL_SET_STATE : 0),
			error ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_QUEUED);
	flush_firmware_streamon_fail(ip);

	return rval;
}

static void __buf_queue(struct vb2_buffer *vb, bool force)
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(vb->vb2_queue);
	struct intel_ipu4_isys_video *av = intel_ipu4_isys_queue_to_video(aq);
	struct intel_ipu4_isys_buffer *ib =
		vb2_buffer_to_intel_ipu4_isys_buffer(vb);
	struct intel_ipu4_isys_pipeline *ip =
		to_intel_ipu4_isys_pipeline(av->vdev.entity.pipe);
	struct intel_ipu4_isys_buffer_list bl;

	struct ipu_fw_isys_frame_buff_set_abi *buf = NULL;
	struct isys_fw_msgs *msg;

	struct intel_ipu4_isys_video *pipe_av =
		container_of(ip, struct intel_ipu4_isys_video, ip);
	unsigned long flags;
	unsigned int i;
	int rval;

	dev_dbg(&av->isys->adev->dev, "buffer: %s: buf_queue %u\n",
		av->vdev.name,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
		vb->v4l2_buf.index
#else
		vb->index
#endif
		);

	for (i = 0; i < vb->num_planes; i++)
		dev_dbg(&av->isys->adev->dev, "iova: plane %u iova 0x%x\n", i,
			(uint32_t)vb2_dma_contig_plane_dma_addr(vb, i));

	spin_lock_irqsave(&aq->lock, flags);
	list_add(&ib->head, &aq->incoming);
	spin_unlock_irqrestore(&aq->lock, flags);

	if (ib->req)
		return;

	if (!pipe_av || !vb->vb2_queue->streaming) {
		dev_dbg(&av->isys->adev->dev,
			"not pipe_av set, adding to incoming\n");
		return;
	}

	mutex_unlock(&av->mutex);
	mutex_lock(&pipe_av->mutex);

	if (!force && ip->nr_streaming != ip->nr_queues) {
		dev_dbg(&av->isys->adev->dev,
			"not streaming yet, adding to incoming\n");
		goto out;
	}

	/*
	 * We just put one buffer to the incoming list of this queue
	 * (above). Let's see whether all queues in the pipeline would
	 * have a buffer.
	 */
	rval = buffer_list_get(ip, &bl);
	if (rval < 0) {
		dev_dbg(&av->isys->adev->dev,
			"not enough buffers available\n");
		goto out;
	}

	msg = intel_ipu4_get_fw_msg_buf(ip);
	if (!msg) {
		rval = -ENOMEM;
		goto out;
	}
	buf = to_frame_msg_buf(msg);

	intel_ipu4_isys_buffer_list_to_ipu_fw_isys_frame_buff_set(
		buf, ip, &bl);

	if (!ip->streaming) {
		dev_dbg(&av->isys->adev->dev,
			"Wow! Got a buffer to start streaming!\n");
		rval = intel_ipu4_isys_stream_start(ip, &bl, true);
		if (rval)
			dev_err(&av->isys->adev->dev,
				"Ouch. Stream start failed.\n");
		goto out;
	}

	/*
	 * We must queue the buffers in the buffer list to the
	 * appropriate video buffer queues BEFORE passing them to the
	 * firmware since we could get a buffer event back before we
	 * have queued them ourselves to the active queue.
	 */
	intel_ipu4_isys_buffer_list_queue(&bl,
				INTEL_IPU4_ISYS_BUFFER_LIST_FL_ACTIVE, 0);

	rval = pipe_av->isys->fwctrl->complex_cmd(pipe_av->isys,
				     ip->stream_handle,
				     buf,
				     to_dma_addr(msg), sizeof(*buf),
				     IPU_FW_ISYS_SEND_TYPE_STREAM_CAPTURE);
	/*
	 * FIXME: mark the buffers in the buffer list if the queue
	 * operation fails.
	 */
	if (!WARN_ON(rval < 0))
		dev_dbg(&av->isys->adev->dev, "queued buffer\n");

out:
	mutex_unlock(&pipe_av->mutex);
	mutex_lock(&av->mutex);
}

static void buf_queue(struct vb2_buffer *vb)
{
	__buf_queue(vb, false);
}

int intel_ipu4_isys_link_fmt_validate(struct intel_ipu4_isys_queue *aq)
{
	struct intel_ipu4_isys_video *av = intel_ipu4_isys_queue_to_video(aq);
	struct v4l2_subdev_format fmt = { 0 };
	struct media_pad *pad = media_entity_remote_pad(av->vdev.entity.pads);
	struct v4l2_subdev *sd;
	int rval;

	if (!pad) {
		dev_dbg(&av->isys->adev->dev,
			"video node %s pad not connected\n", av->vdev.name);
		return -ENOTCONN;
	}

	sd = media_entity_to_v4l2_subdev(pad->entity);

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = pad->index;
	fmt.stream = 0;
	rval = v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt);
	if (rval)
		return rval;

	if (fmt.format.width != av->mpix.width
	    || fmt.format.height != av->mpix.height) {
		dev_dbg(&av->isys->adev->dev,
			"wrong width or height %ux%u (%ux%u expected)\n",
			av->mpix.width, av->mpix.height,
			fmt.format.width, fmt.format.height);
		return -EINVAL;
	}

	if (fmt.format.field != av->mpix.field) {
		dev_dbg(&av->isys->adev->dev,
			"wrong field value 0x%8.8x (0x%8.8x expected)\n",
			av->mpix.field, fmt.format.field);
		return -EINVAL;
	}

	if (fmt.format.code != av->pfmt->code) {
		dev_dbg(&av->isys->adev->dev,
			"wrong media bus code 0x%8.8x (0x%8.8x expected)\n",
			av->pfmt->code, fmt.format.code);
		return -EINVAL;
	}

	return 0;
}

/* Return buffers back to videobuf2. */
static void return_buffers(struct intel_ipu4_isys_queue *aq,
			   enum vb2_buffer_state state)
{
	struct intel_ipu4_isys_video *av = intel_ipu4_isys_queue_to_video(aq);
	int reset_needed = 0;
	unsigned long flags;

	spin_lock_irqsave(&aq->lock, flags);
	while (!list_empty(&aq->incoming)) {
		struct intel_ipu4_isys_buffer *ib =
			list_first_entry(&aq->incoming,
					 struct intel_ipu4_isys_buffer, head);
		struct vb2_buffer *vb =
			intel_ipu4_isys_buffer_to_vb2_buffer(ib);

		list_del(&ib->head);
		spin_unlock_irqrestore(&aq->lock, flags);

		vb2_buffer_done(vb, state);

		dev_dbg(&av->isys->adev->dev,
			"%s: stop_streaming incoming %u\n",
			intel_ipu4_isys_queue_to_video(
				vb2_queue_to_intel_ipu4_isys_queue(
					vb->vb2_queue))->vdev.name,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
			vb->v4l2_buf.index);
#else
			vb->index);
#endif

		spin_lock_irqsave(&aq->lock, flags);
	}

	/*
	 * Something went wrong (FW crash / HW hang / not all buffers
	 * returned from isys) if there are still buffers queued in active
	 * queue. We have to clean up places a bit.
	 */
	while (!list_empty(&aq->active)) {
		struct intel_ipu4_isys_buffer *ib =
			list_first_entry(&aq->active,
				struct intel_ipu4_isys_buffer, head);
		struct vb2_buffer *vb =
			intel_ipu4_isys_buffer_to_vb2_buffer(ib);

		list_del(&ib->head);
		spin_unlock_irqrestore(&aq->lock, flags);

		vb2_buffer_done(vb, state);

		dev_warn(&av->isys->adev->dev,
			"%s: cleaning active queue %u\n",
			intel_ipu4_isys_queue_to_video(
				vb2_queue_to_intel_ipu4_isys_queue(
					vb->vb2_queue))->vdev.name,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
			vb->v4l2_buf.index);
#else
			vb->index);
#endif

		spin_lock_irqsave(&aq->lock, flags);
		reset_needed = 1;
	}

	spin_unlock_irqrestore(&aq->lock, flags);

	if (reset_needed) {
		mutex_lock(&av->isys->mutex);
		av->isys->reset_needed = true;
		mutex_unlock(&av->isys->mutex);
	}
}

static int start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(q);
	struct intel_ipu4_isys_video *av = intel_ipu4_isys_queue_to_video(aq);
	struct intel_ipu4_isys_video *pipe_av;
	struct intel_ipu4_isys_pipeline *ip;
	struct intel_ipu4_isys_buffer_list __bl, *bl = NULL;
	bool first;
	int rval;

	dev_dbg(&av->isys->adev->dev,
		"stream: %s: width %u, height %u, css pixelformat %u\n",
		av->vdev.name, av->mpix.width, av->mpix.height,
		av->pfmt->css_pixelformat);

	mutex_lock(&av->isys->stream_mutex);

	first = !av->vdev.entity.pipe;

	if (first) {
		rval = intel_ipu4_isys_video_prepare_streaming(av, 1);
		if (rval)
			goto out_return_buffers;
	}

	mutex_unlock(&av->isys->stream_mutex);

	rval = aq->link_fmt_validate(aq);
	if (rval) {
		dev_dbg(&av->isys->adev->dev,
			"%s: link format validation failed (%d)\n",
			av->vdev.name, rval);
		goto out_unprepare_streaming;
	}

	ip = to_intel_ipu4_isys_pipeline(av->vdev.entity.pipe);
	pipe_av = container_of(ip, struct intel_ipu4_isys_video, ip);
	mutex_unlock(&av->mutex);

	mutex_lock(&pipe_av->mutex);
	ip->nr_streaming++;
	dev_dbg(&av->isys->adev->dev, "queue %u of %u\n", ip->nr_streaming,
		ip->nr_queues);
	list_add(&aq->node, &ip->queues);
	if (ip->nr_streaming != ip->nr_queues)
		goto out;

	if (list_empty(&av->isys->requests)) {
		bl = &__bl;
		rval = buffer_list_get(ip, bl);
		if (rval) {
			dev_dbg(&av->isys->adev->dev,
				"no request available --- postponing streamon\n");
			goto out;
		}
	}

	rval = intel_ipu4_isys_stream_start(ip, bl, false);
	if (rval)
		goto out_stream_start;

out:
	mutex_unlock(&pipe_av->mutex);
	mutex_lock(&av->mutex);

	return 0;

out_stream_start:
	list_del(&aq->node);
	ip->nr_streaming--;
	mutex_unlock(&pipe_av->mutex);
	mutex_lock(&av->mutex);

out_unprepare_streaming:
	mutex_lock(&av->isys->stream_mutex);
	if (first)
		intel_ipu4_isys_video_prepare_streaming(av, 0);

out_return_buffers:
	mutex_unlock(&av->isys->stream_mutex);
	return_buffers(aq, VB2_BUF_STATE_QUEUED);

	return rval;
}

static void stop_streaming(struct vb2_queue *q)
{
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(q);
	struct intel_ipu4_isys_video *av = intel_ipu4_isys_queue_to_video(aq);
	struct intel_ipu4_isys_pipeline *ip =
		to_intel_ipu4_isys_pipeline(av->vdev.entity.pipe);
	struct intel_ipu4_isys_video *pipe_av =
		container_of(ip, struct intel_ipu4_isys_video, ip);

	if (pipe_av != av) {
		mutex_unlock(&av->mutex);
		mutex_lock(&pipe_av->mutex);
	}

	mutex_lock(&av->isys->stream_mutex);
	if (ip->nr_streaming == ip->nr_queues && ip->streaming)
		intel_ipu4_isys_video_set_streaming(av, 0, NULL);
	if (ip->nr_streaming == 1)
		intel_ipu4_isys_video_prepare_streaming(av, 0);
	mutex_unlock(&av->isys->stream_mutex);

	ip->nr_streaming--;
	list_del(&aq->node);
	ip->streaming = 0;

	if (pipe_av != av) {
		mutex_unlock(&pipe_av->mutex);
		mutex_lock(&av->mutex);
	}

	return_buffers(aq, VB2_BUF_STATE_ERROR);
}

static unsigned int get_sof_sequence_by_timestamp(
	struct intel_ipu4_isys_pipeline *ip,
	struct ipu_fw_isys_resp_info_abi *info)
{
	struct intel_ipu4_isys *isys =
		container_of(ip, struct intel_ipu4_isys_video, ip)->isys;
	u64 time = (u64)info->timestamp[1] << 32 | info->timestamp[0];
	unsigned int i;

	for (i = 0; i < INTEL_IPU4_ISYS_MAX_PARALLEL_SOF; i++)
		if (time == ip->seq[i].timestamp) {
			dev_dbg(&isys->adev->dev,
				"sof: using sequence number %u for timestamp \
				0x%16.16llx\n",
				ip->seq[i].sequence, time);
			return ip->seq[i].sequence;
		}

	dev_dbg(&isys->adev->dev, "SOF: looking for 0x%16.16llx\n", time);
	for (i = 0; i < INTEL_IPU4_ISYS_MAX_PARALLEL_SOF; i++)
		dev_dbg(&isys->adev->dev,
			"SOF: sequence %u, timestamp value 0x%16.16llx\n",
			ip->seq[i].sequence, ip->seq[i].timestamp);
	dev_dbg(&isys->adev->dev, "SOF sequence number not found\n");

	return 0;
}

static u64 get_sof_ns_delta(struct intel_ipu4_isys_video *av,
			struct ipu_fw_isys_resp_info_abi *info)
{
	struct intel_ipu4_bus_device *adev =
		to_intel_ipu4_bus_device(&av->isys->adev->dev);
	struct intel_ipu4_device *isp = adev->isp;
	u64 delta, tsc_now;

	if (!intel_ipu4_buttress_tsc_read(isp, &tsc_now))
		delta = tsc_now -
			((u64)info->timestamp[1] << 32 | info->timestamp[0]);
	else
		delta = 0;

	return intel_ipu4_buttress_tsc_ticks_to_ns(delta);
}

void intel_ipu4_isys_buf_calc_sequence_time(struct intel_ipu4_isys_buffer *ib,
					struct ipu_fw_isys_resp_info_abi *info)
{
	struct vb2_buffer *vb = intel_ipu4_isys_buffer_to_vb2_buffer(ib);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	struct timespec ts_now;
#endif
	struct intel_ipu4_isys_queue *aq =
		vb2_queue_to_intel_ipu4_isys_queue(vb->vb2_queue);
	struct intel_ipu4_isys_video *av = intel_ipu4_isys_queue_to_video(aq);
	struct intel_ipu4_isys_pipeline *ip =
		to_intel_ipu4_isys_pipeline(av->vdev.entity.pipe);
	u64 ns;
	u32 sequence;

	if (ip->has_sof) {
		ns = ktime_get_ns() - get_sof_ns_delta(av, info);
		sequence = get_sof_sequence_by_timestamp(ip, info);
	} else {
		ns = ktime_get_ns();
		sequence = (atomic_inc_return(&ip->sequence) - 1)
			/ ip->nr_queues;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	vb->v4l2_buf.sequence = sequence;
	ts_now = ns_to_timespec(ns);
	vb->v4l2_buf.timestamp.tv_sec = ts_now.tv_sec;
	vb->v4l2_buf.timestamp.tv_usec = ts_now.tv_nsec / NSEC_PER_USEC;

	dev_dbg(&av->isys->adev->dev, "buffer: %s: buffer done %u\n",
		av->vdev.name,
		vb->v4l2_buf.index);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	vbuf->sequence = sequence;
	ts_now = ns_to_timespec(ns);
	vbuf->timestamp.tv_sec = ts_now.tv_sec;
	vbuf->timestamp.tv_usec = ts_now.tv_nsec / NSEC_PER_USEC;

	dev_dbg(&av->isys->adev->dev, "%s: buffer done %u\n", av->vdev.name,
		vb->index);
#else
	vbuf->vb2_buf.timestamp = ns;
	vbuf->sequence = sequence;

	dev_dbg(&av->isys->adev->dev, "buffer: %s: buffer done %u\n",
		av->vdev.name,
		vb->index);
#endif
}

void intel_ipu4_isys_queue_buf_done(struct intel_ipu4_isys_buffer *ib)
{
	struct vb2_buffer *vb = intel_ipu4_isys_buffer_to_vb2_buffer(ib);

	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
}

void intel_ipu4_isys_queue_buf_ready(struct intel_ipu4_isys_pipeline *ip,
				     struct ipu_fw_isys_resp_info_abi *info)
{
	struct intel_ipu4_isys *isys =
		container_of(ip, struct intel_ipu4_isys_video, ip)->isys;
	struct intel_ipu4_isys_queue *aq = ip->output_pins[info->pin_id].aq;
	struct intel_ipu4_isys_buffer *ib;
	struct vb2_buffer *vb;
	unsigned long flags;
	bool first = true;

	dev_dbg(&isys->adev->dev, "buffer: %s: received buffer %8.8x\n",
		intel_ipu4_isys_queue_to_video(aq)->vdev.name, info->pin.addr);

	spin_lock_irqsave(&aq->lock, flags);
	if (list_empty(&aq->active)) {
		spin_unlock_irqrestore(&aq->lock, flags);
		dev_err(&isys->adev->dev, "active queue empty\n");
		return;
	}

	list_for_each_entry_reverse(ib, &aq->active, head) {
		dma_addr_t addr;

		vb = intel_ipu4_isys_buffer_to_vb2_buffer(ib);
		addr = vb2_dma_contig_plane_dma_addr(vb, 0);

		if (info->pin.addr != addr) {
			if (first)
				dev_err(&isys->adev->dev,
					"WARNING: buffer address %pad expected!\n",
					&addr);
			first = false;
			continue;
		}

		dev_dbg(&isys->adev->dev, "buffer: found buffer %pad\n", &addr);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
		vb->v4l2_buf.field = V4L2_FIELD_NONE;
		/*
		 * Use "reserved" field to pass csi2 index and vc.
		 * May need to change to other approach.
		 */
		vb->v4l2_buf.reserved &= 0xFFFFFF00;
		if (ip->csi2)
			vb->v4l2_buf.reserved |= ip->csi2->index << 4;
		vb->v4l2_buf.reserved |= ip->vc;
#else
		to_vb2_v4l2_buffer(vb)->field = V4L2_FIELD_NONE;
#endif

		list_del(&ib->head);
		spin_unlock_irqrestore(&aq->lock, flags);

		intel_ipu4_isys_buf_calc_sequence_time(ib, info);

		/*
		 * For interlaced buffers, the notification to user space
		 * is postponed to capture_done event since the field
		 * information is available only at that time.
		 */
		if (ip->interlaced) {
			spin_lock_irqsave(&ip->short_packet_queue_lock, flags);
			list_add(&ib->head, &ip->pending_interlaced_bufs);
			spin_unlock_irqrestore(&ip->short_packet_queue_lock,
					       flags);
		} else {
			intel_ipu4_isys_queue_buf_done(ib);
		}

		return;
	}

	dev_err(&isys->adev->dev,
		"WARNING: cannot find a matching video buffer!\n");

	spin_unlock_irqrestore(&aq->lock, flags);
}

void intel_ipu4_isys_queue_short_packet_ready(
	struct intel_ipu4_isys_pipeline *ip,
	struct ipu_fw_isys_resp_info_abi *info)
{
	struct intel_ipu4_isys *isys =
		container_of(ip, struct intel_ipu4_isys_video, ip)->isys;
	unsigned long flags;

	dev_dbg(&isys->adev->dev, "receive short packet buffer %8.8x\n",
		info->pin.addr);
	spin_lock_irqsave(&ip->short_packet_queue_lock, flags);
	ip->cur_field = intel_ipu_isys_csi2_get_current_field(ip, info);
	spin_unlock_irqrestore(&ip->short_packet_queue_lock, flags);
}

void intel_ipu4_isys_req_free(struct media_device *mdev,
			      struct media_device_request *req)
{
	struct intel_ipu4_isys_request *ireq = to_intel_ipu4_isys_request(req);

	kfree(ireq);
}

struct media_device_request *intel_ipu4_isys_req_alloc(
	struct media_device *mdev)
{
	struct intel_ipu4_isys_request *ireq;

	ireq = kzalloc(sizeof(*ireq), GFP_KERNEL);
	if (!ireq)
		return NULL;

	INIT_LIST_HEAD(&ireq->buffers);
	spin_lock_init(&ireq->lock);
	INIT_LIST_HEAD(&ireq->head);

	return &ireq->req;
}

void intel_ipu4_isys_req_prepare(struct media_device *mdev,
				 struct intel_ipu4_isys_request *ireq,
				 struct intel_ipu4_isys_pipeline *ip,
				 struct ipu_fw_isys_frame_buff_set_abi *set)
{
	struct intel_ipu4_isys *isys =
		container_of(ip, struct intel_ipu4_isys_video, ip)->isys;
	struct media_device_request *req = &ireq->req;
	struct intel_ipu4_isys_buffer *ib;
	unsigned long flags;

	dev_dbg(&isys->adev->dev, "preparing request %u\n", req->id);

	set->send_irq_sof = 1;
	set->send_resp_sof = 1;

	spin_lock_irqsave(&ireq->lock, flags);

	list_for_each_entry(ib, &ireq->buffers, req_head) {
		struct vb2_buffer *vb =
			intel_ipu4_isys_buffer_to_vb2_buffer(ib);
		struct intel_ipu4_isys_queue *aq =
			vb2_queue_to_intel_ipu4_isys_queue(vb->vb2_queue);

		if (aq->prepare_frame_buff_set)
			aq->prepare_frame_buff_set(vb);

		if (aq->fill_frame_buff_set_pin)
			aq->fill_frame_buff_set_pin(vb, set);

		spin_lock(&aq->lock);
		list_move(&ib->head, &aq->active);
		spin_unlock(&aq->lock);
	}

	spin_unlock_irqrestore(&ireq->lock, flags);
}

static void intel_ipu4_isys_req_dispatch(
	struct media_device *mdev, struct intel_ipu4_isys_request *ireq,
	struct intel_ipu4_isys_pipeline *ip,
	struct ipu_fw_isys_frame_buff_set_abi *set,
	dma_addr_t dma_addr)
{
	struct intel_ipu4_isys_video *pipe_av =
		container_of(ip, struct intel_ipu4_isys_video, ip);
	int rval;

	rval = pipe_av->isys->fwctrl->complex_cmd(
		pipe_av->isys,
		ip->stream_handle,
		set,
		dma_addr, sizeof(*set),
		IPU_FW_ISYS_SEND_TYPE_STREAM_CAPTURE);

	WARN_ON(rval);
}

int intel_ipu4_isys_req_queue(struct media_device *mdev,
			      struct media_device_request *req)
{
	struct intel_ipu4_isys *isys =
		container_of(mdev, struct intel_ipu4_isys, media_dev);
	struct intel_ipu4_isys_request *ireq = to_intel_ipu4_isys_request(req);
	struct intel_ipu4_isys_pipeline *ip;
	struct intel_ipu4_isys_buffer *ib;
	struct media_pipeline *pipe = NULL;
	unsigned long flags;
	bool no_pipe = false;
	int rval = 0;

	spin_lock_irqsave(&ireq->lock, flags);
	if (list_empty(&ireq->buffers)) {
		rval = -ENODATA;
		goto out_list_empty;
	}

	/* Verify that all buffers are related to a single pipeline. */
	list_for_each_entry(ib, &ireq->buffers, req_head) {
		struct vb2_buffer *vb =
			intel_ipu4_isys_buffer_to_vb2_buffer(ib);
		struct intel_ipu4_isys_queue *aq =
			vb2_queue_to_intel_ipu4_isys_queue(vb->vb2_queue);
		struct intel_ipu4_isys_video *av =
			intel_ipu4_isys_queue_to_video(aq);

		dev_dbg(&isys->adev->dev, "%s: device %s, id %u\n", __func__,
			av->vdev.name, vb->
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
			v4l2_buf.
#endif
			index
			);
		if (!pipe) {
			if (!av->vdev.entity.pipe) {
				no_pipe = true;
				continue;
			}

			pipe = av->vdev.entity.pipe;
			dev_dbg(&isys->adev->dev, "%s: pipe %p\n",
				av->vdev.name, pipe);
			continue;
		}

		if (av->vdev.entity.pipe != pipe) {
			dev_dbg(&isys->adev->dev,
				"request %u includes buffers in multiple pipelines\n",
				req->id);
			rval = -EINVAL;
			goto out_list_empty;
		}
	}

	spin_unlock_irqrestore(&ireq->lock, flags);

	mutex_lock(&isys->stream_mutex);

	ip = to_intel_ipu4_isys_pipeline(pipe);

	if (pipe && ip->streaming) {
		struct isys_fw_msgs *msg;
		struct ipu_fw_isys_frame_buff_set_abi *set;

		msg = intel_ipu4_get_fw_msg_buf(ip);
		if (!msg) {
			rval = -ENOMEM;
			goto out_mutex_unlock;
		}

		set = to_frame_msg_buf(msg);

		if (no_pipe) {
			dev_dbg(&isys->adev->dev,
				"request %u includes buffers in and outside pipelines\n",
				req->id);
			rval = -EINVAL;
			goto out_mutex_unlock;
		}

		dev_dbg(&isys->adev->dev,
			"request has a pipeline, dispatching\n");
		intel_ipu4_isys_req_prepare(mdev, ireq, ip, set);
		fw_dump_isys_frame_buff_set(&isys->adev->dev, set,
						ip->nr_output_pins);
		intel_ipu4_isys_req_dispatch(mdev, ireq, ip, set,
					     to_dma_addr(msg));
	} else {
		dev_dbg(&isys->adev->dev,
			"%s: adding request %u to the mdev queue\n", __func__,
			req->id);

		list_add(&ireq->head, &isys->requests);
	}

out_mutex_unlock:
	mutex_unlock(&isys->stream_mutex);

	return rval;

out_list_empty:
	spin_unlock_irqrestore(&ireq->lock, flags);

	return rval;
}

struct vb2_ops intel_ipu4_isys_queue_ops = {
	.queue_setup = queue_setup,
	.wait_prepare = intel_ipu4_isys_queue_unlock,
	.wait_finish = intel_ipu4_isys_queue_lock,
	.buf_init = buf_init,
	.buf_prepare = buf_prepare,
	.buf_finish = buf_finish,
	.buf_cleanup = buf_cleanup,
	.start_streaming = start_streaming,
	.stop_streaming = stop_streaming,
	.buf_queue = buf_queue,
};

int intel_ipu4_isys_queue_init(struct intel_ipu4_isys_queue *aq)
{
	struct intel_ipu4_isys *isys = intel_ipu4_isys_queue_to_video(aq)->isys;
	int rval;

	if (!aq->vbq.io_modes)
		aq->vbq.io_modes = VB2_USERPTR | VB2_MMAP | VB2_DMABUF;
	aq->vbq.drv_priv = aq;
#ifdef MEDIA_IOC_REQUEST_CMD
	aq->vbq.allow_requests = true;
#endif
	aq->vbq.ops = &intel_ipu4_isys_queue_ops;
	aq->vbq.mem_ops = &vb2_dma_contig_memops;
	aq->vbq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	rval = vb2_queue_init(&aq->vbq);
	if (rval)
		return rval;

	aq->ctx = vb2_dma_contig_init_ctx(&isys->adev->dev);
	if (IS_ERR(aq->ctx)) {
		vb2_queue_release(&aq->vbq);
		return PTR_ERR(aq->ctx);
	}

	spin_lock_init(&aq->lock);
	INIT_LIST_HEAD(&aq->active);
	INIT_LIST_HEAD(&aq->incoming);

	return 0;
}

void intel_ipu4_isys_queue_cleanup(struct intel_ipu4_isys_queue *aq)
{
	if (IS_ERR_OR_NULL(aq->ctx))
		return;

	vb2_dma_contig_cleanup_ctx(aq->ctx);
	vb2_queue_release(&aq->vbq);

	aq->ctx = NULL;
}
