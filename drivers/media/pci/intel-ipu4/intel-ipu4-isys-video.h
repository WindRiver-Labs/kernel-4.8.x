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

#ifndef INTEL_IPU4_ISYS_VIDEO_H
#define INTEL_IPU4_ISYS_VIDEO_H

#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "intel-ipu4-isys-queue.h"

#define INTEL_IPU4_ISYS_OUTPUT_PINS 11
#define INTEL_IPU4_NUM_CAPTURE_DONE 2
#define INTEL_IPU4_ISYS_MAX_PARALLEL_SOF 2

struct intel_ipu4_isys;
struct ipu_fw_isys_stream_cfg_data;
struct intel_ipu4_isys_csi2_be_raw;
struct intel_ipu4_isys_csi2_be_soc;
struct ipu_fw_isys_stream_cfg_data_abi;

struct intel_ipu4_isys_pixelformat {
	uint32_t pixelformat;
	uint32_t bpp;
	uint32_t bpp_packed;
	uint32_t bpp_planar;
	uint32_t code;
	uint32_t css_pixelformat;
};

struct sequence_info {
	unsigned int sequence;
	u64 timestamp;
};

struct output_pin_data {
	void (*pin_ready)(struct intel_ipu4_isys_pipeline *ip,
			  struct ipu_fw_isys_resp_info_abi *info);
	struct intel_ipu4_isys_queue *aq;
};

struct intel_ipu4_isys_pipeline {
	struct media_pipeline pipe;
	struct media_pad *external;
	atomic_t sequence;
	unsigned int seq_index;
	struct sequence_info seq[INTEL_IPU4_ISYS_MAX_PARALLEL_SOF];
	int source; /* SSI stream source */
	int stream_handle; /* stream handle for CSS API */
	unsigned int nr_output_pins; /* How many firmware pins? */
	enum intel_ipu4_isl_mode isl_mode;
	struct intel_ipu4_isys_csi2_be *csi2_be;
	struct intel_ipu4_isys_csi2_be_soc *csi2_be_soc;
	struct intel_ipu4_isys_csi2 *csi2;
	/*
	 * Number of capture queues, write access serialised using struct
	 * intel_ipu4_isys.stream_mutex
	 */
	int nr_queues;
	int nr_streaming; /* Number of capture queues streaming */
	int streaming; /* Has streaming been really started? */
	struct list_head queues;
	struct completion stream_open_completion;
	struct completion stream_close_completion;
	struct completion stream_start_completion;
	struct completion stream_stop_completion;
	struct completion capture_ack_completion;
	struct intel_ipu4_isys *isys;

	void (*capture_done[INTEL_IPU4_NUM_CAPTURE_DONE])
		(struct intel_ipu4_isys_pipeline *ip,
		 struct ipu_fw_isys_resp_info_abi *resp);
	struct output_pin_data output_pins[INTEL_IPU4_ISYS_OUTPUT_PINS];
	bool has_sof;
	bool interlaced;
	int error;
	struct intel_ipu4_isys_private_buffer *short_packet_bufs;
	size_t short_packet_buffer_size;
	unsigned int num_short_packet_lines;
	unsigned int short_packet_output_pin;
	unsigned int cur_field;
	struct list_head short_packet_incoming;
	struct list_head short_packet_active;
	spinlock_t short_packet_queue_lock;
	struct list_head pending_interlaced_bufs;
	unsigned int short_packet_trace_index;
	unsigned int vc;
	unsigned int stream_id;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	struct media_entity_graph graph;
#endif
	struct media_entity_enum entity_enum;
};

#define to_intel_ipu4_isys_pipeline(__pipe)				\
	container_of((__pipe), struct intel_ipu4_isys_pipeline, pipe)

struct intel_ipu4_isys_video {
	/* Serialise access to other fields in the struct. */
	struct mutex mutex;
	struct media_pad pad;
	struct video_device vdev;
	struct v4l2_pix_format_mplane mpix;
	const struct intel_ipu4_isys_pixelformat *pfmts;
	const struct intel_ipu4_isys_pixelformat *pfmt;
	struct intel_ipu4_isys_queue aq;
	struct intel_ipu4_isys *isys;
	struct intel_ipu4_isys_pipeline ip;
	unsigned int streaming;
	bool packed;
	unsigned int line_header_length; /* bits */
	unsigned int line_footer_length; /* bits */
	const struct intel_ipu4_isys_pixelformat *(*try_fmt_vid_mplane)(
		struct intel_ipu4_isys_video *av,
		struct v4l2_pix_format_mplane *mpix);
	void (*prepare_firmware_stream_cfg)(
		struct intel_ipu4_isys_video *av,
		struct ipu_fw_isys_stream_cfg_data_abi *cfg);
};

#define intel_ipu4_isys_queue_to_video(__aq) \
	container_of(__aq, struct intel_ipu4_isys_video, aq)

extern const struct intel_ipu4_isys_pixelformat intel_ipu4_isys_pfmts[];
extern const struct intel_ipu4_isys_pixelformat intel_ipu5_isys_pfmts[];
extern const struct intel_ipu4_isys_pixelformat intel_ipu4_isys_pfmts_be_soc[];
extern const struct intel_ipu4_isys_pixelformat intel_ipu4_isys_pfmts_packed[];

const struct intel_ipu4_isys_pixelformat *intel_ipu4_isys_get_pixelformat(
	struct intel_ipu4_isys_video *av, uint32_t pixelformat);

int intel_ipu4_isys_vidioc_querycap(struct file *file, void *fh,
				    struct v4l2_capability *cap);

int intel_ipu4_isys_vidioc_enum_fmt(struct file *file, void *fh,
				      struct v4l2_fmtdesc *f);

const struct intel_ipu4_isys_pixelformat
*intel_ipu4_isys_video_try_fmt_vid_mplane_default(
	struct intel_ipu4_isys_video *av, struct v4l2_pix_format_mplane *mpix);

const struct intel_ipu4_isys_pixelformat
*intel_ipu4_isys_video_try_fmt_vid_mplane(
	struct intel_ipu4_isys_video *av, struct v4l2_pix_format_mplane *mpix,
	int store_csi2_header);

void intel_ipu4_isys_prepare_firmware_stream_cfg_default(
	struct intel_ipu4_isys_video *av,
	struct ipu_fw_isys_stream_cfg_data_abi *cfg);
int intel_ipu4_isys_video_prepare_streaming(struct intel_ipu4_isys_video *av,
					 unsigned int state);
int intel_ipu4_isys_video_set_streaming(struct intel_ipu4_isys_video *av,
				     unsigned int state,
				     struct intel_ipu4_isys_buffer_list *bl);
int intel_ipu4_isys_video_init(struct intel_ipu4_isys_video *av,
			       struct media_entity *source,
			       unsigned int source_pad, unsigned long pad_flags,
			       unsigned int flags);
void intel_ipu4_isys_video_cleanup(struct intel_ipu4_isys_video *av);
void intel_ipu4_isys_video_add_capture_done(
	struct intel_ipu4_isys_pipeline *ip,
	void (*capture_done)(struct intel_ipu4_isys_pipeline *ip,
			     struct ipu_fw_isys_resp_info_abi *resp));

#endif /* INTEL_IPU4_ISYS_VIDEO_H */
