/*
 * Copyright (c) 2014--2016 Intel Corporation.
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

#include <linux/device.h>
#include <linux/module.h>

#include <media/intel-ipu4-isys.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>

#include "intel-ipu4.h"
#include "intel-ipu4-bus.h"
#include "intel-ipu4-isysapi-fw-types.h"
#include "intel-ipu4-isys.h"
#include "intel-ipu4-isys-csi2-be.h"
#include "intel-ipu4-isys-subdev.h"
#include "intel-ipu4-isys-video.h"

/*
 * Raw bayer format pixel order MUST BE MAINTAINED in groups of four codes.
 * Otherwise pixel order calculation below WILL BREAK!
 */
static const uint32_t csi2_be_soc_supported_codes_pad[] = {
	MEDIA_BUS_FMT_RGB565_1X16,
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_SBGGR14_1X14,
	MEDIA_BUS_FMT_SGBRG14_1X14,
	MEDIA_BUS_FMT_SGRBG14_1X14,
	MEDIA_BUS_FMT_SRGGB14_1X14,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	0,
};

static const uint32_t *csi2_be_soc_supported_codes[] = {
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
	csi2_be_soc_supported_codes_pad,
};

static struct v4l2_subdev_internal_ops csi2_be_soc_sd_internal_ops = {
	.open = intel_ipu4_isys_subdev_open,
	.close = intel_ipu4_isys_subdev_close,
};

static const struct v4l2_subdev_core_ops csi2_be_soc_sd_core_ops = {
};

static int set_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static const struct v4l2_subdev_video_ops csi2_be_soc_sd_video_ops = {
	.s_stream = set_stream,
};

static int __subdev_link_validate(
	struct v4l2_subdev *sd, struct media_link *link,
	struct v4l2_subdev_format *source_fmt,
	struct v4l2_subdev_format *sink_fmt)
{
	struct intel_ipu4_isys_pipeline *ip =
		container_of(sd->entity.pipe,
			     struct intel_ipu4_isys_pipeline, pipe);

	ip->csi2_be_soc = to_intel_ipu4_isys_csi2_be_soc(sd);
	return intel_ipu4_isys_subdev_link_validate(sd, link, source_fmt,
			sink_fmt);
}

static int ipu4_isys_csi2_be_soc_set_sel(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct intel_ipu4_isys_subdev *asd = to_intel_ipu4_isys_subdev(sd);
	struct media_pad *pad = &asd->sd.entity.pads[sel->pad];

	if (sel->target == V4L2_SEL_TGT_CROP &&
	    pad->flags & MEDIA_PAD_FL_SOURCE &&
	    asd->valid_tgts[sel->pad].crop) {
		struct v4l2_rect *r;
		unsigned int sink_pad;
		int i;

		for (i = 0; i < asd->nstreams; i++) {
			if (!asd->route[i].flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE)
				continue;
			if (asd->route[i].source == sel->pad) {
				sink_pad = asd->route[i].sink;
				break;
			}
		}

		if (i == asd->nstreams) {
			dev_dbg(&asd->isys->adev->dev, "No sink pad routed.\n");
			return -EINVAL;
		}
		r = __intel_ipu4_isys_get_selection(
			sd, cfg, sel->target, sink_pad, sel->which);

		/* Cropping is not supported by SoC BE.
		 * Only horizontal padding is allowed.
		 */
		sel->r.top = r->top;
		sel->r.left = r->left;
		sel->r.width = clamp(sel->r.width, r->width,
				     INTEL_IPU4_ISYS_MAX_WIDTH);
		sel->r.height = r->height;

		*__intel_ipu4_isys_get_selection(sd, cfg, sel->target, sel->pad,
						sel->which) = sel->r;
		intel_ipu4_isys_subdev_fmt_propagate(
			sd, cfg, NULL, &sel->r,
			INTEL_IPU4_ISYS_SUBDEV_PROP_TGT_SOURCE_CROP,
			sel->pad, sel->which);
		return 0;
	}
	return -EINVAL;
}

static const struct v4l2_subdev_pad_ops csi2_be_soc_sd_pad_ops = {
	.link_validate = __subdev_link_validate,
	.get_fmt = intel_ipu4_isys_subdev_get_ffmt,
	.set_fmt = intel_ipu4_isys_subdev_set_ffmt,
	.get_selection = intel_ipu4_isys_subdev_get_sel,
	.set_selection = ipu4_isys_csi2_be_soc_set_sel,
	.enum_mbus_code = intel_ipu4_isys_subdev_enum_mbus_code,
	.set_routing = intel_ipu4_isys_subdev_set_routing,
	.get_routing = intel_ipu4_isys_subdev_get_routing,
};

static struct v4l2_subdev_ops csi2_be_soc_sd_ops = {
	.core = &csi2_be_soc_sd_core_ops,
	.video = &csi2_be_soc_sd_video_ops,
	.pad = &csi2_be_soc_sd_pad_ops,
};

static struct media_entity_operations csi2_be_soc_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.has_route = intel_ipu4_isys_subdev_has_route,
};

static void csi2_be_soc_set_ffmt(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	struct v4l2_subdev_fh *cfg,
#else
	struct v4l2_subdev_pad_config *cfg,
#endif
	struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *ffmt =
		__intel_ipu4_isys_get_ffmt(sd, cfg, fmt->pad, fmt->stream,
					   fmt->which);
	if (sd->entity.pads[fmt->pad].flags & MEDIA_PAD_FL_SINK) {
		if (fmt->format.field != V4L2_FIELD_ALTERNATE)
			fmt->format.field = V4L2_FIELD_NONE;
		*ffmt = fmt->format;

		intel_ipu4_isys_subdev_fmt_propagate(sd, cfg, &fmt->format,
			NULL, INTEL_IPU4_ISYS_SUBDEV_PROP_TGT_SINK_FMT,
			fmt->pad, fmt->which);
	} else if (sd->entity.pads[fmt->pad].flags & MEDIA_PAD_FL_SOURCE) {
		struct intel_ipu4_isys_subdev *asd =
					to_intel_ipu4_isys_subdev(sd);
		unsigned int sink_pad = 0;
		struct v4l2_mbus_framefmt *sink_ffmt;
		struct v4l2_rect *r;
		int i;

		for (i = 0; i < asd->nsinks; i++)
			if (media_entity_has_route(&sd->entity, fmt->pad, i))
				break;
		if (i != asd->nsinks)
			sink_pad = i;

		sink_ffmt = __intel_ipu4_isys_get_ffmt(sd, cfg, sink_pad,
				fmt->stream, fmt->which);
		r = __intel_ipu4_isys_get_selection(
				sd, cfg, V4L2_SEL_TGT_CROP,
				fmt->pad, fmt->which);

		ffmt->width = r->width;
		ffmt->height = r->height;
		ffmt->code = sink_ffmt->code;
		ffmt->field = sink_ffmt->field;
	}
}

void intel_ipu4_isys_csi2_be_soc_cleanup(
				struct intel_ipu4_isys_csi2_be_soc *csi2_be_soc)
{
	int i;

	v4l2_device_unregister_subdev(&csi2_be_soc->asd.sd);
	intel_ipu4_isys_subdev_cleanup(&csi2_be_soc->asd);
	for (i = 0; i < NR_OF_CSI2_BE_SOC_STREAMS; i++)
		intel_ipu4_isys_video_cleanup(&csi2_be_soc->av[i]);
}

int intel_ipu4_isys_csi2_be_soc_init(
				struct intel_ipu4_isys_csi2_be_soc *csi2_be_soc,
				struct intel_ipu4_isys *isys)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = CSI2_BE_SOC_PAD_SINK(0),
		.format = {
			.width = 4096,
			.height = 3072,
		},
	};
	int rval, i;

	csi2_be_soc->asd.sd.entity.ops = &csi2_be_soc_entity_ops;
	csi2_be_soc->asd.isys = isys;

	rval = intel_ipu4_isys_subdev_init(&csi2_be_soc->asd,
			&csi2_be_soc_sd_ops, 0,
			NR_OF_CSI2_BE_SOC_PADS, NR_OF_CSI2_BE_SOC_STREAMS,
			NR_OF_CSI2_BE_SOC_SOURCE_PADS,
			NR_OF_CSI2_BE_SOC_SINK_PADS, 0);
	if (rval)
		goto fail;

	for (i = CSI2_BE_SOC_PAD_SINK(0); i < NR_OF_CSI2_BE_SOC_SINK_PADS; i++)
		csi2_be_soc->asd.pad[i].flags = MEDIA_PAD_FL_SINK;

	for (i = CSI2_BE_SOC_PAD_SOURCE(0);
	     i < NR_OF_CSI2_BE_SOC_SOURCE_PADS + CSI2_BE_SOC_PAD_SOURCE(0);
	     i++) {
		csi2_be_soc->asd.pad[i].flags = MEDIA_PAD_FL_SOURCE;
		csi2_be_soc->asd.valid_tgts[i].crop = true;
	}

	BUILD_BUG_ON(ARRAY_SIZE(csi2_be_soc_supported_codes) !=
						NR_OF_CSI2_BE_SOC_PADS);
	csi2_be_soc->asd.supported_codes = csi2_be_soc_supported_codes;
	csi2_be_soc->asd.be_mode = INTEL_IPU4_BE_SOC;
	csi2_be_soc->asd.isl_mode = INTEL_IPU4_ISL_OFF;
	csi2_be_soc->asd.set_ffmt = csi2_be_soc_set_ffmt;

	for (i = CSI2_BE_SOC_PAD_SINK(0); i < NR_OF_CSI2_BE_SOC_SINK_PADS;
	     i++) {
		fmt.pad = CSI2_BE_SOC_PAD_SINK(i);
		intel_ipu4_isys_subdev_set_ffmt(&csi2_be_soc->asd.sd,
		    NULL, &fmt);
	}

	intel_ipu4_isys_subdev_set_ffmt(&csi2_be_soc->asd.sd, NULL, &fmt);
	csi2_be_soc->asd.sd.internal_ops = &csi2_be_soc_sd_internal_ops;

	snprintf(csi2_be_soc->asd.sd.name, sizeof(csi2_be_soc->asd.sd.name),
		 INTEL_IPU4_ISYS_ENTITY_PREFIX " CSI2 BE SOC");

	v4l2_set_subdevdata(&csi2_be_soc->asd.sd, &csi2_be_soc->asd);

	mutex_lock(&csi2_be_soc->asd.mutex);
	rval = v4l2_device_register_subdev(&isys->v4l2_dev,
					   &csi2_be_soc->asd.sd);
	if (rval) {
		dev_info(&isys->adev->dev, "can't register v4l2 subdev\n");
		goto fail;
	}

	/* create default route information */
	for (i = 0; i < NR_OF_CSI2_BE_SOC_STREAMS; i++) {
		csi2_be_soc->asd.route[i].sink = CSI2_BE_SOC_PAD_SINK(i);
		csi2_be_soc->asd.route[i].source = CSI2_BE_SOC_PAD_SOURCE(i);
		csi2_be_soc->asd.route[i].flags = 0;
	}

	for (i = 0; i < NR_OF_CSI2_BE_SOC_SOURCE_PADS; i++) {
		csi2_be_soc->asd.stream[CSI2_BE_SOC_PAD_SINK(i)].stream_id[0]
			= 0;
		csi2_be_soc->asd.stream[CSI2_BE_SOC_PAD_SOURCE(i)].stream_id[0]
			= 0;
	}
	for (i = 0; i < NR_OF_CSI2_BE_SOC_STREAMS; i++) {
		csi2_be_soc->asd.route[i].flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE |
					V4L2_SUBDEV_ROUTE_FL_IMMUTABLE;
		bitmap_set(
		csi2_be_soc->asd.stream[CSI2_BE_SOC_PAD_SINK(i)].streams_stat,
		0, 1);
		bitmap_set(
		csi2_be_soc->asd.stream[CSI2_BE_SOC_PAD_SOURCE(i)].streams_stat,
		0, 1);
	}
	mutex_unlock(&csi2_be_soc->asd.mutex);
	for (i = 0; i < NR_OF_CSI2_BE_SOC_SOURCE_PADS; i++) {
		snprintf(csi2_be_soc->av[i].vdev.name,
			sizeof(csi2_be_soc->av[i].vdev.name),
			INTEL_IPU4_ISYS_ENTITY_PREFIX " BE SOC capture %d", i);
		csi2_be_soc->av[i].aq.css_pin_type =
			IPU_FW_ISYS_PIN_TYPE_RAW_SOC;
		csi2_be_soc->av[i].isys = isys;
		csi2_be_soc->av[i].pfmts = intel_ipu4_isys_pfmts_be_soc;

		csi2_be_soc->av[i].try_fmt_vid_mplane =
			intel_ipu4_isys_video_try_fmt_vid_mplane_default;
		csi2_be_soc->av[i].prepare_firmware_stream_cfg =
			intel_ipu4_isys_prepare_firmware_stream_cfg_default;
		csi2_be_soc->av[i].aq.buf_prepare =
			intel_ipu4_isys_buf_prepare;
		csi2_be_soc->av[i].aq.fill_frame_buff_set_pin =
		intel_ipu4_isys_buffer_list_to_ipu_fw_isys_frame_buff_set_pin;
		csi2_be_soc->av[i].aq.link_fmt_validate =
				intel_ipu4_isys_link_fmt_validate;
		csi2_be_soc->av[i].aq.vbq.buf_struct_size =
			sizeof(struct intel_ipu4_isys_video_buffer);

		rval = intel_ipu4_isys_video_init(&csi2_be_soc->av[i],
				&csi2_be_soc->asd.sd.entity,
				CSI2_BE_SOC_PAD_SOURCE(0) + i,
				MEDIA_PAD_FL_SINK, MEDIA_LNK_FL_DYNAMIC);
		if (rval) {
			dev_info(&isys->adev->dev, "can't init video node\n");
			goto fail;
		}
	}

	return 0;

fail:
	intel_ipu4_isys_csi2_be_soc_cleanup(csi2_be_soc);

	return rval;
}

void intel_ipu4_isys_csi2_be_soc_isr(
				struct intel_ipu4_isys_csi2_be_soc *csi2_be_soc)
{
}
