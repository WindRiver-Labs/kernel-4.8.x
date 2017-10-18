/*
 * Copyright (c) 2006-2008 Intel Corporation
 * Copyright (c) 2007 Dave Airlie <airlied@linux.ie>
 * Copyright (c) 2008 Red Hat Inc.
 *
 * DRM core CRTC related functions
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 *
 * Authors:
 *      Keith Packard
 *	Eric Anholt <eric@anholt.net>
 *      Dave Airlie <airlied@linux.ie>
 *      Jesse Barnes <jesse.barnes@intel.com>
 */
#include <linux/ctype.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_atomic.h>
#include <drm/drm_auth.h>
#include <drm/drm_framebuffer.h>

#include "drm_crtc_internal.h"
#include "drm_internal.h"

/*
 * Global properties
 */
static const struct drm_prop_enum_list drm_plane_type_enum_list[] = {
	{ DRM_PLANE_TYPE_OVERLAY, "Overlay" },
	{ DRM_PLANE_TYPE_PRIMARY, "Primary" },
	{ DRM_PLANE_TYPE_CURSOR, "Cursor" },
};

/*
 * Optional properties
 */
/**
 * drm_crtc_force_disable - Forcibly turn off a CRTC
 * @crtc: CRTC to turn off
 *
 * Returns:
 * Zero on success, error code on failure.
 */
int drm_crtc_force_disable(struct drm_crtc *crtc)
{
	struct drm_mode_set set = {
		.crtc = crtc,
	};

	return drm_mode_set_config_internal(&set);
}
EXPORT_SYMBOL(drm_crtc_force_disable);

/**
 * drm_crtc_force_disable_all - Forcibly turn off all enabled CRTCs
 * @dev: DRM device whose CRTCs to turn off
 *
 * Drivers may want to call this on unload to ensure that all displays are
 * unlit and the GPU is in a consistent, low power state. Takes modeset locks.
 *
 * Returns:
 * Zero on success, error code on failure.
 */
int drm_crtc_force_disable_all(struct drm_device *dev)
{
	struct drm_crtc *crtc;
	int ret = 0;

	drm_modeset_lock_all(dev);
	drm_for_each_crtc(crtc, dev)
		if (crtc->enabled) {
			ret = drm_crtc_force_disable(crtc);
			if (ret)
				goto out;
		}
out:
	drm_modeset_unlock_all(dev);
	return ret;
}
EXPORT_SYMBOL(drm_crtc_force_disable_all);

DEFINE_WW_CLASS(crtc_ww_class);

static unsigned int drm_num_crtcs(struct drm_device *dev)
{
	unsigned int num = 0;
	struct drm_crtc *tmp;

	drm_for_each_crtc(tmp, dev) {
		num++;
	}

	return num;
}

static int drm_crtc_register_all(struct drm_device *dev)
{
	struct drm_crtc *crtc;
	int ret = 0;

	drm_for_each_crtc(crtc, dev) {
		if (crtc->funcs->late_register)
			ret = crtc->funcs->late_register(crtc);
		if (ret)
			return ret;
	}

	return 0;
}

static void drm_crtc_unregister_all(struct drm_device *dev)
{
	struct drm_crtc *crtc;

	drm_for_each_crtc(crtc, dev) {
		if (crtc->funcs->early_unregister)
			crtc->funcs->early_unregister(crtc);
	}
}

/**
 * drm_crtc_init_with_planes - Initialise a new CRTC object with
 *    specified primary and cursor planes.
 * @dev: DRM device
 * @crtc: CRTC object to init
 * @primary: Primary plane for CRTC
 * @cursor: Cursor plane for CRTC
 * @funcs: callbacks for the new CRTC
 * @name: printf style format string for the CRTC name, or NULL for default name
 *
 * Inits a new object created as base part of a driver crtc object.
 *
 * Returns:
 * Zero on success, error code on failure.
 */
int drm_crtc_init_with_planes(struct drm_device *dev, struct drm_crtc *crtc,
			      struct drm_plane *primary,
			      struct drm_plane *cursor,
			      const struct drm_crtc_funcs *funcs,
			      const char *name, ...)
{
	struct drm_mode_config *config = &dev->mode_config;
	int ret;

	WARN_ON(primary && primary->type != DRM_PLANE_TYPE_PRIMARY);
	WARN_ON(cursor && cursor->type != DRM_PLANE_TYPE_CURSOR);

	crtc->dev = dev;
	crtc->funcs = funcs;

	INIT_LIST_HEAD(&crtc->commit_list);
	spin_lock_init(&crtc->commit_lock);

	drm_modeset_lock_init(&crtc->mutex);
	ret = drm_mode_object_get(dev, &crtc->base, DRM_MODE_OBJECT_CRTC);
	if (ret)
		return ret;

	if (name) {
		va_list ap;

		va_start(ap, name);
		crtc->name = kvasprintf(GFP_KERNEL, name, ap);
		va_end(ap);
	} else {
		crtc->name = kasprintf(GFP_KERNEL, "crtc-%d",
				       drm_num_crtcs(dev));
	}
	if (!crtc->name) {
		drm_mode_object_unregister(dev, &crtc->base);
		return -ENOMEM;
	}

	crtc->base.properties = &crtc->properties;

	list_add_tail(&crtc->head, &config->crtc_list);
	crtc->index = config->num_crtc++;

	crtc->primary = primary;
	crtc->cursor = cursor;
	if (primary)
		primary->possible_crtcs = 1 << drm_crtc_index(crtc);
	if (cursor)
		cursor->possible_crtcs = 1 << drm_crtc_index(crtc);

	if (drm_core_check_feature(dev, DRIVER_ATOMIC)) {
		drm_object_attach_property(&crtc->base, config->prop_active, 0);
		drm_object_attach_property(&crtc->base, config->prop_mode_id, 0);
	}

	return 0;
}
EXPORT_SYMBOL(drm_crtc_init_with_planes);

/**
 * drm_crtc_cleanup - Clean up the core crtc usage
 * @crtc: CRTC to cleanup
 *
 * This function cleans up @crtc and removes it from the DRM mode setting
 * core. Note that the function does *not* free the crtc structure itself,
 * this is the responsibility of the caller.
 */
void drm_crtc_cleanup(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;

	/* Note that the crtc_list is considered to be static; should we
	 * remove the drm_crtc at runtime we would have to decrement all
	 * the indices on the drm_crtc after us in the crtc_list.
	 */

	kfree(crtc->gamma_store);
	crtc->gamma_store = NULL;

	drm_modeset_lock_fini(&crtc->mutex);

	drm_mode_object_unregister(dev, &crtc->base);
	list_del(&crtc->head);
	dev->mode_config.num_crtc--;

	WARN_ON(crtc->state && !crtc->funcs->atomic_destroy_state);
	if (crtc->state && crtc->funcs->atomic_destroy_state)
		crtc->funcs->atomic_destroy_state(crtc, crtc->state);

	kfree(crtc->name);

	memset(crtc, 0, sizeof(*crtc));
}
EXPORT_SYMBOL(drm_crtc_cleanup);

static unsigned int drm_num_planes(struct drm_device *dev)
{
	unsigned int num = 0;
	struct drm_plane *tmp;

	drm_for_each_plane(tmp, dev) {
		num++;
	}

	return num;
}

/**
 * drm_universal_plane_init - Initialize a new universal plane object
 * @dev: DRM device
 * @plane: plane object to init
 * @possible_crtcs: bitmask of possible CRTCs
 * @funcs: callbacks for the new plane
 * @formats: array of supported formats (DRM_FORMAT\_\*)
 * @format_count: number of elements in @formats
 * @type: type of plane (overlay, primary, cursor)
 * @name: printf style format string for the plane name, or NULL for default name
 *
 * Initializes a plane object of type @type.
 *
 * Returns:
 * Zero on success, error code on failure.
 */
int drm_universal_plane_init(struct drm_device *dev, struct drm_plane *plane,
			     unsigned long possible_crtcs,
			     const struct drm_plane_funcs *funcs,
			     const uint32_t *formats, unsigned int format_count,
			     enum drm_plane_type type,
			     const char *name, ...)
{
	struct drm_mode_config *config = &dev->mode_config;
	int ret;

	ret = drm_mode_object_get(dev, &plane->base, DRM_MODE_OBJECT_PLANE);
	if (ret)
		return ret;

	drm_modeset_lock_init(&plane->mutex);

	plane->base.properties = &plane->properties;
	plane->dev = dev;
	plane->funcs = funcs;
	plane->format_types = kmalloc_array(format_count, sizeof(uint32_t),
					    GFP_KERNEL);
	if (!plane->format_types) {
		DRM_DEBUG_KMS("out of memory when allocating plane\n");
		drm_mode_object_unregister(dev, &plane->base);
		return -ENOMEM;
	}

	if (name) {
		va_list ap;

		va_start(ap, name);
		plane->name = kvasprintf(GFP_KERNEL, name, ap);
		va_end(ap);
	} else {
		plane->name = kasprintf(GFP_KERNEL, "plane-%d",
					drm_num_planes(dev));
	}
	if (!plane->name) {
		kfree(plane->format_types);
		drm_mode_object_unregister(dev, &plane->base);
		return -ENOMEM;
	}

	memcpy(plane->format_types, formats, format_count * sizeof(uint32_t));
	plane->format_count = format_count;
	plane->possible_crtcs = possible_crtcs;
	plane->type = type;

	list_add_tail(&plane->head, &config->plane_list);
	plane->index = config->num_total_plane++;
	if (plane->type == DRM_PLANE_TYPE_OVERLAY)
		config->num_overlay_plane++;

	drm_object_attach_property(&plane->base,
				   config->plane_type_property,
				   plane->type);

	if (drm_core_check_feature(dev, DRIVER_ATOMIC)) {
		drm_object_attach_property(&plane->base, config->prop_fb_id, 0);
		drm_object_attach_property(&plane->base, config->prop_crtc_id, 0);
		drm_object_attach_property(&plane->base, config->prop_crtc_x, 0);
		drm_object_attach_property(&plane->base, config->prop_crtc_y, 0);
		drm_object_attach_property(&plane->base, config->prop_crtc_w, 0);
		drm_object_attach_property(&plane->base, config->prop_crtc_h, 0);
		drm_object_attach_property(&plane->base, config->prop_src_x, 0);
		drm_object_attach_property(&plane->base, config->prop_src_y, 0);
		drm_object_attach_property(&plane->base, config->prop_src_w, 0);
		drm_object_attach_property(&plane->base, config->prop_src_h, 0);
	}

	return 0;
}
EXPORT_SYMBOL(drm_universal_plane_init);

static int drm_plane_register_all(struct drm_device *dev)
{
	struct drm_plane *plane;
	int ret = 0;

	drm_for_each_plane(plane, dev) {
		if (plane->funcs->late_register)
			ret = plane->funcs->late_register(plane);
		if (ret)
			return ret;
	}

	return 0;
}

static void drm_plane_unregister_all(struct drm_device *dev)
{
	struct drm_plane *plane;

	drm_for_each_plane(plane, dev) {
		if (plane->funcs->early_unregister)
			plane->funcs->early_unregister(plane);
	}
}

/**
 * drm_plane_init - Initialize a legacy plane
 * @dev: DRM device
 * @plane: plane object to init
 * @possible_crtcs: bitmask of possible CRTCs
 * @funcs: callbacks for the new plane
 * @formats: array of supported formats (DRM_FORMAT\_\*)
 * @format_count: number of elements in @formats
 * @is_primary: plane type (primary vs overlay)
 *
 * Legacy API to initialize a DRM plane.
 *
 * New drivers should call drm_universal_plane_init() instead.
 *
 * Returns:
 * Zero on success, error code on failure.
 */
int drm_plane_init(struct drm_device *dev, struct drm_plane *plane,
		   unsigned long possible_crtcs,
		   const struct drm_plane_funcs *funcs,
		   const uint32_t *formats, unsigned int format_count,
		   bool is_primary)
{
	enum drm_plane_type type;

	type = is_primary ? DRM_PLANE_TYPE_PRIMARY : DRM_PLANE_TYPE_OVERLAY;
	return drm_universal_plane_init(dev, plane, possible_crtcs, funcs,
					formats, format_count, type, NULL);
}
EXPORT_SYMBOL(drm_plane_init);

/**
 * drm_plane_cleanup - Clean up the core plane usage
 * @plane: plane to cleanup
 *
 * This function cleans up @plane and removes it from the DRM mode setting
 * core. Note that the function does *not* free the plane structure itself,
 * this is the responsibility of the caller.
 */
void drm_plane_cleanup(struct drm_plane *plane)
{
	struct drm_device *dev = plane->dev;

	drm_modeset_lock_all(dev);
	kfree(plane->format_types);
	drm_mode_object_unregister(dev, &plane->base);

	BUG_ON(list_empty(&plane->head));

	/* Note that the plane_list is considered to be static; should we
	 * remove the drm_plane at runtime we would have to decrement all
	 * the indices on the drm_plane after us in the plane_list.
	 */

	list_del(&plane->head);
	dev->mode_config.num_total_plane--;
	if (plane->type == DRM_PLANE_TYPE_OVERLAY)
		dev->mode_config.num_overlay_plane--;
	drm_modeset_unlock_all(dev);

	WARN_ON(plane->state && !plane->funcs->atomic_destroy_state);
	if (plane->state && plane->funcs->atomic_destroy_state)
		plane->funcs->atomic_destroy_state(plane, plane->state);

	kfree(plane->name);

	memset(plane, 0, sizeof(*plane));
}
EXPORT_SYMBOL(drm_plane_cleanup);

/**
 * drm_plane_from_index - find the registered plane at an index
 * @dev: DRM device
 * @idx: index of registered plane to find for
 *
 * Given a plane index, return the registered plane from DRM device's
 * list of planes with matching index.
 */
struct drm_plane *
drm_plane_from_index(struct drm_device *dev, int idx)
{
	struct drm_plane *plane;

	drm_for_each_plane(plane, dev)
		if (idx == plane->index)
			return plane;

	return NULL;
}
EXPORT_SYMBOL(drm_plane_from_index);

/**
 * drm_plane_force_disable - Forcibly disable a plane
 * @plane: plane to disable
 *
 * Forces the plane to be disabled.
 *
 * Used when the plane's current framebuffer is destroyed,
 * and when restoring fbdev mode.
 */
void drm_plane_force_disable(struct drm_plane *plane)
{
	int ret;

	if (!plane->fb)
		return;

	plane->old_fb = plane->fb;
	ret = plane->funcs->disable_plane(plane);
	if (ret) {
		DRM_ERROR("failed to disable plane with busy fb\n");
		plane->old_fb = NULL;
		return;
	}
	/* disconnect the plane from the fb and crtc: */
	drm_framebuffer_unreference(plane->old_fb);
	plane->old_fb = NULL;
	plane->fb = NULL;
	plane->crtc = NULL;
}
EXPORT_SYMBOL(drm_plane_force_disable);

int drm_modeset_register_all(struct drm_device *dev)
{
	int ret;

	ret = drm_plane_register_all(dev);
	if (ret)
		goto err_plane;

	ret = drm_crtc_register_all(dev);
	if  (ret)
		goto err_crtc;

	ret = drm_encoder_register_all(dev);
	if (ret)
		goto err_encoder;

	ret = drm_connector_register_all(dev);
	if (ret)
		goto err_connector;

	return 0;

err_connector:
	drm_encoder_unregister_all(dev);
err_encoder:
	drm_crtc_unregister_all(dev);
err_crtc:
	drm_plane_unregister_all(dev);
err_plane:
	return ret;
}

void drm_modeset_unregister_all(struct drm_device *dev)
{
	drm_connector_unregister_all(dev);
	drm_encoder_unregister_all(dev);
	drm_crtc_unregister_all(dev);
	drm_plane_unregister_all(dev);
}

static int drm_mode_create_standard_properties(struct drm_device *dev)
{
	struct drm_property *prop;
	int ret;

	ret = drm_connector_create_standard_properties(dev);
	if (ret)
		return ret;

	prop = drm_property_create_enum(dev, DRM_MODE_PROP_IMMUTABLE,
					"type", drm_plane_type_enum_list,
					ARRAY_SIZE(drm_plane_type_enum_list));
	if (!prop)
		return -ENOMEM;
	dev->mode_config.plane_type_property = prop;

	prop = drm_property_create_range(dev, DRM_MODE_PROP_ATOMIC,
			"SRC_X", 0, UINT_MAX);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_src_x = prop;

	prop = drm_property_create_range(dev, DRM_MODE_PROP_ATOMIC,
			"SRC_Y", 0, UINT_MAX);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_src_y = prop;

	prop = drm_property_create_range(dev, DRM_MODE_PROP_ATOMIC,
			"SRC_W", 0, UINT_MAX);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_src_w = prop;

	prop = drm_property_create_range(dev, DRM_MODE_PROP_ATOMIC,
			"SRC_H", 0, UINT_MAX);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_src_h = prop;

	prop = drm_property_create_signed_range(dev, DRM_MODE_PROP_ATOMIC,
			"CRTC_X", INT_MIN, INT_MAX);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_crtc_x = prop;

	prop = drm_property_create_signed_range(dev, DRM_MODE_PROP_ATOMIC,
			"CRTC_Y", INT_MIN, INT_MAX);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_crtc_y = prop;

	prop = drm_property_create_range(dev, DRM_MODE_PROP_ATOMIC,
			"CRTC_W", 0, INT_MAX);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_crtc_w = prop;

	prop = drm_property_create_range(dev, DRM_MODE_PROP_ATOMIC,
			"CRTC_H", 0, INT_MAX);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_crtc_h = prop;

	prop = drm_property_create_object(dev, DRM_MODE_PROP_ATOMIC,
			"FB_ID", DRM_MODE_OBJECT_FB);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_fb_id = prop;

	prop = drm_property_create_object(dev, DRM_MODE_PROP_ATOMIC,
			"CRTC_ID", DRM_MODE_OBJECT_CRTC);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_crtc_id = prop;

	prop = drm_property_create_bool(dev, DRM_MODE_PROP_ATOMIC,
			"ACTIVE");
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_active = prop;

	prop = drm_property_create(dev,
			DRM_MODE_PROP_ATOMIC | DRM_MODE_PROP_BLOB,
			"MODE_ID", 0);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.prop_mode_id = prop;

	prop = drm_property_create(dev,
			DRM_MODE_PROP_BLOB,
			"DEGAMMA_LUT", 0);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.degamma_lut_property = prop;

	prop = drm_property_create_range(dev,
			DRM_MODE_PROP_IMMUTABLE,
			"DEGAMMA_LUT_SIZE", 0, UINT_MAX);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.degamma_lut_size_property = prop;

	prop = drm_property_create(dev,
			DRM_MODE_PROP_BLOB,
			"CTM", 0);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.ctm_property = prop;

	prop = drm_property_create(dev,
			DRM_MODE_PROP_BLOB,
			"GAMMA_LUT", 0);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.gamma_lut_property = prop;

	prop = drm_property_create_range(dev,
			DRM_MODE_PROP_IMMUTABLE,
			"GAMMA_LUT_SIZE", 0, UINT_MAX);
	if (!prop)
		return -ENOMEM;
	dev->mode_config.gamma_lut_size_property = prop;

	return 0;
}

/**
 * drm_mode_getresources - get graphics configuration
 * @dev: drm device for the ioctl
 * @data: data pointer for the ioctl
 * @file_priv: drm file for the ioctl call
 *
 * Construct a set of configuration description structures and return
 * them to the user, including CRTC, connector and framebuffer configuration.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_getresources(struct drm_device *dev, void *data,
			  struct drm_file *file_priv)
{
	struct drm_mode_card_res *card_res = data;
	struct list_head *lh;
	struct drm_framebuffer *fb;
	struct drm_connector *connector;
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	int ret = 0;
	int connector_count = 0;
	int crtc_count = 0;
	int fb_count = 0;
	int encoder_count = 0;
	int copied = 0;
	uint32_t __user *fb_id;
	uint32_t __user *crtc_id;
	uint32_t __user *connector_id;
	uint32_t __user *encoder_id;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;


	mutex_lock(&file_priv->fbs_lock);
	/*
	 * For the non-control nodes we need to limit the list of resources
	 * by IDs in the group list for this node
	 */
	list_for_each(lh, &file_priv->fbs)
		fb_count++;

	/* handle this in 4 parts */
	/* FBs */
	if (card_res->count_fbs >= fb_count) {
		copied = 0;
		fb_id = (uint32_t __user *)(unsigned long)card_res->fb_id_ptr;
		list_for_each_entry(fb, &file_priv->fbs, filp_head) {
			if (put_user(fb->base.id, fb_id + copied)) {
				mutex_unlock(&file_priv->fbs_lock);
				return -EFAULT;
			}
			copied++;
		}
	}
	card_res->count_fbs = fb_count;
	mutex_unlock(&file_priv->fbs_lock);

	/* mode_config.mutex protects the connector list against e.g. DP MST
	 * connector hot-adding. CRTC/Plane lists are invariant. */
	mutex_lock(&dev->mode_config.mutex);
	drm_for_each_crtc(crtc, dev)
		crtc_count++;

	drm_for_each_connector(connector, dev)
		connector_count++;

	drm_for_each_encoder(encoder, dev)
		encoder_count++;

	card_res->max_height = dev->mode_config.max_height;
	card_res->min_height = dev->mode_config.min_height;
	card_res->max_width = dev->mode_config.max_width;
	card_res->min_width = dev->mode_config.min_width;

	/* CRTCs */
	if (card_res->count_crtcs >= crtc_count) {
		copied = 0;
		crtc_id = (uint32_t __user *)(unsigned long)card_res->crtc_id_ptr;
		drm_for_each_crtc(crtc, dev) {
			if (put_user(crtc->base.id, crtc_id + copied)) {
				ret = -EFAULT;
				goto out;
			}
			copied++;
		}
	}
	card_res->count_crtcs = crtc_count;

	/* Encoders */
	if (card_res->count_encoders >= encoder_count) {
		copied = 0;
		encoder_id = (uint32_t __user *)(unsigned long)card_res->encoder_id_ptr;
		drm_for_each_encoder(encoder, dev) {
			if (put_user(encoder->base.id, encoder_id +
				     copied)) {
				ret = -EFAULT;
				goto out;
			}
			copied++;
		}
	}
	card_res->count_encoders = encoder_count;

	/* Connectors */
	if (card_res->count_connectors >= connector_count) {
		copied = 0;
		connector_id = (uint32_t __user *)(unsigned long)card_res->connector_id_ptr;
		drm_for_each_connector(connector, dev) {
			if (put_user(connector->base.id,
				     connector_id + copied)) {
				ret = -EFAULT;
				goto out;
			}
			copied++;
		}
	}
	card_res->count_connectors = connector_count;

out:
	mutex_unlock(&dev->mode_config.mutex);
	return ret;
}

/**
 * drm_mode_getcrtc - get CRTC configuration
 * @dev: drm device for the ioctl
 * @data: data pointer for the ioctl
 * @file_priv: drm file for the ioctl call
 *
 * Construct a CRTC configuration structure to return to the user.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_getcrtc(struct drm_device *dev,
		     void *data, struct drm_file *file_priv)
{
	struct drm_mode_crtc *crtc_resp = data;
	struct drm_crtc *crtc;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	crtc = drm_crtc_find(dev, crtc_resp->crtc_id);
	if (!crtc)
		return -ENOENT;

	drm_modeset_lock_crtc(crtc, crtc->primary);
	crtc_resp->gamma_size = crtc->gamma_size;
	if (crtc->primary->fb)
		crtc_resp->fb_id = crtc->primary->fb->base.id;
	else
		crtc_resp->fb_id = 0;

	if (crtc->state) {
		crtc_resp->x = crtc->primary->state->src_x >> 16;
		crtc_resp->y = crtc->primary->state->src_y >> 16;
		if (crtc->state->enable) {
			drm_mode_convert_to_umode(&crtc_resp->mode, &crtc->state->mode);
			crtc_resp->mode_valid = 1;

		} else {
			crtc_resp->mode_valid = 0;
		}
	} else {
		crtc_resp->x = crtc->x;
		crtc_resp->y = crtc->y;
		if (crtc->enabled) {
			drm_mode_convert_to_umode(&crtc_resp->mode, &crtc->mode);
			crtc_resp->mode_valid = 1;

		} else {
			crtc_resp->mode_valid = 0;
		}
	}
	drm_modeset_unlock_crtc(crtc);

	return 0;
}

/**
 * drm_mode_getplane_res - enumerate all plane resources
 * @dev: DRM device
 * @data: ioctl data
 * @file_priv: DRM file info
 *
 * Construct a list of plane ids to return to the user.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_getplane_res(struct drm_device *dev, void *data,
			  struct drm_file *file_priv)
{
	struct drm_mode_get_plane_res *plane_resp = data;
	struct drm_mode_config *config;
	struct drm_plane *plane;
	uint32_t __user *plane_ptr;
	int copied = 0;
	unsigned num_planes;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	config = &dev->mode_config;

	if (file_priv->universal_planes)
		num_planes = config->num_total_plane;
	else
		num_planes = config->num_overlay_plane;

	/*
	 * This ioctl is called twice, once to determine how much space is
	 * needed, and the 2nd time to fill it.
	 */
	if (num_planes &&
	    (plane_resp->count_planes >= num_planes)) {
		plane_ptr = (uint32_t __user *)(unsigned long)plane_resp->plane_id_ptr;

		/* Plane lists are invariant, no locking needed. */
		drm_for_each_plane(plane, dev) {
			/*
			 * Unless userspace set the 'universal planes'
			 * capability bit, only advertise overlays.
			 */
			if (plane->type != DRM_PLANE_TYPE_OVERLAY &&
			    !file_priv->universal_planes)
				continue;

			if (put_user(plane->base.id, plane_ptr + copied))
				return -EFAULT;
			copied++;
		}
	}
	plane_resp->count_planes = num_planes;

	return 0;
}

/**
 * drm_mode_getplane - get plane configuration
 * @dev: DRM device
 * @data: ioctl data
 * @file_priv: DRM file info
 *
 * Construct a plane configuration structure to return to the user.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_getplane(struct drm_device *dev, void *data,
		      struct drm_file *file_priv)
{
	struct drm_mode_get_plane *plane_resp = data;
	struct drm_plane *plane;
	uint32_t __user *format_ptr;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	plane = drm_plane_find(dev, plane_resp->plane_id);
	if (!plane)
		return -ENOENT;

	drm_modeset_lock(&plane->mutex, NULL);
	if (plane->crtc)
		plane_resp->crtc_id = plane->crtc->base.id;
	else
		plane_resp->crtc_id = 0;

	if (plane->fb)
		plane_resp->fb_id = plane->fb->base.id;
	else
		plane_resp->fb_id = 0;
	drm_modeset_unlock(&plane->mutex);

	plane_resp->plane_id = plane->base.id;
	plane_resp->possible_crtcs = plane->possible_crtcs;
	plane_resp->gamma_size = 0;

	/*
	 * This ioctl is called twice, once to determine how much space is
	 * needed, and the 2nd time to fill it.
	 */
	if (plane->format_count &&
	    (plane_resp->count_format_types >= plane->format_count)) {
		format_ptr = (uint32_t __user *)(unsigned long)plane_resp->format_type_ptr;
		if (copy_to_user(format_ptr,
				 plane->format_types,
				 sizeof(uint32_t) * plane->format_count)) {
			return -EFAULT;
		}
	}
	plane_resp->count_format_types = plane->format_count;

	return 0;
}

/**
 * drm_plane_check_pixel_format - Check if the plane supports the pixel format
 * @plane: plane to check for format support
 * @format: the pixel format
 *
 * Returns:
 * Zero of @plane has @format in its list of supported pixel formats, -EINVAL
 * otherwise.
 */
int drm_plane_check_pixel_format(const struct drm_plane *plane, u32 format)
{
	unsigned int i;

	for (i = 0; i < plane->format_count; i++) {
		if (format == plane->format_types[i])
			return 0;
	}

	return -EINVAL;
}

static int check_src_coords(uint32_t src_x, uint32_t src_y,
			    uint32_t src_w, uint32_t src_h,
			    const struct drm_framebuffer *fb)
{
	unsigned int fb_width, fb_height;

	fb_width = fb->width << 16;
	fb_height = fb->height << 16;

	/* Make sure source coordinates are inside the fb. */
	if (src_w > fb_width ||
	    src_x > fb_width - src_w ||
	    src_h > fb_height ||
	    src_y > fb_height - src_h) {
		DRM_DEBUG_KMS("Invalid source coordinates "
			      "%u.%06ux%u.%06u+%u.%06u+%u.%06u\n",
			      src_w >> 16, ((src_w & 0xffff) * 15625) >> 10,
			      src_h >> 16, ((src_h & 0xffff) * 15625) >> 10,
			      src_x >> 16, ((src_x & 0xffff) * 15625) >> 10,
			      src_y >> 16, ((src_y & 0xffff) * 15625) >> 10);
		return -ENOSPC;
	}

	return 0;
}

/*
 * setplane_internal - setplane handler for internal callers
 *
 * Note that we assume an extra reference has already been taken on fb.  If the
 * update fails, this reference will be dropped before return; if it succeeds,
 * the previous framebuffer (if any) will be unreferenced instead.
 *
 * src_{x,y,w,h} are provided in 16.16 fixed point format
 */
static int __setplane_internal(struct drm_plane *plane,
			       struct drm_crtc *crtc,
			       struct drm_framebuffer *fb,
			       int32_t crtc_x, int32_t crtc_y,
			       uint32_t crtc_w, uint32_t crtc_h,
			       /* src_{x,y,w,h} values are 16.16 fixed point */
			       uint32_t src_x, uint32_t src_y,
			       uint32_t src_w, uint32_t src_h)
{
	int ret = 0;

	/* No fb means shut it down */
	if (!fb) {
		plane->old_fb = plane->fb;
		ret = plane->funcs->disable_plane(plane);
		if (!ret) {
			plane->crtc = NULL;
			plane->fb = NULL;
		} else {
			plane->old_fb = NULL;
		}
		goto out;
	}

	/* Check whether this plane is usable on this CRTC */
	if (!(plane->possible_crtcs & drm_crtc_mask(crtc))) {
		DRM_DEBUG_KMS("Invalid crtc for plane\n");
		ret = -EINVAL;
		goto out;
	}

	/* Check whether this plane supports the fb pixel format. */
	ret = drm_plane_check_pixel_format(plane, fb->pixel_format);
	if (ret) {
		char *format_name = drm_get_format_name(fb->pixel_format);
		DRM_DEBUG_KMS("Invalid pixel format %s\n", format_name);
		kfree(format_name);
		goto out;
	}

	/* Give drivers some help against integer overflows */
	if (crtc_w > INT_MAX ||
	    crtc_x > INT_MAX - (int32_t) crtc_w ||
	    crtc_h > INT_MAX ||
	    crtc_y > INT_MAX - (int32_t) crtc_h) {
		DRM_DEBUG_KMS("Invalid CRTC coordinates %ux%u+%d+%d\n",
			      crtc_w, crtc_h, crtc_x, crtc_y);
		ret = -ERANGE;
		goto out;
	}

	ret = check_src_coords(src_x, src_y, src_w, src_h, fb);
	if (ret)
		goto out;

	plane->old_fb = plane->fb;
	ret = plane->funcs->update_plane(plane, crtc, fb,
					 crtc_x, crtc_y, crtc_w, crtc_h,
					 src_x, src_y, src_w, src_h);
	if (!ret) {
		plane->crtc = crtc;
		plane->fb = fb;
		fb = NULL;
	} else {
		plane->old_fb = NULL;
	}

out:
	if (fb)
		drm_framebuffer_unreference(fb);
	if (plane->old_fb)
		drm_framebuffer_unreference(plane->old_fb);
	plane->old_fb = NULL;

	return ret;
}

static int setplane_internal(struct drm_plane *plane,
			     struct drm_crtc *crtc,
			     struct drm_framebuffer *fb,
			     int32_t crtc_x, int32_t crtc_y,
			     uint32_t crtc_w, uint32_t crtc_h,
			     /* src_{x,y,w,h} values are 16.16 fixed point */
			     uint32_t src_x, uint32_t src_y,
			     uint32_t src_w, uint32_t src_h)
{
	int ret;

	drm_modeset_lock_all(plane->dev);
	ret = __setplane_internal(plane, crtc, fb,
				  crtc_x, crtc_y, crtc_w, crtc_h,
				  src_x, src_y, src_w, src_h);
	drm_modeset_unlock_all(plane->dev);

	return ret;
}

/**
 * drm_mode_setplane - configure a plane's configuration
 * @dev: DRM device
 * @data: ioctl data*
 * @file_priv: DRM file info
 *
 * Set plane configuration, including placement, fb, scaling, and other factors.
 * Or pass a NULL fb to disable (planes may be disabled without providing a
 * valid crtc).
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_setplane(struct drm_device *dev, void *data,
		      struct drm_file *file_priv)
{
	struct drm_mode_set_plane *plane_req = data;
	struct drm_plane *plane;
	struct drm_crtc *crtc = NULL;
	struct drm_framebuffer *fb = NULL;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	/*
	 * First, find the plane, crtc, and fb objects.  If not available,
	 * we don't bother to call the driver.
	 */
	plane = drm_plane_find(dev, plane_req->plane_id);
	if (!plane) {
		DRM_DEBUG_KMS("Unknown plane ID %d\n",
			      plane_req->plane_id);
		return -ENOENT;
	}

	if (plane_req->fb_id) {
		fb = drm_framebuffer_lookup(dev, plane_req->fb_id);
		if (!fb) {
			DRM_DEBUG_KMS("Unknown framebuffer ID %d\n",
				      plane_req->fb_id);
			return -ENOENT;
		}

		crtc = drm_crtc_find(dev, plane_req->crtc_id);
		if (!crtc) {
			DRM_DEBUG_KMS("Unknown crtc ID %d\n",
				      plane_req->crtc_id);
			return -ENOENT;
		}
	}

	/*
	 * setplane_internal will take care of deref'ing either the old or new
	 * framebuffer depending on success.
	 */
	return setplane_internal(plane, crtc, fb,
				 plane_req->crtc_x, plane_req->crtc_y,
				 plane_req->crtc_w, plane_req->crtc_h,
				 plane_req->src_x, plane_req->src_y,
				 plane_req->src_w, plane_req->src_h);
}

/**
 * drm_mode_set_config_internal - helper to call ->set_config
 * @set: modeset config to set
 *
 * This is a little helper to wrap internal calls to the ->set_config driver
 * interface. The only thing it adds is correct refcounting dance.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_set_config_internal(struct drm_mode_set *set)
{
	struct drm_crtc *crtc = set->crtc;
	struct drm_framebuffer *fb;
	struct drm_crtc *tmp;
	int ret;

	/*
	 * NOTE: ->set_config can also disable other crtcs (if we steal all
	 * connectors from it), hence we need to refcount the fbs across all
	 * crtcs. Atomic modeset will have saner semantics ...
	 */
	drm_for_each_crtc(tmp, crtc->dev)
		tmp->primary->old_fb = tmp->primary->fb;

	fb = set->fb;

	ret = crtc->funcs->set_config(set);
	if (ret == 0) {
		crtc->primary->crtc = crtc;
		crtc->primary->fb = fb;
	}

	drm_for_each_crtc(tmp, crtc->dev) {
		if (tmp->primary->fb)
			drm_framebuffer_reference(tmp->primary->fb);
		if (tmp->primary->old_fb)
			drm_framebuffer_unreference(tmp->primary->old_fb);
		tmp->primary->old_fb = NULL;
	}

	return ret;
}
EXPORT_SYMBOL(drm_mode_set_config_internal);

/**
 * drm_crtc_get_hv_timing - Fetches hdisplay/vdisplay for given mode
 * @mode: mode to query
 * @hdisplay: hdisplay value to fill in
 * @vdisplay: vdisplay value to fill in
 *
 * The vdisplay value will be doubled if the specified mode is a stereo mode of
 * the appropriate layout.
 */
void drm_crtc_get_hv_timing(const struct drm_display_mode *mode,
			    int *hdisplay, int *vdisplay)
{
	struct drm_display_mode adjusted;

	drm_mode_copy(&adjusted, mode);
	drm_mode_set_crtcinfo(&adjusted, CRTC_STEREO_DOUBLE_ONLY);
	*hdisplay = adjusted.crtc_hdisplay;
	*vdisplay = adjusted.crtc_vdisplay;
}
EXPORT_SYMBOL(drm_crtc_get_hv_timing);

/**
 * drm_crtc_check_viewport - Checks that a framebuffer is big enough for the
 *     CRTC viewport
 * @crtc: CRTC that framebuffer will be displayed on
 * @x: x panning
 * @y: y panning
 * @mode: mode that framebuffer will be displayed under
 * @fb: framebuffer to check size of
 */
int drm_crtc_check_viewport(const struct drm_crtc *crtc,
			    int x, int y,
			    const struct drm_display_mode *mode,
			    const struct drm_framebuffer *fb)

{
	int hdisplay, vdisplay;

	drm_crtc_get_hv_timing(mode, &hdisplay, &vdisplay);

	if (crtc->state &&
	    crtc->primary->state->rotation & (DRM_ROTATE_90 |
					      DRM_ROTATE_270))
		swap(hdisplay, vdisplay);

	return check_src_coords(x << 16, y << 16,
				hdisplay << 16, vdisplay << 16, fb);
}
EXPORT_SYMBOL(drm_crtc_check_viewport);

/**
 * drm_mode_setcrtc - set CRTC configuration
 * @dev: drm device for the ioctl
 * @data: data pointer for the ioctl
 * @file_priv: drm file for the ioctl call
 *
 * Build a new CRTC configuration based on user request.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_setcrtc(struct drm_device *dev, void *data,
		     struct drm_file *file_priv)
{
	struct drm_mode_config *config = &dev->mode_config;
	struct drm_mode_crtc *crtc_req = data;
	struct drm_crtc *crtc;
	struct drm_connector **connector_set = NULL, *connector;
	struct drm_framebuffer *fb = NULL;
	struct drm_display_mode *mode = NULL;
	struct drm_mode_set set;
	uint32_t __user *set_connectors_ptr;
	int ret;
	int i;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	/*
	 * Universal plane src offsets are only 16.16, prevent havoc for
	 * drivers using universal plane code internally.
	 */
	if (crtc_req->x & 0xffff0000 || crtc_req->y & 0xffff0000)
		return -ERANGE;

	drm_modeset_lock_all(dev);
	crtc = drm_crtc_find(dev, crtc_req->crtc_id);
	if (!crtc) {
		DRM_DEBUG_KMS("Unknown CRTC ID %d\n", crtc_req->crtc_id);
		ret = -ENOENT;
		goto out;
	}
	DRM_DEBUG_KMS("[CRTC:%d:%s]\n", crtc->base.id, crtc->name);

	if (crtc_req->mode_valid) {
		/* If we have a mode we need a framebuffer. */
		/* If we pass -1, set the mode with the currently bound fb */
		if (crtc_req->fb_id == -1) {
			if (!crtc->primary->fb) {
				DRM_DEBUG_KMS("CRTC doesn't have current FB\n");
				ret = -EINVAL;
				goto out;
			}
			fb = crtc->primary->fb;
			/* Make refcounting symmetric with the lookup path. */
			drm_framebuffer_reference(fb);
		} else {
			fb = drm_framebuffer_lookup(dev, crtc_req->fb_id);
			if (!fb) {
				DRM_DEBUG_KMS("Unknown FB ID%d\n",
						crtc_req->fb_id);
				ret = -ENOENT;
				goto out;
			}
		}

		mode = drm_mode_create(dev);
		if (!mode) {
			ret = -ENOMEM;
			goto out;
		}

		ret = drm_mode_convert_umode(mode, &crtc_req->mode);
		if (ret) {
			DRM_DEBUG_KMS("Invalid mode\n");
			goto out;
		}

		/*
		 * Check whether the primary plane supports the fb pixel format.
		 * Drivers not implementing the universal planes API use a
		 * default formats list provided by the DRM core which doesn't
		 * match real hardware capabilities. Skip the check in that
		 * case.
		 */
		if (!crtc->primary->format_default) {
			ret = drm_plane_check_pixel_format(crtc->primary,
							   fb->pixel_format);
			if (ret) {
				char *format_name = drm_get_format_name(fb->pixel_format);
				DRM_DEBUG_KMS("Invalid pixel format %s\n", format_name);
				kfree(format_name);
				goto out;
			}
		}

		ret = drm_crtc_check_viewport(crtc, crtc_req->x, crtc_req->y,
					      mode, fb);
		if (ret)
			goto out;

	}

	if (crtc_req->count_connectors == 0 && mode) {
		DRM_DEBUG_KMS("Count connectors is 0 but mode set\n");
		ret = -EINVAL;
		goto out;
	}

	if (crtc_req->count_connectors > 0 && (!mode || !fb)) {
		DRM_DEBUG_KMS("Count connectors is %d but no mode or fb set\n",
			  crtc_req->count_connectors);
		ret = -EINVAL;
		goto out;
	}

	if (crtc_req->count_connectors > 0) {
		u32 out_id;

		/* Avoid unbounded kernel memory allocation */
		if (crtc_req->count_connectors > config->num_connector) {
			ret = -EINVAL;
			goto out;
		}

		connector_set = kmalloc_array(crtc_req->count_connectors,
					      sizeof(struct drm_connector *),
					      GFP_KERNEL);
		if (!connector_set) {
			ret = -ENOMEM;
			goto out;
		}

		for (i = 0; i < crtc_req->count_connectors; i++) {
			connector_set[i] = NULL;
			set_connectors_ptr = (uint32_t __user *)(unsigned long)crtc_req->set_connectors_ptr;
			if (get_user(out_id, &set_connectors_ptr[i])) {
				ret = -EFAULT;
				goto out;
			}

			connector = drm_connector_lookup(dev, out_id);
			if (!connector) {
				DRM_DEBUG_KMS("Connector id %d unknown\n",
						out_id);
				ret = -ENOENT;
				goto out;
			}
			DRM_DEBUG_KMS("[CONNECTOR:%d:%s]\n",
					connector->base.id,
					connector->name);

			connector_set[i] = connector;
		}
	}

	set.crtc = crtc;
	set.x = crtc_req->x;
	set.y = crtc_req->y;
	set.mode = mode;
	set.connectors = connector_set;
	set.num_connectors = crtc_req->count_connectors;
	set.fb = fb;
	ret = drm_mode_set_config_internal(&set);

out:
	if (fb)
		drm_framebuffer_unreference(fb);

	if (connector_set) {
		for (i = 0; i < crtc_req->count_connectors; i++) {
			if (connector_set[i])
				drm_connector_unreference(connector_set[i]);
		}
	}
	kfree(connector_set);
	drm_mode_destroy(dev, mode);
	drm_modeset_unlock_all(dev);
	return ret;
}

/**
 * drm_mode_cursor_universal - translate legacy cursor ioctl call into a
 *     universal plane handler call
 * @crtc: crtc to update cursor for
 * @req: data pointer for the ioctl
 * @file_priv: drm file for the ioctl call
 *
 * Legacy cursor ioctl's work directly with driver buffer handles.  To
 * translate legacy ioctl calls into universal plane handler calls, we need to
 * wrap the native buffer handle in a drm_framebuffer.
 *
 * Note that we assume any handle passed to the legacy ioctls was a 32-bit ARGB
 * buffer with a pitch of 4*width; the universal plane interface should be used
 * directly in cases where the hardware can support other buffer settings and
 * userspace wants to make use of these capabilities.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
static int drm_mode_cursor_universal(struct drm_crtc *crtc,
				     struct drm_mode_cursor2 *req,
				     struct drm_file *file_priv)
{
	struct drm_device *dev = crtc->dev;
	struct drm_framebuffer *fb = NULL;
	struct drm_mode_fb_cmd2 fbreq = {
		.width = req->width,
		.height = req->height,
		.pixel_format = DRM_FORMAT_ARGB8888,
		.pitches = { req->width * 4 },
		.handles = { req->handle },
	};
	int32_t crtc_x, crtc_y;
	uint32_t crtc_w = 0, crtc_h = 0;
	uint32_t src_w = 0, src_h = 0;
	int ret = 0;

	BUG_ON(!crtc->cursor);
	WARN_ON(crtc->cursor->crtc != crtc && crtc->cursor->crtc != NULL);

	/*
	 * Obtain fb we'll be using (either new or existing) and take an extra
	 * reference to it if fb != null.  setplane will take care of dropping
	 * the reference if the plane update fails.
	 */
	if (req->flags & DRM_MODE_CURSOR_BO) {
		if (req->handle) {
			fb = drm_internal_framebuffer_create(dev, &fbreq, file_priv);
			if (IS_ERR(fb)) {
				DRM_DEBUG_KMS("failed to wrap cursor buffer in drm framebuffer\n");
				return PTR_ERR(fb);
			}
			fb->hot_x = req->hot_x;
			fb->hot_y = req->hot_y;
		} else {
			fb = NULL;
		}
	} else {
		fb = crtc->cursor->fb;
		if (fb)
			drm_framebuffer_reference(fb);
	}

	if (req->flags & DRM_MODE_CURSOR_MOVE) {
		crtc_x = req->x;
		crtc_y = req->y;
	} else {
		crtc_x = crtc->cursor_x;
		crtc_y = crtc->cursor_y;
	}

	if (fb) {
		crtc_w = fb->width;
		crtc_h = fb->height;
		src_w = fb->width << 16;
		src_h = fb->height << 16;
	}

	/*
	 * setplane_internal will take care of deref'ing either the old or new
	 * framebuffer depending on success.
	 */
	ret = __setplane_internal(crtc->cursor, crtc, fb,
				crtc_x, crtc_y, crtc_w, crtc_h,
				0, 0, src_w, src_h);

	/* Update successful; save new cursor position, if necessary */
	if (ret == 0 && req->flags & DRM_MODE_CURSOR_MOVE) {
		crtc->cursor_x = req->x;
		crtc->cursor_y = req->y;
	}

	return ret;
}

static int drm_mode_cursor_common(struct drm_device *dev,
				  struct drm_mode_cursor2 *req,
				  struct drm_file *file_priv)
{
	struct drm_crtc *crtc;
	int ret = 0;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	if (!req->flags || (~DRM_MODE_CURSOR_FLAGS & req->flags))
		return -EINVAL;

	crtc = drm_crtc_find(dev, req->crtc_id);
	if (!crtc) {
		DRM_DEBUG_KMS("Unknown CRTC ID %d\n", req->crtc_id);
		return -ENOENT;
	}

	/*
	 * If this crtc has a universal cursor plane, call that plane's update
	 * handler rather than using legacy cursor handlers.
	 */
	drm_modeset_lock_crtc(crtc, crtc->cursor);
	if (crtc->cursor) {
		ret = drm_mode_cursor_universal(crtc, req, file_priv);
		goto out;
	}

	if (req->flags & DRM_MODE_CURSOR_BO) {
		if (!crtc->funcs->cursor_set && !crtc->funcs->cursor_set2) {
			ret = -ENXIO;
			goto out;
		}
		/* Turns off the cursor if handle is 0 */
		if (crtc->funcs->cursor_set2)
			ret = crtc->funcs->cursor_set2(crtc, file_priv, req->handle,
						      req->width, req->height, req->hot_x, req->hot_y);
		else
			ret = crtc->funcs->cursor_set(crtc, file_priv, req->handle,
						      req->width, req->height);
	}

	if (req->flags & DRM_MODE_CURSOR_MOVE) {
		if (crtc->funcs->cursor_move) {
			ret = crtc->funcs->cursor_move(crtc, req->x, req->y);
		} else {
			ret = -EFAULT;
			goto out;
		}
	}
out:
	drm_modeset_unlock_crtc(crtc);

	return ret;

}


/**
 * drm_mode_cursor_ioctl - set CRTC's cursor configuration
 * @dev: drm device for the ioctl
 * @data: data pointer for the ioctl
 * @file_priv: drm file for the ioctl call
 *
 * Set the cursor configuration based on user request.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_cursor_ioctl(struct drm_device *dev,
			  void *data, struct drm_file *file_priv)
{
	struct drm_mode_cursor *req = data;
	struct drm_mode_cursor2 new_req;

	memcpy(&new_req, req, sizeof(struct drm_mode_cursor));
	new_req.hot_x = new_req.hot_y = 0;

	return drm_mode_cursor_common(dev, &new_req, file_priv);
}

/**
 * drm_mode_cursor2_ioctl - set CRTC's cursor configuration
 * @dev: drm device for the ioctl
 * @data: data pointer for the ioctl
 * @file_priv: drm file for the ioctl call
 *
 * Set the cursor configuration based on user request. This implements the 2nd
 * version of the cursor ioctl, which allows userspace to additionally specify
 * the hotspot of the pointer.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_cursor2_ioctl(struct drm_device *dev,
			   void *data, struct drm_file *file_priv)
{
	struct drm_mode_cursor2 *req = data;

	return drm_mode_cursor_common(dev, req, file_priv);
}

int drm_mode_crtc_set_obj_prop(struct drm_mode_object *obj,
			       struct drm_property *property,
			       uint64_t value)
{
	int ret = -EINVAL;
	struct drm_crtc *crtc = obj_to_crtc(obj);

	if (crtc->funcs->set_property)
		ret = crtc->funcs->set_property(crtc, property, value);
	if (!ret)
		drm_object_property_set_value(obj, property, value);

	return ret;
}

/**
 * drm_mode_plane_set_obj_prop - set the value of a property
 * @plane: drm plane object to set property value for
 * @property: property to set
 * @value: value the property should be set to
 *
 * This functions sets a given property on a given plane object. This function
 * calls the driver's ->set_property callback and changes the software state of
 * the property if the callback succeeds.
 *
 * Returns:
 * Zero on success, error code on failure.
 */
int drm_mode_plane_set_obj_prop(struct drm_plane *plane,
				struct drm_property *property,
				uint64_t value)
{
	int ret = -EINVAL;
	struct drm_mode_object *obj = &plane->base;

	if (plane->funcs->set_property)
		ret = plane->funcs->set_property(plane, property, value);
	if (!ret)
		drm_object_property_set_value(obj, property, value);

	return ret;
}
EXPORT_SYMBOL(drm_mode_plane_set_obj_prop);

/**
 * drm_mode_crtc_set_gamma_size - set the gamma table size
 * @crtc: CRTC to set the gamma table size for
 * @gamma_size: size of the gamma table
 *
 * Drivers which support gamma tables should set this to the supported gamma
 * table size when initializing the CRTC. Currently the drm core only supports a
 * fixed gamma table size.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_crtc_set_gamma_size(struct drm_crtc *crtc,
				 int gamma_size)
{
	uint16_t *r_base, *g_base, *b_base;
	int i;

	crtc->gamma_size = gamma_size;

	crtc->gamma_store = kcalloc(gamma_size, sizeof(uint16_t) * 3,
				    GFP_KERNEL);
	if (!crtc->gamma_store) {
		crtc->gamma_size = 0;
		return -ENOMEM;
	}

	r_base = crtc->gamma_store;
	g_base = r_base + gamma_size;
	b_base = g_base + gamma_size;
	for (i = 0; i < gamma_size; i++) {
		r_base[i] = i << 8;
		g_base[i] = i << 8;
		b_base[i] = i << 8;
	}


	return 0;
}
EXPORT_SYMBOL(drm_mode_crtc_set_gamma_size);

/**
 * drm_mode_gamma_set_ioctl - set the gamma table
 * @dev: DRM device
 * @data: ioctl data
 * @file_priv: DRM file info
 *
 * Set the gamma table of a CRTC to the one passed in by the user. Userspace can
 * inquire the required gamma table size through drm_mode_gamma_get_ioctl.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_gamma_set_ioctl(struct drm_device *dev,
			     void *data, struct drm_file *file_priv)
{
	struct drm_mode_crtc_lut *crtc_lut = data;
	struct drm_crtc *crtc;
	void *r_base, *g_base, *b_base;
	int size;
	int ret = 0;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	drm_modeset_lock_all(dev);
	crtc = drm_crtc_find(dev, crtc_lut->crtc_id);
	if (!crtc) {
		ret = -ENOENT;
		goto out;
	}

	if (crtc->funcs->gamma_set == NULL) {
		ret = -ENOSYS;
		goto out;
	}

	/* memcpy into gamma store */
	if (crtc_lut->gamma_size != crtc->gamma_size) {
		ret = -EINVAL;
		goto out;
	}

	size = crtc_lut->gamma_size * (sizeof(uint16_t));
	r_base = crtc->gamma_store;
	if (copy_from_user(r_base, (void __user *)(unsigned long)crtc_lut->red, size)) {
		ret = -EFAULT;
		goto out;
	}

	g_base = r_base + size;
	if (copy_from_user(g_base, (void __user *)(unsigned long)crtc_lut->green, size)) {
		ret = -EFAULT;
		goto out;
	}

	b_base = g_base + size;
	if (copy_from_user(b_base, (void __user *)(unsigned long)crtc_lut->blue, size)) {
		ret = -EFAULT;
		goto out;
	}

	ret = crtc->funcs->gamma_set(crtc, r_base, g_base, b_base, crtc->gamma_size);

out:
	drm_modeset_unlock_all(dev);
	return ret;

}

/**
 * drm_mode_gamma_get_ioctl - get the gamma table
 * @dev: DRM device
 * @data: ioctl data
 * @file_priv: DRM file info
 *
 * Copy the current gamma table into the storage provided. This also provides
 * the gamma table size the driver expects, which can be used to size the
 * allocated storage.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_gamma_get_ioctl(struct drm_device *dev,
			     void *data, struct drm_file *file_priv)
{
	struct drm_mode_crtc_lut *crtc_lut = data;
	struct drm_crtc *crtc;
	void *r_base, *g_base, *b_base;
	int size;
	int ret = 0;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	drm_modeset_lock_all(dev);
	crtc = drm_crtc_find(dev, crtc_lut->crtc_id);
	if (!crtc) {
		ret = -ENOENT;
		goto out;
	}

	/* memcpy into gamma store */
	if (crtc_lut->gamma_size != crtc->gamma_size) {
		ret = -EINVAL;
		goto out;
	}

	size = crtc_lut->gamma_size * (sizeof(uint16_t));
	r_base = crtc->gamma_store;
	if (copy_to_user((void __user *)(unsigned long)crtc_lut->red, r_base, size)) {
		ret = -EFAULT;
		goto out;
	}

	g_base = r_base + size;
	if (copy_to_user((void __user *)(unsigned long)crtc_lut->green, g_base, size)) {
		ret = -EFAULT;
		goto out;
	}

	b_base = g_base + size;
	if (copy_to_user((void __user *)(unsigned long)crtc_lut->blue, b_base, size)) {
		ret = -EFAULT;
		goto out;
	}
out:
	drm_modeset_unlock_all(dev);
	return ret;
}

/**
 * drm_mode_page_flip_ioctl - schedule an asynchronous fb update
 * @dev: DRM device
 * @data: ioctl data
 * @file_priv: DRM file info
 *
 * This schedules an asynchronous update on a given CRTC, called page flip.
 * Optionally a drm event is generated to signal the completion of the event.
 * Generic drivers cannot assume that a pageflip with changed framebuffer
 * properties (including driver specific metadata like tiling layout) will work,
 * but some drivers support e.g. pixel format changes through the pageflip
 * ioctl.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_page_flip_ioctl(struct drm_device *dev,
			     void *data, struct drm_file *file_priv)
{
	struct drm_mode_crtc_page_flip_target *page_flip = data;
	struct drm_crtc *crtc;
	struct drm_framebuffer *fb = NULL;
	struct drm_pending_vblank_event *e = NULL;
	u32 target_vblank = page_flip->sequence;
	int ret = -EINVAL;

	if (page_flip->flags & ~DRM_MODE_PAGE_FLIP_FLAGS)
		return -EINVAL;

	if (page_flip->sequence != 0 && !(page_flip->flags & DRM_MODE_PAGE_FLIP_TARGET))
		return -EINVAL;

	/* Only one of the DRM_MODE_PAGE_FLIP_TARGET_ABSOLUTE/RELATIVE flags
	 * can be specified
	 */
	if ((page_flip->flags & DRM_MODE_PAGE_FLIP_TARGET) == DRM_MODE_PAGE_FLIP_TARGET)
		return -EINVAL;

	if ((page_flip->flags & DRM_MODE_PAGE_FLIP_ASYNC) && !dev->mode_config.async_page_flip)
		return -EINVAL;

	crtc = drm_crtc_find(dev, page_flip->crtc_id);
	if (!crtc)
		return -ENOENT;

	if (crtc->funcs->page_flip_target) {
		u32 current_vblank;
		int r;

		r = drm_crtc_vblank_get(crtc);
		if (r)
			return r;

		current_vblank = drm_crtc_vblank_count(crtc);

		switch (page_flip->flags & DRM_MODE_PAGE_FLIP_TARGET) {
		case DRM_MODE_PAGE_FLIP_TARGET_ABSOLUTE:
			if ((int)(target_vblank - current_vblank) > 1) {
				DRM_DEBUG("Invalid absolute flip target %u, "
					  "must be <= %u\n", target_vblank,
					  current_vblank + 1);
				drm_crtc_vblank_put(crtc);
				return -EINVAL;
			}
			break;
		case DRM_MODE_PAGE_FLIP_TARGET_RELATIVE:
			if (target_vblank != 0 && target_vblank != 1) {
				DRM_DEBUG("Invalid relative flip target %u, "
					  "must be 0 or 1\n", target_vblank);
				drm_crtc_vblank_put(crtc);
				return -EINVAL;
			}
			target_vblank += current_vblank;
			break;
		default:
			target_vblank = current_vblank +
				!(page_flip->flags & DRM_MODE_PAGE_FLIP_ASYNC);
			break;
		}
	} else if (crtc->funcs->page_flip == NULL ||
		   (page_flip->flags & DRM_MODE_PAGE_FLIP_TARGET)) {
		return -EINVAL;
	}

	drm_modeset_lock_crtc(crtc, crtc->primary);
	if (crtc->primary->fb == NULL) {
		/* The framebuffer is currently unbound, presumably
		 * due to a hotplug event, that userspace has not
		 * yet discovered.
		 */
		ret = -EBUSY;
		goto out;
	}

	fb = drm_framebuffer_lookup(dev, page_flip->fb_id);
	if (!fb) {
		ret = -ENOENT;
		goto out;
	}

	if (crtc->state) {
		const struct drm_plane_state *state = crtc->primary->state;

		ret = check_src_coords(state->src_x, state->src_y,
				       state->src_w, state->src_h, fb);
	} else {
		ret = drm_crtc_check_viewport(crtc, crtc->x, crtc->y, &crtc->mode, fb);
	}
	if (ret)
		goto out;

	if (crtc->primary->fb->pixel_format != fb->pixel_format) {
		DRM_DEBUG_KMS("Page flip is not allowed to change frame buffer format.\n");
		ret = -EINVAL;
		goto out;
	}

	if (page_flip->flags & DRM_MODE_PAGE_FLIP_EVENT) {
		e = kzalloc(sizeof *e, GFP_KERNEL);
		if (!e) {
			ret = -ENOMEM;
			goto out;
		}
		e->event.base.type = DRM_EVENT_FLIP_COMPLETE;
		e->event.base.length = sizeof(e->event);
		e->event.user_data = page_flip->user_data;
		ret = drm_event_reserve_init(dev, file_priv, &e->base, &e->event.base);
		if (ret) {
			kfree(e);
			goto out;
		}
	}

	crtc->primary->old_fb = crtc->primary->fb;
	if (crtc->funcs->page_flip_target)
		ret = crtc->funcs->page_flip_target(crtc, fb, e,
						    page_flip->flags,
						    target_vblank);
	else
		ret = crtc->funcs->page_flip(crtc, fb, e, page_flip->flags);
	if (ret) {
		if (page_flip->flags & DRM_MODE_PAGE_FLIP_EVENT)
			drm_event_cancel_free(dev, &e->base);
		/* Keep the old fb, don't unref it. */
		crtc->primary->old_fb = NULL;
	} else {
		crtc->primary->fb = fb;
		/* Unref only the old framebuffer. */
		fb = NULL;
	}

out:
	if (ret && crtc->funcs->page_flip_target)
		drm_crtc_vblank_put(crtc);
	if (fb)
		drm_framebuffer_unreference(fb);
	if (crtc->primary->old_fb)
		drm_framebuffer_unreference(crtc->primary->old_fb);
	crtc->primary->old_fb = NULL;
	drm_modeset_unlock_crtc(crtc);

	return ret;
}

/**
 * drm_mode_config_reset - call ->reset callbacks
 * @dev: drm device
 *
 * This functions calls all the crtc's, encoder's and connector's ->reset
 * callback. Drivers can use this in e.g. their driver load or resume code to
 * reset hardware and software state.
 */
void drm_mode_config_reset(struct drm_device *dev)
{
	struct drm_crtc *crtc;
	struct drm_plane *plane;
	struct drm_encoder *encoder;
	struct drm_connector *connector;

	drm_for_each_plane(plane, dev)
		if (plane->funcs->reset)
			plane->funcs->reset(plane);

	drm_for_each_crtc(crtc, dev)
		if (crtc->funcs->reset)
			crtc->funcs->reset(crtc);

	drm_for_each_encoder(encoder, dev)
		if (encoder->funcs->reset)
			encoder->funcs->reset(encoder);

	mutex_lock(&dev->mode_config.mutex);
	drm_for_each_connector(connector, dev)
		if (connector->funcs->reset)
			connector->funcs->reset(connector);
	mutex_unlock(&dev->mode_config.mutex);
}
EXPORT_SYMBOL(drm_mode_config_reset);

/**
 * drm_mode_create_dumb_ioctl - create a dumb backing storage buffer
 * @dev: DRM device
 * @data: ioctl data
 * @file_priv: DRM file info
 *
 * This creates a new dumb buffer in the driver's backing storage manager (GEM,
 * TTM or something else entirely) and returns the resulting buffer handle. This
 * handle can then be wrapped up into a framebuffer modeset object.
 *
 * Note that userspace is not allowed to use such objects for render
 * acceleration - drivers must create their own private ioctls for such a use
 * case.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_create_dumb_ioctl(struct drm_device *dev,
			       void *data, struct drm_file *file_priv)
{
	struct drm_mode_create_dumb *args = data;
	u32 cpp, stride, size;

	if (!dev->driver->dumb_create)
		return -ENOSYS;
	if (!args->width || !args->height || !args->bpp)
		return -EINVAL;

	/* overflow checks for 32bit size calculations */
	/* NOTE: DIV_ROUND_UP() can overflow */
	cpp = DIV_ROUND_UP(args->bpp, 8);
	if (!cpp || cpp > 0xffffffffU / args->width)
		return -EINVAL;
	stride = cpp * args->width;
	if (args->height > 0xffffffffU / stride)
		return -EINVAL;

	/* test for wrap-around */
	size = args->height * stride;
	if (PAGE_ALIGN(size) == 0)
		return -EINVAL;

	/*
	 * handle, pitch and size are output parameters. Zero them out to
	 * prevent drivers from accidentally using uninitialized data. Since
	 * not all existing userspace is clearing these fields properly we
	 * cannot reject IOCTL with garbage in them.
	 */
	args->handle = 0;
	args->pitch = 0;
	args->size = 0;

	return dev->driver->dumb_create(file_priv, dev, args);
}

/**
 * drm_mode_mmap_dumb_ioctl - create an mmap offset for a dumb backing storage buffer
 * @dev: DRM device
 * @data: ioctl data
 * @file_priv: DRM file info
 *
 * Allocate an offset in the drm device node's address space to be able to
 * memory map a dumb buffer.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_mmap_dumb_ioctl(struct drm_device *dev,
			     void *data, struct drm_file *file_priv)
{
	struct drm_mode_map_dumb *args = data;

	/* call driver ioctl to get mmap offset */
	if (!dev->driver->dumb_map_offset)
		return -ENOSYS;

	return dev->driver->dumb_map_offset(file_priv, dev, args->handle, &args->offset);
}

/**
 * drm_mode_destroy_dumb_ioctl - destroy a dumb backing strage buffer
 * @dev: DRM device
 * @data: ioctl data
 * @file_priv: DRM file info
 *
 * This destroys the userspace handle for the given dumb backing storage buffer.
 * Since buffer objects must be reference counted in the kernel a buffer object
 * won't be immediately freed if a framebuffer modeset object still uses it.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_destroy_dumb_ioctl(struct drm_device *dev,
				void *data, struct drm_file *file_priv)
{
	struct drm_mode_destroy_dumb *args = data;

	if (!dev->driver->dumb_destroy)
		return -ENOSYS;

	return dev->driver->dumb_destroy(file_priv, dev, args->handle);
}

/**
 * drm_rotation_simplify() - Try to simplify the rotation
 * @rotation: Rotation to be simplified
 * @supported_rotations: Supported rotations
 *
 * Attempt to simplify the rotation to a form that is supported.
 * Eg. if the hardware supports everything except DRM_REFLECT_X
 * one could call this function like this:
 *
 * drm_rotation_simplify(rotation, DRM_ROTATE_0 |
 *                       DRM_ROTATE_90 | DRM_ROTATE_180 |
 *                       DRM_ROTATE_270 | DRM_REFLECT_Y);
 *
 * to eliminate the DRM_ROTATE_X flag. Depending on what kind of
 * transforms the hardware supports, this function may not
 * be able to produce a supported transform, so the caller should
 * check the result afterwards.
 */
unsigned int drm_rotation_simplify(unsigned int rotation,
				   unsigned int supported_rotations)
{
	if (rotation & ~supported_rotations) {
		rotation ^= DRM_REFLECT_X | DRM_REFLECT_Y;
		rotation = (rotation & DRM_REFLECT_MASK) |
		           BIT((ffs(rotation & DRM_ROTATE_MASK) + 1) % 4);
	}

	return rotation;
}
EXPORT_SYMBOL(drm_rotation_simplify);

/**
 * drm_mode_config_init - initialize DRM mode_configuration structure
 * @dev: DRM device
 *
 * Initialize @dev's mode_config structure, used for tracking the graphics
 * configuration of @dev.
 *
 * Since this initializes the modeset locks, no locking is possible. Which is no
 * problem, since this should happen single threaded at init time. It is the
 * driver's problem to ensure this guarantee.
 *
 */
void drm_mode_config_init(struct drm_device *dev)
{
	mutex_init(&dev->mode_config.mutex);
	drm_modeset_lock_init(&dev->mode_config.connection_mutex);
	mutex_init(&dev->mode_config.idr_mutex);
	mutex_init(&dev->mode_config.fb_lock);
	mutex_init(&dev->mode_config.blob_lock);
	INIT_LIST_HEAD(&dev->mode_config.fb_list);
	INIT_LIST_HEAD(&dev->mode_config.crtc_list);
	INIT_LIST_HEAD(&dev->mode_config.connector_list);
	INIT_LIST_HEAD(&dev->mode_config.encoder_list);
	INIT_LIST_HEAD(&dev->mode_config.property_list);
	INIT_LIST_HEAD(&dev->mode_config.property_blob_list);
	INIT_LIST_HEAD(&dev->mode_config.plane_list);
	idr_init(&dev->mode_config.crtc_idr);
	idr_init(&dev->mode_config.tile_idr);
	ida_init(&dev->mode_config.connector_ida);

	drm_modeset_lock_all(dev);
	drm_mode_create_standard_properties(dev);
	drm_modeset_unlock_all(dev);

	/* Just to be sure */
	dev->mode_config.num_fb = 0;
	dev->mode_config.num_connector = 0;
	dev->mode_config.num_crtc = 0;
	dev->mode_config.num_encoder = 0;
	dev->mode_config.num_overlay_plane = 0;
	dev->mode_config.num_total_plane = 0;
}
EXPORT_SYMBOL(drm_mode_config_init);

/**
 * drm_mode_config_cleanup - free up DRM mode_config info
 * @dev: DRM device
 *
 * Free up all the connectors and CRTCs associated with this DRM device, then
 * free up the framebuffers and associated buffer objects.
 *
 * Note that since this /should/ happen single-threaded at driver/device
 * teardown time, no locking is required. It's the driver's job to ensure that
 * this guarantee actually holds true.
 *
 * FIXME: cleanup any dangling user buffer objects too
 */
void drm_mode_config_cleanup(struct drm_device *dev)
{
	struct drm_connector *connector, *ot;
	struct drm_crtc *crtc, *ct;
	struct drm_encoder *encoder, *enct;
	struct drm_framebuffer *fb, *fbt;
	struct drm_property *property, *pt;
	struct drm_property_blob *blob, *bt;
	struct drm_plane *plane, *plt;

	list_for_each_entry_safe(encoder, enct, &dev->mode_config.encoder_list,
				 head) {
		encoder->funcs->destroy(encoder);
	}

	list_for_each_entry_safe(connector, ot,
				 &dev->mode_config.connector_list, head) {
		connector->funcs->destroy(connector);
	}

	list_for_each_entry_safe(property, pt, &dev->mode_config.property_list,
				 head) {
		drm_property_destroy(dev, property);
	}

	list_for_each_entry_safe(plane, plt, &dev->mode_config.plane_list,
				 head) {
		plane->funcs->destroy(plane);
	}

	list_for_each_entry_safe(crtc, ct, &dev->mode_config.crtc_list, head) {
		crtc->funcs->destroy(crtc);
	}

	list_for_each_entry_safe(blob, bt, &dev->mode_config.property_blob_list,
				 head_global) {
		drm_property_unreference_blob(blob);
	}

	/*
	 * Single-threaded teardown context, so it's not required to grab the
	 * fb_lock to protect against concurrent fb_list access. Contrary, it
	 * would actually deadlock with the drm_framebuffer_cleanup function.
	 *
	 * Also, if there are any framebuffers left, that's a driver leak now,
	 * so politely WARN about this.
	 */
	WARN_ON(!list_empty(&dev->mode_config.fb_list));
	list_for_each_entry_safe(fb, fbt, &dev->mode_config.fb_list, head) {
		drm_framebuffer_free(&fb->base.refcount);
	}

	ida_destroy(&dev->mode_config.connector_ida);
	idr_destroy(&dev->mode_config.tile_idr);
	idr_destroy(&dev->mode_config.crtc_idr);
	drm_modeset_lock_fini(&dev->mode_config.connection_mutex);
}
EXPORT_SYMBOL(drm_mode_config_cleanup);

struct drm_property *drm_mode_create_rotation_property(struct drm_device *dev,
						       unsigned int supported_rotations)
{
	static const struct drm_prop_enum_list props[] = {
		{ __builtin_ffs(DRM_ROTATE_0) - 1,   "rotate-0" },
		{ __builtin_ffs(DRM_ROTATE_90) - 1,  "rotate-90" },
		{ __builtin_ffs(DRM_ROTATE_180) - 1, "rotate-180" },
		{ __builtin_ffs(DRM_ROTATE_270) - 1, "rotate-270" },
		{ __builtin_ffs(DRM_REFLECT_X) - 1,  "reflect-x" },
		{ __builtin_ffs(DRM_REFLECT_Y) - 1,  "reflect-y" },
	};

	return drm_property_create_bitmask(dev, 0, "rotation",
					   props, ARRAY_SIZE(props),
					   supported_rotations);
}
EXPORT_SYMBOL(drm_mode_create_rotation_property);

/**
 * DOC: Tile group
 *
 * Tile groups are used to represent tiled monitors with a unique
 * integer identifier. Tiled monitors using DisplayID v1.3 have
 * a unique 8-byte handle, we store this in a tile group, so we
 * have a common identifier for all tiles in a monitor group.
 */
static void drm_tile_group_free(struct kref *kref)
{
	struct drm_tile_group *tg = container_of(kref, struct drm_tile_group, refcount);
	struct drm_device *dev = tg->dev;
	mutex_lock(&dev->mode_config.idr_mutex);
	idr_remove(&dev->mode_config.tile_idr, tg->id);
	mutex_unlock(&dev->mode_config.idr_mutex);
	kfree(tg);
}

/**
 * drm_mode_put_tile_group - drop a reference to a tile group.
 * @dev: DRM device
 * @tg: tile group to drop reference to.
 *
 * drop reference to tile group and free if 0.
 */
void drm_mode_put_tile_group(struct drm_device *dev,
			     struct drm_tile_group *tg)
{
	kref_put(&tg->refcount, drm_tile_group_free);
}

/**
 * drm_mode_get_tile_group - get a reference to an existing tile group
 * @dev: DRM device
 * @topology: 8-bytes unique per monitor.
 *
 * Use the unique bytes to get a reference to an existing tile group.
 *
 * RETURNS:
 * tile group or NULL if not found.
 */
struct drm_tile_group *drm_mode_get_tile_group(struct drm_device *dev,
					       char topology[8])
{
	struct drm_tile_group *tg;
	int id;
	mutex_lock(&dev->mode_config.idr_mutex);
	idr_for_each_entry(&dev->mode_config.tile_idr, tg, id) {
		if (!memcmp(tg->group_data, topology, 8)) {
			if (!kref_get_unless_zero(&tg->refcount))
				tg = NULL;
			mutex_unlock(&dev->mode_config.idr_mutex);
			return tg;
		}
	}
	mutex_unlock(&dev->mode_config.idr_mutex);
	return NULL;
}
EXPORT_SYMBOL(drm_mode_get_tile_group);

/**
 * drm_mode_create_tile_group - create a tile group from a displayid description
 * @dev: DRM device
 * @topology: 8-bytes unique per monitor.
 *
 * Create a tile group for the unique monitor, and get a unique
 * identifier for the tile group.
 *
 * RETURNS:
 * new tile group or error.
 */
struct drm_tile_group *drm_mode_create_tile_group(struct drm_device *dev,
						  char topology[8])
{
	struct drm_tile_group *tg;
	int ret;

	tg = kzalloc(sizeof(*tg), GFP_KERNEL);
	if (!tg)
		return ERR_PTR(-ENOMEM);

	kref_init(&tg->refcount);
	memcpy(tg->group_data, topology, 8);
	tg->dev = dev;

	mutex_lock(&dev->mode_config.idr_mutex);
	ret = idr_alloc(&dev->mode_config.tile_idr, tg, 1, 0, GFP_KERNEL);
	if (ret >= 0) {
		tg->id = ret;
	} else {
		kfree(tg);
		tg = ERR_PTR(ret);
	}

	mutex_unlock(&dev->mode_config.idr_mutex);
	return tg;
}
EXPORT_SYMBOL(drm_mode_create_tile_group);

/**
 * drm_crtc_enable_color_mgmt - enable color management properties
 * @crtc: DRM CRTC
 * @degamma_lut_size: the size of the degamma lut (before CSC)
 * @has_ctm: whether to attach ctm_property for CSC matrix
 * @gamma_lut_size: the size of the gamma lut (after CSC)
 *
 * This function lets the driver enable the color correction
 * properties on a CRTC. This includes 3 degamma, csc and gamma
 * properties that userspace can set and 2 size properties to inform
 * the userspace of the lut sizes. Each of the properties are
 * optional. The gamma and degamma properties are only attached if
 * their size is not 0 and ctm_property is only attached if has_ctm is
 * true.
 */
void drm_crtc_enable_color_mgmt(struct drm_crtc *crtc,
				uint degamma_lut_size,
				bool has_ctm,
				uint gamma_lut_size)
{
	struct drm_device *dev = crtc->dev;
	struct drm_mode_config *config = &dev->mode_config;

	if (degamma_lut_size) {
		drm_object_attach_property(&crtc->base,
					   config->degamma_lut_property, 0);
		drm_object_attach_property(&crtc->base,
					   config->degamma_lut_size_property,
					   degamma_lut_size);
	}

	if (has_ctm)
		drm_object_attach_property(&crtc->base,
					   config->ctm_property, 0);

	if (gamma_lut_size) {
		drm_object_attach_property(&crtc->base,
					   config->gamma_lut_property, 0);
		drm_object_attach_property(&crtc->base,
					   config->gamma_lut_size_property,
					   gamma_lut_size);
	}
}
EXPORT_SYMBOL(drm_crtc_enable_color_mgmt);
