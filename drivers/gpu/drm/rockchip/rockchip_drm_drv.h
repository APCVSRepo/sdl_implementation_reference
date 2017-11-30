/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:Mark Yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_drv.h
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _ROCKCHIP_DRM_DRV_H
#define _ROCKCHIP_DRM_DRV_H

#include <drm/drm_fb_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem.h>

#include <linux/module.h>
#include <linux/component.h>

#define ROCKCHIP_MAX_FB_BUFFER	3
#define ROCKCHIP_MAX_CONNECTOR	2
#define ROCKCHIP_MAX_CRTC	2

struct drm_device;
struct drm_connector;
struct iommu_domain;

/*
 * Rockchip drm private crtc funcs.
 * @loader_protect: protect loader logo crtc's power
 * @enable_vblank: enable crtc vblank irq.
 * @disable_vblank: disable crtc vblank irq.
 */
struct rockchip_crtc_funcs {
	int (*loader_protect)(struct drm_crtc *crtc, bool on);
	int (*enable_vblank)(struct drm_crtc *crtc);
	void (*disable_vblank)(struct drm_crtc *crtc);
	void (*wait_for_update)(struct drm_crtc *crtc);
	void (*cancel_pending_vblank)(struct drm_crtc *crtc, struct drm_file *file_priv);
	int (*debugfs_dump)(struct drm_crtc *crtc, struct seq_file *s);
	enum drm_mode_status (*mode_valid)(struct drm_crtc *crtc,
					   const struct drm_display_mode *mode,
					   int output_type);
};

struct drm_rockchip_subdrv {
	struct list_head list;
	struct device *dev;
	struct drm_device *drm_dev;

	int (*open)(struct drm_device *drm_dev, struct device *dev,
		    struct drm_file *file);
	void (*close)(struct drm_device *drm_dev, struct device *dev,
		      struct drm_file *file);
};

struct rockchip_atomic_commit {
	struct work_struct	work;
	struct drm_atomic_state *state;
	struct drm_device *dev;
	struct mutex lock;
};

struct rockchip_crtc_state {
	struct drm_crtc_state base;
	int left_margin;
	int right_margin;
	int top_margin;
	int bottom_margin;
	int afbdc_win_format;
	int afbdc_win_width;
	int afbdc_win_height;
	int afbdc_win_ptr;
	int afbdc_win_id;
	int afbdc_en;
	int dsp_layer_sel;
	int output_type;
	int output_mode;
	int bus_format;
};

#define to_rockchip_crtc_state(s) \
		container_of(s, struct rockchip_crtc_state, base)

/*
 * Rockchip drm_file private structure.
 *
 * @gem_cpu_acquire_list: list of GEM objects we hold acquires on
 */
struct rockchip_drm_file_private {
	struct list_head		gem_cpu_acquire_list;
	struct rockchip_drm_rga_private *rga_priv;
};

struct rockchip_logo {
	struct sg_table *sgt;
	struct drm_mm_node mm;
	dma_addr_t dma_addr;
	phys_addr_t start;
	phys_addr_t size;
	size_t iommu_map_size;
	int count;
};

/*
 * Rockchip drm private structure.
 *
 * @crtc: array of enabled CRTCs, used to map from "pipe" to drm_crtc.
 * @num_pipe: number of pipes for this device.
 * @cpu_fence_context: fence context used for CPU acquire/release
 * @cpu_fence_seqno: fence sequence number
 */
struct rockchip_drm_private {
	struct rockchip_logo *logo;
	struct drm_fb_helper *fbdev_helper;
	struct drm_gem_object *fbdev_bo;
	const struct rockchip_crtc_funcs *crtc_funcs[ROCKCHIP_MAX_CRTC];
	struct drm_atomic_state *state;

	struct rockchip_atomic_commit commit;
	struct iommu_domain *domain;
#ifdef CONFIG_DRM_DMA_SYNC
	unsigned int cpu_fence_context;
	atomic_t cpu_fence_seqno;
#endif
	/* protect drm_mm on multi-threads */
	struct mutex mm_lock;
	struct drm_mm mm;
};

#ifndef MODULE
void rockchip_free_loader_memory(struct drm_device *drm);
#endif
int rockchip_drm_crtc_mode_config(struct drm_crtc *crtc,
                                  int connector_type,
                                  int out_mode);
int rockchip_drm_encoder_get_mux_id(struct device_node *node,
                                    struct drm_encoder *encoder);
void rockchip_drm_atomic_work(struct work_struct *work);
int rockchip_register_crtc_funcs(struct drm_crtc *crtc,
				 const struct rockchip_crtc_funcs *crtc_funcs);
void rockchip_unregister_crtc_funcs(struct drm_crtc *crtc);
int rockchip_drm_dma_attach_device(struct drm_device *drm_dev,
				   struct device *dev);
void rockchip_drm_dma_detach_device(struct drm_device *drm_dev,
				    struct device *dev);
int rockchip_drm_wait_line_flag(struct drm_crtc *crtc, unsigned int line_num,
				unsigned int mstimeout);

int rockchip_drm_register_subdrv(struct drm_rockchip_subdrv *subdrv);
int rockchip_drm_unregister_subdrv(struct drm_rockchip_subdrv *subdrv);

#endif /* _ROCKCHIP_DRM_DRV_H_ */
