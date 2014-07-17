/*
 * drm_sync_helper.h: software fence and helper functions for fences and
 * reservations used for dma buffer access synchronization between drivers.
 *
 * Copyright 2014 Google, Inc.
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

#ifndef _DRM_SYNC_HELPER_H_
#define _DRM_SYNC_HELPER_H_

#include <linux/fence.h>
#include <linux/reservation.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>

struct fence *drm_sw_fence_new(unsigned int context,
			unsigned seqno);

static inline void drm_fence_signal_and_put(struct fence **fence)
{
	if (*fence) {
		fence_signal(*fence);
		fence_put(*fence);
		*fence = NULL;
	}
}

struct drm_reservation_cb;

struct drm_reservation_fence_cb {
	struct fence_cb base;
	struct drm_reservation_cb *parent;
	struct fence *fence;
};

typedef void (*drm_reservation_cb_func_t)(struct drm_reservation_cb *rcb,
					  void *context);

struct drm_reservation_cb {
	struct work_struct work;
	struct drm_reservation_fence_cb **fence_cbs;
	unsigned num_fence_cbs;
	atomic_t count;
	void *context;
	drm_reservation_cb_func_t func;
};

/**
 * Initialize reservation callback
 * @rcb: reservation callback structure to initialize
 * @func: function to call when complete, could be NULL if intended to use with
 *    wait
 * @context: parameter to call func with
 */
void drm_reservation_cb_init(struct drm_reservation_cb *rcb,
			     drm_reservation_cb_func_t func,
			     void *context);

/**
 * Add fences from reservation object to callback
 * @rcb: reservation callback structure
 * @resv: reservation object
 * @exclusive: (for exclusive wait) when true add all fences, otherwise only
 *    exclusive fence
 */
int drm_reservation_cb_add(struct drm_reservation_cb *rcb,
			   struct reservation_object *resv,
			    bool exclusive);

/**
 * Finish adding fences
 * @rcb: reservation callback structure
 */
void drm_reservation_cb_done(struct drm_reservation_cb *rcb);

/**
 * Cleanup reservation callback structure
 * @rcb: reservation callback structure
 */
void drm_reservation_cb_fini(struct drm_reservation_cb *rcb);

void
drm_add_reservation(struct reservation_object *resv,
			struct reservation_object **resvs,
			unsigned long *excl_resvs_bitmap,
			unsigned int *num_resvs, bool exclusive);

int drm_lock_reservations(struct reservation_object **resvs,
			unsigned int num_resvs, struct ww_acquire_ctx *ctx);

void drm_unlock_reservations(struct reservation_object **resvs,
				unsigned int num_resvs,
				struct ww_acquire_ctx *ctx);

#endif
