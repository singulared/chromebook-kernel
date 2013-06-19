/*
 *
 * (C) COPYRIGHT 2012-2013 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */



#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_jm.h>

#define CREATE_TRACE_POINTS

#ifdef CONFIG_MALI_TRACE_TIMELINE
#include "mali_timeline.h"

struct kbase_trace_timeline_desc
{
	char *enum_str;
	char *desc;
	char *format;
	char *format_desc;
};

struct kbase_trace_timeline_desc kbase_trace_timeline_desc_table[] =
{
	#define KBASE_TIMELINE_TRACE_CODE(enum_val, desc, format, format_desc) { #enum_val, desc, format, format_desc }
	#include "mali_kbase_trace_timeline_defs.h"
	#undef KBASE_TIMELINE_TRACE_CODE
};

#define KBASE_NR_TRACE_CODES ARRAY_SIZE(kbase_trace_timeline_desc_table)

ssize_t show_timeline_defs(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i = 0;

	for (i = 0; i < KBASE_NR_TRACE_CODES; ++i) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%s#%s#%s#%s\n", kbase_trace_timeline_desc_table[i].enum_str, kbase_trace_timeline_desc_table[i].desc, kbase_trace_timeline_desc_table[i].format, kbase_trace_timeline_desc_table[i].format_desc);
	}

	if (PAGE_SIZE == ret) {
		/* we attempted to write more than a page full - truncate */
		buf[PAGE_SIZE - 2] = '\n';
		buf[PAGE_SIZE - 1] = '\0';
		ret = PAGE_SIZE - 1;
	}
	return ret;
}

void kbase_timeline_job_slot_submit(kbase_device *kbdev, kbase_context *kctx,
                                    kbase_jd_atom *katom, int js)
{
	lockdep_assert_held(&kbdev->js_data.runpool_irq.lock);

	if(kbdev->timeline.slot_atoms_submitted[js] > 0) {
		KBASE_TIMELINE_JOB_START_NEXT(kctx, js, 1);
	} else {
		base_atom_id atom_number = kbase_jd_atom_id(kctx, katom);
		KBASE_TIMELINE_JOB_START_HEAD(kctx, js, 1);
		KBASE_TIMELINE_JOB_START(kctx, js, atom_number);
	}
	++kbdev->timeline.slot_atoms_submitted[js];

	KBASE_TIMELINE_ATOMS_SUBMITTED(kctx, js, kbdev->timeline.slot_atoms_submitted[js]);
}

void kbase_timeline_job_slot_done(kbase_device *kbdev, kbase_context *kctx,
                                  kbase_jd_atom *katom, int js,
                                  kbasep_js_atom_done_code done_code)
{
	lockdep_assert_held(&kbdev->js_data.runpool_irq.lock);

	if (done_code & KBASE_JS_ATOM_DONE_EVICTED_FROM_NEXT) {
		KBASE_TIMELINE_JOB_START_NEXT(kctx, js, 0);
	} else {
		/* Job finished in JSn_HEAD */
		base_atom_id atom_number = kbase_jd_atom_id(kctx, katom);
		KBASE_TIMELINE_JOB_START_HEAD(kctx, js, 0);
		KBASE_TIMELINE_JOB_STOP(kctx, js, atom_number);
		/* see if we need to trace the job in JSn_NEXT moving to JSn_HEAD */
		if (kbdev->timeline.slot_atoms_submitted[js] > 1) {
			/* Tag events with next_katom's kctx */
			kbase_jm_slot *slot = &kbdev->jm_slots[js];
			kbase_jd_atom *next_katom;
			kbase_context *next_kctx;
			KBASE_DEBUG_ASSERT(kbasep_jm_nr_jobs_submitted(slot) > 0);

			/* Peek the next atom - note that the atom in JSn_HEAD will already
			 * have been dequeued */
			next_katom = kbasep_jm_peek_idx_submit_slot(slot, 0);
			next_kctx = next_katom->kctx;
			KBASE_TIMELINE_JOB_START_NEXT(next_kctx, js, 0);
			KBASE_TIMELINE_JOB_START_HEAD(next_kctx, js, 1);
			KBASE_TIMELINE_JOB_START(next_kctx, js, kbase_jd_atom_id(next_kctx, next_katom));
		}
	}

	--kbdev->timeline.slot_atoms_submitted[js];

	KBASE_TIMELINE_ATOMS_SUBMITTED(kctx, js, kbdev->timeline.slot_atoms_submitted[js]);
}

void kbase_timeline_pm_send_event(kbase_device *kbdev, kbase_timeline_pm_event event_sent)
{
	int uid = 0;
	int old_uid;

	/* If a producer already exists for the event, try to use their UID (multiple-producers) */
	uid = atomic_read(&kbdev->timeline.pm_event_uid[event_sent]);
	old_uid = uid;

	/* Get a new non-zero UID if we don't have one yet */
	while (!uid)
		uid = atomic_inc_return(&kbdev->timeline.pm_event_uid_counter);

	/* Try to use this UID */
	if ( old_uid != atomic_cmpxchg(&kbdev->timeline.pm_event_uid[event_sent], old_uid, uid))
		/* If it changed, raced with another producer: we've lost this UID */
		uid = 0;

	KBASE_TIMELINE_PM_SEND_EVENT(kbdev, event_sent, uid);
}

void kbase_timeline_pm_check_handle_event(kbase_device *kbdev, kbase_timeline_pm_event event)
{
	int uid = atomic_read(&kbdev->timeline.pm_event_uid[event]);

	if (uid != 0) {
		if (uid != atomic_cmpxchg(&kbdev->timeline.pm_event_uid[event], uid, 0))
			/* If it changed, raced with another consumer: we've lost this UID */
			uid = 0;

		KBASE_TIMELINE_PM_HANDLE_EVENT(kbdev, event, uid);
	}
}

void kbase_timeline_pm_handle_event(kbase_device *kbdev, kbase_timeline_pm_event event)
{
	int uid = atomic_read(&kbdev->timeline.pm_event_uid[event]);

	if (uid != atomic_cmpxchg(&kbdev->timeline.pm_event_uid[event], uid, 0))
		/* If it changed, raced with another consumer: we've lost this UID */
		uid = 0;

	KBASE_TIMELINE_PM_HANDLE_EVENT(kbdev, event, uid);
}

#endif /* CONFIG_MALI_TRACE_TIMELINE */
