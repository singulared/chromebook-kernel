/*
 *
 * (C) COPYRIGHT 2010-2013 ARM Limited. All rights reserved.
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



/**
 * @file mali_kbase_pm.c
 * Base kernel power management APIs
 */

#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_midg_regmap.h>

#include <kbase/src/common/mali_kbase_pm.h>

/* Policy operation structures */
extern const kbase_pm_policy kbase_pm_always_on_policy_ops;
extern const kbase_pm_policy kbase_pm_demand_policy_ops;
extern const kbase_pm_policy kbase_pm_coarse_demand_policy_ops;

/** A list of the power policies available in the system */
static const kbase_pm_policy *const policy_list[] = {
#ifdef CONFIG_MALI_NO_MALI
	&kbase_pm_always_on_policy_ops,
	&kbase_pm_coarse_demand_policy_ops,
	&kbase_pm_demand_policy_ops
#else				/* CONFIG_MALI_NO_MALI */
	&kbase_pm_demand_policy_ops,
	&kbase_pm_coarse_demand_policy_ops,
	&kbase_pm_always_on_policy_ops
#endif				/* CONFIG_MALI_NO_MALI */
};

/** The number of policies available in the system.
 * This is derived from the number of functions listed in policy_get_functions.
 */
#define POLICY_COUNT (sizeof(policy_list)/sizeof(*policy_list))

void kbase_pm_register_access_enable(kbase_device *kbdev)
{
	kbase_pm_callback_conf *callbacks;

	callbacks = (kbase_pm_callback_conf *) kbasep_get_config_value(kbdev, kbdev->config_attributes, KBASE_CONFIG_ATTR_POWER_MANAGEMENT_CALLBACKS);

	if (callbacks)
		callbacks->power_on_callback(kbdev);
}

void kbase_pm_register_access_disable(kbase_device *kbdev)
{
	kbase_pm_callback_conf *callbacks;

	callbacks = (kbase_pm_callback_conf *) kbasep_get_config_value(kbdev, kbdev->config_attributes, KBASE_CONFIG_ATTR_POWER_MANAGEMENT_CALLBACKS);

	if (callbacks)
		callbacks->power_off_callback(kbdev);
}

mali_error kbase_pm_init(kbase_device *kbdev)
{
	mali_error ret = MALI_ERROR_NONE;
	kbase_pm_callback_conf *callbacks;

	KBASE_DEBUG_ASSERT(kbdev != NULL);

	mutex_init(&kbdev->pm.lock);

	kbdev->pm.gpu_powered = MALI_FALSE;
	kbdev->pm.suspending = MALI_FALSE;
#ifdef CONFIG_MALI_DEBUG
	kbdev->pm.driver_ready_for_irqs = MALI_FALSE;
#endif /* CONFIG_MALI_DEBUG */
	kbdev->pm.gpu_in_desired_state = MALI_TRUE;
	init_waitqueue_head(&kbdev->pm.gpu_in_desired_state_wait);

	callbacks = (kbase_pm_callback_conf *) kbasep_get_config_value(kbdev, kbdev->config_attributes, KBASE_CONFIG_ATTR_POWER_MANAGEMENT_CALLBACKS);
	if (callbacks) {
		kbdev->pm.callback_power_on = callbacks->power_on_callback;
		kbdev->pm.callback_power_off = callbacks->power_off_callback;
		kbdev->pm.callback_power_suspend =
					callbacks->power_suspend_callback;
		kbdev->pm.callback_power_resume =
					callbacks->power_resume_callback;
		kbdev->pm.callback_power_runtime_init = callbacks->power_runtime_init_callback;
		kbdev->pm.callback_power_runtime_term = callbacks->power_runtime_term_callback;
		kbdev->pm.callback_power_runtime_on = callbacks->power_runtime_on_callback;
		kbdev->pm.callback_power_runtime_off = callbacks->power_runtime_off_callback;
	} else {
		kbdev->pm.callback_power_on = NULL;
		kbdev->pm.callback_power_off = NULL;
		kbdev->pm.callback_power_suspend = NULL;
		kbdev->pm.callback_power_resume = NULL;
		kbdev->pm.callback_power_runtime_init = NULL;
		kbdev->pm.callback_power_runtime_term = NULL;
		kbdev->pm.callback_power_runtime_on = NULL;
		kbdev->pm.callback_power_runtime_off = NULL;
	}

	kbdev->pm.platform_dvfs_frequency = (u32) kbasep_get_config_value(kbdev, kbdev->config_attributes, KBASE_CONFIG_ATTR_POWER_MANAGEMENT_DVFS_FREQ);

	/* Initialise the metrics subsystem */
	ret = kbasep_pm_metrics_init(kbdev);
	if (MALI_ERROR_NONE != ret)
		return ret;

	init_waitqueue_head(&kbdev->pm.l2_powered_wait);
	kbdev->pm.l2_powered = 0;

	init_waitqueue_head(&kbdev->pm.reset_done_wait);
	kbdev->pm.reset_done = MALI_FALSE;

	init_waitqueue_head(&kbdev->pm.zero_active_count_wait);
	kbdev->pm.active_count = 0;

	spin_lock_init(&kbdev->pm.power_change_lock);
	spin_lock_init(&kbdev->pm.gpu_cycle_counter_requests_lock);
	spin_lock_init(&kbdev->pm.gpu_powered_lock);
	return MALI_ERROR_NONE;
}

KBASE_EXPORT_TEST_API(kbase_pm_init)

/**
 * Core state function that clears all desired states.
 */
STATIC void kbasep_pm_core_state_func_clearall(struct kbase_device *kbdev)
{
	kbdev->pm.desired_shader_state = 0u;
	kbdev->pm.desired_tiler_state = 0u;
}

/**
 * Set the new desired state from the given core_state_func, and then start
 * transitioning cores if the desired state changed.
 */
STATIC void kbasep_pm_set_desired_and_update_state(struct kbase_device *kbdev,
                                                   kbase_pm_core_state_func *core_state_func)
{
	u64 prev_shader_state = kbdev->pm.desired_shader_state;
	u64 prev_tiler_state = kbdev->pm.desired_tiler_state;
	lockdep_assert_held(&kbdev->pm.power_change_lock);

	core_state_func(kbdev);

	/* Optimize out redundant state transitions: only transition when we
	 * have something to do */
	if (prev_shader_state != kbdev->pm.desired_shader_state
		|| prev_tiler_state != kbdev->pm.desired_tiler_state) {
		mali_bool cores_are_available;
		if (prev_shader_state != kbdev->pm.desired_shader_state)
			KBASE_TRACE_ADD(kbdev, PM_CORES_CHANGE_DESIRED, NULL, NULL, 0u, (u32) kbdev->pm.desired_shader_state);
		if (prev_tiler_state != kbdev->pm.desired_tiler_state)
			KBASE_TRACE_ADD(kbdev, PM_CORES_CHANGE_DESIRED_TILER, NULL, NULL, 0u, (u32) kbdev->pm.desired_tiler_state);
		cores_are_available = kbase_pm_check_transitions_nolock(kbdev);
		/* Don't need 'cores_are_available', because we don't return anything */
		CSTD_UNUSED(cores_are_available);
	}
}

void kbase_pm_update_cores_state_nolock(struct kbase_device *kbdev, kbase_pm_policy_func policy_func)
{
	lockdep_assert_held(&kbdev->pm.power_change_lock);
	KBASE_DEBUG_ASSERT(0 <= policy_func && policy_func < KBASE_PM_POLICY_FUNC_COUNT);
	/* NOTE: if there isn't a current policy, kbase_pm_set_policy() will retry
	 * the core_state_func[policy_func] and kbase_pm_check_transitions_nolock()
	 * calls anyway */

	if (kbdev->pm.current_policy)
		kbasep_pm_set_desired_and_update_state(kbdev,
		                                       kbdev->pm.current_policy->core_state_func[policy_func]);
}

void kbase_pm_update_cores_state(struct kbase_device *kbdev, kbase_pm_policy_func policy_func)
{
	unsigned long flags;
	spin_lock_irqsave(&kbdev->pm.power_change_lock, flags);
	kbase_pm_update_cores_state_nolock(kbdev, policy_func);
	spin_unlock_irqrestore(&kbdev->pm.power_change_lock, flags);
}

/**
 * Power on the GPU, and optionally the cores too
 *
 * If the GPU or cores are already switched on, then the state of those on
 * component(s) are not modified.
 */
STATIC void kbase_pm_do_poweron(kbase_device *kbdev, mali_bool is_resume)
{
	lockdep_assert_held(&kbdev->pm.lock);

	/* Turn clocks and interrupts on - no-op if we haven't done a previous
	 * kbase_pm_clock_off() */
	kbase_pm_clock_on(kbdev, is_resume);

	/* Turn on any cores the policy needs */
	kbase_pm_update_cores_state(kbdev, KBASE_PM_POLICY_FUNC_ON_ACTIVE_CORE_STATE);

	/* NOTE: We don't wait to reach the desired state, since running atoms
	 * will wait for that state to be reached anyway */
}

/**
 * Power off the GPU and/or any cores requested by the Power Policy
 *
 * When KBASE_PM_POLICY_FLAG_KEEP_GPU_POWERED is not set in poweroff_flags,
 * this forces both the cores and the GPU off - regardless of whether the
 * policy has this flag set.
 *
 * If the GPU or cores are already switched off, then the state off those off
 * component(s) are not modified.
 */
STATIC void kbase_pm_do_poweroff(kbase_device *kbdev,
		kbase_pm_policy_flags poweroff_flags, mali_bool is_suspend)
{
	lockdep_assert_held(&kbdev->pm.lock);
	KBASE_DEBUG_ASSERT( !(poweroff_flags &
	                      ~(kbase_pm_policy_flags)(KBASE_PM_POLICY_FLAG_KEEP_GPU_POWERED)));

	/* NOTE: We won't wait to reach the core's desired state, even if we're
	 * powering off the GPU itself too. It's safe to cut the power whilst
	 * they're transitioning to off, because the cores should be idle and all
	 * cache flushes should already have occurred */

	if ((poweroff_flags & KBASE_PM_POLICY_FLAG_KEEP_GPU_POWERED)) {
		/* Keep the GPU powered, but turn off any cores the policy doesn't need */
		kbase_pm_update_cores_state(kbdev, KBASE_PM_POLICY_FUNC_ON_IDLE_CORE_STATE);

		/* Consume any change-state events */
		kbase_timeline_pm_check_handle_event(kbdev, KBASE_TIMELINE_PM_EVENT_GPU_STATE_CHANGED);
	} else {
		/* The GPU must be powered off; regardless of what the policy says, we
		 * must power off cores and the GPU */
		unsigned long flags;

		spin_lock_irqsave(&kbdev->pm.power_change_lock, flags);
		/* Force off the cores */
		kbasep_pm_set_desired_and_update_state(kbdev,
		                                       &kbasep_pm_core_state_func_clearall);
		spin_unlock_irqrestore(&kbdev->pm.power_change_lock, flags);

		/* Consume any change-state events */
		kbase_timeline_pm_check_handle_event(kbdev, KBASE_TIMELINE_PM_EVENT_GPU_STATE_CHANGED);
		/* Disable interrupts and turn the clock off */
		kbase_pm_clock_off(kbdev, is_suspend);
	}
}

mali_error kbase_pm_powerup(kbase_device *kbdev)
{
	unsigned long flags;
	mali_error ret;

	KBASE_DEBUG_ASSERT(kbdev != NULL);

	mutex_lock(&kbdev->pm.lock);

	/* A suspend won't happen during startup/insmod */
	KBASE_DEBUG_ASSERT(!kbase_pm_is_suspending(kbdev));

	/* Power up the GPU, don't enable IRQs as we are not ready to receive them. */
	ret = kbase_pm_init_hw(kbdev, MALI_FALSE );
	if (ret != MALI_ERROR_NONE)
		return ret;

	kbasep_pm_read_present_cores(kbdev);

	/* Pretend the GPU is active to prevent a power policy turning the GPU cores off */
	kbdev->pm.active_count = 1;

	spin_lock_irqsave(&kbdev->pm.gpu_cycle_counter_requests_lock, flags);
	/* Ensure cycle counter is off */
	kbdev->pm.gpu_cycle_counter_requests = 0;
	kbase_reg_write(kbdev, GPU_CONTROL_REG(GPU_COMMAND), GPU_COMMAND_CYCLE_COUNT_STOP, NULL);
	spin_unlock_irqrestore(&kbdev->pm.gpu_cycle_counter_requests_lock, flags);

	kbdev->pm.current_policy = policy_list[0];
	KBASE_TRACE_ADD(kbdev, PM_CURRENT_POLICY_INIT, NULL, NULL, 0u, kbdev->pm.current_policy->id);

	kbdev->pm.current_policy->init(kbdev);

	/* We are ready to receive IRQ's now as power policy is set up, so enable them now. */
#ifdef CONFIG_MALI_DEBUG
	spin_lock_irqsave(&kbdev->pm.gpu_powered_lock, flags);
	kbdev->pm.driver_ready_for_irqs = MALI_TRUE;
	spin_unlock_irqrestore(&kbdev->pm.gpu_powered_lock, flags);
#endif
	kbase_pm_enable_interrupts(kbdev);

	/* Turn on the GPU and any cores needed by the policy */
	kbase_pm_do_poweron(kbdev, MALI_FALSE);
	mutex_unlock(&kbdev->pm.lock);

	/* Idle the GPU and/or cores, if the policy wants it to */
	kbase_pm_context_idle(kbdev);

	return MALI_ERROR_NONE;
}

KBASE_EXPORT_TEST_API(kbase_pm_powerup)

void kbase_pm_context_active(kbase_device *kbdev)
{
	(void)kbase_pm_context_active_handle_suspend(kbdev, KBASE_PM_SUSPEND_HANDLER_NOT_POSSIBLE);
}

int kbase_pm_context_active_handle_suspend(kbase_device *kbdev, kbase_pm_suspend_handler suspend_handler)
{
	int c;
	int old_count;

	KBASE_DEBUG_ASSERT(kbdev != NULL);

	/* Trace timeline information about how long it took to handle the decision
	 * to powerup. Sometimes the event might be missed due to reading the count
	 * outside of mutex, but this is necessary to get the trace timing
	 * correct. */
	old_count = kbdev->pm.active_count;
	if (old_count == 0)
		kbase_timeline_pm_send_event(kbdev, KBASE_TIMELINE_PM_EVENT_GPU_ACTIVE);

	mutex_lock(&kbdev->pm.lock);
	if (kbase_pm_is_suspending(kbdev))
	{
		switch (suspend_handler) {
		case KBASE_PM_SUSPEND_HANDLER_DONT_REACTIVATE:
			if (kbdev->pm.active_count != 0 )
				break;
			/* FALLTHROUGH */
		case KBASE_PM_SUSPEND_HANDLER_DONT_INCREASE:
			mutex_unlock(&kbdev->pm.lock);
			if (old_count == 0)
				kbase_timeline_pm_handle_event(kbdev, KBASE_TIMELINE_PM_EVENT_GPU_ACTIVE);
			return 1;

		case KBASE_PM_SUSPEND_HANDLER_NOT_POSSIBLE:
			/* FALLTHROUGH */
		default:
			KBASE_DEBUG_ASSERT_MSG(MALI_FALSE,"unreachable");
			break;
		}
	}
	c = ++kbdev->pm.active_count;

	KBASE_TRACE_ADD_REFCOUNT(kbdev, PM_CONTEXT_ACTIVE, NULL, NULL, 0u, c);

	/* Trace the event being handled */
	if (old_count == 0)
		kbase_timeline_pm_handle_event(kbdev, KBASE_TIMELINE_PM_EVENT_GPU_ACTIVE);

	if (c == 1) {
		/* First context active: Power on the GPU and any cores requested by
		 * the policy */
		kbase_pm_do_poweron(kbdev, MALI_FALSE);

		kbasep_pm_record_gpu_active(kbdev);
	}

	mutex_unlock(&kbdev->pm.lock);
	return 0;
}

KBASE_EXPORT_TEST_API(kbase_pm_context_active)

void kbase_pm_context_idle(kbase_device *kbdev)
{
	int c;
	int old_count;

	KBASE_DEBUG_ASSERT(kbdev != NULL);

	/* Trace timeline information about how long it took to handle the decision
	 * to powerdown. Sometimes the event might be missed due to reading the
	 * count outside of mutex, but this is necessary to get the trace timing
	 * correct. */
	old_count = kbdev->pm.active_count;
	if (old_count == 0)
		kbase_timeline_pm_send_event(kbdev, KBASE_TIMELINE_PM_EVENT_GPU_IDLE);

	mutex_lock(&kbdev->pm.lock);

	c = --kbdev->pm.active_count;

	KBASE_TRACE_ADD_REFCOUNT(kbdev, PM_CONTEXT_IDLE, NULL, NULL, 0u, c);

	KBASE_DEBUG_ASSERT(c >= 0);

	/* Trace the event being handled */
	if (old_count == 0)
		kbase_timeline_pm_handle_event(kbdev, KBASE_TIMELINE_PM_EVENT_GPU_IDLE);

	if (c == 0) {
		kbase_pm_policy_flags poweroff_flags = kbdev->pm.current_policy->flags &
		                                       KBASE_PM_POLICY_FLAG_KEEP_GPU_POWERED;

		/* Last context has gone idle */
		kbasep_pm_record_gpu_idle(kbdev);

		/* Powerdown only what the policy wishes to powerdown */
		kbase_pm_do_poweroff(kbdev, poweroff_flags, MALI_FALSE);

		/* Wake up anyone waiting for this to become 0 (e.g. suspend). The
		 * waiters must synchronize with us by locking the pm.lock after
		 * waiting */
		wake_up(&kbdev->pm.zero_active_count_wait);
	}

	mutex_unlock(&kbdev->pm.lock);
}

KBASE_EXPORT_TEST_API(kbase_pm_context_idle)

void kbase_pm_halt(kbase_device *kbdev)
{
	KBASE_DEBUG_ASSERT(kbdev != NULL);

	mutex_lock(&kbdev->pm.lock);
	if (kbdev->pm.current_policy != NULL) {
		/* Turn the GPU off and all the cores, regardless of whether or not the
		 * policy keeps them on */
		kbase_pm_do_poweroff(kbdev, 0u, MALI_FALSE);
	}
	mutex_unlock(&kbdev->pm.lock);
}

KBASE_EXPORT_TEST_API(kbase_pm_halt)

void kbase_pm_term(kbase_device *kbdev)
{
	KBASE_DEBUG_ASSERT(kbdev != NULL);
	KBASE_DEBUG_ASSERT(kbdev->pm.active_count == 0);
	KBASE_DEBUG_ASSERT(kbdev->pm.gpu_cycle_counter_requests == 0);

	if (kbdev->pm.current_policy != NULL) {
		/* Free any resources the policy allocated */
		KBASE_TRACE_ADD(kbdev, PM_CURRENT_POLICY_TERM, NULL, NULL, 0u, kbdev->pm.current_policy->id);
		kbdev->pm.current_policy->term(kbdev);
	}

	/* Shut down the metrics subsystem */
	kbasep_pm_metrics_term(kbdev);
}

KBASE_EXPORT_TEST_API(kbase_pm_term)

int kbase_pm_list_policies(const kbase_pm_policy * const **list)
{
	if (!list)
		return POLICY_COUNT;

	*list = policy_list;

	return POLICY_COUNT;
}

KBASE_EXPORT_TEST_API(kbase_pm_list_policies)

const kbase_pm_policy *kbase_pm_get_policy(kbase_device *kbdev)
{
	KBASE_DEBUG_ASSERT(kbdev != NULL);

	return kbdev->pm.current_policy;
}

KBASE_EXPORT_TEST_API(kbase_pm_get_policy)

void kbase_pm_set_policy(kbase_device *kbdev, const kbase_pm_policy *new_policy)
{
	const kbase_pm_policy * old_policy;
	unsigned long flags;
	KBASE_DEBUG_ASSERT(kbdev != NULL);
	KBASE_DEBUG_ASSERT(new_policy != NULL);

	KBASE_TRACE_ADD(kbdev, PM_SET_POLICY, NULL, NULL, 0u, new_policy->id);

	/* During a policy change we pretend the GPU is active */
	/* A suspend won't happen here, because we're in a syscall from a userspace thread */
	kbase_pm_context_active(kbdev);

	mutex_lock(&kbdev->pm.lock);

	/* Remove the policy to prevent IRQ handlers from working on it */
	spin_lock_irqsave(&kbdev->pm.power_change_lock, flags);
	old_policy = kbdev->pm.current_policy;
	kbdev->pm.current_policy = NULL;
	spin_unlock_irqrestore(&kbdev->pm.power_change_lock, flags);

	KBASE_TRACE_ADD(kbdev, PM_CURRENT_POLICY_TERM, NULL, NULL, 0u, old_policy->id);
	old_policy->term(kbdev);

	KBASE_TRACE_ADD(kbdev, PM_CURRENT_POLICY_INIT, NULL, NULL, 0u, new_policy->id);
	new_policy->init(kbdev);

	spin_lock_irqsave(&kbdev->pm.power_change_lock, flags);
	kbdev->pm.current_policy = new_policy;
	spin_unlock_irqrestore(&kbdev->pm.power_change_lock, flags);

	/* Force the GPU on, and optionally any cores if the new policy requires them */
	kbase_pm_do_poweron(kbdev, MALI_FALSE);

	/* If any core power state changes were previously attempted, but couldn't
	 * be made because the policy was changing (current_policy was NULL), then
	 * re-try them here. KBASE_PM_POLICY_FUNC_ATOM_CORE_STATE is the only one
	 * that can happen, because a) we've done kbase_pm_context_active() above,
	 * and b) we hold kbdev->pm.lock */
	kbase_pm_update_cores_state(kbdev, KBASE_PM_POLICY_FUNC_ATOM_CORE_STATE);

	mutex_unlock(&kbdev->pm.lock);

	/* Now the policy change is finished, we release our fake context active reference */
	kbase_pm_context_idle(kbdev);
}

KBASE_EXPORT_TEST_API(kbase_pm_set_policy)

void kbase_pm_suspend(struct kbase_device *kbdev)
{
	int nr_keep_gpu_powered_ctxs;
	KBASE_DEBUG_ASSERT(kbdev);

	mutex_lock(&kbdev->pm.lock);
	KBASE_DEBUG_ASSERT(!kbase_pm_is_suspending(kbdev));
	kbdev->pm.suspending = MALI_TRUE;
	mutex_unlock(&kbdev->pm.lock);

	/* From now on, the active count will drop towards zero. Sometimes, it'll
	 * go up briefly before going down again. However, once it reaches zero it
	 * will stay there - guaranteeing that we've idled all pm references */

	/* Suspend job scheduler and associated components, so that it releases all
	 * the PM active count references */
	kbasep_js_suspend(kbdev);

	/* Suspend any counter collection that might be happening */
	kbase_instr_hwcnt_suspend(kbdev);

	/* Cancel the keep_gpu_powered calls */
	for (nr_keep_gpu_powered_ctxs = atomic_read(&kbdev->keep_gpu_powered_count);
		 nr_keep_gpu_powered_ctxs > 0 ;
		 --nr_keep_gpu_powered_ctxs ) {
		kbase_pm_context_idle(kbdev);
	}

	/* Wait for the active count to reach zero. This is not the same as
	 * waiting for a power down, since not all policies power down when this
	 * reaches zero. */
	wait_event(kbdev->pm.zero_active_count_wait, kbdev->pm.active_count == 0);

	/* Suspend PM Metric timer on system suspend.
	 * It is ok if kbase_pm_context_idle() is still running, it is safe
	 * to still complete the last active time period - the pm stats will
	 * get reset on resume anyway.
	 */
	kbasep_pm_metrics_suspend(kbdev);

	/* NOTE: We synchronize with anything that was just finishing a
	 * kbase_pm_context_idle() call by locking the pm.lock below */

	/* Force power off the GPU and all cores (regardless of policy), only after
	 * the PM active count reaches zero (otherwise, we risk turning it off
	 * prematurely) */
	mutex_lock(&kbdev->pm.lock);
	kbase_pm_do_poweroff(kbdev, 0u, MALI_TRUE);
	mutex_unlock(&kbdev->pm.lock);
}

void kbase_pm_resume(struct kbase_device *kbdev)
{
	int nr_keep_gpu_powered_ctxs;

	/* MUST happen before any pm_context_active calls occur */
	mutex_lock(&kbdev->pm.lock);
	kbdev->pm.suspending = MALI_FALSE;
	mutex_unlock(&kbdev->pm.lock);

	kbase_pm_do_poweron(kbdev, MALI_TRUE);

	/* Restart PM Metric timer on resume */
	kbasep_pm_metrics_resume(kbdev);

	/* Initial active call, to power on the GPU/cores if needed */
	kbase_pm_context_active(kbdev);

	/* Restore the keep_gpu_powered calls */
	for (nr_keep_gpu_powered_ctxs = atomic_read(&kbdev->keep_gpu_powered_count);
		 nr_keep_gpu_powered_ctxs > 0 ;
		 --nr_keep_gpu_powered_ctxs ) {
		kbase_pm_context_active(kbdev);
	}

	/* Re-enable instrumentation, if it was previously disabled */
	kbase_instr_hwcnt_resume(kbdev);

	/* Resume any blocked atoms (which may cause contexts to be scheduled in
	 * and dependent atoms to run) */
	kbase_resume_suspended_soft_jobs(kbdev);

	/* Resume the Job Scheduler and associated components, and start running
	 * atoms */
	kbasep_js_resume(kbdev);

	/* Matching idle call, to power off the GPU/cores if we didn't actually
	 * need it and the policy doesn't want it on */
	kbase_pm_context_idle(kbdev);
}
