/*
 *
 * (C) COPYRIGHT 2010-2012 ARM Limited. All rights reserved.
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
 * @file mali_kbase_pm_always_on.c
 * "Always on" power management policy
 */

#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_pm.h>

/**
 * Policy function which serves 3 roles in one, determining the cores desired
 * in the following situations:
 * # When running atoms
 * # When switching from idle->active (atom comes in whilst we're idle)
 * # When switching from active->idle (last atom finishes, until we get more
 *   atoms)
 *
 * This policy keeps all cores on in all of those situations. This is allowed
 * in the third situation, since this policy also keeps the GPU powered.
 */
static void always_on_change_gpu_state(kbase_device *kbdev)
{
	/* The following always exceeds the 'needed' and 'inuse' requests */
	u64 new_shader_desired = kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_SHADER);
	u64 new_tiler_desired = kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_TILER);

	kbdev->pm.desired_shader_state = new_shader_desired;
	kbdev->pm.desired_tiler_state = new_tiler_desired;
}
/**
 * Initialize the always_on power policy
 *
 * @param kbdev     The kbase device structure for the device
 */
static void always_on_init(kbase_device *kbdev)
{
	CSTD_UNUSED(kbdev);
}

/**
 * Terminate the always_on power policy
 *
 * This frees the resources that were allocated by @ref always_on_init.
 *
 * @param kbdev     The kbase device structure for the device
 */
static void always_on_term(kbase_device *kbdev)
{
	CSTD_UNUSED(kbdev);
}

/**
 * The @ref kbase_pm_policy structure for the always_on power policy
 *
 * This is the extern structure that defines the always_on power policy's callback and name.
 */
const kbase_pm_policy kbase_pm_always_on_policy_ops = {
	.name = "always_on",
	.init = always_on_init,
	.term = always_on_term,
	.core_state_func = {
		[KBASE_PM_POLICY_FUNC_ATOM_CORE_STATE] = always_on_change_gpu_state,
		[KBASE_PM_POLICY_FUNC_ON_ACTIVE_CORE_STATE] = always_on_change_gpu_state,
		[KBASE_PM_POLICY_FUNC_ON_IDLE_CORE_STATE] = always_on_change_gpu_state
	},
	.flags = KBASE_PM_POLICY_FLAG_KEEP_GPU_POWERED,
	.id = KBASE_PM_POLICY_ID_ALWAYS_ON
};

KBASE_EXPORT_TEST_API(kbase_pm_always_on_policy_ops)
