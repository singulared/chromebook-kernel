/*
 *
 * (C) COPYRIGHT 2012 ARM Limited. All rights reserved.
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
 * @file mali_kbase_pm_coarse_demand.c
 * "Coarse Demand" power management policy
 */

#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_pm.h>

/**
 * Policy function which serves 2 roles in one, determining the cores desired
 * in the following situations:
 * # When running atoms
 * # When switching from idle->active (atom comes in whilst we're idle)
 *
 * This policy activates/keeps all cores on in both of those situations.
 */
static void coarse_demand_change_gpu_state(kbase_device *kbdev)
{
	/* The following always exceeds the 'needed' and 'inuse' requests */
	u64 new_shader_desired = kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_SHADER);
	u64 new_tiler_desired = kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_TILER);

	kbdev->pm.desired_shader_state = new_shader_desired;
	kbdev->pm.desired_tiler_state = new_tiler_desired;
}

/**
 * Policy function determining the cores desired in the following situations:
 * # When switching from active->idle (last atom finishes, until we get more
 *   atoms)
 *
 * Since this policy does not keep the GPU powered, it powers off all cores.
 */
static void coarse_demand_on_idle_change_gpu_state(kbase_device *kbdev)
{
	/* Update the bitmap of the cores we need */
	u64 new_shader_desired = 0u;
	u64 new_tiler_desired = 0u;

	kbdev->pm.desired_shader_state = new_shader_desired;
	kbdev->pm.desired_tiler_state = new_tiler_desired;
}

/**
 * Initialize the coarse_demand power policy
 *
 * @param kbdev     The kbase device structure for the device
 */
static void coarse_demand_init(kbase_device *kbdev)
{
	CSTD_UNUSED(kbdev);
}

/**
 * Terminate the coarse_demand power policy
 *
 * This frees the resources that were allocated by @ref coarse_demand_init.
 *
 * @param kbdev     The kbase device structure for the device
 */
static void coarse_demand_term(kbase_device *kbdev)
{
	CSTD_UNUSED(kbdev);
}

/**
 * The @ref kbase_pm_policy structure for the coarse_demand power policy
 *
 * This is the extern structure that defines the coarse_demand power policy's callback and name.
 */
const kbase_pm_policy kbase_pm_coarse_demand_policy_ops = {
	.name = "coarse_demand",
	.init = coarse_demand_init,
	.term = coarse_demand_term,
	.core_state_func = {
		[KBASE_PM_POLICY_FUNC_ATOM_CORE_STATE] = coarse_demand_change_gpu_state,
		[KBASE_PM_POLICY_FUNC_ON_ACTIVE_CORE_STATE] = coarse_demand_change_gpu_state,
		[KBASE_PM_POLICY_FUNC_ON_IDLE_CORE_STATE] = coarse_demand_on_idle_change_gpu_state
	},
	.flags = 0u,
	.id = KBASE_PM_POLICY_ID_COARSE_DEMAND
};

KBASE_EXPORT_TEST_API(kbase_pm_coarse_demand_policy_ops)
