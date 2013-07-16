/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2013 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file mali_kbase_pm_ca_fixed.h
 * A power policy implementing fixed core availability
 */

#ifndef MALI_KBASE_PM_CA_FIXED_H
#define MALI_KBASE_PM_CA_FIXED_H

/**
 * Private structure for policy instance data.
 *
 * This contains data that is private to the particular power policy that is active.
 */
typedef struct kbasep_pm_ca_policy_fixed {
	/** No state needed - just have a dummy variable here */
	int dummy;
} kbasep_pm_ca_policy_fixed;

#endif /* MALI_KBASE_PM_CA_FIXED_H */

