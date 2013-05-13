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



/* ***** IMPORTANT: THIS IS NOT A NORMAL HEADER FILE         *****
 * *****            DO NOT INCLUDE DIRECTLY                  *****
 * *****            THE LACK OF HEADER GUARDS IS INTENTIONAL ***** */

/*
 * Conventions on Event Names:
 *
 * - The prefix determines something about how the timeline should be
 *   displayed, and is split up into various parts, separated by underscores:
 *  - 'SW' and 'HW' as the first part will be used to determine whether a
 *     timeline is to do with Software or Hardware - effectively, separate
 *     'channels' for Software and Hardware
 *  - 'START', 'STOP', 'ENTER', 'LEAVE' can be used in the second part, and
 *    signify related pairs of events - these are optional.
 *  - 'FLOW' indicates a generic event, which can use dependencies
 * - This gives events such as:
 *  - 'SW_ENTER_FOO'
 *  - 'SW_LEAVE_FOO'
 *  - 'SW_FLOW_BAR_1'
 *  - 'SW_FLOW_BAR_2'
 *  - 'HW_START_BAZ'
 *  - 'HW_STOP_BAZ'
 * - And an unadorned HW event:
 *  - 'HW_BAZ_FROZBOZ'
 */

/*
 * Conventions on parameter names:
 * - anything with 'instance' in the name will have a separate timeline based
 *   on that instances.
 * - underscored-prefixed parameters will by hidden by default on timelines
 *
 * Hence:
 * - Different job slots have their own 'instance', based on the instance value
 * - Per-context info (e.g. atoms on a context) have their own 'instance'
 *   (i.e. each context should be on a different timeline)
 *
 * Note that globally-shared resources can be tagged with a tgid, but we don't
 * want an instance per context:
 * - There's no point having separate Job Slot timelines for each context, that
 *   would be confusing - there's only really 3 job slots!
 * - There's no point having separate Shader-powered timelines for each
 *   context, that would be confusing - all shader cores (whether it be 4, 8,
 *   etc) are shared in the system.
 */

	/*
	 * CTX events
	 */
	/* Separate timelines for each context 'instance'*/
	KBASE_TIMELINE_TRACE_CODE(CTX_SET_NR_ATOMS_IN_FLIGHT,     "CTX: Atoms in flight",            "%d,%d",    "_instance_tgid,_value_number_of_atoms"),
	KBASE_TIMELINE_TRACE_CODE(CTX_FLOW_ATOM_READY,            "CTX: Atoms Ready to Run",         "%d,%d,%d", "_instance_tgid,_consumerof_atom_number,_producerof_atom_number_ready"),

	/*
	 * SW Events
	 */
	/* Separate timelines for each slot 'instance' */
	KBASE_TIMELINE_TRACE_CODE(SW_SET_GPU_SLOT_ACTIVE,         "SW: GPU slot active",             "%d,%d,%d", "_tgid,_instance_slot,_value_number_of_atoms"),
	KBASE_TIMELINE_TRACE_CODE(SW_SET_GPU_SLOT_NEXT,           "SW: GPU atom in NEXT",            "%d,%d,%d", "_tgid,_instance_slot,_value_is_an_atom_in_next"),
	KBASE_TIMELINE_TRACE_CODE(SW_SET_GPU_SLOT_HEAD,           "SW: GPU atom in HEAD",            "%d,%d,%d", "_tgid,_instance_slot,_value_is_an_atom_in_head"),
	KBASE_TIMELINE_TRACE_CODE(SW_SET_GPU_SLOT_STOPPING,       "SW: Try Soft-Stop on GPU slot",   "%d,%d,%d", "_tgid,_instance_slot,_value_is_slot_stopping"),
	/* Shader and overall power is shared - can't have separate instances of
	 * it, just tagging with the context */
	KBASE_TIMELINE_TRACE_CODE(SW_SET_GPU_POWER_ACTIVE,        "SW: GPU power active",            "%d,%d",    "_tgid,_value_is_power_active"),
	KBASE_TIMELINE_TRACE_CODE(SW_SET_GPU_POWER_TILER_ACTIVE,  "SW: GPU tiler powered",           "%d,%d",    "_tgid,_value_number_of_tilers"),
	KBASE_TIMELINE_TRACE_CODE(SW_SET_GPU_POWER_SHADER_ACTIVE, "SW: GPU shaders powered",         "%d,%d",    "_tgid,_value_number_of_shaders"),
	/* SW Power event messaging. _event_type is one from the kbase_pm_event enum  */
	KBASE_TIMELINE_TRACE_CODE(SW_FLOW_PM_SEND_EVENT,          "SW: PM Send Event",               "%d,%d,%d", "_tgid,_event_type,_writerof_pm_event_id"),
	KBASE_TIMELINE_TRACE_CODE(SW_FLOW_PM_HANDLE_EVENT,        "SW: PM Handle Event",             "%d,%d,%d", "_tgid,_event_type,_finalconsumerof_pm_event_id"),

	/*
	 * HW Events
	 */
	KBASE_TIMELINE_TRACE_CODE(HW_START_GPU_JOB_CHAIN_SW_APPROX,     "HW: Job Chain start (SW approximated)", "%d,%d,%d", "_tgid,job_slot,_consumerof_atom_number_ready"),
	KBASE_TIMELINE_TRACE_CODE(HW_STOP_GPU_JOB_CHAIN_SW_APPROX,      "HW: Job Chain stop (SW approximated)",  "%d,%d,%d", "_tgid,job_slot,_producerof_atom_number_completed")
