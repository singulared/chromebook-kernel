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



#ifndef _KBASE_DEBUG_H
#define _KBASE_DEBUG_H

#include <linux/bug.h>

/** @brief If 0, the current file:line is displayed before each message. */
#define KBASE_DEBUG_SKIP_TRACE 0

/** @brief If 0, the current function is displayed before each message. */
#define KBASE_DEBUG_SKIP_FUNCTION_NAME 0

/** @brief Disable the asserts tests if set to 1. Default is to disable the asserts in release. */
#ifndef KBASE_DEBUG_DISABLE_ASSERTS
#ifdef CONFIG_MALI_DEBUG
#define KBASE_DEBUG_DISABLE_ASSERTS 0
#else
#define KBASE_DEBUG_DISABLE_ASSERTS 1
#endif
#endif				/* KBASE_DEBUG_DISABLE_ASSERTS */

/**
 * @brief If 1, disable INFO messages.
 * @note If this is not disabled then Android boot times out.
 */
#ifndef KBASE_DEBUG_DISABLE_INFO
#ifdef CONFIG_MALI_DEBUG
#define KBASE_DEBUG_DISABLE_INFO 0
#else
#define KBASE_DEBUG_DISABLE_INFO 1
#endif
#endif

typedef enum {
	KBASE_UNKNOWN = 0, /**< @brief Unknown module */
	KBASE_MMU,	   /**< @brief ID of Base MMU */
	KBASE_JD,	   /**< @brief ID of Base Job Dispatch */
	KBASE_JM,	   /**< @brief ID of Base Job Manager */
	KBASE_CORE,	   /**< @brief ID of Base Core */
	KBASE_MEM,	   /**< @brief ID of Base Memory */
	KBASE_EVENT,	   /**< @brief ID of Base Event */
	KBASE_CTX,	   /**< @brief ID of Base Context */
	KBASE_PM,	   /**< @brief ID of Base Power Management */
	KBASE_MODULES_ALL  /**< @brief Select all the modules at once / Also gives the number of modules in the enum */
} kbase_module;

/** Function type that is called on an KBASE_DEBUG_ASSERT() or KBASE_DEBUG_ASSERT_MSG() */
typedef void (kbase_debug_assert_hook) (void *);

typedef struct kbasep_debug_assert_cb {
	kbase_debug_assert_hook *func;
	void *param;
} kbasep_debug_assert_cb;

/**
 * @def KBASEP_DEBUG_PRINT_TRACE
 * @brief Private macro containing the format of the trace to display before every message
 * @sa KBASE_DEBUG_SKIP_TRACE, KBASE_DEBUG_SKIP_FUNCTION_NAME
 */
#if KBASE_DEBUG_SKIP_TRACE == 0
#define KBASEP_DEBUG_PRINT_TRACE __FILE__ ":" CSTD_STR2(__LINE__) ":"
#else
#define KBASEP_DEBUG_PRINT_TRACE ""
#endif

/**
 * @def KBASEP_DEBUG_PRINT_FUNCTION
 * @brief Private macro containing the format of the trace to display before every message
 * @sa KBASE_DEBUG_SKIP_TRACE, KBASE_DEBUG_SKIP_FUNCTION_NAME
 */
#if KBASE_DEBUG_SKIP_FUNCTION_NAME == 0
#define KBASEP_DEBUG_PRINT_FUNCTION CSTD_FUNC
#else
#define KBASEP_DEBUG_PRINT_FUNCTION ""
#endif

/**
 * @def KBASEP_DEBUG_ASSERT_OUT(format, ...)
 * @brief (Private) system printing function associated to the @see KBASE_DEBUG_ASSERT_MSG event.
 * @param format Message to display on assert.
 * @param ... format arguments
 */
#ifdef CONFIG_MALI_DEBUG
#define KBASEP_DEBUG_ASSERT_OUT(format, ...) \
	do { \
		pr_err("Mali<ASSERT>: " \
				KBASEP_DEBUG_PRINT_TRACE"%s " format "\n", \
				KBASEP_DEBUG_PRINT_FUNCTION, ##__VA_ARGS__); \
	} while (MALI_FALSE)
#else
#define KBASEP_DEBUG_ASSERT_OUT(format, ...) CSTD_NOP()
#endif

#ifdef CONFIG_MALI_DEBUG
#define KBASE_CALL_ASSERT_HOOK() kbasep_debug_assert_call_hook();
#else
#define KBASE_CALL_ASSERT_HOOK() CSTD_NOP();
#endif

/**
 * @def KBASE_DEBUG_ASSERT(expr)
 * @brief Calls @see KBASE_PRINT_ASSERT and prints the expression @a expr if @a expr is false
 *
 * @note This macro does nothing if the flag @see KBASE_DEBUG_DISABLE_ASSERTS is set to 1
 *
 * @param expr Boolean expression
 */
#define KBASE_DEBUG_ASSERT(expr) \
	KBASE_DEBUG_ASSERT_MSG(expr, #expr)

/**
 * @def KBASE_DEBUG_ASSERT_MSG(expr, ...)
 * @brief Calls @see KBASEP_DEBUG_ASSERT_OUT and prints the given message if @a expr is false
 *
 * @note This macro does nothing if the flag @see KBASE_DEBUG_DISABLE_ASSERTS is set to 1
 *
 * @param expr Boolean expression
 * @param format  Message to display when @a expr is false, as a format string.
 * @param ...  format arguments
 */
#if KBASE_DEBUG_DISABLE_ASSERTS
#define KBASE_DEBUG_ASSERT_MSG(expr, format, ...) CSTD_NOP()
#else
#define KBASE_DEBUG_ASSERT_MSG(expr, format, ...) \
	do { \
		if (MALI_FALSE == (expr)) { \
			KBASEP_DEBUG_ASSERT_OUT(format, ##__VA_ARGS__); \
			KBASE_CALL_ASSERT_HOOK();\
			BUG(); \
		} \
	} while (MALI_FALSE)
#endif				/* KBASE_DEBUG_DISABLE_ASSERTS */

#ifdef CONFIG_MALI_DEBUG
#define KBASE_DEBUG_PRINT_WARN(module, format, ...) \
	do { \
		pr_warn("Mali<WARN, %s>: " \
				KBASEP_DEBUG_PRINT_TRACE "%s " format "\n", \
				kbasep_debug_module_to_str(module), \
				KBASEP_DEBUG_PRINT_FUNCTION, ##__VA_ARGS__); \
	} while (MALI_FALSE)
#else
#define KBASE_DEBUG_PRINT_WARN(module, ...) CSTD_NOP()
#endif

#define KBASE_DEBUG_PRINT_ERROR(module, format, ...) \
	do { \
		pr_err("Mali<ERROR, %s>: " \
				KBASEP_DEBUG_PRINT_TRACE "%s " format "\n", \
				kbasep_debug_module_to_str(module), \
				KBASEP_DEBUG_PRINT_FUNCTION, ##__VA_ARGS__); \
	} while (MALI_FALSE)

#if KBASE_DEBUG_DISABLE_INFO
#define KBASE_DEBUG_PRINT_INFO(module, format, ...) CSTD_NOP()
#else
#define KBASE_DEBUG_PRINT_INFO(module, format, ...)\
	do {\
		pr_info("Mali<INFO, %s>: " \
				KBASEP_DEBUG_PRINT_TRACE "%s " format "\n", \
				kbasep_debug_module_to_str(module), \
				KBASEP_DEBUG_PRINT_FUNCTION, ##__VA_ARGS__); \
	} while (MALI_FALSE)
#endif

#define KBASE_DEBUG_PRINT_RAW_LEVEL(level, module, format, ...) do { \
		printk(level format "\n", ##__VA_ARGS__); \
	} while (MALI_FALSE)

#define KBASE_DEBUG_PRINT_RAW(module, format, ...) \
	KBASE_DEBUG_PRINT_RAW_LEVEL("", module, format, ##__VA_ARGS__)

#define KBASE_DEBUG_PRINT(module, format, ...) \
	KBASE_DEBUG_PRINT_RAW(module, format, ##__VA_ARGS__)

/**
 * @def KBASE_DEBUG_CODE( X )
 * @brief Executes the code inside the macro only in debug mode
 *
 * @param X Code to compile only in debug mode.
 */
#ifdef CONFIG_MALI_DEBUG
#define KBASE_DEBUG_CODE(X) X
#else
#define KBASE_DEBUG_CODE(X) CSTD_NOP()
#endif				/* CONFIG_MALI_DEBUG */

/**
 * @brief Register a function to call on ASSERT
 *
 * Such functions will \b only be called during Debug mode, and for debugging
 * features \b only. Do not rely on them to be called in general use.
 *
 * To disable the hook, supply NULL to \a func.
 *
 * @note This function is not thread-safe, and should only be used to
 * register/deregister once in the module's lifetime.
 *
 * @param[in] func the function to call when an assert is triggered.
 * @param[in] param the parameter to pass to \a func when calling it
 */
void kbase_debug_assert_register_hook(kbase_debug_assert_hook *func, void *param);

/**
 * @brief Call a debug assert hook previously registered with kbase_debug_assert_register_hook()
 *
 * @note This function is not thread-safe with respect to multiple threads
 * registering functions and parameters with
 * kbase_debug_assert_register_hook(). Otherwise, thread safety is the
 * responsibility of the registered hook.
 */
void kbasep_debug_assert_call_hook(void);

/**
 * @brief Convert a module id into a module name.
 *
 * @param module ID of the module to convert
 * @note module names are stored in : @see kbasep_str_modules.
 * @return the name of the given module ID as a string of characters.
 */
const char *kbasep_debug_module_to_str(const kbase_module module);

#endif				/* _KBASE_DEBUG_H */
