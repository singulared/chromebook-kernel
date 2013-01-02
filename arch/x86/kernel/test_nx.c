/*
 * test_nx.c: functional test for NX functionality
 *
 * (C) Copyright 2008 Intel Corporation
 * Author: Arjan van de Ven <arjan@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#define pr_fmt(fmt) "test_nx: " fmt

#include <linux/module.h>
#include <linux/sort.h>
#include <linux/slab.h>

#include <asm/uaccess.h>
#include <asm/asm.h>

/* 0xC3 is the opcode for "ret" */
#define INSN_RET	0xC3
/* 0x90 is the opcode for "nop" */
#define INSN_NOP	0x90

extern int rodata_test_data;

/*
 * This file checks 4 things:
 * 1) Check if the stack is not executable
 * 2) Check if kmalloc memory is not executable
 * 3) Check if the .rodata section is not executable
 * 4) Check if the .data section of a module is not executable
 *
 * To do this, the test code tries to execute memory in stack/kmalloc/etc,
 * and then checks if the expected trap happens.
 *
 * Sadly, this implies having a dynamic exception handling table entry.
 * ... which can be done (and will make Rusty cry)... but it can only
 * be done in a stand-alone module with only 1 entry total.
 * (otherwise we'd have to sort and that's just too messy)
 */

/*
 * We want to set up an exception handling point on our stack,
 * which means a variable value.
 */
static struct exception_table_entry *original_extable;
static struct exception_table_entry replaced_extable;
static void fake_exception_table(void *new)
{
	struct module *mod = THIS_MODULE;

	/*
	 * Note: This module has only 1 exception table entry,
	 * so searching and sorting is not needed. If that changes,
	 * this would be the place to search and re-sort the exception
	 * table.
	 */
	if (mod->num_exentries > 1) {
		pr_err("too many exception table entries!\n");
		pr_err("test results are not reliable.\n");
		return;
	}
	/* Save original exception table when first called. */
	if (original_extable == NULL)
		original_extable = mod->extable;
	/* Replace the original exception table with a duplicate. */
	if (mod->extable == original_extable)
		mod->extable = &replaced_extable;

	/* Record new insn address. */
	mod->extable->insn = (unsigned long)new;

	/* Record new fixup address. */
	mod->extable->fixup = original_extable->fixup;
}

static void restore_exception_table(void)
{
	struct module *mod = THIS_MODULE;

	if (original_extable)
		mod->extable = original_extable;
}


/*
 * exception tables get their symbols translated so we need
 * to use a fake function to put in there, which we can then
 * replace at runtime.
 */
void foo_label(void);

/*
 * returns 0 for not-executable, negative for executable
 *
 * Note: we cannot allow this function to be inlined, because
 * that would give us more than 1 exception table entry.
 * This in turn would break the assumptions above.
 */
static noinline int test_address(void *address)
{
	unsigned long result;

	/* Set up an exception table entry for our address */
	fake_exception_table(address);
	pr_info("calling %p (0x%02X)\n", address, *(unsigned char *)address);
	result = 1;
	asm volatile(
		"foo_label:\n"
		"0:	call *%[fake_code]\n"
		"1:\n"
		".section .fixup,\"ax\"\n"
		"2:	mov %[zero], %[rslt]\n"
		"	ret\n"
		".previous\n"
		_ASM_EXTABLE(0b,2b)
		: [rslt] "=r" (result)
		: [fake_code] "r" (address), [zero] "r" (0UL), "0" (result)
	);
	/* change the exception table back for the next round */
	restore_exception_table();

	if (result)
		return -ENODEV;
	return 0;
}

static unsigned char test_data;
static const int module_rodata_test_data = INSN_RET;

static int test_NX(void)
{
	int ret = 0;
	char stackcode[] = {INSN_RET, INSN_NOP, 0 };
	char *heap;

	test_data = INSN_RET;

	pr_info("Testing NX protection ...\n");

	/* Test 1: check if the stack is not executable */
	if (test_address(&stackcode)) {
		pr_err("stack is executable\n");
		ret = -ENODEV;
	}

	/* Test 2: Check if the heap is executable */
	heap = kmalloc(64, GFP_KERNEL);
	if (!heap)
		return -ENOMEM;
	heap[0] = INSN_RET;

	if (test_address(heap)) {
		pr_err("heap is executable\n");
		ret = -ENODEV;
	}
	kfree(heap);

#ifdef CONFIG_DEBUG_RODATA
	/* Test 3: Check if the kernel .rodata section is executable */
	if (rodata_test_data != INSN_RET) {
		pr_err("kernel .rodata marker has invalid value\n");
		ret = -ENODEV;
	} else if (test_address(&rodata_test_data)) {
		pr_err("kernel .rodata section is executable\n");
		ret = -ENODEV;
	}
#else
	pr_err("kernel .rodata section executable: missing " \
		"CONFIG_DEBUG_RODATA\n");
	ret = -ENODEV;
#endif

#ifdef CONFIG_DEBUG_SET_MODULE_RONX
	/* Test 4: Check if the .data section of a module is executable */
	if (test_address(&test_data)) {
		pr_err(".data section is executable\n");
		ret = -ENODEV;
	}

	/* Test 5: Check if the .rodata section of a module is executable */
	if (module_rodata_test_data != INSN_RET) {
		pr_err("module .rodata marker has invalid value\n");
		ret = -ENODEV;
	} else if (test_address((void *)&module_rodata_test_data)) {
		pr_err("module .rodata section is executable\n");
		ret = -ENODEV;
	}
#else
	pr_err("module .data and .rodata sections executable: missing " \
		"CONFIG_DEBUG_SET_MODULE_RONX\n");
	ret = -ENODEV;
#endif

	pr_info("Done testing.\n");

	return ret;
}

static void test_exit(void)
{ }

module_init(test_NX);
module_exit(test_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Testcase for the NX infrastructure");
MODULE_AUTHOR("Arjan van de Ven <arjan@linux.intel.com>");
