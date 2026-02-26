/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_SCHED_XACCT_H
#define _LINUX_SCHED_XACCT_H

/*
 * Extended task accounting methods:
 */

#include <linux/sched.h>

#ifdef CONFIG_TASK_XACCT
static inline void add_rchar(struct task_struct *tsk, ssize_t amt)
{
	tsk->ioac.rchar += amt;
        // #ifdef VENDOR_EDIT
        // xiwu1.peng@kernel 2021/09/03 add for io acct
        #ifdef CONFIG_TCL_IOACCT
	tsk->io_swi[tsk->io_index].rchar += amt;
        #endif
        // #endif /* VENDOR_EDIT */
}

static inline void add_wchar(struct task_struct *tsk, ssize_t amt)
{
	tsk->ioac.wchar += amt;
        // #ifdef VENDOR_EDIT
        // xiwu1.peng@kernel 2021/09/03 add for io acct
        #ifdef CONFIG_TCL_IOACCT
	tsk->io_swi[tsk->io_index].wchar += amt;
        #endif
        // #endif /* VENDOR_EDIT */
}

static inline void inc_syscr(struct task_struct *tsk)
{
	tsk->ioac.syscr++;
        // #ifdef VENDOR_EDIT
        // xiwu1.peng@kernel 2021/09/03 add for io acct
        #ifdef CONFIG_TCL_IOACCT
	tsk->io_swi[tsk->io_index].syscr++;
        #endif
        // #endif /* VENDOR_EDIT */
}

static inline void inc_syscw(struct task_struct *tsk)
{
	tsk->ioac.syscw++;
        // #ifdef VENDOR_EDIT
        // xiwu1.peng@kernel 2021/09/03 add for io acct
        #ifdef CONFIG_TCL_IOACCT
	tsk->io_swi[tsk->io_index].syscw++;
        #endif
        // #endif /* VENDOR_EDIT */
}

static inline void inc_syscfs(struct task_struct *tsk)
{
	tsk->ioac.syscfs++;
        // #ifdef VENDOR_EDIT
        // xiwu1.peng@kernel 2021/09/03 add for io acct
        #ifdef CONFIG_TCL_IOACCT
	tsk->io_swi[tsk->io_index].syscfs++;
        #endif
        // #endif /* VENDOR_EDIT */
}
#else
static inline void add_rchar(struct task_struct *tsk, ssize_t amt)
{
}

static inline void add_wchar(struct task_struct *tsk, ssize_t amt)
{
}

static inline void inc_syscr(struct task_struct *tsk)
{
}

static inline void inc_syscw(struct task_struct *tsk)
{
}

static inline void inc_syscfs(struct task_struct *tsk)
{
}
#endif

#endif /* _LINUX_SCHED_XACCT_H */
