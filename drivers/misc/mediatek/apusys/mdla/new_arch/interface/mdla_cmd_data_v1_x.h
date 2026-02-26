/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MDLA_CMD_DATA_V1_X_H__
#define __MDLA_CMD_DATA_V1_X_H__

#include <linux/types.h>
#include <linux/list.h>

/* Get priority level */
#include <utilities/mdla_util.h>

struct command_entry;

struct mdla_wait_cmd {
	uint32_t id;           /* [in] command id */
	int32_t  result;       /* [out] success(0), timeout(1) */
	uint64_t queue_time;   /* [out] time queued in driver (ns) */
	uint64_t busy_time;    /* [out] mdla execution time (ns) */
	uint32_t bandwidth;    /* [out] mdla bandwidth */
};

struct mdla_run_cmd {
	uint32_t offset_code_buf;
	uint32_t reserved;
	uint32_t size;
	uint32_t mva;
	uint32_t offset;        /* [in] command byte offset in buf */
	uint32_t count;         /* [in] # of commands */
	uint32_t id;            /* [out] command id */
};

struct mdla_run_cmd_sync {
	struct mdla_run_cmd req;
	struct mdla_wait_cmd res;
	uint32_t mdla_id;
};

struct mdla_wait_entry {
	uint32_t async_id;
	struct list_head list;
	struct mdla_wait_cmd wt;
};

struct mdla_scheduler {
	struct list_head ce_list[PRIORITY_LEVEL];
	struct command_entry *ce[PRIORITY_LEVEL];
	struct command_entry *pro_ce;

	spinlock_t lock;

	void (*enqueue_ce)(u32 core_id, struct command_entry *ce, u32 resume);
	struct command_entry* (*dequeue_ce)(u32 core_id);
	void (*issue_ce)(u32 core_id);
	void (*issue_dual_lowce)(u32 core_id, uint64_t dual_cmd_id);
	int (*process_ce)(u32 core_id);
	void (*complete_ce)(u32 core_id);
	void (*preempt_ce)(u32 core_id, struct command_entry *high_ce);
	u64 (*get_smp_deadline)(int priority);
	void (*set_smp_deadline)(int priority, u64 deadline);
};

#endif /* __MDLA_CMD_DATA_V1_X_H__ */

