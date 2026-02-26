#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/sched.h>
#include <linux/security.h>
#include <uapi/linux/sched/types.h>
#include <linux/cred.h>
#include <linux/printk.h>
#include <linux/delay.h>

// use p->atomic_flags(bit20-21) to mark task state
// check linux/sched.h to avoid conflict
// bit20: set if task is unlimited
// bit21: set if task is scan thread(AsyncTask #) in MediaProvider
#define PFA_MS_UNLIMITED 20
#define PFA_MS_IS_SCANNER 21

TASK_PFA_TEST(MS_UNLIMITED, ms_unlimited)
TASK_PFA_SET(MS_UNLIMITED, ms_unlimited)
TASK_PFA_TEST(MS_IS_SCANNER, ms_is_scanner)
TASK_PFA_SET(MS_IS_SCANNER, ms_is_scanner)

#define SCANNER_NAME "AsyncTask #"
#define SN_LEN (sizeof(SCANNER_NAME)-1)
#define SECCTX_MEDIAPROVIDER "u:r:mediaprovider_app:s0"
#define SECCTX_SZ (sizeof(SECCTX_MEDIAPROVIDER)-1)
#define UID_APP_START 10000
#define BG_PRIO 10

static inline bool current_is_scanner()
{
	uid_t uid;
	char *secctx = NULL;
	u32 secctx_sz = 0, secid;
	bool is_scanner = false;

	if (likely(task_ms_unlimited(current)))
		return false;

	// only limit bg thread
	if (task_nice(current) < BG_PRIO)
		return false;

	if (task_ms_is_scanner(current))
		return true;

	// check uid
	uid = from_kuid(current_user_ns(), current_uid());
	//pr_info("uid = %d\n", uid);
	if (uid < UID_APP_START) {
		task_set_ms_unlimited(current);
		return false;
	}
	// check task name
	if (strncmp(current->comm, SCANNER_NAME, SN_LEN)) {
		task_set_ms_unlimited(current);
		return false;
	}
	// check selinux context
	security_task_getsecid(current, &secid);
	if (security_secid_to_secctx(secid, &secctx, &secctx_sz)) {
		task_set_ms_unlimited(current);
		return false;
	}
	//pr_info("secctx = %s\n", secctx);
	if (!strncmp(secctx, SECCTX_MEDIAPROVIDER,
		min_t(u32, secctx_sz, SECCTX_SZ))) {
		task_set_ms_is_scanner(current);
		is_scanner = true;
	} else {
		task_set_ms_unlimited(current);
	}
	security_release_secctx(secctx, secctx_sz);
	return is_scanner;
}

// sysctl interface
int mediascanner_sleep_msecs = 100;
int mediascanner_util_threshold __read_mostly = 400;
int mediascanner_runtime_threshold __read_mostly = 240000;

static inline unsigned long task_util(struct task_struct *p)
{
	return READ_ONCE(p->se.avg.util_avg);
}

// return true if task run too long
static inline bool check_runtime()
{
	int runtime = current->se.sum_exec_runtime >> 20;

	return runtime >= mediascanner_runtime_threshold &&
		task_util(current) >= mediascanner_util_threshold;
}

void check_and_limit_mediascanner()
{
	// unlimited
	if (mediascanner_runtime_threshold <= 0)
		return;

	if (!current_is_scanner())
		return;

	if (check_runtime())
		msleep_interruptible(mediascanner_sleep_msecs);
}