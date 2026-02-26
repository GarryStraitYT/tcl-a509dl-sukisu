// SPDX-License-Identifier: GPL-2.0

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hardirq.h>
#include <linux/init.h>
#include <linux/kallsyms.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/debug.h>
#include <linux/sched/mm.h>
#include <linux/sched/rt.h>
#include <linux/sched/task.h>
#include <uapi/linux/sched/types.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <mt-plat/aee.h>
#include <mt-plat/monitor_hang.h>
#include <linux/seq_file.h>
#include <linux/jiffies.h>
#include <linux/ptrace.h>
#include <asm/stacktrace.h>
#include <asm/traps.h>
#include <linux/pid.h>
#ifdef CONFIG_MTK_BOOT
#include <mt-plat/mtk_boot_common.h>
#endif
#ifdef CONFIG_MTK_ION
#include <mtk/ion_drv.h>
#endif
#if IS_ENABLED(CONFIG_MTK_GPU_SUPPORT)
//#include <mt-plat/mtk_gpu_utility.h>
#endif
#ifdef CONFIG_MTK_AEE_IPANIC
#include <mt-plat/mboot_params.h>
#endif
#include "mrdump/mrdump_private.h"
#include "mrdump/mrdump_mini.h"
#include "aed/aed.h"
#include <mrdump.h>
// Add by jinggao.zhou for ENCOREVZW-7282, add BOOT DETECT timeout 5 mins 2022/11/01
#ifdef CONFIG_TCL_BOOT_DETECT
#include <linux/uaccess.h>
#include <mtk_drm_assert_ext.h>
enum mtkfb_power_mode {
  	FB_SUSPEND = 0,
  	FB_RESUME,
  	DOZE_SUSPEND,
  	DOZE,
  	MTKFB_POWER_MODE_UNKNOWN,
};
enum disp_pwm_id_t {
  	DISP_PWM0 = 0x1,
  	DISP_PWM1 = 0x2,
  	DISP_PWM_ALL = (DISP_PWM0 | DISP_PWM1)
};
enum mtkfb_power_mode primary_display_set_power_mode(enum mtkfb_power_mode new_mode);
void mtkfb_late_resume(void);
int disp_pwm_set_backlight(enum disp_pwm_id_t id, int level_1024);

#define LOG_BLOCK_SIZE (1024)

int kmsg_numbers=0;
int ui_print_line=0;
int watchdog_print_line=0;
int runtime_print_line=0;
int System_print_line=0;
int Zygote_print_line=0;
int keystore_print_line=0;
int logcat_sum_lines=0;
int device_mapper_print_line=0;
int hidl_print_line=0;
int hidlservice_print_line=0;
int boot_detect_kerenl_display_status=0;
int ui_print_cursor_initflag=0;
int kernel_printk_tofb_ctrl=0;
int current_version=0;// 0: daily 1: user 2: mp 3: mini 4: cer 5: fut

char logcat_msg[1024];

void log_store_to_emmc(void);
void klog_check_to_ui(void);
char kdisplay_bootprof_msg[256]={0};
char kdisplay_bootprof_msg1[256]={0};
char kdisplay_bootprof_msg2[256]={0};
char kdisplay_bootprof_msg3[256]={0};
char kdisplay_bootprof_msg4[256]={0};
char kdisplay_bootprof_msg5[256]={0};
char kdisplay_bootprof_msg6[256]={0};
char kdisplay_bootprof_msg7[256]={0};
char kdisplay_bootprof_msg8[256]={0};
unsigned int get_boot_mode(void);

extern int logcat_to_kernel(void);
extern void mtk_drm_fb_late_resume(void);
void print_ver_info(void);
void botodetect_curtime_to_str(char *cur_time);
#ifdef CONFIG_DRM_MEDIATEK
void hold_lcd_backlight(void)
{
    //pr_info("hang_detect hold_lcd_backlight mtk_drm_fb_late_resume \n");
    mtk_drm_fb_late_resume();
    return;
}
#else
extern int mtkfb_set_backlight_level(unsigned int level);
void hold_lcd_backlight(void)
{
    pr_info("hang_detect hold_lcd_backlight primary_display_set_power_mode\n");
    primary_display_set_power_mode(1);
    mtkfb_late_resume();
    disp_pwm_set_backlight(1,1023);
    return;
}
#endif

static int tct_kernel_printk_tofb_ctrl_get(char *str)
{

//The blue screen function is in the USERDEBUG version, 
//which can be forced to turn on by default, turned on by UARTFLAG=O, and turned off by UARTFLAG=X
// current_version 0: daily 1: user 2: mp 3: mini 4: cer 5: fut
#ifdef CONFIG_TCT_MP2_BLUESCREEN  // ship version will read oem_uart=0x1F
    kstrtoint(str, 0, &kernel_printk_tofb_ctrl);
    current_version=2;
    pr_info("%s:hang_detect Board androidboot.oem_uart=0x%x TARGET_BUILD_SHIP current_version=%d\n",
    __func__,kernel_printk_tofb_ctrl,current_version);
    return 0;
#else
//  if get androidboot.oem_uart= 0x38  will Disable Kernel Display, TELEWEB write UART X to disable
//  enabel 0x30+0x1F =0x4F(O)  disable 0x30+0x28 =0x58(X)
    kstrtoint(str, 0, &kernel_printk_tofb_ctrl);
    current_version=0; //default daily version
    pr_info("%s:hang_detect get androidboot.oem_uart=0x%x [0x1F=ON,0x28=OFF] not ship|mp=2 version current_version=%d\n",
    __func__,kernel_printk_tofb_ctrl,current_version);
    if (kernel_printk_tofb_ctrl==0x28)// userdebug disable function
    {
		kernel_printk_tofb_ctrl=0x28;
		pr_info("%s:hang_detect Board androidboot.oem_uart=%x [0x28] will Disable bluescreen display tofb_ctrl=0x28\n",
		__func__,kernel_printk_tofb_ctrl);
		return 0;
    }

#if defined(TARGET_BUILD_MMITEST)
    kstrtoint(str, 0, &kernel_printk_tofb_ctrl);
    current_version=2;
    pr_info("%s:hang_detect Board androidboot.oem_uart=%x TARGET_BUILD_MMITEST current_version=%d\n",
    __func__,kernel_printk_tofb_ctrl,current_version);
#endif

#if defined(TARGET_BUILD_CERTIFICATION)
    kstrtoint(str, 0, &kernel_printk_tofb_ctrl);
    current_version=4;
    pr_info("%s:hang_detect Board androidboot.oem_uart=%x TARGET_BUILD_CERTIFICATION current_version=%d\n",
    __func__,kernel_printk_tofb_ctrl,current_version);
#endif

#if defined(TARGET_BUILD_USER)
    // for first test
    kstrtoint(str, 0, &kernel_printk_tofb_ctrl);
    current_version=1;
    pr_info("%s:hang_detect Board androidboot.oem_uart=%x TARGET_BUILD_USER current_version=%d\n",
    __func__,kernel_printk_tofb_ctrl,current_version);
#endif


//#ifdef CONFIG_TCL_FORCE_DAILY_KEY_DETECT
    //daily will default open Bluescreen
//		if (current_version==0){
//			kernel_printk_tofb_ctrl=0x10; // all version force trigger boot failed!
//			pr_info("%s:hang_detect Board androidboot.oem_uart=%x Daily UERDEBUG Open Bluescreen display! current_version=%d\n",
//			__func__,kernel_printk_tofb_ctrl,current_version);
//		}
//#endif

#ifdef CONFIG_TCL_FORCE_BOOT_DETECT
		kernel_printk_tofb_ctrl=0x10; // all version force trigger boot failed!
		pr_info("%s:hang_detect Board androidboot.oem_uart=%x TCL_FORCE_BOOT_DETECT ENABLE Bluescreen display! current_version=%d\n",
		__func__,kernel_printk_tofb_ctrl,current_version);
#else
		pr_info("%s:hang_detect Board androidboot.oem_uart=%x TCL_FORCE_BOOT_DETECT DISABLE!\n",
		__func__,kernel_printk_tofb_ctrl);
#endif

#endif
        return 0;
}
__setup("androidboot.oem_uart=",tct_kernel_printk_tofb_ctrl_get);

void klog_bootprof_to_ui(void)
{
	DAL_Printf("\n");	
	DAL_Printf("boot_detect:  BOOTPROF: %s \n", kdisplay_bootprof_msg8);
	DAL_Printf("boot_detect:  BOOTPROF: %s \n", kdisplay_bootprof_msg7);
	DAL_Printf("boot_detect:  BOOTPROF: %s \n", kdisplay_bootprof_msg6);
	DAL_Printf("boot_detect:  BOOTPROF: %s \n", kdisplay_bootprof_msg5);
	DAL_Printf("boot_detect:  BOOTPROF: %s \n", kdisplay_bootprof_msg4);
	DAL_Printf("boot_detect:  BOOTPROF: %s \n", kdisplay_bootprof_msg3);
	DAL_Printf("boot_detect:  BOOTPROF: %s \n", kdisplay_bootprof_msg2);
	DAL_Printf("boot_detect:  BOOTPROF: %s \n", kdisplay_bootprof_msg1);
	DAL_Printf("boot_detect:  BOOTPROF: %s \n", kdisplay_bootprof_msg);
	DAL_Printf("\n");
}
void klog_check_to_ui(void)
{
	char buff[LOG_BLOCK_SIZE];
	struct kmsg_dumper dumper = { .active = true };
	size_t len = 0;
	kmsg_dump_rewind_nolock(&dumper);
	memset(buff, 0, LOG_BLOCK_SIZE);

	DAL_Printf("--Kernel LOG analysis start---------------------------------------\n");
	while (kmsg_dump_get_line_nolock(&dumper, true, buff,
					 LOG_BLOCK_SIZE, &len)) {
	kmsg_numbers++;
	if (  strstr(buff, "Exec service failed")   || strstr(buff, "reboot_target:")
			  ||strstr(buff, "HandlePowerctlMessage") || strstr(buff, "EX_SERVICE_SPECIFIC")
			  ||strstr(buff, "Received sys.powerctl") || strstr(buff, "Received sys.powerctl") )
	{
		DAL_Printf("[%d] %s \n",kmsg_numbers, buff);
	}
	if ( strstr(buff, "device-mapper: verity") ){
				if ( !strstr(buff, "device-mapper: verity: sha256 using") ){
					if ( device_mapper_print_line <10 )
							DAL_Printf("device-mapper %d %s",device_mapper_print_line,buff);
					device_mapper_print_line++;
				}
	}
	if ( strstr(buff, "hwservicemanager") ){
		if ( !strstr(buff, "getFrameworkHalManifest") &&
			   !strstr(buff, "getDeviceHalManifest") &&
			   !strstr(buff, "hwservicemanager.rc") &&
			   !strstr(buff, "start_waiting_for_property") &&
			   !strstr(buff, "starting service") &&
			   !strstr(buff, "hwservicemanager.ready") &&
			   !strstr(buff, "running") &&
			   !strstr(buff, "is ready now") ){
			if ( hidlservice_print_line <5 )
					DAL_Printf("hwservicemanager %d %s",hidlservice_print_line,buff);
			hidlservice_print_line++;
	  }
	}

	memset(buff, 0, LOG_BLOCK_SIZE);
	}
	DAL_Printf("--Kernel LOG analysis End----------------------------------------\n");

}
void boot_detect_tringger()
{
	  char boot_cur_time[128] = {0};
    pr_info("hang_detect primary_display_set_power_mode -->\n");
    hold_lcd_backlight();
    boot_detect_kerenl_display_status=1;  
    DAL_SetColor(0xFFFFFF,0x0000FF);
    DAL_SetScreenColor(0x0000FF);
    DAL_Printf("\n");
    DAL_Printf("\n");
    DAL_Printf("\n");
    pr_info("hang_detect logcat_to_kernel -->\n");
  	DAL_Printf("logcat_to_kernel start =%d ui_print_cursor_initflag=%d END\n",logcat_to_kernel(),ui_print_cursor_initflag);
    DAL_Printf("--Android LOG analysis End------------------------------------\n");
    pr_info("hang_detect logcat_to_kernel <--\n");
    mdelay(10 * 1000);
    klog_bootprof_to_ui();
    mdelay(1 * 1000);
    klog_check_to_ui();
    mdelay(5 * 1000);
    pr_info("hang_detect ,Save log to expdb :Kernel log and Android log\n");
    log_store_to_emmc();
    DAL_Printf("K&A log has saved expdb successfully \n");
    DAL_Printf("boot_cur_time:%s\n",boot_cur_time);
    DAL_Printf("After 120S, SYSTEM REBOOT! Please take a picture [%d] \n",logcat_sum_lines);
    DAL_Printf("\n BOOT Diagnostic System. TOOL BUG send to zhoujinggao@tcl.com \n");
    DAL_Printf("\n Please send Picture to System or Framework Team \n");
}

char *strrpl(char *s, const char *s1, const char *s2)
{
	char *ptr;
	while ( (ptr = strstr(s, s1))!=NULL )
	{
	memmove(ptr + strlen(s2) , ptr + strlen(s1), strlen(ptr) - strlen(s1) + 1);
	memcpy(ptr, &s2[0], strlen(s2));
	}
	return s;
}
#endif

#ifndef TASK_STATE_TO_CHAR_STR
#define TASK_STATE_TO_CHAR_STR "RSDTtZXxKWPNn"
#endif

#ifdef CONFIG_MTK_HANG_DETECT_DB
#define MAX_HANG_INFO_SIZE (2*1024*1024) /* 2M info */
static int MaxHangInfoSize = MAX_HANG_INFO_SIZE;
#define MAX_STRING_SIZE 256
char *Hang_Info;
static int Hang_Info_Size;
static bool watchdog_thread_exist;
static bool system_server_exist;
#endif

static bool Hang_Detect_first;
static bool hd_detect_enabled;
static bool hd_zygote_stopped;
static int hd_timeout = 0x7fffffff;
static int hang_detect_counter = 0x7fffffff;
static int dump_bt_done;
static bool reboot_flag;
static struct name_list *white_list;
#ifdef CONFIG_MTK_ENG_BUILD
	struct proc_dir_entry *pe;
#endif



DECLARE_WAIT_QUEUE_HEAD(dump_bt_start_wait);
DECLARE_WAIT_QUEUE_HEAD(dump_bt_done_wait);
DEFINE_RAW_SPINLOCK(white_list_lock);

#ifdef MODULE
rwlock_t *Ptasklist_lock;
static int (*Pin_sched_functions)(unsigned long addr);
static void (*Pput_task_stack)(struct task_struct *tsk);
#ifdef __aarch64__		/* 64bit */
static void (*Pwalk_stackframe)(struct task_struct *tsk,
				struct stackframe *frame,
			    int (*fn)(struct stackframe *, void *), void *data);
#endif
static const char * (*Parch_vma_name)(struct vm_area_struct *vma);
static void (*Pdo_send_sig_info)(int sig, struct siginfo *info,
	struct task_struct *p, bool group);

int module_fun_init(void)
{

	pr_info("monitor_hang module fun init.");
	Ptasklist_lock = (rwlock_t *)kallsyms_lookup_name("tasklist_lock");
	if (Ptasklist_lock == NULL) {
		pr_warn("Ptasklist_lock is null");
		return 1;
	}
	Pin_sched_functions = (void *)kallsyms_lookup_name(
		"in_sched_functions");
	if (Pin_sched_functions == NULL) {
		pr_warn("Pin_sched_functions is null");
		return 1;
	}
	Pput_task_stack = (void *)kallsyms_lookup_name("put_task_stack");
	if (Pput_task_stack == NULL) {
		pr_warn("Pput_task_stack is null");
		return 1;
	}
#ifdef __aarch64__		/* 64bit */
	Pwalk_stackframe = (void *)kallsyms_lookup_name("walk_stackframe");
	if (Pwalk_stackframe == NULL) {
		pr_warn("Pwalk_stackframe is null");
		return 1;
	}

#endif
	Parch_vma_name = (void *)kallsyms_lookup_name("arch_vma_name");
	if (Parch_vma_name == NULL) {
		pr_warn("Parch_vma_name is null");
		return 1;
	}

	Pdo_send_sig_info = (void *)kallsyms_lookup_name("do_send_sig_info");
	if (Pdo_send_sig_info == NULL) {
		pr_warn("Pdo_send_sig_info is null");
		return 1;
	}
	return 0;
}
#endif

static void ShowStatus(int flag);
static void MonitorHangKick(int lParam);

static void reset_hang_info(void)
{
	Hang_Detect_first = false;
}

int add_white_list(char *name)
{
	struct name_list *new_thread = NULL;
	struct name_list *pList = NULL;

	raw_spin_lock(&white_list_lock);

	if (!white_list) {
		new_thread = kmalloc(sizeof(struct name_list), GFP_KERNEL);
		if (!new_thread) {
			raw_spin_unlock(&white_list_lock);
			return -1;
		}
		strncpy(new_thread->name, name, TASK_COMM_LEN);
		new_thread->name[TASK_COMM_LEN - 1] = 0;
		new_thread->next = NULL;
		white_list = new_thread;
		raw_spin_unlock(&white_list_lock);
		return 0;
	}

	pList = white_list;
	while (pList) {
		/*find same thread name*/
		if (strncmp(pList->name, name, TASK_COMM_LEN) == 0) {
			raw_spin_unlock(&white_list_lock);
			return 0;
		}
		pList = pList->next;
	}

	/*add new thread name*/
	new_thread = kmalloc(sizeof(struct name_list), GFP_KERNEL);
	if (!new_thread) {
		raw_spin_unlock(&white_list_lock);
		return -1;
	}
	strncpy(new_thread->name, name, TASK_COMM_LEN);
	new_thread->next = white_list;
	white_list = new_thread;
	raw_spin_unlock(&white_list_lock);
	return 0;
}

int del_white_list(char *name)
{
	struct name_list *pList = NULL;
	struct name_list *pList_old = NULL;

	if (!white_list)
		return 0;


	raw_spin_lock(&white_list_lock);
	pList = pList_old = white_list;
	while (pList) {
		/*find same thread name*/
		if (strncmp(pList->name, name, TASK_COMM_LEN) == 0) {
			if (pList == white_list) {
				white_list = pList->next;
				kfree(pList);
				raw_spin_unlock(&white_list_lock);
				return 0;
			}

			pList_old->next = pList->next;
			kfree(pList);
			raw_spin_unlock(&white_list_lock);
			return 0;
		}
		pList_old = pList;
		pList = pList->next;
	}
	raw_spin_unlock(&white_list_lock);
	return 0;
}


#ifdef CONFIG_MTK_ENG_BUILD
static int monit_hang_flag = 1;
#define SEQ_printf(m, x...) \
do {                \
	if (m)          \
		seq_printf(m, x);   \
	else            \
		pr_debug(x);        \
} while (0)

static int monitor_hang_show(struct seq_file *m, void *v)
{
	struct name_list *pList = NULL;
#ifdef CONFIG_MTK_HANG_DETECT_DB
	SEQ_printf(m, "[Hang_Detect] show hang_detect_raw\n");
	if (Hang_Info)
		SEQ_printf(m, "%s", Hang_Info);
	else
		SEQ_printf(m, "hang_detect_raw buffer is not ready\n");
#endif
	raw_spin_lock(&white_list_lock);
	pList = white_list;
	while (pList) {
		SEQ_printf(m, "white list process %s\n", pList->name);
		pList = pList->next;
	}
	raw_spin_unlock(&white_list_lock);

	return 0;
}

static int monitor_hang_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, monitor_hang_show, inode->i_private);
}


static ssize_t monitor_hang_proc_write(struct file *filp, const char *ubuf,
		size_t cnt, loff_t *data)
{
	char buf[64];
	long val;
	int ret;

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;

	buf[cnt] = 0;

	ret = kstrtoul(buf, 10, (unsigned long *)&val);

	if (ret < 0)
		return ret;

	if (val == 1) {
		monit_hang_flag = 1;
		pr_debug("[hang_detect] enable ke.\n");
	} else if (val == 0) {
		monit_hang_flag = 0;
		pr_debug("[hang_detect] disable ke.\n");
	} else if (val == 2) {
		reset_hang_info();
		ShowStatus(0);
	} else if (val == 3) {
		reset_hang_info();
		ShowStatus(1);
	} else if (val > 10) {
		show_native_bt_by_pid((int)val);
	}

	return cnt;
}

static const struct file_operations monitor_hang_fops = {
	.open = monitor_hang_proc_open,
	.write = monitor_hang_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int monitor_hang_open(struct inode *inode, struct file *filp)
{

	return 0;
}

static int monitor_hang_release(struct inode *inode, struct file *filp)
{

	return 0;
}

static unsigned int monitor_hang_poll(struct file *file,
		struct poll_table_struct *ptable)
{

	return 0;
}

static ssize_t monitor_hang_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{

	return 0;
}
#ifdef CONFIG_TCL_BOOT_DETECT
static void logcatmsg_screen_wirte(char * logcat_msg)
{
			if ( ui_print_cursor_initflag==0 ){
		      hold_lcd_backlight();
		      mdelay(10);
		      boot_detect_kerenl_display_status=1;
		      DAL_SetColor(0xFFFFFF,0x0000FF);
		      DAL_SetScreenColor(0x0000FF);
		      DAL_Printf("\n");
		      DAL_Printf("\n");
		      DAL_Printf_SetCursor(0,8);
		      DAL_Printf("--Android LOG analysis-1212-1---------------------------------------\n");
		      ui_print_cursor_initflag=1;
			}
			if ( strstr(logcat_msg, "W Watchdog:") ){
		      if ( watchdog_print_line <10 )
		      {
				DAL_Printf("Watchdog %d %d %d %s",watchdog_print_line,logcat_sum_lines,ui_print_cursor_initflag,logcat_msg+20);
		      }
		      watchdog_print_line++;
			}
			if ( strstr(logcat_msg, "E AndroidRuntime:") ){
		      if ( runtime_print_line <10 )
		      {
				DAL_Printf("%d %d %d %s",watchdog_print_line,logcat_sum_lines,ui_print_cursor_initflag,logcat_msg+20);
		      }
		      runtime_print_line++;
			}
			if ( strstr(logcat_msg, "E System  :") ){
		      if ( System_print_line <10 )
		      {
				DAL_Printf("%d %d %d %s",watchdog_print_line,logcat_sum_lines,ui_print_cursor_initflag,logcat_msg+20);
		      }
		      System_print_line++;
			}
			if ( strstr(logcat_msg, "E Zygote  :") ){
		      if ( Zygote_print_line <10 )
		      {
				DAL_Printf("AndroidRuntime %d %d %d %s",Zygote_print_line,logcat_sum_lines,ui_print_cursor_initflag,logcat_msg+20);
		      }
		      Zygote_print_line++;
			}
			if ( strstr(logcat_msg, "E keystore2") ){
		      if ( keystore_print_line <10 )
		      {
				DAL_Printf("keystore2 %d %d %d %s",keystore_print_line,logcat_sum_lines,ui_print_cursor_initflag,logcat_msg+20);
		      }
				keystore_print_line++;
			}
			
			if ( strstr(logcat_msg, "HidlServiceManagement: Waited one second for") ){
		      if ( hidl_print_line <5 )
		      {
						DAL_Printf("HidlServiceManagement %d %d %d %s",hidl_print_line,logcat_sum_lines,ui_print_cursor_initflag,logcat_msg+20);
		      }
				hidl_print_line++;
			}
			if (   strstr(logcat_msg, "E vold")
			  	|| strstr(logcat_msg, "F DEBUG   : Build fingerprint:")
			  	|| strstr(logcat_msg, "F DEBUG   : Cmdline:")
			  	|| strstr(logcat_msg, "F DEBUG   : Abort message:")
			  ){
			  	 DAL_SetColor(0xFFFFFF,0x0000FF);
			  	 DAL_SetScreenColor(0x0000FF);
			  	 if ( ui_print_line <30 )
			  	 {
				   	DAL_Printf("%d %d %d %s",ui_print_line,logcat_sum_lines,ui_print_cursor_initflag,logcat_msg+20);
			  	 }
			  	 ui_print_line++;
			}
			if ( strstr(logcat_msg, "D RecoverySystemService:")){
			  	 DAL_Printf("RecoverySystemService %d %d %d %s",ui_print_line,logcat_sum_lines,ui_print_cursor_initflag,logcat_msg+20);
			}
			if ( strstr(logcat_msg, "I ShutdownThread: Rebooting, reason:"))
		  {
			  	 DAL_Printf("ShutdownThread %d %d %d %s",ui_print_line,logcat_sum_lines,ui_print_cursor_initflag,logcat_msg+20);
			}
			logcat_sum_lines++;

}
#endif

static ssize_t monitor_hang_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	char msg[8] = {0};
#ifndef CONFIG_TCL_BOOT_DETECT
	if (count >= 2) {
		pr_info("hang_detect: invalid input 677\n");
		return -EINVAL;
	}
#else // enable boot detect
	if ( (kernel_printk_tofb_ctrl & 0x10) == 0x10){
			if (count >= 2) {
			if (!buf) {
					pr_info("hang_detect: invalid user buf\n");
					return -EINVAL;
			}
			if ( count >= 1024 )
			{
			  	printk("hang_detect: log data(%d) is biggest!!  \n",count);
			  	return count;
			}
			memset(logcat_msg, 0x00, 1024);
			if (copy_from_user(logcat_msg, buf, count)) {
		      pr_info("hang_detect: copy from user buff\n");
		      return -EFAULT;
			}
			printk("%d %s",logcat_sum_lines,logcat_msg);
			logcatmsg_screen_wirte(logcat_msg);
			return count;
		}
	}
	else{//boot detect function not oem_uart flag enable

#ifdef CONFIG_TCL_FORCE_DAILY_KEY_DETECT
			if ( get_boot_mode() == 0 && current_version == 0 && kernel_printk_tofb_ctrl != 0x28)
			{
				if (count >= 2) {
				if (!buf) {
						pr_info("hang_detect: invalid user buf\n");
						return -EINVAL;
				}
				if ( count >= 1024 )
				{
				  	printk("hang_detect: log data(%d) is biggest!!  \n",count);
				  	return count;
				}
				memset(logcat_msg, 0x00, 1024);
				if (copy_from_user(logcat_msg, buf, count)) {
			      pr_info("hang_detect: copy from user buff\n");
			      return -EFAULT;
				}
				printk("%d_K %s",logcat_sum_lines,logcat_msg);
				logcatmsg_screen_wirte(logcat_msg);
				return count;
				}
			}
#endif
	}
	if (count >= 2) {
		pr_info("hang_detect: invalid input 727\n");
		return -EINVAL;
	}
#endif


		
	if (!buf) {
		pr_info("hang_detect: invalid user buf\n");
		return -EINVAL;
	}

	if (copy_from_user(msg, buf, count)) {
		pr_info("hang_detect: failed to copy from user\n");
		return -EFAULT;
	}

	if (strncmp(current->comm, "init", 4))
		return  -EINVAL;

	if (msg[0] == '0') {
		hd_detect_enabled = false;
		hd_zygote_stopped = true;
		pr_info("hang_detect: disable by stop cmd\n");
	} else if (msg[0] == '1') {
		if (hd_zygote_stopped) {
			hd_detect_enabled = true;
			hd_zygote_stopped = false;
			pr_info("hang_detect: enable by start cmd\n");
		} else {
			pr_info("hang_detect: zygote running\n");
		}
	} else {
		pr_info("hang_detect: invalid control msg\n");
	}
	return count;
}

static long monitor_hang_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;
	static long long monitor_status;
	void __user *argp = (void __user *)arg;
	char name[TASK_COMM_LEN] = {0};
#ifdef CONFIG_TCL_BOOT_DETECT
	char reboot_tast_name[64] = {0};
	int i;
#endif
	if (cmd == HANG_KICK) {
		pr_info("hang_detect HANG_KICK ( %d)\n", (int)arg);
		MonitorHangKick((int)arg);
		return ret;
	}

	if ((cmd == HANG_SET_SF_STATE) &&
		(!strncmp(current->comm, "surfaceflinger", 10) ||
		!strncmp(current->comm, "SWWatchDog", 10))) {
		if (copy_from_user(&monitor_status, argp, sizeof(long long)))
			ret = -EFAULT;
		return ret;
	} else if (cmd == HANG_GET_SF_STATE) {
		if (copy_to_user(argp, &monitor_status,	sizeof(long long)))
			ret = -EFAULT;
		return ret;
	}


#ifdef CONFIG_MTK_HANG_DETECT_DB
#ifdef CONFIG_TCL_BOOT_DETECT
	if (cmd == HANG_SET_REBOOT) {
			reboot_flag = true;
			hang_detect_counter = 5;
			hd_timeout = 5;
			hd_detect_enabled = true;
			pr_info("hang_detect: %s set reboot command.\n", current->comm);
			if((kernel_printk_tofb_ctrl & 0x10) == 0x10)
			{
				if (copy_from_user(reboot_tast_name, argp, 64))
					ret = -EFAULT;

				pr_info("hang_detect:reason: [%s] tofb=[%d]\n", reboot_tast_name,kernel_printk_tofb_ctrl);
				if ( reboot_tast_name == NULL || get_boot_mode() != 0)
				{
					pr_info("hang_detect:reason: [%s]=NULL not trigger! get_boot_mode()=%d \n", reboot_tast_name,get_boot_mode());
					return ret;
				}
				if (
		    	  	!strcmp(reboot_tast_name, "reboot,userspace_failed,watchdog_fork") ||
		    	  	!strcmp(reboot_tast_name, "reboot,userspace_failed,shutdown_aborted") ||
		    	  	!strcmp(reboot_tast_name, "reboot,userspace_failed,init_user0") ||
		    	  	!strcmp(reboot_tast_name, "reboot,userspace_failed,enablefilecrypto") ||
		    	  	!strcmp(reboot_tast_name, "reboot,userspace_failed,enablefilecrypto") ||
		    	  	!strcmp(reboot_tast_name, "reboot,userspace_failed,watchdog_triggered") ||
		    	  	!strcmp(reboot_tast_name, "reboot,userspace_failed,*") ||
							!strcmp(reboot_tast_name, "reboot,userspace_failed,watchdog_triggered,failed_to_start") ||
							!strcmp(reboot_tast_name, "reboot,userspace_failed,watchdog_triggered,failed_to_boot") ||
							!strcmp(reboot_tast_name, "reboot,mount_userdata_failed") ||
							!strcmp(reboot_tast_name, "reboot,mount_userdata_failed,mount_userdata_failed") ||
							!strcmp(reboot_tast_name, "reboot,recovery,init_user0_failed") ||
		    	  	!strcmp(reboot_tast_name, "reboot,recovery,enablefilecrypto_failed") ||
		    	  	!strcmp(reboot_tast_name, "reboot,recovery,set_policy_failed") ||
		    	  	!strcmp(reboot_tast_name, "reboot,recovery,RescueParty") ||
		    	  	!strcmp(reboot_tast_name, "reboot,recovery,rescueparty") ||
		    	  	!strcmp(reboot_tast_name, "reboot,vold-failed") ||
		    	  	!strcmp(reboot_tast_name, "reboot,apexd-failed") ||
		    	  	!strcmp(reboot_tast_name, "reboot,boringssl-self-check-failed") ||
		    	  	!strcmp(reboot_tast_name, "reboot,powerloss") ||
		    	  	!strcmp(reboot_tast_name, "reboot,rescueparty") ||
		    	  	!strcmp(reboot_tast_name, "reboot,RescueParty") ||
		    	  	!strcmp(reboot_tast_name, "reboot,undervoltage") ||
		    	  	!strcmp(reboot_tast_name, "reboot,wdt") ||
		    	  	!strcmp(reboot_tast_name, "shutdown,thermal") ||
		    	  	!strcmp(reboot_tast_name, "shutdown,thermal,battery") ||
		    	  	!strcmp(reboot_tast_name, "reboot,dm-verity_device_corrupted"))
		    	   {

		    	  	DAL_SetColor(0xFFFFFF,0x0000FF);
		    	  	DAL_SetScreenColor(0x0000FF);
		    	  	DAL_Printf("\n");
		    	  	DAL_Printf("\n");
		    	  	DAL_Printf("\n");
		    	  	//DAL_Printf("NORMAL=0,META=1,RECOVERY=2,SW=3,FACTORY=4,ADVMETA=5,ATE_FACTORY=6");
		    	  	//DAL_Printf("ALARM=7,KERNEL_POWER_OFF_CHARGING=8,LOW_POWER_OFF_CHARGING=9,DONGLE=10\n");
		    	  	pr_info("hang_detect:reason: [%s] kernel display trigger !\n", reboot_tast_name);
							print_ver_info();
		    	  	DAL_Printf("ERROR: reason [%s] current->comm=[%s] bmode&tofb=[%d,0x%x,%d] \n",
		    	  	reboot_tast_name,current->comm,get_boot_mode(),kernel_printk_tofb_ctrl,current_version);
		    	  	boot_detect_tringger();
		    	  	DAL_Printf("ERROR: reason [%s] current->comm=[%s] bmode&tofb=[%d,0x%x,%d] \n",
		    	  	reboot_tast_name,current->comm,get_boot_mode(),kernel_printk_tofb_ctrl,current_version);
		    	  	pr_info("hang_detect reason [%s] current->comm=[%s] bmode&tofb=[%d,%d] \n",reboot_tast_name,current->comm,get_boot_mode(),kernel_printk_tofb_ctrl);
		    	  	DAL_Printf("\nSYSTEM REBOOT, BYE BYE ! bmode: NORMAL=0,META=1,RECOVERY=2,SW=3,FACTORY=4 \n");
					    for(i=0;i<120;i++)
					    {
							DAL_Printf("-");
							mdelay(1* 1000);
							hold_lcd_backlight();
							}
						}
		    else{
		    	pr_info("hang_detect:reason: [%s] reboot not display !\n", reboot_tast_name);
		    	  	//DAL_Printf(" NO DISPLAY REASON ERROR :reason [%s] current->comm=%s \n",reboot_tast_name,current->comm);
		    }
			}
			else// kernel_printk_tofb_ctrl not control
				pr_info("hang_detect:reason: kernel display not trigger tofb[%d] !\n",kernel_printk_tofb_ctrl);
			return ret;
	}
#endif
#endif
	if (cmd == HANG_ADD_WHITE_LIST) {
		if (copy_from_user(name, argp, TASK_COMM_LEN - 1))
			ret = -EFAULT;
		ret = add_white_list(name);
		pr_info("hang_detect: add white list %s status %d.\n",
			name, ret);
		return ret;
	}

	if (cmd == HANG_DEL_WHITE_LIST) {
		if (copy_from_user(name, argp, TASK_COMM_LEN - 1))
			ret = -EFAULT;
		ret = del_white_list(name);
		pr_info("hang_detect: del white list %s status %d.\n",
			name, ret);
		return ret;
	}

	return ret;
}


static const struct file_operations Hang_Monitor_fops = {
	.owner = THIS_MODULE,
	.open = monitor_hang_open,
	.release = monitor_hang_release,
	.poll = monitor_hang_poll,
	.read = monitor_hang_read,
	.write = monitor_hang_write,
	.unlocked_ioctl = monitor_hang_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = monitor_hang_ioctl,
#endif
};

static struct miscdevice Hang_Monitor_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "RT_Monitor",
	.fops = &Hang_Monitor_fops,
};

static int FindTaskByName(char *name)
{
	struct task_struct *task;
	int ret = -1;

	if (!name)
		return ret;

#ifdef MODULE
	read_lock(Ptasklist_lock);
#else
	read_lock(&tasklist_lock);
#endif
	for_each_process(task) {
		if (task && !strncmp(task->comm, name, strlen(name))) {
			pr_info("[Hang_Detect] %s found pid:%d.\n",
					task->comm, task->pid);
			ret = task->pid;
			break;
		}
	}
#ifdef MODULE
	read_unlock(Ptasklist_lock);
#else
	read_unlock(&tasklist_lock);
#endif
	return ret;
}

static void Log2HangInfo(const char *fmt, ...)
{
#ifdef CONFIG_MTK_HANG_DETECT_DB
	unsigned long len = 0;
	va_list ap;

	if ((Hang_Info_Size + MAX_STRING_SIZE) >=
			(unsigned long)MaxHangInfoSize)
		return;

	va_start(ap, fmt);
	len = vscnprintf(&Hang_Info[Hang_Info_Size], MAX_STRING_SIZE, fmt, ap);
	va_end(ap);
	Hang_Info_Size += len;
#endif
}

#ifdef CONFIG_MTK_HANG_DETECT_DB
#ifndef MODULE
static void Buff2HangInfo(const char *buff, unsigned long size)
{
	if (((unsigned long)Hang_Info_Size + size)
		>= (unsigned long)MaxHangInfoSize)
		return;

	memcpy(&Hang_Info[Hang_Info_Size], buff, size);
	Hang_Info_Size += size;

}

static void DumpMsdc2HangInfo(void)
{
	char *buff_add = NULL;
	unsigned long buff_size = 0;

	if (get_msdc_aee_buffer) {
		get_msdc_aee_buffer((unsigned long *)&buff_add, &buff_size);
		if (buff_size != 0 && buff_add) {
			if (buff_size > 30 * 1024) {
				buff_add = buff_add + buff_size - 30 * 1024;
				buff_size = 30*1024;
			}
			Buff2HangInfo(buff_add, buff_size);
		}
	}
}

static void DumpMemInfo(void)
{
	char *buff_add = NULL;
	int buff_size = 0;

	if (mlog_get_buffer) {
		mlog_get_buffer(&buff_add, &buff_size);
		if (buff_size <= 0 || !buff_add) {
			pr_info("hang_detect: mlog_get_buffer size %d.\n",
				buff_size);
			return;
		}

		if (buff_size > 3*1024) {
			buff_add = buff_add + buff_size - 3*1024;
			buff_size = 3*1024;
		}

		Buff2HangInfo(buff_add, buff_size);
	}
}
#endif

void get_hang_detect_buffer(unsigned long *addr, unsigned long *size,
			    unsigned long *start)
{
	*addr = (unsigned long)Hang_Info;
	*start = 0;
	*size = MaxHangInfoSize;
}

void trigger_hang_detect_db(void)
{
	pr_notice("[Hang_Detect] we  triger DB.\n");

#ifdef CONFIG_MTK_AEE_IPANIC
	aee_rr_rec_hang_detect_timeout_count(hd_timeout);
	if ((!watchdog_thread_exist & system_server_exist)
		&& reboot_flag == false)
		aee_rr_rec_hang_detect_timeout_count(COUNT_ANDROID_REBOOT);
#endif

#ifdef CONFIG_MTK_ENG_BUILD
	if (monit_hang_flag == 1) {
#endif
#ifdef CONFIG_MTK_AEE_IPANIC
		mrdump_mini_add_hang_raw((unsigned long)Hang_Info,
			MaxHangInfoSize);
		mrdump_mini_add_extra_misc();
		mrdump_common_die(AEE_FIQ_STEP_HANG_DETECT,
		AEE_REBOOT_MODE_HANG_DETECT,
		"Hang Detect", NULL);
#else
		mrdump_common_die(0, AEE_REBOOT_MODE_HANG_DETECT,
		"Hang Detect", NULL);
#endif

#ifdef CONFIG_MTK_ENG_BUILD
	}
#endif

}
#endif


#ifdef CONFIG_STACKTRACE
/* copy from arch/armxx/kernel/stacktrace.c file */
/* Linux will skip shed and lock function address */
/* We need this information for hand issue although it have some risk*/
#ifdef __aarch64__		/* 64bit */
struct stack_trace_data {
	struct stack_trace *trace;
	unsigned int no_sched_functions;
	unsigned int skip;
};

static int save_trace(struct stackframe *frame, void *d)
{
	struct stack_trace_data *data = d;
	struct stack_trace *trace = data->trace;
	unsigned long addr = frame->pc;

#ifdef MODULE
	if (data->no_sched_functions && Pin_sched_functions(addr))
#else
	if (data->no_sched_functions && in_sched_functions(addr))
#endif
		return 0;
	if (data->skip) {
		data->skip--;
		return 0;
	}

	trace->entries[trace->nr_entries++] = addr;

	return trace->nr_entries >= trace->max_entries;
}

static void save_stack_trace_tsk_me(struct task_struct *tsk,
	struct stack_trace *trace)
{
	struct stack_trace_data data;
	struct stackframe frame;

	data.trace = trace;
	data.skip = trace->skip;

	if (tsk != current) {
		data.no_sched_functions = 0; /* modify to 0 */
		frame.fp = thread_saved_fp(tsk);
		/* frame.sp = thread_saved_sp(tsk); */
		frame.pc = thread_saved_pc(tsk);
	} else {
		data.no_sched_functions = 0;
		frame.fp = (unsigned long)__builtin_frame_address(0);
		/* frame.sp = current_stack_pointer; */
		frame.pc = (unsigned long)save_stack_trace_tsk_me;
	}
#ifdef CONFIG_FUNCTION_GRAPH_TRACER
	frame.graph = tsk->curr_ret_stack;
#endif

#if defined(MODULE) && defined(__aarch64__)
	Pwalk_stackframe(tsk, &frame, save_trace, &data);
#else
	walk_stackframe(tsk, &frame, save_trace, &data);
#endif
	if (trace->nr_entries < trace->max_entries)
		trace->entries[trace->nr_entries++] = ULONG_MAX;
}


#else
struct stack_trace_data {
	struct stack_trace *trace;
	unsigned long last_pc;
	unsigned int no_sched_functions;
	unsigned int skip;
};

static int save_trace(struct stackframe *frame, void *d)
{
	struct stack_trace_data *data = d;
	struct stack_trace *trace = data->trace;
	struct pt_regs *regs;
	unsigned long addr = frame->pc;

#ifdef MODULE
	if (data->no_sched_functions && Pin_sched_functions(addr))
#else
	if (data->no_sched_functions && in_sched_functions(addr))
#endif
		return 0;
	if (data->skip) {
		data->skip--;
		return 0;
	}

	trace->entries[trace->nr_entries++] = addr;

	if (trace->nr_entries >= trace->max_entries)
		return 1;

	/*
	 * in_exception_text() is designed to test if the PC is one of
	 * the functions which has an exception stack above it, but
	 * unfortunately what is in frame->pc is the return LR value,
	 * not the saved PC value.  So, we need to track the previous
	 * frame PC value when doing this.
	 */
	data->last_pc = frame->pc;

#ifdef __aarch64__
	addr = data->last_pc;
	if (!in_exception_text(addr))
		return 0;
#else
	if (!in_entry_text(frame->pc))
		return 0;
#endif

	regs = (struct pt_regs *)frame->sp;

	trace->entries[trace->nr_entries++] = regs->ARM_pc;

	return trace->nr_entries >= trace->max_entries;
}

/* This must be noinline to so that our skip calculation works correctly */
static noinline void __save_stack_trace(struct task_struct *tsk,
	struct stack_trace *trace, unsigned int nosched)
{
	struct stack_trace_data data;
	struct stackframe frame;

	data.trace = trace;
	data.last_pc = ULONG_MAX;
	data.skip = trace->skip;
	data.no_sched_functions = nosched;

	if (tsk != current) {
		frame.fp = thread_saved_fp(tsk);
		frame.sp = thread_saved_sp(tsk);
		frame.lr = 0;		/* recovered from the stack */
		frame.pc = thread_saved_pc(tsk);
	} else {
		/* We don't want this function nor the caller */
		data.skip += 2;
		frame.fp = (unsigned long)__builtin_frame_address(0);
		frame.sp = current_stack_pointer;
		frame.lr = (unsigned long)__builtin_return_address(0);
		frame.pc = (unsigned long)__save_stack_trace;
	}

#if defined(MODULE) && defined(__aarch64__)
	Pwalk_stackframe(tsk, &frame, save_trace, &data);
#else
	walk_stackframe(&frame, save_trace, &data);
#endif
	if (trace->nr_entries < trace->max_entries)
		trace->entries[trace->nr_entries++] = ULONG_MAX;
}

static void save_stack_trace_tsk_me(struct task_struct *tsk,
	struct stack_trace *trace)
{
	__save_stack_trace(tsk, trace, 0); /* modify to 0*/
}

#endif

static void get_kernel_bt(struct task_struct *tsk)
{
	struct stack_trace trace;
	unsigned long stacks[32];
	int i;

	trace.entries = stacks;
	/*save backtraces */
	trace.nr_entries = 0;
	trace.max_entries = 32;
	trace.skip = 0;
	save_stack_trace_tsk_me(tsk, &trace);
	for (i = 0; i < trace.nr_entries; i++) {
		Log2HangInfo("<%lx> %pS\n", (long)trace.entries[i],
				(void *)trace.entries[i]);
		hang_log("<%lx> %pS\n", (long)trace.entries[i],
				(void *)trace.entries[i]);
	}
}

#endif

static long long nsec_high(unsigned long long nsec)
{
	if ((long long)nsec < 0) {
		nsec = -nsec;
		do_div(nsec, 1000000);
		return -nsec;
	}
	do_div(nsec, 1000000);

	return nsec;
}

static unsigned long nsec_low(unsigned long long nsec)
{
	if ((long long)nsec < 0)
		nsec = -nsec;

	return do_div(nsec, 1000000);
}

void show_thread_info(struct task_struct *p, bool dump_bt)
{
	unsigned int state;
	char stat_nam[] = TASK_STATE_TO_CHAR_STR;

	state = p->state ? __ffs(p->state) + 1 : 0;

	Log2HangInfo("%-15.15s %c ", p->comm,
		state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?');
	hang_log("%-15.15s %c ", p->comm,
		state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?');
	Log2HangInfo("%lld.%06ld %d %lu %lu 0x%x 0x%lx %d ",
		nsec_high(p->se.sum_exec_runtime),
		nsec_low(p->se.sum_exec_runtime),
		task_pid_nr(p), p->nvcsw, p->nivcsw, p->flags,
		(unsigned long)task_thread_info(p)->flags,
		p->tgid);
	hang_log("%lld.%06ld %d %lu %lu 0x%x 0x%lx %d ",
		nsec_high(p->se.sum_exec_runtime),
		nsec_low(p->se.sum_exec_runtime),
		task_pid_nr(p), p->nvcsw, p->nivcsw, p->flags,
		(unsigned long)task_thread_info(p)->flags,
		p->tgid);
#ifdef CONFIG_SCHED_INFO
	Log2HangInfo("%llu", p->sched_info.last_arrival);
	hang_log("%llu", p->sched_info.last_arrival);
#endif
	Log2HangInfo("\n");

	/* nvscw: voluntary context switch.  */
	/* requires a resource that is unavailable. */
	/* nivcsw: involuntary context switch. */
	/* time slice out or when higher-priority thread to run*/
#ifdef	CONFIG_MTK_HANG_DETECT_DB
	if (!strcmp(p->comm, "watchdog"))
		watchdog_thread_exist = true;
	if (!strcmp(p->comm, "system_server"))
		system_server_exist = true;
#endif

#ifdef CONFIG_STACKTRACE
	if (dump_bt || ((p->state == TASK_RUNNING ||
			p->state & TASK_UNINTERRUPTIBLE) &&
			!strstr(p->comm, "wdtk")))
	/* Catch kernel-space backtrace */
		get_kernel_bt(p);
#endif
}

static int DumpThreadNativeMaps_log(pid_t pid, struct task_struct *current_task)
{
	struct vm_area_struct *vma;
	int mapcount = 0;
	struct file *file;
	int flags;
	struct mm_struct *mm;
	struct pt_regs *user_ret;
	char tpath[512];
	char *path_p = NULL;
	struct path base_path;

	if (!current_task)
		return -ESRCH;
	user_ret = task_pt_regs(current_task);

	if (!user_mode(user_ret)) {
		pr_info(" %s,%d:%s: in user_mode",
			__func__, pid, current_task->comm);
		return -1;
	}

	if (!get_task_mm(current_task)) {
		pr_info(" %s,%d:%s: current_task->mm == NULL",
			__func__, pid, current_task->comm);
		return -1;
	}

	down_read(&current_task->mm->mmap_sem);
	vma = current_task->mm->mmap;
	pr_info("Dump native maps files:\n");
	while (vma && (mapcount < current_task->mm->map_count)) {
		file = vma->vm_file;
		flags = vma->vm_flags;
		if (file) {	/* !!!!!!!!only dump 1st mmaps!!!!!!!!!!!! */
			if (flags & VM_EXEC) {
			/* we only catch code section for reduce maps space */
				base_path = file->f_path;
				path_p = d_path(&base_path, tpath, 512);
				pr_info("%08lx-%08lx %c%c%c%c    %s\n",
				vma->vm_start,
				vma->vm_end, flags & VM_READ ? 'r' : '-',
				flags & VM_WRITE ? 'w' : '-',
				flags & VM_EXEC ? 'x' : '-',
				flags & VM_MAYSHARE ? 's' : 'p', path_p);
			}
		} else {
#ifdef MODULE
			const char *name = Parch_vma_name(vma);
#else
			const char *name = arch_vma_name(vma);
#endif

			mm = vma->vm_mm;
			if (!name) {
				if (mm) {
					if (vma->vm_start <= mm->start_brk &&
					    vma->vm_end >= mm->brk) {
						name = "[heap]";
				} else if (vma->vm_start <= mm->start_stack &&
					vma->vm_end >= mm->start_stack) {
					name = "[stack]";
				}
				} else {
					name = "[vdso]";
				}
			}

			if (flags & VM_EXEC) {
				pr_info("%08lx-%08lx %c%c%c%c %s\n",
				vma->vm_start,
				vma->vm_end, flags & VM_READ ? 'r' : '-',
				flags & VM_WRITE ? 'w' : '-',
				flags & VM_EXEC ? 'x' : '-',
				flags & VM_MAYSHARE ? 's' : 'p', name);
			}
		}
		vma = vma->vm_next;
		mapcount++;
	}
	up_read(&current_task->mm->mmap_sem);
	mmput(current_task->mm);

	return 0;
}

static int DumpThreadNativeInfo_By_tid_log(pid_t tid,
	struct task_struct *current_task)
{

	struct pt_regs *user_ret;
	struct vm_area_struct *vma;
	int ret = -1;

	if (!current_task)
		return -ESRCH;
	user_ret = task_pt_regs(current_task);

	if (!user_mode(user_ret)) {
		pr_info(" %s,%d:%s,fail in user_mode",
			__func__, tid, current_task->comm);
		return ret;
	}

	if (!current_task->mm) {
		pr_info(" %s,%d:%s, current_task->mm == NULL",
			__func__, tid, current_task->comm);
		return ret;
	}

#ifndef __aarch64__		/* 32bit */
{
	unsigned int tmpfp, tmp, tmpLR;
	unsigned int native_bt[16];
	unsigned int userstack_start = 0;
	unsigned int userstack_end = 0;
	int copied, frames;

	pr_info(" pc/lr/sp 0x%lx/0x%lx/0x%lx\n",
		(long)(user_ret->ARM_pc), (long)(user_ret->ARM_lr),
	     (long)(user_ret->ARM_sp));
	pr_info("r12-r0 0x%lx/0x%lx/0x%lx/0x%lx\n",
		(long)(user_ret->ARM_ip), (long)(user_ret->ARM_fp),
		(long)(user_ret->ARM_r10), (long)(user_ret->ARM_r9));
	pr_info("0x%lx/0x%lx/0x%lx/0x%lx/0x%lx\n",
		(long)(user_ret->ARM_r8), (long)(user_ret->ARM_r7),
		(long)(user_ret->ARM_r6), (long)(user_ret->ARM_r5),
		(long)(user_ret->ARM_r4));
	pr_info("0x%lx/0x%lx/0x%lx/0x%lx\n",
		(long)(user_ret->ARM_r3), (long)(user_ret->ARM_r2),
		(long)(user_ret->ARM_r1), (long)(user_ret->ARM_r0));

	userstack_start = (unsigned long)user_ret->ARM_sp;

	down_read(&current_task->mm->mmap_sem);
	vma = current_task->mm->mmap;
	while (vma) {
		if (vma->vm_start <= userstack_start &&
			vma->vm_end >= userstack_start) {
			userstack_end = vma->vm_end;
			break;
		}
		vma = vma->vm_next;
		if (vma == current_task->mm->mmap)
			break;
	}
	up_read(&current_task->mm->mmap_sem);

	if (userstack_end == 0) {
		pr_info(" %s,%d:%s,userstack_end == 0",
			__func__, tid, current_task->comm);
		return ret;
	}

	native_bt[0] = user_ret->ARM_pc;
	native_bt[1] = user_ret->ARM_lr;	/* lr */
	frames = 2;
	tmpfp = user_ret->ARM_fp;
	while ((unsigned long)tmpfp < userstack_end &&
		(unsigned long)tmpfp > userstack_start) {
		copied = access_process_vm(current_task,
			(unsigned long)tmpfp, &tmp,
			sizeof(tmp), 0);
		if (copied != sizeof(tmp)) {
			pr_info("access_process_vm	fp error\n");
			return -EIO;
		}
		if (((unsigned long)tmp >= userstack_start) &&
			((unsigned long)tmp <= userstack_end - 4)) {
			/* CLANG */
			copied = access_process_vm(current_task,
				(unsigned long)tmpfp + 4,
				&tmpLR, sizeof(tmpLR), 0);
			if (copied != sizeof(tmpLR)) {
				pr_info("access_process_vm	pc error\n");
				return -EIO;
			}
			tmpfp = tmp;
			native_bt[frames] = tmpLR - 4;
			frames++;
		} else {
			copied = access_process_vm(current_task,
				(unsigned long)tmpfp - 4,
				&tmpLR, sizeof(tmpLR), 0);
			if (copied != sizeof(tmpLR)) {
				pr_info("access_process_vm	pc error\n");
				return -EIO;
			}
			tmpfp = tmpLR;
			native_bt[frames] = tmp - 4;
			frames++;
		}
		if (frames >= 16)
			break;
	}
	for (copied = 0; copied < frames; copied++)
		pr_info("#%d pc %x\n", copied, native_bt[copied]);

	pr_info("tid(%d:%s), frame %d. tmpfp(0x%x),userstack_start(0x%x),userstack_end(0x%x)\n",
		tid, current_task->comm, frames, tmpfp,
		userstack_start, userstack_end);
}
#else
	/* K64_U32 for current task */
	if (compat_user_mode(user_ret)) {	/* K64_U32 for check reg */
		unsigned int tmpfp, tmp, tmpLR;
		unsigned long native_bt[16];
		unsigned long userstack_start = 0;
		unsigned long userstack_end = 0;
		int copied, frames;

		pr_info("K64+ U32 pc/lr/sp 0x%lx/0x%lx/0x%lx\n",
			(long)(user_ret->user_regs.pc),
			(long)(user_ret->user_regs.regs[14]),
			(long)(user_ret->user_regs.regs[13]));
		pr_info("r12-r0 0x%lx/0x%lx/0x%lx/0x%lx\n",
			(long)(user_ret->user_regs.regs[12]),
			(long)(user_ret->user_regs.regs[11]),
		    (long)(user_ret->user_regs.regs[10]),
		    (long)(user_ret->user_regs.regs[9]));
		pr_info("0x%lx/0x%lx/0x%lx/0x%lx/0x%lx\n",
			(long)(user_ret->user_regs.regs[8]),
			(long)(user_ret->user_regs.regs[7]),
		    (long)(user_ret->user_regs.regs[6]),
		    (long)(user_ret->user_regs.regs[5]),
		    (long)(user_ret->user_regs.regs[4]));
		pr_info("0x%lx/0x%lx/0x%lx/0x%lx\n",
		    (long)(user_ret->user_regs.regs[3]),
		    (long)(user_ret->user_regs.regs[2]),
		    (long)(user_ret->user_regs.regs[1]),
		    (long)(user_ret->user_regs.regs[0]));
		userstack_start = (unsigned long)user_ret->user_regs.regs[13];
		down_read(&current_task->mm->mmap_sem);
		vma = current_task->mm->mmap;
		while (vma) {
			if (vma->vm_start <= userstack_start &&
				vma->vm_end >= userstack_start) {
				userstack_end = vma->vm_end;
				break;
			}
			vma = vma->vm_next;
			if (vma == current_task->mm->mmap)
				break;
		}
		up_read(&current_task->mm->mmap_sem);

		if (userstack_end == 0) {
			pr_info("Dump native stack failed:\n");
			return ret;
		}

		native_bt[0] = user_ret->user_regs.pc;
		native_bt[1] = user_ret->user_regs.regs[14] - 4;	/* lr */
		tmpfp = user_ret->user_regs.regs[11];
		frames = 2;
		while ((unsigned long)tmpfp < userstack_end &&
			(unsigned long)tmpfp > userstack_start) {
			copied = access_process_vm(current_task,
				(unsigned long)tmpfp, &tmp,
				sizeof(tmp), 0);
			if (copied != sizeof(tmp)) {
				pr_info("access_process_vm	fp error\n");
				return -EIO;
			}
			if (((unsigned long)tmp >= userstack_start) &&
				((unsigned long)tmp <= userstack_end - 4)) {
				/* CLANG */
				copied = access_process_vm(current_task,
					(unsigned long)tmpfp + 4,
					&tmpLR, sizeof(tmpLR), 0);
				if (copied != sizeof(tmpLR)) {
					pr_info("access_process_vm	pc error\n");
					return -EIO;
				}
				tmpfp = tmp;
				native_bt[frames] = tmpLR - 4;
				frames++;
			} else {
				copied = access_process_vm(current_task,
					(unsigned long)tmpfp - 4,
					&tmpLR, sizeof(tmpLR), 0);
				if (copied != sizeof(tmpLR)) {
					pr_info("access_process_vm	pc error\n");
					return -EIO;
				}
				tmpfp = tmpLR;
				native_bt[frames] = tmp - 4;
				frames++;
			}
			if (frames >= 16)
				break;
		}
		for (copied = 0; copied < frames; copied++)
			pr_info("#%d pc %lx\n", copied, native_bt[copied]);

		pr_info("tid(%d:%s), frame %d. tmpfp(0x%x),userstack_start(0x%lx),userstack_end(0x%lx)\n",
			tid, current_task->comm, frames,
			tmpfp, userstack_start, userstack_end);
	} else {		/*K64+U64 */
		unsigned long userstack_start = 0;
		unsigned long userstack_end = 0;
		unsigned long tmpfp, tmp, tmpLR;
		unsigned long native_bt[16];
		int copied, frames;

		pr_info(" K64+ U64 pc/lr/sp 0x%16lx/0x%16lx/0x%16lx\n",
		     (long)(user_ret->user_regs.pc),
		     (long)(user_ret->user_regs.regs[30]),
		     (long)(user_ret->user_regs.sp));

		userstack_start = (unsigned long)user_ret->user_regs.sp;

		down_read(&current_task->mm->mmap_sem);
		vma = current_task->mm->mmap;
		while (vma != NULL) {
			if (vma->vm_start <= userstack_start &&
				vma->vm_end >= userstack_start) {
				userstack_end = vma->vm_end;
				break;
			}
			vma = vma->vm_next;
			if (vma == current_task->mm->mmap)
				break;
		}
		up_read(&current_task->mm->mmap_sem);
		if (!userstack_end) {
			pr_info("Dump native stack failed:\n");
			return ret;
		}

		native_bt[0] = user_ret->user_regs.pc;
		native_bt[1] = user_ret->user_regs.regs[30];
		tmpfp = user_ret->user_regs.regs[29];
		frames = 2;
		while (tmpfp < userstack_end && tmpfp > userstack_start) {
			copied = access_process_vm(current_task,
				(unsigned long)tmpfp, &tmp, sizeof(tmp), 0);
			if (copied != sizeof(tmp)) {
				pr_info("access_process_vm  fp error\n");
				return -EIO;
			}
			copied = access_process_vm(current_task,
				(unsigned long)tmpfp + 0x08, &tmpLR,
				sizeof(tmpLR), 0);
			if (copied != sizeof(tmpLR)) {
				pr_info("access_process_vm  pc error\n");
				return -EIO;
			}
			tmpfp = tmp;
			native_bt[frames] = tmpLR;
			frames++;
			if (frames >= 16)
				break;
		}
		for (copied = 0; copied < frames; copied++)
			pr_info("#%d pc %lx\n", copied, native_bt[copied]);

		pr_info("tid(%d:%s),frame %d. tmpfp(0x%lx),userstack_start(0x%lx),userstack_end(0x%lx)\n",
			tid, current_task->comm, frames, tmpfp,
			userstack_start, userstack_end);
	}
#endif

	return 0;
}

void show_native_bt_by_pid(int task_pid)
{
	struct task_struct *t, *p;
	struct pid *pid;
	int count = 0;
	unsigned int state = 0;
	char stat_nam[] = TASK_STATE_TO_CHAR_STR;

	pid = find_get_pid(task_pid);
	t = p = get_pid_task(pid, PIDTYPE_PID);

	if (p && try_get_task_stack(p)) {
		pr_info("show_bt_by_pid: %d: %s.\n", task_pid, t->comm);

		DumpThreadNativeMaps_log(task_pid, p);
		/* catch maps to Userthread_maps */
		/* change send ptrace_stop to send signal stop */
#ifdef MODULE
		Pdo_send_sig_info(SIGSTOP, SEND_SIG_FORCED, p, true);
#else
		do_send_sig_info(SIGSTOP, SEND_SIG_FORCED, p, true);
#endif
		do {
			if (t && try_get_task_stack(t)) {
				pid_t tid = 0;

				get_task_struct(t);
				tid = task_pid_vnr(t);
				state = t->state ? __ffs(t->state) + 1 : 0;
				pr_info("%s sysTid=%d, pid=%d\n",
					t->comm, tid, task_pid);
				DumpThreadNativeInfo_By_tid_log(tid, t);
				/* catch user-space bt */
#ifdef MODULE
				Pput_task_stack(t);
#else
				put_task_stack(t);
#endif
				put_task_struct(t);
			}
			if ((++count) % 5 == 4)
				msleep(20);
		} while_each_thread(p, t);
		/* change send ptrace_stop to send signal stop */
		if (stat_nam[state] != 'T')
#ifdef MODULE
			Pdo_send_sig_info(SIGSTOP, SEND_SIG_FORCED, p, true);
#else
			do_send_sig_info(SIGCONT, SEND_SIG_FORCED, p, true);
#endif
#ifdef MODULE
		Pput_task_stack(t);
#else
		put_task_stack(p);
#endif
		put_task_struct(p);
	} else if (p)
		put_task_struct(p);
	put_pid(pid);
}
EXPORT_SYMBOL(show_native_bt_by_pid);



static int DumpThreadNativeMaps(pid_t pid, struct task_struct *current_task)
{
	struct vm_area_struct *vma;
	int mapcount = 0;
	struct file *file;
	int flags;
	struct mm_struct *mm;
	struct pt_regs *user_ret;
	char tpath[512];
	char *path_p = NULL;
	struct path base_path;
	unsigned long long pgoff = 0;

	if (!current_task)
		return -ESRCH;
	user_ret = task_pt_regs(current_task);

	if (!user_mode(user_ret)) {
		pr_info(" %s,%d:%s: in user_mode", __func__, pid,
				current_task->comm);
		return -1;
	}

	if (!current_task->mm) {
		pr_info(" %s,%d:%s: current_task->mm == NULL", __func__, pid,
				current_task->comm);
		return -1;
	}

	down_read(&current_task->mm->mmap_sem);
	vma = current_task->mm->mmap;
	Log2HangInfo("Dump native maps files:\n");
	hang_log("Dump native maps files:\n");
	while (vma && (mapcount < current_task->mm->map_count)) {
		file = vma->vm_file;
		flags = vma->vm_flags;
		pgoff = ((loff_t)vma->vm_pgoff) << PAGE_SHIFT;
		if (file) {	/* !!!!!!!!only dump 1st mmaps!!!!!!!!!!!! */
			if (flags & VM_EXEC) {
				/* we only catch code section for reduce
				 * maps space
				 */
				base_path = file->f_path;
				path_p = d_path(&base_path, tpath, 512);
				Log2HangInfo("%08lx-%08lx %c%c%c%c %08llx %s\n",
					vma->vm_start, vma->vm_end,
					flags & VM_READ ? 'r' : '-',
					flags & VM_WRITE ? 'w' : '-',
					flags & VM_EXEC ? 'x' : '-',
					flags & VM_MAYSHARE ? 's' : 'p',
					pgoff, path_p);
				hang_log("%08lx-%08lx %c%c%c%c %08llx %s\n",
					vma->vm_start, vma->vm_end,
					flags & VM_READ ? 'r' : '-',
					flags & VM_WRITE ? 'w' : '-',
					flags & VM_EXEC ? 'x' : '-',
					flags & VM_MAYSHARE ? 's' : 'p',
					pgoff, path_p);
			}
		} else {
#ifdef MODULE
			const char *name = Parch_vma_name(vma);
#else
			const char *name = arch_vma_name(vma);
#endif
			mm = vma->vm_mm;
			if (!name) {
				if (mm) {
					if (vma->vm_start <= mm->start_brk &&
					    vma->vm_end >= mm->brk) {
						name = "[heap]";
					} else if (vma->vm_start <=
							mm->start_stack &&
						   vma->vm_end >=
							mm->start_stack) {
						name = "[stack]";
					}
				} else {
					name = "[vdso]";
				}
			}

			if (flags & VM_EXEC) {
				Log2HangInfo("%08lx-%08lx %c%c%c%c %08llx %s\n",
					vma->vm_start, vma->vm_end,
					flags & VM_READ ? 'r' : '-',
					flags & VM_WRITE ? 'w' : '-',
					flags & VM_EXEC ? 'x' : '-',
					flags & VM_MAYSHARE ? 's' : 'p', pgoff, name);
				hang_log("%08lx-%08lx %c%c%c%c %08llx %s\n",
					vma->vm_start, vma->vm_end,
					flags & VM_READ ? 'r' : '-',
					flags & VM_WRITE ? 'w' : '-',
					flags & VM_EXEC ? 'x' : '-',
					flags & VM_MAYSHARE ? 's' : 'p', pgoff,  name);
			}
		}
		vma = vma->vm_next;
		mapcount++;
	}
	up_read(&current_task->mm->mmap_sem);

	return 0;
}

static int DumpThreadNativeInfo_By_tid(pid_t tid,
	struct task_struct *current_task)
{
	struct pt_regs *user_ret;
	struct vm_area_struct *vma;
	unsigned long userstack_start = 0;
	unsigned long userstack_end = 0, length = 0;
	int ret = -1;

	if (current_task == NULL)
		return -ESRCH;
	user_ret = task_pt_regs(current_task);

	if (!user_mode(user_ret)) {
		pr_info(" %s,%d:%s,fail in user_mode", __func__, tid,
				current_task->comm);
		return ret;
	}

	if (current_task->mm == NULL) {
		pr_info(" %s,%d:%s, current_task->mm == NULL", __func__, tid,
				current_task->comm);
		return ret;
	}
#ifndef __aarch64__		/* 32bit */
	Log2HangInfo(" pc/lr/sp 0x%08x/0x%08x/0x%08x\n", user_ret->ARM_pc,
			user_ret->ARM_lr, user_ret->ARM_sp);
	Log2HangInfo("r12-r0 0x%08x/0x%08x/0x%08x/0x%08x\n",
		(long)(user_ret->ARM_ip), (long)(user_ret->ARM_fp),
		(long)(user_ret->ARM_r10), (long)(user_ret->ARM_r9));
	Log2HangInfo("0x%08x/0x%08x/0x%08x/0x%08x/0x%08x\n",
		(long)(user_ret->ARM_r8), (long)(user_ret->ARM_r7),
		(long)(user_ret->ARM_r6), (long)(user_ret->ARM_r5),
		(long)(user_ret->ARM_r4));
	Log2HangInfo("0x%08x/0x%08x/0x%08x/0x%08x\n",
		(long)(user_ret->ARM_r3), (long)(user_ret->ARM_r2),
		(long)(user_ret->ARM_r1), (long)(user_ret->ARM_r0));

	userstack_start = (unsigned long)user_ret->ARM_sp;

	down_read(&current_task->mm->mmap_sem);
	vma = current_task->mm->mmap;
	while (vma != NULL) {
		if (vma->vm_start <= userstack_start &&
			vma->vm_end >= userstack_start) {
			userstack_end = vma->vm_end;
			break;
		}
		vma = vma->vm_next;
		if (vma == current_task->mm->mmap)
			break;
	}
	up_read(&current_task->mm->mmap_sem);

	if (userstack_end == 0) {
		pr_info(" %s,%d:%s,userstack_end == 0", __func__,
				tid, current_task->comm);
		return ret;
	}
	length = userstack_end - userstack_start;


	/* dump native stack to buffer */
	{
		unsigned long SPStart = 0, SPEnd = 0;
		int tempSpContent[4], copied;

		SPStart = userstack_start;
		SPEnd = SPStart + length;
		Log2HangInfo("UserSP_start:%08x,Length:%x,End:%08x\n",
				SPStart, length, SPEnd);
		while (SPStart < SPEnd) {
			copied =
			    access_process_vm(current_task, SPStart,
					&tempSpContent, sizeof(tempSpContent),
					0);
			if (copied != sizeof(tempSpContent)) {
				pr_info(
				  "access_process_vm  SPStart error,sizeof(tempSpContent)=%x\n"
				  , (unsigned int)sizeof(tempSpContent));
				/* return -EIO; */
			}
			if (tempSpContent[0] != 0 ||
				tempSpContent[1] != 0 ||
				tempSpContent[2] != 0 ||
				tempSpContent[3] != 0) {
				Log2HangInfo("%08x:%08x %08x %08x %08x\n", SPStart,
						tempSpContent[0],
						tempSpContent[1],
						tempSpContent[2],
						tempSpContent[3]);
			}
			SPStart += 4 * 4;
		}
	}
#else	/* 64bit, First deal with K64+U64, the last time to deal with K64+U32 */
	/* K64_U32 for current task */
	if (compat_user_mode(user_ret)) {	/* K64_U32 for check reg */
		Log2HangInfo("K64+ U32 pc/lr/sp 0x%16lx/0x%16lx/0x%16lx\n",
			(long)(user_ret->user_regs.pc),
			(long)(user_ret->user_regs.regs[14]),
			(long)(user_ret->user_regs.regs[13]));
		Log2HangInfo("r12-r0 0x%lx/0x%lx/0x%lx/0x%lx\n",
			(long)(user_ret->user_regs.regs[12]),
			(long)(user_ret->user_regs.regs[11]),
			(long)(user_ret->user_regs.regs[10]),
			(long)(user_ret->user_regs.regs[9]));
		Log2HangInfo("0x%lx/0x%lx/0x%lx/0x%lx/0x%lx\n",
			(long)(user_ret->user_regs.regs[8]),
			(long)(user_ret->user_regs.regs[7]),
			(long)(user_ret->user_regs.regs[6]),
			(long)(user_ret->user_regs.regs[5]),
			(long)(user_ret->user_regs.regs[4]));
		Log2HangInfo("0x%lx/0x%lx/0x%lx/0x%lx\n",
			(long)(user_ret->user_regs.regs[3]),
			(long)(user_ret->user_regs.regs[2]),
			(long)(user_ret->user_regs.regs[1]),
			(long)(user_ret->user_regs.regs[0]));
		userstack_start = (unsigned long)user_ret->user_regs.regs[13];
		down_read(&current_task->mm->mmap_sem);
		vma = current_task->mm->mmap;
		while (vma != NULL) {
			if (vma->vm_start <= userstack_start &&
				vma->vm_end >= userstack_start) {
				userstack_end = vma->vm_end;
				break;
			}
			vma = vma->vm_next;
			if (vma == current_task->mm->mmap)
				break;
		}
		up_read(&current_task->mm->mmap_sem);

		if (userstack_end == 0) {
			pr_info("Dump native stack failed:\n");
			return ret;
		}

		length = userstack_end - userstack_start;

		/*  dump native stack to buffer */
		{
			unsigned long SPStart = 0, SPEnd = 0;
			int tempSpContent[4], copied;

			SPStart = userstack_start;
			SPEnd = SPStart + length;
			Log2HangInfo("UserSP_start:%x,Length:%x,End:%x\n",
				SPStart, length, SPEnd);
			while (SPStart < SPEnd) {
				copied = access_process_vm(current_task,
						SPStart, &tempSpContent,
						sizeof(tempSpContent), 0);
				if (copied != sizeof(tempSpContent)) {
					pr_info(
					  "access_process_vm  SPStart error,sizeof(tempSpContent)=%x\n",
					  (unsigned int)sizeof(tempSpContent));
					/* return -EIO; */
				}
				if (tempSpContent[0] != 0 ||
					tempSpContent[1] != 0 ||
					tempSpContent[2] != 0 ||
					tempSpContent[3] != 0) {
					Log2HangInfo("%08x:%x %x %x %x\n",
							SPStart,
							tempSpContent[0],
							tempSpContent[1],
							tempSpContent[2],
							tempSpContent[3]);
				}
				SPStart += 4 * 4;
			}
		}
	} else {		/*K64+U64 */
		userstack_start = (unsigned long)user_ret->user_regs.sp;

		down_read(&current_task->mm->mmap_sem);
		vma = current_task->mm->mmap;
		while (vma != NULL) {
			if (vma->vm_start <= userstack_start &&
					vma->vm_end >= userstack_start) {
				userstack_end = vma->vm_end;
				break;
			}
			vma = vma->vm_next;
			if (vma == current_task->mm->mmap)
				break;
		}
		up_read(&current_task->mm->mmap_sem);
		if (userstack_end == 0) {
			pr_info("Dump native stack failed:\n");
			return ret;
		}

		{
			unsigned long tmpfp, tmp, tmpLR;
			int copied, frames;
			unsigned long native_bt[16];

			native_bt[0] = user_ret->user_regs.pc;
			native_bt[1] = user_ret->user_regs.regs[30];
			tmpfp = user_ret->user_regs.regs[29];
			frames = 2;
			while (tmpfp < userstack_end &&
					tmpfp > userstack_start) {
				copied =
				    access_process_vm(current_task,
						    (unsigned long)tmpfp, &tmp,
						      sizeof(tmp), 0);
				if (copied != sizeof(tmp)) {
					pr_info("access_process_vm  fp error\n");
					return -EIO;
				}
				copied =
				    access_process_vm(current_task,
						    (unsigned long)tmpfp + 0x08,
						      &tmpLR, sizeof(tmpLR), 0);
				if (copied != sizeof(tmpLR)) {
					pr_info("access_process_vm  pc error\n");
					return -EIO;
				}
				tmpfp = tmp;
				native_bt[frames] = tmpLR;
				frames++;
				if (frames >= 16)
					break;
			}
			for (copied = 0; copied < frames; copied++) {
				/*  #00 pc 000000000006c760
				 *  /system/lib64/ libc.so (__epoll_pwait+8)
				 */
				Log2HangInfo("#%d pc %lx\n", copied,
						native_bt[copied]);
			}
		}
	}
#endif

	return 0;
}

static void show_bt_by_pid(int task_pid)
{
	struct task_struct *t, *p;
	struct pid *pid;
#ifdef __aarch64__
	struct pt_regs *user_ret;
#endif
	int count = 0, dump_native = 0;
	unsigned int state = 0;
	char stat_nam[] = TASK_STATE_TO_CHAR_STR;
	pid = find_get_pid(task_pid);
	t = p = get_pid_task(pid, PIDTYPE_PID);
	if (p != NULL) {
		if (try_get_task_stack(p)) {
			Log2HangInfo("%s: %d: %s.\n", __func__, task_pid, t->comm);
			hang_log("%s: %d: %s.\n", __func__, task_pid, t->comm);
#ifndef __aarch64__	 /* 32bit */
			if (!strcmp(t->comm, "system_server"))
				dump_native = 1;
			else
				dump_native = 0;
#else
			user_ret = task_pt_regs(t);

			if (!user_mode(user_ret)) {
				pr_info(" %s,%d:%s,fail in user_mode", __func__,
						task_pid, t->comm);
				dump_native = 0;
			} else	if (!t->mm) {
				pr_info(" %s,%d:%s, current_task->mm == NULL", __func__,
						task_pid, t->comm);
				dump_native = 0;
			} else if (compat_user_mode(user_ret)) {
				/* K64_U32 for check reg */
				if (!strcmp(t->comm, "system_server"))
					dump_native = 1;
				else
					dump_native = 0;
			} else
				dump_native = 1;
#endif
			if (dump_native == 1)
				/* catch maps to Userthread_maps */
				DumpThreadNativeMaps(task_pid, p);
#ifdef MODULE
			Pput_task_stack(t);
#else
			put_task_stack(p);
#endif
		} else {
			state = p->state ? __ffs(p->state) + 1 : 0;
			Log2HangInfo("%s pid %d state %c, flags %d. stack is null.\n",
				t->comm, task_pid, state < sizeof(stat_nam) - 1 ?
				stat_nam[state] : '?', t->flags);
			hang_log("%s pid %d state %c, flags %d. stack is null.\n",
				t->comm, task_pid, state < sizeof(stat_nam) - 1 ?
				stat_nam[state] : '?', t->flags);
		}
		do {
			if (t && try_get_task_stack(t)) {
				pid_t tid = 0;

				get_task_struct(t);
				tid = task_pid_vnr(t);
				state = t->state ? __ffs(t->state) + 1 : 0;
				/* catch kernel bt */
				show_thread_info(t, true);

				Log2HangInfo("%s sysTid=%d, pid=%d\n", t->comm,
						tid, task_pid);
				hang_log("%s sysTid=%d, pid=%d\n", t->comm,
						tid, task_pid);

				if (dump_native == 1)
					DumpThreadNativeInfo_By_tid(tid, t);
#ifdef MODULE
				Pput_task_stack(t);
#else
				put_task_stack(t);
#endif
				put_task_struct(t);
			}
			if ((++count) % 5 == 4)
				msleep(20);
			Log2HangInfo("-\n");
		} while_each_thread(p, t);
		put_task_struct(p);
	}
	put_pid(pid);
}

static int show_white_list_bt(struct task_struct *p)
{
	struct name_list *pList = NULL;

	if (!white_list)
		return -1;
	raw_spin_lock(&white_list_lock);
	pList = white_list;
	while (pList) {
		if (!strcmp(p->comm, pList->name)) {
			raw_spin_unlock(&white_list_lock);
			show_bt_by_pid(p->pid);
			return 0;
		}
		pList = pList->next;
	}
	raw_spin_unlock(&white_list_lock);
	return -1;
}


static void hang_dump_backtrace(void)
{
	struct task_struct *p, *t, *system_server_task = NULL;
	struct task_struct *monkey_task = NULL;
	struct task_struct *aee_aed_task = NULL;

#ifdef CONFIG_MTK_HANG_DETECT_DB
	watchdog_thread_exist = false;
	system_server_exist = false;
#endif
	Log2HangInfo("dump backtrace start: %llu\n", local_clock());

	rcu_read_lock();
	for_each_process(p) {
		get_task_struct(p);
		if (Hang_Detect_first == false) {
			if (!strcmp(p->comm, "system_server"))
				system_server_task = p;
			if (strstr(p->comm, "monkey"))
				monkey_task = p;
			if (!strcmp(p->comm, "aee_aed"))
				aee_aed_task = p;
		}
		/* specify process, need dump maps file and native backtrace */
		if (!strcmp(p->comm, "surfaceflinger") ||
			!strcmp(p->comm, "init") ||
			!strcmp(p->comm, "system_server") ||
			!strcmp(p->comm, "mmcqd/0")  ||
			!strcmp(p->comm, "debuggerd64") ||
			!strcmp(p->comm, "mmcqd/1") ||
			!strcmp(p->comm, "vold") ||
			!strcmp(p->comm, "vdc") ||
			!strcmp(p->comm, "debuggerd")) {
			show_bt_by_pid(p->pid);
			put_task_struct(p);
			continue;
		}
		//test if there's any process need dump
		if (!show_white_list_bt(p)) {
			put_task_struct(p);
			continue;
		}
		for_each_thread(p, t) {
			if (try_get_task_stack(t)) {
				get_task_struct(t);
				show_thread_info(t, false);
#ifdef MODULE
				Pput_task_stack(t);
#else
				put_task_stack(t);
#endif
				put_task_struct(t);
			}
		}
		put_task_struct(p);
	}
	rcu_read_unlock();
	Log2HangInfo("dump backtrace end.\n");

	if (Hang_Detect_first == false) {
		if (aee_aed_task)
			send_sig_info(SIGUSR1, SEND_SIG_PRIV,
				aee_aed_task);
		if (system_server_task)
#ifdef MODULE
			Pdo_send_sig_info(SIGSTOP, SEND_SIG_FORCED,
				system_server_task, true);
#else
			do_send_sig_info(SIGQUIT, SEND_SIG_FORCED,
				system_server_task, true);
#endif
		if (monkey_task)
#ifdef MODULE
			Pdo_send_sig_info(SIGSTOP, SEND_SIG_FORCED,
				monkey_task, true);
#else
			do_send_sig_info(SIGQUIT, SEND_SIG_FORCED,
				monkey_task, true);
#endif
	}
}

static void ShowStatus(int flag)
{

#ifdef CONFIG_MTK_HANG_DETECT_DB
#ifndef MODULE
	if (Hang_Detect_first)	{ /* the last dump */
		DumpMemInfo();
		DumpMsdc2HangInfo();
	}
#endif
#endif

	hang_dump_backtrace();

	if (Hang_Detect_first)	{ /* the last dump */
		/* debug_locks = 1; */
		debug_show_all_locks();
#ifndef MODULE
		show_free_areas(0, NULL);
		if (show_task_mem)
			show_task_mem();
#endif
#ifdef CONFIG_MTK_ION
		ion_mm_heap_memory_detail();
#endif
#if IS_ENABLED(CONFIG_MTK_GPU_SUPPORT)
		mtk_dump_gpu_memory_usage();
#endif
#ifdef CONFIG_MTK_WQ_DEBUG
		wq_debug_dump();
#endif

	}
}

static int dump_last_thread(void *arg)
{
	/* unsigned long flags; */
	struct sched_param param = {
		.sched_priority = 99
	};
	sched_setscheduler(current, SCHED_FIFO, &param);
	pr_info("[Hang_Detect] dump last thread.\n");
	ShowStatus(1);
	dump_bt_done = 1;
	wake_up_interruptible(&dump_bt_done_wait);
	return 0;
}

static int hang_detect_dump_thread(void *arg)
{
	/* unsigned long flags; */
	struct sched_param param = {
		.sched_priority = 99
	};

	sched_setscheduler(current, SCHED_FIFO, &param);
	msleep(120 * 1000);
	dump_bt_done = 1;
	while (1) {
		wait_event_interruptible(dump_bt_start_wait, dump_bt_done == 0);
		ShowStatus(0);
		dump_bt_done = 1;
		wake_up_interruptible(&dump_bt_done_wait);
	}
	pr_notice("[Hang_Detect] hang_detect dump thread exit.\n");
	return 0;
}

void wake_up_dump(void)
{
	dump_bt_done = 0;
	wake_up_interruptible(&dump_bt_start_wait);
	if (dump_bt_done != 1)
		wait_event_interruptible_timeout(dump_bt_done_wait,
			dump_bt_done == 1, HZ*10);
}


bool CheckWhiteList(void)
{
	struct name_list *pList = NULL;

	if (!white_list)
		return true;


	raw_spin_lock(&white_list_lock);
	pList = white_list;
	while (pList) {
		if (FindTaskByName(pList->name) < 0) {
			/* not fond the Task */
			raw_spin_unlock(&white_list_lock);
			return false;
		}
		pList = pList->next;
	}
	raw_spin_unlock(&white_list_lock);
	return true;
}

static int hang_detect_thread(void *arg)
{
	/* unsigned long flags; */
	struct sched_param param = {
		.sched_priority = 99
	};
	struct task_struct *hd_thread;

	sched_setscheduler(current, SCHED_FIFO, &param);
	reset_hang_info();
	msleep(120 * 1000);
	pr_debug("[Hang_Detect] hang_detect thread starts.\n");

#ifdef BOOT_UP_HANG
	hd_timeout = 9;
	hang_detect_counter = 9;
	hd_detect_enabled = true;
#endif

	while (1) {
		pr_info("[Hang_Detect] hang_detect thread counts down %d:%d, status %d.\n",
			hang_detect_counter, hd_timeout, hd_detect_enabled);
#ifdef BOOT_UP_HANG
		if (hd_detect_enabled)
#else
		if (hd_detect_enabled && CheckWhiteList())
#endif
		{

			if (hang_detect_counter <= 0) {
				Log2HangInfo(
					"[Hang_detect]Dump the %d time process bt.\n",
					Hang_Detect_first ? 2 : 1);
#ifdef CONFIG_MTK_HANG_DETECT_DB
				if (!Hang_Detect_first) {
					memset(Hang_Info, 0, MaxHangInfoSize);
					Hang_Info_Size = 0;
				}
#endif
				if (Hang_Detect_first == true
					&& dump_bt_done != 1) {
		/* some time dump thread will block in dumping native bt */
		/* so create new thread to dump enough kernel bt */
					hd_thread = kthread_create(
						dump_last_thread,
						NULL, "hang_detect2");
					if (hd_thread)
						wake_up_process(hd_thread);
				if (dump_bt_done != 1)
					wait_event_interruptible_timeout(
							dump_bt_done_wait,
							dump_bt_done == 1,
							HZ*10);
				} else
					wake_up_dump();


				if (Hang_Detect_first == true) {
#ifdef CONFIG_MTK_HANG_DETECT_DB
					trigger_hang_detect_db();
#else
					BUG();
#endif
				} else
					Hang_Detect_first = true;
			}
			hang_detect_counter--;
		}

		msleep((HD_INTER) * 1000);
	}
	return 0;
}

void MonitorHangKick(int lParam)
{
	if (reboot_flag) {
		pr_info("[Hang_Detect] in reboot flow.\n");
		return;
	}

	if (lParam == 0) {
		hd_detect_enabled = 0;
		hang_detect_counter = hd_timeout;
		pr_info("[Hang_Detect] hang_detect disabled\n");
	} else if (lParam > 0) {
		/* lParem=0x1000|timeout,only set in aee call when NE in
		 *  system_server so only change hang_detect_counter when
		 *  call from AEE
		 * Others ioctl, will change
		 * hd_detect_enabled & hang_detect_counter
		 */
		if (lParam & 0x1000) {
			hang_detect_counter = hd_timeout =
			  ((long)(lParam & 0x0fff) + HD_INTER - 1) / (HD_INTER);
		} else {
			hd_detect_enabled = 1;
			hang_detect_counter = hd_timeout =
			    ((long)lParam + HD_INTER - 1) / (HD_INTER);
		}

		if (hd_timeout < 10) {
			/* hang detect min timeout is 10 (5min) */
			hang_detect_counter = 10;
			hd_timeout = 10;
		}
		pr_info("[Hang_Detect] hang_detect enabled %d\n", hd_timeout);
	}
	reset_hang_info();
}

int hang_detect_init(void)
{

	struct task_struct *hd_thread;

	pr_debug("[Hang_Detect] Initialize proc\n");
	hd_thread = kthread_create(hang_detect_thread, NULL, "hang_detect");
	if (hd_thread)
		wake_up_process(hd_thread);

	hd_thread = kthread_create(hang_detect_dump_thread, NULL,
			"hang_detect1");
	if (hd_thread)
		wake_up_process(hd_thread);

	return 0;
}

static int __init monitor_hang_init(void)
{
	int err = 0;

	if (!aee_is_enable())
		return err;

#ifdef MODULE
	if (module_fun_init() == 1)
		return 1;
#endif

#ifdef CONFIG_MTK_HANG_DETECT_DB
	Hang_Info = kmalloc(MAX_HANG_INFO_SIZE, GFP_KERNEL);
	if (Hang_Info == NULL)
		return 1;
#endif

	err = misc_register(&Hang_Monitor_dev);
	if (unlikely(err)) {
		pr_notice("failed to register Hang_Monitor_dev device!\n");
		return err;
	}
	hang_detect_init();

#ifdef CONFIG_MTK_ENG_BUILD
	pe = proc_create("monitor_hang", 0664, NULL, &monitor_hang_fops);
	if (!pe)
		return -ENOMEM;
#endif
	return err;
}

static void __exit monitor_hang_exit(void)
{
	if (!aee_is_enable())
		return;

	misc_deregister(&Hang_Monitor_dev);
#ifdef CONFIG_MTK_HANG_DETECT_DB
	/* kfree(NULL) is safe */
	kfree(Hang_Info);
#endif
#ifdef CONFIG_MTK_ENG_BUILD
	if (pe)
		proc_remove(pe);
#endif
}


module_init(monitor_hang_init);
module_exit(monitor_hang_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MonitorHang Driver");
MODULE_AUTHOR("MediaTek Inc.");
