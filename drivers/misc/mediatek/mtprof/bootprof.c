// SPDX-License-Identifier: GPL-2.0

#include <linux/kallsyms.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/proc_fs.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/utsname.h>
#include <linux/uaccess.h>

#ifdef MODULE
#include <linux/tracepoint.h>
#include <trace/events/initcall.h>
#endif

#define BOOT_STR_SIZE 256
#define BUF_COUNT 12
#define LOGS_PER_BUF 80
#define MSG_SIZE 128
// Add by jinggao.zhou for ENCOREVZW-7282, add BOOT DETECT timeout 5 mins 2022/11/01
#ifdef CONFIG_TCL_BOOT_DETECT
#include <mtk_drm_assert_ext.h>
#include <linux/ktime.h>
#include <linux/rtc.h>
extern char kdisplay_bootprof_msg[BOOT_STR_SIZE];
extern char kdisplay_bootprof_msg1[BOOT_STR_SIZE];
extern char kdisplay_bootprof_msg2[BOOT_STR_SIZE];
extern char kdisplay_bootprof_msg3[BOOT_STR_SIZE];
extern char kdisplay_bootprof_msg4[BOOT_STR_SIZE];
extern char kdisplay_bootprof_msg5[BOOT_STR_SIZE];
extern char kdisplay_bootprof_msg6[BOOT_STR_SIZE];
extern char kdisplay_bootprof_msg7[BOOT_STR_SIZE];
extern char kdisplay_bootprof_msg8[BOOT_STR_SIZE];

static struct timer_list bootcomplete_delay_notify_timer;
static struct task_struct *bootcomplete_delay_notify_thread;
static bool bootcomplete_delay_notify_flag;
static DECLARE_WAIT_QUEUE_HEAD(bootcomplete_delay_notify_waiter);
void bootcomplete_fivemins_delay_report(void);
extern int kernel_printk_tofb_ctrl;
extern int boot_detect_kerenl_display_status;
unsigned int get_boot_mode(void);
void klog_check_to_ui(void);
extern int current_version;
#endif
#ifdef CONFIG_HELAEYE_BSP_BOOT_ON
#include <tcl/tkperf.h>
#include <generated/utsrelease.h>
#include <linux/delay.h>
#include <linux/alarmtimer.h>
#ifdef CONFIG_HELAEYE_BSP_TMO24H_TCP_RETRAN_ON
struct alarm hela24_notify_alarm;
static struct task_struct *hela24_notify_thread;
static bool hela24_notify_flag;
static DECLARE_WAIT_QUEUE_HEAD(hela24_notify_waiter);
struct wakeup_source* hela24_notify_lock;
static DEFINE_MUTEX(hela24_notify_mutex);
extern int md_5228_count;
extern int gms_5228_count;
#endif

static struct timer_list hela_delay_notify_timer;
static struct task_struct *hela_delay_notify_thread;
static bool hela_delay_notify_flag;
static DECLARE_WAIT_QUEUE_HEAD(hela_delay_notify_waiter);
static int heraeye_getlastbsp_error(void);

static char *s_ap_platform_info;
char *bootprof_get_ap_platform(void)
{
  	struct device_node *node;
  	int ret;
  	u32 ap_plat_numb;

  	if (!s_ap_platform_info) {
  		node = of_find_compatible_node(NULL, NULL,
  				"mediatek,mddriver");
  		if (!node)
  			return NULL;

  		ret = of_property_read_u32(node,
  				"mediatek,ap_plat_info", &ap_plat_numb);
  		if (ret < 0)
  			return NULL;

  		s_ap_platform_info = kzalloc(16, GFP_KERNEL);
  		scnprintf(s_ap_platform_info, 16, "MT%d", ap_plat_numb);
  	}

  	return s_ap_platform_info;
}


void hela_delay_report_task(struct timer_list *t)
{
  	hela_delay_notify_flag = true;
  	wake_up_interruptible(&hela_delay_notify_waiter);
  	pr_info("hela_boot_delay_report_task 5 min trigger \n");

}


int hela_delay_notify_handler(void *unused)
{

    	wait_event_interruptible(hela_delay_notify_waiter,
  			(hela_delay_notify_flag == true));
  			
      pr_info("hela_boot_delay_notify_handler + \n");
  		/*Notify */
  		heraeye_getlastbsp_error();
  		hela_delay_notify_flag = false;

  	return 0;
}

// Eliminate the problem of exception reported by Tguard after startup, delaying 2 minutes
void hela_twomins_delay_report(void)
{
	  unsigned long hela_delay_notify_interval;
	 	hela_delay_notify_interval = HZ * 60 * 2 ;// test use 2 mins 
  	timer_setup(&hela_delay_notify_timer, hela_delay_report_task, TIMER_DEFERRABLE);
  	mod_timer(&hela_delay_notify_timer, jiffies + hela_delay_notify_interval);

  	hela_delay_notify_thread = kthread_run(hela_delay_notify_handler, 0,
  		"hela_delay_notify_thread");
  	if (IS_ERR(hela_delay_notify_thread))
  		pr_notice("Failed to create hela_delay_notify_thread\n");
  	else
  		pr_info("hela_boot_driver Create hela_delay_notify_thread : done\n");

}

#ifdef CONFIG_HELAEYE_BSP_MODEMCRASH_ON
extern int md_ccci_lasterrcode;
extern char modem_build_ver[64];
char *ccci_get_md_info_str(int md_id);
#endif

#ifdef CONFIG_HELAEYE_BSP_AUDIO_ON

int audio_driver_lasterrcode=AUDIO_DRV_NORMAL;

#endif

#ifdef CONFIG_HELAEYE_BSP_EMMC_UFS_SD_ON
int emmc_ufs_sd_lasterrcode=0;
extern char hela_emmc_errinfo[128];
#endif

#ifdef CONFIG_HELAEYE_BSP_LCD_ON
extern int lcd_driver_lasterrcode;
char mtkfb_hera_lcm_name[128]={"ili7835_hehui_edo_fhdplus_amoled_cmd"};

int lcd_driver_lasterrcode=LCD_DRV_NORMAL;
char *bootprof_get_ap_platform(void);
void heraeye_lcd_fail(int lastcode){
	char heraeye_cur_chipinfo[128] = {0};
  char heraeye_cur_time[128] = {0};
  char kernel_version[16]={0};
  const char *strval = UTS_RELEASE;
  char heraeye_info[64]={0};
  strncpy(kernel_version,strval,8);

  heraeye_curtime_to_str(heraeye_cur_time);
  sprintf(heraeye_cur_chipinfo, "%s_%s_%s_%s",bootprof_get_ap_platform(),heraeye_get_project_str(),kernel_version,mtkfb_hera_lcm_name);
  
  lcd_driver_lasterrcode=lastcode;
  printk("[helaeye]heraeye_getlastbsp_error:lcd_driver_lasterrcode=0x%x \n",lcd_driver_lasterrcode);
   
  switch (lcd_driver_lasterrcode) {
  	case LCD_DRV_INIT_ERROR:
  		strcpy(heraeye_info,"LCD_DRV_INIT_ERROR");
  		break;
  	case LCD_DRV_PARM_ERROR:
  		strcpy(heraeye_info,"LCD_DRV_PARM_ERROR");
  		break;
  	case LCD_DRV_ESD_ERROR:
  		strcpy(heraeye_info,"LCD_DRV_ESD_ERROR");
  		break;
  	case LCD_DRV_RECOVERY_ERROR:
  		strcpy(heraeye_info,"LCD_DRV_RECOVERY_ERROR");
  		break;
  	default:
  		strcpy(heraeye_info,"LCD_ERR_default");
  		WARN(1, "lcd error %d \n",lcd_driver_lasterrcode);
  }

  pr_info("hela_boot_lcd %s %s %s %d %s 0x%x",
            TKPERF_PERF_LCD_INFO,
             heraeye_cur_time,
             heraeye_cur_chipinfo,
             lcd_driver_lasterrcode,
             heraeye_info,
             TKPERF_LCD_EVENTID);


  heraeye_driver_log(TKPERF_DRIVER_LCD,"%s %s %s %d %s 0x%x",
            TKPERF_PERF_LCD_INFO,
             heraeye_cur_time,
             heraeye_cur_chipinfo,
             lcd_driver_lasterrcode,
             heraeye_info,
             TKPERF_LCD_EVENTID);
   }
#endif

#ifdef CONFIG_HELAEYE_BSP_CHARGER_ON

int charger_driver_lasterrcode=CHARGER_DRV_NORMAL;
int charger_driver_lasterrcode_value=0;

enum bat_temp_state_enum {
  	BAT_TEMP_LOW = 0,
  	BAT_TEMP_NORMAL,
  	BAT_TEMP_HIGH
  };

void heraeye_bat_check_temp(int errtype, int charger_driver_value){
	char heraeye_cur_chipinfo[128] = {0};
  char heraeye_cur_time[128] = {0};
  char kernel_version[16]={0};
  char heraeye_info[32]={0};
  const char *strval = UTS_RELEASE;
  strncpy(kernel_version,strval,8);

  heraeye_curtime_to_str(heraeye_cur_time);
  sprintf(heraeye_cur_chipinfo, "%s_%s_%s_%s",bootprof_get_ap_platform(),heraeye_get_project_str(),kernel_version,"pmic");
  charger_driver_lasterrcode=CHARGER_DRV_BATTEMP_ERROR;

  switch (errtype) {
  	case BAT_TEMP_LOW:
  		strcpy(heraeye_info,"BAT_TEMP_LOW");
  		charger_driver_lasterrcode=BAT_TEMP_LOW;
  		break;
  	case BAT_TEMP_HIGH:
  		strcpy(heraeye_info,"BAT_TEMP_HIGH");
  		charger_driver_lasterrcode=BAT_TEMP_HIGH;
  		break;
  	case CHARGER_DRV_PE_ERROR:
  		strcpy(heraeye_info,"CHARGER_DRV_PE_ERROR");
  		charger_driver_lasterrcode=CHARGER_DRV_PE_ERROR;
  		break;
  	case CHARGER_DRV_VBUS_ERROR:	
  		strcpy(heraeye_info,"CHARGER_DRV_VBUS_ERROR");
  		charger_driver_lasterrcode=CHARGER_DRV_VBUS_ERROR;
  		break;

  	default:
  		strcpy(heraeye_info,"CHARGER_ERR_default");
  		WARN(1, "charger error %d \n",charger_driver_lasterrcode);
  }

  pr_info("hela_boot_charger %s %s %s %d %d %s 0x%x",
            TKPERF_PERF_CHARGER_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            charger_driver_lasterrcode,
            charger_driver_value,
            heraeye_info,
            TKPERF_CHARGER_EVENTID);

  heraeye_driver_log(TKPERF_DRIVER_CHARGER,"%s %s %s %d %d %s 0x%x",
            TKPERF_PERF_CHARGER_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            charger_driver_lasterrcode,
            charger_driver_value,
            heraeye_info,
            TKPERF_CHARGER_EVENTID);
   }



#endif

#ifdef CONFIG_HELAEYE_BSP_SENSOR_ON
int sensor_driver_lasterrcode=0;
int sensor_driver_lasterrcode_id=0;
int sensor_driver_lasterrcode_cmd_or_errcode=0;
#define    SENSOR_HUB_GET_DATA		2


void heraeye_sensor_fail(int errtype, int sensorid, int cmd_or_errcode){
	char heraeye_cur_chipinfo[128] = {0};
  char heraeye_cur_time[128] = {0};
	char kernel_version[16]={0};
  const char *strval = UTS_RELEASE;
	char heraeye_info[64]={0};
  strncpy(kernel_version,strval,8);

  sensor_driver_lasterrcode=errtype;
  sensor_driver_lasterrcode_id=sensorid;
  sensor_driver_lasterrcode_cmd_or_errcode=cmd_or_errcode;

  switch (sensor_driver_lasterrcode) {
  	case SENSOR_POWER_UP:
  		strcpy(heraeye_info,"SENSOR_POWER_UP");
  		break;
  	case SENSOR_HUB_GET_DATA:
  		strcpy(heraeye_info,"SENSOR_HUB_GET_DATA");
  		break;
    case SENSOR_DRV_INIT_ERROR:
  		strcpy(heraeye_info,"SENSOR_DRV_INIT_ERROR");
  		break;	
  		
  	default:
  		strcpy(heraeye_info,"SENSOR_ERR_default");
  		WARN(1, "sensor init  error %d \n",sensor_driver_lasterrcode);
  }

  heraeye_curtime_to_str(heraeye_cur_time);
  sprintf(heraeye_cur_chipinfo, "%s_%s_%s_%s",bootprof_get_ap_platform(),
  heraeye_get_project_str(),kernel_version,"sensor");

  pr_info("hela_boot_sensor %s %s %s %d %d %d %s 0x%x",
            TKPERF_PERF_SENSOR_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            sensor_driver_lasterrcode,
            sensor_driver_lasterrcode_id,
            sensor_driver_lasterrcode_cmd_or_errcode,
            heraeye_info,
            TKPERF_SENSOR_EVENTID);  
            
  heraeye_driver_log(TKPERF_DRIVER_SENSOR,"%s %s %s %d %d %d %s 0x%x",
            TKPERF_PERF_SENSOR_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            sensor_driver_lasterrcode,
            sensor_driver_lasterrcode_id,
            sensor_driver_lasterrcode_cmd_or_errcode,
            heraeye_info,
            TKPERF_SENSOR_EVENTID);  
   }
   
#endif

//[SYSD SYS] Begin modified by zhoujinggao for OLQ-15 on 2022-07-07

#ifdef CONFIG_HELAEYE_BSP_TP_ON
char mtk_tpchip_name[32]={"TP_CHIP_UNKNOW"};
int tp_driver_lasterrcode=TP_DRV_NORMAL;
int tp_driver_lasterrcode_cmd_or_errcode=0;

EXPORT_SYMBOL(tp_driver_lasterrcode);
EXPORT_SYMBOL(tp_driver_lasterrcode_cmd_or_errcode);
EXPORT_SYMBOL(mtk_tpchip_name);

static int tct_tp_get(char *str)
{
      int str_len;
      str_len=strlen(str);
      if(str_len<32&&str_len>1)
      strcpy(mtk_tpchip_name,str);

      printk("[hela_boot_tp] %s:Board use tp %s, mtk_tpchip_name=%s \n",
      __func__,str,mtk_tpchip_name);
      return 0;
}
__setup("driver.tpvendor=",tct_tp_get);

void heraeye_tp_fail(int errtype, char *tpname, int cmd_or_errcode){
	char heraeye_cur_chipinfo[128] = {0};
  char heraeye_cur_time[128] = {0};
	char kernel_version[16]={0};
  const char *strval = UTS_RELEASE;
	char heraeye_info[64]={0};
  strncpy(kernel_version,strval,8);

  tp_driver_lasterrcode=errtype;
  tp_driver_lasterrcode_cmd_or_errcode=cmd_or_errcode;

  switch (tp_driver_lasterrcode) {
  	case TP_ERR_INIT:
  		strcpy(heraeye_info,"TP_ERR_INIT");
  		break;
  	case TP_ERR_GETINFO:
  		strcpy(heraeye_info,"TP_ERR_GETINFO");
  		break;
    case TP_ERR_UPGRADE_FAIL:
  		strcpy(heraeye_info,"TP_ERR_UPGRADE_FAIL");
  		break;
  	case TKPERF_TP_ESD_TIGER:
  		strcpy(heraeye_info,"TKPERF_TP_ESD_TIGER");
  		break;
  	default:
  		strcpy(heraeye_info,"TP_ERR_INFO_UNKNOW");
  		WARN(1, "tp init  error %d \n",sensor_driver_lasterrcode);
  }

  heraeye_curtime_to_str(heraeye_cur_time);
  sprintf(heraeye_cur_chipinfo, "%s_%s_%s_%s",bootprof_get_ap_platform(),
  heraeye_get_project_str(),kernel_version,tpname);

  pr_info("hela_boot_tp %s %s %s %d %s %d %s 0x%x",
            TKPERF_PERF_TP_DRIVER_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            tp_driver_lasterrcode,
            tpname,
            tp_driver_lasterrcode_cmd_or_errcode,
            heraeye_info,
            TKPERF_TP_EVENTID);  
            
  heraeye_driver_log(TKPERF_DRIVER_TP,"%s %s %s %d %s %d %s 0x%x",
            TKPERF_PERF_TP_DRIVER_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            tp_driver_lasterrcode,
            tpname,
            tp_driver_lasterrcode_cmd_or_errcode,
            heraeye_info,
            TKPERF_TP_EVENTID);  
   }

#endif

#ifdef CONFIG_HELAEYE_BSP_TMO24H_TCP_RETRAN_ON


void heraeye_tcp24_driverlasttotoal_fail(int port5228_retran,int port5228_tran,int charger_err,
 int md_err, int lcd_err, int flash_err, int sensor_err )

{
	char heraeye_cur_chipinfo[128] = {0};
  char heraeye_cur_time[128] = {0};
	char kernel_version[16]={0};
  const char *strval = UTS_RELEASE;
  strncpy(kernel_version,strval,8);

  heraeye_curtime_to_str(heraeye_cur_time);
  sprintf(heraeye_cur_chipinfo, "%s_%s_%s",bootprof_get_ap_platform(),
  heraeye_get_project_str(),kernel_version);

  pr_info("hela_boot_tcp %s %s %s %d %d %d %d %d %d %d 0x%x",
            TKPERF_PERF_TMO24H_TCP_RETRAN_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            port5228_retran,
            port5228_tran,
            charger_err,
            md_err,
            lcd_err,
            flash_err,
            sensor_err,
            TKPERF_DRIVER_24GCM);
//  MODEL3-10529 need support remote control by jinggao.zhou 20230317
  heraeye_driver_log(TKPERF_DRIVER_24GCM,"%s %s %s %d %d %d %d %d %d %d 0x%x",
            TKPERF_PERF_TMO24H_TCP_RETRAN_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            port5228_retran,
            port5228_tran,
						charger_err,
            md_err,
            lcd_err,
            flash_err,
            sensor_err,
            TKPERF_24GCM_EVENTID);

//  OLQ-21 not need log by jinggao.zhou 0831
//  tkperf_log(GFP_KERNEL,"%s %s %s %d %d %d %d %d %d %d 0x%x 0x%x",
//            TKPERF_PERF_TMO24H_TCP_RETRAN_INFO,
//            heraeye_cur_time,
//            heraeye_cur_chipinfo,
//            port5228_retran,
//            port5228_tran,
//						charger_err,
//            md_err,
//            lcd_err,
//            flash_err,
//            sensor_err,
//            TKPERF_24GCM_EVENTID,
//            TKPERF_24GCM_EVENTID);


}

int hela24_notify_handler(void *unused)
{

		do {
    		wait_event_interruptible(hela24_notify_waiter,
  			(hela24_notify_flag == true));

  		__pm_stay_awake(hela24_notify_lock);
  		mutex_lock(&hela24_notify_mutex);

  		/*Notify */

      pr_info("hela_boot_tcp md_5228_count %d/%d tp=%d charger=%d md=%d lcd=%d emmc=%d audio=%d sensor=%d  \n",md_5228_count,
      gms_5228_count,
      tp_driver_lasterrcode,
      charger_driver_lasterrcode,
      md_ccci_lasterrcode,
      lcd_driver_lasterrcode,
      emmc_ufs_sd_lasterrcode,
      audio_driver_lasterrcode,
      sensor_driver_lasterrcode
      );

      if( md_5228_count >=100 )
      {
				heraeye_tcp24_driverlasttotoal_fail(md_5228_count,gms_5228_count,charger_driver_lasterrcode,
				md_ccci_lasterrcode,lcd_driver_lasterrcode,emmc_ufs_sd_lasterrcode,sensor_driver_lasterrcode);
				md_5228_count=0;
				gms_5228_count=0;// fix overrun
      }

  		hela24_notify_flag = false;
  		mutex_unlock(&hela24_notify_mutex);
  		__pm_relax(hela24_notify_lock);


		} while (!kthread_should_stop());

  	return 0;
}

static enum alarmtimer_restart hera24_notify_callback(struct alarm *alarm, ktime_t now)
{
  	struct rtc_time tm;
  	struct timespec64 tv = { 0 };
  	/* android time */
  	struct rtc_time tm_android;
  	struct timespec64 tv_android = { 0 };

  	ktime_get_real_ts64(&tv);
  	tv_android = tv;
  	rtc_time64_to_tm(tv.tv_sec, &tm);
  	tv_android.tv_sec -= sys_tz.tz_minuteswest * 60;
  	rtc_time64_to_tm(tv_android.tv_sec, &tm_android);
  	printk("hera24_notify_callback %d-%02d-%02d %02d:%02d:%02d.%u UTC;"
  		"android time %d-%02d-%02d %02d:%02d:%02d.%03d\n",
  		tm.tm_year + 1900, tm.tm_mon + 1,
  		tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec,
  		(unsigned int)(tv.tv_nsec / 1000), tm_android.tm_year + 1900,
  		tm_android.tm_mon + 1, tm_android.tm_mday, tm_android.tm_hour,
  		tm_android.tm_min, tm_android.tm_sec,
  		(unsigned int)(tv_android.tv_nsec / 1000));
  	hela24_notify_flag = true;
  	wake_up_interruptible(&hela24_notify_waiter);
  	alarm_forward_now(&hela24_notify_alarm, ns_to_ktime(60 * 60 * 24LL * NSEC_PER_SEC));
    return ALARMTIMER_RESTART;
}

void hela_24hmon_init(void)
{
    alarm_init(&hela24_notify_alarm, ALARM_BOOTTIME, hera24_notify_callback);
    alarm_start_relative(&hela24_notify_alarm, ns_to_ktime(60 * 60 * 24LL * NSEC_PER_SEC));

  	hela24_notify_lock = wakeup_source_register(NULL, "hela24_notify_lock wakelock");
  	hela24_notify_thread = kthread_run(hela24_notify_handler, 0,
  		"hela24_nt_thread");
  	if (IS_ERR(hela24_notify_thread))
  		pr_notice("Failed to create hela24_notify_thread\n");
  	else
  		pr_info("hela_boot_tcp Create hela24_notify_thread : done\n");
}
#endif

//[SYSD SYS] Begin modified by zhoujinggao for OLQ-17 on 2022-07-14
#ifdef CONFIG_HELAEYE_BSP_ABFOTA_ON

int AB_retry_count=0;
static int tct_AB_retry_count_get(char *str)
{
      kstrtoint(str, 0, &AB_retry_count);
      pr_info("[helaeye_fota] %s:Board use AB_retry_count %s,AB_retry_count=%d \n",
      __func__,str,AB_retry_count);
      return 0;
}
__setup("androidboot.AB_retry_count=",tct_AB_retry_count_get);

#endif


#endif

struct log_t {
	/* task cmdline for first 16 bytes
	 * and boot event for the rest
	 */
	char *comm_event;
	pid_t pid;
	u64 timestamp;
};

static struct log_t *bootprof[BUF_COUNT];
static unsigned long log_count;
static DEFINE_MUTEX(bootprof_lock);
static bool enabled;
static int bootprof_lk_t, bootprof_pl_t, bootprof_logo_t;
static u64 timestamp_on, timestamp_off;
static bool boot_finish;

#ifdef CONFIG_BOOTPROF_THRESHOLD_MS
#define BOOTPROF_THRESHOLD (CONFIG_BOOTPROF_THRESHOLD_MS*1000000)
#else
#define BOOTPROF_THRESHOLD 15000000
#endif

#ifdef MODULE
static unsigned long long start_time;

/* Data structures to store tracepoints information */
struct bf_tp {
	const char *name;
	void *func;
	struct tracepoint *tp;
	void *data;
	bool init;
};
#else /*Build-in*/
module_param_named(pl_t, bootprof_pl_t, int, 0644);
module_param_named(lk_t, bootprof_lk_t, int, 0644);
module_param_named(logo_t, bootprof_logo_t, int, 0644);
#endif

static long long msec_high(unsigned long long nsec)
{
	if ((long long)nsec < 0) {
		nsec = -nsec;
		do_div(nsec, 1000000);
		return -nsec;
	}
	do_div(nsec, 1000000);

	return nsec;
}

static unsigned long msec_low(unsigned long long nsec)
{
	if ((long long)nsec < 0)
		nsec = -nsec;

	return do_div(nsec, 1000000);
}

bool mt_boot_finish(void)
{
	return boot_finish;
}
EXPORT_SYMBOL_GPL(mt_boot_finish);

void bootprof_log_boot(char *str)
{
	unsigned long long ts;
	struct log_t *p = NULL;
	size_t n;

	if (!str) {
		pr_info("[BOOTPROF] Null buffer. Skip log.\n");
		return;
	}

	if (!enabled)
		return;
	n = strlen(str) + 1;

	ts = sched_clock();
	pr_info("BOOTPROF:%10lld.%06ld:%s\n", msec_high(ts), msec_low(ts), str);

#ifdef CONFIG_TCL_BOOT_DETECT
	strcpy(kdisplay_bootprof_msg8,kdisplay_bootprof_msg7);
	strcpy(kdisplay_bootprof_msg7,kdisplay_bootprof_msg6);
	strcpy(kdisplay_bootprof_msg6,kdisplay_bootprof_msg5);
	strcpy(kdisplay_bootprof_msg5,kdisplay_bootprof_msg4);
	strcpy(kdisplay_bootprof_msg4,kdisplay_bootprof_msg3);
	strcpy(kdisplay_bootprof_msg3,kdisplay_bootprof_msg2);
	strcpy(kdisplay_bootprof_msg2,kdisplay_bootprof_msg1);
	strcpy(kdisplay_bootprof_msg1,kdisplay_bootprof_msg);
	strcpy(kdisplay_bootprof_msg,str);
	//pr_info("hang_detect: bootprof_log_boot BOOTPROF: %s \n", kdisplay_bootprof_msg);

#endif

	mutex_lock(&bootprof_lock);
	if (log_count >= (LOGS_PER_BUF * BUF_COUNT)) {
		pr_info("[BOOTPROF] not enuough bootprof buffer\n");
		goto out;
	} else if (log_count && !(log_count % LOGS_PER_BUF)) {
		bootprof[log_count / LOGS_PER_BUF] =
			kcalloc(LOGS_PER_BUF, sizeof(struct log_t),
				GFP_ATOMIC | __GFP_NORETRY | __GFP_NOWARN);
	}
	if (!bootprof[log_count / LOGS_PER_BUF]) {
		pr_info("no memory for bootprof\n");
		goto out;
	}
	p = &bootprof[log_count / LOGS_PER_BUF][log_count % LOGS_PER_BUF];

	p->timestamp = ts;
	p->pid = current->pid;
	n += TASK_COMM_LEN;

	p->comm_event = kzalloc(n, GFP_ATOMIC | __GFP_NORETRY |
			  __GFP_NOWARN);
	if (!p->comm_event) {
		enabled = false;
		goto out;
	}

	memcpy(p->comm_event, current->comm, TASK_COMM_LEN);
	memcpy(p->comm_event + TASK_COMM_LEN, str, n - TASK_COMM_LEN);
	log_count++;
out:
	mutex_unlock(&bootprof_lock);
}
EXPORT_SYMBOL_GPL(bootprof_log_boot);

static void bootprof_bootloader(void)
{
	struct device_node *node;
	int err = 0;

	node = of_find_node_by_name(NULL, "bootprof");
	if (node) {
		err |= of_property_read_s32(node, "pl_t", &bootprof_pl_t);
		err |= of_property_read_s32(node, "lk_t", &bootprof_lk_t);
		err |= of_property_read_s32(node, "lk_logo_t",
						&bootprof_logo_t);

		pr_info("BOOTPROF: DT(Err:0x%x) pl_t=%d, lk_t=%d, lk_logo_t=%d\n",
			err, bootprof_pl_t, bootprof_lk_t, bootprof_logo_t);
	}
#ifdef CONFIG_TCL_BOOT_DETECT
if ((kernel_printk_tofb_ctrl & 0x10) == 0x10){
		printk("BOOTPROF: bootcomplete_fivemins_delay_report 0926\n");
		bootcomplete_fivemins_delay_report();
	}
#endif
}

void bootprof_initcall(initcall_t fn, unsigned long long ts)
{
	/* log more than threshold initcalls */
	unsigned long msec_rem;
	char msgbuf[MSG_SIZE];
	int len;

	if (ts > BOOTPROF_THRESHOLD) {
		msec_rem = do_div(ts, NSEC_PER_MSEC);
		len = scnprintf(msgbuf, sizeof(msgbuf),
			"initcall: %ps %5llu.%06lums",
			fn, ts, msec_rem);
		if (len < 0)
			pr_info("BOOTPROF: initcall - Invalid argument.\n");
		bootprof_log_boot(msgbuf);
	}
}

#ifndef MODULE
/*Build-in*/
void bootprof_probe(unsigned long long ts, struct device *dev,
			   struct device_driver *drv, unsigned long probe)
{
	/* log more than threshold probes*/
	unsigned long msec_rem;
	char msgbuf[MSG_SIZE];
	int pos, len;

	if (ts <= BOOTPROF_THRESHOLD)
		return;
	msec_rem = do_div(ts, NSEC_PER_MSEC);

	pos = scnprintf(msgbuf, sizeof(msgbuf), "probe: probe=%ps",
					(void *)probe);
	if (pos < 0)
		pos = 0;

	if (drv) {
		len = scnprintf(msgbuf + pos, sizeof(msgbuf) - pos,
				" drv=%s(%ps)", drv->name ? drv->name : "",
				(void *)drv);
		if (len >= 0)
			pos += len;
	}

	if (dev && dev->init_name) {
		len = scnprintf(msgbuf + pos, sizeof(msgbuf) - pos,
				" dev=%s(%ps)", dev->init_name, (void *)dev);
		if (len >= 0)
			pos += len;
	}

	scnprintf(msgbuf + pos, sizeof(msgbuf) - pos,
			" %5llu.%06lums", ts, msec_rem);
	bootprof_log_boot(msgbuf);
}

void bootprof_pdev_register(unsigned long long ts, struct platform_device *pdev)
{
	/* log more than threshold register*/
	unsigned long msec_rem;
	char msgbuf[MSG_SIZE];
	int len;

	if (ts <= BOOTPROF_THRESHOLD || !pdev)
		return;
	msec_rem = do_div(ts, NSEC_PER_MSEC);
	len = scnprintf(msgbuf, sizeof(msgbuf),
			"probe: pdev=%s(%ps) %5llu.%06lums",
			pdev->name, (void *)pdev, ts, msec_rem);
	if (len < 0)
		pr_info("BOOTPROF: pdev - Invalid argument.\n");

	bootprof_log_boot(msgbuf);
}

static void bootup_finish(void)
{
	initcall_debug = 0;
}
#endif /*MODULE END*/

#ifdef CONFIG_HELAEYE_BSP_BOOT_ON

struct tag_bootmode {
		u32 size;
		u32 tag;
		u32 bootmode;
		u32 boottype;
};

int hela_get_boot_type(void)
{
  	struct tag_bootmode *tags = NULL;
  	struct device_node *node = NULL;
  	unsigned long size = 0;
  	int ret = BOOTDEV_UFS;

  	node = of_find_node_by_path("/chosen");
  	if (!node)
  		node = of_find_node_by_path("/chosen@0");

  	if (node) {
  		tags = (struct tag_bootmode *)of_get_property(node,
  				"atag,boot", (int *)&size);
  	} else
  		pr_notice("[%s] of_chosen not found\n", __func__);
  
  	if (tags) {
  		ret = tags->boottype;
  		if ((ret > 2) || (ret < 0))
  			ret = BOOTDEV_SDMMC;
  	} else {
  		pr_notice("[%s] 'atag,boot' is not found\n", __func__);
  	}

  	return ret;
}


static int heraeye_getlastbsp_error(void)
{
	char heraeye_cur_chipinfo[128] = {0};
  char heraeye_cur_time[128] = {0};
  char kernel_version[16]={0};
  const char *strval = UTS_RELEASE;
  char heraeye_info[64]={0};
  strncpy(kernel_version,strval,8);

  heraeye_curtime_to_str(heraeye_cur_time);
  sprintf(heraeye_cur_chipinfo, "%s_%s_%s",heraeye_get_arch_str(),heraeye_get_project_str(),kernel_version);

  pr_info("[helaeye_boot]Platform:%s Ker_ver: %s modem_ver: %s chip:%s ufs/emmc:%d tp:[%d %d] lcd:[%d]\n",
  bootprof_get_ap_platform(),kernel_version,modem_build_ver,
  heraeye_cur_chipinfo,hela_get_boot_type(),
  tp_driver_lasterrcode,tp_driver_lasterrcode_cmd_or_errcode,
  lcd_driver_lasterrcode
  );

/*  debug boot error code */

//  md_ccci_lasterrcode=-CCCI_ERR_LOAD_IMG_SIGN_FAIL;

//  audio_driver_lasterrcode=9;

//	emmc_ufs_sd_lasterrcode=9;

//  charger_driver_lasterrcode=9;
//  charger_driver_lasterrcode_value=0x2;

//  sensor_driver_lasterrcode=9;
//  sensor_driver_lasterrcode_id=1;
// 	sensor_driver_lasterrcode_cmd_or_errcode=-1;

//  lcd_driver_lasterrcode=1;
//  lcd_driver_lasterrcode=LCD_DRV_INIT_ERROR;



#ifdef CONFIG_HELAEYE_BSP_MODEMCRASH_ON
// start get md ccci driver init error 
  if (md_ccci_lasterrcode<0) {

  switch (md_ccci_lasterrcode) {
  	case -CCCI_ERR_LOAD_IMG_SIGN_FAIL:
  		strcpy(heraeye_info,"CCCI_ERR_LOAD_IMG_SIGN_FAIL");
  		break;
  	case -CCCI_ERR_LOAD_IMG_CIPHER_FAIL:
  		strcpy(heraeye_info,"CCCI_ERR_LOAD_IMG_CIPHER_FAIL");
  		break;
  	case -CCCI_ERR_LOAD_IMG_FILE_OPEN:
  		strcpy(heraeye_info,"CCCI_ERR_LOAD_IMG_FILE_OPEN");
  		break;
  	case -CCCI_ERR_LOAD_IMG_MD_CHECK:
  		strcpy(heraeye_info,"CCCI_ERR_LOAD_IMG_MD_CHECK");
  		break;
  	default:
  		strcpy(heraeye_info,"CCCI_ERR_default");
  		WARN(1, "modem loader error %d \n",md_ccci_lasterrcode);
  }

  pr_info("helaeye_boot %s %s %s %d %s 0x%x",
            TKPERF_PERF_MODEM_DRIVER_CRASH_INFO,
            heraeye_cur_time,
            modem_build_ver,
            md_ccci_lasterrcode,
            heraeye_info,
            TKPERF_MD_EVENTID);
  heraeye_driver_log(TKPERF_DRIVER_MODEMCCCI,"%s %s %s %d %s 0x%x",
            TKPERF_PERF_MODEM_DRIVER_CRASH_INFO,
            heraeye_cur_time,
            modem_build_ver,
            md_ccci_lasterrcode,
            heraeye_info,
            TKPERF_MD_EVENTID);
  }


// end get modem driver init error
#endif

#ifdef CONFIG_HELAEYE_BSP_AUDIO_ON

  if (audio_driver_lasterrcode>0) {
  pr_info("hela_boot %s %s %s %d %d 0x%x",
            TKPERF_PERF_AUDIO_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            AUDIO_DRV_ALSA_INIT_ERROR,
            audio_driver_lasterrcode,
            TKPERF_AUDIO_EVENTID);
	mdelay(500);// add by jinggao for tkperf netlink socket bug 2022/10/09 OLQ-32
  heraeye_driver_log(TKPERF_DRIVER_AUDIO,"%s %s %s %d %d 0x%x",
            TKPERF_PERF_AUDIO_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            AUDIO_DRV_ALSA_INIT_ERROR,
            audio_driver_lasterrcode,
            TKPERF_AUDIO_EVENTID);
}

#endif

#ifdef CONFIG_HELAEYE_BSP_EMMC_UFS_SD_ON
  if (emmc_ufs_sd_lasterrcode>0) {
  pr_info("hela_boot %s %s %s %d %s 0x%x",
            TKPERF_PERF_EMMC_UFS_SD_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            emmc_ufs_sd_lasterrcode,
            hela_emmc_errinfo,
            TKPERF_EMMC_UFS_SD_EVENTID);
  mdelay(500);// add by jinggao for tkperf netlink socket bug 2022/10/09 OLQ-32
  heraeye_driver_log(TKPERF_DRIVER_EMMC,"%s %s %s %d %s 0x%x",
            TKPERF_PERF_EMMC_UFS_SD_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            emmc_ufs_sd_lasterrcode,
            hela_emmc_errinfo,
            TKPERF_EMMC_UFS_SD_EVENTID);
}
#endif

#ifdef CONFIG_HELAEYE_BSP_LCD_ON

  if (lcd_driver_lasterrcode>0) {

  sprintf(heraeye_cur_chipinfo, "%s_%s_%s_%s",bootprof_get_ap_platform(),
  heraeye_get_project_str(),kernel_version,mtkfb_hera_lcm_name);

  switch (lcd_driver_lasterrcode) {
  	case LCD_DRV_INIT_ERROR:
  		strcpy(heraeye_info,"LCD_DRV_INIT_ERROR");
  		break;
  	case LCD_DRV_PARM_ERROR:
  		strcpy(heraeye_info,"LCD_DRV_PARM_ERROR");
  		break;
  	case LCD_DRV_ESD_ERROR:
  		strcpy(heraeye_info,"LCD_DRV_ESD_ERROR");
  		break;
  	case LCD_DRV_RECOVERY_ERROR:
  		strcpy(heraeye_info,"LCD_DRV_RECOVERY_ERROR");
  		break;
  	default:
  		strcpy(heraeye_info,"LCD_ERR_default");
  		WARN(1, "lcd init  error %d \n",lcd_driver_lasterrcode);
  }

  pr_info("hela_boot_lcd %s %s %s %d %s 0x%x",
            TKPERF_PERF_LCD_INFO,
             heraeye_cur_time,
             heraeye_cur_chipinfo,
             lcd_driver_lasterrcode,
             heraeye_info,
             TKPERF_LCD_EVENTID);
  mdelay(500);// add by jinggao for tkperf netlink socket bug 2022/10/09 OLQ-32
  heraeye_driver_log(TKPERF_DRIVER_LCD,"%s %s %s %d %s 0x%x",
            TKPERF_PERF_LCD_INFO,
             heraeye_cur_time,
             heraeye_cur_chipinfo,
             lcd_driver_lasterrcode,
             heraeye_info,
             TKPERF_LCD_EVENTID);
}
#endif

#ifdef CONFIG_HELAEYE_BSP_CHARGER_ON

if (charger_driver_lasterrcode>0) {
    mdelay(500);// add by jinggao for tkperf netlink socket bug 2022/10/09 OLQ-32
		heraeye_bat_check_temp(charger_driver_lasterrcode,charger_driver_lasterrcode_value);
}
#endif

#ifdef CONFIG_HELAEYE_BSP_SENSOR_ON

if (sensor_driver_lasterrcode>0) {
	  mdelay(500);// add by jinggao for tkperf netlink socket bug 2022/10/09 OLQ-32
		heraeye_sensor_fail(sensor_driver_lasterrcode,sensor_driver_lasterrcode_id,
		sensor_driver_lasterrcode_cmd_or_errcode);
}

#endif

#ifdef CONFIG_HELAEYE_BSP_TP_ON

if (tp_driver_lasterrcode>0) {
	  mdelay(500);// add by jinggao for tkperf netlink socket bug 2022/10/09 OLQ-32
		heraeye_tp_fail(tp_driver_lasterrcode,mtk_tpchip_name,
		tp_driver_lasterrcode_cmd_or_errcode);
}
#endif


#ifdef CONFIG_HELAEYE_BSP_ABFOTA_ON

//  preloader will reduce_retry_count if ab fota boot fail
if ( AB_retry_count!=7 && AB_retry_count!=1 && AB_retry_count!=4)
{

	pr_info("hela_boot_fota %s %s %s %d 0x%x",
            TKPERF_PERF_ABFOTA_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            AB_retry_count,
            TKPERF_ABFOTA_EVENTID);
//  MODEL3-10529 need support remote control by jinggao.zhou FOTA & 24CCCI not need log 20230317 
	mdelay(500);
  heraeye_driver_log(TKPERF_DRIVER_ABFOTA,"%s %s %s %d 0x%x",
            TKPERF_PERF_ABFOTA_INFO,
            heraeye_cur_time,
            heraeye_cur_chipinfo,
            AB_retry_count,
            TKPERF_ABFOTA_EVENTID);

//  OLQ-22 not need log by jinggao.zhou 0831 
//  tkperf_log(GFP_KERNEL,"%s %s %s %d 0x%x 0x%x",
//            TKPERF_PERF_ABFOTA_INFO,
//            heraeye_cur_time,
//            heraeye_cur_chipinfo,
//            AB_retry_count,
//            TKPERF_ABFOTA_EVENTID,
//            TKPERF_ABFOTA_EVENTID);
}
#endif


	return 0;
}
#endif
#ifdef CONFIG_TCL_BOOT_DETECT

int logcat_to_kernel(void)
{
	static char const start_path[] = "/system/bin/logcat";
 	static char *envp[] = {
  		"PATH=/sbin:/vendor/bin:/system/bin",
  		NULL,
  	};
 	static char *argv[] = {
  		(char *)start_path,
  		"EacBootAnimation:S",
  		"-f", "/dev/RT_Monitor",
  		"-t", "50000",
  		NULL,
  	};
 	pr_err("hang_detect: calling %s %s %s %s %s %s",argv[0],argv[1],argv[2],argv[3],argv[4],argv[5]);
 	return call_usermodehelper(start_path, argv, envp, UMH_WAIT_PROC);
}

void bootcomplete_delay_report_task(struct timer_list *t)
{
  	bootcomplete_delay_notify_flag = true;
  	pr_info("bootcomplete_delay_report_task boot_finish=%d boot_detect_kerenl_display_status =%d get_boot_mode=%d \n",
  	boot_finish,boot_detect_kerenl_display_status,get_boot_mode());
  	if (!boot_finish && !boot_detect_kerenl_display_status && get_boot_mode() == 0){
			wake_up_interruptible(&bootcomplete_delay_notify_waiter);
			pr_info("bootcomplete_delay_report_task  not boot complete 5 min trigger \n");
  	}
}

extern void log_store_to_emmc(void);
int logcat_to_kernel(void);
void klog_bootprof_to_ui(void);

void print_ver_info(void)
{
#ifdef CONFIG_TCT_MP2_BLUESCREEN
  	DAL_Printf("   The diagnostic system found some issue . TCT 2022-11  SHIP MP=2 VERSION\n");
#endif
#if defined(TARGET_BUILD_USERDEBUG) 
  	DAL_Printf("   The diagnostic system found some issue . TCT 2022-11  USERDEBUG VERSION\n");
#endif
#if defined(TARGET_BUILD_MMITEST)
  	DAL_Printf("   The diagnostic system found some issue . TCT 2022-11  MMITEST VERSION\n");
#endif
#if defined(TARGET_BUILD_CERTIFICATION)
  	DAL_Printf("   The diagnostic system found some issue . TCT 2022-11  CERTIFICATION VERSION\n");
#endif
#if defined(TARGET_BUILD_USER)
  	DAL_Printf("   The diagnostic system found some issue . TCT 2022-11  USER VERSION\n");
#endif
}
void botodetect_curtime_to_str(char *cur_time)
{
      struct rtc_time tm;
        	/* android time */
      struct timeval tv = { 0 };
      struct timeval tv_android = { 0 };
    	struct rtc_time tm_android;
  
      if (cur_time == NULL)
          return;

      do_gettimeofday(&tv);
    	tv_android = tv;
    	rtc_time_to_tm(tv.tv_sec, &tm);
    	tv_android.tv_sec -= sys_tz.tz_minuteswest * 60;
    	rtc_time_to_tm(tv_android.tv_sec, &tm_android);
  
      sprintf(cur_time, "%d-%d-%d_%02d:%02d:%02d", tm_android.tm_year+1900, tm_android.tm_mon+1, tm_android.tm_mday,
              tm_android.tm_hour, tm_android.tm_min, tm_android.tm_sec);
      return;
  }

int bootcomplete_delay_notify_handler(void *unused)
{
    char boot_cur_time[128] = {0};
  	wait_event_interruptible(bootcomplete_delay_notify_waiter,
  			(bootcomplete_delay_notify_flag == true));
  	boot_detect_kerenl_display_status=1;
  	botodetect_curtime_to_str(&boot_cur_time[0]);
  	DAL_SetColor(0xFFFFFF,0x0000FF);
  	DAL_SetScreenColor(0x000000);
  	DAL_Printf("\n");
  	DAL_Printf("\n");
  	DAL_Printf("\n");
  	DAL_Printf("hang_detect:boot timeout 5mins current->comm=[%s] kernel_printk_tofb_ctrl=[0x%x] get_boot_mode=[%d] current_version=%d\n",
  	current->comm,kernel_printk_tofb_ctrl,get_boot_mode(),current_version);
  	//DAL_Printf("NORMAL=0,META=1,RECOVERY=2,SW=3,FACTORY=4,ADVMETA=5,ATE_FACTORY=6");
  	//DAL_Printf("ALARM=7,KERNEL_POWER_OFF_CHARGING=8,LOW_POWER_OFF_CHARGING=9,DONGLE=10\n");
  	print_ver_info();
  	DAL_Printf("The diagnostic system found that the system did not start in 5 minutes. TCT 2022-10 \n");
  	pr_info("bootcomplete_delay_notify_handler + \n");
  		/*Notify */
  	DAL_Printf("logcat_kernel2()=%d\n",logcat_to_kernel());
  	klog_bootprof_to_ui();  
  	klog_check_to_ui();
  	log_store_to_emmc();
  	DAL_Printf("\n");
  	DAL_Printf("Kernel Android log have been saved to the expdb partition.\n");
  	DAL_Printf("boot_cur_time:%s\n",boot_cur_time);
  	DAL_Printf("Please use teleweb to read back the expdb partition.\n");
  	DAL_Printf("\n BOOT Diagnostic System. TOOL BUG send to zhoujinggao@tcl.com \n");
    DAL_Printf("\n Please send Picture to System or Framework Team \n");
  	bootcomplete_delay_notify_flag = false;
  	return 0;
}
// Eliminate the problem of exception reported after startup, delaying 2 minutes
void bootcomplete_fivemins_delay_report(void)
{
	  unsigned long bootcomplete_delay_notify_interval;
	  bootcomplete_delay_notify_interval = HZ * 60 * 5 ;// test use 2 mins 
	  timer_setup(&bootcomplete_delay_notify_timer, bootcomplete_delay_report_task, TIMER_DEFERRABLE);
	  mod_timer(&bootcomplete_delay_notify_timer, jiffies + bootcomplete_delay_notify_interval);
	  bootcomplete_delay_notify_thread = kthread_run(bootcomplete_delay_notify_handler, 0,
  		"bootcomplete_delay_notify_thread");
	  if (IS_ERR(bootcomplete_delay_notify_thread))
			pr_notice("Failed to create bootcomplete_delay_notify_thread\n");
	  else
			pr_info("bootcomplete_boot_driver Create bootcomplete_delay_notify_thread : done\n");
}
#endif
static void mt_bootprof_switch(int on)
{
	mutex_lock(&bootprof_lock);
	if (enabled ^ on) {
		unsigned long long ts = sched_clock();

		pr_info("BOOTPROF:%10lld.%06ld: %s\n",
		       msec_high(ts), msec_low(ts), on ? "ON" : "OFF");

		if (on) {
			enabled = 1;
			timestamp_on = ts;
		} else {
			enabled = 0;
			timestamp_off = ts;
			if (!boot_finish)
				boot_finish = true;
#ifndef MODULE
			bootup_finish();
#endif
//[SYSD SYS] add tp hela by jinggao.zhou@tcl.com for Online Quality OLQ-10

//#if MP_BRANCH_VALUE == 2
//    pr_info("BOOTPROF: Please note that this is the shipping version. Please note whether the HeLa function is turned off\n");
//#endif


#ifdef CONFIG_HELAEYE_BSP_BOOT_ON
    // now will get all drivers last error to HELAEYE log 
    pr_info("BOOTPROF: START HELAEYE BOOT CHECK LOG AND 24H Moniter Thread \n");
#ifdef CONFIG_TKPERF
    tkperf_log(GFP_KERNEL, "BOOTPROF: START HELAEYE LOG Delay 2mins AND 24H Moniter Thread fix First loss data\n");
#endif

// Eliminate the problem of exception reported by Tguard after startup, delaying 2 minutes
// [SYSD SYS] add by jinggao.zhou@tcl.com for SOCAOSP13-9861
    hela_twomins_delay_report();

//[SYSD SYS] add tp hela by jinggao.zhou@tcl.com for Online Quality OLQ-16
#ifdef CONFIG_HELAEYE_BSP_TMO24H_TCP_RETRAN_ON
    hela_24hmon_init();
#endif

#endif
		}
	}
	mutex_unlock(&bootprof_lock);
}

static ssize_t
mt_bootprof_write(struct file *filp, const char *ubuf, size_t cnt, loff_t *data)
{
	char buf[BOOT_STR_SIZE];
	size_t copy_size = cnt;

	if (cnt >= sizeof(buf))
		copy_size = BOOT_STR_SIZE - 1;

	if (copy_from_user(&buf, ubuf, copy_size))
		return -EFAULT;

	if (cnt == 1 && buf[0] == '1') {
		mt_bootprof_switch(1);
		return 1;
	} else if (cnt == 1 && buf[0] == '0') {
		mt_bootprof_switch(0);
		return 1;
	}

	buf[copy_size] = 0;
	bootprof_log_boot(buf);
#ifdef CONFIG_TCL_BOOT_DETECT
	strcpy(kdisplay_bootprof_msg8,kdisplay_bootprof_msg7);
	strcpy(kdisplay_bootprof_msg7,kdisplay_bootprof_msg6);
	strcpy(kdisplay_bootprof_msg6,kdisplay_bootprof_msg5);
	strcpy(kdisplay_bootprof_msg5,kdisplay_bootprof_msg4);
	strcpy(kdisplay_bootprof_msg4,kdisplay_bootprof_msg3);
	strcpy(kdisplay_bootprof_msg3,kdisplay_bootprof_msg2);
	strcpy(kdisplay_bootprof_msg2,kdisplay_bootprof_msg1);
	strcpy(kdisplay_bootprof_msg1,kdisplay_bootprof_msg);
	strcpy(kdisplay_bootprof_msg,buf);
	//pr_info("hang_detect: mt_bootprof_write BOOTPROF: %s \n", kdisplay_bootprof_msg);
#endif

	return cnt;
}

static int mt_bootprof_show(struct seq_file *m, void *v)
{
	int i;
	struct log_t *p;

	if (!m) {
		pr_info("seq_file is Null.\n");
		return 0;
	}
	seq_puts(m, "----------------------------------------\n");
	seq_printf(m, "%d	    BOOT PROF (unit:msec)\n", enabled);
	seq_puts(m, "----------------------------------------\n");

	if (bootprof_pl_t > 0 && bootprof_lk_t > 0) {
		seq_printf(m, "%10d        : %s\n", bootprof_pl_t, "preloader");
		if (bootprof_logo_t > 0) {
			seq_printf(m, "%10d        : %s (%s: %d)\n",
			bootprof_lk_t, "lk", "Start->Show logo",
			bootprof_logo_t);
		} else {
			seq_printf(m, "%10d        : %s\n",
			bootprof_lk_t, "lk");
		}
		seq_puts(m, "----------------------------------------\n");
	}

	seq_printf(m, "%10lld.%06ld : ON (Threshold:%5lldms)\n",
		   msec_high(timestamp_on), msec_low(timestamp_on),
		   msec_high(BOOTPROF_THRESHOLD));

	for (i = 0; i < log_count; i++) {
		p = &bootprof[i / LOGS_PER_BUF][i % LOGS_PER_BUF];
		if (!p->comm_event)
			continue;

		seq_printf(m, "%10llu.%06lu :%5d-%-16s: %s\n",
			   msec_high(p->timestamp),
			   msec_low(p->timestamp),
			   p->pid, p->comm_event,
			   p->comm_event + TASK_COMM_LEN);
	}

	seq_printf(m, "%10lld.%06ld : OFF\n",
		   msec_high(timestamp_off), msec_low(timestamp_off));
	seq_puts(m, "----------------------------------------\n");
	return 0;
}

/*** Seq operation of mtprof ****/
static int mt_bootprof_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_bootprof_show, inode->i_private);
}

static const struct file_operations mt_bootprof_fops = {
	.open = mt_bootprof_open,
	.write = mt_bootprof_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#ifdef MODULE

/*  initcalls tracepoint cb if initcall_debug=1 */
static __init_or_module void
tp_initcall_start_cb(void *data, initcall_t fn)
{
	unsigned long long *start_ts  = (unsigned long long *)data;
	*start_ts  = sched_clock();
}

static __init_or_module void
tp_initcall_finish_cb(void *data, initcall_t fn, int ret)
{
	unsigned long long *start_ts = (unsigned long long *)data;
	unsigned long long end_ts, duration;

	/*For bootprof module without initcall_start*/
	if (*start_ts == 0) {
		bootprof_log_boot("Kernel_init_done");
		return;
	}
	end_ts = sched_clock();
	duration = end_ts - *start_ts;
	bootprof_initcall(fn, duration);
	*start_ts = 0;
}

static struct bf_tp tp_table[] = {
	{.name = "initcall_start", .func = tp_initcall_start_cb,
	.data = &start_time},
	{.name = "initcall_finish", .func = tp_initcall_finish_cb,
	.data = &start_time},
};

/* Find the struct tracepoint* associated with a given tracepoint */
/* name. */
static void tp_lookup(struct tracepoint *tp, void *ignore)
{
	int i;

	if (!tp || !tp->name)
		return;

	for (i = 0; i < sizeof(tp_table) / sizeof(struct bf_tp); i++) {
		if (strcmp(tp_table[i].name, tp->name) == 0)
			tp_table[i].tp = tp;
	}
}

/* claen up initcalls tracepoints */
static void tp_cleanup(void)
{
	int i;

	for (i = 0; i < sizeof(tp_table) / sizeof(struct bf_tp); i++) {
		if (tp_table[i].init) {
			tracepoint_probe_unregister(tp_table[i].tp,
				tp_table[i].func, tp_table[i].data);
			tp_table[i].init = false;
		}
	}
}

/* Register initcalls tracepoints */
static void tp_init(void)
{
	int i;
	/* Install the tracepoints */
	for_each_kernel_tracepoint(tp_lookup, NULL);
	for (i = 0; i < sizeof(tp_table) / sizeof(struct bf_tp); i++) {
		if (!tp_table[i].tp) {
			pr_info("[BOOTPROF]TP: %s not found\n",
					tp_table[i].name);
			/* Unload previously loaded */
			tp_cleanup();
			return;
		}
		tracepoint_probe_register(tp_table[i].tp, tp_table[i].func,
						tp_table[i].data);
		tp_table[i].init = true;
	}
}

static int __init bootprof_init(void)
{
	struct proc_dir_entry *pe;

	memset(bootprof, 0, sizeof(struct log_t *) * BUF_COUNT);
	bootprof[0] = kcalloc(LOGS_PER_BUF, sizeof(struct log_t),
			GFP_ATOMIC | __GFP_NORETRY | __GFP_NOWARN);
	if (!bootprof[0])
		goto fail;

	pe = proc_create("bootprof", 0664, NULL, &mt_bootprof_fops);
	if (!pe)
		return -ENOMEM;

	bootprof_bootloader();
	tp_init();
	mt_bootprof_switch(1);
fail:
	return 0;
}

static void __exit bootprof_exit(void)
{
	struct log_t *p = NULL;
	int i;

	enabled = 0;
	tp_cleanup();

	if (log_count > 0) {
		mutex_lock(&bootprof_lock);
		for (i = 0; i < log_count; i++) {
			p = &bootprof[i / LOGS_PER_BUF][i % LOGS_PER_BUF];
			kfree(p->comm_event);
		}

		for (i = 0; i < ((log_count / LOGS_PER_BUF) + 1); i++)
			kfree(bootprof[i]);

		mutex_unlock(&bootprof_lock);
	}
	remove_proc_entry("bootprof", NULL);
	pr_info("bootprof module exit.\n");
}
module_init(bootprof_init);
module_exit(bootprof_exit);

MODULE_DESCRIPTION("MEDIATEK BOOT TIME PROFILING");
MODULE_LICENSE("GPL v2");
#else /*Build-in*/
static int __init init_boot_prof(void)
{
	struct proc_dir_entry *pe;

	pe = proc_create("bootprof", 0664, NULL, &mt_bootprof_fops);
	if (!pe)
		return -ENOMEM;
	return 0;
}

static int __init init_bootprof_buf(void)
{
	memset(bootprof, 0, sizeof(struct log_t *) * BUF_COUNT);
	bootprof[0] = kcalloc(LOGS_PER_BUF, sizeof(struct log_t),
			      GFP_ATOMIC | __GFP_NORETRY | __GFP_NOWARN);
	if (!bootprof[0])
		goto fail;

	bootprof_bootloader();
	mt_bootprof_switch(1);
fail:
	return 0;
}

early_initcall(init_bootprof_buf);
device_initcall(init_boot_prof);
#endif /*MODULE END*/
