// SPDX-License-Identifier: GPL-2.0

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reboot.h>

#include "mtk_charger.h"
#if IS_ENABLED(CONFIG_TCT_CHARGER)
#include "mtk_battery.h"
#endif


#ifdef CONFIG_HELAEYE_BSP_CHARGER_ON
#include <tcl/tkperf.h>
#include <generated/utsrelease.h>
#endif

#if IS_ENABLED(CONFIG_TCT_CHARGER)

#if IS_ENABLED(CONFIG_TCT_CHARGER_GCS)

#define NONSTANDARD_CHARGER_NONE 0x0
#define NONSTANDARD_CHARGER_NONSTAND 0x1
#define NONSTANDARD_CHARGER_SLOW 0x2
#define NONSTANDARD_CHARGER_INVALIED 0xFF
#endif

#endif

/* Begin modified by dapeng.qiao for task 11038299 on 2021-05-1 */
#ifdef TCT_BMS_SW_SUPPORT
extern signed int g_chr_vol;
extern signed int g_chr_type;
#endif
/* End modified by dapeng.qiao for task 11038299 on 2021-05-1 */

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

/* Begin added by jin.wang for task 11700191 on 2022-1-17 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
#define MINI_SUSPEND_CHG_SOC	(62)
#define MINI_RESUME_CHG_SOC	(45)
static int mini_soc_limited = 0;
static int soc_limited = 1;
#endif
/* End added by jin.wang */

/* Begin added by bitao.xiong for AOSP13TMO-4319 on 2022-08-03 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
static struct mtk_battery *gm = NULL;
#endif
/* End added by bitao.xiong for AOSP13TMO-4319 on 2022-08-03 */

/* Begin added by bitao.xiong for ENCORECKT-2669 on 2022-08-31 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
static bool is_mmitest = false;
static bool is_inproductionflag = false;
static int __init inproductionflag_setup(char *str)
{
	if (!strncmp(str, "true", 4))
		is_inproductionflag = true;
	else
		is_inproductionflag = false;
	return 0;
}
__setup("androidboot.inproductionflag=", inproductionflag_setup);

static int __init mmitest_setup(char *str)
{
	if (!strncmp(str, "true", 4))
		is_mmitest = true;
	else
		is_mmitest = false;
	return 0;
}
__setup("androidboot.mmitest=", mmitest_setup);
#endif
/* End added by bitao.xiong for ENCORECKT-2669 on 2022-08-31 */

int chr_get_debug_level(void)
{
	struct power_supply *psy;
	static struct mtk_charger *info;
	int ret;

	if (info == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
		if (psy == NULL)
			ret = CHRLOG_DEBUG_LEVEL;
		else {
			info =
			(struct mtk_charger *)power_supply_get_drvdata(psy);
			if (info == NULL)
				ret = CHRLOG_DEBUG_LEVEL;
			else
				ret = info->log_level;
		}
	} else
		ret = info->log_level;

	return ret;
}

void _wake_up_charger(struct mtk_charger *info)
{
	unsigned long flags;

	if (info == NULL)
		return;

	spin_lock_irqsave(&info->slock, flags);
	if (!info->charger_wakelock->active)
		__pm_stay_awake(info->charger_wakelock);
	spin_unlock_irqrestore(&info->slock, flags);
	info->charger_thread_timeout = true;
	wake_up(&info->wait_que);
}

bool is_disable_charger(struct mtk_charger *info)
{
	if (info == NULL)
		return true;

	if (info->disable_charger == true || IS_ENABLED(CONFIG_POWER_EXT))
		return true;
	else
		return false;
}

int _mtk_enable_charging(struct mtk_charger *info,
	bool en)
{
	chr_debug("%s en:%d\n", __func__, en);
	if (info->algo.enable_charging != NULL)
		return info->algo.enable_charging(info, en);
	return false;
}

int mtk_charger_notifier(struct mtk_charger *info, int event)
{
	return srcu_notifier_call_chain(&info->evt_nh, event, NULL);
}

/* Begin added by hailong.chen for task 9785237 on 2020-10-10 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
#define CHARGER_PROP_READ(para, prop_str, default_val)\
do {\
	if (of_property_read_u32(np, prop_str, &val) >= 0)\
		para = val;\
	else {\
		chr_err("use default "#default_val":%d\n", default_val);\
		para = default_val;\
	}\
} while (0)

#define CHARGER_PROP_READ_SUFFIX(para, prop_str, suffix)\
do {\
	snprintf(prop_name, sizeof(prop_name), "%s_%s", prop_str, suffix);\
	if (of_property_read_u32(np, prop_name, &val) >= 0) {\
		para = val;\
	} else {\
		chr_err("prop: %s not found,use defalut parameter\n", prop_name);\
	}\
} while (0)

static void read_charger_para(struct mtk_charger *info,
							  struct device_node *np)
{
	u32 val;
	CHARGER_PROP_READ(info->data.jeita_temp_above_t4_current,
					  "jeita_temp_above_t4_current",
					  JEITA_TEMP_ABOVE_T4_CURRENT);
	CHARGER_PROP_READ(info->data.jeita_temp_t3_to_t4_current,
					  "jeita_temp_t3_to_t4_current",
					  JEITA_TEMP_T3_TO_T4_CURRENT);
	CHARGER_PROP_READ(info->data.jeita_temp_t2_to_t3_current,
					  "jeita_temp_t2_to_t3_current",
					  JEITA_TEMP_T2_TO_T3_CURRENT);
	CHARGER_PROP_READ(info->data.jeita_temp_t1_to_t2_current,
					  "jeita_temp_t1_to_t2_current",
					  JEITA_TEMP_T1_TO_T2_CURRENT);
	CHARGER_PROP_READ(info->data.jeita_temp_t0_to_t1_current,
					  "jeita_temp_t0_to_t1_current",
					  JEITA_TEMP_T0_TO_T1_CURRENT);
	CHARGER_PROP_READ(info->data.jeita_temp_below_t0_current,
					  "jeita_temp_below_t0_current",
					  JEITA_TEMP_BELOW_T0_CURRENT);

	/*---------------------slave charger config                               */
	CHARGER_PROP_READ(info->data.slave_jeita_temp_above_t4_current,
					  "slave_jeita_temp_above_t4_current",
					  SLAVE_JEITA_TEMP_ABOVE_T4_CURRENT);
	CHARGER_PROP_READ(info->data.slave_jeita_temp_t3_to_t4_current,
					  "slave_jeita_temp_t3_to_t4_current",
					  SLAVE_JEITA_TEMP_T3_TO_T4_CURRENT);
	CHARGER_PROP_READ(info->data.slave_jeita_temp_t2_to_t3_current,
					  "slave_jeita_temp_t2_to_t3_current",
					  SLAVE_JEITA_TEMP_T2_TO_T3_CURRENT);
	CHARGER_PROP_READ(info->data.slave_jeita_temp_t1_to_t2_current,
					  "slave_jeita_temp_t1_to_t2_current",
					  SLAVE_JEITA_TEMP_T1_TO_T2_CURRENT);
	CHARGER_PROP_READ(info->data.slave_jeita_temp_t0_to_t1_current,
					  "slave_jeita_temp_t0_to_t1_current",
					  SLAVE_JEITA_TEMP_T0_TO_T1_CURRENT);
	CHARGER_PROP_READ(info->data.slave_jeita_temp_below_t0_current,
					  "slave_jeita_temp_below_t0_current",
					  SLAVE_JEITA_TEMP_BELOW_T0_CURRENT);

	info->enable_step_chg = of_property_read_bool(np, "enable_step_chg");
	CHARGER_PROP_READ(info->data.step_chg_vbat,
					  "step_chg_vbat",
					  STEP_CHG_VBAT);
	CHARGER_PROP_READ(info->data.step_chg_vbat_hysteresis,
					  "step_chg_vbat_hysteresis",
					  STEP_CHG_VBAT_HYSTERESIS);
	CHARGER_PROP_READ(info->data.setp_chg_current,
					  "setp_chg_current",
					  STEP_CHG_CURRENT);
	CHARGER_PROP_READ(info->data.slave_setp_chg_current,
					  "slave_setp_chg_current",
					  SLAVE_STEP_CHG_CURRENT);
}

static void override_charger_para(struct mtk_charger *info,
								  struct device_node *np)
{
	char prop_name[128];
	u32 val;

	if (info->chg1_dev &&
		info->chg1_dev->props.alias_name) {
		CHARGER_PROP_READ_SUFFIX(info->data.max_charger_voltage,
								 "max_charger_voltage",
								 info->chg1_dev->props.alias_name);
		info->data.max_charger_voltage_setting = info->data.max_charger_voltage;
	} else {
		chr_err("not found charger,use default charger parameter\n");
	}
}
#endif
/* End added by hailong.chen for task 9785237 on 2020-10-10 */

static void mtk_charger_parse_dt(struct mtk_charger *info,
				struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 val = 0;
	struct device_node *boot_node = NULL;
	struct tag_bootmode *tag = NULL;

	boot_node = of_parse_phandle(dev->of_node, "bootmode", 0);
	if (!boot_node)
		chr_err("%s: failed to get boot mode phandle\n", __func__);
	else {
		tag = (struct tag_bootmode *)of_get_property(boot_node,
							"atag,boot", NULL);
		if (!tag)
			chr_err("%s: failed to get atag,boot\n", __func__);
		else {
			chr_err("%s: size:0x%x tag:0x%x bootmode:0x%x boottype:0x%x\n",
				__func__, tag->size, tag->tag,
				tag->bootmode, tag->boottype);
			info->bootmode = tag->bootmode;
			info->boottype = tag->boottype;
		}
	}

	if (of_property_read_string(np, "algorithm_name",
		&info->algorithm_name) < 0) {
		chr_err("%s: no algorithm_name name\n", __func__);
		info->algorithm_name = "Basic";
	}

	if (strcmp(info->algorithm_name, "Basic") == 0) {
		chr_err("found Basic\n");
		mtk_basic_charger_init(info);
	} else if (strcmp(info->algorithm_name, "Pulse") == 0) {
		chr_err("found Pulse\n");
		mtk_pulse_charger_init(info);
	}

/* Begin added by jin.wang for task 11466469 on 2021-9-3 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	info->enable_hv_charging =
			of_property_read_bool(np, "enable_hv_charging");
#endif
/* End added by jin.wang for task 11466469 on 2021-9-3 */

	info->disable_charger = of_property_read_bool(np, "disable_charger");
	info->enable_sw_safety_timer =
			of_property_read_bool(np, "enable_sw_safety_timer");
	info->sw_safety_timer_setting = info->enable_sw_safety_timer;

	/* common */

	if (of_property_read_u32(np, "charger_configuration", &val) >= 0)
		info->config = val;
	else {
		chr_err("use default charger_configuration:%d\n",
			SINGLE_CHARGER);
		info->config = SINGLE_CHARGER;
	}

	if (of_property_read_u32(np, "battery_cv", &val) >= 0)
		info->data.battery_cv = val;
	else {
		chr_err("use default BATTERY_CV:%d\n", BATTERY_CV);
		info->data.battery_cv = BATTERY_CV;
	}

	if (of_property_read_u32(np, "max_charger_voltage", &val) >= 0)
		info->data.max_charger_voltage = val;
	else {
		chr_err("use default V_CHARGER_MAX:%d\n", V_CHARGER_MAX);
		info->data.max_charger_voltage = V_CHARGER_MAX;
	}
	info->data.max_charger_voltage_setting = info->data.max_charger_voltage;

	if (of_property_read_u32(np, "min_charger_voltage", &val) >= 0)
		info->data.min_charger_voltage = val;
	else {
		chr_err("use default V_CHARGER_MIN:%d\n", V_CHARGER_MIN);
		info->data.min_charger_voltage = V_CHARGER_MIN;
	}

	/* sw jeita */
	info->enable_sw_jeita = of_property_read_bool(np, "enable_sw_jeita");
/* Begin added by dapeng.qiao for task 11024165 on 2021-04-13 */
#if IS_ENABLED(TARGET_BUILD_MMITEST)
	info->enable_sw_jeita = false;
	info->sw_jeita.sm = TEMP_T2_TO_T3;
	chr_err("disable for Mini SW enable_sw_jeita=%d\n", info->enable_sw_jeita);
#endif
/* End added by dapeng.qiao for task 11024165 on 2021-04-13 */
	if (of_property_read_u32(np, "jeita_temp_above_t4_cv", &val) >= 0)
		info->data.jeita_temp_above_t4_cv = val;
	else {
		chr_err("use default JEITA_TEMP_ABOVE_T4_CV:%d\n",
			JEITA_TEMP_ABOVE_T4_CV);
		info->data.jeita_temp_above_t4_cv = JEITA_TEMP_ABOVE_T4_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t3_to_t4_cv", &val) >= 0)
		info->data.jeita_temp_t3_to_t4_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T3_TO_T4_CV:%d\n",
			JEITA_TEMP_T3_TO_T4_CV);
		info->data.jeita_temp_t3_to_t4_cv = JEITA_TEMP_T3_TO_T4_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t2_to_t3_cv", &val) >= 0)
		info->data.jeita_temp_t2_to_t3_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T2_TO_T3_CV:%d\n",
			JEITA_TEMP_T2_TO_T3_CV);
		info->data.jeita_temp_t2_to_t3_cv = JEITA_TEMP_T2_TO_T3_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t1_to_t2_cv", &val) >= 0)
		info->data.jeita_temp_t1_to_t2_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T1_TO_T2_CV:%d\n",
			JEITA_TEMP_T1_TO_T2_CV);
		info->data.jeita_temp_t1_to_t2_cv = JEITA_TEMP_T1_TO_T2_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t0_to_t1_cv", &val) >= 0)
		info->data.jeita_temp_t0_to_t1_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T0_TO_T1_CV:%d\n",
			JEITA_TEMP_T0_TO_T1_CV);
		info->data.jeita_temp_t0_to_t1_cv = JEITA_TEMP_T0_TO_T1_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_below_t0_cv", &val) >= 0)
		info->data.jeita_temp_below_t0_cv = val;
	else {
		chr_err("use default JEITA_TEMP_BELOW_T0_CV:%d\n",
			JEITA_TEMP_BELOW_T0_CV);
		info->data.jeita_temp_below_t0_cv = JEITA_TEMP_BELOW_T0_CV;
	}

	if (of_property_read_u32(np, "temp_t4_thres", &val) >= 0)
		info->data.temp_t4_thres = val;
	else {
		chr_err("use default TEMP_T4_THRES:%d\n",
			TEMP_T4_THRES);
		info->data.temp_t4_thres = TEMP_T4_THRES;
	}

	if (of_property_read_u32(np, "temp_t4_thres_minus_x_degree", &val) >= 0)
		info->data.temp_t4_thres_minus_x_degree = val;
	else {
		chr_err("use default TEMP_T4_THRES_MINUS_X_DEGREE:%d\n",
			TEMP_T4_THRES_MINUS_X_DEGREE);
		info->data.temp_t4_thres_minus_x_degree =
					TEMP_T4_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t3_thres", &val) >= 0)
		info->data.temp_t3_thres = val;
	else {
		chr_err("use default TEMP_T3_THRES:%d\n",
			TEMP_T3_THRES);
		info->data.temp_t3_thres = TEMP_T3_THRES;
	}

	if (of_property_read_u32(np, "temp_t3_thres_minus_x_degree", &val) >= 0)
		info->data.temp_t3_thres_minus_x_degree = val;
	else {
		chr_err("use default TEMP_T3_THRES_MINUS_X_DEGREE:%d\n",
			TEMP_T3_THRES_MINUS_X_DEGREE);
		info->data.temp_t3_thres_minus_x_degree =
					TEMP_T3_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t2_thres", &val) >= 0)
		info->data.temp_t2_thres = val;
	else {
		chr_err("use default TEMP_T2_THRES:%d\n",
			TEMP_T2_THRES);
		info->data.temp_t2_thres = TEMP_T2_THRES;
	}

	if (of_property_read_u32(np, "temp_t2_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t2_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T2_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T2_THRES_PLUS_X_DEGREE);
		info->data.temp_t2_thres_plus_x_degree =
					TEMP_T2_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t1_thres", &val) >= 0)
		info->data.temp_t1_thres = val;
	else {
		chr_err("use default TEMP_T1_THRES:%d\n",
			TEMP_T1_THRES);
		info->data.temp_t1_thres = TEMP_T1_THRES;
	}

	if (of_property_read_u32(np, "temp_t1_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t1_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T1_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T1_THRES_PLUS_X_DEGREE);
		info->data.temp_t1_thres_plus_x_degree =
					TEMP_T1_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t0_thres", &val) >= 0)
		info->data.temp_t0_thres = val;
	else {
		chr_err("use default TEMP_T0_THRES:%d\n",
			TEMP_T0_THRES);
		info->data.temp_t0_thres = TEMP_T0_THRES;
	}

	if (of_property_read_u32(np, "temp_t0_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t0_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T0_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T0_THRES_PLUS_X_DEGREE);
		info->data.temp_t0_thres_plus_x_degree =
					TEMP_T0_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_neg_10_thres", &val) >= 0)
		info->data.temp_neg_10_thres = val;
	else {
		chr_err("use default TEMP_NEG_10_THRES:%d\n",
			TEMP_NEG_10_THRES);
		info->data.temp_neg_10_thres = TEMP_NEG_10_THRES;
	}

	/* battery temperature protection */
	info->thermal.sm = BAT_TEMP_NORMAL;
	info->thermal.enable_min_charge_temp =
		of_property_read_bool(np, "enable_min_charge_temp");

	if (of_property_read_u32(np, "min_charge_temp", &val) >= 0)
		info->thermal.min_charge_temp = val;
	else {
		chr_err("use default MIN_CHARGE_TEMP:%d\n",
			MIN_CHARGE_TEMP);
		info->thermal.min_charge_temp = MIN_CHARGE_TEMP;
	}

	if (of_property_read_u32(np, "min_charge_temp_plus_x_degree", &val)
		>= 0) {
		info->thermal.min_charge_temp_plus_x_degree = val;
	} else {
		chr_err("use default MIN_CHARGE_TEMP_PLUS_X_DEGREE:%d\n",
			MIN_CHARGE_TEMP_PLUS_X_DEGREE);
		info->thermal.min_charge_temp_plus_x_degree =
					MIN_CHARGE_TEMP_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "max_charge_temp", &val) >= 0)
		info->thermal.max_charge_temp = val;
	else {
		chr_err("use default MAX_CHARGE_TEMP:%d\n",
			MAX_CHARGE_TEMP);
		info->thermal.max_charge_temp = MAX_CHARGE_TEMP;
	}

	if (of_property_read_u32(np, "max_charge_temp_minus_x_degree", &val)
		>= 0) {
		info->thermal.max_charge_temp_minus_x_degree = val;
	} else {
		chr_err("use default MAX_CHARGE_TEMP_MINUS_X_DEGREE:%d\n",
			MAX_CHARGE_TEMP_MINUS_X_DEGREE);
		info->thermal.max_charge_temp_minus_x_degree =
					MAX_CHARGE_TEMP_MINUS_X_DEGREE;
	}

	/* charging current */
	if (of_property_read_u32(np, "usb_charger_current", &val) >= 0) {
		info->data.usb_charger_current = val;
	} else {
		chr_err("use default USB_CHARGER_CURRENT:%d\n",
			USB_CHARGER_CURRENT);
		info->data.usb_charger_current = USB_CHARGER_CURRENT;
	}

	if (of_property_read_u32(np, "ac_charger_current", &val) >= 0) {
		info->data.ac_charger_current = val;
	} else {
		chr_err("use default AC_CHARGER_CURRENT:%d\n",
			AC_CHARGER_CURRENT);
		info->data.ac_charger_current = AC_CHARGER_CURRENT;
	}

	if (of_property_read_u32(np, "ac_charger_input_current", &val) >= 0)
		info->data.ac_charger_input_current = val;
	else {
		chr_err("use default AC_CHARGER_INPUT_CURRENT:%d\n",
			AC_CHARGER_INPUT_CURRENT);
		info->data.ac_charger_input_current = AC_CHARGER_INPUT_CURRENT;
	}

/* [BSP]Begin added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
#if IS_ENABLED(CONFIG_TCT_CHARGER) && IS_ENABLED(CONFIG_TCT_CHARGER_GCS)
	if (of_property_read_u32(np, "weak_ac_charger_input_current", &val) >= 0)
		info->data.weak_ac_charger_input_current = val;
	else {
		chr_err("use default WEAK_AC_CHARGER_INPUT_CURRENT:%d\n",
			WEAK_AC_CHARGER_INPUT_CURRENT);
		info->data.weak_ac_charger_input_current = WEAK_AC_CHARGER_INPUT_CURRENT;
	}
#endif
/* [BSP]End added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */

	if (of_property_read_u32(np, "charging_host_charger_current", &val)
		>= 0) {
		info->data.charging_host_charger_current = val;
	} else {
		chr_err("use default CHARGING_HOST_CHARGER_CURRENT:%d\n",
			CHARGING_HOST_CHARGER_CURRENT);
		info->data.charging_host_charger_current =
					CHARGING_HOST_CHARGER_CURRENT;
	}

	/* dynamic mivr */
	info->enable_dynamic_mivr =
			of_property_read_bool(np, "enable_dynamic_mivr");
	info->enable_sw_aicl =
			of_property_read_bool(np, "enable_sw_aicl");

	if (of_property_read_u32(np, "min_charger_voltage_1", &val) >= 0)
		info->data.min_charger_voltage_1 = val;
	else {
		chr_err("use default V_CHARGER_MIN_1: %d\n", V_CHARGER_MIN_1);
		info->data.min_charger_voltage_1 = V_CHARGER_MIN_1;
	}

	if (of_property_read_u32(np, "min_charger_voltage_2", &val) >= 0)
		info->data.min_charger_voltage_2 = val;
	else {
		chr_err("use default V_CHARGER_MIN_2: %d\n", V_CHARGER_MIN_2);
		info->data.min_charger_voltage_2 = V_CHARGER_MIN_2;
	}

	if (of_property_read_u32(np, "max_dmivr_charger_current", &val) >= 0)
		info->data.max_dmivr_charger_current = val;
	else {
		chr_err("use default MAX_DMIVR_CHARGER_CURRENT: %d\n",
			MAX_DMIVR_CHARGER_CURRENT);
		info->data.max_dmivr_charger_current =
					MAX_DMIVR_CHARGER_CURRENT;
	}

/* Begin added by hailong.chen for task 9785237 on 2020-10-10 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	read_charger_para(info, np);
	override_charger_para(info, np);
#endif
/* End added by hailong.chen for task 9785237 on 2020-10-10 */
}

static void mtk_charger_start_timer(struct mtk_charger *info)
{
	struct timespec time, time_now;
	ktime_t ktime;
	int ret = 0;

	/* If the timer was already set, cancel it */
	ret = alarm_try_to_cancel(&info->charger_timer);
	if (ret < 0) {
		chr_err("%s: callback was running, skip timer\n", __func__);
		return;
	}

	get_monotonic_boottime(&time_now);
	time.tv_sec = info->polling_interval;
	time.tv_nsec = 0;
	info->endtime = timespec_add(time_now, time);
	ktime = ktime_set(info->endtime.tv_sec, info->endtime.tv_nsec);

	chr_err("%s: alarm timer start:%d, %ld %ld\n", __func__, ret,
		info->endtime.tv_sec, info->endtime.tv_nsec);
	alarm_start(&info->charger_timer, ktime);
}

/* Begin modified by hailong.chen for task 9777034 on 2020-08-20 */
#if !IS_ENABLED(TARGET_BUILD_MMITEST)
static void check_battery_exist(struct mtk_charger *info)
{
	unsigned int i = 0;
	int count = 0;
	//int boot_mode = get_boot_mode();

	if (is_disable_charger(info))
		return;

	for (i = 0; i < 3; i++) {
		if (is_battery_exist(info) == false)
			count++;
	}

#ifdef FIXME
	if (count >= 3) {
		/*1 = META_BOOT, 5 = ADVMETA_BOOT*/
		/*6 = ATE_FACTORY_BOOT */
		if (boot_mode == 1 || boot_mode == 5 ||
		    boot_mode == 6)
			chr_info("boot_mode = %d, bypass battery check\n",
				boot_mode);
		else {
			chr_err("battery doesn't exist, shutdown\n");
			orderly_poweroff(true);
		}
	}
#endif
}
#endif
/* End modified by hailong.chen for task 9777034 on 2020-08-20 */

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static void check_dynamic_mivr(struct mtk_charger *info)
{
	int i = 0, ret = 0;
	int vbat = 0;
	bool is_fast_charge = false;
	struct chg_alg_device *alg = NULL;

	if (!info->enable_dynamic_mivr)
		return;

	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		if (alg == NULL)
			continue;
		ret = chg_alg_is_algo_ready(alg);
		if (ret == ALG_RUNNING) {
			is_fast_charge = true;
			break;
		}
	}

	if (!is_fast_charge) {
		vbat = get_battery_voltage(info);
		if (vbat < info->data.min_charger_voltage_2 / 1000 - 200)
			charger_dev_set_mivr(info->chg1_dev,
				info->data.min_charger_voltage_2);
		else if (vbat < info->data.min_charger_voltage_1 / 1000 - 200)
			charger_dev_set_mivr(info->chg1_dev,
				info->data.min_charger_voltage_1);
		else
			charger_dev_set_mivr(info->chg1_dev,
				info->data.min_charger_voltage);
	}
}
#endif
/* End del by jin.wang */

/* Begin added by bitao.xiong for AOSP13TMO-4319 on 2022-08-03 */
#if defined(CONFIG_TCT_CHARGER)
static int health = POWER_SUPPLY_HEALTH_GOOD;
static void sw_jeita_state_machine_init(struct mtk_charger *info)
{
	struct sw_jeita_data *sw_jeita;

	if (IS_ERR_OR_NULL(info))
		return;

	if (info->enable_sw_jeita == true) {
		sw_jeita = &info->sw_jeita;
		info->battery_temp = get_battery_temperature(info);

		if (info->battery_temp >= info->data.temp_t4_thres)
			sw_jeita->sm = TEMP_ABOVE_T4;
		else if (info->battery_temp > info->data.temp_t3_thres)
			sw_jeita->sm = TEMP_T3_TO_T4;
		else if (info->battery_temp >= info->data.temp_t2_thres)
			sw_jeita->sm = TEMP_T2_TO_T3;
		else if (info->battery_temp >= info->data.temp_t1_thres)
			sw_jeita->sm = TEMP_T1_TO_T2;
		else if (info->battery_temp >= info->data.temp_t0_thres)
			sw_jeita->sm = TEMP_T0_TO_T1;
		else
			sw_jeita->sm = TEMP_BELOW_T0;

		chr_err("[%s] tmp:%d sm:%d\n", __func__, info->battery_temp, sw_jeita->sm);
	}
}

static void battery_update_health(int health)
{
	struct battery_data *bat_data = NULL;
	if (gm == NULL)
		gm = get_mtk_battery();
	if (gm) {
		bat_data = &gm->bs_data;
		bat_data->bat_health = health;
	}
}
#endif
/* End added by bitao.xiong for AOSP13TMO-4319 on 2022-08-03 */

/* Begin modified by hailong.chen for task 9777034 on 2020-08-20 */
#if IS_ENABLED(TARGET_BUILD_MMITEST)
void do_sw_jeita_state_machine(struct mtk_charger *info)
{
	struct sw_jeita_data *sw_jeita;
	sw_jeita = &info->sw_jeita;
	sw_jeita->pre_sm = sw_jeita->sm;
	sw_jeita->charging = true;
	sw_jeita->sm = TEMP_T2_TO_T3;
	sw_jeita->cv = 0;

	chr_err("[SW_JEITA]preState:%d newState:%d tmp:%d cv:%d\n",
		sw_jeita->pre_sm, sw_jeita->sm, info->battery_temp,
		sw_jeita->cv);
}
#else
/* sw jeita */
void do_sw_jeita_state_machine(struct mtk_charger *info)
{
	struct sw_jeita_data *sw_jeita;
/* Begin added by hailong.chen for task 9785237 on 2020-10-10 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	int vbat_now = 0;
#endif
/* End add by hailong.chen */

	sw_jeita = &info->sw_jeita;
	sw_jeita->pre_sm = sw_jeita->sm;
	sw_jeita->charging = true;

	/* JEITA battery temp Standard */
	if (info->battery_temp >= info->data.temp_t4_thres) {
		chr_err("[SW_JEITA] Battery Over high Temperature(%d) !!\n",
			info->data.temp_t4_thres);

		sw_jeita->sm = TEMP_ABOVE_T4;
		sw_jeita->charging = false;
	} else if (info->battery_temp > info->data.temp_t3_thres) {
		/* control 45 degree to normal behavior */
		if ((sw_jeita->sm == TEMP_ABOVE_T4)
		    && (info->battery_temp
			>= info->data.temp_t4_thres_minus_x_degree)) {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d,not allow charging yet!!\n",
				info->data.temp_t4_thres_minus_x_degree,
				info->data.temp_t4_thres);

			sw_jeita->charging = false;
		} else {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
				info->data.temp_t3_thres,
				info->data.temp_t4_thres);

			sw_jeita->sm = TEMP_T3_TO_T4;
		}
	} else if (info->battery_temp >= info->data.temp_t2_thres) {
		if (((sw_jeita->sm == TEMP_T3_TO_T4)
		     && (info->battery_temp
			 >= info->data.temp_t3_thres_minus_x_degree))
		    || ((sw_jeita->sm == TEMP_T1_TO_T2)
			&& (info->battery_temp
			    <= info->data.temp_t2_thres_plus_x_degree))) {
			chr_err("[SW_JEITA] Battery Temperature not recovery to normal temperature charging mode yet!!\n");
		} else {
			chr_err("[SW_JEITA] Battery Normal Temperature between %d and %d !!\n",
				info->data.temp_t2_thres,
				info->data.temp_t3_thres);
			sw_jeita->sm = TEMP_T2_TO_T3;
		}
	} else if (info->battery_temp >= info->data.temp_t1_thres) {
		if ((sw_jeita->sm == TEMP_T0_TO_T1
		     || sw_jeita->sm == TEMP_BELOW_T0)
		    && (info->battery_temp
			<= info->data.temp_t1_thres_plus_x_degree)) {
			if (sw_jeita->sm == TEMP_T0_TO_T1) {
				chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
					info->data.temp_t1_thres_plus_x_degree,
					info->data.temp_t2_thres);
			}
			if (sw_jeita->sm == TEMP_BELOW_T0) {
				chr_err("[SW_JEITA] Battery Temperature between %d and %d,not allow charging yet!!\n",
					info->data.temp_t1_thres,
					info->data.temp_t1_thres_plus_x_degree);
				sw_jeita->charging = false;
			}
		} else {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
				info->data.temp_t1_thres,
				info->data.temp_t2_thres);

			sw_jeita->sm = TEMP_T1_TO_T2;
		}
	} else if (info->battery_temp >= info->data.temp_t0_thres) {
		if ((sw_jeita->sm == TEMP_BELOW_T0)
		    && (info->battery_temp
			<= info->data.temp_t0_thres_plus_x_degree)) {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d,not allow charging yet!!\n",
				info->data.temp_t0_thres,
				info->data.temp_t0_thres_plus_x_degree);

			sw_jeita->charging = false;
		} else {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
				info->data.temp_t0_thres,
				info->data.temp_t1_thres);

			sw_jeita->sm = TEMP_T0_TO_T1;
		}
	} else {
		chr_err("[SW_JEITA] Battery below low Temperature(%d) !!\n",
			info->data.temp_t0_thres);
		sw_jeita->sm = TEMP_BELOW_T0;
		sw_jeita->charging = false;
	}

	/* set CV after temperature changed */
	/* In normal range, we adjust CV dynamically */
	if (sw_jeita->sm != TEMP_T2_TO_T3) {
		if (sw_jeita->sm == TEMP_ABOVE_T4)
			sw_jeita->cv = info->data.jeita_temp_above_t4_cv;
		else if (sw_jeita->sm == TEMP_T3_TO_T4)
			sw_jeita->cv = info->data.jeita_temp_t3_to_t4_cv;
/* Begin del by jin.wang for jira 2064 on 2021-10-25 */
#if !IS_ENABLED(CONFIG_TCT_CHARGER)
		else if (sw_jeita->sm == TEMP_T2_TO_T3)
			sw_jeita->cv = 0;
#endif
/* End del by jin.wang */
		else if (sw_jeita->sm == TEMP_T1_TO_T2)
			sw_jeita->cv = info->data.jeita_temp_t1_to_t2_cv;
		else if (sw_jeita->sm == TEMP_T0_TO_T1)
			sw_jeita->cv = info->data.jeita_temp_t0_to_t1_cv;
		else if (sw_jeita->sm == TEMP_BELOW_T0)
			sw_jeita->cv = info->data.jeita_temp_below_t0_cv;
		else
			sw_jeita->cv = info->data.battery_cv;
	} else {
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		sw_jeita->cv = info->data.battery_cv;
#else
		sw_jeita->cv = 0;
#endif
	}
/* End mod by jin.wang */

/* Begin added by hailong.chen for task 9785237 on 2020-10-10 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if ((info->enable_step_chg) && (sw_jeita->sm == TEMP_T2_TO_T3)) {
		vbat_now = get_battery_voltage(info) * 1000;
		if (info->chg_data[CHG1_SETTING].charging_current_limit_by_vbat == -1) {
			if(vbat_now > info->data.step_chg_vbat) {
				info->chg_data[CHG1_SETTING].charging_current_limit_by_vbat = info->data.setp_chg_current;
				info->chg_data[CHG2_SETTING].charging_current_limit_by_vbat = info->data.slave_setp_chg_current;
			}
		} else {
			if (vbat_now < (info->data.step_chg_vbat - info->data.step_chg_vbat_hysteresis)) {
				info->chg_data[CHG1_SETTING].charging_current_limit_by_vbat = -1;
				info->chg_data[CHG2_SETTING].charging_current_limit_by_vbat = -1;
			}
		}
	} else {
		info->chg_data[CHG1_SETTING].charging_current_limit_by_vbat = -1;
		info->chg_data[CHG2_SETTING].charging_current_limit_by_vbat = -1;
	}

	chr_err("[SW_JEITA]preState:%d newState:%d tmp:%d cv:%d chg:(%d %d) vbat_now:%d\n",
			sw_jeita->pre_sm, sw_jeita->sm, info->battery_temp,
			sw_jeita->cv, info->chg_data[CHG1_SETTING].charging_current_limit_by_vbat,
			info->chg_data[CHG2_SETTING].charging_current_limit_by_vbat, vbat_now);
#else
	chr_err("[SW_JEITA]preState:%d newState:%d tmp:%d cv:%d\n",
		sw_jeita->pre_sm, sw_jeita->sm, info->battery_temp,
		sw_jeita->cv);
#endif
/* End added by hailong.chen for task 9785237 on 2020-10-10 */

/* Begin added by bitao.xiong for AOSP13TMO-4319 on 2022-08-03 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	switch(sw_jeita->sm) {
	case TEMP_BELOW_T0:
		health = POWER_SUPPLY_HEALTH_COLD;
		break;
	case TEMP_T0_TO_T1:
	case TEMP_T1_TO_T2:
		health = POWER_SUPPLY_HEALTH_COOL;
		break;
	case TEMP_T2_TO_T3:
		health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case TEMP_T3_TO_T4:
		health = POWER_SUPPLY_HEALTH_WARM;
		break;
	case TEMP_ABOVE_T4:
		health = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	default:
		health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	}
	battery_update_health(health);
#endif
/* End added by bitao.xiong for AOSP13TMO-4319 on 2022-08-03 */
}
#endif
/* End modified by hailong.chen for task 9777034 on 2020-08-20 */

static int mtk_chgstat_notify(struct mtk_charger *info)
{
	int ret = 0;
	char *env[2] = { "CHGSTAT=1", NULL };

	chr_err("%s: 0x%x\n", __func__, info->notify_code);
	ret = kobject_uevent_env(&info->pdev->dev.kobj, KOBJ_CHANGE, env);
	if (ret)
		chr_err("%s: kobject_uevent_fail, ret=%d", __func__, ret);

	return ret;
}

/* Begin add by jin.wang for jira on 2021-11-30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static ssize_t thermal_disable_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: %d\n", __func__, pinfo->thermal_disable);
	return sprintf(buf, "%d\n", pinfo->thermal_disable);
}

static ssize_t thermal_disable_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp == 0)
			pinfo->thermal_disable = false;
		else
			pinfo->thermal_disable = true;

	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}
static DEVICE_ATTR_RW(thermal_disable);
#endif
/* End add by jin.wang */

static ssize_t sw_jeita_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: %d\n", __func__, pinfo->enable_sw_jeita);
	return sprintf(buf, "%d\n", pinfo->enable_sw_jeita);
}

static ssize_t sw_jeita_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp == 0)
			pinfo->enable_sw_jeita = false;
		else
			pinfo->enable_sw_jeita = true;

	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}

static DEVICE_ATTR_RW(sw_jeita);
/* sw jeita end*/

static ssize_t chr_type_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: %d\n", __func__, pinfo->chr_type);
	return sprintf(buf, "%d\n", pinfo->chr_type);
}

static ssize_t chr_type_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0)
		pinfo->chr_type = temp;
	else
		chr_err("%s: format error!\n", __func__);

	return size;
}

static DEVICE_ATTR_RW(chr_type);

static ssize_t Pump_Express_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret = 0, i = 0;
	bool is_ta_detected = false;
	struct mtk_charger *pinfo = dev->driver_data;
	struct chg_alg_device *alg = NULL;

	if (!pinfo) {
		chr_err("%s: pinfo is null\n", __func__);
		return sprintf(buf, "%d\n", is_ta_detected);
	}

	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = pinfo->alg[i];
		if (alg == NULL)
			continue;
		ret = chg_alg_is_algo_ready(alg);
		if (ret == ALG_RUNNING) {
			is_ta_detected = true;
			break;
		}
	}
	chr_err("%s: idx = %d, detect = %d\n", __func__, i, is_ta_detected);
	return sprintf(buf, "%d\n", is_ta_detected);
}

static DEVICE_ATTR_RO(Pump_Express);

static ssize_t ADC_Charger_Voltage_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	int vbus = get_vbus(pinfo); /* mV */

	chr_err("%s: %d\n", __func__, vbus);
	return sprintf(buf, "%d\n", vbus);
}

static DEVICE_ATTR_RO(ADC_Charger_Voltage);

static ssize_t Charger_Config_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	int chg_cfg = pinfo->config;

	chr_err("%s: %d\n", __func__, chg_cfg);
	return sprintf(buf, "%d\n", chg_cfg);
}

static DEVICE_ATTR_RO(Charger_Config);

/* Begin mod by jin.wang task 2064 on 2021.11.25 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static ssize_t input_current_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	int aicr = 0;

	//mutex_lock(&pinfo->charger_lock);
	aicr = pinfo->chg_data[CHG1_SETTING].ibus_limit_by_test;
	pr_err("%s: %d\n", __func__, aicr);
	//mutex_unlock(&pinfo->charger_lock);
	return sprintf(buf, "%d\n", aicr);
}
#else
static ssize_t input_current_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	int aicr = 0;

	aicr = pinfo->chg_data[CHG1_SETTING].thermal_input_current_limit;
	chr_err("%s: %d\n", __func__, aicr);
	return sprintf(buf, "%d\n", aicr);
}
#endif
/* End mod by jin.wang */

/* Begin mod by jin.wang task 2064 on 2021.11.25 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static ssize_t input_current_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	struct charger_data *chg_data;
	signed int temp;

	mutex_lock(&pinfo->charger_lock);
	chg_data = &pinfo->chg_data[CHG1_SETTING];
	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp < 0)
			chg_data->ibus_limit_by_test = -1;
		else
			chg_data->ibus_limit_by_test = temp;
	} else {
		chr_err("%s: format error!\n", __func__);
	}
	pr_err("%s: userspace set ibus: %d\n", __func__, temp);
	mutex_unlock(&pinfo->charger_lock);
	return size;
}
#else
static ssize_t input_current_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	struct charger_data *chg_data;
	signed int temp;

	chg_data = &pinfo->chg_data[CHG1_SETTING];
	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp < 0)
			chg_data->thermal_input_current_limit = 0;
		else
			chg_data->thermal_input_current_limit = temp;
	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}
#endif
/* End mod by jin.wang */

static DEVICE_ATTR_RW(input_current);

static ssize_t charger_log_level_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: %d\n", __func__, pinfo->log_level);
	return sprintf(buf, "%d\n", pinfo->log_level);
}

static ssize_t charger_log_level_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp < 0) {
			chr_err("%s: val is invalid: %ld\n", __func__, temp);
			temp = 0;
		}
		pinfo->log_level = temp;
		chr_err("%s: log_level=%d\n", __func__, pinfo->log_level);

	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}

static DEVICE_ATTR_RW(charger_log_level);

static ssize_t BatteryNotify_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_info("%s: 0x%x\n", __func__, pinfo->notify_code);

	return sprintf(buf, "%u\n", pinfo->notify_code);
}

static ssize_t BatteryNotify_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int reg = 0;
	int ret = 0;

	if (buf != NULL && size != 0) {
		ret = kstrtouint(buf, 16, &reg);
		if (ret < 0) {
			chr_err("%s: failed, ret = %d\n", __func__, ret);
			return ret;
		}
		pinfo->notify_code = reg;
		chr_info("%s: store code=0x%x\n", __func__, pinfo->notify_code);
		mtk_chgstat_notify(pinfo);
	}
	return size;
}

static DEVICE_ATTR_RW(BatteryNotify);

/* Begin added by bin.song.hz for defect10297066 on 2020.12.03 */
static ssize_t BatJeitaStatus_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	int sm = BAT_TEMP_NORMAL;
	if (pinfo->enable_sw_jeita) {
		if (TEMP_BELOW_T0 == pinfo->sw_jeita.sm) {//init value is TEMP_BELOW_T0
			if (pinfo->battery_temp <= pinfo->data.temp_t0_thres_plus_x_degree)
				sm = BAT_TEMP_LOW;
			else if (pinfo->battery_temp >= pinfo->data.temp_t4_thres)
				sm = BAT_TEMP_HIGH;
			else
				sm = BAT_TEMP_NORMAL;
		} else if (TEMP_ABOVE_T4 == pinfo->sw_jeita.sm)
			sm = BAT_TEMP_HIGH;
		else
			sm = BAT_TEMP_NORMAL;
	} else {
		if (BAT_TEMP_LOW == pinfo->thermal.sm) {//init value is BAT_TEMP_LOW
			if (pinfo->battery_temp < pinfo->thermal.min_charge_temp_plus_x_degree)
				sm =BAT_TEMP_LOW;
			else if (pinfo->battery_temp >= pinfo->thermal.max_charge_temp)
				sm =BAT_TEMP_HIGH;
			else
				sm = BAT_TEMP_NORMAL;
		} else
			sm = pinfo->thermal.sm;
	}
	return sprintf(buf, "%u\n", sm);
}

static DEVICE_ATTR_RO(BatJeitaStatus);
/* End added by bin.song.hz for defect10297066 on 2020.12.03 */

/* Begin modify by tangshan for GAIAGL-5497 on 20230731 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
static ssize_t show_soc_limited(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	pr_debug("[minisw] %s : %d\n", __func__, soc_limited);
	return sprintf(buf, "%u\n", soc_limited);
}

static ssize_t store_soc_limited(struct device *dev,
		struct device_attribute *attr, const char *buf,  size_t size)
{
	unsigned int val = 0;
	int ret;

	if (buf != NULL && size != 0) {
		ret = kstrtouint(buf, 10, &val);
		soc_limited = val;
		pr_err("[minisw] store soc_limited = %d\n", soc_limited);
	}
	return size;
}
static DEVICE_ATTR(soc_limited, 0644, show_soc_limited, store_soc_limited);
#endif
/* End modify by tangshan for GAIAGL-5497 on 20230731 */
/* End add by jin.wang */

/* procfs */
static int mtk_chg_current_cmd_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;

	seq_printf(m, "%d %d\n", pinfo->usb_unlimited, pinfo->cmd_discharging);
	return 0;
}

static int mtk_chg_current_cmd_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_current_cmd_show, PDE_DATA(node));
}

static ssize_t mtk_chg_current_cmd_write(struct file *file,
		const char *buffer, size_t count, loff_t *data)
{
	int len = 0;
	char desc[32] = {0};
	int current_unlimited = 0;
	int cmd_discharging = 0;
	struct mtk_charger *info = PDE_DATA(file_inode(file));

	if (!info)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	if (sscanf(desc, "%d %d", &current_unlimited, &cmd_discharging) == 2) {
		info->usb_unlimited = current_unlimited;
		if (cmd_discharging == 1) {
			info->cmd_discharging = true;
			charger_dev_enable(info->chg1_dev, false);
			charger_dev_do_event(info->chg1_dev,
					EVENT_DISCHARGE, 0);
		} else if (cmd_discharging == 0) {
			info->cmd_discharging = false;
			charger_dev_enable(info->chg1_dev, true);
			charger_dev_do_event(info->chg1_dev,
					EVENT_RECHARGE, 0);
		}

		chr_info("%s: current_unlimited=%d, cmd_discharging=%d\n",
			__func__, current_unlimited, cmd_discharging);
		return count;
	}

	chr_err("bad argument, echo [usb_unlimited] [disable] > current_cmd\n");
	return count;
}

static const struct file_operations mtk_chg_current_cmd_fops = {
	.owner = THIS_MODULE,
	.open = mtk_chg_current_cmd_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = mtk_chg_current_cmd_write,
};

static int mtk_chg_en_power_path_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;
	bool power_path_en = true;

	charger_dev_is_powerpath_enabled(pinfo->chg1_dev, &power_path_en);
	seq_printf(m, "%d\n", power_path_en);

	return 0;
}

static int mtk_chg_en_power_path_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_en_power_path_show, PDE_DATA(node));
}

static ssize_t mtk_chg_en_power_path_write(struct file *file,
		const char *buffer, size_t count, loff_t *data)
{
	int len = 0, ret = 0;
	char desc[32] = {0};
	unsigned int enable = 0;
	struct mtk_charger *info = PDE_DATA(file_inode(file));

	if (!info)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	ret = kstrtou32(desc, 10, &enable);
	if (ret == 0) {
		charger_dev_enable_powerpath(info->chg1_dev, enable);
		chr_info("%s: enable power path = %d\n", __func__, enable);
		return count;
	}

	chr_err("bad argument, echo [enable] > en_power_path\n");
	return count;
}

static const struct file_operations mtk_chg_en_power_path_fops = {
	.owner = THIS_MODULE,
	.open = mtk_chg_en_power_path_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = mtk_chg_en_power_path_write,
};

static int mtk_chg_en_safety_timer_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;
	bool safety_timer_en = false;
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	int ret = 0;
#endif

#if IS_ENABLED(CONFIG_TCT_CHARGER)
	ret = charger_dev_is_safety_timer_enabled(pinfo->chg1_dev, &safety_timer_en);
	if (ret < 0)
		seq_printf(m, "%d\n", pinfo->enable_sw_safety_timer);
	else
		seq_printf(m, "%d\n", safety_timer_en);
#else
	charger_dev_is_safety_timer_enabled(pinfo->chg1_dev, &safety_timer_en);
	seq_printf(m, "%d\n", safety_timer_en);
#endif

	return 0;
}

static int mtk_chg_en_safety_timer_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_en_safety_timer_show, PDE_DATA(node));
}

static ssize_t mtk_chg_en_safety_timer_write(struct file *file,
	const char *buffer, size_t count, loff_t *data)
{
	int len = 0, ret = 0;
	char desc[32] = {0};
	unsigned int enable = 0;
	struct mtk_charger *info = PDE_DATA(file_inode(file));

	if (!info)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	ret = kstrtou32(desc, 10, &enable);
	if (ret == 0) {
		charger_dev_enable_safety_timer(info->chg1_dev, enable);
		chr_info("%s: enable safety timer = %d\n", __func__, enable);

		/* SW safety timer */
		if (info->sw_safety_timer_setting == true) {
			if (enable)
				info->enable_sw_safety_timer = true;
			else
				info->enable_sw_safety_timer = false;
		}

		return count;
	}

	chr_err("bad argument, echo [enable] > en_safety_timer\n");
	return count;
}

static const struct file_operations mtk_chg_en_safety_timer_fops = {
	.owner = THIS_MODULE,
	.open = mtk_chg_en_safety_timer_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = mtk_chg_en_safety_timer_write,
};

int mtk_chg_enable_vbus_ovp(bool enable)
{
	static struct mtk_charger *pinfo;
	int ret = 0;
	u32 sw_ovp = 0;
	struct power_supply *psy;

	if (pinfo == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
		if (psy == NULL) {
			chr_err("[%s]psy is not rdy\n", __func__);
			return -1;
		}

		pinfo = (struct mtk_charger *)power_supply_get_drvdata(psy);
		if (pinfo == NULL) {
			chr_err("[%s]mtk_gauge is not rdy\n", __func__);
			return -1;
		}
	}

	if (enable)
		sw_ovp = pinfo->data.max_charger_voltage_setting;
	else
		sw_ovp = 15000000;

	/* Enable/Disable SW OVP status */
	pinfo->data.max_charger_voltage = sw_ovp;

	disable_hw_ovp(pinfo, enable);

	chr_err("[%s] en:%d ovp:%d\n",
			    __func__, enable, sw_ovp);
	return ret;
}

/* Begin added by bitao.xiong for AOSP13TMO-4319 on 2022-08-03 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
int battery_do_health_update(struct mtk_charger *info)
{
	if (info != NULL && (!mutex_is_locked(&info->charger_lock))) {
		info->battery_temp = get_battery_temperature(info);
		do_sw_jeita_state_machine(info);
		chr_err("%s:  (jeita:%d sm:%d cv:%d en:%d) \n", __func__,
		info->enable_sw_jeita, info->sw_jeita.sm,
		info->sw_jeita.cv, info->sw_jeita.charging);
	}
	return 0;
}
EXPORT_SYMBOL(battery_do_health_update);
#endif
/* End added by bitao.xiong for AOSP13TMO-4319 on 2022-08-03 */

/* return false if vbus is over max_charger_voltage */
static bool mtk_chg_check_vbus(struct mtk_charger *info)
{
	int vchr = 0;

	vchr = get_vbus(info) * 1000; /* uV */
/* Begin modified by dapeng.qiao for task 11038299 on 2021-05-1 */
#ifdef TCT_BMS_SW_SUPPORT
    g_chr_vol = vchr;
#endif
/* End modified by dapeng.qiao for task 11038299 on 2021-05-1 */
	if (vchr > info->data.max_charger_voltage) {
		chr_err("%s: vbus(%d mV) > %d mV\n", __func__, vchr / 1000,
			info->data.max_charger_voltage / 1000);
		return false;
	}
	return true;
}

static void mtk_battery_notify_VCharger_check(struct mtk_charger *info)
{
#if defined(BATTERY_NOTIFY_CASE_0001_VCHARGER)
	int vchr = 0;

	vchr = get_vbus(info) * 1000; /* uV */
	if (vchr < info->data.max_charger_voltage)
		info->notify_code &= ~CHG_VBUS_OV_STATUS;
	else {
		info->notify_code |= CHG_VBUS_OV_STATUS;
		chr_err("[BATTERY] charger_vol(%d mV) > %d mV\n",
			vchr / 1000, info->data.max_charger_voltage / 1000);
		mtk_chgstat_notify(info);
	}
#endif
}

static void mtk_battery_notify_VBatTemp_check(struct mtk_charger *info)
{
#if defined(BATTERY_NOTIFY_CASE_0002_VBATTEMP)
	if (info->battery_temp >= info->thermal.max_charge_temp) {
		info->notify_code |= CHG_BAT_OT_STATUS;
		chr_err("[BATTERY] bat_temp(%d) out of range(too high)\n",
			info->battery_temp);
		mtk_chgstat_notify(info);
	} else {
		info->notify_code &= ~CHG_BAT_OT_STATUS;
	}

	if (info->enable_sw_jeita == true) {
		if (info->battery_temp < info->data.temp_neg_10_thres) {
			info->notify_code |= CHG_BAT_LT_STATUS;
			chr_err("bat_temp(%d) out of range(too low)\n",
				info->battery_temp);
			mtk_chgstat_notify(info);
		} else {
			info->notify_code &= ~CHG_BAT_LT_STATUS;
		}
	} else {
#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
		if (info->battery_temp < info->thermal.min_charge_temp) {
			info->notify_code |= CHG_BAT_LT_STATUS;
			chr_err("bat_temp(%d) out of range(too low)\n",
				info->battery_temp);
			mtk_chgstat_notify(info);
		} else {
			info->notify_code &= ~CHG_BAT_LT_STATUS;
		}
#endif
	}
#endif
}

static void mtk_battery_notify_UI_test(struct mtk_charger *info)
{
	switch (info->notify_test_mode) {
	case 1:
		info->notify_code = CHG_VBUS_OV_STATUS;
		chr_debug("[%s] CASE_0001_VCHARGER\n", __func__);
		break;
	case 2:
		info->notify_code = CHG_BAT_OT_STATUS;
		chr_debug("[%s] CASE_0002_VBATTEMP\n", __func__);
		break;
	case 3:
		info->notify_code = CHG_OC_STATUS;
		chr_debug("[%s] CASE_0003_ICHARGING\n", __func__);
		break;
	case 4:
		info->notify_code = CHG_BAT_OV_STATUS;
		chr_debug("[%s] CASE_0004_VBAT\n", __func__);
		break;
	case 5:
		info->notify_code = CHG_ST_TMO_STATUS;
		chr_debug("[%s] CASE_0005_TOTAL_CHARGINGTIME\n", __func__);
		break;
	case 6:
		info->notify_code = CHG_BAT_LT_STATUS;
		chr_debug("[%s] CASE6: VBATTEMP_LOW\n", __func__);
		break;
	case 7:
		info->notify_code = CHG_TYPEC_WD_STATUS;
		chr_debug("[%s] CASE7: Moisture Detection\n", __func__);
		break;
	default:
		chr_debug("[%s] Unknown BN_TestMode Code: %x\n",
			__func__, info->notify_test_mode);
	}
	mtk_chgstat_notify(info);
}

static void mtk_battery_notify_check(struct mtk_charger *info)
{
	if (info->notify_test_mode == 0x0000) {
		mtk_battery_notify_VCharger_check(info);
		mtk_battery_notify_VBatTemp_check(info);
	} else {
		mtk_battery_notify_UI_test(info);
	}
}

/* Begin del by jin.wang for task 2064 on 2021-10-5 */
#if !IS_ENABLED(CONFIG_TCT_CHARGER)
static void mtk_chg_get_tchg(struct mtk_charger *info)
{
	int ret;
	int tchg_min = -127, tchg_max = -127;
	struct charger_data *pdata;

	pdata = &info->chg_data[CHG1_SETTING];
	ret = charger_dev_get_temperature(info->chg1_dev, &tchg_min, &tchg_max);
	if (ret < 0) {
		pdata->junction_temp_min = -127;
		pdata->junction_temp_max = -127;
	} else {
		pdata->junction_temp_min = tchg_min;
		pdata->junction_temp_max = tchg_max;
	}

	if (info->chg2_dev) {
		pdata = &info->chg_data[CHG2_SETTING];
		ret = charger_dev_get_temperature(info->chg2_dev,
			&tchg_min, &tchg_max);

		if (ret < 0) {
			pdata->junction_temp_min = -127;
			pdata->junction_temp_max = -127;
		} else {
			pdata->junction_temp_min = tchg_min;
			pdata->junction_temp_max = tchg_max;
		}
	}
}
#endif
/* End del by jin.wang */


/* Begin mod by jin.wang for androidT on 2022-4-12 */
#if IS_ENABLED(CONFIG_TCT_CHARGER_GCS)
#if !IS_ENABLED(TARGET_BUILD_MMITEST)
/* [BSP]Begin deleted by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
#if 0
/* Begin added by bitao.xiong for defect-10090020 on 2020-11-19 */
#if IS_ENABLED(CONFIG_CHARGER_BQ24158_V1) || IS_ENABLED(CONFIG_CHARGER_BQ24158)
static int pmic_get_charging_current(void)
{
	int ret = 0;
	int v_batsns = 0, v_isense = 0;

	ret = gauge_get_property(GAUGE_PROP_ISENSE_VOLTAGE, &v_isense);
	if (ret)
		return ret;

	ret = gauge_get_property(GAUGE_PROP_BATTERY_VOLTAGE, &v_batsns);
	if (ret)
		return ret;

	return ((v_isense - v_batsns) * 1000 / 68) * 1000;
}
#endif

static void mtk_charger_check_input_avg_current(struct mtk_charger *info, bool is_charger_on)
{
	int chg_current = 0;
	int i = 0, ichg = 0;

	if (info == NULL) {
		info->input_avg_current = 0;
		return;
	}

	if (is_charger_on == true) {
		for (i = 0; i < 5; i++) {
			msleep(200);
			#if IS_ENABLED(CONFIG_CHARGER_BQ24158_V1) || IS_ENABLED(CONFIG_CHARGER_BQ24158)
			ichg = pmic_get_charging_current();
			#else
			ichg = get_ibus(info) * 1000;
			#endif
			if (ichg <= 0)
				return;
			chg_current += ichg;
		}
		info->input_avg_current = chg_current / i;
	} else {
		info->input_avg_current = 0;
	}
}

/* Begin added by bitao.xiong for defect-11669535 on 2021-11-16 */
static bool is_nonstandard_charger_by_current(struct mtk_charger *info)
{
	int threshold = 70;
	int vbat = 0;
	int ret;

	if (info == NULL)
		return false;

	ret = gauge_get_property(GAUGE_PROP_BATTERY_VOLTAGE, &vbat);
	if (ret)
		return false;
#if IS_ENABLED(CONFIG_CHARGER_BQ24158_V1) || IS_ENABLED(CONFIG_CHARGER_BQ24158)
	if (vbat < 3800)
		threshold = 100;
	else if (vbat < 4000)
		threshold = 90;
	else
		threshold = 85;
#endif
	chr_err("%s, vbat=%d, threshold=%d, input_avg_current=%d, ac_charger_current=%d, \
			 ac_charger_input_current=%d, threshold_current=%d\n", __func__, vbat, threshold,
					info->input_avg_current, info->data.ac_charger_current,
					info->data.ac_charger_input_current,
					info->data.ac_charger_input_current * threshold / 100);
	if (info->input_avg_current > 0  && info->input_avg_current < info->data.ac_charger_input_current * threshold / 100)
		return true;
	else
		return false;
}
/* End added by bitao.xiong for defect-11669535 on 2021-11-16 */

static void mtk_charger_check_nonstandard_charger(struct mtk_charger *info, bool is_charger_on)
{
	struct charger_data *chg_data;
	bool is_fast_charge = false;
	struct chg_alg_device *alg = NULL;
	int i = 0, ret;
	int chr_type;
	struct timespec time_now, diff_time;

	if (info == NULL) {
		info->input_avg_current = 0;
		return;
	}
	chg_data = &info->chg_data[CHG1_SETTING];
	chr_type = get_charger_type(info);
	chr_err("%s, chg_type=%d, %d, %d, %d, %d, enable_hv_charging=%d\n", __func__, get_charger_type(info),
		chg_data->thermal_input_current_limit, chg_data->thermal_charging_current_limit,
		info->enable_sw_jeita,info->sw_jeita.sm, info->enable_hv_charging);

	if (info->nonstand_chg_type != NONSTANDARD_CHARGER_INVALIED)
		return;

	if (chr_type == POWER_SUPPLY_TYPE_USB || chr_type == POWER_SUPPLY_TYPE_USB_CDP)
		info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;
	else if (chr_type == POWER_SUPPLY_TYPE_USB_FLOAT) {
		get_monotonic_boottime(&time_now);
		diff_time = timespec_sub(time_now, info->charging_begin_time);
		if((diff_time.tv_sec < 25)&&(diff_time.tv_sec >= 10)) {
			info->nonstand_chg_type = NONSTANDARD_CHARGER_NONSTAND;
			if (!IS_ERR_OR_NULL(info->bat_psy))
				power_supply_changed(info->bat_psy);
		}
	} else if (chr_type == POWER_SUPPLY_TYPE_USB_DCP && is_charger_on) {
		if (chg_data->thermal_input_current_limit == -1 &&
				chg_data->thermal_charging_current_limit == -1 &&
				info->enable_sw_jeita && info->sw_jeita.sm == TEMP_T2_TO_T3 &&
				get_uisoc(info) < 80 && get_uisoc(info) > 1) {
			get_monotonic_boottime(&time_now);
			diff_time = timespec_sub(time_now, info->charging_begin_time);
			if((diff_time.tv_sec < 70)&&(diff_time.tv_sec >= 40)) {
				for (i = 0; i < MAX_ALG_NO; i++) {
					alg = info->alg[i];
					if (alg == NULL)
						continue;
					ret = chg_alg_is_algo_ready(alg);
					if (ret == ALG_RUNNING) {
						is_fast_charge = true;
						info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;
						break;
					}
				}

				if (!is_fast_charge) {
					mtk_charger_check_input_avg_current(info, is_charger_on);
					if (is_nonstandard_charger_by_current(info)) {
						info->nonstand_chg_type = NONSTANDARD_CHARGER_SLOW;
						if (!IS_ERR_OR_NULL(info->bat_psy))
							power_supply_changed(info->bat_psy);
					} else {
						info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;
					}
				}
				chr_err("%s, nonstand_chg_type=0x%x, is_fast_charge=%d\n", __func__,  info->nonstand_chg_type, is_fast_charge);
			}
		}
	}
}

static ssize_t nonstand_charge_type_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	struct mtk_charger *info = dev->driver_data;
#if IS_ENABLED(TARGET_BUILD_CERTIFICATION) || IS_ENABLED(TARGET_BUILD_MMITEST) \
		|| IS_ENABLED(DISABLE_TEMPERATURE_DETECTION_AND_THERMAL_POLICY)
	chr_err("%s: This is not cu Version, nonstand_charge_type is always NONSTANDARD_CHARGER_NONE\n", __func__);
	info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;
#else
	chr_err("%s: type:0x%x, input_avg_current=%d\n", __func__, info->nonstand_chg_type, info->input_avg_current);
#endif
	return sprintf(buf, "0x%x\n", info->nonstand_chg_type);
}
#endif
/* [BSP]End deleted by bitao.xiong for SNTBBH-4343 on 2022/12/20 */

/* [BSP]Begin modified by bitao.xiong for BORANAOM-2761 on 2023/01/31 */
#if 0
static ssize_t nonstand_charge_type_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	struct mtk_charger *info = dev->driver_data;
	struct charger_data *pdata1 = &info->chg_data[CHG1_SETTING];
	const int weak_chg_input_current = info->data.weak_ac_charger_input_current;

	if (get_charger_type(info) == POWER_SUPPLY_TYPE_UNKNOWN)
		info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;

	if (info->nonstand_chg_type != NONSTANDARD_CHARGER_INVALIED)
		return sprintf(buf, "0x%x\n", info->nonstand_chg_type);

	if (get_charger_type(info) == POWER_SUPPLY_TYPE_USB_FLOAT)
		info->nonstand_chg_type = NONSTANDARD_CHARGER_NONSTAND;
	else if(get_charger_type(info) == POWER_SUPPLY_TYPE_USB_DCP) {
		if (get_uisoc(info) < 80) {
			if (pdata1->input_current_limit_by_aicl > 0) {
				if (pdata1->input_current_limit_by_aicl <= weak_chg_input_current)
					info->nonstand_chg_type = NONSTANDARD_CHARGER_SLOW;
				else
					info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;
			} else {
				info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;
			}
		} else {
			info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;
		}
	} else {
		info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;
	}

	chr_err("[cwldebug %s] input_current_limit_by_aicl=%d nonstand_chg_type=%d\n", __func__, pdata1->input_current_limit_by_aicl, info->nonstand_chg_type);
	return sprintf(buf, "0x%x\n", info->nonstand_chg_type);
}
#else
static ssize_t nonstand_charge_type_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	struct mtk_charger *info = dev->driver_data;

	if (info->nonstand_chg_type != NONSTANDARD_CHARGER_INVALIED)
		return sprintf(buf, "0x%x\n", info->nonstand_chg_type);

	if (get_charger_type(info) == POWER_SUPPLY_TYPE_USB_FLOAT)
		info->nonstand_chg_type = NONSTANDARD_CHARGER_NONSTAND;
	else
		info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;

	chr_err("[cwldebug %s] nonstand_chg_type=%d\n", __func__, info->nonstand_chg_type);
	return sprintf(buf, "0x%x\n", info->nonstand_chg_type);
}
#endif
/* [BSP]End modified by bitao.xiong for BORANAOM-2761 on 2023/01/31 */
DEVICE_ATTR_RO(nonstand_charge_type);
/* End added by bitao.xiong for defect-10090020 on 2020-11-19 */
#endif
#endif
/* End mod by jin.wang */

#ifdef CONFIG_HELAEYE_BSP_CHARGER_ON
extern int charger_driver_lasterrcode;
extern int charger_driver_lasterrcode_value;
void heraeye_bat_check_temp(int errtype, int charger_driver_value);
#endif

/* Begin mod by jin.wang for androidT on 2022-4-14 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static void charger_check_status(struct mtk_charger *info)
{
	bool charging = true;
	int temperature;
	struct battery_thermal_protection_data *thermal;
/* [BSP]Begin modified by bitao.xiong for SNTTF-635 on 2022/11/04 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	int uisoc = 1;
#endif
/* [BSP]End modified by bitao.xiong for SNTTF-635 on 2022/11/04 */

	struct mtk_battery *gm = get_mtk_battery();

	if (get_charger_type(info) == POWER_SUPPLY_TYPE_UNKNOWN)
		return;

	temperature = info->battery_temp;
	thermal = &info->thermal;

	if (info->enable_sw_jeita == true) {
		do_sw_jeita_state_machine(info);
		if (info->sw_jeita.charging == false) {
			charging = false;
			goto stop_charging;
		}
	} else {

		if (thermal->enable_min_charge_temp) {
			if (temperature < thermal->min_charge_temp) {
				chr_err("Battery Under Temperature or NTC fail %d %d\n",
					temperature, thermal->min_charge_temp);
				thermal->sm = BAT_TEMP_LOW;
#ifdef CONFIG_HELAEYE_BSP_CHARGER_ON
				heraeye_bat_check_temp(BAT_TEMP_LOW,temperature);
#endif
				charging = false;
				goto stop_charging;
			} else if (thermal->sm == BAT_TEMP_LOW) {
				if (temperature >=
				    thermal->min_charge_temp_plus_x_degree) {
					chr_err("Battery Temperature raise from %d to %d(%d), allow charging!!\n",
					thermal->min_charge_temp,
					temperature,
					thermal->min_charge_temp_plus_x_degree);
					thermal->sm = BAT_TEMP_NORMAL;
				} else {
					charging = false;
					goto stop_charging;
				}
			}
		}

		if (temperature >= thermal->max_charge_temp) {
			chr_err("Battery over Temperature or NTC fail %d %d\n",
				temperature, thermal->max_charge_temp);
			thermal->sm = BAT_TEMP_HIGH;
			charging = false;
#ifdef CONFIG_HELAEYE_BSP_CHARGER_ON
				heraeye_bat_check_temp(BAT_TEMP_HIGH,temperature);
#endif
			goto stop_charging;
		} else if (thermal->sm == BAT_TEMP_HIGH) {
			if (temperature
			    < thermal->max_charge_temp_minus_x_degree) {
				chr_err("Battery Temperature raise from %d to %d(%d), allow charging!!\n",
				thermal->max_charge_temp,
				temperature,
				thermal->max_charge_temp_minus_x_degree);
				thermal->sm = BAT_TEMP_NORMAL;
			} else {
				charging = false;
				goto stop_charging;
			}
		}
	}

	if (!mtk_chg_check_vbus(info)) {
		charging = false;
		goto stop_charging;
	}

/* [BSP]Begin modified by bitao.xiong for SNTTF-635 on 2022/11/04 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (is_mmitest || is_inproductionflag) {
		chr_err("[minisw] soc_limited = %d\n", soc_limited);
		if (soc_limited) {
			uisoc = get_uisoc(info);
			if (uisoc >= MINI_SUSPEND_CHG_SOC) {
				chr_err("uisoc up to %d(%d), stop chg!!\n",
						uisoc, MINI_SUSPEND_CHG_SOC);
				mini_soc_limited = 1;
				charging = false;
				goto stop_charging;
			} else if (mini_soc_limited) {
				if (uisoc <= MINI_RESUME_CHG_SOC) {
					chr_err("uisoc down to %d(%d), resume chg!!\n",
							uisoc, MINI_RESUME_CHG_SOC);
					mini_soc_limited = 0;
				} else {
					charging = false;
					goto stop_charging;
				}
			}
		} else {
			mini_soc_limited = 0;
		}
	}
#endif
/* [BSP]End modified by bitao.xiong for SNTTF-635 on 2022/11/04 */

/* Begin added by hailong.chen for task 9777034 on 2020-08-20 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (gm && gm->chg_disable) {
		chr_err("cmd disable charging\n");
		charging = false;
		goto stop_charging;
	}

	if (!strncmp(get_battery_type(info), "NA:NA:NA:NA", strlen("NA:NA:NA:NA"))) {
		chr_err("Third-party battery,stop charging\n");
		charging = false;
		goto stop_charging;
	}
#endif
/* End added by hailong.chen for task 9777034 on 2020-08-20 */

	if (info->cmd_discharging)
		charging = false;
	if (info->safety_timeout)
		charging = false;
	if (info->vbusov_stat)
		charging = false;
	/* Begin Added by tangshan.bai for LEVIN-6148 */
    #if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
    if (gm->peak_enforce_full)
		charging = false;
    #endif
    /* End Added by tangshan.bai for LEVIN-6148 */

stop_charging:
	/* Begin Added by tangshan.bai for LEVIN-6148 */
    #if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
    if (gm->peak_enforce_full)
    	gm->bat_state = BAT_ENFORCE_TO_FULL;
    else{
		if (false == charging)
			gm->bat_state = BAT_NOTCHARGING;
		else
			gm->bat_state = BAT_CHARGING;
    }

    #endif
    /* End Added by tangshan.bai for LEVIN-6148 */

	mtk_battery_notify_check(info);

	chr_err("tmp:%d (jeita:%d sm:%d cv:%d en:%d) (sm:%d) en:%d c:%d s:%d ov:%d %d %d\n",
		temperature, info->enable_sw_jeita, info->sw_jeita.sm,
		info->sw_jeita.cv, info->sw_jeita.charging, thermal->sm,
		charging, info->cmd_discharging, info->safety_timeout,
		info->vbusov_stat, info->can_charging, charging);

	if (charging != info->can_charging)
		_mtk_enable_charging(info, charging);

	info->can_charging = charging;
	/* [BSP]Begin added by bitao.xiong for SNTBBH-2995 on 2022/11/22 */
	#if IS_ENABLED(CONFIG_TCT_CHARGER_GCS)
	#if !defined(TARGET_BUILD_MMITEST)
	/* [BSP]Begin deleted by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
	#if 0
	mtk_charger_check_nonstandard_charger(info, info->can_charging);
	#endif
	/* [BSP]End deleted by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
	#endif
	#endif
	/* [BSP]End added by bitao.xiong for SNTBBH-2995 on 2022/11/22 */
}
#else
static void charger_check_status(struct mtk_charger *info)
{
	bool charging = true;
	int temperature;
	struct battery_thermal_protection_data *thermal;
	#if IS_ENABLED(CONFIG_TCT_CHARGER)
	int uisoc = 1;
	/* Begin added by bitao.xiong for task-9895401 on 2020-09-11 */
	union power_supply_propval prop;
	/* End added by bitao.xiong for task-9895401 on 2020-09-11 */
	/* Begin added by bitao.xiong for task-9878354 on 2020-09-05 */
	struct fuel_gauge_custom_data *fg_cust_data;
	int ret;
	/* End added by bitao.xiong for task-9878354 on 2020-09-05 */
	#endif

	if (get_charger_type(info) == POWER_SUPPLY_TYPE_UNKNOWN)
		return;

	#if IS_ENABLED(CONFIG_TCT_CHARGER)
    if (info->chg1_dev != NULL)
        charger_dev_kick_wdt(info->chg1_dev);

    if (info->chg2_dev != NULL)
        charger_dev_kick_wdt(info->chg2_dev);
	#endif

	temperature = info->battery_temp;
	thermal = &info->thermal;

	if (info->enable_sw_jeita == true) {
		do_sw_jeita_state_machine(info);
		if (info->sw_jeita.charging == false) {
			charging = false;
			goto stop_charging;
		}
	} else {

		if (thermal->enable_min_charge_temp) {
			if (temperature < thermal->min_charge_temp) {
				chr_err("Battery Under Temperature or NTC fail %d %d\n",
					temperature, thermal->min_charge_temp);
				thermal->sm = BAT_TEMP_LOW;
				charging = false;
#ifdef CONFIG_HELAEYE_BSP_CHARGER_ON
				heraeye_bat_check_temp(BAT_TEMP_LOW,temperature);
#endif
				goto stop_charging;
			} else if (thermal->sm == BAT_TEMP_LOW) {
				if (temperature >=
				    thermal->min_charge_temp_plus_x_degree) {
					chr_err("Battery Temperature raise from %d to %d(%d), allow charging!!\n",
					thermal->min_charge_temp,
					temperature,
					thermal->min_charge_temp_plus_x_degree);
					thermal->sm = BAT_TEMP_NORMAL;
				} else {
					charging = false;
					goto stop_charging;
				}
			}
		}

		if (temperature >= thermal->max_charge_temp) {
			chr_err("Battery over Temperature or NTC fail %d %d\n",
				temperature, thermal->max_charge_temp);
			thermal->sm = BAT_TEMP_HIGH;
			charging = false;
#ifdef CONFIG_HELAEYE_BSP_CHARGER_ON
			heraeye_bat_check_temp(BAT_TEMP_HIGH,temperature);
#endif
			goto stop_charging;
		} else if (thermal->sm == BAT_TEMP_HIGH) {
			if (temperature
			    < thermal->max_charge_temp_minus_x_degree) {
				chr_err("Battery Temperature raise from %d to %d(%d), allow charging!!\n",
				thermal->max_charge_temp,
				temperature,
				thermal->max_charge_temp_minus_x_degree);
				thermal->sm = BAT_TEMP_NORMAL;
			} else {
				charging = false;
				goto stop_charging;
			}
		}
	}

/* Begin del by jin.wang for task 2064 on 2021-10-5 */
#if !defined(CONFIG_TCT_CHARGER)
	mtk_chg_get_tchg(info);
#endif
/* End del by jin.wang */

	if (!mtk_chg_check_vbus(info)) {
		charging = false;
		goto stop_charging;
	}

/* Begin modified by bitao.xiong for ENCORECKT-2669 on 2022-08-31 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (gm == NULL)
		gm = get_mtk_battery();
	if (is_mmitest || is_inproductionflag) {
		fg_cust_data = &gm->fg_cust_data;
		chr_err("[minisw] soc_limited = %d\n", soc_limited);
		if (fg_cust_data->embedded_sel && soc_limited) {
			uisoc = get_uisoc(info);
			if (uisoc >= MINI_SUSPEND_CHG_SOC) {
				chr_err("uisoc up to %d(%d), stop chg!!\n",
						uisoc, MINI_SUSPEND_CHG_SOC);
				mini_soc_limited = 1;
				charging = false;
				goto stop_charging;
			} else if (mini_soc_limited) {
				if (uisoc <= MINI_RESUME_CHG_SOC) {
					chr_err("uisoc down to %d(%d), resume chg!!\n",
							uisoc, MINI_RESUME_CHG_SOC);
					mini_soc_limited = 0;
				} else {
					charging = false;
					goto stop_charging;
				}
			}
		} else {
			mini_soc_limited = 0;
		}
	}
#endif
/* End modified by bitao.xiong for ENCORECKT-2669 on 2022-08-31 */

/* Begin added by hailong.chen for task 9777034 on 2020-08-20 */
#if defined(CONFIG_TCT_CHARGER)
	if (gm->chg_disable) {
		chr_err("cmd disable charging\n");
		charging = false;
		goto stop_charging;
	}
//Begin delete by tanghsan.bai for task 10968812 on 20 2021-03-29
//Begin add by rd.linsheng.liu for default 10430693 on 2020/12/15
 //   #ifndef CONFIG_WRIGHTPRO_DEVICEINFO
	if (!strncmp(get_battery_type(info), "NA:NA:NA:NA", strlen("NA:NA:NA:NA"))) {
			chr_err("Third-party battery,stop charging\n");
			charging = false;
			goto stop_charging;
	}
 //   #endif
//End add by rd.linsheng.liu for default 10430693 on 2020/12/15
// End delete by tanghsan.bai for task 10968812 on 20 2021-03-29
#endif
/* End added by hailong.chen for task 9777034 on 2020-08-20 */

	if (info->cmd_discharging)
		charging = false;
	if (info->safety_timeout)
		charging = false;
	if (info->vbusov_stat)
		charging = false;

	#if IS_ENABLED(CONFIG_TCT_CHARGER)
	/* Begin added by bitao.xiong for task-9895401 on 2020-09-11 */
	if (IS_ERR_OR_NULL(info->bat_psy)) {
		info->bat_psy = devm_power_supply_get_by_phandle(&info->pdev->dev,
			"gauge");
		if (IS_ERR_OR_NULL(info->bat_psy)) {
			chr_err("%s: get battery power supply failed\n", __func__);
			charging = false;
		}
	} else {
		#if !defined(TARGET_BUILD_MMITEST)
		ret = power_supply_get_property(info->bat_psy,
				POWER_SUPPLY_PROP_DEBUG_BATTERY, &prop);
		if (!ret && prop.intval) {
			chr_err("%s: disable charge for 3rd battery\n", __func__);
			charging = false;
		}
		#endif
		ret = power_supply_get_property(info->bat_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
		if (!ret && !prop.intval) {
			charging = false;
		}
	}
	/* End added by bitao.xiong for task-9895401 on 2020-09-11 */
	#endif

stop_charging:
	mtk_battery_notify_check(info);

	chr_err("tmp:%d (jeita:%d sm:%d cv:%d en:%d) (sm:%d) en:%d c:%d s:%d ov:%d %d %d\n",
		temperature, info->enable_sw_jeita, info->sw_jeita.sm,
		info->sw_jeita.cv, info->sw_jeita.charging, thermal->sm,
		charging, info->cmd_discharging, info->safety_timeout,
		info->vbusov_stat, info->can_charging, charging);

	if (charging != info->can_charging)
		_mtk_enable_charging(info, charging);

	info->can_charging = charging;
	#if IS_ENABLED(CONFIG_TCT_CHARGER_GCS)
	#if !defined(TARGET_BUILD_MMITEST)
	/* [BSP]Begin deleted by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
	#if 0
	mtk_charger_check_nonstandard_charger(info, info->can_charging);
	#endif
	/* [BSP]End deleted by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
	#endif
	#endif
}
#endif

static bool charger_init_algo(struct mtk_charger *info)
{
/* Begin mod by jin.wang task 2064 on 2021.10.25 */
#if IS_ENABLED(CONFIG_MTK_PUMP_EXPRESS_40_SUPPORT) \
	|| IS_ENABLED(CONFIG_MTK_PD_SUPPORT) \
	|| IS_ENABLED(CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT) \
	|| IS_ENABLED(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	struct chg_alg_device * alg = NULL;
	int idx = 0;
#endif
/* End mod by jin.wang */

/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev) {
/* [BSP]Begin added by bitao.xiong for SNTTF-5676 on 2022/11/29 */
#if defined(TARGET_BUILD_MMITEST)
		info->enable_sw_safety_timer = false;
		charger_dev_enable_safety_timer(info->chg1_dev, false);
#else
		charger_dev_enable_safety_timer(info->chg1_dev, true);
#endif
/* [BSP]End added by bitao.xiong for SNTTF-5676 on 2022/11/29 */
		chr_err("Found primary charger\n");
	} else {
		chr_err("*** Error : can't find primary charger ***\n");
		return false;
	}

	chr_err("[%s], config is %d\n", __func__, info->config);
	if ((info->config == DUAL_CHARGERS_IN_PARALLEL)
		|| (info->config == DUAL_CHARGERS_IN_SERIES)) {
		info->chg2_dev = get_charger_by_name("secondary_chg");
		if (info->chg2_dev) {
/* [BSP]Begin added by bitao.xiong for SNTTF-5676 on 2022/11/29 */
#if defined(TARGET_BUILD_MMITEST)
			charger_dev_enable_safety_timer(info->chg2_dev, false);
#else
			charger_dev_enable_safety_timer(info->chg2_dev, true);
#endif
/* [BSP]End added by bitao.xiong for SNTTF-5676 on 2022/11/29 */
			chr_err("Found secondary charger\n");
		} else {
			chr_err("*** Warning : can't find secondary charger ***\n");
			info->config = SINGLE_CHARGER;
		}
	}
#endif

#if IS_ENABLED(CONFIG_MTK_PUMP_EXPRESS_40_SUPPORT)
	alg = get_chg_alg_by_name("pe4");
	info->alg[idx] = alg;
	if (alg == NULL)
		chr_err("get pe4 fail\n");
	else {
		chr_err("get pe4 success\n");
		alg->config = info->config;
		chg_alg_init_algo(alg);
		register_chg_alg_notifier(alg, &info->chg_alg_nb);
	}
	idx++;
#endif

#if IS_ENABLED(CONFIG_MTK_PD_SUPPORT)
	alg = get_chg_alg_by_name("pd");
	info->alg[idx] = alg;
	if (alg == NULL)
		chr_err("get pd fail\n");
	else {
		chr_err("get pd success\n");
		alg->config = info->config;
		chg_alg_init_algo(alg);
		register_chg_alg_notifier(alg, &info->chg_alg_nb);
	}
	idx++;
#endif

#if IS_ENABLED(CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT)
	alg = get_chg_alg_by_name("pe2");
	info->alg[idx] = alg;
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (alg == NULL) {
		chr_err("Error: get pe2 fail, wait\n");
		return false;
	}
#else
	if (alg == NULL)
		chr_err("get pe2 fail\n");
#endif
	else {
		chr_err("get pe2 success\n");
		alg->config = info->config;
		chg_alg_init_algo(alg);
		register_chg_alg_notifier(alg, &info->chg_alg_nb);
	}
	idx++;
#endif

#if IS_ENABLED(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	alg = get_chg_alg_by_name("pe");
	info->alg[idx] = alg;
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (alg == NULL) {
		chr_err("Error: get pe fail, delay\n");
		return false;
	}
#else
	if (alg == NULL)
		chr_err("get pe fail\n");
#endif
	else {
		chr_err("get pe success\n");
		alg->config = info->config;
		chg_alg_init_algo(alg);
		register_chg_alg_notifier(alg, &info->chg_alg_nb);
	}
#endif
/* End mod by jin.wang */

/* Begin del by jin.wang task 2064 on 2021.10.8 */
#if !IS_ENABLED(CONFIG_TCT_CHARGER)
	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev)
		chr_err("Found primary charger\n");
	else {
		chr_err("*** Error : can't find primary charger ***\n");
		return false;
	}

/* Begin modified by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
	chr_err("[%s], config is %d\n", __func__, info->config);
	if (info->config == DUAL_CHARGERS_IN_SERIES) {
		info->chg2_dev = get_charger_by_name("secondary_chg");
		if (info->chg2_dev)
			chr_err("Found secondary charger\n");
		else {
			chr_err("*** Error : can't find secondary charger ***\n");
			return false;
		}
	}
#endif
/* End del by jin.wang */

	chr_err("register chg1 notifier %d %d\n",
		info->chg1_dev != NULL, info->algo.do_event != NULL);
	if (info->chg1_dev != NULL && info->algo.do_event != NULL) {
		chr_err("register chg1 notifier done\n");
		info->chg1_nb.notifier_call = info->algo.do_event;
		register_charger_device_notifier(info->chg1_dev,
						&info->chg1_nb);
		charger_dev_set_drvdata(info->chg1_dev, info);
	}

	return true;
}

static int mtk_charger_plug_out(struct mtk_charger *info)
{
	struct charger_data *pdata1 = &info->chg_data[CHG1_SETTING];
	struct charger_data *pdata2 = &info->chg_data[CHG2_SETTING];
	struct chg_alg_device *alg;
	struct chg_alg_notify notify;
	int i;

	chr_err("%s\n", __func__);
	info->chr_type = POWER_SUPPLY_TYPE_UNKNOWN;
/* Begin added by dapeng.qiao for task 11038299 on 2021-05-1 */
#ifdef TCT_BMS_SW_SUPPORT
    g_chr_type = POWER_SUPPLY_TYPE_UNKNOWN;
#endif
/* End added by dapeng.qiao for task 11038299 on 2021-05-1 */
	info->charger_thread_polling = false;
/* Begin added by geng.sun for PR SNTTF-4846 on 2022-09-22 */
	info->pd_reset = false;
/* End added by geng.sun for PR SNTTF-4846 on 2022-09-22 */
	pdata1->disable_charging_count = 0;
	pdata2->disable_charging_count = 0;

/* Begin add by jin.wang for jira-2064 on 2021-11-25 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	pdata1->ibus_limit_by_others = -1;
	pdata1->ibat_limit_by_others = -1;
#endif
/* End add by jin.wang */

	notify.evt = EVT_PLUG_OUT;
	notify.value = 0;
	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		chg_alg_notifier_call(alg, &notify);
	}

	charger_dev_set_input_current(info->chg1_dev, 100000);
	charger_dev_set_mivr(info->chg1_dev, info->data.min_charger_voltage);
	charger_dev_plug_out(info->chg1_dev);

/* Begin mod by jin.wang for androidT on 2022-4-12 */
#if IS_ENABLED(CONFIG_TCT_CHARGER_GCS)
	info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;
	info->input_avg_current = 0;
#endif
/* End mod by jin.wang */

/* Begin add by jin.wang for jira on 2021-10-25 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	set_hv_flag(0);
#endif
/* End add by jin.wang */

/* Begin Added by tangshan.bai for LEVIN-6148 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
    chr_err("peak managment function CABLE_PLUG_OUT\n");
    calculate_coulomb_charged(EVT_PLUG_OUT);//CABLE_PLUG_out
    gm->bat_state = BAT_DISCHARGING;
#endif
#endif
/* End Added by tangshan.bai for LEVIN-6148 */
	return 0;
}

static int mtk_charger_plug_in(struct mtk_charger *info,
				int chr_type)
{
	struct chg_alg_device *alg;
	struct chg_alg_notify notify;
	/* [BSP]Begin added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
	#if IS_ENABLED(CONFIG_TCT_CHARGER)
	struct charger_data *pdata1 = &info->chg_data[CHG1_SETTING];
	static struct power_supply *chg_psy;
	#endif
	/* [BSP]End added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
	int i;

	chr_debug("%s\n",
		__func__);
/*Begin: modify for BORATF-6962 by wanglin.chen on 2023.07.06*/
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (info->chr_type == POWER_SUPPLY_TYPE_UNKNOWN) {
		if (chg_psy == NULL) {
			chg_psy = power_supply_get_by_name("mtk_charger_type");
		}
		charger_dev_set_input_current(info->chg1_dev, 500000);
		charger_dev_set_charging_current(info->chg1_dev, 500000);
		charger_dev_enable(info->chg1_dev, true);
		power_supply_changed(chg_psy);
	}
#endif
/*End: modify for BORATF-6962 by wanglin.chen on 2023.07.06*/

	info->chr_type = chr_type;
/* Begin added by dapeng.qiao for task 11038299 on 2021-05-1 */
#ifdef TCT_BMS_SW_SUPPORT
    g_chr_type = info->chr_type;
#endif
/* End added by dapeng.qiao for task 11038299 on 2021-05-1 */
	info->charger_thread_polling = true;

	info->can_charging = true;
	//info->enable_dynamic_cv = true;
	info->safety_timeout = false;
	info->vbusov_stat = false;
/* [BSP]Begin added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	info->data.sw_aicl_done = false; 
	pdata1->input_current_limit_by_aicl = -1;
#endif
/* [BSP]End added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
/* Begin add by jin.wang task 2064 on 2021.11.30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	info->first_redet = true;
#endif
/* End add by jin.wang */

/* Begin added by jin.wang for task 11700191 on 2022-1-25 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
if (is_mmitest || is_inproductionflag)
	mini_soc_limited = 0;
#endif
/* End add by jin.wang */

	chr_err("mtk_is_charger_on plug in, type:%d\n", chr_type);

	notify.evt = EVT_PLUG_IN;
	notify.value = 0;
	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		chg_alg_notifier_call(alg, &notify);
	}

	charger_dev_plug_in(info->chg1_dev);
	get_monotonic_boottime(&info->charging_begin_time);

/* Begin mod by jin.wang for androidT on 2022-4-12 */
#if IS_ENABLED(CONFIG_TCT_CHARGER_GCS)
#if IS_ENABLED(TARGET_BUILD_CERTIFICATION) || IS_ENABLED(TARGET_BUILD_MMITEST) \
		|| IS_ENABLED(DISABLE_TEMPERATURE_DETECTION_AND_THERMAL_POLICY)
	info->nonstand_chg_type = NONSTANDARD_CHARGER_NONE;
#else
	info->nonstand_chg_type = NONSTANDARD_CHARGER_INVALIED;
#endif
#endif
/* End mod by jin.wang */

/* [BSP]Begin modified by bitao.xiong for SNTTF-635 on 2022/10/29 */
#if !IS_ENABLED(CONFIG_TCT_PROJECT_SONATA)
	// add begin by TCT-cuiping.shi
	charger_dev_set_eoc_current(info->chg1_dev, 200000);
	charger_dev_enable_termination(info->chg1_dev, true);
	// add end by TCT-cuiping.shi
#endif
/* [BSP]End modified by bitao.xiong for SNTTF-635 on 2022/10/29 */

/* Begin Added by tangshan.bai for LEVIN-6148 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
    chr_err("peak managment function CABLE_PLUG_IN\n");
    calculate_coulomb_charged(EVT_PLUG_IN);//CABLE_PLUG_IN
    gm->bat_state = BAT_CHARGING;
	get_monotonic_boottime(&gm->old_time);
#endif
#endif
/* End Added by tangshan.bai for LEVIN-6148 */
	return 0;
}


static bool mtk_is_charger_on(struct mtk_charger *info)
{
	int chr_type;

// 0713 find a bug 
#ifdef CONFIG_HELAEYE_BSP_CHARGER_ON

//  int vbus_value=get_vbus(info);
 
//  if(vbus_value<4000)
//	{
//		 heraeye_bat_check_temp(CHARGER_DRV_VBUS_ERROR,vbus_value);
// }

#endif


	chr_type = get_charger_type(info);
	if (chr_type == POWER_SUPPLY_TYPE_UNKNOWN) {
		if (info->chr_type != POWER_SUPPLY_TYPE_UNKNOWN) {
			mtk_charger_plug_out(info);
			mutex_lock(&info->cable_out_lock);
			info->cable_out_cnt = 0;
			mutex_unlock(&info->cable_out_lock);
		}
	} else {
		if (info->chr_type == POWER_SUPPLY_TYPE_UNKNOWN)
			mtk_charger_plug_in(info, chr_type);
		else
			info->chr_type = chr_type;

		if (info->cable_out_cnt > 0) {
			mtk_charger_plug_out(info);
			mtk_charger_plug_in(info, chr_type);
			mutex_lock(&info->cable_out_lock);
			info->cable_out_cnt = 0;
			mutex_unlock(&info->cable_out_lock);
		}
	}

	if (chr_type == POWER_SUPPLY_TYPE_UNKNOWN)
		return false;

	return true;
}

static void kpoc_power_off_check(struct mtk_charger *info)
{
	unsigned int boot_mode = info->bootmode;
	int vbus = 0;
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	int vbus_absent_cnt = 0;
	bool vbus_online = true;
#endif
	/* 8 = KERNEL_POWER_OFF_CHARGING_BOOT */
	/* 9 = LOW_POWER_OFF_CHARGING_BOOT */
	if (boot_mode == 8 || boot_mode == 9) {	
	#if IS_ENABLED(CONFIG_TCT_CHARGER)
	/*Begin: modify for ENCORETF-10841 by wanglin.chen on 2023.05.10*/
		#if 0
		vbus = get_vbus(info);
		if (vbus >= 0 && vbus < 2500 && !mtk_is_charger_on(info) && !info->pd_reset) 
		#else
		do {
			vbus = get_vbus(info);
			if (vbus >= 0 && vbus < 2500 && !mtk_is_charger_on(info) && !info->pd_reset) {
				msleep(100);
				vbus_absent_cnt++;
				vbus_online = false;
			} else {
				vbus_online = true;
				break;
			}
		} while(vbus_absent_cnt < 3);
		if (vbus_online == false)
		#endif
	/*End: modify for ENCORETF-10841 by wanglin.chen on 2023.05.10*/
	#endif
		{
			chr_err("Unplug Charger/USB in KPOC mode, vbus=%d, shutdown\n", vbus);
			kernel_power_off();
		}
	}
}

// Begin modified by zhangkun for MODEL3-4826 on 2022-11-21
#ifdef CONFIG_TCT_PROJECT_MODEL_3
static void model3_kpoc_power_off_check(struct mtk_charger *info)
{
	unsigned int boot_mode = info->bootmode;
	int vbus = 0;
        int i=0;
	/* 8 = KERNEL_POWER_OFF_CHARGING_BOOT */
	/* 9 = LOW_POWER_OFF_CHARGING_BOOT */
	if (boot_mode == 8 || boot_mode == 9) {
		vbus = get_vbus(info);
		if (vbus >= 0 && vbus < 2500 && !mtk_is_charger_on(info) && !info->pd_reset) {
			chr_err("Unplug Charger/USB in KPOC mode, vbus=%d, shutdown\n", vbus);
				for (i=0;i<3;i++){
					mdelay(1000);
					if (vbus >= 0 && vbus < 2500 && !mtk_is_charger_on(info) && !info->pd_reset){
                                                printk("model3_kpoc_power_off_check=%d",i);
		                                if(i==2)
						kernel_power_off();
					}
                                        else{
                                                printk("model3_kpoc_power_off_check_NO=%d",i);
						break;
                                        }
                                }
                        }
		}
}
#endif
// End modified by zhangkun for MODEL3-4826 on 2022-11-21

static char *dump_charger_type(int type)
{
	switch (type) {
	case POWER_SUPPLY_TYPE_UNKNOWN:
		return "none";
	case POWER_SUPPLY_TYPE_USB:
		return "usb";
	case POWER_SUPPLY_TYPE_USB_CDP:
		return "usb-h";
	case POWER_SUPPLY_TYPE_USB_DCP:
		return "std";
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		return "nonstd";
	default:
		return "unknown";
	}
}

/* Begin mod by jin.wang task 2064 on 2021.11.30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
extern void tct_start_det_charger(int delay_ms);
extern void tct_stop_det_charger(void);
#else
/* Begin added by bin.song.hz for task 10431118 on 2020-12-07 */
extern void tct_detect_charger(void);
/* End added by bin.song.hz for task 10431118 on 2020-12-07 */
#endif
/* End mod by jin.wang */

/* [BSP]Begin added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
void sw_dev_run_aicl(struct mtk_charger *info, int *sw_aicl)
{
	struct charger_data *pdata1 = &info->chg_data[CHG1_SETTING];
	/*note: do not modify vchr_min modify by wanglin.chen on 2022.12.15*/
	const int vchr_min = 4500000;//4700000;
	int aicl[]={500000, 600000, 700000, 800000, 900000,
		1000000, 1100000, 1200000, 1300000, 1400000,
		1500000, 1600000, 1700000, 1800000, 1900000, 2000000, 2100000};
	/*note: do not modify vchr_min modify by wanglin.chen on 2022.12.15*/
	int step_max = sizeof(aicl)/sizeof(aicl[0]);
	int step = 0;
	int vchr = 0;
	int bat_temp = 0;

	if(info->can_charging == false) {
		*sw_aicl = -1;
		return;
	}

	if (pdata1->input_current_limit_by_aicl < 0) {
		charger_dev_enable(info->chg1_dev, true);
		do {
			charger_dev_set_input_current(info->chg1_dev, aicl[step]);
			charger_dev_set_charging_current(info->chg1_dev, aicl[step]);
			vchr = get_vbus(info) * 1000; /* uV */
			chr_err("[cwldebug %s] step=%d vchr=%d\n", __func__, step, vchr);
			if(vchr >= vchr_min)
				step++;
			else
				break;
			msleep(100);
		} while(step < step_max);
		step--;
		if (step < 0) {
			*sw_aicl = -1;
		} else {
			*sw_aicl = aicl[step];
		}
	}
}

static void sw_check_aicl(struct mtk_charger *info)
{
	struct charger_data *pdata1 = &info->chg_data[CHG1_SETTING];
	int hw_aicl = -1, sw_aicl = -1;

	/* AICL */
	if (info->chr_type == POWER_SUPPLY_TYPE_USB_DCP) {
		if (get_uisoc(info) < 80 && !info->data.sw_aicl_done) {
			charger_dev_run_aicl(info->chg1_dev, &hw_aicl);

			if (hw_aicl > 0) {
				pdata1->input_current_limit_by_aicl = hw_aicl;
			} else {
		/*Begin: add dts contrl sw_aicl by wanglin.chen on 2023.02.10*/
				if ((info->enable_sw_aicl) && (!info->data.sw_aicl_done)) {
					sw_dev_run_aicl(info, &sw_aicl);
					if (sw_aicl > 0) {
						pdata1->input_current_limit_by_aicl = sw_aicl;
						info->data.sw_aicl_done = true;
					}
				}
			}
		/*End: add dts contrl sw_aicl  by wanglin.chen on 2023.02.10*/
			chr_err("[cwldebug %s] sw_aicl=%d hw_aicl=%d\n", __func__, sw_aicl, hw_aicl);
		}
	}
}
#endif
/* [BSP]End added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */

static int charger_routine_thread(void *arg)
{
	struct mtk_charger *info = arg;
	unsigned long flags;
	static bool is_module_init_done;
	bool is_charger_on;

/* Begin mod by jin.wang task 2064 on 2021.10.25 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	int is_basic = 1;
	static int last_basic = 1;
#endif
/* End mod by jin.wang */

/* Begin add by jin.wang ALM 11700191 on 2022.1.13 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH) \
	&& IS_ENABLED(TARGET_BUILD_MMITEST)
	struct power_supply *psy = NULL;
#endif
/* End add by jin.wang */

	while (1) {
		wait_event(info->wait_que,
			(info->charger_thread_timeout == true));

		while (is_module_init_done == false) {
			if (charger_init_algo(info) == true)
				is_module_init_done = true;
			else {
				chr_err("charger_init fail\n");
				msleep(5000);
			}
		}

/* Begin add by jin.wang for androidT on 2022.4.12 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		pr_err("%s: v4 starting\n", __func__);
#endif
/* End add by jin.wang */

		mutex_lock(&info->charger_lock);
		spin_lock_irqsave(&info->slock, flags);
		if (!info->charger_wakelock->active)
			__pm_stay_awake(info->charger_wakelock);
		spin_unlock_irqrestore(&info->slock, flags);
		info->charger_thread_timeout = false;

		info->battery_temp = get_battery_temperature(info);
		chr_err("Vbat=%d vbus:%d ibus:%d I=%d T=%d uisoc:%d type:%s>%s pd:%d\n",
			get_battery_voltage(info),
			get_vbus(info),
			get_ibus(info),
			get_battery_current(info),
			info->battery_temp,
			get_uisoc(info),
			dump_charger_type(info->chr_type),
			dump_charger_type(get_charger_type(info)),
			info->pd_type);

		is_charger_on = mtk_is_charger_on(info);

/* Begin mod by jin.wang task 2064 on 2021.11.30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		if (is_charger_on) {
			if (get_vbus(info) < 2000) {
				chr_err("vbus is too low, retry det!\n");
				tct_start_det_charger(0);
			} else if ((info->chr_type == POWER_SUPPLY_TYPE_USB_FLOAT)
						&& info->first_redet) {
				chr_err("float type, retry det!\n");
				info->first_redet = false;
				tct_start_det_charger(5000);
			}
		} else {
			tct_stop_det_charger();
		}
#else
        /* Begin added by tangshan for CIVICS-3955 (2V->4V) 20230208 */
        /* Begin added by bin.song.hz for task 10431118 on 2020-12-07 */
        if(get_vbus(info) < 4000 && is_charger_on)
        {
            chr_err("is_charger_on = %d, vbus is too low(%d)!\n", is_charger_on, get_vbus(info));
            tct_detect_charger();
        }
        /* End added by bin.song.hz for task 10431118 on 2020-12-07 */
        /* End added by tangshan for CIVICS-3955 (2V->4V) 20230208 */
#endif
/* End mod by jin.wang */

		if (info->charger_thread_polling == true)
			mtk_charger_start_timer(info);

/* Begin modified by hailong.chen for task 9777034 on 2020-08-20 */
#if !IS_ENABLED(TARGET_BUILD_MMITEST)
		check_battery_exist(info);
#endif
/* End modified by hailong.chen for task 9777034 on 2020-08-20 */

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		check_dynamic_mivr(info);
#endif
/* End del by jin.wang */
		charger_check_status(info);
/* [BSP]Begin added by wanglin.chen for SNTBBH-4343 on 2023/01/04 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		sw_check_aicl(info);
#endif
/* [BSP]End added by wanglin.chen for SNTBBH-4343 on 2022/01/04 */
/* Begin Added by tangshan.bai for LEVIN-6148 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
    	record_fg_maxvalue_batteryverify();
//#ifdef TCT_BMS_SW_SUPPORT
        cal_batt_life_discharge_or_charging();
        if (gm == NULL)
			gm = get_mtk_battery();
		if(gm->peak_enforce_full && gm->display_soc < 10000)
			uisoc_tracking_to_full_work();
//#endif
#endif
#endif
/* End Added by tangshan.bai for LEVIN-6148 */
// Begin modified by zhangkun for MODEL3-4826 on 2022-11-21
#ifdef CONFIG_TCT_PROJECT_MODEL_3
		model3_kpoc_power_off_check(info);
#else
		kpoc_power_off_check(info);
#endif
// Eed modified by zhangkun for MODEL3-4826 on 2022-11-21
/* Begin mod by jin.wang task 2064 on 2021.10.25 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		is_basic = 1;
		if (is_disable_charger(info) == false &&
			is_charger_on == true &&
			info->can_charging == true) {
			if (info->algo.do_algorithm)
				is_basic = info->algo.do_algorithm(info);
		} else {
			chr_debug("disable charging %d %d %d\n",
					is_disable_charger(info),
					is_charger_on,
					info->can_charging);
		}
		if (last_basic != is_basic) {
			set_hv_flag(!is_basic);
			last_basic = is_basic;
		}
#else
		if (is_disable_charger(info) == false &&
			is_charger_on == true &&
			info->can_charging == true) {
			if (info->algo.do_algorithm)
				info->algo.do_algorithm(info);
		} else
			chr_debug("disable charging %d %d %d\n",
			is_disable_charger(info),
			is_charger_on,
			info->can_charging);
#endif
/* End mod by jin.wang */

		spin_lock_irqsave(&info->slock, flags);
		__pm_relax(info->charger_wakelock);
		spin_unlock_irqrestore(&info->slock, flags);

/*Add-start by weijun for task JTVZ-7583 on 2022/02/23*/
#if defined(CONFIG_TCT_CHG_JETTA_VZW)
    if (is_charger_on == true)
       power_supply_changed(info->bat_psy);
#endif
/*End added by weijun for task JTVZ-7583 on 2022/02/23*/

/* Begin add by jin.wang task 2064 on 2021.11.23 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		if (info->chg1_dev != NULL)
			charger_dev_dump_registers(info->chg1_dev);
		if (info->chg2_dev != NULL)
			charger_dev_dump_registers(info->chg2_dev);
#endif
/* End add by jin.wang */

		chr_debug("%s end , %d\n",
			__func__, info->charger_thread_timeout);
		mutex_unlock(&info->charger_lock);

/* Begin add by jin.wang for androidT on 2022.4.12 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH) \
	&& IS_ENABLED(TARGET_BUILD_MMITEST)
		if (IS_ERR_OR_NULL(psy)) {
			psy = power_supply_get_by_name("battery");
		}
		if (!IS_ERR_OR_NULL(psy))
			power_supply_changed(psy);
#endif
/* End add by jin.wang */
	}

	return 0;
}


#ifdef CONFIG_PM
static int charger_pm_event(struct notifier_block *notifier,
			unsigned long pm_event, void *unused)
{
	struct timespec now;
	struct mtk_charger *info;

	info = container_of(notifier,
		struct mtk_charger, pm_notifier);

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		info->is_suspend = true;
		chr_debug("%s: enter PM_SUSPEND_PREPARE\n", __func__);
		break;
	case PM_POST_SUSPEND:
		info->is_suspend = false;
		chr_debug("%s: enter PM_POST_SUSPEND\n", __func__);
		get_monotonic_boottime(&now);

		if (timespec_compare(&now, &info->endtime) >= 0 &&
			info->endtime.tv_sec != 0 &&
			info->endtime.tv_nsec != 0) {
			chr_err("%s: alarm timeout, wake up charger\n",
				__func__);
			__pm_relax(info->charger_wakelock);
			info->endtime.tv_sec = 0;
			info->endtime.tv_nsec = 0;
			_wake_up_charger(info);
		}
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}
#endif /* CONFIG_PM */

static enum alarmtimer_restart
	mtk_charger_alarm_timer_func(struct alarm *alarm, ktime_t now)
{
	struct mtk_charger *info =
	container_of(alarm, struct mtk_charger, charger_timer);

	if (info->is_suspend == false) {
		chr_err("%s: not suspend, wake up charger\n", __func__);
		_wake_up_charger(info);
	} else {
		chr_err("%s: alarm timer timeout\n", __func__);
		__pm_stay_awake(info->charger_wakelock);
	}

	return ALARMTIMER_NORESTART;
}

static void mtk_charger_init_timer(struct mtk_charger *info)
{
	alarm_init(&info->charger_timer, ALARM_BOOTTIME,
			mtk_charger_alarm_timer_func);
	mtk_charger_start_timer(info);

#ifdef CONFIG_PM
	if (register_pm_notifier(&info->pm_notifier))
		chr_err("%s: register pm failed\n", __func__);
#endif /* CONFIG_PM */
}

static int mtk_charger_setup_files(struct platform_device *pdev)
{
	int ret = 0;
	struct proc_dir_entry *battery_dir = NULL, *entry = NULL;
	struct mtk_charger *info = platform_get_drvdata(pdev);

/* Begin add by jin.wang for jira on 2021-11-5 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	ret = device_create_file(&(pdev->dev), &dev_attr_thermal_disable);
	if (ret)
		goto _out;
#endif
/* End add by jin.wang */

	ret = device_create_file(&(pdev->dev), &dev_attr_sw_jeita);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_chr_type);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_Pump_Express);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_ADC_Charger_Voltage);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_Charger_Config);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_input_current);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_charger_log_level);
	if (ret)
		goto _out;

	/* Battery warning */
	ret = device_create_file(&(pdev->dev), &dev_attr_BatteryNotify);
	if (ret)
		goto _out;

/* Begin mod by jin.wang for androidT on 2022-4-12 */
#if IS_ENABLED(CONFIG_TCT_CHARGER_GCS)
#if !IS_ENABLED(TARGET_BUILD_MMITEST)
	ret = device_create_file(&(pdev->dev), &dev_attr_nonstand_charge_type);
	if (ret)
		goto _out;
#endif
#endif
/* End mod by jin.wang */

/* Begin added by bin.song.hz for defect10297066 on 2020.12.03 */
	ret = device_create_file(&(pdev->dev), &dev_attr_BatJeitaStatus);
	if (ret)
		goto _out;
/* End added by bin.song.hz for defect10297066 on 2020.12.03 */

/* Begin add by jin.wang for jira 11700191 on 2022-1-17 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (is_mmitest || is_inproductionflag){
		ret = device_create_file(&(pdev->dev), &dev_attr_soc_limited);
		if (ret)
			goto _out;
	}
#endif
/* End add by jin.wang */

	battery_dir = proc_mkdir("mtk_battery_cmd", NULL);
	if (!battery_dir) {
		chr_err("%s: mkdir /proc/mtk_battery_cmd failed\n", __func__);
		return -ENOMEM;
	}

	entry = proc_create_data("current_cmd", 0644, battery_dir,
			&mtk_chg_current_cmd_fops, info);
	if (!entry) {
		ret = -ENODEV;
		goto fail_procfs;
	}
	entry = proc_create_data("en_power_path", 0644, battery_dir,
			&mtk_chg_en_power_path_fops, info);
	if (!entry) {
		ret = -ENODEV;
		goto fail_procfs;
	}
	entry = proc_create_data("en_safety_timer", 0644, battery_dir,
			&mtk_chg_en_safety_timer_fops, info);
	if (!entry) {
		ret = -ENODEV;
		goto fail_procfs;
	}

	return 0;

fail_procfs:
	remove_proc_subtree("mtk_battery_cmd", NULL);
_out:
	return ret;
}

void mtk_charger_get_atm_mode(struct mtk_charger *info)
{
	char atm_str[64] = {0};
	char *ptr = NULL, *ptr_e = NULL;
	char keyword[] = "androidboot.atm=";
	int size = 0;

	info->atm_enabled = false;

	ptr = strstr(saved_command_line, keyword);
	if (ptr != 0) {
		ptr_e = strstr(ptr, " ");
		if (ptr_e == 0)
			goto end;

		size = ptr_e - (ptr + strlen(keyword));
		if (size <= 0)
			goto end;
		strncpy(atm_str, ptr + strlen(keyword), size);
		atm_str[size] = '\0';

		if (!strncmp(atm_str, "enable", strlen("enable")))
			info->atm_enabled = true;
	}
end:
	chr_err("%s: atm_enabled = %d\n", __func__, info->atm_enabled);
}

static int psy_charger_property_is_writeable(struct power_supply *psy,
					       enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		return 1;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return 1;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
/* Begin add by jin.wang for jira on 2021-11-5 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	case POWER_SUPPLY_PROP_OTHERS_IBUS_LIMIT:
	case POWER_SUPPLY_PROP_OTHERS_IBAT_LIMIT:
		return 1;
#endif
/* End add by jin.wang */
	default:
		return 0;
	}
}

static enum power_supply_property charger_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
/* Begin add by jin.wang for jira on 2021-11-5 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	POWER_SUPPLY_PROP_OTHERS_IBUS_LIMIT,
	POWER_SUPPLY_PROP_OTHERS_IBAT_LIMIT,
#endif
/* End add by jin.wang */
};

static int psy_charger_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mtk_charger *info;
	struct charger_device *chg;

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);

	chr_err("%s psp:%d\n",
		__func__, psp);


	if (info->psy1 != NULL &&
		info->psy1 == psy)
		chg = info->chg1_dev;
	else if (info->psy2 != NULL &&
		info->psy2 == psy)
		chg = info->chg2_dev;
	else {
		chr_err("%s fail\n", __func__);
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = is_charger_exist(info);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (chg != NULL)
			val->intval = true;
		else
			val->intval = false;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = info->enable_hv_charging;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_vbus(info);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (chg == info->chg1_dev)
			val->intval =
				info->chg_data[CHG1_SETTING].junction_temp_max;
		else if (chg == info->chg2_dev)
			val->intval =
				info->chg_data[CHG2_SETTING].junction_temp_max;
		else
			val->intval = -127;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = get_charger_charging_current(info, chg);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = get_charger_input_current(info, chg);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = info->chr_type;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_BOOT:
		val->intval = get_charger_zcv(info, chg);
		break;
/* Begin add by jin.wang for jira on 2021-11-5 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	case POWER_SUPPLY_PROP_OTHERS_IBUS_LIMIT:
		if (chg == info->chg1_dev)
			val->intval =
				info->chg_data[CHG1_SETTING].ibus_limit_by_others;
		else if (chg == info->chg2_dev)
			val->intval =
				info->chg_data[CHG2_SETTING].ibus_limit_by_others;
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_OTHERS_IBAT_LIMIT:
		if (chg == info->chg1_dev)
			val->intval =
				info->chg_data[CHG1_SETTING].ibat_limit_by_others;
		else if (chg == info->chg2_dev)
			val->intval =
				info->chg_data[CHG2_SETTING].ibat_limit_by_others;
		else
			return -EINVAL;
		break;
#endif
/* End add by jin.wang */
	default:
		return -EINVAL;
	}

	return 0;
}

int psy_charger_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	struct mtk_charger *info;
	int idx;

/* Begin del by jin.wang for jira on 2021-11-5 */
#if !IS_ENABLED(CONFIG_TCT_CHARGER)
	chr_err("%s: prop:%d %d\n", __func__, psp, val->intval);
#endif
/* End del by jin.wang */

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);

	if (info->psy1 != NULL &&
		info->psy1 == psy)
		idx = CHG1_SETTING;
	else if (info->psy2 != NULL &&
		info->psy2 == psy)
		idx = CHG2_SETTING;
	else {
		chr_err("%s fail\n", __func__);
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
/* Begin mod by jin.wang for jira on 2021-11-30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		if (info->thermal_disable) {
			pr_err("%s: skip thermal vbus limit:%d\n",
				__func__, val->intval);
		} else {
			if (val->intval > 0)
				info->enable_hv_charging = true;
			else
				info->enable_hv_charging = false;

			pr_err("%s: thermal vbus limit:%d\n",
					__func__, val->intval);
		}
#else
		if (val->intval > 0)
			info->enable_hv_charging = true;
		else
			info->enable_hv_charging = false;
#endif
/* End mod by jin.wang */
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
/* Begin mod by jin.wang for jira on 2021-11-30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		mutex_lock(&info->charger_lock);
		if (info->thermal_disable) {
			pr_err("%s: skip thermal ibat limit [%d]:%d\n",
					__func__, idx, val->intval);
			info->chg_data[idx].thermal_charging_current_limit = -1;
		} else {
			info->chg_data[idx].thermal_charging_current_limit =
					val->intval;
			pr_err("%s: thermal ibat limit [%d]:%d\n",
					__func__, idx, val->intval);
		}
		mutex_unlock(&info->charger_lock);
#else
		info->chg_data[idx].thermal_charging_current_limit =
			val->intval;
#endif
/* End mod by jin.wang */
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
/* Begin mod by jin.wang for jira on 2021-11-30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		mutex_lock(&info->charger_lock);
		if (info->thermal_disable) {
			pr_err("%s: skip thermal ibus limit [%d]:%d\n",
					__func__, idx, val->intval);
			info->chg_data[idx].thermal_input_current_limit = -1;
		} else {
			info->chg_data[idx].thermal_input_current_limit =
					val->intval;
			pr_err("%s: thermal ibus limit [%d]:%d\n",
					__func__, idx, val->intval);
		}
		mutex_unlock(&info->charger_lock);
#else
		info->chg_data[idx].thermal_input_current_limit =
			val->intval;
#endif
/* End mod by jin.wang */
		break;
/* Begin add by jin.wang for jira on 2021-11-25 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	case POWER_SUPPLY_PROP_OTHERS_IBUS_LIMIT:
		mutex_lock(&info->charger_lock);
		info->chg_data[idx].ibus_limit_by_others =
				val->intval;
		chr_err("%s: others ibus limit [%d]:%d\n",
				__func__, idx, val->intval);
		mutex_unlock(&info->charger_lock);
		break;
	case POWER_SUPPLY_PROP_OTHERS_IBAT_LIMIT:
		mutex_lock(&info->charger_lock);
		info->chg_data[idx].ibat_limit_by_others =
				val->intval;
		chr_err("%s: others ibat limit [%d]:%d\n",
				__func__, idx, val->intval);
		mutex_unlock(&info->charger_lock);
		break;
#endif
/* End add by jin.wang */
	default:
		return -EINVAL;
	}
	_wake_up_charger(info);

	return 0;
}

static void mtk_charger_external_power_changed(struct power_supply *psy)
{
	struct mtk_charger *info;
	union power_supply_propval prop, prop2;
	struct power_supply *chg_psy = NULL;
	int ret;

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);
	chg_psy = info->chg_psy;

	if (IS_ERR_OR_NULL(chg_psy)) {
		pr_notice("%s Couldn't get chg_psy\n", __func__);
		chg_psy = devm_power_supply_get_by_phandle(&info->pdev->dev,
			"charger");
		info->chg_psy = chg_psy;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop2);
	}

	pr_notice("%s event, name:%s online:%d type:%d vbus:%d\n", __func__,
		psy->desc->name, prop.intval, prop2.intval,
		get_vbus(info));

	mtk_is_charger_on(info);
	_wake_up_charger(info);
}

int notify_adapter_event(struct notifier_block *notifier,
			unsigned long evt, void *val)
{
	struct mtk_charger *pinfo = NULL;

	chr_err("%s %d\n", __func__, evt);

	pinfo = container_of(notifier,
		struct mtk_charger, pd_nb);

	switch (evt) {
	case  MTK_PD_CONNECT_NONE:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify Detach\n");
		pinfo->pd_type = MTK_PD_CONNECT_NONE;
		mutex_unlock(&pinfo->pd_lock);
		/* reset PE40 */
		break;

	case MTK_PD_CONNECT_HARD_RESET:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify HardReset\n");
		pinfo->pd_type = MTK_PD_CONNECT_NONE;
		pinfo->pd_reset = true;
		mutex_unlock(&pinfo->pd_lock);
		_wake_up_charger(pinfo);
		/* reset PE40 */
		break;

	case MTK_PD_CONNECT_PE_READY_SNK:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify fixe voltage ready\n");
		pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK;
		mutex_unlock(&pinfo->pd_lock);
		/* PD is ready */
		break;

	case MTK_PD_CONNECT_PE_READY_SNK_PD30:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify PD30 ready\r\n");
		pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK_PD30;
		mutex_unlock(&pinfo->pd_lock);
		/* PD30 is ready */
		break;

	case MTK_PD_CONNECT_PE_READY_SNK_APDO:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify APDO Ready\n");
		pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK_APDO;
		mutex_unlock(&pinfo->pd_lock);
		/* PE40 is ready */
		_wake_up_charger(pinfo);
		break;

	case MTK_PD_CONNECT_TYPEC_ONLY_SNK:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify Type-C Ready\n");
		pinfo->pd_type = MTK_PD_CONNECT_TYPEC_ONLY_SNK;
		mutex_unlock(&pinfo->pd_lock);
		/* type C is ready */
		_wake_up_charger(pinfo);
		break;
	case MTK_TYPEC_WD_STATUS:
		chr_err("wd status = %d\n", *(bool *)val);
		pinfo->water_detected = *(bool *)val;
		if (pinfo->water_detected == true)
			pinfo->notify_code |= CHG_TYPEC_WD_STATUS;
		else
			pinfo->notify_code &= ~CHG_TYPEC_WD_STATUS;
		mtk_chgstat_notify(pinfo);
		break;
	}
	return NOTIFY_DONE;
}

int chg_alg_event(struct notifier_block *notifier,
			unsigned long event, void *data)
{
	chr_err("%s: evt:%d\n", __func__, event);

	return NOTIFY_DONE;
}


static int mtk_charger_probe(struct platform_device *pdev)
{
	struct mtk_charger *info = NULL;
	int i;
	char *name = NULL;

	chr_err("%s: starts\n", __func__);

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	platform_set_drvdata(pdev, info);
	info->pdev = pdev;

	mtk_charger_parse_dt(info, &pdev->dev);

	mutex_init(&info->cable_out_lock);
	mutex_init(&info->charger_lock);
	mutex_init(&info->pd_lock);
	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s",
		"charger suspend wakelock");
	info->charger_wakelock =
		wakeup_source_register(NULL, name);
	spin_lock_init(&info->slock);

	init_waitqueue_head(&info->wait_que);
	info->polling_interval = CHARGING_INTERVAL;
	mtk_charger_init_timer(info);
#ifdef CONFIG_PM
	info->pm_notifier.notifier_call = charger_pm_event;
#endif /* CONFIG_PM */
	srcu_init_notifier_head(&info->evt_nh);
	mtk_charger_setup_files(pdev);
	mtk_charger_get_atm_mode(info);

	for (i = 0; i < CHGS_SETTING_MAX; i++) {
		info->chg_data[i].thermal_charging_current_limit = -1;
		info->chg_data[i].thermal_input_current_limit = -1;
/* Begin added by hailong.chen for task 9785237 on 2020-10-10 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		info->chg_data[i].charging_current_limit_by_jeita = -1;
		info->chg_data[i].charging_current_limit_by_vbat = -1;
#endif
/* End added by hailong.chen for task 9785237 on 2020-10-10 */
		info->chg_data[i].input_current_limit_by_aicl = -1;
/* Begin add by jin.wang task 2064 on 2021.11.5 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		info->chg_data[i].ibus_limit_by_others = -1;
		info->chg_data[i].ibat_limit_by_others = -1;
#endif
/* End add by jin.wang */

/* Begin add by jin.wang task 2064 on 2021.11.25 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		info->chg_data[i].ibus_limit_by_test = -1;
#endif
/* End add by jin.wang */
	}

/* Begin del by jin.wang for task 11466469 on 2021-9-3 */
#if !IS_ENABLED(CONFIG_TCT_CHARGER)
	info->enable_hv_charging = true;
#endif
/* End del by jin.wang for task 11466469 on 2021-9-3 */

	info->psy_desc1.name = "mtk-master-charger";
	info->psy_desc1.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_desc1.properties = charger_psy_properties;
	info->psy_desc1.num_properties = ARRAY_SIZE(charger_psy_properties);
	info->psy_desc1.get_property = psy_charger_get_property;
	info->psy_desc1.set_property = psy_charger_set_property;
	info->psy_desc1.property_is_writeable =
			psy_charger_property_is_writeable;
	info->psy_desc1.external_power_changed =
		mtk_charger_external_power_changed;
	info->psy_cfg1.drv_data = info;
	info->psy1 = power_supply_register(&pdev->dev, &info->psy_desc1,
			&info->psy_cfg1);

	info->chg_psy = devm_power_supply_get_by_phandle(&pdev->dev,
		"charger");
	if (IS_ERR_OR_NULL(info->chg_psy))
		chr_err("%s: devm power fail to get chg_psy\n", __func__);

	info->bat_psy = devm_power_supply_get_by_phandle(&pdev->dev,
		"gauge");
	if (IS_ERR_OR_NULL(info->bat_psy))
		chr_err("%s: devm power fail to get bat_psy\n", __func__);

	if (IS_ERR(info->psy1))
		chr_err("register psy1 fail:%d\n",
			PTR_ERR(info->psy1));

	info->psy_desc2.name = "mtk-slave-charger";
	info->psy_desc2.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_desc2.properties = charger_psy_properties;
	info->psy_desc2.num_properties = ARRAY_SIZE(charger_psy_properties);
	info->psy_desc2.get_property = psy_charger_get_property;
	info->psy_desc2.set_property = psy_charger_set_property;
	info->psy_desc2.property_is_writeable =
			psy_charger_property_is_writeable;
	info->psy_cfg2.drv_data = info;
	info->psy2 = power_supply_register(&pdev->dev, &info->psy_desc2,
			&info->psy_cfg2);

	if (IS_ERR(info->psy2))
		chr_err("register psy2 fail:%d\n",
			PTR_ERR(info->psy2));

	info->log_level = CHRLOG_DEBUG_LEVEL;

	info->pd_adapter = get_adapter_by_name("pd_adapter");
	if (!info->pd_adapter)
		chr_err("%s: No pd adapter found\n");
	else {
		info->pd_nb.notifier_call = notify_adapter_event;
		register_adapter_device_notifier(info->pd_adapter,
						 &info->pd_nb);
	}

	info->chg_alg_nb.notifier_call = chg_alg_event;

/* Begin add by jin.wang for jira on 2021-10-25 */
#if defined(CONFIG_TCT_CHARGER)
	info->sw_jeita.charging = true;
	info->battery_temp = get_battery_temperature(info); // Add for init the temp before power off charge display by geng.sun
	sw_jeita_state_machine_init(info); /* Added by bitao.xiong for AOSP13TMO-4319 on 2022-08-03 */
#endif
/* End add by jin.wang */

	kthread_run(charger_routine_thread, info, "charger_thread");

	return 0;
}

static int mtk_charger_remove(struct platform_device *dev)
{
	return 0;
}

static void mtk_charger_shutdown(struct platform_device *dev)
{
	struct mtk_charger *info = platform_get_drvdata(dev);
	int i;

	for (i = 0; i < MAX_ALG_NO; i++) {
		if (info->alg[i] == NULL)
			continue;
		chg_alg_stop_algo(info->alg[i]);
	}
}

static const struct of_device_id mtk_charger_of_match[] = {
	{.compatible = "mediatek,charger",},
	{},
};

MODULE_DEVICE_TABLE(of, mtk_charger_of_match);

struct platform_device mtk_charger_device = {
	.name = "charger",
	.id = -1,
};

static struct platform_driver mtk_charger_driver = {
	.probe = mtk_charger_probe,
	.remove = mtk_charger_remove,
	.shutdown = mtk_charger_shutdown,
	.driver = {
		   .name = "charger",
		   .of_match_table = mtk_charger_of_match,
	},
};

static int __init mtk_charger_init(void)
{
	return platform_driver_register(&mtk_charger_driver);
}
late_initcall(mtk_charger_init);

static void __exit mtk_charger_exit(void)
{
	platform_driver_unregister(&mtk_charger_driver);
}
module_exit(mtk_charger_exit);


MODULE_AUTHOR("wy.chuang <wy.chuang@mediatek.com>");
MODULE_DESCRIPTION("MTK Charger Driver");
MODULE_LICENSE("GPL");
