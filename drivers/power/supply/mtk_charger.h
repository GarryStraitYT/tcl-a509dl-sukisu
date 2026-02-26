/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __MTK_CHARGER_H
#define __MTK_CHARGER_H

#include <linux/alarmtimer.h>
#include "charger_class.h"
#include "adapter_class.h"
#include "mtk_charger_algorithm_class.h"
#if IS_ENABLED(CONFIG_TCT_CHARGER)
#include <linux/notifier.h>
#endif

#define CHARGING_INTERVAL 10
#define CHARGING_FULL_INTERVAL 20

#define CHRLOG_ERROR_LEVEL	1
#define CHRLOG_INFO_LEVEL	2
#define CHRLOG_DEBUG_LEVEL	3

extern int chr_get_debug_level(void);

#define chr_err(fmt, args...)					\
do {								\
	if (chr_get_debug_level() >= CHRLOG_ERROR_LEVEL) {	\
		pr_notice(fmt, ##args);				\
	}							\
} while (0)

#define chr_info(fmt, args...)					\
do {								\
	if (chr_get_debug_level() >= CHRLOG_INFO_LEVEL) {	\
		pr_notice_ratelimited(fmt, ##args);		\
	}							\
} while (0)

#define chr_debug(fmt, args...)					\
do {								\
	if (chr_get_debug_level() >= CHRLOG_DEBUG_LEVEL) {	\
		pr_notice(fmt, ##args);				\
	}							\
} while (0)

struct mtk_charger;
#define BATTERY_CV 4350000
#define V_CHARGER_MAX 6500000 /* 6.5 V */
#define V_CHARGER_MIN 4600000 /* 4.6 V */

#define USB_CHARGER_CURRENT_SUSPEND		0 /* def CONFIG_USB_IF */
#define USB_CHARGER_CURRENT_UNCONFIGURED	70000 /* 70mA */
#define USB_CHARGER_CURRENT_CONFIGURED		500000 /* 500mA */
#define USB_CHARGER_CURRENT			500000 /* 500mA */
#define AC_CHARGER_CURRENT			2050000
#define AC_CHARGER_INPUT_CURRENT		3200000
#define NON_STD_AC_CHARGER_CURRENT		500000
#define CHARGING_HOST_CHARGER_CURRENT		650000

/* [BSP]Begin added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
#define WEAK_AC_CHARGER_INPUT_CURRENT    1500000
/* [BSP]End added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */

/* dynamic mivr */
#define V_CHARGER_MIN_1 4400000 /* 4.4 V */
#define V_CHARGER_MIN_2 4200000 /* 4.2 V */
#define MAX_DMIVR_CHARGER_CURRENT 1800000 /* 1.8 A */

/* battery warning */
#define BATTERY_NOTIFY_CASE_0001_VCHARGER
#define BATTERY_NOTIFY_CASE_0002_VBATTEMP

/* charging abnormal status */
#define CHG_VBUS_OV_STATUS	(1 << 0)
#define CHG_BAT_OT_STATUS	(1 << 1)
#define CHG_OC_STATUS		(1 << 2)
#define CHG_BAT_OV_STATUS	(1 << 3)
#define CHG_ST_TMO_STATUS	(1 << 4)
#define CHG_BAT_LT_STATUS	(1 << 5)
#define CHG_TYPEC_WD_STATUS	(1 << 6)

/* Battery Temperature Protection */
#define MIN_CHARGE_TEMP  0
#define MIN_CHARGE_TEMP_PLUS_X_DEGREE	6
#define MAX_CHARGE_TEMP  50
#define MAX_CHARGE_TEMP_MINUS_X_DEGREE	47

#define MAX_ALG_NO 10

enum bat_temp_state_enum {
	BAT_TEMP_LOW = 0,
	BAT_TEMP_NORMAL,
	BAT_TEMP_HIGH
};

enum chg_dev_notifier_events {
	EVENT_FULL,
	EVENT_RECHARGE,
	EVENT_DISCHARGE,
};

struct battery_thermal_protection_data {
	int sm;
	bool enable_min_charge_temp;
	int min_charge_temp;
	int min_charge_temp_plus_x_degree;
	int max_charge_temp;
	int max_charge_temp_minus_x_degree;
};

/* sw jeita */
#define JEITA_TEMP_ABOVE_T4_CV	4240000
#define JEITA_TEMP_T3_TO_T4_CV	4240000
#define JEITA_TEMP_T2_TO_T3_CV	4340000
#define JEITA_TEMP_T1_TO_T2_CV	4240000
#define JEITA_TEMP_T0_TO_T1_CV	4040000
#define JEITA_TEMP_BELOW_T0_CV	4040000
#define TEMP_T4_THRES  50
#define TEMP_T4_THRES_MINUS_X_DEGREE 47
#define TEMP_T3_THRES  45
#define TEMP_T3_THRES_MINUS_X_DEGREE 39
#define TEMP_T2_THRES  10
#define TEMP_T2_THRES_PLUS_X_DEGREE 16
#define TEMP_T1_THRES  0
#define TEMP_T1_THRES_PLUS_X_DEGREE 6
#define TEMP_T0_THRES  0
#define TEMP_T0_THRES_PLUS_X_DEGREE  0
#define TEMP_NEG_10_THRES 0

/* Begin added by hailong.chen for task 9785237 on 2020-10-10 */
#if defined(CONFIG_TCT_CHARGER)
#define JEITA_TEMP_ABOVE_T4_CURRENT			0
#define JEITA_TEMP_T3_TO_T4_CURRENT			1000000
#define JEITA_TEMP_T2_TO_T3_CURRENT			1000000
#define JEITA_TEMP_T1_TO_T2_CURRENT			1000000
#define JEITA_TEMP_T0_TO_T1_CURRENT			500000
#define JEITA_TEMP_BELOW_T0_CURRENT			0

#define SLAVE_JEITA_TEMP_ABOVE_T4_CURRENT	0
#define SLAVE_JEITA_TEMP_T3_TO_T4_CURRENT	1000000
#define SLAVE_JEITA_TEMP_T2_TO_T3_CURRENT	1000000
#define SLAVE_JEITA_TEMP_T1_TO_T2_CURRENT	1000000
#define SLAVE_JEITA_TEMP_T0_TO_T1_CURRENT	500000
#define SLAVE_JEITA_TEMP_BELOW_T0_CURRENT	0

#define STEP_CHG_VBAT						4200000
#define STEP_CHG_VBAT_HYSTERESIS			80000
#define STEP_CHG_CURRENT					1000000
#define SLAVE_STEP_CHG_CURRENT				1000000
#endif
/* End added by hailong.chen for task 9785237 on 2020-10-10 */

enum sw_jeita_state_enum {
	TEMP_BELOW_T0 = 0,
	TEMP_T0_TO_T1,
	TEMP_T1_TO_T2,
	TEMP_T2_TO_T3,
	TEMP_T3_TO_T4,
	TEMP_ABOVE_T4
};

struct sw_jeita_data {
	int sm;
	int pre_sm;
	int cv;
	bool charging;
	bool error_recovery_flag;
};

struct mtk_charger_algorithm {

	int (*do_algorithm)(struct mtk_charger *info);
	int (*enable_charging)(struct mtk_charger *info, bool en);
	int (*do_event)(struct notifier_block *nb, unsigned long ev, void *v);
	int (*change_current_setting)(struct mtk_charger *info);
	void *algo_data;
};

struct charger_custom_data {
	int battery_cv;	/* uv */
	int max_charger_voltage;
	int max_charger_voltage_setting;
	int min_charger_voltage;

	int usb_charger_current;
	int ac_charger_current;
	int ac_charger_input_current;
	int charging_host_charger_current;
/* [BSP]Begin added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
#if IS_ENABLED(CONFIG_TCT_CHARGER_GCS)
	int weak_ac_charger_input_current;
#endif
/* [BSP]End added by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
/*Begin:add for sw_aicl check by wanglin.chen on 2023.01.12*/
#if defined(CONFIG_TCT_CHARGER)
	bool sw_aicl_done;
#endif
/*End:add for sw_aicl check by wanglin.chen on 2023.01.12*/
	/* sw jeita */
	int jeita_temp_above_t4_cv;
	int jeita_temp_t3_to_t4_cv;
	int jeita_temp_t2_to_t3_cv;
	int jeita_temp_t1_to_t2_cv;
	int jeita_temp_t0_to_t1_cv;
	int jeita_temp_below_t0_cv;
	int temp_t4_thres;
	int temp_t4_thres_minus_x_degree;
	int temp_t3_thres;
	int temp_t3_thres_minus_x_degree;
	int temp_t2_thres;
	int temp_t2_thres_plus_x_degree;
	int temp_t1_thres;
	int temp_t1_thres_plus_x_degree;
	int temp_t0_thres;
	int temp_t0_thres_plus_x_degree;
	int temp_neg_10_thres;

/* Begin added by hailong.chen for task 9785237 on 2020-10-10 */
#if defined(CONFIG_TCT_CHARGER)
	int jeita_temp_above_t4_current;
	int jeita_temp_t3_to_t4_current;
	int jeita_temp_t2_to_t3_current;
	int jeita_temp_t1_to_t2_current;
	int jeita_temp_t0_to_t1_current;
	int jeita_temp_below_t0_current;

	int slave_jeita_temp_above_t4_current;
	int slave_jeita_temp_t3_to_t4_current;
	int slave_jeita_temp_t2_to_t3_current;
	int slave_jeita_temp_t1_to_t2_current;
	int slave_jeita_temp_t0_to_t1_current;
	int slave_jeita_temp_below_t0_current;

	int step_chg_vbat;
	int step_chg_vbat_hysteresis;
	int setp_chg_current;
	int slave_setp_chg_current;
#endif
/* End added by hailong.chen for task 9785237 on 2020-10-10 */

	/* battery temperature protection */
	int mtk_temperature_recharge_support;
	int max_charge_temp;
	int max_charge_temp_minus_x_degree;
	int min_charge_temp;
	int min_charge_temp_plus_x_degree;

	/* dynamic mivr */
	int min_charger_voltage_1;
	int min_charger_voltage_2;
	int max_dmivr_charger_current;

};

struct charger_data {
	int input_current_limit;
	int charging_current_limit;

	int force_charging_current;
	int thermal_input_current_limit;
	int thermal_charging_current_limit;
	int disable_charging_count;
	int input_current_limit_by_aicl;
	int junction_temp_min;
	int junction_temp_max;
/* Begin added by hailong.chen for task 9785237 on 2020-10-10 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	int charging_current_limit_by_jeita;
	int charging_current_limit_by_vbat;
#endif
/* End added by hailong.chen for task 9785237 on 2020-10-10 */

/* Begin add by jin.wang for jira on 2021-11-5 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	int ibus_limit_by_others;
	int ibat_limit_by_others;
#endif
/* End add by jin.wang */

/* Begin add by jin.wang for jira on 2021-11-25 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	int ibus_limit_by_test;
#endif
/* End add by jin.wang */
};

enum chg_data_idx_enum {
	CHG1_SETTING,
	CHG2_SETTING,
	CHGS_SETTING_MAX,
};

struct mtk_charger {
	struct platform_device *pdev;
	struct charger_device *chg1_dev;
	struct notifier_block chg1_nb;
	struct charger_device *chg2_dev;

	struct charger_data chg_data[CHGS_SETTING_MAX];
	struct chg_limit_setting setting;
	enum charger_configuration config;

	struct power_supply_desc psy_desc1;
	struct power_supply_config psy_cfg1;
	struct power_supply *psy1;

	struct power_supply_desc psy_desc2;
	struct power_supply_config psy_cfg2;
	struct power_supply *psy2;

	struct power_supply  *chg_psy;
	struct power_supply  *bat_psy;

	struct adapter_device *pd_adapter;
	struct notifier_block pd_nb;
	struct mutex pd_lock;
	int pd_type;
	bool pd_reset;

	u32 bootmode;
	u32 boottype;

	int chr_type;

/* Begin add by jin.wang task 2064 on 2021.11.30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	bool first_redet;
#endif
/* End add by jin.wang */

	int usb_state;

	struct mutex cable_out_lock;
	int cable_out_cnt;

	/* system lock */
	spinlock_t slock;
	struct wakeup_source *charger_wakelock;
	struct mutex charger_lock;

	/* thread related */
	wait_queue_head_t  wait_que;
	bool charger_thread_timeout;
	unsigned int polling_interval;
	bool charger_thread_polling;

	/* alarm timer */
	struct alarm charger_timer;
	struct timespec endtime;
	bool is_suspend;
	struct notifier_block pm_notifier;

	/* notify charger user */
	struct srcu_notifier_head evt_nh;

	/* common info */
	int log_level;
	bool usb_unlimited;
	bool disable_charger;
	int battery_temp;
	bool can_charging;
	bool cmd_discharging;
	bool safety_timeout;
	bool vbusov_stat;
	bool is_chg_done;
	/* ATM */
	bool atm_enabled;

	const char *algorithm_name;
	struct mtk_charger_algorithm algo;

	/* dtsi custom data */
	struct charger_custom_data data;

	/* battery warning */
	unsigned int notify_code;
	unsigned int notify_test_mode;

	/* sw safety timer */
	bool enable_sw_safety_timer;
	bool sw_safety_timer_setting;
	struct timespec charging_begin_time;

/* Begin add by jin.wang for jira on 2021-11-30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	bool thermal_disable;
#endif
/* End add by jin.wang */

	/* sw jeita */
	bool enable_sw_jeita;
	struct sw_jeita_data sw_jeita;

	/* battery thermal protection */
	struct battery_thermal_protection_data thermal;

	struct chg_alg_device *alg[MAX_ALG_NO];
	struct notifier_block chg_alg_nb;
	bool enable_hv_charging;

	/* water detection */
	bool water_detected;

	bool enable_dynamic_mivr;
	bool enable_sw_aicl;
/* Begin mod by jin.wang for androidT on 2022-4-12 */
#if IS_ENABLED(CONFIG_TCT_CHARGER_GCS)
	int input_avg_current;
	int nonstand_chg_type;
#endif
/* End mod by jin.wang */

/* Begin added by hailong.chen for task 9785237 on 2020-10-10 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	bool enable_step_chg;
#endif
/* End added by hailong.chen for task 9785237 on 2020-10-10 */
};

/* functions which framework needs*/
extern int mtk_basic_charger_init(struct mtk_charger *info);
extern int mtk_pulse_charger_init(struct mtk_charger *info);
extern int get_uisoc(struct mtk_charger *info);
extern int get_battery_voltage(struct mtk_charger *info);
extern int get_battery_temperature(struct mtk_charger *info);
extern int get_battery_current(struct mtk_charger *info);
extern int get_vbus(struct mtk_charger *info);
extern int get_ibus(struct mtk_charger *info);
extern bool is_battery_exist(struct mtk_charger *info);
extern int get_charger_type(struct mtk_charger *info);
extern int disable_hw_ovp(struct mtk_charger *info, int en);
extern bool is_charger_exist(struct mtk_charger *info);
extern int get_charger_temperature(struct mtk_charger *info,
	struct charger_device *chg);
extern int get_charger_charging_current(struct mtk_charger *info,
	struct charger_device *chg);
extern int get_charger_input_current(struct mtk_charger *info,
	struct charger_device *chg);
extern int get_charger_zcv(struct mtk_charger *info,
	struct charger_device *chg);
extern void _wake_up_charger(struct mtk_charger *info);

/* functions for other */
extern int mtk_chg_enable_vbus_ovp(bool enable);
/* Begin added by hailong.chen for task 9785241 on 2020-10-21 */
#if defined(CONFIG_TCT_CHARGER)
extern const char *get_battery_type(struct mtk_charger *info);
extern int set_hv_flag(int hv_flag);
extern int battery_do_health_update(struct mtk_charger *info); /* Added by bitao.xiong for AOSP13TMO-4319 on 2022-08-03 */
#endif
/* End added by hailong.chen for task 9785241 on 2020-10-21 */


#endif /* __MTK_CHARGER_H */
