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
/* Begin Added by tangshan.bai for LEVIN-6148 */
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
#include "mtk_battery.h"
#include <linux/ktime.h>
#include <linux/rtc.h>
#endif
/* End Added by tangshan.bai for LEVIN-6148 */
/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static int __maybe_unused _uA_to_mA(int uA)
#else
static int _uA_to_mA(int uA)
#endif
/* End mod by jin.wang */
{
	if (uA == -1)
		return -1;
	else
		return uA / 1000;
}

static void select_cv(struct mtk_charger *info)
{
/* Begin mod by jin.wang task 2064 on 2021.10.8 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (info->sw_jeita.cv != 0)
		info->setting.cv = info->sw_jeita.cv;
	else
		info->setting.cv = info->data.battery_cv;
#else
	u32 constant_voltage;

	if (info->enable_sw_jeita) {
		if (info->sw_jeita.cv != 0) {
			info->setting.cv = info->sw_jeita.cv;
			return;
		}
	}

	constant_voltage = info->data.battery_cv;
	info->setting.cv = constant_voltage;
#endif
/* End mod by jin.wang */
}

static bool is_typec_adapter(struct mtk_charger *info)
{
	int rp;

	rp = adapter_dev_get_property(info->pd_adapter, TYPEC_RP_LEVEL);
/* Begin add by jin.wang task 2064 on 2021.10.25 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (rp < 0)
		return false;
#endif
/* End add by jin.wang */

	if (info->pd_type == MTK_PD_CONNECT_TYPEC_ONLY_SNK &&
			rp != 500 &&
			info->chr_type != POWER_SUPPLY_TYPE_USB &&
			info->chr_type != POWER_SUPPLY_TYPE_USB_CDP)
		return true;

	return false;
}

static bool support_fast_charging(struct mtk_charger *info)
{
	struct chg_alg_device *alg;
	int i = 0, state = 0;
	bool ret = false;

/* Begin add by jin.wang task 2064 on 2021.11.1 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if (!info->enable_hv_charging)
		return false;
#endif
/* End add by jin.wang */

	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		if (alg == NULL)
			continue;

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		chg_alg_set_current_limit(alg, &info->setting);
#endif
/* End del by jin.wang */

		state = chg_alg_is_algo_ready(alg);
		chr_debug("%s %s ret:%s\n", __func__, dev_name(&alg->dev),
			chg_alg_state_to_str(state));

/* Begin mod by jin.wang task 2064 on 2021.10.26 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		if (state == ALG_READY || state == ALG_RUNNING
			|| state == ALG_DONE) {
#else
		if (state == ALG_READY || state == ALG_RUNNING) {
#endif
/* End mod by jin.wang */
			ret = true;
			break;
		}
	}
	return ret;
}

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static bool select_charging_current_limit(struct mtk_charger *info,
	struct chg_limit_setting *setting)
{
	struct charger_data *pdata, *pdata2;
	bool is_basic = false;
	u32 ichg1_min = 0, aicr1_min = 0;
	int total_thermal_ibus_limit = -1;
	int total_thermal_ibat_limit = -1;
	int ret;

	select_cv(info);

	pdata = &info->chg_data[CHG1_SETTING];
	pdata2 = &info->chg_data[CHG2_SETTING];

	info->setting.total_ibus_limit = pdata->input_current_limit;
	info->setting.total_ibatt_limit = info->data.jeita_temp_t2_to_t3_current;

	if (info->thermal_disable) {
		pdata->thermal_charging_current_limit = -1;
		pdata->thermal_input_current_limit = -1;
		pdata2->thermal_charging_current_limit = -1;
		pdata2->thermal_input_current_limit = -1;
	}

	if (info->usb_unlimited) {
		pdata->input_current_limit =
					info->data.ac_charger_input_current;
		pdata->charging_current_limit =
					info->data.ac_charger_current;
		is_basic = true;
		goto done;
	}

	if (info->water_detected) {
		pdata->input_current_limit = info->data.usb_charger_current;
		pdata->charging_current_limit = info->data.usb_charger_current;
		is_basic = true;
		goto done;
	}

	if ((info->bootmode == 1) ||
	    (info->bootmode == 5)) {
		pdata->input_current_limit = 200000; /* 200mA */
		pdata->charging_current_limit = 200000;
		is_basic = true;
		goto done;
	}

	if (info->atm_enabled == true
		&& (info->chr_type == POWER_SUPPLY_TYPE_USB ||
		info->chr_type == POWER_SUPPLY_TYPE_USB_CDP)
		) {
		pdata->input_current_limit = 100000; /* 100mA */
		pdata->charging_current_limit = 100000;
		is_basic = true;
		goto done;
	}

	if (info->chr_type == POWER_SUPPLY_TYPE_USB) {
		pdata->input_current_limit =
				info->data.usb_charger_current;
		/* it can be larger */
		pdata->charging_current_limit =
				info->data.usb_charger_current;
		is_basic = true;
	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB_CDP) {
		pdata->input_current_limit =
			info->data.charging_host_charger_current;
		pdata->charging_current_limit =
			info->data.charging_host_charger_current;
		is_basic = true;

	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB_DCP) {
		pdata->input_current_limit =
			info->data.ac_charger_input_current;
		pdata->charging_current_limit =
			info->data.ac_charger_current;
	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB_FLOAT) {
		/* NONSTANDARD_CHARGER */
		pdata->input_current_limit =
			info->data.usb_charger_current;
		pdata->charging_current_limit =
			info->data.usb_charger_current;
		is_basic = true;
	}

	if (!is_basic && support_fast_charging(info))
		is_basic = false;
	else {
		is_basic = true;

		/* AICL */
/* [BSP]Begin modified by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
#if 0
		charger_dev_run_aicl(info->chg1_dev,
			&pdata->input_current_limit_by_aicl);
#endif
/* [BSP]End modified by bitao.xiong for SNTBBH-4343 on 2022/12/20 */

/*Begin:modify by wanglin.chen on 2023.01.11*/
#if 0
		if (info->enable_dynamic_mivr) {
			if (pdata->input_current_limit_by_aicl >
				info->data.max_dmivr_charger_current)
				pdata->input_current_limit_by_aicl =
					info->data.max_dmivr_charger_current;
		}
#endif
/*End:modify by wanglin.chen on 2023.01.11*/
		if (is_typec_adapter(info)) {
			if (adapter_dev_get_property(info->pd_adapter, TYPEC_RP_LEVEL)
				== 3000) {
				pdata->input_current_limit = 3000000;
				pdata->charging_current_limit = 3000000;
			} else if (adapter_dev_get_property(info->pd_adapter,
				TYPEC_RP_LEVEL) == 1500) {
				pdata->input_current_limit = 1500000;
				pdata->charging_current_limit = 2000000;
			} else {
				chr_err("type-C: inquire rp error\n");
				pdata->input_current_limit = 500000;
				pdata->charging_current_limit = 500000;
			}

			chr_err("type-C:%d current:%d\n",
				info->pd_type,
				adapter_dev_get_property(info->pd_adapter,
					TYPEC_RP_LEVEL));
		}
	}

#if !IS_ENABLED(TARGET_BUILD_MMITEST)
	if (info->enable_sw_jeita) {
		if (IS_ENABLED(CONFIG_USBIF_COMPLIANCE)
			&& info->chr_type == POWER_SUPPLY_TYPE_USB)
			chr_debug("USBIF & STAND_HOST skip current check\n");
		else {
			switch (info->sw_jeita.sm) {
			case TEMP_BELOW_T0:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_below_t0_current;
				break;
			case TEMP_T0_TO_T1:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_t0_to_t1_current;
				break;
			case TEMP_T1_TO_T2:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_t1_to_t2_current;
				break;
			case TEMP_T2_TO_T3:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_t2_to_t3_current;
				break;
			case TEMP_T3_TO_T4:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_t3_to_t4_current;
				break;
			case TEMP_ABOVE_T4:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_above_t4_current;
				break;
			default:
				pdata->charging_current_limit_by_jeita = -1;
				break;
			}
			chr_err("[%s]sw_jeita.sm:%d,jeita(%d %d %d)\n",
					__func__, info->sw_jeita.sm,
					pdata->input_current_limit, pdata->charging_current_limit, pdata->charging_current_limit_by_jeita);

			if ((pdata->charging_current_limit_by_jeita != -1)
				&& pdata->charging_current_limit > pdata->charging_current_limit_by_jeita) {
				pdata->charging_current_limit = pdata->charging_current_limit_by_jeita;
			}
		}
	}

	if (pdata->charging_current_limit_by_vbat != -1) {
		if (pdata->charging_current_limit > pdata->charging_current_limit_by_vbat)
			pdata->charging_current_limit = pdata->charging_current_limit_by_vbat;
	}

	if (pdata->thermal_charging_current_limit != -1) {
		if (pdata->thermal_charging_current_limit <
			pdata->charging_current_limit) {
			pdata->charging_current_limit =
					pdata->thermal_charging_current_limit;
		}
	}

	if (pdata->thermal_input_current_limit != -1) {
		if (pdata->thermal_input_current_limit <
			pdata->input_current_limit) {
			pdata->input_current_limit =
					pdata->thermal_input_current_limit;
		}
	}
#else
	pdata->charging_current_limit_by_jeita = -1;
	pdata->charging_current_limit_by_vbat = -1;
	pdata->thermal_charging_current_limit = -1;
	pdata->thermal_input_current_limit = -1;
#endif

	/* Robust apk will limit ibus value */
	if (pdata->ibus_limit_by_test != -1) {
		if (pdata->ibus_limit_by_test <
			pdata->input_current_limit) {
			pdata->input_current_limit =
					pdata->ibus_limit_by_test;
			info->setting.total_ibus_limit =
					pdata->ibus_limit_by_test;
		}
	}

	/* audio part will limit ibus / ibat value */
	if (pdata->ibus_limit_by_others != -1) {
		if (pdata->ibus_limit_by_others <
			pdata->input_current_limit) {
			pdata->input_current_limit =
					pdata->ibus_limit_by_others;
		}
		is_basic = true;
	}
	if (pdata->ibat_limit_by_others != -1) {
		if (pdata->ibat_limit_by_others <
			pdata->charging_current_limit) {
			pdata->charging_current_limit =
					pdata->ibat_limit_by_others;
		}
		is_basic = true;
	}

	if (is_basic == true && pdata->input_current_limit_by_aicl != -1) {
		if (pdata->input_current_limit_by_aicl <
		    pdata->input_current_limit)
			pdata->input_current_limit =
					pdata->input_current_limit_by_aicl;
	}

done:
	ret = charger_dev_get_min_charging_current(info->chg1_dev, &ichg1_min);
	if (ret != -ENOTSUPP && pdata->charging_current_limit < ichg1_min) {
		pdata->charging_current_limit = 0;
		chr_err("min_charging_current is too low %d %d\n",
			pdata->charging_current_limit, ichg1_min);
		is_basic = true;
	}

	ret = charger_dev_get_min_input_current(info->chg1_dev, &aicr1_min);
	if (ret != -ENOTSUPP && pdata->input_current_limit < aicr1_min) {
		pdata->input_current_limit = 0;
		pr_err("min_input_current is too low %d %d\n",
			pdata->input_current_limit, aicr1_min);
		is_basic = true;
	}

	if (!is_basic) {
		if ((pdata->charging_current_limit_by_jeita != -1)
			&& (info->setting.total_ibatt_limit
				> pdata->charging_current_limit_by_jeita)) {
			info->setting.total_ibatt_limit =
				pdata->charging_current_limit_by_jeita;
		}
		if ((pdata->charging_current_limit_by_vbat != -1)
			&& (info->setting.total_ibatt_limit
				> pdata->charging_current_limit_by_vbat)) {
			info->setting.total_ibatt_limit =
				pdata->charging_current_limit_by_vbat;
		}

		/* check upper thermal ibus limit */
		if (pdata->thermal_input_current_limit != -1) {
			total_thermal_ibus_limit =
				pdata->thermal_input_current_limit;
		}
		if ((pdata2->thermal_input_current_limit != -1)
			&& (total_thermal_ibus_limit != -1)) {
			total_thermal_ibus_limit +=
				pdata2->thermal_input_current_limit;
		}
		if ((total_thermal_ibus_limit != -1)
			&& (info->setting.total_ibus_limit
				> total_thermal_ibus_limit)) {
			info->setting.total_ibus_limit =
				total_thermal_ibus_limit;
		}

		/* check upper thermal ibatt limit */
		if (pdata->thermal_charging_current_limit != -1) {
			total_thermal_ibat_limit =
				pdata->thermal_charging_current_limit;
		}
		if ((pdata2->thermal_charging_current_limit != -1)
			&& (total_thermal_ibat_limit != -1)) {
			total_thermal_ibat_limit +=
				pdata2->thermal_charging_current_limit;
		}
		if ((total_thermal_ibat_limit != -1)
			&& (info->setting.total_ibatt_limit
				> total_thermal_ibat_limit)) {
			info->setting.total_ibatt_limit =
				total_thermal_ibat_limit;
		}

		if ((info->setting.total_ibatt_limit < 2000000)
			|| (info->setting.total_ibus_limit < 1000000)) {
			is_basic = true;
		} else {
			if (info->setting.total_ibus_limit == INT_MAX) {
				info->setting.total_ibus_limit = -1;
			}
			if (info->setting.total_ibatt_limit == INT_MAX) {
				info->setting.total_ibatt_limit = -1;
			}
		}
	}

	chr_err("m:%d type:%d:%d unlimited:%d usbif:%d aicl:%d atm:%d bm:%d b:%d,%d\n",
		info->config,
		info->chr_type, info->pd_type,
		info->usb_unlimited,
		IS_ENABLED(CONFIG_USBIF_COMPLIANCE),
		pdata->input_current_limit_by_aicl, info->atm_enabled,
		info->bootmode, is_basic, info->thermal_disable);

	chr_err("chg1:%d,%d,%d,%d chg2-therm:%d,%d others:%d,%d,%d total:%d,%d,%d,%d\n",
		pdata->thermal_input_current_limit,
		pdata->thermal_charging_current_limit,
		pdata->input_current_limit,
		pdata->charging_current_limit,
		pdata2->thermal_input_current_limit,
		pdata2->thermal_charging_current_limit,
		pdata->ibus_limit_by_others,
		pdata->ibat_limit_by_others,
		pdata->ibus_limit_by_test,
		total_thermal_ibus_limit,
		total_thermal_ibat_limit,
		info->setting.total_ibus_limit,
		info->setting.total_ibatt_limit);

	return is_basic;
}
#else
static bool select_charging_current_limit(struct mtk_charger *info,
	struct chg_limit_setting *setting)
{
	struct charger_data *pdata, *pdata2;
	bool is_basic = false;
	u32 ichg1_min = 0, aicr1_min = 0;
	int ret;

	select_cv(info);

	pdata = &info->chg_data[CHG1_SETTING];
	pdata2 = &info->chg_data[CHG2_SETTING];
	if (info->usb_unlimited) {
		pdata->input_current_limit =
					info->data.ac_charger_input_current;
		pdata->charging_current_limit =
					info->data.ac_charger_current;
		is_basic = true;
		goto done;
	}

	if (info->water_detected) {
		pdata->input_current_limit = info->data.usb_charger_current;
		pdata->charging_current_limit = info->data.usb_charger_current;
		is_basic = true;
		goto done;
	}

	if ((info->bootmode == 1) ||
	    (info->bootmode == 5)) {
		pdata->input_current_limit = 200000; /* 200mA */
		is_basic = true;
		goto done;
	}

	if (info->atm_enabled == true
		&& (info->chr_type == POWER_SUPPLY_TYPE_USB ||
		info->chr_type == POWER_SUPPLY_TYPE_USB_CDP)
		) {
		pdata->input_current_limit = 100000; /* 100mA */
		is_basic = true;
		goto done;
	}

	if (info->chr_type == POWER_SUPPLY_TYPE_USB) {
		pdata->input_current_limit =
				info->data.usb_charger_current;
		/* it can be larger */
		pdata->charging_current_limit =
				info->data.usb_charger_current;
		is_basic = true;
	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB_CDP) {
		pdata->input_current_limit =
			info->data.charging_host_charger_current;
		pdata->charging_current_limit =
			info->data.charging_host_charger_current;
		is_basic = true;

	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB_DCP) {
		pdata->input_current_limit =
			info->data.ac_charger_input_current;
		pdata->charging_current_limit =
			info->data.ac_charger_current;
		if (info->config == DUAL_CHARGERS_IN_SERIES) {
			pdata2->input_current_limit =
				pdata->input_current_limit;
			pdata2->charging_current_limit = 2000000;
#if IS_ENABLED(CONFIG_TCT_CHARGER) /* change by bitao.xiong for PARALLEL charge */
		} else if (info->config == DUAL_CHARGERS_IN_PARALLEL) {
			pdata2->input_current_limit =
				pdata->input_current_limit;
			pdata2->charging_current_limit = 2000000;
#endif
		}
	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB_FLOAT) {
		/* NONSTANDARD_CHARGER */
		pdata->input_current_limit =
			info->data.usb_charger_current;
		pdata->charging_current_limit =
			info->data.usb_charger_current;
		is_basic = true;
	}

/* Begin mod by jin.wang task 2064 on 2021.10.8 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (!is_basic && support_fast_charging(info))
#else
	if (support_fast_charging(info))
#endif
/* End mod by jin.wang */
		is_basic = false;
	else {
		is_basic = true;
		/* AICL */
/* [BSP]Begin modified by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
#if 0
		charger_dev_run_aicl(info->chg1_dev,
			&pdata->input_current_limit_by_aicl);
#endif
/* [BSP]End modified by bitao.xiong for SNTBBH-4343 on 2022/12/20 */
		if (info->enable_dynamic_mivr) {
			if (pdata->input_current_limit_by_aicl >
				info->data.max_dmivr_charger_current)
				pdata->input_current_limit_by_aicl =
					info->data.max_dmivr_charger_current;
		}
		if (is_typec_adapter(info)) {
			if (adapter_dev_get_property(info->pd_adapter, TYPEC_RP_LEVEL)
				== 3000) {
				pdata->input_current_limit = 3000000;
				pdata->charging_current_limit = 3000000;
			} else if (adapter_dev_get_property(info->pd_adapter,
				TYPEC_RP_LEVEL) == 1500) {
				pdata->input_current_limit = 1500000;
				pdata->charging_current_limit = 2000000;
			} else {
				chr_err("type-C: inquire rp error\n");
				pdata->input_current_limit = 500000;
				pdata->charging_current_limit = 500000;
			}

			chr_err("type-C:%d current:%d\n",
				info->pd_type,
				adapter_dev_get_property(info->pd_adapter,
					TYPEC_RP_LEVEL));
		}
	}

/* Begin added by hailong.chen for task 9785237 on 2020-10-10 */
#if !IS_ENABLED(TARGET_BUILD_MMITEST) //added by dapeng.qiao for task 11024165 on 2021-4-14
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (info->enable_sw_jeita) {
		if (IS_ENABLED(CONFIG_USBIF_COMPLIANCE)
			&& info->chr_type == POWER_SUPPLY_TYPE_USB)
			chr_debug("USBIF & STAND_HOST skip current check\n");
		else {
			switch (info->sw_jeita.sm) {
			case TEMP_BELOW_T0:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_below_t0_current;
				pdata2->charging_current_limit_by_jeita = info->data.slave_jeita_temp_below_t0_current;
				break;
			case TEMP_T0_TO_T1:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_t0_to_t1_current;
				pdata2->charging_current_limit_by_jeita = info->data.slave_jeita_temp_t0_to_t1_current;
				break;
			case TEMP_T1_TO_T2:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_t1_to_t2_current;
				pdata2->charging_current_limit_by_jeita = info->data.slave_jeita_temp_t1_to_t2_current;
				break;
			case TEMP_T2_TO_T3:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_t2_to_t3_current;
				pdata2->charging_current_limit_by_jeita = info->data.slave_jeita_temp_t2_to_t3_current;
				break;
			case TEMP_T3_TO_T4:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_t3_to_t4_current;
				pdata2->charging_current_limit_by_jeita = info->data.slave_jeita_temp_t3_to_t4_current;
				break;
			case TEMP_ABOVE_T4:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_above_t4_current;
				pdata2->charging_current_limit_by_jeita = info->data.slave_jeita_temp_t3_to_t4_current;
				break;
			default:
				pdata->charging_current_limit_by_jeita = info->data.jeita_temp_t2_to_t3_current;
				pdata2->charging_current_limit_by_jeita = info->data.slave_jeita_temp_t2_to_t3_current;
				chr_err("bad sw_jeita.sm %d,please check it!\n", info->sw_jeita.sm);
				break;
			}

			chr_err("[%s]sw_jeita.sm:%d,jeita(%d %d)\n",
					__func__, info->sw_jeita.sm,
					pdata->input_current_limit, pdata->charging_current_limit);
			if (pdata->charging_current_limit > pdata->charging_current_limit_by_jeita)
				pdata->charging_current_limit = pdata->charging_current_limit_by_jeita;
		}

		if (info->config == DUAL_CHARGERS_IN_SERIES) {
			if (pdata2->charging_current_limit > pdata2->charging_current_limit_by_jeita) {
				pdata2->charging_current_limit = pdata2->charging_current_limit_by_jeita;
				chr_err("[%s] SERIES pdata2->charging_current_limit=%d  pdata2->charging_current_limit_by_jeita=%d\n",
					__func__, pdata->charging_current_limit, pdata2->charging_current_limit_by_jeita);
			}
		}
/* Begin modified by jin.wang task 2064 on 2021.10.5 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		else if (info->config == DUAL_CHARGERS_IN_PARALLEL) {
			if (pdata2->charging_current_limit > pdata2->charging_current_limit_by_jeita) {
				pdata2->charging_current_limit = pdata2->charging_current_limit_by_jeita;
				chr_err("[%s] PARALLEL pdata2->charging_current_limit=%d  pdata2->charging_current_limit_by_jeita=%d\n",
					__func__, pdata->charging_current_limit, pdata2->charging_current_limit_by_jeita);
            		}
		}
#endif
/* End modified by jin.wang */
	}

	if (pdata->charging_current_limit_by_vbat != -1) {
		if (pdata->charging_current_limit > pdata->charging_current_limit_by_vbat)
			pdata->charging_current_limit = pdata->charging_current_limit_by_vbat;
	}

	if (pdata2->charging_current_limit_by_vbat != -1) {
		if (pdata2->charging_current_limit > pdata2->charging_current_limit_by_vbat)
			pdata2->charging_current_limit = pdata2->charging_current_limit_by_vbat;
	}
#else
	if (info->enable_sw_jeita) {
		if (IS_ENABLED(CONFIG_USBIF_COMPLIANCE)
			&& info->chr_type == POWER_SUPPLY_TYPE_USB)
			chr_debug("USBIF & STAND_HOST skip current check\n");
		else {
			if (info->sw_jeita.sm == TEMP_T0_TO_T1) {
				pdata->input_current_limit = 500000;
				pdata->charging_current_limit = 350000;
			}
		}
	}
#endif
#endif        //added by dapeng.qiao for task 11024165 on 2021-4-14
/* End added by hailong.chen for task 9785237 on 2020-10-10 */

	if (pdata->thermal_charging_current_limit != -1) {
		if (pdata->thermal_charging_current_limit <
			pdata->charging_current_limit) {
			pdata->charging_current_limit =
					pdata->thermal_charging_current_limit;
			info->setting.charging_current_limit1 =
					pdata->thermal_charging_current_limit;
		}
	} else
		info->setting.charging_current_limit1 = -1;

	if (pdata->thermal_input_current_limit != -1) {
		if (pdata->thermal_input_current_limit <
			pdata->input_current_limit) {
			pdata->input_current_limit =
					pdata->thermal_input_current_limit;
			info->setting.input_current_limit1 =
					pdata->input_current_limit;
		}
	} else
		info->setting.input_current_limit1 = -1;

	if (pdata2->thermal_charging_current_limit != -1) {
		if (pdata2->thermal_charging_current_limit <
			pdata2->charging_current_limit) {
			pdata2->charging_current_limit =
					pdata2->thermal_charging_current_limit;
			info->setting.charging_current_limit2 =
					pdata2->charging_current_limit;
		}
	} else
		info->setting.charging_current_limit2 = -1;

	if (pdata2->thermal_input_current_limit != -1) {
		if (pdata2->thermal_input_current_limit <
			pdata2->input_current_limit) {
			pdata2->input_current_limit =
					pdata2->thermal_input_current_limit;
			info->setting.input_current_limit2 =
					pdata2->input_current_limit;
		}
	} else
		info->setting.input_current_limit2 = -1;

	if (is_basic == true && pdata->input_current_limit_by_aicl != -1) {
		if (pdata->input_current_limit_by_aicl <
		    pdata->input_current_limit)
			pdata->input_current_limit =
					pdata->input_current_limit_by_aicl;
	}
done:

	ret = charger_dev_get_min_charging_current(info->chg1_dev, &ichg1_min);
	if (ret != -ENOTSUPP && pdata->charging_current_limit < ichg1_min) {
		pdata->charging_current_limit = 0;
		chr_err("min_charging_current is too low %d %d\n",
			pdata->charging_current_limit, ichg1_min);
		is_basic = true;
		info->enable_hv_charging = false;
	}

	ret = charger_dev_get_min_input_current(info->chg1_dev, &aicr1_min);
	if (ret != -ENOTSUPP && pdata->input_current_limit < aicr1_min) {
		pdata->input_current_limit = 0;
		chr_err("min_input_current is too low %d %d\n",
			pdata->input_current_limit, aicr1_min);
		is_basic = true;
		info->enable_hv_charging = false;
	}

/* Begin added by weijun for usbif current limit on 2022-02-25 */
#if IS_ENABLED(TARGET_BUILD_CERTIFICATION) \
	|| IS_ENABLED(TARGET_BUILD_GCF)
	if (pdata->input_current_limit > 1500000) {
		pdata->input_current_limit = 1500000;
	}

	if (pdata->charging_current_limit > 1500000) {
		pdata->charging_current_limit = 1500000;
	}
#endif
/* End added by weijun for usbif current limit on 2022-02-25 */

	chr_err("m:%d chg1:%d,%d,%d,%d chg2:%d,%d,%d,%d type:%d:%d usb_unlimited:%d usbif:%d usbsm:%d aicl:%d atm:%d bm:%d b:%d\n",
		info->config,
		_uA_to_mA(pdata->thermal_input_current_limit),
		_uA_to_mA(pdata->thermal_charging_current_limit),
		_uA_to_mA(pdata->input_current_limit),
		_uA_to_mA(pdata->charging_current_limit),
		_uA_to_mA(pdata2->thermal_input_current_limit),
		_uA_to_mA(pdata2->thermal_charging_current_limit),
		_uA_to_mA(pdata2->input_current_limit),
		_uA_to_mA(pdata2->charging_current_limit),
		info->chr_type, info->pd_type,
		info->usb_unlimited,
		IS_ENABLED(CONFIG_USBIF_COMPLIANCE), info->usb_state,
		pdata->input_current_limit_by_aicl, info->atm_enabled,
		info->bootmode, is_basic);

	return is_basic;
}
#endif
/* End mod by jin.wang */

/* Begin Added by tangshan.bai for LEVIN-6148 */
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
void cal_batt_life_discharge_or_charging(void)
{
    int fg_coulomb;
    unsigned long batt_life;
    struct timespec ts;
    struct rtc_time tm;
    struct mtk_battery *gm;
    gm = get_mtk_battery();

    fg_coulomb = 0;
    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec, &tm);
    chr_err("fg_life1:%d-%02d-%02d %02d:%02d:%02d.%09lu\n",
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
    fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
    batt_life = cal_batt_life(gm->g_begin_v_soc,gm->fg_cust_data.v_soc,gm->g_begin_fg_coulomb,fg_coulomb);
    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec, &tm);
    chr_err("fg_life2:%d-%02d-%02d %02d:%02d:%02d.%09lu\n",
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
}
#endif
/* End Added by tangshan.bai for LEVIN-6148 */
/* Begin Added by tangshan.bai for LEVIN-6148 */
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
static void peak_managment(struct mtk_charger *info,bool *chg_done)
{
	struct mtk_battery *gm;
	gm = get_mtk_battery();

    if((gm->BAT_peak_level > 100) || (gm->BAT_peak_level < 80)){ //not allow too small
        gm->BAT_peak_level = 100;
    }
    if(gm->BatteryVerify & BIT(3)){
        gm->BAT_peak_level = 100;//restart or timing start verify or be verifying
    }

    if(gm->BAT_peak_level < 100){
        if ((gm->soc >= gm->BAT_peak_level) || (gm->ui_soc >= gm->BAT_peak_level))
        {
            gm->peak_enforce_full = 1;
            gm->bat_state = BAT_ENFORCE_TO_FULL;
            charger_dev_enable(info->chg1_dev, false); //stop charging
			charger_dev_enable(info->chg2_dev, false);
            chr_err("peak managment enforce battery full!\n");
        }else{
            if ((gm->soc <= (gm->BAT_peak_level-1)) && (gm->ui_soc <= gm->BAT_peak_level-1)){
                gm->peak_enforce_full = 0;
            }
        }
    }else{
    	gm->peak_enforce_full = 0;
        chr_err("peak managment Battery is Verifying or not start\n");
    }

    chr_err("peak gm->soc:%d, BAT_peak_level:%d,info->is_chg_done:%d,chg_done:%d,BatteryVerify:0x%x,enforce_full:%d\n",\
        gm->soc,gm->BAT_peak_level,info->is_chg_done,*chg_done,gm->BatteryVerify,gm->peak_enforce_full);

}
#endif
/* End Added by tangshan.bai for LEVIN-6148 */

static int do_algorithm(struct mtk_charger *info)
{
	struct chg_alg_device *alg;
	struct charger_data *pdata;
	struct chg_alg_notify notify;
	bool is_basic = true;
	bool chg_done = false;
	int i;
	int ret;
/* Begin del by jin.wang task 2064 on 2021.10.26 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	int val = 0;
#endif
/* End del by jin.wang */

	pdata = &info->chg_data[CHG1_SETTING];
	charger_dev_is_charging_done(info->chg1_dev, &chg_done);

/* Begin Added by tangshan.bai for LEVIN-6148 */
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
	peak_managment(info,&chg_done);
#endif
/* End Added by tangshan.bai for LEVIN-6148 */

	is_basic = select_charging_current_limit(info, &info->setting);

	if (info->is_chg_done != chg_done) {
		if (chg_done) {
			charger_dev_do_event(info->chg1_dev, EVENT_FULL, 0);
			chr_err("%s battery full\n", __func__);
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
			calculate_coulomb_charged(EVT_FULL);
#endif
		} else {
			charger_dev_do_event(info->chg1_dev, EVENT_RECHARGE, 0);
			chr_err("%s battery recharge\n", __func__);
		}
	}
//Begin Modified by qiuguangliang for 10956228 on 2021-05-21
	chr_err("%s is_basic:%d is_chg_done:%d chg_done:%d\n",
			__func__, is_basic, info->is_chg_done, chg_done);
//End Modified by qiuguangliang for 10956228 on 2021-05-21

	if (is_basic != true) {
		is_basic = true;
		for (i = 0; i < MAX_ALG_NO; i++) {
			alg = info->alg[i];
			if (alg == NULL)
				continue;

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
			if (!info->enable_hv_charging ||
			    pdata->charging_current_limit == 0 ||
			    pdata->input_current_limit == 0) {
				chg_alg_get_prop(alg, ALG_MAX_VBUS, &val);
				if (val > 5000)
					chg_alg_stop_algo(alg);
				chr_err("%s: alg:%s alg_vbus:%d\n", __func__,
					dev_name(&alg->dev), val);
				continue;
			}
#endif
/* End del by jin.wang */

			if (chg_done != info->is_chg_done) {
				if (chg_done) {
					notify.evt = EVT_FULL;
					notify.value = 0;
				} else {
					notify.evt = EVT_RECHARGE;
					notify.value = 0;
				}
				chg_alg_notifier_call(alg, &notify);
				chr_err("%s notify:%d\n", __func__, notify.evt);
			}

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
			chg_alg_set_current_limit(alg, &info->setting);
#endif
/* End del by jin.wang */

			ret = chg_alg_is_algo_ready(alg);

			chr_err("%s %s ret:%s\n", __func__,
				dev_name(&alg->dev),
				chg_alg_state_to_str(ret));

			if (ret == ALG_INIT_FAIL || ret == ALG_TA_NOT_SUPPORT) {
				/* try next algorithm */
				continue;
/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
			} else if (ret == ALG_TA_CHECKING || ret == ALG_NOT_READY) {
#else
			} else if (ret == ALG_TA_CHECKING || ret == ALG_DONE ||
						ret == ALG_NOT_READY) {
#endif
/* End mod by jin.wang */
				/* wait checking , use basic first */
				is_basic = true;
				break;
/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
			} else if (ret == ALG_READY || ret == ALG_RUNNING
				|| ret == ALG_DONE) {
#else
			} else if (ret == ALG_READY || ret == ALG_RUNNING) {
#endif
/* End mod by jin.wang */
				/* [BSP]Begin deleted by bitao.xiong for SNTBBH-4690 on 2023/01/10 */
				//is_basic = false;
				/* [BSP]End deleted by bitao.xiong for SNTBBH-4690 on 2023/01/10 */
/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
				chg_alg_set_current_limit(alg, &info->setting);
#endif
/* End add by jin.wang */
				chg_alg_start_algo(alg);
				/* [BSP]Begin added by bitao.xiong for SNTBBH-4690 on 2023/01/10 */
				ret = chg_alg_is_algo_ready(alg);

				chr_err("%s %s after chg_alg_start_algo,chg_alg_is_algo_ready ret:%s\n", __func__,
					dev_name(&alg->dev),
					chg_alg_state_to_str(ret));
				if (ret == ALG_READY || ret == ALG_RUNNING
					|| ret == ALG_DONE)
					is_basic = false;
				/* [BSP]End added by bitao.xiong for SNTBBH-4690 on 2023/01/10 */
				break;
			} else {
				chr_err("algorithm ret is error");
				is_basic = true;
			}
		}
	}
/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	else {  // is_basic == true
		if (info->enable_hv_charging != true ||
		    pdata->charging_current_limit == 0 ||
		    pdata->input_current_limit == 0) {
			for (i = 0; i < MAX_ALG_NO; i++) {
				alg = info->alg[i];
				if (alg == NULL)
					continue;

				chg_alg_get_prop(alg, ALG_MAX_VBUS, &val);
				if (val > 5000 && chg_alg_is_algo_running(alg))
					chg_alg_stop_algo(alg);

				chr_err("%s: Stop hv charging. en_hv:%d alg:%s alg_vbus:%d\n",
					__func__, info->enable_hv_charging,
					dev_name(&alg->dev), val);
			}
		}
	}
#endif
/* End del by jin.wang */

/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if (is_basic) {
		for (i = 0; i < MAX_ALG_NO; i++) {
			alg = info->alg[i];
			if (alg == NULL)
				continue;

			chg_alg_stop_algo(alg);

			//chr_err("%s: Stop hv charging. en_hv:%d alg:%s alg_vbus:%d\n",
			//		__func__, info->enable_hv_charging,
			//		dev_name(&alg->dev), val);
		}
	}
#endif
/* End add by jin.wang */

	info->is_chg_done = chg_done;

	if (is_basic == true) {
		charger_dev_set_input_current(info->chg1_dev,
			pdata->input_current_limit);
		charger_dev_set_charging_current(info->chg1_dev,
			pdata->charging_current_limit);
		charger_dev_set_constant_voltage(info->chg1_dev,
			info->setting.cv);

/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		if (info->enable_dynamic_mivr) {
			int vbat = get_battery_voltage(info);
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
#endif
/* End add by jin.wang */
	}

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if (pdata->input_current_limit == 0 ||
	    pdata->charging_current_limit == 0) {
		charger_dev_enable(info->chg1_dev, false);
		charger_dev_enable(info->chg2_dev, false);
	} else {
		charger_dev_enable(info->chg1_dev, true);
	}
#else
	if (pdata->input_current_limit == 0 ||
	    pdata->charging_current_limit == 0)
		charger_dev_enable(info->chg1_dev, false);
	else
		charger_dev_enable(info->chg1_dev, true);
#endif
/* End mod by jin.wang */

/* Begin del by jin.wang task 2064 on 2021.11.23 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if (info->chg1_dev != NULL)
		charger_dev_dump_registers(info->chg1_dev);

	if (info->chg2_dev != NULL)
		charger_dev_dump_registers(info->chg2_dev);
#endif
/* End del by jin.wang */

/* Begin mod by jin.wang task 2064 on 2021.10.25 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	return is_basic;
#else
	return 0;
#endif
/* End mod by jin.wang */
}

static int enable_charging(struct mtk_charger *info,
						bool en)
{
	int i;
	struct chg_alg_device *alg;

/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	struct chg_alg_notify notify;
#endif
/* End add by jin.wang */

	chr_err("%s %d\n", __func__, en);

	if (en == false) {
		for (i = 0; i < MAX_ALG_NO; i++) {
			alg = info->alg[i];
			if (alg == NULL)
				continue;
			chg_alg_stop_algo(alg);
		}
		charger_dev_enable(info->chg1_dev, false);
		charger_dev_do_event(info->chg1_dev, EVENT_DISCHARGE, 0);
	} else {
		charger_dev_enable(info->chg1_dev, true);
		charger_dev_do_event(info->chg1_dev, EVENT_RECHARGE, 0);

/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		for (i = 0; i < MAX_ALG_NO; i++) {
			alg = info->alg[i];
			if (alg == NULL)
				continue;
			notify.evt = EVT_RECHARGE;
			notify.value = 0;
			chg_alg_notifier_call(alg, &notify);
		}
#endif
/* End add by jin.wang */
	}

	return 0;
}

static int charger_dev_event(struct notifier_block *nb, unsigned long event,
				void *v)
{
	struct chg_alg_device *alg;
	struct chg_alg_notify notify;
	struct mtk_charger *info =
			container_of(nb, struct mtk_charger, chg1_nb);
	struct chgdev_notify *data = v;
	int i;

	chr_err("%s %d\n", __func__, event);

	switch (event) {
	case CHARGER_DEV_NOTIFY_EOC:
		notify.evt = EVT_FULL;
		notify.value = 0;
	for (i = 0; i < 10; i++) {
		alg = info->alg[i];
		chg_alg_notifier_call(alg, &notify);
	}

		break;
	case CHARGER_DEV_NOTIFY_RECHG:
		pr_info("%s: recharge\n", __func__);
		break;
	case CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT:
		info->safety_timeout = true;
		pr_info("%s: safety timer timeout\n", __func__);
		break;
	case CHARGER_DEV_NOTIFY_VBUS_OVP:
		info->vbusov_stat = data->vbusov_stat;
		pr_info("%s: vbus ovp = %d\n", __func__, info->vbusov_stat);
		break;
	default:
		return NOTIFY_DONE;
	}

	if (info->chg1_dev->is_polling_mode == false)
		_wake_up_charger(info);

	return NOTIFY_DONE;
}



int mtk_basic_charger_init(struct mtk_charger *info)
{
	info->algo.do_algorithm = do_algorithm;
	info->algo.enable_charging = enable_charging;
	info->algo.do_event = charger_dev_event;
	//info->change_current_setting = mtk_basic_charging_current;
	return 0;
}



