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

#include "mtk_pe2.h"
#include "mtk_charger_algorithm_class.h"
/* [BSP]Begin added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */
#include "charger_class.h"
/* [BSP]End added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */

#ifdef CONFIG_HELAEYE_BSP_CHARGER_ON
#include <tcl/tkperf.h>
#include <generated/utsrelease.h>
void heraeye_bat_check_temp(int errtype, int charger_driver_value);

#endif


/* Begin add by jin.wang task 2064 on 2021.11.18 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static int main_pct = -1;
module_param(main_pct, int, 0644);

static int force_ibus = -1;
module_param(force_ibus, int, 0644);

static int force_ibat = -1;
module_param(force_ibat, int, 0644);
#endif
/* End add by jin.wang */

static int pe2_dbg_level = PE2_DEBUG_LEVEL;

int pe2_get_debug_level(void)
{
	return pe2_dbg_level;
}

static int pe2_plugout_reset(struct chg_alg_device *alg)
{
	int ret = 0, cnt = 0;
	struct mtk_pe20 *pe2;

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !defined(CONFIG_TCT_CHARGER)
	pe2_dbg("%s\n", __func__);
#endif
/* End del by jin.wang */

	pe2 = dev_get_drvdata(&alg->dev);

/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if defined(CONFIG_TCT_CHARGER)
	pe2_dbg("%s: state=%d\n", __func__, pe2->state);
#endif
/* End add by jin.wang */

	switch (pe2->state) {
	case PE2_HW_UNINIT:
	case PE2_HW_FAIL:
/*start to del by weijun for pe20 adapter fast pluin/out when detect*/
#if !defined(CONFIG_TCT_CHARGER)
	case PE2_HW_READY:
#endif
/*end to del by weijun for pe20 adapter fast pluin/out when detect*/
		break;
	case PE2_TA_NOT_SUPPORT:
		pe2->state = PE2_HW_READY;
		break;
	case PE2_RUN:
	case PE2_TUNING:
	case PE2_POSTCC:
/*start to add by weijun for pe20 adapter fast pluin/out when detect*/
#if defined(CONFIG_TCT_CHARGER)
	case PE2_HW_READY:
#endif
/*end to add by weijun for pe20 adapter fast pluin/out when detect*/
		/* set flag to end PE2 thread asap */
		mutex_lock(&pe2->cable_out_lock);
		pe2->is_cable_out_occur = true;
		mutex_unlock(&pe2->cable_out_lock);

		while (mutex_trylock(&pe2->access_lock) == 0) {
			pe2_err("%s:pe2 is running state:%d cnt:%d\n",
				__func__, pe2->state,
				cnt);
			cnt++;
			msleep(100);
		}

		mutex_lock(&pe2->cable_out_lock);
		pe2->is_cable_out_occur = false;
		mutex_unlock(&pe2->cable_out_lock);

		pe2->idx = -1;
		pe2->vbus = 5000000; /* mV */

		/* Enable OVP */
		ret = pe2_hal_enable_vbus_ovp(alg, true);
		if (ret < 0)
			pe2_err("%s:enable vbus ovp fail, ret:%d\n",
				__func__, ret);

		/* Set MIVR for vbus 5V */
		ret = pe2_hal_set_mivr(alg, CHG1,
			pe2->min_charger_voltage); /* uV */
		if (ret < 0)
			pe2_err("%s:set mivr fail, ret:%d\n",
				__func__, ret);

/* Begin mod by jin.wang task 2064 on 2022.4.1 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		if ((alg->config == DUAL_CHARGERS_IN_PARALLEL)
			|| (alg->config == DUAL_CHARGERS_IN_SERIES)) {
			pe2_hal_enable_charger(alg, CHG2, false);
			pe2_hal_charger_enable_chip(alg, CHG2, false);
		}
#else
		if (alg->config == DUAL_CHARGERS_IN_SERIES) {
			pe2_hal_enable_charger(alg, CHG2, false);
			pe2_hal_charger_enable_chip(alg, CHG2, false);
		}
#endif
/* End mod by jin.wang */
		pe2_dbg("%s: OK\n", __func__);
		pe2->state = PE2_HW_READY;
		mutex_unlock(&pe2->access_lock);
		break;
	default:
		pe2_dbg("%s: error state:%d\n", __func__,
			pe2->state);
		break;
	}
	return 0;
}

int pe2_reset_ta_vchr(struct chg_alg_device *alg)
{
	int ret, chr_volt = 0, ret_value = -1;
	u32 retry_cnt = 0;
	struct mtk_pe20 *pe2;
	int chg_cnt, i, is_chip_enabled;

	pe2_dbg("%s: starts\n", __func__);
	pe2 = dev_get_drvdata(&alg->dev);

	ret = pe2_hal_set_mivr(alg, 0, pe2->min_charger_voltage);
	if (ret < 0)
		pe2_err("%s:set mivr fail, ret:%d\n",
			__func__, ret);

	/* Reset TA's charging voltage */
	do {
		chg_cnt = pe2_hal_get_charger_cnt(alg);
/* Begin mod by jin.wang task 2064 on 2022.4.1 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		if (chg_cnt > 1 && ((alg->config == DUAL_CHARGERS_IN_PARALLEL)
			|| (alg->config == DUAL_CHARGERS_IN_SERIES))) {
#else
		if (chg_cnt > 1 && alg->config == DUAL_CHARGERS_IN_SERIES) {
#endif
/* End mod by jin.wang */
			for (i = CHG2; i < CHG_MAX; i++) {
				is_chip_enabled =
						pe2_hal_is_chip_enable(alg, i);
			/* [BSP]Begin modified by bitao.xiong for SNTTF-635 on 2022/10/29 */
			#if IS_ENABLED(CONFIG_TCT_CHARGER)
				if (is_chip_enabled) {
					pe2_hal_enable_charger(alg, i, false);
					msleep(250);
				}
			#else
				if (is_chip_enabled)
					pe2_hal_enable_charger(alg, i, false);
			#endif
			/* [BSP]End modified by bitao.xiong for SNTTF-635 on 2022/10/29 */
			}
		}

		pe2_hal_reset_ta(alg, CHG1);
		if (ret < 0)
			pe2_err("%s:reset TA fail, ret:%d\n",
				__func__, ret);
		msleep(250);

		/* Check charger's voltage */
		chr_volt = pe2_hal_get_vbus(alg);
		pe2_dbg("%s: ori_vbus:%d vbus:%d\n", __func__,
			pe2->ta_vchr_org, chr_volt);
		if (abs(chr_volt - pe2->ta_vchr_org) <= 1000000) {
			pe2->vbus = chr_volt;
			pe2->idx = -1;
			ret_value = 0;
			break;
		}
		retry_cnt++;
	} while (retry_cnt < 3);

	if (ret_value != 0) {
		pe2_err("%s: failed, ret = %d\n", __func__, ret);
		ret = pe2_hal_set_mivr(alg, CHG1, pe2->vbus - 500000);
		if (ret < 0)
			pe2_err("%s:set mivr fail, ret:%d\n",
			__func__, ret);
		return -EHAL;
	}
	/* Measure VBAT */
	pe2->vbat_orig = pe2_hal_get_vbus(alg);
	pe2_dbg("%s: OK\n", __func__);

	return ret;
}

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static void pe2_check_cable_impedance(struct chg_alg_device *alg)
{
	int ret = 0;
	int vchr1, vchr2, cable_imp;
	unsigned int aicr_value;
	bool mivr_state = false;
	struct timespec ptime[2], diff;
	struct mtk_pe20 *pe2;
	int input_current;


	pe2_dbg("%s: starts type:%d\n", __func__,
		alg->config);
	pe2 = dev_get_drvdata(&alg->dev);

	if (alg->config == SINGLE_CHARGER)
		input_current = pe2->sc_input_current;
	else if (alg->config == DUAL_CHARGERS_IN_SERIES)
		input_current = pe2->dcs_input_current;
	else
		input_current = pe2->sc_input_current;

	if (pe2->vbat_orig > pe2->vbat_cable_imp_threshold) {
		pe2_info("VBAT > %dmV, directly set aicr to %dmA\n",
			pe2->vbat_cable_imp_threshold / 1000,
			input_current / 1000);
		pe2->aicr_cable_imp = input_current;
		goto end;
	}

	/* Disable cable drop compensation */
	pe2_hal_enable_cable_drop_comp(alg, false);

	get_monotonic_boottime(&ptime[0]);

	/* Set ichg = 2500mA, set MIVR */
	pe2_hal_set_charging_current(alg, CHG1, 2500000);
	mdelay(240);
	pe2_hal_set_mivr(alg, CHG1, pe2->min_charger_voltage);

	get_monotonic_boottime(&ptime[1]);
	diff = timespec_sub(ptime[1], ptime[0]);

	aicr_value = 800000;
	pe2_hal_set_input_current(alg, CHG1, aicr_value);

	/* To wait for soft-start */
	msleep(150);

	ret = pe2_hal_get_mivr_state(alg, CHG1, &mivr_state);
	if (ret != -ENOTSUPP && mivr_state) {
		pe2->aicr_cable_imp = 1000000;
		goto end;
	}

	vchr1 = pe2_hal_get_vbus(alg);

	aicr_value = 500000;
	pe2_hal_set_input_current(alg, CHG1, aicr_value);
	msleep(20);

	vchr2 = pe2_hal_get_vbus(alg);

	/*
	 * Calculate cable impedance (|V1 - V2|) / (|I2 - I1|)
	 * m_ohm = (mv * 10 * 1000) / (mA * 10)
	 * m_ohm = (uV * 10) / (mA * 10)
	 */
	cable_imp = (abs(vchr1 - vchr2) * 10) / (7400 - 4625);

	pe2_info("%s: cable_imp:%d mohm, vchr1:%d, vchr2:%d, time:%ld\n",
		__func__, cable_imp, vchr1 / 1000, vchr2 / 1000,
		diff.tv_nsec);

	/* Recover cable drop compensation */
	aicr_value = 100000;
	pe2_hal_set_input_current(alg, CHG1, aicr_value);
	msleep(250);

	if (cable_imp < pe2->cable_imp_threshold) {
		pe2->aicr_cable_imp = input_current;
		pe2_info("Normal cable\n");
	} else {
		pe2->aicr_cable_imp = 1000000; /* uA */
		pe2_info("Bad cable\n");
	}
	pe2_hal_set_input_current(alg, CHG1, pe2->aicr_cable_imp);

	pe2_info("%s: set aicr:%dmA, vbat:%dmV, mivr_state:%d\n",
		__func__, pe2->aicr_cable_imp / 1000,
		pe2->vbat_orig / 1000, mivr_state);
	return;

end:
	pe2_info("%s not started: set aicr:%dmA, vbat:%dmV, mivr_state:%d\n",
		__func__, pe2->aicr_cable_imp / 1000,
		pe2->vbat_orig / 1000, mivr_state);
}
#endif
/* End del by jin.wang */

static int _pe20_set_ta_vchr(struct chg_alg_device *alg, u32 chr_volt)
{
	int ret = 0, ret_value = 0;
	struct mtk_pe20 *pe2;
	int chg_cnt, i;
	bool is_chip_enabled;

	pe2_dbg("%s: starts\n", __func__);
	pe2 = dev_get_drvdata(&alg->dev);

	/* Not to set chr volt if cable is plugged out */
	if (pe2->is_cable_out_occur) {
		pe2_err("%s: failed, cable out\n", __func__);
		return -ECABLEOUT;
	}

	chg_cnt = pe2_hal_get_charger_cnt(alg);
/* Begin mod by jin.wang task 2064 on 2022.4.1 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (chg_cnt > 1 && ((alg->config == DUAL_CHARGERS_IN_PARALLEL)
		|| (alg->config == DUAL_CHARGERS_IN_SERIES))) {
#else
	if (chg_cnt > 1 && alg->config == DUAL_CHARGERS_IN_SERIES) {
#endif
/* End mod by jin.wang */
		for (i = CHG2; i < CHG_MAX; i++) {
			is_chip_enabled = pe2_hal_is_chip_enable(alg, i);
			if (is_chip_enabled) {
				ret = pe2_hal_enable_charger(alg, i, false);
				if (ret < 0) {
					pe2_err("%s:disable chip fail,idx:%d ret:%d\n",
					__func__, i, ret);
					ret_value = -EHAL;
				}
			}
		}
	}
	ret = pe2_hal_send_ta20_current_pattern(alg, chr_volt);
	if (ret < 0) {
		pe2_err("%s: pattern failed, ret = %d\n", __func__, ret);
		ret_value = -EHAL;
	}

	return ret_value;
}

static int pe20_set_ta_vchr(struct chg_alg_device *alg, u32 chr_volt)
{
	int ret = 0, ret_value = 0;
	int vchr_before, vchr_after, vchr_delta;

/* Begin mod by jin.wang task 5195 on 2022.1.6 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	const u32 sw_retry_cnt_max = 0;
	const u32 retry_cnt_max = 5;
#else
	const u32 sw_retry_cnt_max = 3;
	const u32 retry_cnt_max = 5;
#endif
/* End mod by jin.wang */

	u32 sw_retry_cnt = 0, retry_cnt = 0;
	struct mtk_pe20 *pe2;

	pe2_dbg("%s: starts, target:%d\n", __func__, chr_volt / 1000);
	pe2 = dev_get_drvdata(&alg->dev);

	do {
		vchr_before = pe2_hal_get_vbus(alg);
		ret = _pe20_set_ta_vchr(alg, chr_volt);
		vchr_after = pe2_hal_get_vbus(alg);

		vchr_delta = abs(vchr_after - chr_volt);

		/*
		 * It is successful if VBUS difference to target is
		 * less than 500mV.
		 */
		 pe2_err("ret:%d delta:%d %d %d\n",
		 ret, vchr_delta,
		 vchr_after, chr_volt);

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		if (vchr_delta < 1000000 && ret == 0) {
#else
		if (vchr_delta < 500000 && ret == 0) {
#endif
/* End mod by jin.wang */
			pe2_dbg("%s: OK, vchr = (%d, %d), vchr_target = %dmV\n",
				__func__, vchr_before / 1000, vchr_after / 1000,
				chr_volt / 1000);
			return ret_value;
		}

		if (ret == 0 || sw_retry_cnt >= sw_retry_cnt_max)
			retry_cnt++;
		else
			sw_retry_cnt++;

		ret = pe2_hal_set_mivr(alg, CHG1, pe2->min_charger_voltage);
		if (ret < 0)
			pe2_err("%s:set mivr fail, ret:%d\n",
			__func__, ret);

		pe2_dbg("%s: retry_cnt = (%d, %d), vchr = (%d, %d), vchr_target = %dmV\n",
			__func__, sw_retry_cnt, retry_cnt, vchr_before / 1000,
			vchr_after / 1000, chr_volt / 1000);
		pe2_dbg("%s: %d %d %d %d\n", __func__,
			pe2->is_cable_out_occur,
			pe2_hal_get_charger_type(alg),
			retry_cnt,
			retry_cnt_max);

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	} while (!pe2->is_cable_out_occur &&
		 (pe2_hal_get_charger_type(alg) ==
		 POWER_SUPPLY_TYPE_USB_DCP) &&
		 (retry_cnt < retry_cnt_max));
#else
	} while (!pe2->is_cable_out_occur &&
		 (pe2_hal_get_charger_type(alg) !=
		 POWER_SUPPLY_TYPE_USB_DCP) &&
		 (retry_cnt < retry_cnt_max));
#endif
/* End mod by jin.wang */

	if (pe2->is_cable_out_occur)
		ret_value = -ECABLEOUT;
	else
		ret_value = -EHAL;

	pe2_dbg("%s: failed, vchr_org = %dmV, vchr_after = %dmV, target_vchr = %dmV\n",
		__func__, pe2->ta_vchr_org / 1000, vchr_after / 1000,
		chr_volt / 1000);

	return ret_value;
}


static int pe20_detect_ta(struct chg_alg_device *alg)
{
	int ret = 0;
	struct mtk_pe20 *pe2;

	pe2_dbg("%s: starts\n", __func__);

	pe2 = dev_get_drvdata(&alg->dev);
	pe2->ta_vchr_org = pe2_hal_get_vbus(alg);

	/* Disable OVP */
	ret = pe2_hal_enable_vbus_ovp(alg, false);
	if (ret < 0)
		goto err;
	if (abs(pe2->ta_vchr_org - 8500000) > 500000)
		ret = pe20_set_ta_vchr(alg, 8500000);
	else
		ret = pe20_set_ta_vchr(alg, 6500000);

	pe2_dbg("%s: ret:%d\n", __func__, ret);

	return ret;
err:
	pe2_hal_enable_vbus_ovp(alg, true);
	pe2_err("%s: failed, ret = %d\n", __func__, ret);
	return ret;
}

/* Begin added by hailong.chen for task 9209018 on 2020-04-14 */
#if (defined (CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT ) && defined (CONFIG_CHARGER_TYPE_MT6357))
extern unsigned int disable_vsrc_for_pe2(void);
#endif
/* End added by hailong.chen for task 9209018 on 2020-04-14 */

static int __pe2_check_charger(struct chg_alg_device *alg)
{
/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	int ret = 0;
#else
	int ret = 0, ret_value = 0;
#endif
/* End mod by jin.wang */

	struct mtk_pe20 *pe2;
/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	int uisoc;
#endif
/* End del by jin.wang */

    /* Begin added by hailong.chen for task 9209018 on 2020-04-14 */
#if (IS_ENABLED (CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT ) \
	&& IS_ENABLED (CONFIG_CHARGER_TYPE_MT6357))
    unsigned int vsrc_value = 0;
#endif
    /* End added by hailong.chen for task 9209018 on 2020-04-14 */

	pe2 = dev_get_drvdata(&alg->dev);

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	uisoc = pe2_hal_get_uisoc(alg);
#endif
/* End del by jin.wang */

    /* Begin added by hailong.chen for task 9209018 on 2020-04-14 */
#if (IS_ENABLED (CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT ) \
	&& IS_ENABLED (CONFIG_CHARGER_TYPE_MT6357))
    vsrc_value = disable_vsrc_for_pe2();
    if(vsrc_value)
        pe2_err("%s: disable_vsrc OK\n", __func__);
    else
        pe2_err("%s: disable_vsrc Err\n", __func__);
#endif
    /* End added by hailong.chen for task 9209018 on 2020-04-14 */

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pe2_err("%s uisoc:%d s:%d end:%d type:%d", __func__,
		uisoc,
		pe2->ta_start_battery_soc,
		pe2->ta_stop_battery_soc,
		pe2_hal_get_charger_type(alg));

	if (pe2_hal_get_charger_type(alg) !=
		POWER_SUPPLY_TYPE_USB_DCP) {
		ret_value = ALG_TA_NOT_SUPPORT;
		goto out;
	}
#endif
/* End del by jin.wang */

	if (pe2->is_cable_out_occur)
		goto out;

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if (uisoc < pe2->ta_start_battery_soc ||
		uisoc >= pe2->ta_stop_battery_soc) {
		ret_value = ALG_TA_CHECKING;
		goto out;
	}
#endif
/* End del by jin.wang */

	ret = pe2_reset_ta_vchr(alg);
	if (ret != 0)
		goto out;

	if (pe2->is_cable_out_occur)
		goto out;

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pe2_check_cable_impedance(alg);

	if (pe2->is_cable_out_occur)
		goto out;
#endif
/* End del by jin.wang */

	ret = pe20_detect_ta(alg);
	if (ret < 0)
		goto out;

	pe2_dbg("%s: OK, state = %d\n",
		__func__, pe2->state);

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	return 0;
#else
	return ret;
#endif
/* End mod by jin.wang */

out:

/* Begin del by jin.wang task 2064 on 2021.10.25 */
#if !defined(CONFIG_TCT_CHARGER)
	if (ret_value == 0)
		ret_value = ALG_TA_NOT_SUPPORT;
#endif
/* End del by jin.wang */

	pe2_dbg("%s:SOC:(%d,%d,%d),state:%d,chr_type:%d,ret:%d,plugout:%d\n",
		__func__,
		pe2_hal_get_uisoc(alg),
		pe2->ta_start_battery_soc,
		pe2->ta_stop_battery_soc,
		pe2->state,
		pe2_hal_get_charger_type(alg),
		ret,
		pe2->is_cable_out_occur);

/* Begin mod by jin.wang task 2064 on 2021.10.25 */
#if defined(CONFIG_TCT_CHARGER)
	return ALG_TA_NOT_SUPPORT;
#else
	return ret_value;
#endif
/* End mod by jin.wang */
}

static int pe2_leave(struct chg_alg_device *alg)
{
	int ret = 0;
	struct mtk_pe20 *pe2;

	pe2_dbg("%s: starts\n", __func__);
	pe2 = dev_get_drvdata(&alg->dev);

	ret = pe2_reset_ta_vchr(alg);
	if (ret != 0) {
		pe2_err("%s: failed, state = %d, ret = %d\n",
			__func__, pe2->state, ret);
	}

	ret = pe2_hal_enable_vbus_ovp(alg, true);
	if (ret != 0) {
		pe2_err("%s: enable vbus ovp fai,ret:%d\n",
			__func__, ret);
	}

	pe2_hal_set_mivr(alg, CHG1, pe2->min_charger_voltage);
	if (ret != 0) {
		pe2_err("%s:set mivr fail,ret:%d\n",
			__func__, ret);
	}

	pe2_dbg("%s: OK\n", __func__);
	return ret;
}

static int _pe2_init_algo(struct chg_alg_device *alg)
{
	struct mtk_pe20 *pe2;
	int ret, cnt;

	pe2 = dev_get_drvdata(&alg->dev);
	mutex_lock(&pe2->access_lock);
	if (pe2_hal_init_hardware(alg) != 0) {
		pe2->state = PE2_HW_FAIL;
		pe2_err("%s:init hw fail\n", __func__);
	} else
		pe2->state = PE2_HW_READY;
	ret = pe2_hal_set_efficiency_table(pe2->alg);
	if (ret != 0)
		pe2_err("%s: use default table, %d\n", __func__, ret);

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if ((alg->config == DUAL_CHARGERS_IN_PARALLEL)
		|| (alg->config == DUAL_CHARGERS_IN_SERIES)) {
		cnt = pe2_hal_get_charger_cnt(alg);
		if (cnt != 2)
			alg->config = SINGLE_CHARGER;
	} else
		alg->config = SINGLE_CHARGER;
#else
	if (alg->config == DUAL_CHARGERS_IN_PARALLEL) {
		pe2_err("%s does not support DUAL_CHARGERS_IN_PARALLEL\n",
			__func__);
		alg->config = SINGLE_CHARGER;
	} else if (alg->config == DUAL_CHARGERS_IN_SERIES) {
		cnt = pe2_hal_get_charger_cnt(alg);
		if (cnt == 2)
			alg->config = DUAL_CHARGERS_IN_SERIES;
		else
			alg->config = SINGLE_CHARGER;
	} else
		alg->config = SINGLE_CHARGER;
#endif
/* End mod by jin.wang */

	mutex_unlock(&pe2->access_lock);
	pe2_dbg("%s config:%d\n", __func__, alg->config);
	return 0;
}

static char *pe2_state_to_str(int state)
{
	switch (state) {
	case PE2_HW_UNINIT:
		return "PE2_HW_UNINIT";
	case PE2_HW_FAIL:
		return "PE2_HW_FAIL";
	case PE2_HW_READY:
		return "PE2_HW_READY";
/* Begin add by jin.wang task 2064 on 2021.11.18 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	case PE2_TA_CHECKING:
		return "PE2_TA_CHECKING";
#endif
/* End add by jin.wang */
	case PE2_TA_NOT_SUPPORT:
		return "PE2_TA_NOT_SUPPORT";
	case PE2_RUN:
		return "PE2_RUN";
	case PE2_TUNING:
		return "PE2_TUNING";
	case PE2_POSTCC:
		return "PE2_POSTCC";
	default:
		break;
	}
	pe2_err("%s unknown state:%d\n", __func__
		, state);
	return "PE2_UNKNOWN";
}

/* Begin add by jin.wang task 2064 on 2021.11.18 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static int __pe2_is_algo_ready(struct chg_alg_device *alg, struct mtk_pe20 *pe2)
{
	int uisoc;

	uisoc = pe2_hal_get_uisoc(alg);
	if (uisoc < 0) {
		pe2_err("%s unknown uisoc %d\n", __func__, uisoc);
		return ALG_TA_CHECKING;
	}

	if (pe2_hal_get_charger_type(alg) !=
		POWER_SUPPLY_TYPE_USB_DCP) {
		return ALG_TA_NOT_SUPPORT;
	} else if (uisoc >= pe2->ta_stop_battery_soc) {
		return ALG_TA_NOT_SUPPORT;
	}

	return ALG_READY;
}
#endif
/* End add by jin.wang */

static int _pe2_is_algo_ready(struct chg_alg_device *alg)
{
	struct mtk_pe20 *pe2;

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	int ret_value;
#else
	int ret_value, uisoc;
#endif
/* End mod by jin.wang */

	pe2 = dev_get_drvdata(&alg->dev);

	mutex_lock(&pe2->access_lock);
	__pm_stay_awake(pe2->suspend_lock);
	pe2_dbg("%s state:%s\n", __func__,
		pe2_state_to_str(pe2->state));

	switch (pe2->state) {
	case PE2_HW_UNINIT:
	case PE2_HW_FAIL:
		ret_value = ALG_INIT_FAIL;
		break;
	case PE2_HW_READY:
/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		ret_value = __pe2_is_algo_ready(alg, pe2);
#else
		uisoc = pe2_hal_get_uisoc(alg);
		if (pe2_hal_get_charger_type(alg) !=
			POWER_SUPPLY_TYPE_USB_DCP) {
			ret_value = ALG_TA_NOT_SUPPORT;
		} else if (uisoc < pe2->ta_start_battery_soc ||
			uisoc >= pe2->ta_stop_battery_soc ||
			pe2->charging_current_limit1 != -1 ||
			pe2->charging_current_limit2 != -1) {
			ret_value = ALG_NOT_READY;
		} else {
			ret_value = ALG_READY;
		}
#endif
/* End mod by jin.wang */
		break;
/* Begin add by jin.wang task 2064 on 2021.11.18 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	case PE2_TA_CHECKING:
		ret_value = __pe2_is_algo_ready(alg, pe2);
		break;
#endif
/* End add by jin.wang */
	case PE2_TA_NOT_SUPPORT:
		ret_value = ALG_TA_NOT_SUPPORT;
		break;
	case PE2_RUN:
	case PE2_TUNING:
	case PE2_POSTCC:
/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		ret_value = __pe2_is_algo_ready(alg, pe2);
		if (ret_value == ALG_READY)
			ret_value = ALG_RUNNING;
#else
		ret_value = ALG_RUNNING;
#endif
/* End mod by jin.wang */
		break;
	default:
		ret_value = ALG_INIT_FAIL;
		break;
	}
	__pm_relax(pe2->suspend_lock);
	mutex_unlock(&pe2->access_lock);

	return ret_value;
}

static int pe2_sc_set_charger(struct chg_alg_device *alg)
{
	struct mtk_pe20 *pe2;
	int ichg1_min = -1, aicr1_min = -1;
	int ret;

	pe2 = dev_get_drvdata(&alg->dev);

	if (pe2->input_current_limit1 == 0 ||
		pe2->charging_current_limit1 == 0) {
		pr_notice("input/charging current is 0, end PE2\n");
		return -1;
	}

	mutex_lock(&pe2->data_lock);
	if (pe2->charging_current_limit1 != -1) {
		if (pe2->charging_current_limit1 <=
			pe2->sc_charger_current)
			pe2->charging_current1 =
				pe2->charging_current_limit1;
		ret = pe2_hal_get_min_charging_current(alg, CHG1, &ichg1_min);
		if (ret != -ENOTSUPP &&
			pe2->charging_current_limit1 < ichg1_min)
			pe2->charging_current1 = 0;
	} else {
		pe2->charging_current1 = pe2->sc_charger_current;
	}

	if (pe2->input_current_limit1 != -1 &&
		pe2->input_current_limit1 <=
		pe2->sc_input_current) {
		pe2->input_current1 = pe2->input_current_limit1;
		ret = pe2_hal_get_min_input_current(alg, CHG1, &aicr1_min);
		if (ret != -ENOTSUPP &&
			pe2->input_current_limit1 < aicr1_min)
			pe2->input_current1 = 0;
	} else {
		pe2->input_current1 = pe2->sc_input_current;
	}

	mutex_unlock(&pe2->data_lock);

	if (pe2->input_current1 == 0 ||
		pe2->charging_current1 == 0) {
		pe2_err("current is zero %d %d\n",
			pe2->input_current1,
			pe2->charging_current1);
		return -1;
	}

/* Begin mod by jin.wang task 2064 on 2021.11.19 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pe2_hal_set_charging_current(alg, CHG1,
		((force_ibat == -1) ?
			pe2->charging_current1 : force_ibat));
	pe2_hal_set_input_current(alg, CHG1,
		((force_ibus == -1) ?
			pe2->input_current1 : force_ibus));
#else
	pe2_hal_set_charging_current(alg,
		CHG1, pe2->charging_current1);
	pe2_hal_set_input_current(alg,
		CHG1, pe2->input_current1);
#endif
/* End mod by jin.wang */
	pe2_hal_set_cv(alg,
		CHG1, pe2->cv);

	pe2_dbg("%s m:%d s:%d cv:%d chg1:%d,%d min:%d:%d\n", __func__,
		alg->config,
		pe2->state,
		pe2->cv,
		pe2->input_current1,
		pe2->charging_current1,
		ichg1_min,
		aicr1_min);

	return 0;
}

static int pe2_dcs_set_charger(struct chg_alg_device *alg)
{
	struct mtk_pe20 *pe2;
	//bool chg1_enable = true;
	bool chg2_enable = true;
	bool chg2_chip_enabled = false;
	int ret;
	int ichg1_min = -1, ichg2_min = -1;
/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	int aicr1_min = -1, aicr2_min = -1;
	int total_ibat = 0;
#else
	int aicr1_min = -1;
#endif
/* End mod by jin.wang */
	pe2 = dev_get_drvdata(&alg->dev);

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if (pe2->input_current_limit1 == 0 ||
		pe2->input_current_limit2 == 0 ||
		pe2->charging_current_limit1 == 0 ||
		pe2->charging_current_limit2 == 0) {
		pr_notice("input/charging current is 0, end PE2\n");
		return -1;
	}
#else
	if (pe2->input_current_limit1 == 0 ||
		pe2->charging_current_limit1 == 0 ||
		pe2->charging_current_limit2 == 0) {
		pr_notice("input/charging current is 0, end PE2\n");
		return -1;
	}
#endif
/* End mod by jin.wang */

	mutex_lock(&pe2->data_lock);

/* Begin mod by jin.wang task 2064 on 2021.11.18 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if (likely(force_ibat == -1)) {
		total_ibat = pe2->dcs_chg1_charger_current
				+ pe2->dcs_chg2_charger_current;
	} else {
		total_ibat = force_ibat;
	}

	pe2->dcs_chg1_charger_current =
		(total_ibat * pe2->main_pct / 100);
	pe2->dcs_chg2_charger_current =
		total_ibat - pe2->dcs_chg1_charger_current;

	if (force_ibus != -1)
		pe2->dcs_input_current = force_ibus;

	if (pe2->input_current_limit1 != -1 &&
		pe2->input_current_limit1 <
				(pe2->dcs_input_current * pe2->main_pct / 100)) {
			pe2->input_current1 = pe2->input_current_limit1;
	} else {
		pe2->input_current1 =
			(pe2->dcs_input_current * pe2->main_pct / 100);
	}
	ret = pe2_hal_get_min_input_current(alg, CHG1, &aicr1_min);
	if (ret != -ENOTSUPP &&
		pe2->input_current1 < aicr1_min) {
		pe2->input_current1 = 0;
	}

	if (pe2->charging_current_limit1 != -1 &&
		pe2->charging_current_limit1 <
		pe2->dcs_chg1_charger_current) {
		pe2->charging_current1 = pe2->charging_current_limit1;
	} else {
		pe2->charging_current1 = pe2->dcs_chg1_charger_current;
	}
	ret = pe2_hal_get_min_charging_current(alg, CHG1, &ichg1_min);
	if (ret != -ENOTSUPP &&
		pe2->charging_current1 < ichg1_min) {
		pe2->charging_current1 = 0;
	}

	if (pe2->input_current_limit2 != -1 &&
		pe2->input_current_limit2 <
			(pe2->dcs_input_current * (100 - pe2->main_pct) / 100)) {
		pe2->input_current2 = pe2->input_current_limit2;
	} else {
		pe2->input_current2 =
			(pe2->dcs_input_current * (100 - pe2->main_pct) / 100);
	}
	ret = pe2_hal_get_min_input_current(alg, CHG2, &aicr2_min);
	if (ret != -ENOTSUPP &&
		pe2->input_current2 < aicr2_min) {
		pe2->input_current2 = 0;
	}

	if (pe2->charging_current_limit2 != -1 &&
		pe2->charging_current_limit2 <
		pe2->dcs_chg2_charger_current) {
		pe2->charging_current2 = pe2->charging_current_limit2;
	} else {
		pe2->charging_current2 = pe2->dcs_chg2_charger_current;
	}
	ret = pe2_hal_get_min_charging_current(alg, CHG2, &ichg2_min);
	if (ret != -ENOTSUPP &&
		pe2->charging_current2 < ichg2_min) {
		pe2->charging_current2 = 0;
	}
#else
	if (pe2->input_current_limit1 != -1 &&
		pe2->input_current_limit1 <
		pe2->dcs_input_current) {
		pe2->input_current1 = pe2->input_current_limit1;
		ret = pe2_hal_get_min_input_current(alg, CHG1, &aicr1_min);
		if (ret != -ENOTSUPP &&
			pe2->input_current_limit1 < aicr1_min)
			pe2->input_current1 = 0;
	} else
		pe2->input_current1 = pe2->dcs_input_current;

	if (pe2->charging_current_limit1 != -1 &&
		pe2->charging_current_limit1 <
		pe2->dcs_chg1_charger_current) {
		pe2->charging_current1 = pe2->charging_current_limit1;
		ret = pe2_hal_get_min_charging_current(alg, CHG1, &ichg1_min);
		if (ret != -ENOTSUPP &&
			pe2->charging_current_limit1 < ichg1_min)
			pe2->charging_current1 = 0;
	} else
		pe2->charging_current1 = pe2->dcs_chg1_charger_current;
#endif
/* End mod by jin.wang */

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if (pe2->state == PE2_RUN)
		pe2->charging_current2 = pe2->dcs_chg2_charger_current;

	if (pe2->charging_current_limit2 != -1 &&
		pe2->charging_current_limit2 <
		pe2->charging_current2) {
		pe2->charging_current2 = pe2->charging_current_limit2;
		ret = pe2_hal_get_min_charging_current(alg, CHG2, &ichg1_min);
		if (ret != -ENOTSUPP &&
			pe2->charging_current_limit2 < ichg1_min)
			pe2->charging_current2 = 0;
	}
#endif
/* End del by jin.wang */

	mutex_unlock(&pe2->data_lock);

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if (pe2->input_current1 == 0 ||
		pe2->input_current2 == 0 ||
		pe2->charging_current1 == 0 ||
		pe2->charging_current2 == 0) {
		pe2_err("current is zero %d %d %d %d\n",
			pe2->input_current1,
			pe2->input_current2,
			pe2->charging_current1,
			pe2->charging_current2);
		pe2_hal_enable_charger(alg, CHG2, false);
		pe2_hal_charger_enable_chip(alg, CHG2, false);
		return -1;
	}
#else
	if (pe2->input_current1 == 0 ||
		pe2->charging_current1 == 0 ||
		pe2->charging_current2 == 0) {
		pe2_err("current is zero %d %d %d\n",
			pe2->input_current1,
			pe2->charging_current1,
			pe2->charging_current2);
		pe2_hal_enable_charger(alg, CHG2, false);
		pe2_hal_charger_enable_chip(alg, CHG2, false);
		return -1;
	}
#endif
/* End mod by jin.wang */

	chg2_chip_enabled = pe2_hal_is_chip_enable(alg, CHG2);
	pe2_err("chg2_en:%d %d %d\n",
		chg2_enable, chg2_chip_enabled, pe2->state);
	if (pe2->state == PE2_RUN) {
		if (!chg2_chip_enabled)
			pe2_hal_charger_enable_chip(alg, CHG2, true);

/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		pe2_hal_set_cv(alg, CHG2, pe2->cv);
#endif
/* End add by jin.wang */

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		pe2_hal_enable_charger(alg, CHG2, true);
#endif
/* End del by jin.wang */

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		pe2_hal_set_input_current(alg,
			CHG2, pe2->input_current2);
#else
		pe2_hal_set_input_current(alg,
			CHG2, pe2->charging_current2);
#endif
/* End mod by jin.wang */

		pe2_hal_set_charging_current(alg,
			CHG2, pe2->charging_current2);

/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		pe2_hal_enable_charger(alg, CHG2, true);
#endif
/* End add by jin.wang */

/* Begin add by weijun for eoc on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
               pe2_hal_set_eoc_current(alg, CHG1,
                       pe2->dual_polling_ieoc);
               pe2_hal_enable_termination(alg, CHG1, true);
               pe2_hal_safety_check(alg, pe2->dual_polling_ieoc);
#else
		pe2_hal_set_eoc_current(alg, CHG1,
			pe2->dual_polling_ieoc);
		pe2_hal_enable_termination(alg, CHG1, false);
		pe2_hal_safety_check(alg, pe2->dual_polling_ieoc);
#endif
/* End add by weijun */
	} else if (pe2->state == PE2_TUNING) {
		if (!chg2_chip_enabled)
			pe2_hal_charger_enable_chip(alg, CHG2, true);
		pe2_hal_enable_charger(alg, CHG2, true);
		pe2_hal_set_eoc_current(alg, CHG1, pe2->dual_polling_ieoc);
		pe2_hal_enable_termination(alg, CHG1, false);
		pe2_hal_safety_check(alg, pe2->dual_polling_ieoc);
	} else if (pe2->state == PE2_POSTCC) {
		pe2_hal_set_eoc_current(alg, CHG1, 150000);
		pe2_hal_enable_termination(alg, CHG1, true);
	} else {
		pe2_err("%s state error!", __func__);
		return -1;
	}

	pe2_hal_set_charging_current(alg,
		CHG1, pe2->charging_current1);
	pe2_hal_set_input_current(alg,
		CHG1, pe2->input_current1);
	pe2_hal_set_cv(alg,
		CHG1, pe2->cv);

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pe2_dbg("%s m:%d s:%d cv:%d chg1:%d,%d chg2:%d,%d chg2en:%d min:%d,%d,%d,%d\n",
		__func__,
		alg->config,
		pe2->state,
		pe2->cv,
		pe2->input_current1,
		pe2->charging_current1,
		pe2->input_current2,
		pe2->charging_current2,
		chg2_enable,
		ichg1_min,
		ichg2_min,
		aicr1_min,
		aicr2_min);
#else
	pe2_dbg("%s m:%d s:%d cv:%d chg1:%d,%d chg2:%d,%d chg2en:%d min:%d,%d,%d\n",
		__func__,
		alg->config,
		pe2->state,
		pe2->cv,
		pe2->input_current1,
		pe2->charging_current1,
		pe2->input_current2,
		pe2->charging_current2,
		chg2_enable,
		ichg1_min,
		ichg2_min,
		aicr1_min);
#endif
/* End mod by jin.wang */
	return 0;
}

/* [BSP]Begin added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */
static bool primary_chg_is_sy6970qcc(void)
{
	static struct charger_device *primary_chg = NULL;

	if (IS_ERR_OR_NULL(primary_chg)) {
		primary_chg = get_charger_by_name("primary_chg");
		if (IS_ERR_OR_NULL(primary_chg))
			pe2_err("Error: can't find primary chg\n");
	}

	if (!IS_ERR_OR_NULL(primary_chg) && strlen(primary_chg->props.alias_name) != 0)
		return !strncmp(primary_chg->props.alias_name, "sy6970qcc", strlen("sy6970qcc"));
	else
		return false;
}
/* [BSP]End added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */

static int __pe2_run(struct chg_alg_device *alg)
{
	struct mtk_pe20 *pe2;
	int i;
	int vbat, vbus, ichg;
	int pre_vbus, pre_idx;
	int tune = 0, pes = 0; /* For log, to know the state of PE+20 */
	u32 size;

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	int ret = 0, ret_value = 0, vchr;
	/* [BSP]Begin added by bitao.xiong for SNTTF-635 on 2022/10/29 */
	#if IS_ENABLED(CONFIG_TCT_PROJECT_SONATA)
	int uisoc;
	#endif
	/* [BSP]End added by bitao.xiong for SNTTF-635 on 2022/10/29 */
#else
	int ret = 0, ret_value = 0, vchr, uisoc;
#endif
/* End mod by jin.wang */

	pe2 = dev_get_drvdata(&alg->dev);

	if (pe2->is_cable_out_occur) {
		ret_value = ALG_TA_NOT_SUPPORT;
		goto out;
	}

	vbat = pe2_hal_get_vbat(alg);
	vbus = pe2_hal_get_vbus(alg);
	ichg = pe2_hal_get_ibat(alg);

	pre_vbus = pe2->vbus;
	pre_idx = pe2->idx;

	/* PE+ leaves unexpectedly */
	vchr = pe2_hal_get_vbus(alg);
	if (abs(vchr - pe2->ta_vchr_org) < 1000000) {
		pe2_err("%s: PE+20 leave unexpectedly, recheck TA %d %d\n",
			__func__,
			vchr,
			pe2->ta_vchr_org);
		ret = pe2_leave(alg);
		pe2->ta_vchr_org = vchr;

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		ret_value = ALG_TA_NOT_SUPPORT;
#else
		ret_value = ALG_TA_CHECKING;
#endif
/* End mod by jin.wang */

#ifdef CONFIG_HELAEYE_BSP_CHARGER_ON
  heraeye_bat_check_temp(CHARGER_DRV_PE_ERROR,(vchr<<16)|(pe2->ta_vchr_org));
#endif

		goto out;
	}

/* Begin del by jin.wang task 2064 on 2021.11.2 */
/* [BSP]Begin modified by bitao.xiong for SNTTF-635 on 2022/10/29 */
#if IS_ENABLED(CONFIG_TCT_PROJECT_SONATA) || !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	ichg = pe2_hal_get_ibat(alg);
	/* Check SOC & Ichg */
	uisoc = pe2_hal_get_uisoc(alg);
	if (uisoc > pe2->ta_stop_battery_soc &&
		ichg > 0 && ichg < pe2->pe20_ichg_level_threshold) {
		ret = pe2_leave(alg);
		pe2_err("%s: OK, SOC = (%d,%d), stop PE+20\n", __func__,
			uisoc, pe2->ta_stop_battery_soc);
		ret_value = ALG_DONE;
		goto out;
	}
#endif
/* [BSP]End modified by bitao.xiong for SNTTF-635 on 2022/10/29 */
/* End del by jin.wang */

	/* [BSP]Begin added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */
	if (primary_chg_is_sy6970qcc() && vbat > pe2->cv * 102 / 100 && pe2->cv <= 4100000) {
		pe2_err("%s: sy6970qcc, vbat = %d,cv=%d, stop PE+20\n", __func__,
			vbat, pe2->cv);
		ret = pe2_leave(alg);
		ret_value = ALG_DONE;
		goto out;
	}
	/* [BSP]End added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */

	size = ARRAY_SIZE(pe2->profile);
	for (i = 0; i < size; i++) {
		tune = 0;

		/* Exceed this level, check next level */
		if (vbat > (pe2->profile[i].vbat + 100000))
			continue;

		/* If vbat is still 30mV larger than the lower level
		 * Do not down grade
		 */
		if (i < pe2->idx && vbat > (pe2->profile[i].vbat + 30000))
			continue;

		if (pe2->vbus != pe2->profile[i].vchr)
			tune = 1;

		pe2->vbus = pe2->profile[i].vchr;
		pe2->idx = i;

		if (abs(vbus - pe2->vbus) >= 1000000)
			tune = 2;

		if (tune != 0) {
			ret = pe20_set_ta_vchr(alg, pe2->vbus);
			if (ret == 0)
				pe2_hal_set_mivr(alg, CHG1, pe2->vbus - 500000);
			else {
				pe2_leave(alg);
				ret_value = ALG_TA_NOT_SUPPORT;
/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
				goto out;
#endif
/* End add by jin.wang */
			}
		} else
			pe2_hal_set_mivr(alg, CHG1, pe2->vbus - 500000);
		break;
	}
	pes = 2;

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if ((alg->config == DUAL_CHARGERS_IN_PARALLEL)
		|| (alg->config == DUAL_CHARGERS_IN_SERIES)) {
#else
	if (alg->config == DUAL_CHARGERS_IN_SERIES) {
#endif
/* End mod by jin.wang */
		if (pe2_dcs_set_charger(alg) != 0) {
			ret = pe2_leave(alg);
/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
			ret_value = ALG_TA_NOT_SUPPORT;
#else
			ret_value = ALG_DONE;
#endif
/* End mod by jin.wang */
			goto out;
		}
	} else {
		if (pe2_sc_set_charger(alg) != 0) {
			ret = pe2_leave(alg);
/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
			ret_value = ALG_TA_NOT_SUPPORT;
#else
			ret_value = ALG_DONE;
#endif
/* End mod by jin.wang */
			goto out;
		}
	}

/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	ret_value = ALG_DONE;
#endif
/* End add by jin.wang */

	pe2_dbg("%s cv:%d chg1:%d,%d chg2:%d,%d\n", __func__,
		pe2->cv,
		pe2->input_current1,
		pe2->charging_current1,
		pe2->input_current2,
		pe2->charging_current2);

out:

	pe2_dbg("%s: vbus = (%d, %d), idx = (%d, %d), I = %d plugout=%d\n",
		__func__, pre_vbus / 1000, pe2->vbus / 1000, pre_idx,
		pe2->idx, ichg / 1000, pe2->is_cable_out_occur);

	pe2_dbg("%s: SOC = %d, state = %s, tune = %d, pes = %d, vbat = %d, ret = %d:%d\n",
		__func__, pe2_hal_get_uisoc(alg),
		pe2_state_to_str(pe2->state),
		tune, pes,
		vbat / 1000, ret, ret_value);
	return ret_value;
}

static int _pe2_start_algo(struct chg_alg_device *alg)
{
	int ret = 0, ret_value = 0;
	struct mtk_pe20 *pe2;
	bool again;

	pe2 = dev_get_drvdata(&alg->dev);
	mutex_lock(&pe2->access_lock);
	__pm_stay_awake(pe2->suspend_lock);

	do {
		pe2_info("%s state:%d %s %d\n", __func__,
			pe2->state,
			pe2_state_to_str(pe2->state),
			again);

		again = false;
		switch (pe2->state) {
		case PE2_HW_UNINIT:
		case PE2_HW_FAIL:
			ret_value = ALG_INIT_FAIL;
			break;
		case PE2_HW_READY:
			ret = __pe2_check_charger(alg);
			if (ret == 0) {
				pe2->state = PE2_RUN;
				ret_value = ALG_READY;
				again = true;
			} else if (ret == ALG_TA_CHECKING)
				ret_value = ALG_TA_CHECKING;
/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
			else if (pe2->charging_current_limit1 != -1 ||
				pe2->charging_current_limit2 != -1)
				ret_value = ALG_NOT_READY;
#endif
/* End del by jin.wang */
			else {
				pe2->state = PE2_TA_NOT_SUPPORT;
				ret_value = ALG_TA_NOT_SUPPORT;
			}
			break;
		case PE2_TA_NOT_SUPPORT:
			ret_value = ALG_TA_NOT_SUPPORT;
			break;
		case PE2_RUN:
		case PE2_TUNING:
		case PE2_POSTCC:
			ret = __pe2_run(alg);

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
			if (ret == ALG_DONE) {
				pe2->state = PE2_RUN;
				ret_value = ALG_RUNNING;
			} else {
				pe2->state = PE2_TA_NOT_SUPPORT;
				ret_value = ALG_TA_NOT_SUPPORT;
			}
#else
			if (ret == ALG_TA_NOT_SUPPORT)
				pe2->state = PE2_TA_NOT_SUPPORT;
			else if (ret == ALG_TA_CHECKING) {
				pe2->state = PE2_RUN;
				again = true;
			} else if (ret == ALG_DONE)
				pe2->state = PE2_HW_READY;
			ret_value = ret;
#endif
/* End mod by jin.wang */
			break;
		default:
			pe2_err("PE2 unknown state:%d\n", pe2->state);
			ret_value = ALG_INIT_FAIL;
			break;
		}
	} while (again == true);
	__pm_relax(pe2->suspend_lock);
	mutex_unlock(&pe2->access_lock);

	return ret_value;
}


static bool _pe2_is_algo_running(struct chg_alg_device *alg)
{
	struct mtk_pe20 *pe2;

	pe2_dbg("%s\n", __func__);
	pe2 = dev_get_drvdata(&alg->dev);

	if (pe2->state == PE2_RUN ||
		pe2->state == PE2_TUNING ||
		pe2->state == PE2_POSTCC)
		return true;

	return false;
}

static int _pe2_stop_algo(struct chg_alg_device *alg)
{
	struct mtk_pe20 *pe2;

	pe2 = dev_get_drvdata(&alg->dev);

	pe2_dbg("%s %d\n", __func__, pe2->state);
	if (pe2->state == PE2_RUN ||
		pe2->state == PE2_TUNING ||
		pe2->state == PE2_POSTCC) {
		pe2_reset_ta_vchr(alg);
		pe2->state = PE2_HW_READY;

/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		if ((alg->config == DUAL_CHARGERS_IN_PARALLEL)
			|| (alg->config == DUAL_CHARGERS_IN_SERIES)) {
#else
		if (alg->config == DUAL_CHARGERS_IN_SERIES) {
#endif
/* End mod by jin.wang */
			pe2_hal_enable_charger(alg, CHG2, false);
			pe2_hal_charger_enable_chip(alg, CHG2, false);
		}
	}

	return 0;
}

static int pe2_full_event(struct chg_alg_device *alg)
{
	struct mtk_pe20 *pe2;
/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	int ret;
	bool chg_en, chg2_enabled = false;
	int ichg2, ichg2_min;
#endif
/* End del by jin.wang */

	int ret_value = 0;

	pe2 = dev_get_drvdata(&alg->dev);
	switch (pe2->state) {
	case PE2_HW_UNINIT:
	case PE2_HW_FAIL:
	case PE2_HW_READY:
	case PE2_TA_NOT_SUPPORT:
		break;
	case PE2_RUN:
	case PE2_TUNING:
	case PE2_POSTCC:
/* Begin mod by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		pe2_err("%s evt full, force\n",  __func__);
		if ((alg->config == DUAL_CHARGERS_IN_PARALLEL)
			|| (alg->config == DUAL_CHARGERS_IN_SERIES)) {
			pe2_hal_enable_charger(alg, CHG2, false);
			pe2_hal_charger_enable_chip(alg, CHG2, false);
		}
		pe2_leave(alg);
		pe2->state = PE2_TA_NOT_SUPPORT;
#else
		if (alg->config == DUAL_CHARGERS_IN_SERIES) {
			pe2_hal_is_charger_enable(alg, CHG2, &chg_en);
			chg2_enabled = pe2_hal_is_chip_enable(alg, CHG2);

			if (!chg_en || !chg2_enabled) {
				/* notify eoc , fix me */
				pe2->state = PE2_HW_READY;
				pe2_err("charging done:%d %d\n",
					__func__, chg_en, chg2_enabled);
				if (alg->is_polling_mode == false)
					ret_value = 1;
			} else {
				pe2_hal_get_charging_current(
					alg, CHG2, &ichg2);
				ret = pe2_hal_get_min_charging_current(
					alg, CHG2, &ichg2_min);
				if (ret == -ENOTSUPP)
					ichg2_min = 100000;

				pe2_err("ichg2:%d, ichg2_min:%d state:%d\n",
					ichg2, ichg2_min, pe2->state);
				if (ichg2 - 500000 <= ichg2_min) {
					pe2->state = PE2_POSTCC;
					pe2_hal_enable_charger(alg,
						CHG2, false);
					pe2_hal_set_eoc_current(alg,
						CHG1, 150000);
					pe2_hal_enable_termination(alg,
						CHG1, true);
				} else {
					pe2->state = PE2_TUNING;
					mutex_lock(&pe2->data_lock);
					if (pe2->charging_current2 >= 500000)
						pe2->charging_current2 = ichg2
							- 500000;
					pe2_hal_set_charging_current(alg,
						CHG2, pe2->charging_current2);
					mutex_unlock(&pe2->data_lock);
				}
				ret_value = 1;
			}

		} else {
			if (pe2->state == PE2_RUN) {
				pe2_err("%s evt full\n",  __func__);
				pe2_leave(alg);
				pe2->state = PE2_HW_READY;
			}
		}
#endif
/* End add by jin.wang */
		break;
	default:
		pe2_dbg("%s: error state:%d\n", __func__,
			pe2->state);
		break;
	}

	return ret_value;
}

static int _pe2_notifier_call(struct chg_alg_device *alg,
			 struct chg_alg_notify *notify)
{
	struct mtk_pe20 *pe2;
	int ret_value = 0;

	pe2 = dev_get_drvdata(&alg->dev);
	pe2_err("%s evt:%d state:%s\n", __func__, notify->evt,
		pe2_state_to_str(pe2->state));

	switch (notify->evt) {
	case EVT_PLUG_OUT:
		ret_value = pe2_plugout_reset(alg);
		break;
	case EVT_FULL:
		ret_value = pe2_full_event(alg);
		break;
/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	case EVT_RECHARGE:
		pe2_err("%s evt recharge, retry\n", __func__);
		pe2->state = PE2_HW_READY;
		break;
#endif
/* End add by jin.wang */
	default:
		ret_value = -EINVAL;
	}
	return ret_value;
}

static void mtk_pe2_parse_dt(struct mtk_pe20 *pe2,
				struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 val;

	/* PE 2.0 */
	if (of_property_read_u32(np, "pe20_ichg_level_threshold", &val) >= 0)
		pe2->pe20_ichg_level_threshold = val;
	else {
		pr_notice("use default PE20_ICHG_LEAVE_THRESHOLD:%d\n",
			PE20_ICHG_LEAVE_THRESHOLD);
		pe2->pe20_ichg_level_threshold =
						PE20_ICHG_LEAVE_THRESHOLD;
	}

	if (of_property_read_u32(np, "ta_start_battery_soc", &val) >= 0)
		pe2->ta_start_battery_soc = val;
	else {
		pr_notice("use default TA_START_BATTERY_SOC:%d\n",
			TA_START_BATTERY_SOC);
		pe2->ta_start_battery_soc = TA_START_BATTERY_SOC;
	}

	if (of_property_read_u32(np, "ta_stop_battery_soc", &val) >= 0)
		pe2->ta_stop_battery_soc = val;
	else {
		pr_notice("use default TA_STOP_BATTERY_SOC:%d\n",
			TA_STOP_BATTERY_SOC);
		pe2->ta_stop_battery_soc = TA_STOP_BATTERY_SOC;
	}

	if (of_property_read_u32(np, "min_charger_voltage", &val) >= 0)
		pe2->min_charger_voltage = val;
	else {
		pr_notice("use default V_CHARGER_MIN:%d\n", PE20_V_CHARGER_MIN);
		pe2->min_charger_voltage = PE20_V_CHARGER_MIN;
	}

	/* cable measurement impedance */
	if (of_property_read_u32(np, "cable_imp_threshold", &val) >= 0)
		pe2->cable_imp_threshold = val;
	else {
		pr_notice("use default CABLE_IMP_THRESHOLD:%d\n",
			PE2_CABLE_IMP_THRESHOLD);
		pe2->cable_imp_threshold = PE2_CABLE_IMP_THRESHOLD;
	}

	if (of_property_read_u32(np, "vbat_cable_imp_threshold", &val) >= 0)
		pe2->vbat_cable_imp_threshold = val;
	else {
		pr_notice("use default VBAT_CABLE_IMP_THRESHOLD:%d\n",
			PE2_VBAT_CABLE_IMP_THRESHOLD);
		pe2->vbat_cable_imp_threshold = PE2_VBAT_CABLE_IMP_THRESHOLD;
	}

	/* single charger */
	if (of_property_read_u32(np, "sc_input_current", &val) >= 0)
		pe2->sc_input_current = val;
	else {
		pr_notice("use default SC_INPUT_CURRENT:%d\n",
			SC_INPUT_CURRENT);
		pe2->sc_input_current = SC_INPUT_CURRENT;
	}

	if (of_property_read_u32(np, "sc_charger_current", &val) >= 0)
		pe2->sc_charger_current = val;
	else {
		pr_notice("use default SC_CHARGING_CURRENT:%d\n",
			SC_CHARGING_CURRENT);
		pe2->sc_charger_current = SC_CHARGING_CURRENT;
	}

	/* dual charger in series */
	if (of_property_read_u32(np, "dcs_input_current", &val) >= 0)
		pe2->dcs_input_current = val;
	else {
		pr_notice("use default DCS_INPUT_CURRENT:%d\n",
			DCS_INPUT_CURRENT);
		pe2->dcs_input_current = DCS_INPUT_CURRENT;
	}

	if (of_property_read_u32(np, "dcs_chg1_charger_current", &val) >= 0)
		pe2->dcs_chg1_charger_current = val;
	else {
		pr_notice("use default DCS_CHG1_CHARGER_CURRENT:%d\n",
			DCS_CHG1_CHARGER_CURRENT);
		pe2->dcs_chg1_charger_current = DCS_CHG1_CHARGER_CURRENT;
	}

	if (of_property_read_u32(np, "dcs_chg2_charger_current", &val) >= 0)
		pe2->dcs_chg2_charger_current = val;
	else {
		pr_notice("use default DCS_CHG2_CHARGER_CURRENT:%d\n",
			SC_CHARGING_CURRENT);
		pe2->dcs_chg2_charger_current = DCS_CHG2_CHARGER_CURRENT;
	}

	if (of_property_read_u32(np, "slave_mivr_diff", &val) >= 0)
		pe2->pe2_slave_mivr_diff = val;
	else {
		pr_notice("use default slave_mivr_diff:%d\n",
			PE2_SLAVE_MIVR_DIFF);
		pe2->pe2_slave_mivr_diff = PE2_SLAVE_MIVR_DIFF;
	}

	if (of_property_read_u32(np, "dual_polling_ieoc", &val) >= 0)
		pe2->dual_polling_ieoc = val;
	else {
		pr_notice("use default dual_polling_ieoc :%d\n", 750000);
		pe2->dual_polling_ieoc = 750000;
	}

/* Begin add by jin.wang task 2064 on 2021.10.26 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if (of_property_read_u32(np, "main_pct", &val) >= 0)
		pe2->main_pct = val;
	if (pe2->main_pct < 30 || pe2->main_pct > 100) {
		pr_notice("PE2 use default main_pct:50\n");
		pe2->main_pct = 50;
	}
#endif
/* End add by jin.wang */
}

int _pe2_get_prop(struct chg_alg_device *alg,
		enum chg_alg_props s, int *value)
{

	pr_notice("%s\n", __func__);
	if (s == ALG_MAX_VBUS)
		*value = 10000;
	else
		pr_notice("%s does not support prop:%d\n", __func__, s);
	return 0;
}

int _pe2_set_prop(struct chg_alg_device *alg,
		enum chg_alg_props s, int value)
{
	pr_notice("%s %d %d\n", __func__, s, value);
	return 0;
}

int _pe2_set_setting(struct chg_alg_device *alg_dev,
	struct chg_limit_setting *setting)
{
	struct mtk_pe20 *pe2;

/* Begin del by jin.wang task 2064 on 2021.11.2 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pe2_dbg("%s cv:%d icl:%d,%d cc:%d,%d\n",
		__func__,
		setting->cv,
		setting->input_current_limit1,
		setting->input_current_limit2,
		setting->charging_current_limit1,
		setting->charging_current_limit2);
#endif
/* End del by jin.wang */

	pe2 = dev_get_drvdata(&alg_dev->dev);

	mutex_lock(&pe2->access_lock);
	__pm_stay_awake(pe2->suspend_lock);
	pe2->cv = setting->cv;

/* Begin mod by jin.wang task 2064 on 2021.11.18 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	if (main_pct >= 20 && main_pct <= 100) {
		pe2->main_pct = main_pct;
	}

	if (alg_dev->config == DUAL_CHARGERS_IN_PARALLEL) {
		if (setting->total_ibus_limit != -1) {
			pe2->input_current_limit1 =
				setting->total_ibus_limit * pe2->main_pct / 100;
			pe2->input_current_limit2 =
				setting->total_ibus_limit - pe2->input_current_limit1;
		} else {
			pe2->input_current_limit1 = -1;
			pe2->input_current_limit2 = -1;
		}

		if (setting->total_ibatt_limit != -1) {
			pe2->charging_current_limit1 =
				setting->total_ibatt_limit * pe2->main_pct / 100;
			pe2->charging_current_limit2 =
				setting->total_ibatt_limit - pe2->charging_current_limit1;
		} else {
			pe2->charging_current_limit1 = -1;
			pe2->charging_current_limit2 = -1;
		}
	} else {
		pe2->input_current_limit1 = setting->total_ibus_limit;
		pe2->charging_current_limit1 = setting->total_ibatt_limit;
		pe2->input_current_limit2 = -1;
		pe2->charging_current_limit2 = -1;
	}
#else
	pe2->input_current_limit1 = setting->input_current_limit1;
	pe2->charging_current_limit1 = setting->charging_current_limit1;
	pe2->input_current_limit2 = setting->input_current_limit2;
	pe2->charging_current_limit2 = setting->charging_current_limit2;
#endif
/* End mod by jin.wang */

/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pe2_dbg("%s(): cv:%d,ibus:%d(%d,%d),ibatt:%d(%d,%d)\n",
		__func__, setting->cv,
		setting->total_ibus_limit,
		pe2->input_current_limit1,
		pe2->input_current_limit2,
		setting->total_ibatt_limit,
		pe2->charging_current_limit1,
		pe2->charging_current_limit2);
#endif
/* End add by jin.wang */

	__pm_relax(pe2->suspend_lock);
	mutex_unlock(&pe2->access_lock);

	return 0;
}

static struct chg_alg_ops pe2_alg_ops = {
	.init_algo = _pe2_init_algo,
	.is_algo_ready = _pe2_is_algo_ready,
	.start_algo = _pe2_start_algo,
	.is_algo_running = _pe2_is_algo_running,
	.stop_algo = _pe2_stop_algo,
	.notifier_call = _pe2_notifier_call,
	.get_prop = _pe2_get_prop,
	.set_prop = _pe2_set_prop,
	.set_current_limit = _pe2_set_setting,
};

static int mtk_pe2_probe(struct platform_device *pdev)
{
	struct mtk_pe20 *pe2 = NULL;

	pr_notice("%s: starts\n", __func__);

	pe2 = devm_kzalloc(&pdev->dev, sizeof(*pe2), GFP_KERNEL);
	if (!pe2)
		return -ENOMEM;
	platform_set_drvdata(pdev, pe2);
	pe2->pdev = pdev;

	pe2->suspend_lock =
		wakeup_source_register(NULL, "PE+20 suspend wakelock");

	mutex_init(&pe2->access_lock);
	mutex_init(&pe2->cable_out_lock);
	mutex_init(&pe2->data_lock);

	pe2->ta_vchr_org = 5000000;
	pe2->idx = -1;
	pe2->vbus = 5000000;
	pe2->state = PE2_HW_UNINIT;

/* Begin add by jin.wang task 2064 on 2021.10.26 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pe2->input_current_limit1 = -1;
	pe2->input_current_limit2 = -1;
	pe2->charging_current_limit1 = -1;
	pe2->charging_current_limit2 = -1;
#endif
/* End add by jin.wang */

	mtk_pe2_parse_dt(pe2, &pdev->dev);
	pe2->bat_psy = devm_power_supply_get_by_phandle(&pdev->dev, "gauge");

	if (IS_ERR_OR_NULL(pe2->bat_psy))
		pe2_err("%s: devm power fail to get bat_psy\n", __func__);

	pe2->profile[0].vbat = 3400000;
	pe2->profile[1].vbat = 3500000;
	pe2->profile[2].vbat = 3600000;
	pe2->profile[3].vbat = 3700000;
	pe2->profile[4].vbat = 3800000;
	pe2->profile[5].vbat = 3900000;
	pe2->profile[6].vbat = 4000000;
	pe2->profile[7].vbat = 4100000;
	pe2->profile[8].vbat = 4200000;
	pe2->profile[9].vbat = 4300000;

/* Begin mod by jin.wang to enlarge ibatt on 2022.4.1 */
/* change begin by bitao.xiong for PARALLEL charge */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	pe2->profile[0].vchr = 9000000;
	pe2->profile[1].vchr = 9000000;
	pe2->profile[2].vchr = 9000000;
	pe2->profile[3].vchr = 9000000;
	pe2->profile[4].vchr = 9000000;
	pe2->profile[5].vchr = 9000000;
	pe2->profile[6].vchr = 9000000;
	pe2->profile[7].vchr = 9000000;
	pe2->profile[8].vchr = 9000000;
	pe2->profile[9].vchr = 9000000;
#else
	pe2->profile[0].vchr = 8000000;
	pe2->profile[1].vchr = 8500000;
	pe2->profile[2].vchr = 8500000;
	pe2->profile[3].vchr = 9000000;
	pe2->profile[4].vchr = 9000000;
	pe2->profile[5].vchr = 9000000;
	pe2->profile[6].vchr = 9500000;
	pe2->profile[7].vchr = 9500000;
	pe2->profile[8].vchr = 10000000;
	pe2->profile[9].vchr = 10000000;
#endif
/* change end by bitao.xiong for PARALLEL charge */
/* End mod by jin.wang */

	pe2->alg = chg_alg_device_register("pe2", &pdev->dev,
					pe2, &pe2_alg_ops, NULL);

	return 0;
}

static int mtk_pe2_remove(struct platform_device *dev)
{
	return 0;
}

static void mtk_pe2_shutdown(struct platform_device *dev)
{

}

static const struct of_device_id mtk_pe2_of_match[] = {
	{.compatible = "mediatek,charger,pe2",},
	{},
};

MODULE_DEVICE_TABLE(of, mtk_pe2_of_match);

struct platform_device pe2_device = {
	.name = "pe2",
	.id = -1,
};

static struct platform_driver pe2_driver = {
	.probe = mtk_pe2_probe,
	.remove = mtk_pe2_remove,
	.shutdown = mtk_pe2_shutdown,
	.driver = {
		   .name = "pe2",
		   .of_match_table = mtk_pe2_of_match,
	},
};

static int __init mtk_pe2_init(void)
{
	return platform_driver_register(&pe2_driver);
}
late_initcall(mtk_pe2_init);

static void __exit mtk_pe2_exit(void)
{
	platform_driver_unregister(&pe2_driver);
}
module_exit(mtk_pe2_exit);


MODULE_AUTHOR("wy.chuang <wy.chuang@mediatek.com>");
MODULE_DESCRIPTION("MTK Pump Express 2 algorithm Driver");
MODULE_LICENSE("GPL");

