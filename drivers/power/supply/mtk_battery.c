// SPDX-License-Identifier: GPL-2.0

#include <linux/cdev.h>		/* cdev */
#include <linux/err.h>	/* IS_ERR, PTR_ERR */
#include <linux/init.h>		/* For init/exit macros */
#include <linux/irq.h>
#include <linux/irqdesc.h>	/*irq_to_desc*/
#include <linux/kernel.h>
#include <linux/kthread.h>	/* For Kthread_run */
#include <linux/math64.h>
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/netlink.h>	/* netlink */
#include <linux/of_fdt.h>	/*of_dt API*/
#include <linux/of.h>
#include <linux/of_address.h>	/* Add by bing-zhang for getting ocv from preloader on 20210827 */
#include <linux/platform_device.h>	/* platform device */
#include <linux/proc_fs.h>
#include <linux/reboot.h>	/*kernel_power_off*/
#include <linux/sched.h>	/* For wait queue*/
#include <linux/skbuff.h>	/* netlink */
#include <linux/socket.h>	/* netlink */
#include <linux/time.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>		/* For wait queue*/
#include <net/sock.h>		/* netlink */
#include "mtk_battery.h"
#include "mtk_battery_table.h"
#if IS_ENABLED(CONFIG_TCT_CHARGER)
/* Begin added by bitao.xiong for task-9796564 on 2020-08-20 */
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/of_device.h>
#include "mtk_charger.h"
/* End added by bitao.xiong for task-9796564 on 2020-08-20 */
#endif

/* Begin added by hailong.chen for task 9777034 on 2020-08-20 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
#include <linux/mfd/mt6357/registers.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>
static int battery_resistance_id = 0;
static int fixtemp = 0;
static int fixtemp_val = 250;
#endif
/* End added by hailong.chen for task 9777034 on 2020-08-20 */

//Begin Modified by qiuguangliang for 11043275 on 2021-04-23
#include <mtk_charger.h>
//End Modified by qiuguangliang for 11043275 on 2021-04-23

/* Begin added by dapeng.qiao for task 11038299 on 2021-05-1 */
#ifdef TCT_BMS_SW_SUPPORT
signed int g_chr_vol;
int g_fg_current, g_bat_vol;
int g_bat_status, g_bat_capacity, g_tbat_precise;
int g_chr_type = POWER_SUPPLY_TYPE_UNKNOWN;
extern bool bms_sw_support;
#endif
/* End added by dapeng.qiao for task 11038299 on 2021-05-1 */

/* Begin Added by tangshan.bai for LEVIN-6148 */
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
extern void uisoc_tracking_to_zero_work(struct work_struct *work);
int fg_max_monotone;
#endif
/* End Added by tangshan.bai for LEVIN-6148 */

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

int __attribute__ ((weak))
	mtk_battery_daemon_init(struct platform_device *pdev)
{
	struct mtk_battery *gm;
	struct mtk_gauge *gauge;

	gauge = dev_get_drvdata(&pdev->dev);
	gm = gauge->gm;

	gm->algo.active = true;
	bm_err("[%s]: weak function,kernel algo=%d\n", __func__,
		gm->algo.active);
	return -EIO;
}

int __attribute__ ((weak))
	wakeup_fg_daemon(unsigned int flow_state, int cmd, int para1)
{
	return 0;
}

void __attribute__ ((weak))
	fg_sw_bat_cycle_accu(struct mtk_battery *gm)
{
}

void __attribute__ ((weak))
	notify_fg_chr_full(struct mtk_battery *gm)
{
}

void __attribute__ ((weak))
	fg_drv_update_daemon(struct mtk_battery *gm)
{
}

/* Begin add by jin.wang for jira 2064 on 2021-11-30 */
//#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
#if 0
static void battery_update_chg_status(struct mtk_battery *gm);
#endif
/* End add by jin.wang */

void enable_gauge_irq(struct mtk_gauge *gauge,
	enum gauge_irq irq)
{
	struct irq_desc *desc;

	if (irq >= GAUGE_IRQ_MAX)
		return;

	desc = irq_to_desc(gauge->irq_no[irq]);
	bm_err("%s irq_no:%d:%d depth:%d\n",
		__func__, irq, gauge->irq_no[irq],
		desc->depth);
	if (desc->depth == 1)
		enable_irq(gauge->irq_no[irq]);
}

void disable_gauge_irq(struct mtk_gauge *gauge,
	enum gauge_irq irq)
{
	struct irq_desc *desc;

	if (irq >= GAUGE_IRQ_MAX)
		return;

	if (gauge->irq_no[irq] == 0)
		return;

	desc = irq_to_desc(gauge->irq_no[irq]);
	bm_err("%s irq_no:%d:%d depth:%d\n",
		__func__, irq, gauge->irq_no[irq],
		desc->depth);
	if (desc->depth == 0)
		disable_irq_nosync(gauge->irq_no[irq]);
}

struct mtk_battery *get_mtk_battery(void)
{
	struct mtk_gauge *gauge;
	struct power_supply *psy;

	psy = power_supply_get_by_name("mtk-gauge");
	if (psy == NULL) {
		bm_err("[%s]psy is not rdy\n", __func__);
		return NULL;
	}

	gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);
	if (gauge == NULL) {
		bm_err("[%s]mtk_gauge is not rdy\n", __func__);
		return NULL;
	}
	return gauge->gm;
}

int bat_get_debug_level(void)
{
	struct mtk_gauge *gauge;
	struct power_supply *psy;
	static struct mtk_battery *gm;

	if (gm == NULL) {
		psy = power_supply_get_by_name("mtk-gauge");
		if (psy == NULL)
			return BMLOG_DEBUG_LEVEL;
		gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);
		if (gauge == NULL || gauge->gm == NULL)
			return BMLOG_DEBUG_LEVEL;
		gm = gauge->gm;
	}
	return gm->log_level;
}

bool is_algo_active(struct mtk_battery *gm)
{
	return gm->algo.active;
}

/* Begin modified by hailong.chen for task 9785237 on 2020-10-31 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
/* Begin mod by jin.wang for androidT on 2022-4-18 */
static const char *default_batt_type = "NA:NA:NA:NA";  // "Unknown Battery";
/* End mod by jin.wang */

static int64_t convert_battery_id_ohm(int batt_id_mv, int rpull_up_ohm, int vpull_up_mv)
{
	int64_t resistor_value_ohm, denom;

	if (batt_id_mv == 0) {
		/* vadc not correct or batt id line grounded, report 0 kohms */
		return 0;
	}
	/* calculate the battery id resistance reported via ADC */
	denom = div64_s64(vpull_up_mv * 1000LL, batt_id_mv) - 1000LL;

	if (denom == 0) {
		/* batt id connector might be open, return 0 kohms */
		return 0;
	}
	resistor_value_ohm = div64_s64(rpull_up_ohm * 1000LL + denom/2, denom);

	pr_debug("batt id voltage = %d, resistor value = %lld\n", batt_id_mv, resistor_value_ohm);

	return resistor_value_ohm;
}

static unsigned int pmic_get_register_value(struct regmap *map,
	unsigned int addr,
	unsigned int mask,
	unsigned int shift)
{
	unsigned int value = 0;

	regmap_read(map, addr, &value);
	value =
		(value &
		(mask << shift))
		>> shift;
	return value;
}

static int get_vpull_up(struct device_node *np)
{
	struct device_node *pmic_node;
	struct platform_device *pmic_pdev;
	struct mt6397_chip *chip;
	struct regmap *regmap;
	unsigned int vio18_vocal = 0;
	unsigned int vio18_votrim = 0;

	pmic_node = of_parse_phandle(np, "pmic", 0);
	if (!pmic_node) {
		bm_err("get pmic_node fail\n");
		goto err;
	}

	pmic_pdev = of_find_device_by_node(pmic_node);
	if (!pmic_pdev) {
		bm_err("get pmic_pdev fail\n");
		goto err;
	}
	chip = dev_get_drvdata(&(pmic_pdev->dev));

	if (!chip) {
		bm_err("get chip fail\n");
		goto err;
	}

	regmap = chip->regmap;

	switch (chip->chip_id) {
	case MT6357_CHIP_ID:
		vio18_vocal = pmic_get_register_value(regmap, MT6357_VIO18_ANA_CON0, 0xF, 0x0);
		vio18_votrim = pmic_get_register_value(regmap, MT6357_VIO18_ELR_0, 0xF, 0x0);
		break;
	default:
		bm_err("unsupported chip: 0x%x,if need, add code\n", chip->chip_id);
		goto err;
	}
err:
	return ((180 + vio18_vocal + (vio18_votrim & 0x8) - (vio18_votrim & 0x7)) * 10);
}

#if IS_ENABLED(CONFIG_OF)
static int fg_read_dts_val(const struct device_node *np,
		const char *node_srting,
		int *param, int unit);
#endif

#if IS_ENABLED(CONFIG_TCT_DEVICEINFO)
extern char battery_info_module_name[256];
#endif

int fgauge_get_profile_id(void)
{
	struct mtk_battery *gm = NULL;
	struct mtk_gauge *gauge = NULL;
	struct device_node *np = NULL;
	char node_name[128];
	//static bool first_run = true; /* Del by bitao.xiong for RACKT-1448(IEEE1725) on 2022-05-17 */
	bool found_battery = false;
	int batt_id_mv = 0, rpull_up_ohm = 0, vpull_up_mv = 0, dts_battery_id_ohm = 0;
	int rc = 0, id = 0, batt_id_pct = 15;
/* Begin added by bitao.xiong for battery-id pull-up resistor task-11599163 on 2021-12-29 */
#if IS_ENABLED(CONFIG_TCT_CHG_AUSTINTF)
	int rpull_up_ohm_v1 = 0, battery_resistance_id_v1 = 0;
#endif
/* End added by bitao.xiong for battery-id pull-up resistor task-11599163 on 2021-12-29 */

	gm = get_mtk_battery();
	gauge = gm->gauge;
	np = gauge->pdev->dev.of_node;

	/* Begin del by bitao.xiong for RACKT-1448(IEEE1725) on 2022-05-17 */
	/*
	if (!first_run)
		return gm->battery_id;

	first_run = false;
	*/
	/* End del by bitao.xiong for RACKT-1448(IEEE1725) on 2022-05-17 */
	batt_id_mv = gauge_get_int_property(GAUGE_PROP_BATTERY_ID);
/* Begin modified by bitao.xiong for battery-id pull-up resistor task-11599163 on 2021-12-29 */
#if IS_ENABLED(CONFIG_TCT_CHG_AUSTINTF)
	rc = fg_read_dts_val(np, "battery-id-pullup-ohm-v1", &rpull_up_ohm_v1, 1);
	rc = fg_read_dts_val(np, "battery-id-pullup-ohm", &rpull_up_ohm, 1);
#else
	rc = fg_read_dts_val(np, "battery-id-pullup-ohm", &rpull_up_ohm, 1);
#endif
/* End modified by bitao.xiong for battery-id pull-up resistor task-11599163 on 2021-12-29 */
	if (rc)
		rpull_up_ohm = 200000;//default 200K
	vpull_up_mv = get_vpull_up(np);
/* Begin modified by bitao.xiong for battery-id pull-up resistor task-11599163 on 2021-12-29 */
#if IS_ENABLED(CONFIG_TCT_CHG_AUSTINTF)
	battery_resistance_id = convert_battery_id_ohm(batt_id_mv, rpull_up_ohm, vpull_up_mv);
	battery_resistance_id_v1 = convert_battery_id_ohm(batt_id_mv, rpull_up_ohm_v1, vpull_up_mv);
	bm_err("batt id=%dmV %dohm,rpull_up=%dohm %dmV\n", batt_id_mv, battery_resistance_id, rpull_up_ohm, vpull_up_mv);
	bm_err("batt id_v1=%dmV %dohm,rpull_up=%dohm %dmV\n", batt_id_mv, battery_resistance_id_v1, rpull_up_ohm_v1, vpull_up_mv);
#else
	battery_resistance_id = convert_battery_id_ohm(batt_id_mv, rpull_up_ohm, vpull_up_mv);
	bm_err("batt id=%dmV %dohm,rpull_up=%dohm %dmV\n", batt_id_mv, battery_resistance_id, rpull_up_ohm, vpull_up_mv);
#endif
/* End modified by bitao.xiong for battery-id pull-up resistor task-11599163 on 2021-12-29 */

	for (id = 0; id < TOTAL_BATTERY_NUMBER; id++) {
		sprintf(node_name, "battery%d-id-ohm", id);
		rc = fg_read_dts_val(np, node_name, &dts_battery_id_ohm, 1);
		if (rc)
			continue;
		sprintf(node_name, "battery%d-id-range-pct", id);
		fg_read_dts_val(np, node_name, &batt_id_pct, 1);
/* Begin modified by bitao.xiong for battery-id pull-up resistor task-11599163 on 2021-12-29 */
#if IS_ENABLED(CONFIG_TCT_CHG_AUSTINTF)
		if ((abs(dts_battery_id_ohm - battery_resistance_id) * 100) < (batt_id_pct * dts_battery_id_ohm)
			|| (abs(dts_battery_id_ohm - battery_resistance_id_v1) * 100) < (batt_id_pct * dts_battery_id_ohm)) {
#else
		if ((abs(dts_battery_id_ohm - battery_resistance_id) * 100) <= (batt_id_pct * dts_battery_id_ohm)) {/*modify by zhangkun for MODEL3-2115*/
#endif
/* End modified by bitao.xiong for battery-id pull-up resistor task-11599163 on 2021-12-29 */
			found_battery = true;
			gm->is_debug_battery = false; /* Add by bitao.xiong for RACKT-1448(IEEE1725) on 2022-05-17 */
			gm->battery_id = id;
			sprintf(node_name, "battery%d-type", id);
			rc = of_property_read_string(np, node_name, &gm->battery_type);

/* Begin mod by jin.wang for androidT on 2022-4-12 */
#if IS_ENABLED(CONFIG_TCT_DEVICEINFO)
			if (rc < 0) {
				gm->battery_type = default_batt_type;
			} else {
				sprintf(battery_info_module_name, "%s", gm->battery_type);
			}
#else
			if (rc < 0)
				gm->battery_type = default_batt_type;
#endif
/* End mod by jin.wang */

			bm_err("found battery%d %s\n", id, gm->battery_type);
			break;
		}
	}

	if (!found_battery) {
		gm->battery_id = 0;
		gm->is_debug_battery = true;
		gm->battery_type = default_batt_type;
	}
	return gm->battery_id;
}
#else
int fgauge_get_profile_id(void)
{
	return 0;
}
#endif
/* End modified by hailong.chen for task 9785237 on 2020-10-31 */

int wakeup_fg_algo_cmd(
	struct mtk_battery *gm, unsigned int flow_state, int cmd, int para1)
{

	bm_debug("[%s] 0x%x %d %d\n", __func__, flow_state, cmd, para1);
	if (gm->disableGM30) {
		bm_err("FG daemon is disabled\n");
		return -1;
	}
	if (is_algo_active(gm) == true)
		do_fg_algo(gm, flow_state);
	else
		wakeup_fg_daemon(flow_state, cmd, para1);

	return 0;
}

int wakeup_fg_algo(struct mtk_battery *gm, unsigned int flow_state)
{
	return wakeup_fg_algo_cmd(gm, flow_state, 0, 0);
}

bool is_recovery_mode(void)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_debug("%s, bootmdoe = %d\n", gm->bootmode);

	/* RECOVERY_BOOT */
	if (gm->bootmode == 2)
		return true;

	return false;
}

bool is_kernel_power_off_charging(void)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_debug("%s, bootmdoe = %d\n", gm->bootmode);

	/* KERNEL_POWER_OFF_CHARGING_BOOT */
	if (gm->bootmode == 8)
		return true;

	return false;
}

/* ============================================================ */
/* power supply: battery */
/* ============================================================ */
int check_cap_level(int uisoc)
{
	if (uisoc >= 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (uisoc >= 80 && uisoc < 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	else if (uisoc >= 20 && uisoc < 80)
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	else if (uisoc > 0 && uisoc < 20)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (uisoc == 0)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
}

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	/* Begin added by hailong.chen for task 9777034 on 2020-08-20 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_ISENSECURRENT,
	POWER_SUPPLY_PROP_BATT_ID,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	/* begin add by bing-zhang for getting ocv from preloader on 20210827 */
	POWER_SUPPLY_PROP_OCV_PL,
	POWER_SUPPLY_PROP_SOC_PL,
	/* end add by bing-zhang for getting ocv from preloader on 20210827 */
	POWER_SUPPLY_PROP_COULOMB_COUNT,
	POWER_SUPPLY_PROP_TCL_FIXTEMP,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_TYPE,
	POWER_SUPPLY_PROP_DEBUG_BATTERY,
/* Begin Added by tangshan.bai for LEVIN-6148 */
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
    POWER_SUPPLY_PROP_PEAK_LEVEL,
    POWER_SUPPLY_PROP_BATTERY_VERIFY,
    POWER_SUPPLY_PROP_BATTERY_VSOC,
    POWER_SUPPLY_PROP_BATT_RESISTANCE,
    POWER_SUPPLY_PROP_CHARGING_CYCLE_TABLE,
    POWER_SUPPLY_PROP_BATTERYUSOC,
#endif
/* End Added by tangshan.bai for LEVIN-6148*/
#endif
	/* End added by hailong.chen for task 9777034 on 2020-08-20 */

};

/* Begin added by jin.wang for jira 2064 at 2021-10-25 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
struct charger_device *primary_chg = NULL;
struct charger_device *slave_chg = NULL;
static int battery_get_icl_settled(void)
{
	unsigned int main_icl = 0, sub_icl = 0;
	int ret = 0;

	if(!primary_chg) {
		primary_chg = get_charger_by_name("primary_chg");
		if (!primary_chg) {
			pr_err("Error: can't find primary chg\n");
			return -ENODEV;
		}
	}

	ret = charger_dev_get_input_current(primary_chg, &main_icl);
	if (ret < 0) {
		pr_err("Error: can't read primary icl\n");
		return -EIO;
	}

	if(!slave_chg)
		slave_chg = get_charger_by_name("secondary_chg");

	if (slave_chg) {
		ret = charger_dev_get_input_current(slave_chg, &sub_icl);
		if (ret < 0) {
			pr_err("Error: can't read slave icl\n");
			return -EIO;
		}
	}

	return (int)(main_icl + sub_icl);
}
#endif
/* End added by jin.wang */


#if defined(CONFIG_TCT_FEATURE_SLEEP_CHARGE) || defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
/* Begin added by dapeng.qiao for task 11603976 on 2021-10-12 */
//#if IS_ENABLED(CONFIG_TCT_CHARGER)
//#if defined(CONFIG_TCT_FEATURE_SLEEP_CHARGE)
static int bserachFirstOverVlaue_sleepcharge_current(struct remain_guage *nums, unsigned int length, unsigned int value) {
    int low = 0;
    int high = length - 1;

    if(high <= 0) return 0;
    if(value >= nums[high].current_now) return length-1;
    while (low <= high) {
        int mid = low + ((high - low) >> 1);
        if(low == high) return mid;
        if (nums[mid].current_now <= value) {
            if ((mid == 0) || nums[mid-1].current_now > value){
                if(mid){
                    return mid-1;
                }
            }
            else high = mid - 1;
        } else {
            low = mid + 1;
        }
    }
    return length-1;
}

static int bserachFirstOverVlaue_sleepcharge_guage(struct remain_guage *nums, unsigned int length, unsigned int value) {
    int low = 0;
    int high = length - 1;

    if(high <= 0) return 0;
    if(value >= nums[high].charged_guage) return length-1;
    while (low <= high) {
        int mid = low + ((high - low) >> 1);
        if(low == high) return mid;
        if (nums[mid].charged_guage > value) {
            if ((mid == 0) || nums[mid-1].charged_guage <= value){
                if(mid){
                    return mid-1;
                }
            }
            else high = mid - 1;
        } else {
            low = mid + 1;
        }
    }
    return length-1;
}
//#endif
//#endif
#ifdef CONFIG_TCT_FEATURE_PEAK_MANAGMENT
#define ENABLE_CALCULATE_PEAK_TAIL_TIME
#endif

//#define MAX_CHARGE_CURRENT remain_zcv[0].current_now
//#define MAX_CHARGE_CURRENT_CHANGE 1987
//#define MAX_CHARGE_CURRENT_CHANGE_POINT 120
//#define MAX_CHARGE_REMAIN_GAUGE remain_zcv[120].charged_guage
//if peak hide 8,now sleep charge need stop at 80,so we need calculate time of 12% instead of 20%
struct smart_sleep_chrging_data remain_data;
struct smart_peak_management_data peak_data;
static int remain_guage_charge_time(int current_now, int remain_mAh){
    unsigned int remain_mAh_CurvedSlope;
    unsigned int remain_mAh_horizontal;
    unsigned int remain_second;
    unsigned int remain_second_horizontal;
    unsigned int remain_second_CurvedSlope;
    int Cnow;
    unsigned int current_lookup;
    unsigned int qmax;
    int l_remain_mAh;
#ifdef ENABLE_CALCULATE_PEAK_TAIL_TIME
    unsigned int q_max_now;
    unsigned int qmax_hidden;
    unsigned int BAT_peak_level;
//    struct mtk_gauge *gauge;
//    struct power_supply *psy;
    static struct mtk_battery *gm;
#endif
    unsigned int count_1;
    unsigned int count_2;
    unsigned int count_3;
   struct remain_guage *remain_zcv;
    int array_count ;
    int max_charge_current;
    int current_change_value;
    int current_change_point;
    int MaxCurvedSlopeSecond;

    remain_zcv = &remain_data.remain_zcv[0];
    array_count = remain_data.remain_array_count;
    max_charge_current = remain_data.max_charge_current;
    current_change_value = remain_data.max_charge_current_change;
    current_change_point = remain_data.max_charge_current_change_point;
    MaxCurvedSlopeSecond = remain_data.max_charge_current_change_point_RemainSecond;
    count_1 = 0;
    count_2 = 0;
    count_3 = 0;
    remain_mAh_CurvedSlope = 0;
    remain_mAh_horizontal = 0;
    remain_second = 0;
    remain_second_horizontal = 0;
    remain_second_CurvedSlope = 0;
    qmax = remain_zcv[array_count-1].charged_guage;
    l_remain_mAh = remain_mAh;
    if(l_remain_mAh < 0) {l_remain_mAh = 0; }
    if(l_remain_mAh > qmax){ l_remain_mAh = qmax;}
    bm_err("[%s] charge_current: %d, change_value: %d, change_point:%d, RemainSecond:%d\n", \
        __func__, max_charge_current, current_change_value, current_change_point, MaxCurvedSlopeSecond);
#ifdef ENABLE_CALCULATE_PEAK_TAIL_TIME
    gm = get_mtk_battery();
    q_max_now = gm->fg_table_cust_data.fg_profile[gm->battery_id].q_max;
    bm_err("[%s] q_max_now:%d, qmax:%d, line:%d\n", __func__, q_max_now, qmax, __LINE__);
    if(q_max_now > qmax){
        qmax = q_max_now;
    }
    BAT_peak_level = gm->BAT_peak_level;
    if(gm->BatteryVerify & BIT(3)){
        qmax_hidden = qmax*(100 - BAT_peak_level)/100;
        if(qmax >= l_remain_mAh){
            if(qmax_hidden >= l_remain_mAh){
                remain_second = (100 - BAT_peak_level)*10;
                bm_err("[%s] qmax_hidden:%d, remain_mAh:%d, remain_second:%d\n", __func__, qmax_hidden, l_remain_mAh, remain_second);
                return remain_second;
            }else{
                bm_err("[%s] need deal qmax_hidden:%d, remain_mAh:%d, remain_second:%d\n", __func__, qmax_hidden, l_remain_mAh, remain_second);
            }
        }else{
            l_remain_mAh = qmax;
            bm_err("[%s] qmax:%d is too small, remain_mAh:%d, qmax_hidden:%d\n", __func__,qmax, l_remain_mAh, qmax_hidden);
        }
    }
#endif
    Cnow = abs(current_now);
    if(Cnow < 1) Cnow = 1;
    if(current_now <= 0)
    {//discharging
        if(l_remain_mAh >= remain_zcv[current_change_point].charged_guage){
            remain_mAh_CurvedSlope = remain_zcv[current_change_point].charged_guage;
            remain_second_CurvedSlope = remain_zcv[current_change_point].remain_second;
            current_lookup = current_now;
            if(l_remain_mAh > remain_mAh_CurvedSlope){
                remain_mAh_horizontal = l_remain_mAh - remain_mAh_CurvedSlope;
            }else{
                remain_mAh_horizontal = 0;
            }
            remain_second_horizontal = (remain_mAh_horizontal * 3600)/Cnow;
            remain_second = remain_second_CurvedSlope + remain_second_horizontal + Cnow;
        }else{
            count_1 = bserachFirstOverVlaue_sleepcharge_guage(remain_zcv, array_count, qmax - l_remain_mAh);
            current_lookup = current_now;
            remain_second = remain_zcv[count_1].remain_second + Cnow;
        }
        bm_err("[%s] current_now:%d, line:%d\n", __func__,current_now,__LINE__);
    }else{      //charging
        if(Cnow < max_charge_current-5){//not max current charging
            count_2 = bserachFirstOverVlaue_sleepcharge_current(remain_zcv, array_count, Cnow);
            remain_mAh_CurvedSlope = qmax - remain_zcv[count_2].charged_guage;
            remain_second_CurvedSlope = remain_zcv[count_2].remain_second;
            current_lookup = remain_zcv[count_2].current_now;        //verify, need equal to current_now
            bm_err("[%s] current_now:%d,max_charge_current:%d line:%d\n", __func__,current_now,max_charge_current,__LINE__);
            if(l_remain_mAh > remain_mAh_CurvedSlope){
                remain_mAh_horizontal = l_remain_mAh - remain_mAh_CurvedSlope;
            }else{
                remain_mAh_horizontal = 0;
            }
            remain_second_horizontal = (remain_mAh_horizontal * 3600)/Cnow;
            remain_second = remain_second_CurvedSlope + remain_second_horizontal;
        }else{  //max current charging
            remain_mAh_CurvedSlope = remain_zcv[current_change_point].charged_guage;//
            count_1 = bserachFirstOverVlaue_sleepcharge_guage(remain_zcv, array_count, qmax - l_remain_mAh);//already charged mAh
            remain_second = remain_zcv[count_1].remain_second;
            current_lookup = remain_zcv[count_1].current_now;        //verify, need equal to current_now
            if(l_remain_mAh > remain_mAh_CurvedSlope){
                remain_mAh_horizontal = l_remain_mAh - remain_mAh_CurvedSlope;
            }else{
                remain_mAh_horizontal = 0;
            }
            remain_second_CurvedSlope = remain_zcv[current_change_point].remain_second;
            remain_second_horizontal = (remain_mAh_horizontal * 3600)/Cnow;
            bm_err("[%s] current_now:%d,max_charge_current:%d line:%d\n", __func__,current_now,max_charge_current,__LINE__);
        }
#ifdef ENABLE_CALCULATE_PEAK_TAIL_TIME
        if(l_remain_mAh >= qmax_hidden){
            if(qmax_hidden >= remain_mAh_CurvedSlope){
                remain_mAh_horizontal = l_remain_mAh - qmax_hidden;
                remain_second_horizontal = (remain_mAh_horizontal * 3600)/Cnow;
                remain_second = remain_second_horizontal;
                    bm_err("[%s] qmax_hidden:%d, line:%d,Rsec:%d\n", __func__,qmax_hidden,__LINE__,remain_second);
            }else{    //first calculate normal remain time, then sub hidden time
                remain_mAh_CurvedSlope -= qmax_hidden;
                if(qmax_hidden){    //hide gauge < remain tail gauge
                    count_3 = bserachFirstOverVlaue_sleepcharge_guage(remain_zcv, array_count, qmax - qmax_hidden);
                    bm_err("[%s] qmax_hidden:%d, line:%d,Rsec:%d\n", __func__,qmax_hidden,__LINE__,remain_second);
                    remain_second_CurvedSlope -= remain_zcv[count_3].remain_second;
                    remain_second = remain_second_horizontal + remain_second_horizontal;
                }
            }
        }else{
            remain_second = 0;
            bm_err("[%s] remain_mAh is too small, line:%d\n", __func__, __LINE__);
        }
#endif
    }

    bm_err("[%s] cnt1:%d,cnt2:%d,cnt3:%d,Cpoint:%d,Cnow:%d,Ccal:%d,RmAh:%d,RmAhH:%d,RmAhCS:%d,Rsec:%d,RsecH:%d,RsecCS:%d\n",
        __func__,count_1,count_2,count_3,current_change_point,current_now,current_lookup,l_remain_mAh,remain_mAh_horizontal,
        remain_mAh_CurvedSlope,remain_second,remain_second_horizontal,remain_second_CurvedSlope);

    return remain_second;
}
/* End added by dapeng.qiao for task 11603976 on 2021-10-12 */

#else
/* Begin added by dapeng.qiao for task 11603976 on 2021-10-12 */
struct remain_guage {
	unsigned int current_now;
	unsigned int charge_soc;
	unsigned int vbat;
	unsigned int remain_guage;
	unsigned int remain_second;
	unsigned int charged_second;
};

static struct remain_guage remain_zcv[] = {
    {2001,     0,     3750,     0,     11599,     0 },
    {2001,     71,     3839,     33,     11539,     60 },
    {2001,     142,     3882,     67,     11479,     120 },
    {2001,     213,     3913,     100,     11419,     180 },
    {2001,     284,     3930,     133,     11359,     240 },
    {2001,     355,     3937,     167,     11299,     300 },
    {2001,     425,     3941,     200,     11239,     360 },
    {2001,     496,     3944,     234,     11179,     420 },
    {2001,     567,     3948,     267,     11119,     480 },
    {2001,     638,     3952,     300,     11059,     540 },
    {2001,     709,     3956,     334,     10999,     600 },
    {2001,     780,     3960,     367,     10939,     660 },
    {2001,     851,     3965,     400,     10879,     720 },
    {2001,     922,     3970,     434,     10819,     780 },
    {2001,     993,     3975,     467,     10759,     840 },
    {2001,     1063,     3981,     500,     10699,     900 },
    {2001,     1134,     3986,     534,     10639,     960 },
    {2001,     1205,     3992,     567,     10579,     1020 },
    {2001,     1276,     3997,     600,     10519,     1080 },
    {2001,     1347,     4002,     634,     10459,     1140 },
    {2001,     1418,     4006,     667,     10399,     1200 },
    {2001,     1489,     4011,     701,     10339,     1260 },
    {2001,     1560,     4015,     734,     10279,     1320 },
    {2001,     1631,     4019,     767,     10219,     1380 },
    {2001,     1702,     4023,     801,     10159,     1440 },
    {2001,     1772,     4027,     834,     10099,     1500 },
    {2001,     1843,     4030,     867,     10039,     1560 },
    {2001,     1914,     4034,     901,     9979,     1620 },
    {2001,     1985,     4037,     934,     9919,     1680 },
    {2001,     2056,     4040,     967,     9859,     1740 },
    {2001,     2127,     4043,     1001,     9799,     1800 },
    {2001,     2198,     4045,     1034,     9739,     1860 },
    {2002,     2269,     4047,     1067,     9679,     1920 },
    {2001,     2340,     4049,     1101,     9619,     1980 },
    {2001,     2411,     4051,     1134,     9559,     2040 },
    {2001,     2482,     4053,     1168,     9499,     2100 },
    {2001,     2552,     4054,     1201,     9439,     2160 },
    {2001,     2623,     4056,     1234,     9379,     2220 },
    {2001,     2694,     4057,     1268,     9319,     2280 },
    {2001,     2765,     4059,     1301,     9259,     2340 },
    {2001,     2836,     4060,     1334,     9199,     2400 },
    {2001,     2907,     4062,     1368,     9139,     2460 },
    {2001,     2978,     4063,     1401,     9079,     2520 },
    {2001,     3049,     4065,     1434,     9019,     2580 },
    {2001,     3120,     4067,     1468,     8959,     2640 },
    {2001,     3190,     4069,     1501,     8899,     2700 },
    {2001,     3261,     4071,     1534,     8839,     2760 },
    {2001,     3332,     4073,     1568,     8779,     2820 },
    {2001,     3403,     4075,     1601,     8719,     2880 },
    {2001,     3474,     4076,     1635,     8659,     2940 },
    {2001,     3545,     4078,     1668,     8599,     3000 },
    {2001,     3616,     4081,     1701,     8539,     3060 },
    {2001,     3687,     4083,     1735,     8479,     3120 },
    {2001,     3758,     4085,     1768,     8419,     3180 },
    {2001,     3829,     4089,     1801,     8359,     3240 },
    {2001,     3900,     4091,     1835,     8299,     3300 },
    {2001,     3970,     4094,     1868,     8239,     3360 },
    {2001,     4041,     4097,     1901,     8179,     3420 },
    {2001,     4112,     4100,     1935,     8119,     3480 },
    {2001,     4183,     4103,     1968,     8059,     3540 },
    {2001,     4254,     4106,     2001,     7999,     3600 },
    {2001,     4325,     4109,     2035,     7939,     3660 },
    {2001,     4396,     4113,     2068,     7879,     3720 },
    {2001,     4467,     4116,     2101,     7819,     3780 },
    {2001,     4538,     4120,     2135,     7759,     3840 },
    {2001,     4609,     4123,     2168,     7699,     3900 },
    {2001,     4679,     4127,     2202,     7639,     3960 },
    {2001,     4750,     4130,     2235,     7579,     4020 },
    {2001,     4821,     4134,     2268,     7519,     4080 },
    {2001,     4892,     4138,     2302,     7459,     4140 },
    {2001,     4963,     4141,     2335,     7399,     4200 },
    {2001,     5034,     4146,     2368,     7339,     4260 },
    {2001,     5105,     4151,     2402,     7279,     4320 },
    {2001,     5176,     4155,     2435,     7219,     4380 },
    {2001,     5247,     4160,     2468,     7159,     4440 },
    {2001,     5317,     4164,     2502,     7099,     4500 },
    {2001,     5388,     4169,     2535,     7039,     4560 },
    {2001,     5459,     4173,     2568,     6979,     4620 },
    {2001,     5530,     4178,     2602,     6919,     4680 },
    {2001,     5601,     4182,     2635,     6859,     4740 },
    {2001,     5672,     4187,     2669,     6799,     4800 },
    {2001,     5743,     4191,     2702,     6739,     4860 },
    {2001,     5814,     4196,     2735,     6679,     4920 },
    {2001,     5885,     4201,     2769,     6619,     4980 },
    {2001,     5956,     4206,     2802,     6559,     5040 },
    {2001,     6027,     4211,     2835,     6499,     5100 },
    {2001,     6097,     4216,     2869,     6439,     5160 },
    {2001,     6168,     4221,     2902,     6379,     5220 },
    {2001,     6239,     4227,     2935,     6319,     5280 },
    {2001,     6310,     4232,     2969,     6259,     5340 },
    {2001,     6381,     4238,     3002,     6199,     5400 },
    {2001,     6452,     4244,     3035,     6139,     5460 },
    {2001,     6523,     4250,     3069,     6079,     5520 },
    {2001,     6594,     4256,     3102,     6019,     5580 },
    {2001,     6665,     4263,     3136,     5959,     5640 },
    {2001,     6735,     4269,     3169,     5899,     5700 },
    {2001,     6806,     4276,     3202,     5839,     5760 },
    {2001,     6877,     4283,     3236,     5779,     5820 },
    {2001,     6948,     4290,     3269,     5719,     5880 },
    {2001,     7019,     4298,     3302,     5659,     5940 },
    {2001,     7090,     4307,     3336,     5599,     6000 },
    {2001,     7161,     4315,     3369,     5539,     6060 },
    {2001,     7232,     4323,     3402,     5479,     6120 },
    {2001,     7303,     4332,     3436,     5419,     6180 },
    {2001,     7373,     4340,     3469,     5359,     6240 },
    {2001,     7444,     4348,     3502,     5299,     6300 },
    {2001,     7515,     4356,     3536,     5239,     6360 },
    {2001,     7586,     4364,     3569,     5179,     6420 },
    {2001,     7657,     4371,     3603,     5119,     6480 },
    {2001,     7728,     4379,     3636,     5059,     6540 },
    {2001,     7799,     4386,     3669,     4999,     6600 },
    {2001,     7870,     4394,     3703,     4939,     6660 },
    {1987,     7941,     4400,     3736,     4879,     6720 },      //change here
    {1920,     8010,     4400,     3768,     4819,     6780 },
    {1860,     8077,     4400,     3800,     4759,     6840 },
    {1803,     8142,     4400,     3830,     4699,     6900 },
    {1749,     8205,     4400,     3860,     4639,     6960 },
    {1695,     8266,     4400,     3889,     4579,     7020 },
    {1643,     8325,     4400,     3917,     4519,     7080 },
    {1592,     8382,     4400,     3943,     4459,     7140 },
    {1542,     8437,     4400,     3970,     4399,     7200 },
    {1493,     8491,     4400,     3995,     4339,     7260 },
    {1445,     8543,     4400,     4019,     4279,     7320 },
    {1398,     8594,     4400,     4043,     4219,     7380 },
    {1353,     8642,     4400,     4066,     4159,     7440 },
    {1310,     8689,     4400,     4088,     4099,     7500 },
    {1269,     8735,     4400,     4110,     4039,     7560 },
    {1228,     8779,     4400,     4130,     3979,     7620 },
    {1190,     8822,     4400,     4151,     3919,     7680 },
    {1152,     8863,     4400,     4170,     3859,     7740 },
    {1115,     8904,     4400,     4189,     3799,     7800 },
    {1080,     8943,     4400,     4207,     3739,     7860 },
    {1046,     8980,     4400,     4225,     3679,     7920 },
    {1014,     9017,     4400,     4242,     3619,     7980 },
    {983,     9052,     4400,     4259,     3559,     8040 },
    {951,     9086,     4400,     4275,     3499,     8100 },
    {918,     9119,     4400,     4290,     3439,     8160 },
    {887,     9151,     4400,     4305,     3379,     8220 },
    {857,     9182,     4400,     4320,     3319,     8280 },
    {830,     9212,     4400,     4334,     3259,     8340 },
    {802,     9241,     4400,     4348,     3199,     8400 },
    {777,     9269,     4400,     4361,     3139,     8460 },
    {753,     9296,     4400,     4374,     3079,     8520 },
    {729,     9322,     4400,     4386,     3019,     8580 },
    {706,     9348,     4400,     4398,     2959,     8640 },
    {684,     9372,     4400,     4409,     2899,     8700 },
    {664,     9396,     4400,     4421,     2839,     8760 },
    {644,     9419,     4400,     4432,     2779,     8820 },
    {624,     9442,     4400,     4442,     2719,     8880 },
    {606,     9464,     4400,     4452,     2659,     8940 },
    {588,     9485,     4400,     4462,     2599,     9000 },
    {571,     9505,     4400,     4472,     2539,     9060 },
    {553,     9525,     4400,     4481,     2479,     9120 },
    {537,     9544,     4400,     4490,     2419,     9180 },
    {520,     9563,     4400,     4499,     2359,     9240 },
    {505,     9581,     4400,     4508,     2299,     9300 },
    {490,     9599,     4400,     4516,     2239,     9360 },
    {475,     9616,     4400,     4524,     2179,     9420 },
    {461,     9632,     4400,     4532,     2119,     9480 },
    {448,     9649,     4400,     4539,     2059,     9540 },
    {436,     9664,     4400,     4547,     1999,     9600 },
    {423,     9679,     4400,     4554,     1939,     9660 },
    {411,     9694,     4400,     4561,     1879,     9720 },
    {400,     9709,     4400,     4568,     1819,     9780 },
    {389,     9723,     4400,     4574,     1759,     9840 },
    {378,     9736,     4400,     4581,     1699,     9900 },
    {368,     9749,     4400,     4587,     1639,     9960 },
    {357,     9762,     4400,     4593,     1579,     10020 },
    {346,     9775,     4400,     4599,     1519,     10080 },
    {336,     9787,     4400,     4604,     1459,     10140 },
    {327,     9798,     4400,     4610,     1399,     10200 },
    {317,     9810,     4400,     4615,     1339,     10260 },
    {309,     9821,     4400,     4620,     1279,     10320 },
    {300,     9832,     4400,     4626,     1219,     10380 },
    {292,     9842,     4400,     4630,     1159,     10440 },
    {285,     9852,     4400,     4635,     1099,     10500 },
    {277,     9862,     4400,     4640,     1039,     10560 },
    {270,     9872,     4400,     4645,     979,     10620 },
    {263,     9881,     4400,     4649,     919,     10680 },
    {256,     9891,     4400,     4653,     859,     10740 },
    {250,     9900,     4400,     4658,     799,     10800 },
    {244,     9908,     4400,     4662,     739,     10860 },
    {238,     9917,     4400,     4666,     679,     10920 },
    {232,     9925,     4400,     4670,     619,     10980 },
    {226,     9933,     4400,     4673,     559,     11040 },
    {221,     9941,     4400,     4677,     499,     11100 },
    {215,     9949,     4400,     4681,     439,     11160 },
    {210,     9957,     4400,     4684,     379,     11220 },
    {205,     9964,     4400,     4688,     319,     11280 },
    {200,     9971,     4400,     4691,     259,     11340 },
    {195,     9978,     4400,     4694,     199,     11400 },
    {190,     9985,     4400,     4698,     139,     11460 },
    {186,     9991,     4400,     4701,     79,     11520 },
    {181,     9998,     4400,     4704,     19,     11580 },
    {180,     10000,     4400,     4705,     0,     11599 },
};
#define REMAIN_ARRAY_COUNT (sizeof(remain_zcv)/sizeof(struct remain_guage))

static int remain_guage_cal(unsigned int current_now, unsigned int remain_mAh){
    unsigned char count;
    unsigned int remain_mAh_cal;
    unsigned int remain_second;
    unsigned int charge_soc;
    unsigned int current_t;
    unsigned int remain_mAh_1;
    unsigned int remain_second_1;
    unsigned int qmax;

	count = 0;
	charge_soc = 0;
	remain_mAh_cal = 0;
	remain_second = 0;
    remain_mAh_1 = 0;

    if(current_now == 0) current_now = 1;

    qmax = remain_zcv[REMAIN_ARRAY_COUNT-1].remain_guage;
	do{
		if((current_now >= remain_zcv[count].current_now)&&(remain_mAh >= qmax - remain_zcv[count].remain_guage)){
			charge_soc = remain_zcv[count].charge_soc;
			remain_mAh_cal = qmax - remain_zcv[count].remain_guage;
			remain_second = remain_zcv[count].remain_second;
			current_t = remain_zcv[count].current_now;
	        bm_err("[%s] count:%d ,current_t:%d ,charge_soc:%d ,remain_guage_t:%d ,remain_second_t:%d\n", __func__, \
		         count, current_t, charge_soc, remain_mAh_cal, remain_second);
			break;
		}
	}while(++count < REMAIN_ARRAY_COUNT);
    if(count == REMAIN_ARRAY_COUNT){
        charge_soc = remain_zcv[count-1].charge_soc;
        remain_mAh_cal = qmax - remain_zcv[count-1].remain_guage;
        remain_second = remain_zcv[count-1].remain_second;
        bm_err("[%s] count:%d, current_t: %d ,charge_soc_remain: %d ,remain_guage_t: %d ,remain_second_t: %d\n", __func__, \
	         REMAIN_ARRAY_COUNT, current_t, charge_soc, remain_mAh_cal, remain_second);
    }
    if(remain_mAh > remain_mAh_cal){
        remain_mAh_1 = remain_mAh - remain_mAh_cal;
        remain_second_1 = (remain_mAh_1 * 3600)/current_now;
        remain_second += remain_second_1;
        bm_err("[%s] remain_mAh_1: %d, remain_mAh: %d, remain_second_1:%d\n", __func__, remain_mAh_1, remain_mAh, remain_second_1);
    }else{
        bm_err("[%s] remain_mAh is too small, remain_mAh: %d, remain_mAh_cal:%d\n", __func__,  remain_mAh, remain_mAh_cal);
    }
	msleep(1);
	bm_err("[%s] current_now: %d, remain_mAh: %d, current_t: %d ,charge_soc: %d ,remain_guage_t: %d ,remain_second_t: %d\n", __func__, \
		current_now, remain_mAh, current_t, charge_soc, remain_mAh_cal, remain_second);

	return remain_second;
}
/* End added by dapeng.qiao for task 11603976 on 2021-10-12 */
#endif

#if IS_ENABLED(CONFIG_TCT_CHARGER)
struct mtk_charger *get_mtk_charger(void)
{
	struct mtk_charger *info = NULL;
	struct power_supply *chg_psy = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		pr_notice("%s Couldn't get chg_psy\n", __func__);
		return 0;
	} else {
		info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	}
	return info;
}

static void wake_up_charger(struct mtk_charger *info)
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
#endif

static int battery_psy_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret = 0;

	struct mtk_battery *gm;
	struct battery_data *bs_data;

	gm = (struct mtk_battery *)power_supply_get_drvdata(psy);
	bs_data = &gm->bs_data;

	if (gm->algo.active == true)
		bs_data->bat_capacity = gm->ui_soc;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		//battery_update_chg_status(gm);
// Begin modified by zhangkun for MODEL3-6558 on 2022-12.05
#ifdef CONFIG_TCT_PROJECT_MODEL_3
	if ((bs_data->bat_health == POWER_SUPPLY_HEALTH_COLD)||(bs_data->bat_health == POWER_SUPPLY_HEALTH_OVERHEAT)){
		bs_data->bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
#endif
// End modified by zhangkun for MODEL3-6558 on 2022-12.05
		val->intval = bs_data->bat_status;
       /* Begin added by dapeng.qiao for task 11038299 on 2021-05-1 */
        #ifdef TCT_BMS_SW_SUPPORT
            g_bat_status = bs_data->bat_status;
        #endif
        /* End added by dapeng.qiao for task 11038299 on 2021-05-1 */
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bs_data->bat_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		bs_data->bat_present =
			gauge_get_int_property(GAUGE_PROP_BATTERY_EXIST);
		val->intval = bs_data->bat_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = bs_data->bat_technology;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* 1 = META_BOOT, 4 = FACTORY_BOOT 5=ADVMETA_BOOT */
		/* 6= ATE_factory_boot */
		if (gm->bootmode == 1 || gm->bootmode == 4
			|| gm->bootmode == 5 || gm->bootmode == 6) {
			val->intval = 75;
			break;
		}

		if (gm->fixed_uisoc != 0xffff)
			val->intval = gm->fixed_uisoc;
		else
			val->intval = bs_data->bat_capacity;
/* Begin Added by tangshan.bai for LEVIN-6148 */
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
		if(gm->display_soc != -1){
			val->intval = (gm->display_soc +50) / 100;
			pr_err("peak soc capacity:%d\n",val->intval);
		}
#endif
/* End Added by tangshan.bai for LEVIN-6148 */
/* Begin added by dapeng.qiao for task 11038299 on 2021-05-1 */
#ifdef TCT_BMS_SW_SUPPORT
		g_bat_capacity = bs_data->bat_capacity;
#endif
/* End added by dapeng.qiao for task 11038299 on 2021-05-1 */
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval =
			gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT)
			* 100;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval =
			gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT)
			* 100;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval =
			gm->fg_table_cust_data.fg_profile[
				gm->battery_id].q_max * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = gm->ui_soc *
			gm->fg_table_cust_data.fg_profile[
				gm->battery_id].q_max * 1000 / 100;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		gauge_get_property(GAUGE_PROP_BATTERY_VOLTAGE,
			&bs_data->bat_batt_vol);
		val->intval = bs_data->bat_batt_vol * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		force_get_tbat(gm, true);
		val->intval = gm->tbat_precise;
/* Begin added by hailong.chen for task 9777034 on 2020-08-20 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		if(fixtemp == 1)
			val->intval = fixtemp_val;
#endif
/* End added by hailong.chen for task 9777034 on 2020-08-20 */

/* Begin added by dapeng.qiao for task 11038299 on 2021-05-1 */
#ifdef TCT_BMS_SW_SUPPORT
		g_tbat_precise = gm->tbat_precise;
#endif
/* End added by dapeng.qiao for task 11038299 on 2021-05-1 */
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = check_cap_level(bs_data->bat_capacity);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		/* full or unknown must return 0 */

/* Begin modified by dapeng.qiao for task SOCAOSP13-9123 on 2022-09-12 */
#if defined(CONFIG_TCT_FEATURE_SLEEP_CHARGE) || defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
//#if defined(CONFIG_TCT_PROJECT_RIO_GO) || defined(CONFIG_TCT_PROJECT_PASSAT)
		ret = check_cap_level(bs_data->bat_capacity);
		if ((ret == POWER_SUPPLY_CAPACITY_LEVEL_FULL) ||
			(ret == POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN))
			val->intval = 0;
		else {
			int q_max_now = gm->fg_table_cust_data.fg_profile[
						gm->battery_id].q_max;

            int uisoc = bs_data->bat_capacity;
			int remain_ui = 100 - bs_data->bat_capacity;
			int remain_mah = remain_ui * q_max_now / 10;
            int gm_soc = gm->soc;
            int gm_v_soc = gm->fg_cust_data.v_soc;
            int gm_c_soc = gm->fg_cust_data.c_soc;
            int fg_coulomb = 0;
            int time_to_full = 0;
            int current_avg = gm->sw_iavg;
            int current_now = gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT);
            if (remain_data.enable_smart_slp_chg)
            {
                remain_mah = (10000 - gm->fg_cust_data.v_soc) * q_max_now / 1000;
                fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
                time_to_full = remain_guage_charge_time(current_now/10, remain_mah/10);
                bm_err("TimeToFull:%d,Cavg:%d,Cnow:%d,Rmah:%d,qmax:%d,Car:%d,uisoc:%d,GmSoc:%d,Vsoc:%d,Csoc:%d\n",\
                    time_to_full, current_avg/10, current_now/10, remain_mah/10,\
                    q_max_now, fg_coulomb, uisoc,gm_soc, gm_v_soc, gm_c_soc);
            }else{
                if (current_now != 0)
                    time_to_full = remain_mah * 3600 / current_now;
                bm_debug("time_to_full:%d, remain:ui:%d mah:%d, fgcurrent:%d, qmax:%d\n",
                    time_to_full, remain_ui, remain_mah,
                    current_now, q_max_now);
            }
            val->intval = abs(time_to_full);
#else
		ret = check_cap_level(bs_data->bat_capacity);
		if ((ret == POWER_SUPPLY_CAPACITY_LEVEL_FULL) ||
			(ret == POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN))
			val->intval = 0;
		else {
			int q_max_now = gm->fg_table_cust_data.fg_profile[
						gm->battery_id].q_max;
			int remain_ui = 100 - bs_data->bat_capacity;
			int remain_mah = remain_ui * q_max_now / 10;
            int gm_soc = gm->soc;
			int remain_mah_c = gm_soc * q_max_now / 10;
            int fg_coulomb = 0;
			int current_now =
			gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT);
			int current_avg = gm->sw_iavg;
            int int_temp = 0;
			int time_to_full = 0;
			int time_to_full2 = 0;
			int time_to_full3 = 0;
			int time_to_full4 = 0;
            bool avg_flag = false;
            bool soc_flag = false;

            fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
            int_temp = current_now - current_avg;
            bm_err("%s %d int_temp:%d\n",__func__, __LINE__, int_temp);
            if(current_avg/abs(int_temp) >10)
                avg_flag = true;
            int_temp = remain_ui*100 - gm_soc;
            bm_err("%s %d int_temp:%d\n",__func__, __LINE__, int_temp);
            if(gm_soc/abs(int_temp) >10)
                soc_flag = true;

            if (current_now > 0){
                if(current_now < 100){
                    time_to_full = remain_guage_cal(100, remain_mah/10);
                    time_to_full2 = remain_mah * 3600 / 100;
                    time_to_full3 = remain_guage_cal(100, remain_mah/10);
                    time_to_full4 = remain_guage_cal(100, remain_mah_c/10);
                }else{
                    time_to_full = remain_guage_cal(current_now/10, remain_mah/10);
                    time_to_full2 = remain_mah * 3600 / current_now;
                    if(avg_flag){
                        time_to_full3 = remain_guage_cal(current_now/10, remain_mah/10);
                    }else{
                        time_to_full3 = remain_guage_cal(current_avg/10, remain_mah/10);
                    }
                    if(soc_flag){
                        time_to_full4 = remain_guage_cal(current_avg/10, remain_mah/10);
                    }else{
                        time_to_full4 = remain_guage_cal(current_avg/10, remain_mah_c/10);
                    }
                }
            }
            bm_err("time_to_full:%d,full2:%d,full3:%d,full4:%d,c_avg:%d,f_avg:%d,f_soc:%d,r_ui:%d,gm_ui:%d,remain_mah:%d,gm_mah:%d,c_now:%d,qmax:%d,car:%d\n",
                time_to_full, time_to_full2, time_to_full3, time_to_full4, current_avg/10, avg_flag, soc_flag, remain_ui,\
                100-gm_soc, remain_mah/10, remain_mah_c/10, current_now/10, q_max_now,fg_coulomb);
            val->intval = abs(time_to_full);
#endif
/* End modified by dapeng.qiao for task SOCAOSP13-9123 on 2022-09-12 */
		}
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (check_cap_level(bs_data->bat_capacity) ==
			POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN)
			val->intval = 0;
		else {
			int q_max_mah = 0;
			int q_max_uah = 0;

			q_max_mah =
				gm->fg_table_cust_data.fg_profile[
				gm->battery_id].q_max / 10;

			q_max_uah = q_max_mah * 1000;
			if (q_max_uah <= 100000) {
				bm_debug("%s q_max_mah:%d q_max_uah:%d\n",
					__func__, q_max_mah, q_max_uah);
				q_max_uah = 100001;
			}
			val->intval = q_max_uah;
		}
		break;

/* Begin added by hailong.chen for task 9777034 on 2020-08-20 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		val->intval = battery_get_icl_settled();
		break;
	case POWER_SUPPLY_PROP_ISENSECURRENT:
		val->intval =
			gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT) * 100;
		break;
	case POWER_SUPPLY_PROP_BATT_ID:
		val->intval = gauge_get_int_property(GAUGE_PROP_BATTERY_ID);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		val->intval = battery_resistance_id;
		break;
	/* begin add by bing-zhang for getting ocv from preloader on 20210827 */
	case POWER_SUPPLY_PROP_OCV_PL:
		val->intval = gm->pl_bat_vol;
		break;
	case POWER_SUPPLY_PROP_SOC_PL:
		val->intval = gm->soc_pl;
		break;
	/* end add by bing-zhang for getting ocv from preloader on 20210827 */
	case POWER_SUPPLY_PROP_COULOMB_COUNT:
		val->intval = gauge_get_int_property(GAUGE_PROP_COULOMB);
		break;
	case POWER_SUPPLY_PROP_TCL_FIXTEMP:
		val->intval = fixtemp;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !gm->chg_disable;
		break;
	case POWER_SUPPLY_PROP_BATTERY_TYPE:
		val->strval =  gm->battery_type;
		break;
	case POWER_SUPPLY_PROP_DEBUG_BATTERY:
		val->intval = gm->is_debug_battery;
		break;
/* Begin Added by tangshan.bai for LEVIN-6148 */
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
	case POWER_SUPPLY_PROP_PEAK_LEVEL:
		val->intval = gm->BAT_peak_level;
		break;
	case POWER_SUPPLY_PROP_BATTERY_VERIFY:
		val->intval = gm->BatteryVerify;
		break;
	case POWER_SUPPLY_PROP_BATTERY_VSOC:
		val->intval = gm->fg_cust_data.v_soc;
		break;
	case POWER_SUPPLY_PROP_BATT_RESISTANCE:
		val->strval = gm->batt_resistance;
		break;
	case POWER_SUPPLY_PROP_CHARGING_CYCLE_TABLE:
		val->strval = gm->charge_cycle_table;
		break;
	case POWER_SUPPLY_PROP_BATTERYUSOC:
		// val->intval = (gm->display_soc +50)/100;
		val->intval = gm->display_soc;
		break;
#endif
/* End Added by tangshan.bai for LEVIN-6148 */
#endif
/* End added by hailong.chen for task 9777034 on 2020-08-20 */
	default:
		ret = -EINVAL;
		break;
		}

	bm_debug("%s psp:%d ret:%d val:%d",
		__func__, psp, ret, val->intval);

	return ret;
}

#if 0
#if IS_ENABLED(CONFIG_TCT_CHARGER)
struct mtk_charger *get_mtk_charger(void)
{
	struct mtk_charger *info = NULL;
	struct power_supply *chg_psy = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		pr_notice("%s Couldn't get chg_psy\n", __func__);
		return 0;
	} else {
		info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	}
	return info;
}

static void wake_up_charger(struct mtk_charger *info)
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
#endif
#endif

/* Begin added by hailong.chen for task 9777034 on 2020-08-20 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
static int battery_psy_set_property(struct power_supply *psy,
	enum power_supply_property psp,
	const union power_supply_propval *val)
{
	struct mtk_charger *info = get_mtk_charger();
	struct mtk_battery *gm;
	int ret = 0;

	gm = psy->drv_data;
	switch (psp) {
	case POWER_SUPPLY_PROP_TCL_FIXTEMP:
/* Begin mod by jin.wang to split fixtemp and fixtemp_val */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		fixtemp = val->intval >> 16;
		fixtemp_val = val->intval & 0xFFFF;
#else
		fixtemp = val->intval;
#endif
/* End mod by jin.wang */
		break;
	/* Begin added by bitao.xiong for task-9895401 on 2020-09-11 */
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		gm->chg_disable = !val->intval;
		wake_up_charger(info);
		break;
/* Begin Added by tangshan.bai for LEVIN-6148 */
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
	case POWER_SUPPLY_PROP_PEAK_LEVEL:
		gm->BAT_peak_level = val->intval;
		bm_err("%s: userspace write decimal peak_level:%d\n", __func__,val->intval);
        break;
	case POWER_SUPPLY_PROP_BATTERY_VERIFY:
        gm->BatteryVerify &= 0x000000F0;    //here clear BIT(3)
        fg_max_monotone = 0;
        if(val->intval & BIT(3)){
            gm->BatteryVerify |= BIT(3);
//            if (gm->bs_data.bat_status == POWER_SUPPLY_STATUS_CHARGING){
                gm->BatteryVerify |= BIT(1);
//            }
        }else{
            gm->BatteryVerify |= BIT(0);
        }
        bm_err("%s: userspace write Hexadecimal,only 0x0008 will be accept, not care other bit, BatteryVerify:%x,charging:%d\n",\
            __func__,gm->BatteryVerify,gm->bs_data.bat_status);
        break;
    case POWER_SUPPLY_PROP_BATTERYUSOC:
    		if(!gm->gauge->hw_status.rtc_invalid){
				// gm->display_soc = val->intval*100;
				gm->display_soc = val->intval;
				bm_err("%s: userspace write usoc:%d\n", __func__,val->intval);
    		}
    		else
				bm_err("%s: userspace rtc_invalid:%d, usoc:%d\n", __func__,gm->gauge->hw_status.rtc_invalid,val->intval);
    	break;
#endif
/* End Added by tangshan.bai for LEVIN-6148 */
	/* End added by bitao.xiong for task-9895401 on 2020-09-11 */
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int battery_psy_property_is_writeable(struct power_supply *psy,
	enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_TCL_FIXTEMP:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
/* Begin Added by tangshan.bai for LEVIN-6148 */
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
	case POWER_SUPPLY_PROP_PEAK_LEVEL:
	case POWER_SUPPLY_PROP_BATTERY_VERIFY:
	case POWER_SUPPLY_PROP_BATTERYUSOC:
#endif
/* End Added by tangshan.bai for LEVIN-6148 */
		return 1;
	default:
		break;
	}
	return 0;
}
#endif
/* End added by hailong.chen for task 9777034 on 2020-08-20 */

/* Begin Added by tangshan.bai for LEVIN-6148 */
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
static int bserachFirstOverVlaue(struct peak_guage *nums, unsigned int length, unsigned int value) {
    int low = 0;
    int high = length - 1;

    if(high <= 0) return 0;
    if(value >= nums[high].fg_v_soc) return length-1;
    while (low <= high) {
        int mid = low + ((high - low) >> 1);
        if(low == high) return mid;
        if (nums[mid].fg_v_soc > value) {
            if ((mid == 0) || nums[mid-1].fg_v_soc <= value){
                if(mid){
                    return mid-1;
                }
            }
            else high = mid - 1;
        } else {
            low = mid + 1;
        }
    }
    return length-1;
}

enum BSEARCH_TYPE{
    BEGIN_V_SOC,
    STOP_V_SOC,
};
static int bserach_begin_log_coulomb(enum BSEARCH_TYPE type)
{
    unsigned int count_1;
    unsigned int array_count;
    int log_coulomb;
    int v_soc;
    struct peak_guage *peak_zcv;
    struct mtk_battery *gm;

    gm = get_mtk_battery();
    peak_zcv = &peak_data.peak_zcv[0];
    array_count = peak_data.peak_array_count;
    log_coulomb = 0;
    switch(type){
    case BEGIN_V_SOC:
        v_soc = gm->g_begin_v_soc;
        break;
    case STOP_V_SOC:
        v_soc = gm->g_stop_v_soc;
        break;
    defalut:
        break;
    }

    count_1 = bserachFirstOverVlaue(peak_zcv, array_count, v_soc);
    log_coulomb = peak_zcv[count_1].fg_coulomb;

    return log_coulomb;
}

int record_fg_maxvalue_batteryverify(void){
    int fg_stop;
    int fg_max;
    struct mtk_battery *gm;

    gm = get_mtk_battery();
    fg_stop = 0;
    fg_max = 0;
    fg_stop = gauge_get_int_property(GAUGE_PROP_COULOMB);
    if(gm->BatteryVerify & BIT(3)) {
        fg_max = fg_max_monotone;
        if(fg_max < fg_stop){
            fg_max = fg_stop;
            fg_max_monotone = fg_max;
        }
        if(gm->BatteryVerify & BIT(2)) {
            return fg_max;
        }
        if((10000 == gm->fg_cust_data.v_soc) && (fg_stop <= 0)){
            return fg_max;
        }
    }else{
        fg_max = fg_stop;
        fg_max_monotone = fg_max;
    }
    if(fg_stop != fg_max_monotone){
        bm_err("%s %d, fg_max_monotone:%d,fg_max:%d\n",__func__, __LINE__,fg_max_monotone,fg_stop);
    }
    return fg_max;
}
#define LIFE_ARRAY_LENGTH 10
unsigned int life_array[LIFE_ARRAY_LENGTH];
unsigned int life_array_avg;
unsigned int life_cnt;
unsigned long cal_batt_life(unsigned int log_vsoc_start,unsigned int log_vsoc_stop,int fg_start,int fg_stop){

    static unsigned long cal_life;
    int coulomb_diff;
    int cal_coulomb_diff;
    int l_begin_log_coulomb;
    int l_stop_log_coulomb;
    unsigned int count_1;
    unsigned int count_2;
    unsigned int count_life;
    unsigned int array_count;
    struct peak_guage *peak_zcv;
    struct mtk_battery *gm;

    gm = get_mtk_battery();
    peak_zcv = &peak_data.peak_zcv[0];
    array_count = peak_data.peak_array_count;
    count_1 = 0;
    count_2 = 0;
    count_life = 0;
    if(gm->BatteryVerify & BIT(2)) {
        return cal_life;
    }
    if((10000 == gm->fg_cust_data.v_soc) && (fg_stop <= 0)){
        return cal_life;
    }
    cal_life = 0;
    coulomb_diff = abs(fg_stop - fg_start);
    cal_coulomb_diff = 0;
    count_1 = bserachFirstOverVlaue(peak_zcv, array_count, log_vsoc_start);
    count_2 = bserachFirstOverVlaue(peak_zcv, array_count, log_vsoc_stop);
    l_begin_log_coulomb = peak_zcv[count_1].fg_coulomb;
    l_stop_log_coulomb = peak_zcv[count_2].fg_coulomb;
    cal_coulomb_diff = abs(l_stop_log_coulomb - l_begin_log_coulomb);
    if((cal_coulomb_diff >= 1) || (cal_coulomb_diff <= -1)){
        cal_life = (100*coulomb_diff/cal_coulomb_diff);
    }
    if(cal_life >= 0xFF){
        cal_life = 0xFF;
    }
    count_life = life_cnt % LIFE_ARRAY_LENGTH;
    life_array[count_life] = cal_life;
    life_cnt++;
    count_life = 0;
    life_array_avg = 0;
    for(count_life = 0; count_life < LIFE_ARRAY_LENGTH; count_life++){
        life_array_avg +=  life_array[count_life];
    }
    bm_err("life_array,1:%d,2:%d,3:%d,4:%d,5:%d,6:%d,7:%d,8:%d,9:%d,10:%d,life_cnt:%d\n",life_array[0],life_array[1],\
    life_array[2],life_array[3],life_array[4],life_array[5],life_array[6],life_array[7],life_array[8],life_array[9],life_cnt);
    life_array_avg /=  LIFE_ARRAY_LENGTH;

    cal_life = life_array_avg;
    if(gm->BatteryVerify & BIT(1)) {
        gm->BatteryVerify &= 0x00FF;
        gm->BatteryVerify |= (cal_life << 8);
    }

    bm_err("[%s] life:%ld cnt1:%d,cnt2:%d,bvsoc:%d,svsoc:%d,BlCar:%d,SlCar:%d,LFgDiff:%d,BCar:%d,SCar:%d,FgDiff:%d,Verify:0x%x,life_array_avg:%d\n", \
        __func__, cal_life, count_1, count_2, log_vsoc_start, log_vsoc_stop,
        l_begin_log_coulomb, l_stop_log_coulomb, cal_coulomb_diff, fg_start, fg_stop, coulomb_diff,gm->BatteryVerify,life_array_avg);

    return cal_life;
}

void calculate_coulomb_charged(enum chg_alg_notifier_events plug_state){
    int v_soc_diff;
    int coulomb_diff;
    long cal_coulomb_diff;
    unsigned int q_max;
    int cal_life;
    struct mtk_battery *gm;
    struct peak_guage *peak_zcv;
    int array_count ;
    unsigned int count_1;
    unsigned int count_2;

    peak_zcv = &peak_data.peak_zcv[0];
    array_count = peak_data.peak_array_count;
    count_1 = 0;
    count_2 = 0;
    v_soc_diff = 0;
    coulomb_diff = 0;
    cal_coulomb_diff = 0;
    cal_life = 0;
    gm = get_mtk_battery();

    if(gm->BatteryVerify & BIT(3)) {
        if(gm->BatteryVerify & BIT(2)) {
             bm_err("%s,Already full, if recaculate, reclear BatteryVerify 0x%x\n",__func__, gm->BatteryVerify);
             return;
        }else{
            bm_err("%s, Allow start BatteryVerify:0x%x\n",__func__, gm->BatteryVerify);
            gm->BatteryVerify &= 0x000000F8;
        }
    }

    switch (plug_state) {
    case EVT_PLUG_OUT:
    case EVT_FULL:
        gm->g_stop_fg_coulomb = record_fg_maxvalue_batteryverify();
        gm->g_stop_v_soc = gm->fg_cust_data.v_soc;
        gm->g_stop_c_soc = gm->fg_cust_data.c_soc;
        v_soc_diff = gm->g_stop_v_soc - gm->g_begin_v_soc;
        coulomb_diff = abs(gm->g_stop_fg_coulomb - gm->g_begin_fg_coulomb);
        q_max = gm->fg_table_cust_data.fg_profile[gm->battery_id].q_max;
    	gm->k_daemon = gm->daemon_uisoc;
    	gm->k_display = gm->display_soc;

        if(gm->BatteryVerify & BIT(3)) {
            bm_err("%s: Bvsoc=%d,Svsoc=%d,Bfg=%d,Sfg=%d,q_max=%d,fgdiff=%d,QmaxTab:%d\n", \
            __func__,gm->g_begin_v_soc,gm->g_stop_v_soc,gm->g_begin_fg_coulomb,gm->g_stop_fg_coulomb,q_max,coulomb_diff,peak_zcv[array_count-1].fg_coulomb);
            count_2 = bserachFirstOverVlaue(peak_zcv, array_count, gm->g_stop_v_soc);
            gm->g_stop_log_coulomb = peak_zcv[count_2].fg_coulomb;
            cal_coulomb_diff = abs(gm->g_stop_log_coulomb - gm->g_begin_log_coulomb);
            if((cal_coulomb_diff >= 1) || (cal_coulomb_diff <= -1)){
                cal_life = (int)(100*coulomb_diff/cal_coulomb_diff);//5000*40/2000
            }
            if(cal_life >= 0xFF){
                cal_life = 0xFF;
            }
            bm_err("%s %d,life_array_avg 0x%x\n",__func__, __LINE__,life_array_avg);
            if(0 < life_array_avg){
                cal_life = life_array_avg;
            }
            bm_err("%s line:%d, BatteryVerify:0x%x,life:%ld,life_array_avg:%d\n",__func__, __LINE__,gm->BatteryVerify,cal_life,life_array_avg);
            gm->BatteryVerify |= (cal_life << 8);
            bm_err("%s line:%d, verify stop, BatteryVerify:0x%x,v_soc_diff:%d\n",__func__, __LINE__,gm->BatteryVerify,v_soc_diff);

            if(EVT_FULL == plug_state){
                gm->BatteryVerify |= BIT(2);       //calculate state and hide 0% life
            }else{
                if(v_soc_diff >= 7000){
                    gm->BatteryVerify |= BIT(2);
                }else{
                    gm->BatteryVerify |= BIT(0);       //END
                }
            }
        }
        bm_err("%s %d,plug out, BatteryVerify 0x%x\n",__func__, __LINE__,gm->BatteryVerify);
        break;
    case EVT_PLUG_IN:
    case EVT_RECHARGE:
        gm->g_begin_fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
        fg_max_monotone = gm->g_begin_fg_coulomb;
        gm->g_begin_v_soc = gm->fg_cust_data.v_soc;
        gm->g_begin_c_soc = gm->fg_cust_data.c_soc;
        gm->g_stop_fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
        gm->g_stop_v_soc = gm->fg_cust_data.v_soc;
        gm->g_stop_c_soc = gm->fg_cust_data.c_soc;
    	gm->k_daemon = 10000 - gm->daemon_uisoc;
    	gm->k_display = 10000 - gm->display_soc;
    	if (work_busy(&gm->tracking_to_zero_work.work))//cancel battery tracking to zero work
    		cancel_delayed_work_sync(&gm->tracking_to_zero_work);

        if(gm->BatteryVerify & BIT(3)) {
            bm_err("%s, Allow start BatteryVerify:0x%x\n",__func__, gm->BatteryVerify);
            gm->BatteryVerify |= BIT(1);       //calculate state and hide 0% life
        }
        count_1 = bserachFirstOverVlaue(peak_zcv, array_count, gm->g_begin_v_soc);
        gm->g_begin_log_coulomb = peak_zcv[count_1].fg_coulomb;
        gm->g_stop_log_coulomb = gm->g_begin_log_coulomb;
        cal_coulomb_diff = 0;
        cal_life = 0;
        memset(&life_array, 0, sizeof(life_array));
        life_array_avg = 0;
        life_cnt = 0;

        if(EVT_PLUG_IN == plug_state)
            bm_err("%s,plug in, BatteryVerify 0x%x\n",__func__, gm->BatteryVerify);
        else
            bm_err("%s,recharge, BatteryVerify 0x%x\n",__func__, gm->BatteryVerify);
        break;
    defalut:
        break;
    }
    bm_err("[%s] life:%ld,cnt1:%d,cnt2:%d,1vsoc:%d,2vsoc:%d,1lcar:%d,2lcar:%d,lfgdiff:%d,1car:%d,2car:%d,fgdiff:%d,Verify:0x%x\n", \
    __func__, cal_life, count_1, count_2, gm->g_begin_v_soc, gm->g_stop_v_soc, gm->g_begin_log_coulomb, \
    gm->g_stop_log_coulomb, cal_coulomb_diff, gm->g_begin_fg_coulomb,gm->g_stop_fg_coulomb,coulomb_diff,gm->BatteryVerify);
}
#endif
/* End Added by tangshan.bai for LEVIN-6148 */

/* Begin add by jin.wang for jira 2064 on 2021-11-30 */
//#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
#if 0
static void battery_update_chg_status(struct mtk_battery *gm)
{
	struct battery_data *bs_data = NULL;
	struct charger_device *primary_chg = NULL;
	union power_supply_propval online = {0,};
	union power_supply_propval status = {POWER_SUPPLY_STATUS_UNKNOWN,};
	int ret = 0;

	bs_data = &gm->bs_data;
	bs_data->bat_status = POWER_SUPPLY_STATUS_UNKNOWN;

	if (IS_ERR_OR_NULL(bs_data->chg_psy)) {
		bm_err("%s retry to get chg_psy\n", __func__);
		bs_data->chg_psy =
			devm_power_supply_get_by_phandle(
				&gm->gauge->pdev->dev, "charger");
	}
	if (IS_ERR_OR_NULL(bs_data->chg_psy)) {
		bm_err("%s can't find chg_psy\n", __func__);
		return;
	}

	ret = power_supply_get_property(bs_data->chg_psy,
		POWER_SUPPLY_PROP_ONLINE, &online);
	if (!ret) {
		if (!online.intval) {
			bs_data->bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			ret = power_supply_get_property(bs_data->chg_psy,
				POWER_SUPPLY_PROP_STATUS, &status);
			if(ret) {
				pr_err("V0: Get status from %s failed use primary_chg\n",
						bs_data->chg_psy->desc->name);
				primary_chg = get_charger_by_name("primary_chg");
				charger_dev_get_charging_status(primary_chg,
							&status.intval);
			}
			bs_data->bat_status = status.intval;
		}
	}
}

static void mtk_battery_external_power_changed(struct power_supply *psy)
{
	struct mtk_battery *gm;
	struct battery_data *bs_data;
	union power_supply_propval online = {0,};
	union power_supply_propval prop_type = {POWER_SUPPLY_USB_TYPE_UNKNOWN,};
	int cur_chr_type;

	struct power_supply *chg_psy = NULL;
	int ret;

	gm = psy->drv_data;
	bs_data = &gm->bs_data;
	chg_psy = bs_data->chg_psy;

	if (IS_ERR_OR_NULL(chg_psy)) {
		chg_psy = devm_power_supply_get_by_phandle(&gm->gauge->pdev->dev,
							   "charger");
		bm_err("%s retry to get chg_psy\n", __func__);
		bs_data->chg_psy = chg_psy;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &online);
		if (!ret && online.intval) {
			fg_sw_bat_cycle_accu(gm);
		}

		battery_update_chg_status(gm);
		if (bs_data->bat_status == POWER_SUPPLY_STATUS_FULL
			&& gm->b_EOC != true) {
			bm_err("POWER_SUPPLY_STATUS_FULL\n");
			gm->b_EOC = true;
			notify_fg_chr_full(gm);
		} else
			gm->b_EOC = false;

		battery_update(gm);

		/* check charger type */
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop_type);

		/* plug in out */
		cur_chr_type = prop_type.intval;
		if (cur_chr_type == POWER_SUPPLY_USB_TYPE_UNKNOWN) {
			if (gm->chr_type != POWER_SUPPLY_USB_TYPE_UNKNOWN)
				wakeup_fg_algo(gm, FG_INTR_CHARGER_OUT);
		} else {
			if (gm->chr_type == POWER_SUPPLY_USB_TYPE_UNKNOWN)
				wakeup_fg_algo(gm, FG_INTR_CHARGER_IN);
		}
	}

	bm_err("%s: V3: name:%s online:%d, status:%d, EOC:%d, cur_chr_type:%d old:%d\n",
		__func__, psy->desc->name, online.intval, bs_data->bat_status,
		gm->b_EOC, cur_chr_type, gm->chr_type);

	gm->chr_type = cur_chr_type;
}
#else
static void mtk_battery_external_power_changed(struct power_supply *psy)
{
	struct mtk_battery *gm;
	struct battery_data *bs_data;
	union power_supply_propval online, status;
	union power_supply_propval prop_type;
	int cur_chr_type;

	struct power_supply *chg_psy = NULL;
	int ret;

//Begin Modified by qiuguangliang for 10941217 on 2021-04-23
	struct charger_device *primary_chg = NULL;
	bool charge_done = 0;
//End Modified by qiuguangliang for 10941217 on 2021-04-23

	gm = psy->drv_data;
	bs_data = &gm->bs_data;
	chg_psy = bs_data->chg_psy;

	if (IS_ERR_OR_NULL(chg_psy)) {
		chg_psy = devm_power_supply_get_by_phandle(&gm->gauge->pdev->dev,
							   "charger");
		bm_err("%s retry to get chg_psy\n", __func__);
		/* Begin added by bitao.xiong for alm-11715746 on 2022-03-19 */
		if (IS_ERR_OR_NULL(chg_psy)) {
			primary_chg = get_charger_by_name("primary_chg");
			if (!IS_ERR_OR_NULL(primary_chg) && strlen(primary_chg->props.alias_name) != 0)
				chg_psy = power_supply_get_by_name(primary_chg->props.alias_name);
		}
		/* End added by bitao.xiong for alm-11715746 on 2022-03-19 */
		bs_data->chg_psy = chg_psy;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &online);

		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_STATUS, &status);

		if (!online.intval)
			bs_data->bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
		else {
			/* Begin added by bitao.xiong for RAUSC-1032 on 2022-05-25 */
			if ((bs_data->bat_health == POWER_SUPPLY_HEALTH_WARM)
				&& status.intval == POWER_SUPPLY_STATUS_FULL) {
				pr_err("soc %d, force bat_status\n", bs_data->bat_capacity);
				status.intval = POWER_SUPPLY_STATUS_CHARGING;
			}
			/* End added by bitao.xiong for RAUSC-1032 on 2022-05-25 */

            /*modify begin by weijun for fully charged showing on 2022.2.10*/
			if (status.intval == POWER_SUPPLY_STATUS_NOT_CHARGING) {
				bs_data->bat_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
			} else if (status.intval == POWER_SUPPLY_STATUS_FULL) {
				bs_data->bat_status =
					POWER_SUPPLY_STATUS_FULL;
            } else {
				bs_data->bat_status =
					POWER_SUPPLY_STATUS_CHARGING;
            }
            /*modify end by weijun for fully charged showing on 2022.2.10*/
			fg_sw_bat_cycle_accu(gm);
		}

//Begin Modified by qiuguangliang for 11043275 on 2021-04-23
		if(ret) {
			printk("Get status from %s failed use primary_chg \n", chg_psy->desc->name);
			primary_chg = get_charger_by_name("primary_chg");
			charger_dev_is_charging_done(primary_chg, &charge_done);
			if(charge_done) {
				status.intval = POWER_SUPPLY_STATUS_FULL;
				bs_data->bat_status = POWER_SUPPLY_STATUS_FULL;
			}
		}
//End Modified by qiuguangliang for 11043275 on 2021-04-23
		if (status.intval == POWER_SUPPLY_STATUS_FULL
			&& gm->b_EOC != true) {
			bm_err("POWER_SUPPLY_STATUS_FULL\n");
			gm->b_EOC = true;
			notify_fg_chr_full(gm);
		} else
			gm->b_EOC = false;

		battery_update(gm);

		/* check charger type */
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop_type);

		/* plug in out */
		cur_chr_type = prop_type.intval;

/* Begin modified by jin.wang for task 11395446 on 2021-08-06 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		if (cur_chr_type == POWER_SUPPLY_USB_TYPE_UNKNOWN) {
			if (gm->chr_type != POWER_SUPPLY_USB_TYPE_UNKNOWN)
				wakeup_fg_algo(gm, FG_INTR_CHARGER_OUT);
		} else {
			if (gm->chr_type == POWER_SUPPLY_USB_TYPE_UNKNOWN)
				wakeup_fg_algo(gm, FG_INTR_CHARGER_IN);
		}
#else
		if (cur_chr_type == POWER_SUPPLY_TYPE_UNKNOWN) {
			if (gm->chr_type != POWER_SUPPLY_TYPE_UNKNOWN)
				wakeup_fg_algo(gm, FG_INTR_CHARGER_OUT);
		} else {
			if (gm->chr_type == POWER_SUPPLY_TYPE_UNKNOWN)
				wakeup_fg_algo(gm, FG_INTR_CHARGER_IN);
		}
#endif
/* End modified by jin.wang for task 11395446 on 2021-08-06 */
	}

	bm_err("%s event, name:%s online:%d, status:%d, EOC:%d, cur_chr_type:%d old:%d\n",
		__func__, psy->desc->name, online.intval, status.intval,
		gm->b_EOC, cur_chr_type, gm->chr_type);

	gm->chr_type = cur_chr_type;

}
#endif
/* End mod by jin.wang */

void battery_service_data_init(struct mtk_battery *gm)
{
	struct battery_data *bs_data;

	bs_data = &gm->bs_data;
	bs_data->psd.name = "battery",
	bs_data->psd.type = POWER_SUPPLY_TYPE_BATTERY;
	bs_data->psd.properties = battery_props;
	bs_data->psd.num_properties = ARRAY_SIZE(battery_props);
	bs_data->psd.get_property = battery_psy_get_property;
	/* Begin added by hailong.chen for task 9777034 on 2020-08-20 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	bs_data->psd.set_property = battery_psy_set_property;
	bs_data->psd.property_is_writeable = battery_psy_property_is_writeable;
#endif
	/* End added by hailong.chen for task 9777034 on 2020-08-20 */
	bs_data->psd.external_power_changed =
		mtk_battery_external_power_changed;
	bs_data->psy_cfg.drv_data = gm;

	bs_data->bat_status = POWER_SUPPLY_STATUS_DISCHARGING,
	bs_data->bat_health = POWER_SUPPLY_HEALTH_GOOD,
	bs_data->bat_present = 1,
	bs_data->bat_technology = POWER_SUPPLY_TECHNOLOGY_LION,
	bs_data->bat_capacity = -1,
	bs_data->bat_batt_vol = 0,
	bs_data->bat_batt_temp = 0,

	gm->fixed_uisoc = 0xffff;
}

/* ============================================================ */
/* voltage to battery temperature */
/* ============================================================ */
int BattThermistorConverTemp(struct mtk_battery *gm, int Res)
{
	int i = 0;
	int RES1 = 0, RES2 = 0;
	int TBatt_Value = -2000, TMP1 = 0, TMP2 = 0;
	struct fuelgauge_temperature *ptable;

	ptable = gm->tmp_table;
	if (Res >= ptable[0].TemperatureR) {
		TBatt_Value = -400;
	} else if (Res <= ptable[20].TemperatureR) {
		TBatt_Value = 600;
	} else {
		RES1 = ptable[0].TemperatureR;
		TMP1 = ptable[0].BatteryTemp;

		for (i = 0; i <= 20; i++) {
			if (Res >= ptable[i].TemperatureR) {
				RES2 = ptable[i].TemperatureR;
				TMP2 = ptable[i].BatteryTemp;
				break;
			}
			{	/* hidden else */
				RES1 = ptable[i].TemperatureR;
				TMP1 = ptable[i].BatteryTemp;
			}
		}

		TBatt_Value = (((Res - RES2) * TMP1) +
			((RES1 - Res) * TMP2)) * 10 / (RES1 - RES2);
	}
	bm_debug("[%s] %d %d %d %d %d %d\n",
		__func__,
		RES1, RES2, Res, TMP1,
		TMP2, TBatt_Value);

	return TBatt_Value;
}

int BattVoltToTemp(struct mtk_battery *gm, int dwVolt, int volt_cali)
{
	long long TRes_temp;
	long long TRes;
	int sBaTTMP = -100;
	int vbif28 = gm->rbat.rbat_pull_up_volt;
	int delta_v;
	int vbif28_raw;
	int ret;

	TRes_temp = (gm->rbat.rbat_pull_up_r * (long long) dwVolt);
	ret = gauge_get_property(GAUGE_PROP_BIF_VOLTAGE,
		&vbif28_raw);

	if (ret != -ENOTSUPP) {
		vbif28 = vbif28_raw + volt_cali;
		delta_v = abs(vbif28 - dwVolt);
		if (delta_v == 0)
			delta_v = 1;
#if IS_ENABLED(__LP64__) || IS_ENABLED(_LP64)
			do_div(TRes_temp, delta_v);
#else
			TRes_temp = div_s64(TRes_temp, delta_v);
#endif
		if (vbif28 > 3000 || vbif28 < 1700)
			bm_debug("[RBAT_PULL_UP_VOLT_BY_BIF] vbif28:%d\n",
				vbif28_raw);
	} else {
		delta_v = abs(gm->rbat.rbat_pull_up_volt - dwVolt);
		if (delta_v == 0)
			delta_v = 1;
#if IS_ENABLED(__LP64__) || IS_ENABLED(_LP64)
		do_div(TRes_temp, delta_v);
#else
		TRes_temp = div_s64(TRes_temp, delta_v);
#endif
	}

#if IS_ENABLED(RBAT_PULL_DOWN_R)
	TRes = (TRes_temp * RBAT_PULL_DOWN_R);

#if IS_ENABLED(__LP64__) || IS_ENABLED(_LP64)
	do_div(TRes, abs(RBAT_PULL_DOWN_R - TRes_temp));
#else
	TRes_temp = div_s64(TRes, abs(RBAT_PULL_DOWN_R - TRes_temp));
#endif

#else
	TRes = TRes_temp;
#endif

	sBaTTMP = BattThermistorConverTemp(gm, (int)TRes);

/* Begin mod by jin.wang task 2064 on 2021.11.25 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pr_err("[%s]-v2: %d %d %d %d %lld %d\n",
		__func__,
		dwVolt, gm->rbat.rbat_pull_up_r,
		vbif28, volt_cali, TRes, sBaTTMP);
#else
	bm_debug("[%s] %d %d %d %d\n",
		__func__,
		dwVolt, gm->rbat.rbat_pull_up_r,
		vbif28, volt_cali);
#endif
/* End mod by jin.wang */

	return sBaTTMP;
}

int force_get_tbat_internal(struct mtk_battery *gm, bool update)
{
	int bat_temperature_volt = 2;
	int bat_temperature_val = 0;
	static int pre_bat_temperature_val = -1;
	int fg_r_value = 0;
	int fg_meter_res_value = 0;
	int fg_current_temp = 0;
	bool fg_current_state = false;
	int bat_temperature_volt_temp = 0;
	int vol_cali = 0;
	static int pre_bat_temperature_volt_temp, pre_bat_temperature_volt;
	static int pre_fg_current_temp;
	static int pre_fg_current_state;
	static int pre_fg_r_value;
	static int pre_bat_temperature_val2;
	static struct timespec pre_time;
	struct timespec ctime, dtime;

	if (update == true || pre_bat_temperature_val == -1) {
		/* Get V_BAT_Temperature */
		gauge_get_property(GAUGE_PROP_BATTERY_TEMPERATURE_ADC,
			&bat_temperature_volt);

		if (bat_temperature_volt != 0) {
			fg_r_value = gm->fg_cust_data.com_r_fg_value;
			if (gm->no_bat_temp_compensate == 0)
				fg_meter_res_value =
				gm->fg_cust_data.com_fg_meter_resistance;
			else
				fg_meter_res_value = 0;

			gauge_get_property(GAUGE_PROP_BATTERY_CURRENT,
				&fg_current_temp);

			if (fg_current_temp > 0)
				fg_current_state = true;

			fg_current_temp = abs(fg_current_temp) / 10;

			if (fg_current_state == true) {
				bat_temperature_volt_temp =
					bat_temperature_volt;
				bat_temperature_volt =
				bat_temperature_volt -
				((fg_current_temp *
					(fg_meter_res_value + fg_r_value))
						/ 10000);
				vol_cali =
					-((fg_current_temp *
					(fg_meter_res_value + fg_r_value))
						/ 10000);
			} else {
				bat_temperature_volt_temp =
					bat_temperature_volt;
				bat_temperature_volt =
				bat_temperature_volt +
				((fg_current_temp *
				(fg_meter_res_value + fg_r_value)) / 10000);
				vol_cali =
					((fg_current_temp *
					(fg_meter_res_value + fg_r_value))
					/ 10000);
			}

			bat_temperature_val =
				BattVoltToTemp(gm,
				bat_temperature_volt,
				vol_cali);

/* Begin mod by jin.wang for btemp compensation on 2021-11-23 */
//#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
#if 0
			if ((fg_current_state == true)
				&& (fg_current_temp >= 1000)) {
				if (bat_temperature_val >= 450) {
					bat_temperature_val += 40;
				} else if (bat_temperature_val >= 250) {
					bat_temperature_val += 30;
				} else if (bat_temperature_val >= 0) {
					bat_temperature_val += 20;
				}
			}
#endif
/* End mod by jin.wang */
		}

/* Begin mod by jin.wang task 2064 on 2021.11.25 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		pr_err("[%s] %d,%d,%d,%d,%d,%d r:%d %d %d\n",
			__func__,
			bat_temperature_volt_temp, bat_temperature_volt,
			fg_current_state, fg_current_temp,
			fg_r_value, bat_temperature_val,
			fg_meter_res_value, fg_r_value,
			gm->no_bat_temp_compensate);
#else
		bm_notice("[%s] %d,%d,%d,%d,%d,%d r:%d %d %d\n",
			__func__,
			bat_temperature_volt_temp, bat_temperature_volt,
			fg_current_state, fg_current_temp,
			fg_r_value, bat_temperature_val,
			fg_meter_res_value, fg_r_value,
			gm->no_bat_temp_compensate);
#endif
/* End mod by jin.wang */

		if (pre_bat_temperature_val2 == 0) {
			pre_bat_temperature_volt_temp =
				bat_temperature_volt_temp;
			pre_bat_temperature_volt = bat_temperature_volt;
			pre_fg_current_temp = fg_current_temp;
			pre_fg_current_state = fg_current_state;
			pre_fg_r_value = fg_r_value;
			pre_bat_temperature_val2 = bat_temperature_val;
			get_monotonic_boottime(&pre_time);
		} else {
			get_monotonic_boottime(&ctime);
			dtime = timespec_sub(ctime, pre_time);

			if (((dtime.tv_sec <= 20) &&
				(abs(pre_bat_temperature_val2 -
				bat_temperature_val) >= 50)) ||
				bat_temperature_val >= 580) {
				bm_err("[%s][err] current:%d,%d,%d,%d,%d,%d pre:%d,%d,%d,%d,%d,%d\n",
					__func__,
					bat_temperature_volt_temp,
					bat_temperature_volt,
					fg_current_state,
					fg_current_temp,
					fg_r_value,
					bat_temperature_val,
					pre_bat_temperature_volt_temp,
					pre_bat_temperature_volt,
					pre_fg_current_state,
					pre_fg_current_temp,
					pre_fg_r_value,
					pre_bat_temperature_val2);
				/*pmic_auxadc_debug(1);*/
				WARN_ON(1);
			}

			pre_bat_temperature_volt_temp =
				bat_temperature_volt_temp;
			pre_bat_temperature_volt = bat_temperature_volt;
			pre_fg_current_temp = fg_current_temp;
			pre_fg_current_state = fg_current_state;
			pre_fg_r_value = fg_r_value;
			pre_bat_temperature_val2 = bat_temperature_val;
			pre_time = ctime;
			bm_trace(
				"[%s] current:%d,%d,%d,%d,%d,%d pre:%d,%d,%d,%d,%d,%d time:%d\n",
				__func__,
				bat_temperature_volt_temp, bat_temperature_volt,
				fg_current_state, fg_current_temp,
				fg_r_value, bat_temperature_val,
				pre_bat_temperature_volt_temp,
				pre_bat_temperature_volt,
				pre_fg_current_state, pre_fg_current_temp,
				pre_fg_r_value,
				pre_bat_temperature_val2, (int)dtime.tv_sec);
		}
	} else {
		bat_temperature_val = pre_bat_temperature_val;
	}

/* Begin added by bin.song.hz for task 10480451 on 2020-12-18 */
#if defined(DISABLE_TEMPERATURE_DETECTION_AND_THERMAL_POLICY) \
	|| defined(TARGET_BUILD_MMITEST)
    bm_err("[%s] force 25C,actual temp is %d\n", __func__, bat_temperature_val);
    bat_temperature_val = 250;
#endif
/* End added by bin.song.hz for task 10480451 on 2020-12-18 */

	gm->tbat_precise = bat_temperature_val;

	return bat_temperature_val / 10;
}

int force_get_tbat(struct mtk_battery *gm, bool update)
{
	int bat_temperature_val = 0;

	if (gm->is_probe_done == false) {
		gm->tbat_precise = 250;
		gm->cur_bat_temp = 25;
		return 25;
	}

	if (gm->fixed_bat_tmp != 0xffff) {
		gm->cur_bat_temp = gm->fixed_bat_tmp;
		gm->tbat_precise = gm->fixed_bat_tmp * 10;
		return gm->fixed_bat_tmp;
	}

	bat_temperature_val = force_get_tbat_internal(gm, true);
	gm->cur_bat_temp = bat_temperature_val;

	return bat_temperature_val;
}

/* ============================================================ */
/* gaugel hal interface */
/* ============================================================ */
int gauge_get_property(enum gauge_property gp,
	int *val)
{
	struct mtk_gauge *gauge;
	struct power_supply *psy;
	struct mtk_gauge_sysfs_field_info *attr;

	psy = power_supply_get_by_name("mtk-gauge");
	if (psy == NULL)
		return -ENODEV;

	gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);
	attr = gauge->attr;

	if (attr == NULL) {
		bm_err("%s attr =NULL\n", __func__);
		return -ENODEV;
	}
	if (attr[gp].prop == gp) {
		mutex_lock(&gauge->ops_lock);
		attr[gp].get(gauge, &attr[gp], val);
		mutex_unlock(&gauge->ops_lock);
	} else {
		bm_err("%s gp:%d idx error\n", __func__, gp);
		return -ENOTSUPP;
	}

	return 0;
}

int gauge_get_int_property(enum gauge_property gp)
{
	int val;

	gauge_get_property(gp, &val);
	return val;
}

int gauge_set_property(enum gauge_property gp,
	int val)
{
	struct mtk_gauge *gauge;
	struct power_supply *psy;
	struct mtk_gauge_sysfs_field_info *attr;

	psy = power_supply_get_by_name("mtk-gauge");
	if (psy == NULL)
		return -ENODEV;

	gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);
	attr = gauge->attr;

	if (attr == NULL) {
		bm_err("%s attr =NULL\n", __func__);
		return -ENODEV;
	}
	if (attr[gp].prop == gp) {
		mutex_lock(&gauge->ops_lock);
		attr[gp].set(gauge, &attr[gp], val);
		mutex_unlock(&gauge->ops_lock);
	} else {
		bm_err("%s gp:%d idx error\n", __func__, gp);
		return -ENOTSUPP;
	}

	return 0;
}

/* ============================================================ */
/* load .h/dtsi */
/* ============================================================ */

void fg_custom_init_from_header(struct mtk_battery *gm)
{
	int i, j;
	struct fuel_gauge_custom_data *fg_cust_data;
	struct fuel_gauge_table_custom_data *fg_table_cust_data;
	int version = 0;

	fg_cust_data = &gm->fg_cust_data;
	fg_table_cust_data = &gm->fg_table_cust_data;

	fgauge_get_profile_id();

	fg_cust_data->versionID1 = FG_DAEMON_CMD_FROM_USER_NUMBER;
	fg_cust_data->versionID2 = sizeof(gm->fg_cust_data);
	fg_cust_data->versionID3 = FG_KERNEL_CMD_FROM_USER_NUMBER;

	if (gm->gauge != NULL) {
		gauge_get_property(GAUGE_PROP_HW_VERSION, &version);
		fg_cust_data->hardwareVersion = version;
		fg_cust_data->pl_charger_status =
			gm->gauge->hw_status.pl_charger_status;
	}

	fg_cust_data->q_max_L_current = Q_MAX_L_CURRENT;
	fg_cust_data->q_max_H_current = Q_MAX_H_CURRENT;
	fg_cust_data->q_max_sys_voltage =
		UNIT_TRANS_10 * g_Q_MAX_SYS_VOLTAGE[gm->battery_id];

	fg_cust_data->pseudo1_en = PSEUDO1_EN;
	fg_cust_data->pseudo100_en = PSEUDO100_EN;
	fg_cust_data->pseudo100_en_dis = PSEUDO100_EN_DIS;
	fg_cust_data->pseudo1_iq_offset = UNIT_TRANS_100 *
		g_FG_PSEUDO1_OFFSET[gm->battery_id];

	/* iboot related */
	fg_cust_data->qmax_sel = QMAX_SEL;
	fg_cust_data->iboot_sel = IBOOT_SEL;
	fg_cust_data->shutdown_system_iboot = SHUTDOWN_SYSTEM_IBOOT;

	/* multi-temp gague 0% related */
	fg_cust_data->multi_temp_gauge0 = MULTI_TEMP_GAUGE0;

	/*hw related */
	fg_cust_data->car_tune_value = UNIT_TRANS_10 * CAR_TUNE_VALUE;
	fg_cust_data->fg_meter_resistance = FG_METER_RESISTANCE;
	fg_cust_data->com_fg_meter_resistance = FG_METER_RESISTANCE;
	fg_cust_data->r_fg_value = UNIT_TRANS_10 * R_FG_VALUE;
	fg_cust_data->com_r_fg_value = UNIT_TRANS_10 * R_FG_VALUE;

	/* Aging Compensation */
	fg_cust_data->aging_one_en = AGING_ONE_EN;
	fg_cust_data->aging1_update_soc = UNIT_TRANS_100 * AGING1_UPDATE_SOC;
	fg_cust_data->aging1_load_soc = UNIT_TRANS_100 * AGING1_LOAD_SOC;
	fg_cust_data->aging4_update_soc = UNIT_TRANS_100 * AGING4_UPDATE_SOC;
	fg_cust_data->aging4_load_soc = UNIT_TRANS_100 * AGING4_LOAD_SOC;
	fg_cust_data->aging5_update_soc = UNIT_TRANS_100 * AGING5_UPDATE_SOC;
	fg_cust_data->aging5_load_soc = UNIT_TRANS_100 * AGING5_LOAD_SOC;
	fg_cust_data->aging6_update_soc = UNIT_TRANS_100 * AGING6_UPDATE_SOC;
	fg_cust_data->aging6_load_soc = UNIT_TRANS_100 * AGING6_LOAD_SOC;
	fg_cust_data->aging_temp_diff = AGING_TEMP_DIFF;
	fg_cust_data->aging_temp_low_limit = AGING_TEMP_LOW_LIMIT;
	fg_cust_data->aging_temp_high_limit = AGING_TEMP_HIGH_LIMIT;
	fg_cust_data->aging_100_en = AGING_100_EN;
	fg_cust_data->difference_voltage_update = DIFFERENCE_VOLTAGE_UPDATE;
	fg_cust_data->aging_factor_min = UNIT_TRANS_100 * AGING_FACTOR_MIN;
	fg_cust_data->aging_factor_diff = UNIT_TRANS_100 * AGING_FACTOR_DIFF;
	/* Aging Compensation 2*/
	fg_cust_data->aging_two_en = AGING_TWO_EN;
	/* Aging Compensation 3*/
	fg_cust_data->aging_third_en = AGING_THIRD_EN;
	fg_cust_data->aging_4_en = AGING_4_EN;
	fg_cust_data->aging_5_en = AGING_5_EN;
	fg_cust_data->aging_6_en = AGING_6_EN;

	/* ui_soc related */
	fg_cust_data->diff_soc_setting = DIFF_SOC_SETTING;
	fg_cust_data->keep_100_percent = UNIT_TRANS_100 * KEEP_100_PERCENT;
	fg_cust_data->difference_full_cv = DIFFERENCE_FULL_CV;
	fg_cust_data->diff_bat_temp_setting = DIFF_BAT_TEMP_SETTING;
	fg_cust_data->diff_bat_temp_setting_c = DIFF_BAT_TEMP_SETTING_C;
	fg_cust_data->discharge_tracking_time = DISCHARGE_TRACKING_TIME;
	fg_cust_data->charge_tracking_time = CHARGE_TRACKING_TIME;
	fg_cust_data->difference_fullocv_vth = DIFFERENCE_FULLOCV_VTH;
	fg_cust_data->difference_fullocv_ith =
		UNIT_TRANS_10 * DIFFERENCE_FULLOCV_ITH;
	fg_cust_data->charge_pseudo_full_level = CHARGE_PSEUDO_FULL_LEVEL;
	fg_cust_data->over_discharge_level = OVER_DISCHARGE_LEVEL;
	fg_cust_data->full_tracking_bat_int2_multiply =
		FULL_TRACKING_BAT_INT2_MULTIPLY;

	/* pre tracking */
	fg_cust_data->fg_pre_tracking_en = FG_PRE_TRACKING_EN;
	fg_cust_data->vbat2_det_time = VBAT2_DET_TIME;
	fg_cust_data->vbat2_det_counter = VBAT2_DET_COUNTER;
	fg_cust_data->vbat2_det_voltage1 = VBAT2_DET_VOLTAGE1;
	fg_cust_data->vbat2_det_voltage2 = VBAT2_DET_VOLTAGE2;
	fg_cust_data->vbat2_det_voltage3 = VBAT2_DET_VOLTAGE3;

	/* sw fg */
	fg_cust_data->difference_fgc_fgv_th1 = DIFFERENCE_FGC_FGV_TH1;
	fg_cust_data->difference_fgc_fgv_th2 = DIFFERENCE_FGC_FGV_TH2;
	fg_cust_data->difference_fgc_fgv_th3 = DIFFERENCE_FGC_FGV_TH3;
	fg_cust_data->difference_fgc_fgv_th_soc1 = DIFFERENCE_FGC_FGV_TH_SOC1;
	fg_cust_data->difference_fgc_fgv_th_soc2 = DIFFERENCE_FGC_FGV_TH_SOC2;
	fg_cust_data->nafg_time_setting = NAFG_TIME_SETTING;
	fg_cust_data->nafg_ratio = NAFG_RATIO;
	fg_cust_data->nafg_ratio_en = NAFG_RATIO_EN;
	fg_cust_data->nafg_ratio_tmp_thr = NAFG_RATIO_TMP_THR;
	fg_cust_data->nafg_resistance = NAFG_RESISTANCE;

	/* ADC resistor  */
	fg_cust_data->r_charger_1 = R_CHARGER_1;
	fg_cust_data->r_charger_2 = R_CHARGER_2;

	/* mode select */
	fg_cust_data->pmic_shutdown_current = PMIC_SHUTDOWN_CURRENT;
	fg_cust_data->pmic_shutdown_sw_en = PMIC_SHUTDOWN_SW_EN;
	fg_cust_data->force_vc_mode = FORCE_VC_MODE;
	fg_cust_data->embedded_sel = EMBEDDED_SEL;
	fg_cust_data->loading_1_en = LOADING_1_EN;
	fg_cust_data->loading_2_en = LOADING_2_EN;
	fg_cust_data->diff_iavg_th = DIFF_IAVG_TH;

	fg_cust_data->shutdown_gauge0 = SHUTDOWN_GAUGE0;
	fg_cust_data->shutdown_1_time = SHUTDOWN_1_TIME;
	fg_cust_data->shutdown_gauge1_xmins = SHUTDOWN_GAUGE1_XMINS;
	fg_cust_data->shutdown_gauge0_voltage = SHUTDOWN_GAUGE0_VOLTAGE;
	fg_cust_data->shutdown_gauge1_vbat_en = SHUTDOWN_GAUGE1_VBAT_EN;
	fg_cust_data->shutdown_gauge1_vbat = SHUTDOWN_GAUGE1_VBAT;
	fg_cust_data->power_on_car_chr = POWER_ON_CAR_CHR;
	fg_cust_data->power_on_car_nochr = POWER_ON_CAR_NOCHR;
	fg_cust_data->shutdown_car_ratio = SHUTDOWN_CAR_RATIO;

	/* ZCV update */
	fg_cust_data->zcv_suspend_time = ZCV_SUSPEND_TIME;
	fg_cust_data->sleep_current_avg = SLEEP_CURRENT_AVG;
	fg_cust_data->zcv_car_gap_percentage = ZCV_CAR_GAP_PERCENTAGE;

	/* dod_init */
	fg_cust_data->hwocv_oldocv_diff = HWOCV_OLDOCV_DIFF;
	fg_cust_data->hwocv_oldocv_diff_chr = HWOCV_OLDOCV_DIFF_CHR;
	fg_cust_data->hwocv_swocv_diff = HWOCV_SWOCV_DIFF;
	fg_cust_data->hwocv_swocv_diff_lt = HWOCV_SWOCV_DIFF_LT;
	fg_cust_data->hwocv_swocv_diff_lt_temp = HWOCV_SWOCV_DIFF_LT_TEMP;
	fg_cust_data->swocv_oldocv_diff = SWOCV_OLDOCV_DIFF;
	fg_cust_data->swocv_oldocv_diff_chr = SWOCV_OLDOCV_DIFF_CHR;
	fg_cust_data->vbat_oldocv_diff = VBAT_OLDOCV_DIFF;
	fg_cust_data->swocv_oldocv_diff_emb = SWOCV_OLDOCV_DIFF_EMB;
	fg_cust_data->vir_oldocv_diff_emb = VIR_OLDOCV_DIFF_EMB;
	fg_cust_data->vir_oldocv_diff_emb_lt = VIR_OLDOCV_DIFF_EMB_LT;
	fg_cust_data->vir_oldocv_diff_emb_tmp = VIR_OLDOCV_DIFF_EMB_TMP;

	fg_cust_data->pmic_shutdown_time = UNIT_TRANS_60 * PMIC_SHUTDOWN_TIME;
	fg_cust_data->tnew_told_pon_diff = TNEW_TOLD_PON_DIFF;
	fg_cust_data->tnew_told_pon_diff2 = TNEW_TOLD_PON_DIFF2;
	gm->ext_hwocv_swocv = EXT_HWOCV_SWOCV;
	gm->ext_hwocv_swocv_lt = EXT_HWOCV_SWOCV_LT;
	gm->ext_hwocv_swocv_lt_temp = EXT_HWOCV_SWOCV_LT_TEMP;

	fg_cust_data->dc_ratio_sel = DC_RATIO_SEL;
	fg_cust_data->dc_r_cnt = DC_R_CNT;

	fg_cust_data->pseudo1_sel = PSEUDO1_SEL;

	fg_cust_data->d0_sel = D0_SEL;
	fg_cust_data->dlpt_ui_remap_en = DLPT_UI_REMAP_EN;

	fg_cust_data->aging_sel = AGING_SEL;
	fg_cust_data->bat_par_i = BAT_PAR_I;

	fg_cust_data->fg_tracking_current = FG_TRACKING_CURRENT;
	fg_cust_data->fg_tracking_current_iboot_en =
		FG_TRACKING_CURRENT_IBOOT_EN;
	fg_cust_data->ui_fast_tracking_en = UI_FAST_TRACKING_EN;
	fg_cust_data->ui_fast_tracking_gap = UI_FAST_TRACKING_GAP;

	fg_cust_data->bat_plug_out_time = BAT_PLUG_OUT_TIME;
	fg_cust_data->keep_100_percent_minsoc = KEEP_100_PERCENT_MINSOC;

	fg_cust_data->uisoc_update_type = UISOC_UPDATE_TYPE;

	fg_cust_data->battery_tmp_to_disable_gm30 = BATTERY_TMP_TO_DISABLE_GM30;
	fg_cust_data->battery_tmp_to_disable_nafg = BATTERY_TMP_TO_DISABLE_NAFG;
	fg_cust_data->battery_tmp_to_enable_nafg = BATTERY_TMP_TO_ENABLE_NAFG;

	fg_cust_data->low_temp_mode = LOW_TEMP_MODE;
	fg_cust_data->low_temp_mode_temp = LOW_TEMP_MODE_TEMP;

	/* current limit for uisoc 100% */
	fg_cust_data->ui_full_limit_en = UI_FULL_LIMIT_EN;
	fg_cust_data->ui_full_limit_soc0 = UI_FULL_LIMIT_SOC0;
	fg_cust_data->ui_full_limit_ith0 = UI_FULL_LIMIT_ITH0;
	fg_cust_data->ui_full_limit_soc1 = UI_FULL_LIMIT_SOC1;
	fg_cust_data->ui_full_limit_ith1 = UI_FULL_LIMIT_ITH1;
	fg_cust_data->ui_full_limit_soc2 = UI_FULL_LIMIT_SOC2;
	fg_cust_data->ui_full_limit_ith2 = UI_FULL_LIMIT_ITH2;
	fg_cust_data->ui_full_limit_soc3 = UI_FULL_LIMIT_SOC3;
	fg_cust_data->ui_full_limit_ith3 = UI_FULL_LIMIT_ITH3;
	fg_cust_data->ui_full_limit_soc4 = UI_FULL_LIMIT_SOC4;
	fg_cust_data->ui_full_limit_ith4 = UI_FULL_LIMIT_ITH4;
	fg_cust_data->ui_full_limit_time = UI_FULL_LIMIT_TIME;

	/* voltage limit for uisoc 1% */
	fg_cust_data->ui_low_limit_en = UI_LOW_LIMIT_EN;
	fg_cust_data->ui_low_limit_soc0 = UI_LOW_LIMIT_SOC0;
	fg_cust_data->ui_low_limit_vth0 = UI_LOW_LIMIT_VTH0;
	fg_cust_data->ui_low_limit_soc1 = UI_LOW_LIMIT_SOC1;
	fg_cust_data->ui_low_limit_vth1 = UI_LOW_LIMIT_VTH1;
	fg_cust_data->ui_low_limit_soc2 = UI_LOW_LIMIT_SOC2;
	fg_cust_data->ui_low_limit_vth2 = UI_LOW_LIMIT_VTH2;
	fg_cust_data->ui_low_limit_soc3 = UI_LOW_LIMIT_SOC3;
	fg_cust_data->ui_low_limit_vth3 = UI_LOW_LIMIT_VTH3;
	fg_cust_data->ui_low_limit_soc4 = UI_LOW_LIMIT_SOC4;
	fg_cust_data->ui_low_limit_vth4 = UI_LOW_LIMIT_VTH4;
	fg_cust_data->ui_low_limit_time = UI_LOW_LIMIT_TIME;

	fg_cust_data->moving_battemp_en = MOVING_BATTEMP_EN;
	fg_cust_data->moving_battemp_thr = MOVING_BATTEMP_THR;

	if (version == GAUGE_HW_V2001) {
		bm_debug("GAUGE_HW_V2001 disable nafg\n");
		fg_cust_data->disable_nafg = 1;
	}

	fg_table_cust_data->active_table_number = ACTIVE_TABLE;

	if (fg_table_cust_data->active_table_number == 0)
		fg_table_cust_data->active_table_number = 5;

	bm_debug("fg active table:%d\n",
		fg_table_cust_data->active_table_number);

	fg_table_cust_data->temperature_tb0 = TEMPERATURE_TB0;
	fg_table_cust_data->temperature_tb1 = TEMPERATURE_TB1;

	fg_table_cust_data->fg_profile[0].size =
		sizeof(fg_profile_t0[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[0].fg_profile,
			&fg_profile_t0[gm->battery_id],
			sizeof(fg_profile_t0[gm->battery_id]));

	fg_table_cust_data->fg_profile[1].size =
		sizeof(fg_profile_t1[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[1].fg_profile,
			&fg_profile_t1[gm->battery_id],
			sizeof(fg_profile_t1[gm->battery_id]));

	fg_table_cust_data->fg_profile[2].size =
		sizeof(fg_profile_t2[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[2].fg_profile,
			&fg_profile_t2[gm->battery_id],
			sizeof(fg_profile_t2[gm->battery_id]));

	fg_table_cust_data->fg_profile[3].size =
		sizeof(fg_profile_t3[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[3].fg_profile,
			&fg_profile_t3[gm->battery_id],
			sizeof(fg_profile_t3[gm->battery_id]));

	fg_table_cust_data->fg_profile[4].size =
		sizeof(fg_profile_t4[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[4].fg_profile,
			&fg_profile_t4[gm->battery_id],
			sizeof(fg_profile_t4[gm->battery_id]));

	fg_table_cust_data->fg_profile[5].size =
		sizeof(fg_profile_t5[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[5].fg_profile,
			&fg_profile_t5[gm->battery_id],
			sizeof(fg_profile_t5[gm->battery_id]));

	fg_table_cust_data->fg_profile[6].size =
		sizeof(fg_profile_t6[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[6].fg_profile,
			&fg_profile_t6[gm->battery_id],
			sizeof(fg_profile_t6[gm->battery_id]));

	fg_table_cust_data->fg_profile[7].size =
		sizeof(fg_profile_t7[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[7].fg_profile,
			&fg_profile_t7[gm->battery_id],
			sizeof(fg_profile_t7[gm->battery_id]));

	fg_table_cust_data->fg_profile[8].size =
		sizeof(fg_profile_t8[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[8].fg_profile,
			&fg_profile_t8[gm->battery_id],
			sizeof(fg_profile_t8[gm->battery_id]));

	fg_table_cust_data->fg_profile[9].size =
		sizeof(fg_profile_t9[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[9].fg_profile,
			&fg_profile_t9[gm->battery_id],
			sizeof(fg_profile_t9[gm->battery_id]));

	for (i = 0; i < MAX_TABLE; i++) {
		struct fuelgauge_profile_struct *p;

		p = &fg_table_cust_data->fg_profile[i].fg_profile[0];
		fg_table_cust_data->fg_profile[i].temperature =
			g_temperature[i];
		fg_table_cust_data->fg_profile[i].q_max =
			g_Q_MAX[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].q_max_h_current =
			g_Q_MAX_H_CURRENT[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].pseudo1 =
			UNIT_TRANS_100 * g_FG_PSEUDO1[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].pseudo100 =
			UNIT_TRANS_100 * g_FG_PSEUDO100[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].pmic_min_vol =
			g_PMIC_MIN_VOL[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].pon_iboot =
			g_PON_SYS_IBOOT[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].qmax_sys_vol =
			g_QMAX_SYS_VOL[i][gm->battery_id];
		/* shutdown_hl_zcv */
		fg_table_cust_data->fg_profile[i].shutdown_hl_zcv =
			g_SHUTDOWN_HL_ZCV[i][gm->battery_id];

		for (j = 0; j < 100; j++)
			if (p[j].resistance2 == 0)
				p[j].resistance2 = p[j].resistance;
	}

	/* init battery temperature table */
	gm->rbat.type = 10;
	gm->rbat.rbat_pull_up_r = RBAT_PULL_UP_R;
	gm->rbat.rbat_pull_up_volt = RBAT_PULL_UP_VOLT;
	gm->rbat.bif_ntc_r = BIF_NTC_R;

	if (IS_ENABLED(BAT_NTC_47)) {
		gm->rbat.type = 47;
		gm->rbat.rbat_pull_up_r = RBAT_PULL_UP_R;
	}
}

#if IS_ENABLED(CONFIG_OF)
static int fg_read_dts_val(const struct device_node *np,
		const char *node_srting,
		int *param, int unit)
{
	static unsigned int val;

	if (!of_property_read_u32(np, node_srting, &val)) {
		*param = (int)val * unit;
		bm_debug("Get %s: %d\n",
			 node_srting, *param);
	} else {
		bm_err("Get %s failed\n", node_srting);
		return -1;
	}
	return 0;
}

static int fg_read_dts_val_by_idx(const struct device_node *np,
		const char *node_srting,
		int idx, int *param, int unit)
{
	unsigned int val;

	if (!of_property_read_u32_index(np, node_srting, idx, &val)) {
		*param = (int)val * unit;
		bm_debug("Get %s %d: %d\n",
			 node_srting, idx, *param);
	} else {
		bm_err("Get %s failed, idx %d\n", node_srting, idx);
		return -1;
	}
	return 0;
}

static void fg_custom_parse_table(struct mtk_battery *gm,
		const struct device_node *np,
		const char *node_srting,
		struct fuelgauge_profile_struct *profile_struct, int column)
{
	int mah, voltage, resistance, idx, saddles, resistance2;
	struct fuelgauge_profile_struct *profile_p;

	profile_p = profile_struct;

	saddles = gm->fg_table_cust_data.fg_profile[0].size;
	idx = 0;

	bm_err("%s: %s, %d, column:%d\n",
		__func__,
		node_srting, saddles, column);

	while (!of_property_read_u32_index(np, node_srting, idx, &mah)) {
		idx++;
		if (!of_property_read_u32_index(
			np, node_srting, idx, &voltage)) {
		}
		idx++;
		if (!of_property_read_u32_index(
				np, node_srting, idx, &resistance)) {
		}
		idx++;
		if (column == 4) {
			if (!of_property_read_u32_index(
				np, node_srting, idx, &resistance2))
				idx++;
		} else
			resistance2 = resistance;

		bm_debug("%s: mah: %d, voltage: %d, resistance: %d, resistance2: %d\n",
			__func__, mah, voltage, resistance, resistance2);

		profile_p->mah = mah;
		profile_p->voltage = voltage;
		profile_p->resistance = resistance;
		profile_p->resistance2 = resistance2;
		profile_p++;

		if (idx >= (saddles * column))
			break;
	}

	if (idx == 0) {
		bm_err("[%s] cannot find %s in dts\n",
			__func__, node_srting);
		return;
	}

	profile_p--;

	while (idx < (100 * column)) {
		profile_p++;
		profile_p->mah = mah;
		profile_p->voltage = voltage;
		profile_p->resistance = resistance;
		profile_p->resistance2 = resistance2;
		idx = idx + column;
	}
}

/*begin modify by tangshan.bai for LEVIN-4508 on 20220719--base on ODIN5G-7905*/
#if defined(CONFIG_TCT_FEATURE_SLEEP_CHARGE) || defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
static void smart_slp_chg_table(const struct device_node *np,
		const char *node_srting,
		struct smart_sleep_chrging_data *remain_data)
{
	unsigned int current_now;
	unsigned int charge_soc;
	unsigned int vbat;
	unsigned int charged_guage;
	unsigned int remain_second;
	unsigned int charged_second;
	//unsigned int fg_v_soc;
	//unsigned int fg_coulomb;
	int idx = 0;

	struct remain_guage *remain_p = &remain_data->remain_zcv[0];

	bm_err("%s begin\n",__func__);
	while (!of_property_read_u32_index(np, node_srting, idx, &current_now)) {
		idx++;
		if (!of_property_read_u32_index(
			np, node_srting, idx, &charge_soc)) {
		}
		idx++;
		if (!of_property_read_u32_index(
				np, node_srting, idx, &vbat)) {
		}
		idx++;
		if (!of_property_read_u32_index(
			np, node_srting, idx, &charged_guage)) {
		}
		idx++;
		if (!of_property_read_u32_index(
				np, node_srting, idx, &remain_second)) {
		}
		idx++;
		if (!of_property_read_u32_index(
			np, node_srting, idx, &charged_second)) {
		}
		idx++;
		bm_trace("[c_now:%d c_soc:%d vbat:%d r_fg:%d r_sec:%d c_sec:%d]\n",current_now,
			charge_soc, vbat, charged_guage, remain_second, charged_second);

		remain_p->current_now = current_now;
		remain_p->charge_soc = charge_soc;
		remain_p->vbat = vbat;
		remain_p->charged_guage =charged_guage;
		remain_p->remain_second = remain_second;
		remain_p->charged_second = charged_second;

		remain_p++;
		if (idx >= MAX_DATA_NUMBER*6){
			bm_err("%s:some data are abandoned!!!!\n",__func__);
			break;
		}
	}
	remain_data->remain_array_count = (int)idx/6;
    bm_err("%s stop remain_array_count:%d\n",__func__,remain_data->remain_array_count);
}

static void smart_peak_chg_table(const struct device_node *np,
        const char *node_srting,
        struct smart_peak_management_data *peak_data)
{
    unsigned int fg_v_soc;
    unsigned int fg_coulomb;
    int idx = 0;

    struct peak_guage *peak_p = &peak_data->peak_zcv[0];

    bm_err("%s begin\n",__func__);
    while (!of_property_read_u32_index(np, node_srting, idx, &fg_v_soc)) {
        idx++;
        if (!of_property_read_u32_index(np, node_srting, idx, &fg_coulomb)) {
        }
        idx++;
        bm_err("[vsoc:%d car:%d]\n",fg_v_soc,fg_coulomb);

        peak_p->fg_v_soc = fg_v_soc;
        peak_p->fg_coulomb = fg_coulomb;

        peak_p++;
        if (idx >= MAX_FG_DATA_NUMBER*2){
            bm_err("%s:some data are abandoned!!!!\n",__func__);
            break;
        }
    }
    peak_data->peak_array_count = (int)idx/2;
    bm_err("%s peak_array_count:%d\n",__func__,peak_data->peak_array_count);
}
#endif
/*End modify by tangshan.bai for LEVIN-4508 on 20220719--base on ODIN5G-7905*/

void fg_custom_init_from_dts(struct platform_device *dev,
	struct mtk_battery *gm)
{
	struct device_node *np = dev->dev.of_node;
	unsigned int val;
	int bat_id, multi_battery, active_table, i, j, ret, column;
	char node_name[128];
	struct fuel_gauge_custom_data *fg_cust_data;
	struct fuel_gauge_table_custom_data *fg_table_cust_data;

	gm->battery_id = fgauge_get_profile_id();
	bat_id = gm->battery_id;
	fg_cust_data = &gm->fg_cust_data;
	fg_table_cust_data = &gm->fg_table_cust_data;

	bm_err("%s\n", __func__);

	fg_cust_data->disable_nafg =
		of_property_read_bool(np, "DISABLE_NAFG");
	bm_err("disable_nafg:%d\n",
		fg_cust_data->disable_nafg);

	bm_err("swocv_v:%d swocv_i:%d shutdown_time:%d\n",
		gm->ptim_lk_v, gm->ptim_lk_i, gm->pl_shutdown_time);

	fg_read_dts_val(np, "MULTI_BATTERY", &(multi_battery), 1);
	fg_read_dts_val(np, "ACTIVE_TABLE", &(active_table), 1);

	fg_read_dts_val(np, "Q_MAX_L_CURRENT", &(fg_cust_data->q_max_L_current),
		1);
	fg_read_dts_val(np, "Q_MAX_H_CURRENT", &(fg_cust_data->q_max_H_current),
		1);
	fg_read_dts_val_by_idx(np, "g_Q_MAX_SYS_VOLTAGE", gm->battery_id,
		&(fg_cust_data->q_max_sys_voltage), UNIT_TRANS_10);

	fg_read_dts_val(np, "PSEUDO1_EN", &(fg_cust_data->pseudo1_en), 1);
	fg_read_dts_val(np, "PSEUDO100_EN", &(fg_cust_data->pseudo100_en), 1);
	fg_read_dts_val(np, "PSEUDO100_EN_DIS",
		&(fg_cust_data->pseudo100_en_dis), 1);
	fg_read_dts_val_by_idx(np, "g_FG_PSEUDO1_OFFSET", gm->battery_id,
		&(fg_cust_data->pseudo1_iq_offset), UNIT_TRANS_100);

	/* iboot related */
	fg_read_dts_val(np, "QMAX_SEL", &(fg_cust_data->qmax_sel), 1);
	fg_read_dts_val(np, "IBOOT_SEL", &(fg_cust_data->iboot_sel), 1);
	fg_read_dts_val(np, "SHUTDOWN_SYSTEM_IBOOT",
		&(fg_cust_data->shutdown_system_iboot), 1);

	/*hw related */
	fg_read_dts_val(np, "CAR_TUNE_VALUE", &(fg_cust_data->car_tune_value),
		UNIT_TRANS_10);
	gm->gauge->hw_status.car_tune_value =
		fg_cust_data->car_tune_value;

	fg_read_dts_val(np, "FG_METER_RESISTANCE",
		&(fg_cust_data->fg_meter_resistance), 1);
	ret = fg_read_dts_val(np, "COM_FG_METER_RESISTANCE",
		&(fg_cust_data->com_fg_meter_resistance), 1);
	if (ret == -1)
		fg_cust_data->com_fg_meter_resistance =
			fg_cust_data->fg_meter_resistance;

	fg_read_dts_val(np, "NO_BAT_TEMP_COMPENSATE",
		&(gm->no_bat_temp_compensate), 1);
	fg_read_dts_val(np, "R_FG_VALUE", &(fg_cust_data->r_fg_value),
		UNIT_TRANS_10);
	gm->gauge->hw_status.r_fg_value =
		fg_cust_data->r_fg_value;

	ret = fg_read_dts_val(np, "COM_R_FG_VALUE",
		&(fg_cust_data->com_r_fg_value), UNIT_TRANS_10);
	if (ret == -1)
		fg_cust_data->com_r_fg_value = fg_cust_data->r_fg_value;

	fg_read_dts_val(np, "FULL_TRACKING_BAT_INT2_MULTIPLY",
		&(fg_cust_data->full_tracking_bat_int2_multiply), 1);
	fg_read_dts_val(np, "enable_tmp_intr_suspend",
		&(gm->enable_tmp_intr_suspend), 1);

	/* Aging Compensation */
	fg_read_dts_val(np, "AGING_ONE_EN", &(fg_cust_data->aging_one_en), 1);
	fg_read_dts_val(np, "AGING1_UPDATE_SOC",
		&(fg_cust_data->aging1_update_soc), UNIT_TRANS_100);
	fg_read_dts_val(np, "AGING1_LOAD_SOC",
		&(fg_cust_data->aging1_load_soc), UNIT_TRANS_100);
	fg_read_dts_val(np, "AGING_TEMP_DIFF",
		&(fg_cust_data->aging_temp_diff), 1);
	fg_read_dts_val(np, "AGING_100_EN", &(fg_cust_data->aging_100_en), 1);
	fg_read_dts_val(np, "DIFFERENCE_VOLTAGE_UPDATE",
		&(fg_cust_data->difference_voltage_update), 1);
	fg_read_dts_val(np, "AGING_FACTOR_MIN",
		&(fg_cust_data->aging_factor_min), UNIT_TRANS_100);
	fg_read_dts_val(np, "AGING_FACTOR_DIFF",
		&(fg_cust_data->aging_factor_diff), UNIT_TRANS_100);
	/* Aging Compensation 2*/
	fg_read_dts_val(np, "AGING_TWO_EN", &(fg_cust_data->aging_two_en), 1);
	/* Aging Compensation 3*/
	fg_read_dts_val(np, "AGING_THIRD_EN", &(fg_cust_data->aging_third_en),
		1);

	/* ui_soc related */
	fg_read_dts_val(np, "DIFF_SOC_SETTING",
		&(fg_cust_data->diff_soc_setting), 1);
	fg_read_dts_val(np, "KEEP_100_PERCENT",
		&(fg_cust_data->keep_100_percent), UNIT_TRANS_100);
	fg_read_dts_val(np, "DIFFERENCE_FULL_CV",
		&(fg_cust_data->difference_full_cv), 1);
	fg_read_dts_val(np, "DIFF_BAT_TEMP_SETTING",
		&(fg_cust_data->diff_bat_temp_setting), 1);
	fg_read_dts_val(np, "DIFF_BAT_TEMP_SETTING_C",
		&(fg_cust_data->diff_bat_temp_setting_c), 1);
	fg_read_dts_val(np, "DISCHARGE_TRACKING_TIME",
		&(fg_cust_data->discharge_tracking_time), 1);
	fg_read_dts_val(np, "CHARGE_TRACKING_TIME",
		&(fg_cust_data->charge_tracking_time), 1);
	fg_read_dts_val(np, "DIFFERENCE_FULLOCV_VTH",
		&(fg_cust_data->difference_fullocv_vth), 1);
	fg_read_dts_val(np, "DIFFERENCE_FULLOCV_ITH",
		&(fg_cust_data->difference_fullocv_ith), UNIT_TRANS_10);
	fg_read_dts_val(np, "CHARGE_PSEUDO_FULL_LEVEL",
		&(fg_cust_data->charge_pseudo_full_level), 1);
	fg_read_dts_val(np, "OVER_DISCHARGE_LEVEL",
		&(fg_cust_data->over_discharge_level), 1);

	/* pre tracking */
	fg_read_dts_val(np, "FG_PRE_TRACKING_EN",
		&(fg_cust_data->fg_pre_tracking_en), 1);
	fg_read_dts_val(np, "VBAT2_DET_TIME",
		&(fg_cust_data->vbat2_det_time), 1);
	fg_read_dts_val(np, "VBAT2_DET_COUNTER",
		&(fg_cust_data->vbat2_det_counter), 1);
	fg_read_dts_val(np, "VBAT2_DET_VOLTAGE1",
		&(fg_cust_data->vbat2_det_voltage1), 1);
	fg_read_dts_val(np, "VBAT2_DET_VOLTAGE2",
		&(fg_cust_data->vbat2_det_voltage2), 1);
	fg_read_dts_val(np, "VBAT2_DET_VOLTAGE3",
		&(fg_cust_data->vbat2_det_voltage3), 1);

	/* sw fg */
	fg_read_dts_val(np, "DIFFERENCE_FGC_FGV_TH1",
		&(fg_cust_data->difference_fgc_fgv_th1), 1);
	fg_read_dts_val(np, "DIFFERENCE_FGC_FGV_TH2",
		&(fg_cust_data->difference_fgc_fgv_th2), 1);
	fg_read_dts_val(np, "DIFFERENCE_FGC_FGV_TH3",
		&(fg_cust_data->difference_fgc_fgv_th3), 1);
	fg_read_dts_val(np, "DIFFERENCE_FGC_FGV_TH_SOC1",
		&(fg_cust_data->difference_fgc_fgv_th_soc1), 1);
	fg_read_dts_val(np, "DIFFERENCE_FGC_FGV_TH_SOC2",
		&(fg_cust_data->difference_fgc_fgv_th_soc2), 1);
	fg_read_dts_val(np, "NAFG_TIME_SETTING",
		&(fg_cust_data->nafg_time_setting), 1);
	fg_read_dts_val(np, "NAFG_RATIO", &(fg_cust_data->nafg_ratio), 1);
	fg_read_dts_val(np, "NAFG_RATIO_EN", &(fg_cust_data->nafg_ratio_en), 1);
	fg_read_dts_val(np, "NAFG_RATIO_TMP_THR",
		&(fg_cust_data->nafg_ratio_tmp_thr), 1);
	fg_read_dts_val(np, "NAFG_RESISTANCE", &(fg_cust_data->nafg_resistance),
		1);

	/* mode select */
	fg_read_dts_val(np, "PMIC_SHUTDOWN_CURRENT",
		&(fg_cust_data->pmic_shutdown_current), 1);
	fg_read_dts_val(np, "PMIC_SHUTDOWN_SW_EN",
		&(fg_cust_data->pmic_shutdown_sw_en), 1);
	fg_read_dts_val(np, "FORCE_VC_MODE", &(fg_cust_data->force_vc_mode), 1);
	fg_read_dts_val(np, "EMBEDDED_SEL", &(fg_cust_data->embedded_sel), 1);
	fg_read_dts_val(np, "LOADING_1_EN", &(fg_cust_data->loading_1_en), 1);
	fg_read_dts_val(np, "LOADING_2_EN", &(fg_cust_data->loading_2_en), 1);
	fg_read_dts_val(np, "DIFF_IAVG_TH", &(fg_cust_data->diff_iavg_th), 1);

	fg_read_dts_val(np, "SHUTDOWN_GAUGE0", &(fg_cust_data->shutdown_gauge0),
		1);
	fg_read_dts_val(np, "SHUTDOWN_1_TIME", &(fg_cust_data->shutdown_1_time),
		1);
	fg_read_dts_val(np, "SHUTDOWN_GAUGE1_XMINS",
		&(fg_cust_data->shutdown_gauge1_xmins), 1);
	fg_read_dts_val(np, "SHUTDOWN_GAUGE0_VOLTAGE",
		&(fg_cust_data->shutdown_gauge0_voltage), 1);
	fg_read_dts_val(np, "SHUTDOWN_GAUGE1_VBAT_EN",
		&(fg_cust_data->shutdown_gauge1_vbat_en), 1);
	fg_read_dts_val(np, "SHUTDOWN_GAUGE1_VBAT",
		&(fg_cust_data->shutdown_gauge1_vbat), 1);

	/* ZCV update */
	fg_read_dts_val(np, "ZCV_SUSPEND_TIME",
		&(fg_cust_data->zcv_suspend_time), 1);
	fg_read_dts_val(np, "SLEEP_CURRENT_AVG",
		&(fg_cust_data->sleep_current_avg), 1);
	fg_read_dts_val(np, "ZCV_CAR_GAP_PERCENTAGE",
		&(fg_cust_data->zcv_car_gap_percentage), 1);

	/* dod_init */
	fg_read_dts_val(np, "HWOCV_OLDOCV_DIFF",
		&(fg_cust_data->hwocv_oldocv_diff), 1);
	fg_read_dts_val(np, "HWOCV_OLDOCV_DIFF_CHR",
		&(fg_cust_data->hwocv_oldocv_diff_chr), 1);
	fg_read_dts_val(np, "HWOCV_SWOCV_DIFF",
		&(fg_cust_data->hwocv_swocv_diff), 1);
	fg_read_dts_val(np, "HWOCV_SWOCV_DIFF_LT",
		&(fg_cust_data->hwocv_swocv_diff_lt), 1);
	fg_read_dts_val(np, "HWOCV_SWOCV_DIFF_LT_TEMP",
		&(fg_cust_data->hwocv_swocv_diff_lt_temp), 1);
	fg_read_dts_val(np, "SWOCV_OLDOCV_DIFF",
		&(fg_cust_data->swocv_oldocv_diff), 1);
	fg_read_dts_val(np, "SWOCV_OLDOCV_DIFF_CHR",
		&(fg_cust_data->swocv_oldocv_diff_chr), 1);
	fg_read_dts_val(np, "VBAT_OLDOCV_DIFF",
		&(fg_cust_data->vbat_oldocv_diff), 1);
	fg_read_dts_val(np, "SWOCV_OLDOCV_DIFF_EMB",
		&(fg_cust_data->swocv_oldocv_diff_emb), 1);

	fg_read_dts_val(np, "PMIC_SHUTDOWN_TIME",
		&(fg_cust_data->pmic_shutdown_time), UNIT_TRANS_60);
	/* begin add by bing-zhang for getting ocv from preloader on 20210827 */
	fg_read_dts_val(np, "boot_voltage", &gm->pl_bat_vol, 1);
	/* end add by bing-zhang for getting ocv from preloader on 20210827 */
	fg_read_dts_val(np, "TNEW_TOLD_PON_DIFF",
		&(fg_cust_data->tnew_told_pon_diff), 1);
	fg_read_dts_val(np, "TNEW_TOLD_PON_DIFF2",
		&(fg_cust_data->tnew_told_pon_diff2), 1);
	fg_read_dts_val(np, "EXT_HWOCV_SWOCV",
		&(gm->ext_hwocv_swocv), 1);
	fg_read_dts_val(np, "EXT_HWOCV_SWOCV_LT",
		&(gm->ext_hwocv_swocv_lt), 1);
	fg_read_dts_val(np, "EXT_HWOCV_SWOCV_LT_TEMP",
		&(gm->ext_hwocv_swocv_lt_temp), 1);

	fg_read_dts_val(np, "DC_RATIO_SEL", &(fg_cust_data->dc_ratio_sel), 1);
	fg_read_dts_val(np, "DC_R_CNT", &(fg_cust_data->dc_r_cnt), 1);

	fg_read_dts_val(np, "PSEUDO1_SEL", &(fg_cust_data->pseudo1_sel), 1);

	fg_read_dts_val(np, "D0_SEL", &(fg_cust_data->d0_sel), 1);
	fg_read_dts_val(np, "AGING_SEL", &(fg_cust_data->aging_sel), 1);
	fg_read_dts_val(np, "BAT_PAR_I", &(fg_cust_data->bat_par_i), 1);
	fg_read_dts_val(np, "RECORD_LOG", &(fg_cust_data->record_log), 1);


	fg_read_dts_val(np, "FG_TRACKING_CURRENT",
		&(fg_cust_data->fg_tracking_current), 1);
	fg_read_dts_val(np, "FG_TRACKING_CURRENT_IBOOT_EN",
		&(fg_cust_data->fg_tracking_current_iboot_en), 1);
	fg_read_dts_val(np, "UI_FAST_TRACKING_EN",
		&(fg_cust_data->ui_fast_tracking_en), 1);
	fg_read_dts_val(np, "UI_FAST_TRACKING_GAP",
		&(fg_cust_data->ui_fast_tracking_gap), 1);

	fg_read_dts_val(np, "BAT_PLUG_OUT_TIME",
		&(fg_cust_data->bat_plug_out_time), 1);
	fg_read_dts_val(np, "KEEP_100_PERCENT_MINSOC",
		&(fg_cust_data->keep_100_percent_minsoc), 1);

	fg_read_dts_val(np, "UISOC_UPDATE_TYPE",
		&(fg_cust_data->uisoc_update_type), 1);

	fg_read_dts_val(np, "BATTERY_TMP_TO_DISABLE_GM30",
		&(fg_cust_data->battery_tmp_to_disable_gm30), 1);
	fg_read_dts_val(np, "BATTERY_TMP_TO_DISABLE_NAFG",
		&(fg_cust_data->battery_tmp_to_disable_nafg), 1);
	fg_read_dts_val(np, "BATTERY_TMP_TO_ENABLE_NAFG",
		&(fg_cust_data->battery_tmp_to_enable_nafg), 1);

	fg_read_dts_val(np, "LOW_TEMP_MODE", &(fg_cust_data->low_temp_mode), 1);
	fg_read_dts_val(np, "LOW_TEMP_MODE_TEMP",
		&(fg_cust_data->low_temp_mode_temp), 1);

	/* current limit for uisoc 100% */
	fg_read_dts_val(np, "UI_FULL_LIMIT_EN",
		&(fg_cust_data->ui_full_limit_en), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_SOC0",
		&(fg_cust_data->ui_full_limit_soc0), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_ITH0",
		&(fg_cust_data->ui_full_limit_ith0), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_SOC1",
		&(fg_cust_data->ui_full_limit_soc1), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_ITH1",
		&(fg_cust_data->ui_full_limit_ith1), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_SOC2",
		&(fg_cust_data->ui_full_limit_soc2), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_ITH2",
		&(fg_cust_data->ui_full_limit_ith2), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_SOC3",
		&(fg_cust_data->ui_full_limit_soc3), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_ITH3",
		&(fg_cust_data->ui_full_limit_ith3), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_SOC4",
		&(fg_cust_data->ui_full_limit_soc4), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_ITH4",
		&(fg_cust_data->ui_full_limit_ith4), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_TIME",
		&(fg_cust_data->ui_full_limit_time), 1);

	/* voltage limit for uisoc 1% */
	fg_read_dts_val(np, "UI_LOW_LIMIT_EN", &(fg_cust_data->ui_low_limit_en),
		1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_SOC0",
		&(fg_cust_data->ui_low_limit_soc0), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_VTH0",
		&(fg_cust_data->ui_low_limit_vth0), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_SOC1",
		&(fg_cust_data->ui_low_limit_soc1), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_VTH1",
		&(fg_cust_data->ui_low_limit_vth1), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_SOC2",
		&(fg_cust_data->ui_low_limit_soc2), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_VTH2",
		&(fg_cust_data->ui_low_limit_vth2), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_SOC3",
		&(fg_cust_data->ui_low_limit_soc3), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_VTH3",
		&(fg_cust_data->ui_low_limit_vth3), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_SOC4",
		&(fg_cust_data->ui_low_limit_soc4), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_VTH4",
		&(fg_cust_data->ui_low_limit_vth4), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_TIME",
		&(fg_cust_data->ui_low_limit_time), 1);

	/* average battemp */
	fg_read_dts_val(np, "MOVING_BATTEMP_EN",
		&(fg_cust_data->moving_battemp_en), 1);
	fg_read_dts_val(np, "MOVING_BATTEMP_THR",
		&(fg_cust_data->moving_battemp_thr), 1);

	gm->disableGM30 = of_property_read_bool(
		np, "DISABLE_MTKBATTERY");
	fg_read_dts_val(np, "MULTI_TEMP_GAUGE0",
		&(fg_cust_data->multi_temp_gauge0), 1);
	fg_read_dts_val(np, "FGC_FGV_TH1",
		&(fg_cust_data->difference_fgc_fgv_th1), 1);
	fg_read_dts_val(np, "FGC_FGV_TH2",
		&(fg_cust_data->difference_fgc_fgv_th2), 1);
	fg_read_dts_val(np, "FGC_FGV_TH3",
		&(fg_cust_data->difference_fgc_fgv_th3), 1);
	fg_read_dts_val(np, "UISOC_UPDATE_T",
		&(fg_cust_data->uisoc_update_type), 1);
	fg_read_dts_val(np, "UIFULLLIMIT_EN",
		&(fg_cust_data->ui_full_limit_en), 1);
	fg_read_dts_val(np, "MTK_CHR_EXIST", &(fg_cust_data->mtk_chr_exist), 1);

	fg_read_dts_val(np, "GM30_DISABLE_NAFG", &(fg_cust_data->disable_nafg),
		1);
	fg_read_dts_val(np, "FIXED_BATTERY_TEMPERATURE", &(gm->fixed_bat_tmp),
		1);

	fg_read_dts_val(np, "ACTIVE_TABLE",
		&(fg_table_cust_data->active_table_number), 1);

/* Begin modified by hailong.chen for task 9785237 on 2020-10-31 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (fg_table_cust_data->active_table_number == 0)
		fg_table_cust_data->active_table_number = 5;
#else
#if IS_ENABLED(CONFIG_MTK_ADDITIONAL_BATTERY_TABLE)
	if (fg_table_cust_data->active_table_number == 0)
		fg_table_cust_data->active_table_number = 5;
#else
	if (fg_table_cust_data->active_table_number == 0)
		fg_table_cust_data->active_table_number = 4;
#endif
#endif
/* End modified by hailong.chen for task 9785237 on 2020-10-31 */

	bm_err("fg active table:%d\n",
		fg_table_cust_data->active_table_number);

	/* battery temperature  related*/
	fg_read_dts_val(np, "RBAT_PULL_UP_R", &(gm->rbat.rbat_pull_up_r), 1);
	fg_read_dts_val(np, "RBAT_PULL_UP_VOLT",
		&(gm->rbat.rbat_pull_up_volt), 1);

	/* battery temperature, TEMPERATURE_T0 ~ T9 */
	for (i = 0; i < fg_table_cust_data->active_table_number; i++) {
		sprintf(node_name, "TEMPERATURE_T%d", i);
		fg_read_dts_val(np, node_name,
			&(fg_table_cust_data->fg_profile[i].temperature), 1);
		}

	fg_read_dts_val(np, "TEMPERATURE_TB0",
		&(fg_table_cust_data->temperature_tb0), 1);
	fg_read_dts_val(np, "TEMPERATURE_TB1",
		&(fg_table_cust_data->temperature_tb1), 1);

	for (i = 0; i < MAX_TABLE; i++) {
		struct fuelgauge_profile_struct *p;

		p = &fg_table_cust_data->fg_profile[i].fg_profile[0];
		fg_read_dts_val_by_idx(np, "g_temperature", i,
			&(fg_table_cust_data->fg_profile[i].temperature), 1);
		fg_read_dts_val_by_idx(np, "g_Q_MAX",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].q_max), 1);
		fg_read_dts_val_by_idx(np, "g_Q_MAX_H_CURRENT",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].q_max_h_current),
			1);
		fg_read_dts_val_by_idx(np, "g_FG_PSEUDO1",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].pseudo1),
			UNIT_TRANS_100);
		fg_read_dts_val_by_idx(np, "g_FG_PSEUDO100",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].pseudo100),
			UNIT_TRANS_100);
		fg_read_dts_val_by_idx(np, "g_PMIC_MIN_VOL",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].pmic_min_vol), 1);
		fg_read_dts_val_by_idx(np, "g_PON_SYS_IBOOT",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].pon_iboot), 1);
		fg_read_dts_val_by_idx(np, "g_QMAX_SYS_VOL",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].qmax_sys_vol), 1);
		fg_read_dts_val_by_idx(np, "g_SHUTDOWN_HL_ZCV",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].shutdown_hl_zcv),
			1);
		for (j = 0; j < 100; j++) {
			if (p[j].resistance2 == 0)
				p[j].resistance2 = p[j].resistance;
	}
	}

	if (bat_id >= 0 && bat_id < TOTAL_BATTERY_NUMBER) {
		sprintf(node_name, "Q_MAX_SYS_VOLTAGE_BAT%d", bat_id);
		fg_read_dts_val(np, node_name,
			&(fg_cust_data->q_max_sys_voltage), UNIT_TRANS_10);
		sprintf(node_name, "PSEUDO1_IQ_OFFSET_BAT%d", bat_id);
		fg_read_dts_val(np, node_name,
			&(fg_cust_data->pseudo1_iq_offset), UNIT_TRANS_100);
	} else
		bm_err(
		"get Q_MAX_SYS_VOLTAGE_BAT, PSEUDO1_IQ_OFFSET_BAT %d failed\n",
		bat_id);

	if (fg_cust_data->multi_temp_gauge0 == 0) {
		int i = 0;
		int min_vol;

		min_vol = fg_table_cust_data->fg_profile[0].pmic_min_vol;
		if (!of_property_read_u32(np, "PMIC_MIN_VOL", &val)) {
			for (i = 0; i < MAX_TABLE; i++)
				fg_table_cust_data->fg_profile[i].pmic_min_vol =
				(int)val;
				bm_debug("Get PMIC_MIN_VOL: %d\n",
					min_vol);
		} else {
			bm_err("Get PMIC_MIN_VOL failed\n");
		}

		if (!of_property_read_u32(np, "POWERON_SYSTEM_IBOOT", &val)) {
			for (i = 0; i < MAX_TABLE; i++)
				fg_table_cust_data->fg_profile[i].pon_iboot =
				(int)val * UNIT_TRANS_10;

			bm_debug("Get POWERON_SYSTEM_IBOOT: %d\n",
				fg_table_cust_data->fg_profile[0].pon_iboot);
		} else {
			bm_err("Get POWERON_SYSTEM_IBOOT failed\n");
		}
	}

	if (active_table == 0 && multi_battery == 0) {
		fg_read_dts_val(np, "g_FG_PSEUDO100_T0",
			&(fg_table_cust_data->fg_profile[0].pseudo100),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO100_T1",
			&(fg_table_cust_data->fg_profile[1].pseudo100),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO100_T2",
			&(fg_table_cust_data->fg_profile[2].pseudo100),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO100_T3",
			&(fg_table_cust_data->fg_profile[3].pseudo100),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO100_T4",
			&(fg_table_cust_data->fg_profile[4].pseudo100),
			UNIT_TRANS_100);
/* Begin added by hailong.chen for task 9785237 on 2020-10-31 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		fg_read_dts_val(np, "g_FG_PSEUDO1_T0",
			&(fg_table_cust_data->fg_profile[0].pseudo1),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO1_T1",
			&(fg_table_cust_data->fg_profile[1].pseudo1),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO1_T2",
			&(fg_table_cust_data->fg_profile[2].pseudo1),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO1_T3",
			&(fg_table_cust_data->fg_profile[3].pseudo1),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO1_T4",
			&(fg_table_cust_data->fg_profile[4].pseudo1),
			UNIT_TRANS_100);
#endif
/* End added by hailong.chen for task 9785237 on 2020-10-31 */
	}

	/* compatiable with old dtsi*/
	if (active_table == 0) {
		fg_read_dts_val(np, "TEMPERATURE_T0",
			&(fg_table_cust_data->fg_profile[0].temperature), 1);
		fg_read_dts_val(np, "TEMPERATURE_T1",
			&(fg_table_cust_data->fg_profile[1].temperature), 1);
		fg_read_dts_val(np, "TEMPERATURE_T2",
			&(fg_table_cust_data->fg_profile[2].temperature), 1);
		fg_read_dts_val(np, "TEMPERATURE_T3",
			&(fg_table_cust_data->fg_profile[3].temperature), 1);
		fg_read_dts_val(np, "TEMPERATURE_T4",
			&(fg_table_cust_data->fg_profile[4].temperature), 1);
	}

	for (i = 0; i < fg_table_cust_data->active_table_number; i++) {
		sprintf(node_name, "battery%d_profile_t%d_num", bat_id, i);
		fg_read_dts_val(np, node_name,
			&(fg_table_cust_data->fg_profile[i].size), 1);

		/* compatiable with old dtsi table*/
		sprintf(node_name, "battery%d_profile_t%d_col", bat_id, i);
		ret = fg_read_dts_val(np, node_name, &(column), 1);
		if (ret == -1)
			column = 3;

		if (column < 3 || column > 4) {
			bm_err("%s, %s,column:%d ERROR!",
				__func__, node_name, column);
			/* correction */
			column = 3;
	}

/* Begin added by hailong.chen for task 9785237 on 2020-10-31 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	sprintf(node_name, "g_Q_MAX_T%d_%d", i, bat_id);
	fg_read_dts_val(np, node_name,
		&(fg_table_cust_data->fg_profile[i].q_max), 1);
	sprintf(node_name, "g_Q_MAX_T%d_H_CURRENT_%d", i, bat_id);
	fg_read_dts_val(np, node_name,
		&(fg_table_cust_data->fg_profile[i].q_max_h_current), 1);
	sprintf(node_name, "g_SHUTDOWN_HL_ZCV_T%d_%d", i, bat_id);
	fg_read_dts_val(np, node_name,
		&(fg_table_cust_data->fg_profile[i].shutdown_hl_zcv), 1);
#endif
/* End added by hailong.chen for task 9785237 on 2020-10-31 */

		sprintf(node_name, "battery%d_profile_t%d", bat_id, i);
		fg_custom_parse_table(gm, np, node_name,
			fg_table_cust_data->fg_profile[i].fg_profile, column);
	}
/*begin modify by tangshan.bai for LEVIN-4508 on 20220719--base on ODIN5G-7905*/
#if defined(CONFIG_TCT_FEATURE_SLEEP_CHARGE) || defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
	/*smart sleep chrging*/
	remain_data.enable_smart_slp_chg = of_property_read_bool(np, "enable_smart_slp_chg");
	fg_read_dts_val(np, "smart_slp_chg_max_charge_current",\
        &(remain_data.max_charge_current), 1);
	fg_read_dts_val(np, "smart_slp_chg_max_charge_current_change",\
        &(remain_data.max_charge_current_change), 1);
	fg_read_dts_val(np, "smart_slp_chg_max_charge_current_change_point",\
        &(remain_data.max_charge_current_change_point), 1);
	fg_read_dts_val(np, "smart_slp_chg_max_charge_current_change_point_RemainSecond",\
        &(remain_data.max_charge_current_change_point_RemainSecond), 1);

	if(remain_data.enable_smart_slp_chg){
		smart_slp_chg_table(np, "remain_zcv",&remain_data);
	}else{
		bm_err("%s, disable smart sleep charging\n",__func__);
    }

    peak_data.enable_smart_peak_chg = of_property_read_bool(np, "enable_smart_peak_chg");
    if(peak_data.enable_smart_peak_chg){
        smart_peak_chg_table(np, "peak_zcv",&peak_data);
    }else{
        bm_err("%s, disable smart peak charging\n",__func__);
    }
#endif
/*End modify by tangshan.bai for LEVIN-4508 on 20220719--base on ODIN5G-7905*/
}

#endif	/* end of CONFIG_OF */

/* Begin modified dapeng.qiao for task 11038299 on 2021-05-1 */
#ifdef TCT_BMS_SW_SUPPORT
struct alarm print_fg_log_alarm;

static enum alarmtimer_restart print_fg_log_callback(struct alarm *alarm, ktime_t now)
{
    struct timespec ts;
    struct rtc_time tm;

    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec, &tm);
    bm_err("print_log_for_fg:%d-%02d-%02d %02d:%02d:%02d.%09lu \
status:%d,type:%d,vbat:%d,capacity:%d,current:%d,temperature:%d,Vbus:%d\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec,
            g_bat_status, g_chr_type, g_bat_vol,
            g_bat_capacity, g_fg_current, g_tbat_precise, g_chr_vol);

    alarm_forward_now(&print_fg_log_alarm, ns_to_ktime(10LL * NSEC_PER_SEC));
    return ALARMTIMER_RESTART;
}

void print_fg_log_init(void)
{
    alarm_init(&print_fg_log_alarm, ALARM_BOOTTIME, print_fg_log_callback);
    alarm_start_relative(&print_fg_log_alarm, ns_to_ktime(10LL * NSEC_PER_SEC));
	bm_err("print_log_for_fg init");
}
#endif
/* End modified dapeng.qiao for task 11038299 on 2021-05-1 */

/* ============================================================ */
/* power supply battery */
/* ============================================================ */
void battery_update_psd(struct mtk_battery *gm)
{
	struct battery_data *bat_data = &gm->bs_data;

	gauge_get_property(GAUGE_PROP_BATTERY_VOLTAGE, &bat_data->bat_batt_vol);
	bat_data->bat_batt_temp = force_get_tbat(gm, true);
}

void battery_update(struct mtk_battery *gm)
{
	struct battery_data *bat_data = &gm->bs_data;
	struct power_supply *bat_psy = bat_data->psy;

/* Begin added by bitao.xiong for AOSP13TMO-4085 on 2022-07-21 */
#if defined(CONFIG_TCT_CHARGER)
	struct mtk_charger *info = get_mtk_charger();
#endif
/* End added by bitao.xiong for AOSP13TMO-4085 on 2022-07-21 */

	if (gm->is_probe_done == false || bat_psy == NULL) {
		bm_err("[%s]battery is not rdy:probe:%d\n",
			__func__, gm->is_probe_done);
		return;
	}

/* Begin mod by jin.wang for jira 2064 on 2021-11-23 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	gm->log_level = BMLOG_INFO_LEVEL;
#endif
/* End mod by jin.wang */

	battery_update_psd(gm);
	bat_data->bat_technology = POWER_SUPPLY_TECHNOLOGY_LION;

/* Begin modified by bitao.xiong for AOSP13TMO-4085 on 2022-07-21 */
#if !IS_ENABLED(CONFIG_TCT_CHARGER)
	bat_data->bat_health = POWER_SUPPLY_HEALTH_GOOD;
#else
	if (info != NULL) {
		battery_do_health_update(info);
	}
#endif
/* End modified by bitao.xiong for AOSP13TMO-4085 on 2022-07-21 */

	bat_data->bat_present =
		gauge_get_int_property(GAUGE_PROP_BATTERY_EXIST);

	if (battery_get_int_property(BAT_PROP_DISABLE))
		bat_data->bat_capacity = 50;

	if (gm->algo.active == true)
		bat_data->bat_capacity = gm->ui_soc;

/* Begin modified by hailong.chen for task 9777037 on 2020-10-09 */
#if IS_ENABLED(DISABLE_TEMPERATURE_DETECTION_AND_THERMAL_POLICY) \
	|| IS_ENABLED(TARGET_BUILD_CERTIFICATION)
	if ((bat_data->bat_capacity <= 3) && (bat_data->bat_batt_vol >= 3250)) {
		bm_err("[%s] batt_vol > 3250mV, hold soc to 3%\n", __func__);
		bat_data->bat_capacity = 3;
		gm->ui_soc = 3;
	}
#endif
/* End modified by hailong.chen for task 9777037 on 2020-10-09 */

	power_supply_changed(bat_psy);

}

/* ============================================================ */
/* interrupt handler */
/* ============================================================ */
void disable_fg(struct mtk_battery *gm)
{
	gm->disableGM30 = true;
	gm->ui_soc = 50;
	gm->bs_data.bat_capacity = 50;
	disable_gauge_irq(gm->gauge, COULOMB_H_IRQ);
	disable_gauge_irq(gm->gauge, COULOMB_L_IRQ);
	disable_gauge_irq(gm->gauge, VBAT_H_IRQ);
	disable_gauge_irq(gm->gauge, VBAT_L_IRQ);
	disable_gauge_irq(gm->gauge, NAFG_IRQ);
	disable_gauge_irq(gm->gauge, BAT_PLUGOUT_IRQ);
	disable_gauge_irq(gm->gauge, ZCV_IRQ);
	disable_gauge_irq(gm->gauge, FG_N_CHARGE_L_IRQ);
	disable_gauge_irq(gm->gauge, FG_IAVG_H_IRQ);
	disable_gauge_irq(gm->gauge, FG_IAVG_L_IRQ);
	disable_gauge_irq(gm->gauge, BAT_TMP_H_IRQ);
	disable_gauge_irq(gm->gauge, BAT_TMP_L_IRQ);
}

bool fg_interrupt_check(struct mtk_battery *gm)
{
	if (gm->disableGM30) {
		disable_fg(gm);
		return false;
	}

	return true;
}

int fg_coulomb_int_h_handler(struct gauge_consumer *consumer)
{
	struct mtk_battery *gm;
	int fg_coulomb = 0;

	gm = get_mtk_battery();
	fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);

	gm->coulomb_int_ht = fg_coulomb + gm->coulomb_int_gap;
	gm->coulomb_int_lt = fg_coulomb - gm->coulomb_int_gap;

	gauge_coulomb_start(&gm->coulomb_plus, gm->coulomb_int_gap);
	gauge_coulomb_start(&gm->coulomb_minus, -gm->coulomb_int_gap);

	bm_err("[%s] car:%d ht:%d lt:%d gap:%d\n",
		__func__,
		fg_coulomb, gm->coulomb_int_ht,
		gm->coulomb_int_lt, gm->coulomb_int_gap);

	wakeup_fg_algo(gm, FG_INTR_BAT_INT1_HT);

	return 0;
}

int fg_coulomb_int_l_handler(struct gauge_consumer *consumer)
{
	struct mtk_battery *gm;
	int fg_coulomb = 0;

	gm = get_mtk_battery();
	fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);

	fg_sw_bat_cycle_accu(gm);

	gm->coulomb_int_ht = fg_coulomb + gm->coulomb_int_gap;
	gm->coulomb_int_lt = fg_coulomb - gm->coulomb_int_gap;

	gauge_coulomb_start(&gm->coulomb_plus, gm->coulomb_int_gap);
	gauge_coulomb_start(&gm->coulomb_minus, -gm->coulomb_int_gap);

	bm_err("[%s] car:%d ht:%d lt:%d gap:%d\n",
		__func__,
		fg_coulomb, gm->coulomb_int_ht,
		gm->coulomb_int_lt, gm->coulomb_int_gap);
	wakeup_fg_algo(gm, FG_INTR_BAT_INT1_LT);

	return 0;
}

int fg_bat_int2_h_handler(struct gauge_consumer *consumer)
{
	struct mtk_battery *gm;
	int fg_coulomb = 0;

	gm = get_mtk_battery();
	fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
	bm_debug("[%s] car:%d ht:%d\n",
		__func__,
		fg_coulomb, gm->uisoc_int_ht_en);
	fg_sw_bat_cycle_accu(gm);
	wakeup_fg_algo(gm, FG_INTR_BAT_INT2_HT);
	return 0;
}

int fg_bat_int2_l_handler(struct gauge_consumer *consumer)
{
	struct mtk_battery *gm;
	int fg_coulomb = 0;

	gm = get_mtk_battery();
	fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
	bm_debug("[%s] car:%d ht:%d\n",
		__func__,
		fg_coulomb, gm->uisoc_int_lt_gap);
	fg_sw_bat_cycle_accu(gm);
	wakeup_fg_algo(gm, FG_INTR_BAT_INT2_LT);
	return 0;
}

/* ============================================================ */
/* sysfs */
/* ============================================================ */
static int temperature_get(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int *val)
{
	gm->bs_data.bat_batt_temp = force_get_tbat(gm, true);
	*val = gm->bs_data.bat_batt_temp;
	bm_debug("%s %d\n", __func__, *val);
	return 0;
}

static int temperature_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->fixed_bat_tmp = val;
	bm_debug("%s %d\n", __func__, val);
	return 0;
}

static int log_level_get(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int *val)
{
	*val = gm->log_level;
	return 0;
}

static int log_level_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->log_level = val;
	return 0;
}

static int coulomb_int_gap_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	int fg_coulomb = 0;

	gauge_get_property(GAUGE_PROP_COULOMB, &fg_coulomb);
	gm->coulomb_int_gap = val;

	gm->coulomb_int_ht = fg_coulomb + gm->coulomb_int_gap;
	gm->coulomb_int_lt = fg_coulomb - gm->coulomb_int_gap;
	gauge_coulomb_start(&gm->coulomb_plus, gm->coulomb_int_gap);
	gauge_coulomb_start(&gm->coulomb_minus, -gm->coulomb_int_gap);

	bm_debug("[%s]BAT_PROP_COULOMB_INT_GAP = %d car:%d\n",
		__func__,
		gm->coulomb_int_gap, fg_coulomb);
	return 0;
}

static int uisoc_ht_int_gap_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->uisoc_int_ht_gap = val;
	gauge_coulomb_start(&gm->uisoc_plus, gm->uisoc_int_ht_gap);
	bm_debug("[%s]BATTERY_UISOC_INT_HT_GAP = %d\n",
		__func__,
		gm->uisoc_int_ht_gap);
	return 0;
}

static int uisoc_lt_int_gap_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->uisoc_int_lt_gap = val;
	gauge_coulomb_start(&gm->uisoc_minus, -gm->uisoc_int_lt_gap);
	bm_debug("[%s]BATTERY_UISOC_INT_LT_GAP = %d\n",
		__func__,
		gm->uisoc_int_lt_gap);
	return 0;
}

static int en_uisoc_ht_int_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->uisoc_int_ht_en = val;
	if (gm->uisoc_int_ht_en == 0)
		gauge_coulomb_stop(&gm->uisoc_plus);
	bm_debug("[%s][fg_bat_int2] FG_DAEMON_CMD_ENABLE_FG_BAT_INT2_HT = %d\n",
		__func__,
		gm->uisoc_int_ht_en);

	return 0;
}

static int en_uisoc_lt_int_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->uisoc_int_lt_en = val;
	if (gm->uisoc_int_lt_en == 0)
		gauge_coulomb_stop(&gm->uisoc_minus);
	bm_debug("[%s][fg_bat_int2] FG_DAEMON_CMD_ENABLE_FG_BAT_INT2_HT = %d\n",
		__func__,
		gm->uisoc_int_lt_en);

	return 0;
}
//Begin add by tangshan.bai for LEVIN-4508 on 2022-03-03
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
static int uisoc_get(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int *val)
{
	*val = gm->ui_soc * 100;
	return 0;
}
#endif
//end add by tangshan.bai for LEVIN-4508 on 2022-03-03

static int uisoc_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	int daemon_ui_soc;
	int old_uisoc;
	struct timespec now_time, diff;
	struct mtk_battery_algo *algo;
	struct fuel_gauge_table_custom_data *ptable;
	struct fuel_gauge_custom_data *pdata;

	algo = &gm->algo;
	ptable = &gm->fg_table_cust_data;
	pdata = &gm->fg_cust_data;
	daemon_ui_soc = val;

	if (daemon_ui_soc < 0) {
		bm_debug("[%s] error,daemon_ui_soc:%d\n",
			__func__,
			daemon_ui_soc);
		daemon_ui_soc = 0;
	}

	pdata->ui_old_soc = daemon_ui_soc;
	old_uisoc = gm->ui_soc;

	if (gm->disableGM30 == true)
		gm->ui_soc = 50;
	else
		gm->ui_soc = (daemon_ui_soc + 50) / 100;

	/* when UISOC changes, check the diff time for smooth */
	if (old_uisoc != gm->ui_soc) {
		get_monotonic_boottime(&now_time);
		diff = timespec_sub(now_time, gm->uisoc_oldtime);

		bm_debug("[%s] FG_DAEMON_CMD_SET_KERNEL_UISOC = %d %d GM3:%d old:%d diff=%ld\n",
			__func__,
			daemon_ui_soc, gm->ui_soc,
			gm->disableGM30, old_uisoc, diff.tv_sec);
		gm->uisoc_oldtime = now_time;

		gm->bs_data.bat_capacity = gm->ui_soc;
		battery_update(gm);
	} else {
		bm_debug("[%s] FG_DAEMON_CMD_SET_KERNEL_UISOC = %d %d GM3:%d\n",
			__func__,
			daemon_ui_soc, gm->ui_soc, gm->disableGM30);
		/* ac_update(&ac_main); */
		gm->bs_data.bat_capacity = gm->ui_soc;
		battery_update(gm);
	}
	return 0;
}

static int disable_get(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int *val)
{
	*val = gm->disableGM30;
	return 0;
}

static int disable_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->disableGM30 = val;
	if (gm->disableGM30 == true)
		battery_update(gm);
	return 0;
}

static int init_done_get(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int *val)
{
	*val = gm->init_flag;
	return 0;
}

static int init_done_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->init_flag = val;

	bm_debug("[%s] init_flag = %d\n",
		__func__,
		gm->init_flag);

	return 0;
}

static int reset_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	int car;

	if (gm->disableGM30)
		return 0;

	/* must handle sw_ncar before reset car */
	fg_sw_bat_cycle_accu(gm);
	gm->bat_cycle_car = 0;
	car = gauge_get_int_property(GAUGE_PROP_COULOMB);
	gm->log.car_diff += car;

	bm_err("%s car:%d\n",
		__func__, car);

	gauge_coulomb_before_reset(gm);
	gauge_set_property(GAUGE_PROP_RESET, 0);
	gauge_coulomb_after_reset(gm);
	get_monotonic_boottime(&gm->sw_iavg_time);
	gm->sw_iavg_car = gauge_get_int_property(GAUGE_PROP_COULOMB);
	gm->bat_cycle_car = 0;

	return 0;
}

static ssize_t bat_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy;
	struct mtk_battery *gm;
	struct mtk_battery_sysfs_field_info *battery_attr;
	int val;
	ssize_t ret;

	ret = kstrtos32(buf, 0, &val);
	if (ret < 0)
		return ret;

	psy = dev_get_drvdata(dev);
	gm = (struct mtk_battery *)power_supply_get_drvdata(psy);

	battery_attr = container_of(attr,
		struct mtk_battery_sysfs_field_info, attr);
	if (battery_attr->set != NULL)
		battery_attr->set(gm, battery_attr, val);

	return count;
}

static ssize_t bat_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct power_supply *psy;
	struct mtk_battery *gm;
	struct mtk_battery_sysfs_field_info *battery_attr;
	int val = 0;
	ssize_t count;

	psy = dev_get_drvdata(dev);
	gm = (struct mtk_battery *)power_supply_get_drvdata(psy);

	battery_attr = container_of(attr,
		struct mtk_battery_sysfs_field_info, attr);
	if (battery_attr->get != NULL)
		battery_attr->get(gm, battery_attr, &val);

	count = scnprintf(buf, PAGE_SIZE, "%d\n", val);
	return count;
}

/* Must be in the same order as BAT_PROP_* */
static struct mtk_battery_sysfs_field_info battery_sysfs_field_tbl[] = {
	BAT_SYSFS_FIELD_RW(temperature, BAT_PROP_TEMPERATURE),
	BAT_SYSFS_FIELD_WO(coulomb_int_gap, BAT_PROP_COULOMB_INT_GAP),
	BAT_SYSFS_FIELD_WO(uisoc_ht_int_gap, BAT_PROP_UISOC_HT_INT_GAP),
	BAT_SYSFS_FIELD_WO(uisoc_lt_int_gap, BAT_PROP_UISOC_LT_INT_GAP),
	BAT_SYSFS_FIELD_WO(en_uisoc_ht_int, BAT_PROP_ENABLE_UISOC_HT_INT),
	BAT_SYSFS_FIELD_WO(en_uisoc_lt_int, BAT_PROP_ENABLE_UISOC_LT_INT),
//Begin add by tangshan.bai for LEVIN-4508 on 2022-03-03
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
	BAT_SYSFS_FIELD_RW(uisoc, BAT_PROP_UISOC),
#else
	BAT_SYSFS_FIELD_WO(uisoc, BAT_PROP_UISOC),
#endif
//End add by tangshan.bai for LEVIN-4508 on 2022-03-03
	BAT_SYSFS_FIELD_RW(disable, BAT_PROP_DISABLE),
	BAT_SYSFS_FIELD_RW(init_done, BAT_PROP_INIT_DONE),
	BAT_SYSFS_FIELD_WO(reset, BAT_PROP_FG_RESET),
	BAT_SYSFS_FIELD_RW(log_level, BAT_PROP_LOG_LEVEL),
};

int battery_get_property(enum battery_property bp,
			    int *val)
{
	struct mtk_battery *gm;
	struct power_supply *psy;

	psy = power_supply_get_by_name("battery");
	if (psy == NULL)
		return -ENODEV;

	gm = (struct mtk_battery *)power_supply_get_drvdata(psy);
	if (battery_sysfs_field_tbl[bp].prop == bp)
		battery_sysfs_field_tbl[bp].get(gm,
			&battery_sysfs_field_tbl[bp], val);
	else {
		bm_err("%s bp:%d idx error\n", __func__, bp);
		return -ENOTSUPP;
	}

	return 0;
}

int battery_get_int_property(enum battery_property bp)
{
	int val;

	battery_get_property(bp, &val);
	return val;
}

int battery_set_property(enum battery_property bp,
			    int val)
{
	struct mtk_battery *gm;
	struct power_supply *psy;

	psy = power_supply_get_by_name("battery");
	if (psy == NULL)
		return -ENODEV;

	gm = (struct mtk_battery *)power_supply_get_drvdata(psy);

	if (battery_sysfs_field_tbl[bp].prop == bp)
		battery_sysfs_field_tbl[bp].set(gm,
			&battery_sysfs_field_tbl[bp], val);
	else {
		bm_err("%s bp:%d idx error\n", __func__, bp);
		return -ENOTSUPP;
	}
	return 0;
}

static struct attribute *
	battery_sysfs_attrs[ARRAY_SIZE(battery_sysfs_field_tbl) + 1];

static const struct attribute_group battery_sysfs_attr_group = {
	.attrs = battery_sysfs_attrs,
};

static void battery_sysfs_init_attrs(void)
{
	int i, limit = ARRAY_SIZE(battery_sysfs_field_tbl);

	for (i = 0; i < limit; i++)
		battery_sysfs_attrs[i] = &battery_sysfs_field_tbl[i].attr.attr;

	battery_sysfs_attrs[limit] = NULL; /* Has additional entry for this */
}

static int battery_sysfs_create_group(struct power_supply *psy)
{
	battery_sysfs_init_attrs();

	return sysfs_create_group(&psy->dev.kobj,
			&battery_sysfs_attr_group);
}

/* ============================================================ */
/* nafg monitor */
/* ============================================================ */
void fg_nafg_monitor(struct mtk_battery *gm)
{
	int nafg_cnt = 0;
	struct timespec now_time, dtime;

	if (gm->disableGM30 || gm->cmd_disable_nafg || gm->ntc_disable_nafg)
		return;

	now_time.tv_sec = 0;
	now_time.tv_nsec = 0;
	dtime.tv_sec = 0;
	dtime.tv_nsec = 0;

	nafg_cnt = gauge_get_int_property(GAUGE_PROP_NAFG_CNT);

	if (gm->last_nafg_cnt != nafg_cnt) {
		gm->last_nafg_cnt = nafg_cnt;
		get_monotonic_boottime(&gm->last_nafg_update_time);
	} else {
		get_monotonic_boottime(&now_time);
		dtime = timespec_sub(now_time, gm->last_nafg_update_time);
		if (dtime.tv_sec >= 600) {
			gm->is_nafg_broken = true;
			wakeup_fg_algo_cmd(
				gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_DISABLE_NAFG,
				true);
		}
	}
	bm_debug("[%s]time:%d nafg_cnt:%d, now:%d, last_t:%d\n",
		__func__,
		(int)dtime.tv_sec,
		gm->last_nafg_cnt,
		(int)now_time.tv_sec,
		(int)gm->last_nafg_update_time.tv_sec);

}

/* ============================================================ */
/* periodic timer */
/* ============================================================ */
void fg_drv_update_hw_status(struct mtk_battery *gm)
{
	ktime_t ktime;

/* Begin modified by dapeng.qiao for task 11038299 on 2021-05-1 */
#ifdef TCT_BMS_SW_SUPPORT
    g_bat_vol = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);
    g_fg_current = gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT);

	bm_err("car[%d,%ld,%ld,%ld,%ld] tmp:%d soc:%d uisoc:%d vbat:%d ibat:%d algo:%d gm3:%d %d %d %d,boot:%d\n",
		gauge_get_int_property(GAUGE_PROP_COULOMB),
		gm->coulomb_plus.end, gm->coulomb_minus.end,
		gm->uisoc_plus.end, gm->uisoc_minus.end,
		force_get_tbat_internal(gm, true),
		gm->soc, gm->ui_soc,
		g_bat_vol, g_fg_current,
		gm->algo.active,
		gm->disableGM30, gm->fg_cust_data.disable_nafg,
		gm->ntc_disable_nafg, gm->cmd_disable_nafg,
		gm->bootmode);
#else
	bm_err("car[%d,%ld,%ld,%ld,%ld] tmp:%d soc:%d uisoc:%d vbat:%d ibat:%d algo:%d gm3:%d %d %d %d,boot:%d\n",
		gauge_get_int_property(GAUGE_PROP_COULOMB),
		gm->coulomb_plus.end, gm->coulomb_minus.end,
		gm->uisoc_plus.end, gm->uisoc_minus.end,
		force_get_tbat_internal(gm, true),
		gm->soc, gm->ui_soc,
		gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE),
		gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT),
		gm->algo.active,
		gm->disableGM30, gm->fg_cust_data.disable_nafg,
		gm->ntc_disable_nafg, gm->cmd_disable_nafg,
		gm->bootmode);
#endif
/* End modified by dapeng.qiao for task 11038299 on 2021-05-1 */

	fg_drv_update_daemon(gm);

	/* kernel mode need regular update info */
	if (gm->algo.active == true)
		battery_update(gm);

	if (bat_get_debug_level() >= BMLOG_DEBUG_LEVEL)
		ktime = ktime_set(10, 0);
	else
		ktime = ktime_set(60, 0);

	hrtimer_start(&gm->fg_hrtimer, ktime, HRTIMER_MODE_REL);
}

int battery_update_routine(void *arg)
{
	struct mtk_battery *gm = (struct mtk_battery *)arg;

	battery_update_psd(gm);
	while (1) {
		bm_err("%s\n", __func__);
		wait_event(gm->wait_que, (gm->fg_update_flag > 0));
		gm->fg_update_flag = 0;
//Begin add by tangshan.bai for LEVIN-4508 on 2022-03-03
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
        if ((gm->soc < gm->BAT_peak_level) && (gm->ui_soc < gm->BAT_peak_level))
        	gm->peak_enforce_full = false;
        else
        	gm->peak_enforce_full = true;
#endif
//End add by tangshan.bai for LEVIN-4508 on 2022-03-03
		fg_drv_update_hw_status(gm);
	}
}

void fg_update_routine_wakeup(struct mtk_battery *gm)
{
	bm_err("%s\n", __func__);
	gm->fg_update_flag = 1;
	wake_up(&gm->wait_que);
}

enum hrtimer_restart fg_drv_thread_hrtimer_func(struct hrtimer *timer)
{
	struct mtk_battery *gm;

	bm_err("%s\n", __func__);
	gm = container_of(timer,
		struct mtk_battery, fg_hrtimer);
	fg_update_routine_wakeup(gm);
	return HRTIMER_NORESTART;
}

void fg_drv_thread_hrtimer_init(struct mtk_battery *gm)
{
	ktime_t ktime;

	ktime = ktime_set(10, 0);
	hrtimer_init(&gm->fg_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	gm->fg_hrtimer.function = fg_drv_thread_hrtimer_func;
	hrtimer_start(&gm->fg_hrtimer, ktime, HRTIMER_MODE_REL);
}

/* ============================================================ */
/* alarm timer handler */
/* ============================================================ */
static void tracking_timer_work_handler(struct work_struct *data)
{
	struct mtk_battery *gm;

	gm = container_of(data,
		struct mtk_battery, tracking_timer_work);
	bm_debug("[%s]\n", __func__);
	wakeup_fg_algo(gm, FG_INTR_FG_TIME);
}

static enum alarmtimer_restart tracking_timer_callback(
	struct alarm *alarm, ktime_t now)
{
	struct mtk_battery *gm;

	gm = container_of(alarm,
		struct mtk_battery, tracking_timer);
	bm_debug("[%s]\n", __func__);
	schedule_work(&gm->tracking_timer_work);
	return ALARMTIMER_NORESTART;
}

static void one_percent_timer_work_handler(struct work_struct *data)
{
	struct mtk_battery *gm;

	gm = container_of(data,
		struct mtk_battery, one_percent_timer_work);
	bm_debug("[%s]\n", __func__);
	wakeup_fg_algo_cmd(gm, FG_INTR_FG_TIME, 0, 1);
}

static enum alarmtimer_restart one_percent_timer_callback(
	struct alarm *alarm, ktime_t now)
{
	struct mtk_battery *gm;

	gm = container_of(alarm,
		struct mtk_battery, one_percent_timer);
	bm_debug("[%s]\n", __func__);
	schedule_work(&gm->one_percent_timer_work);
	return ALARMTIMER_NORESTART;
}

static void sw_uisoc_timer_work_handler(struct work_struct *data)
{
	struct mtk_battery *gm;

	gm = container_of(data,
		struct mtk_battery, one_percent_timer_work);
	bm_debug("[%s] %d %d\n", __func__,
		gm->soc, gm->ui_soc);
	if (gm->soc > gm->ui_soc)
		wakeup_fg_algo(gm, FG_INTR_BAT_INT2_HT);
	else if (gm->soc < gm->ui_soc)
		wakeup_fg_algo(gm, FG_INTR_BAT_INT2_LT);
}

static enum alarmtimer_restart sw_uisoc_timer_callback(
	struct alarm *alarm, ktime_t now)
{
	struct mtk_battery *gm;

	gm = container_of(alarm,
		struct mtk_battery, sw_uisoc_timer);
	bm_debug("[%s]\n", __func__);
	schedule_work(&gm->sw_uisoc_timer_work);
	return ALARMTIMER_NORESTART;
}

/* ============================================================ */
/* power misc */
/* ============================================================ */
static void wake_up_power_misc(struct shutdown_controller *sdd)
{
	sdd->timeout = true;
	wake_up(&sdd->wait_que);
}

static void wake_up_overheat(struct shutdown_controller *sdd)
{
	sdd->overheat = true;
	wake_up(&sdd->wait_que);
}

#if IS_ENABLED(CONFIG_TCT_CHARGER)
/* Begin added by bitao.xiong for task-10031392 on 2020-10-10 */
static void wake_up_overcold(struct shutdown_controller *sdd)
{
	sdd->overcold = true;
	wake_up(&sdd->wait_que);
}
/* End added by bitao.xiong for task-10031392 on 2020-10-10 */
#endif

void set_shutdown_vbat_lt(struct mtk_battery *gm, int vbat_lt, int vbat_lt_lv1)
{
	gm->sdc.vbat_lt = vbat_lt;
	gm->sdc.vbat_lt_lv1 = vbat_lt_lv1;
}

int get_shutdown_cond(struct mtk_battery *gm)
{
	int ret = 0;
	int vbat = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);
	struct shutdown_controller *sdc;

	sdc = &gm->sdc;
	if (sdc->shutdown_status.is_soc_zero_percent)
		ret |= 1;
	if (sdc->shutdown_status.is_uisoc_one_percent)
		ret |= 1;
	if (sdc->lowbatteryshutdown)
		ret |= 1;
	bm_debug("%s ret:%d %d %d %d vbat:%d\n",
		__func__,
	ret, sdc->shutdown_status.is_soc_zero_percent,
	sdc->shutdown_status.is_uisoc_one_percent,
	sdc->lowbatteryshutdown, vbat);

	return ret;
}

void set_shutdown_cond_flag(struct mtk_battery *gm, int val)
{
	gm->sdc.shutdown_cond_flag = val;
}

int get_shutdown_cond_flag(struct mtk_battery *gm)
{
	return gm->sdc.shutdown_cond_flag;
}

int disable_shutdown_cond(struct mtk_battery *gm, int shutdown_cond)
{
	int now_current;
	int now_is_charging = 0;
	int now_is_kpoc;
	struct shutdown_controller *sdc;

	sdc = &gm->sdc;
	now_current = gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT);
	now_is_kpoc = is_kernel_power_off_charging();

/* todo: can not get charger status now */
/*	if (mt_get_charger_type() != CHARGER_UNKNOWN)*/
/*		now_is_charging = 1;*/

	bm_debug("%s %d, is kpoc %d curr %d is_charging %d flag:%d lb:%d\n",
		__func__,
		shutdown_cond, now_is_kpoc, now_current, now_is_charging,
		sdc->shutdown_cond_flag,
		gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE));

	switch (shutdown_cond) {
#ifdef SHUTDOWN_CONDITION_LOW_BAT_VOLT
	case LOW_BAT_VOLT:
		sdc->shutdown_status.is_under_shutdown_voltage = false;
		sdc->lowbatteryshutdown = false;
		bm_debug("disable LOW_BAT_VOLT avgvbat %d ,threshold:%d %d %d\n",
		sdc->avgvbat,
		BAT_VOLTAGE_HIGH_BOUND,
		sdc->vbat_lt,
		sdc->vbat_lt_lv1);
		break;
#endif
	default:
		break;
	}
	return 0;
}

int set_shutdown_cond(struct mtk_battery *gm, int shutdown_cond)
{
	int now_current;
	int now_is_charging = 0;
	int now_is_kpoc;
	int vbat;
	struct shutdown_controller *sdc;
	struct shutdown_condition *sds;
	int enable_lbat_shutdown;

#ifdef SHUTDOWN_CONDITION_LOW_BAT_VOLT
	enable_lbat_shutdown = 1;
#else
	enable_lbat_shutdown = 0;
#endif

	now_current = gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT);
	now_is_kpoc = is_kernel_power_off_charging();
	vbat = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);
	sdc = &gm->sdc;
	sds = &gm->sdc.shutdown_status;

	if (now_current >= 0)
		now_is_charging = 1;

/* Begin mod by jin.wang task 2064 on 2021.11.25 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	bm_err("%s %d %d kpoc %d curr %d is_charging %d flag:%d lb:%d\n",
		__func__,
		shutdown_cond, enable_lbat_shutdown,
		now_is_kpoc, now_current, now_is_charging,
		sdc->shutdown_cond_flag, vbat);
#else
	bm_debug("%s %d %d kpoc %d curr %d is_charging %d flag:%d lb:%d\n",
		__func__,
		shutdown_cond, enable_lbat_shutdown,
		now_is_kpoc, now_current, now_is_charging,
		sdc->shutdown_cond_flag, vbat);
#endif
/* End mod by jin.wang */

	if (sdc->shutdown_cond_flag == 1)
		return 0;

	if (sdc->shutdown_cond_flag == 2 && shutdown_cond != LOW_BAT_VOLT)
		return 0;

	if (sdc->shutdown_cond_flag == 3 && shutdown_cond != DLPT_SHUTDOWN)
		return 0;

	switch (shutdown_cond) {
	case OVERHEAT:
		mutex_lock(&sdc->lock);
		sdc->shutdown_status.is_overheat = true;
		mutex_unlock(&sdc->lock);
/* Begin mod by jin.wang task 2064 on 2021.11.25 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
		bm_err("[%s]OVERHEAT shutdown!\n", __func__);
#else
		bm_debug("[%s]OVERHEAT shutdown!\n", __func__);
#endif
/* End mod by jin.wang */

		kernel_power_off();
		break;
	case SOC_ZERO_PERCENT:
		if (sdc->shutdown_status.is_soc_zero_percent != true) {
			mutex_lock(&sdc->lock);
			if (now_is_kpoc != 1) {
				if (now_is_charging != 1) {
					sds->is_soc_zero_percent =
						true;
					get_monotonic_boottime(
						&sdc->pre_time[
						SOC_ZERO_PERCENT]);
/* Begin mod by jin.wang task 2064 on 2021.11.25 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
					bm_err("[%s]soc_zero_percent shutdown\n",
						__func__);
#else
					bm_debug("[%s]soc_zero_percent shutdown\n",
						__func__);
#endif
/* End mod by jin.wang */
					wakeup_fg_algo(gm, FG_INTR_SHUTDOWN);
				}
			}
			mutex_unlock(&sdc->lock);
		}
		break;
	case UISOC_ONE_PERCENT:
		if (sdc->shutdown_status.is_uisoc_one_percent != true) {
			mutex_lock(&sdc->lock);
			if (now_is_kpoc != 1) {
				if (now_is_charging != 1) {
					sds->is_uisoc_one_percent =
						true;
					get_monotonic_boottime(
					&sdc->pre_time[UISOC_ONE_PERCENT]);
					bm_debug("[%s]uisoc 1 percent shutdown\n",
						__func__);
					wakeup_fg_algo(gm, FG_INTR_SHUTDOWN);
				}
			}
			mutex_unlock(&sdc->lock);
		}
		break;
#ifdef SHUTDOWN_CONDITION_LOW_BAT_VOLT
	case LOW_BAT_VOLT:
		if (sdc->shutdown_status.is_under_shutdown_voltage != true) {
			int i;

			mutex_lock(&sdc->lock);
			if (now_is_kpoc != 1) {
				sds->is_under_shutdown_voltage = true;
				for (i = 0; i < AVGVBAT_ARRAY_SIZE; i++)
					sdc->batdata[i] =
						VBAT2_DET_VOLTAGE1 / 10;
				sdc->batidx = 0;
			}
			bm_debug("LOW_BAT_VOLT:vbat %d %d",
				vbat, VBAT2_DET_VOLTAGE1 / 10);
			mutex_unlock(&sdc->lock);
		}
		break;
#endif
	case DLPT_SHUTDOWN:
		if (sdc->shutdown_status.is_dlpt_shutdown != true) {
			mutex_lock(&sdc->lock);
			sdc->shutdown_status.is_dlpt_shutdown = true;
			get_monotonic_boottime(&sdc->pre_time[DLPT_SHUTDOWN]);
			wakeup_fg_algo(gm, FG_INTR_DLPT_SD);
			mutex_unlock(&sdc->lock);
		}
		break;

	default:
		break;
	}

	wake_up_power_misc(sdc);

	return 0;
}

int next_waketime(int polling)
{
	if (polling <= 0)
		return 0;
	else
		return 10;
}

static int shutdown_event_handler(struct mtk_battery *gm)
{
	struct timespec now, duraction;
	int polling = 0;
	static int ui_zero_time_flag;
	static int down_to_low_bat;
	int now_current = 0;
	int current_ui_soc = gm->ui_soc;
	int current_soc = gm->soc;
	int vbat = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);
	int tmp = 25;
	struct shutdown_controller *sdd = &gm->sdc;

	now.tv_sec = 0;
	now.tv_nsec = 0;
	duraction.tv_sec = 0;
	duraction.tv_nsec = 0;

	get_monotonic_boottime(&now);

	bm_debug("%s:soc_zero:%d,ui 1percent:%d,dlpt_shut:%d,under_shutdown_volt:%d\n",
		__func__,
		sdd->shutdown_status.is_soc_zero_percent,
		sdd->shutdown_status.is_uisoc_one_percent,
		sdd->shutdown_status.is_dlpt_shutdown,
		sdd->shutdown_status.is_under_shutdown_voltage);

	if (sdd->shutdown_status.is_soc_zero_percent) {
		if (current_ui_soc == 0) {
			duraction = timespec_sub(
				now, sdd->pre_time[SOC_ZERO_PERCENT]);
			polling++;
			if (duraction.tv_sec >= SHUTDOWN_TIME) {
				bm_debug("soc zero shutdown\n");
				kernel_power_off();
				return next_waketime(polling);
			}
		} else if (current_soc > 0) {
			sdd->shutdown_status.is_soc_zero_percent = false;
		} else {
			/* ui_soc is not zero, check it after 10s */
			polling++;
		}
	}

	if (sdd->shutdown_status.is_uisoc_one_percent) {
		now_current = gauge_get_int_property(
			GAUGE_PROP_BATTERY_CURRENT);

		if (current_ui_soc == 0) {
			duraction =
				timespec_sub(
				now, sdd->pre_time[UISOC_ONE_PERCENT]);
			if (duraction.tv_sec >= SHUTDOWN_TIME) {
				bm_debug("uisoc one percent shutdown\n");
				kernel_power_off();
				return next_waketime(polling);
			}
		} else if (now_current > 0 && current_soc > 0) {
			polling = 0;
			sdd->shutdown_status.is_uisoc_one_percent = 0;
			bm_debug("disable uisoc_one_percent shutdown cur:%d soc:%d\n",
				now_current, current_soc);
			return next_waketime(polling);
		}
		/* ui_soc is not zero, check it after 10s */
		polling++;

	}

	if (sdd->shutdown_status.is_dlpt_shutdown) {
		duraction = timespec_sub(now, sdd->pre_time[DLPT_SHUTDOWN]);
		polling++;
		if (duraction.tv_sec >= SHUTDOWN_TIME) {
			bm_debug("dlpt shutdown count, %d\n",
				(int)duraction.tv_sec);
			return next_waketime(polling);
		}
	}

	if (sdd->shutdown_status.is_under_shutdown_voltage) {

		int vbatcnt = 0, i;

		sdd->batdata[sdd->batidx] = vbat;

		for (i = 0; i < AVGVBAT_ARRAY_SIZE; i++)
			vbatcnt += sdd->batdata[i];
		sdd->avgvbat = vbatcnt / AVGVBAT_ARRAY_SIZE;
		tmp = force_get_tbat(gm, true);

		bm_debug("lbatcheck vbat:%d avgvbat:%d %d,%d tmp:%d,bound:%d,th:%d %d,en:%d\n",
			vbat,
			sdd->avgvbat,
			sdd->vbat_lt,
			sdd->vbat_lt_lv1,
			tmp,
			BAT_VOLTAGE_LOW_BOUND,
			LOW_TEMP_THRESHOLD,
			LOW_TMP_BAT_VOLTAGE_LOW_BOUND,
			LOW_TEMP_DISABLE_LOW_BAT_SHUTDOWN);

		if (sdd->avgvbat < BAT_VOLTAGE_LOW_BOUND) {
			/* avg vbat less than 3.4v */
			sdd->lowbatteryshutdown = true;
			polling++;

			if (down_to_low_bat == 0) {
				if (IS_ENABLED(
					LOW_TEMP_DISABLE_LOW_BAT_SHUTDOWN)) {
					if (tmp >= LOW_TEMP_THRESHOLD) {
						down_to_low_bat = 1;
						bm_debug("normal tmp, battery voltage is low shutdown\n");
						wakeup_fg_algo(gm,
							FG_INTR_SHUTDOWN);
					} else if (sdd->avgvbat <=
						LOW_TMP_BAT_VOLTAGE_LOW_BOUND) {
						down_to_low_bat = 1;
						bm_debug("cold tmp, battery voltage is low shutdown\n");
						wakeup_fg_algo(gm,
							FG_INTR_SHUTDOWN);
					} else
						bm_debug("low temp disable low battery sd\n");
				} else {
					down_to_low_bat = 1;
					bm_debug("[%s]avg vbat is low to shutdown\n",
						__func__);
					wakeup_fg_algo(gm, FG_INTR_SHUTDOWN);
				}
			}

			if ((current_ui_soc == 0) && (ui_zero_time_flag == 0)) {
				get_monotonic_boottime(
					&sdd->pre_time[LOW_BAT_VOLT]);
				ui_zero_time_flag = 1;
			}

			if (current_ui_soc == 0) {
				duraction = timespec_sub(
					now, sdd->pre_time[LOW_BAT_VOLT]);
				if (duraction.tv_sec >= SHUTDOWN_TIME) {
					bm_debug("low bat shutdown, over %d second\n",
						SHUTDOWN_TIME);
					kernel_power_off();
					return next_waketime(polling);
				}
			}
		} else {
			/* greater than 3.4v, clear status */
			down_to_low_bat = 0;
			ui_zero_time_flag = 0;
			sdd->pre_time[LOW_BAT_VOLT].tv_sec = 0;
			sdd->lowbatteryshutdown = false;
			polling++;
		}

		polling++;
			bm_debug("[%s][UT] V %d ui_soc %d dur %d [%d:%d:%d:%d] batdata[%d] %d\n",
				__func__,
			sdd->avgvbat, current_ui_soc,
			(int)duraction.tv_sec,
			down_to_low_bat, ui_zero_time_flag,
			(int)sdd->pre_time[LOW_BAT_VOLT].tv_sec,
			sdd->lowbatteryshutdown,
			sdd->batidx, sdd->batdata[sdd->batidx]);

		sdd->batidx++;
		if (sdd->batidx >= AVGVBAT_ARRAY_SIZE)
			sdd->batidx = 0;
	}

	bm_debug(
		"%s %d avgvbat:%d sec:%d lowst:%d\n",
		__func__,
		polling, sdd->avgvbat,
		(int)duraction.tv_sec, sdd->lowbatteryshutdown);

	return next_waketime(polling);

}

static enum alarmtimer_restart power_misc_kthread_fgtimer_func(
	struct alarm *alarm, ktime_t now)
{
	struct shutdown_controller *info =
		container_of(
			alarm, struct shutdown_controller, kthread_fgtimer);

	wake_up_power_misc(info);
	return ALARMTIMER_NORESTART;
}

static void power_misc_handler(void *arg)
{
	struct mtk_battery *gm = arg;
	struct shutdown_controller *sdd = &gm->sdc;
	struct timespec time, time_now, end_time;
	ktime_t ktime;
	int secs = 0;

	secs = shutdown_event_handler(gm);
	if (secs != 0 && gm->disableGM30 == false) {
		get_monotonic_boottime(&time_now);
		time.tv_sec = secs;
		time.tv_nsec = 0;
		end_time = timespec_add(time_now, time);
		ktime = ktime_set(end_time.tv_sec, end_time.tv_nsec);

		alarm_start(&sdd->kthread_fgtimer, ktime);
		bm_debug("%s:set new alarm timer:%ds\n",
			__func__, secs);
	}
}

static int power_misc_routine_thread(void *arg)
{
	struct mtk_battery *gm = arg;
	struct shutdown_controller *sdd = &gm->sdc;

	while (1) {
		#if IS_ENABLED(CONFIG_TCT_CHARGER)
		wait_event(sdd->wait_que, (sdd->timeout == true)
			|| (sdd->overheat == true)
			|| (sdd->overcold == true));
		#else
		wait_event(sdd->wait_que, (sdd->timeout == true)
			|| (sdd->overheat == true));
		#endif
		if (sdd->timeout == true) {
			sdd->timeout = false;
			power_misc_handler(gm);
		}
		if (sdd->overheat == true) {
			sdd->overheat = false;
			bm_debug("%s battery overheat~ power off\n",
				__func__);
			kernel_power_off();
			return 1;
		}
		#if IS_ENABLED(CONFIG_TCT_CHARGER)
		/* Begin added by bitao.xiong for task-10031392 on 2020-10-10 */
		if (sdd->overcold == true) {
			sdd->overcold = false;
			bm_debug("%s battery overcold~ power off\n",
				__func__);
			kernel_power_off();
			return 1;
		}
		/* End added by bitao.xiong for task-10031392 on 2020-10-10 */
		#endif
	}

	return 0;
}

static int mtk_power_misc_psy_event(
	struct notifier_block *nb, unsigned long event, void *v)
{
	struct power_supply *psy = v;
	struct shutdown_controller *sdc;
	struct mtk_battery *gm;
	int tmp = 0;

	gm = get_mtk_battery();

	if (strcmp(psy->desc->name, "battery") == 0) {
		if (gm != NULL) {
			sdc = container_of(
				nb, struct shutdown_controller, psy_nb);
			if (gm->cur_bat_temp >= BATTERY_SHUTDOWN_TEMPERATURE) {
				bm_debug(
					"%d battery temperature >= %d,shutdown",
					gm->cur_bat_temp, tmp);

				wake_up_overheat(sdc);
			}
			#if IS_ENABLED(CONFIG_TCT_CHARGER)
			/* Begin added by bitao.xiong for task-10031392 on 2020-10-10 */
			if (gm->cur_bat_temp <= BATTERY_SHUTDOWN_TEMPERATURE_COLD) {
				bm_debug(
					"%d battery temperature >= %d,shutdown",
					gm->cur_bat_temp, tmp);

				wake_up_overcold(sdc);
			}
			/* End added by bitao.xiong for task-10031392 on 2020-10-10 */
			#endif
		}
	}

	return NOTIFY_DONE;
}

void mtk_power_misc_init(struct mtk_battery *gm)
{
	mutex_init(&gm->sdc.lock);
	alarm_init(&gm->sdc.kthread_fgtimer, ALARM_BOOTTIME,
		power_misc_kthread_fgtimer_func);
	init_waitqueue_head(&gm->sdc.wait_que);

	kthread_run(power_misc_routine_thread, gm, "power_misc_thread");

	gm->sdc.psy_nb.notifier_call = mtk_power_misc_psy_event;
	power_supply_reg_notifier(&gm->sdc.psy_nb);
}

int battery_psy_init(struct platform_device *pdev)
{
	struct mtk_battery *gm;
	struct mtk_gauge *gauge;
	int ret;

	bm_err("[%s]\n", __func__);
	gm = devm_kzalloc(&pdev->dev, sizeof(*gm), GFP_KERNEL);
	if (!gm)
		return -ENOMEM;

	gauge = dev_get_drvdata(&pdev->dev);
	gauge->gm = gm;
	gm->gauge = gauge;
	mutex_init(&gm->ops_lock);

	gm->bs_data.chg_psy = devm_power_supply_get_by_phandle(&pdev->dev,
							 "charger");
	if (IS_ERR_OR_NULL(gm->bs_data.chg_psy))
		bm_err("[BAT_probe] %s: fail to get chg_psy !!\n", __func__);

	battery_service_data_init(gm);
	gm->bs_data.psy =
		power_supply_register(
			&(pdev->dev), &gm->bs_data.psd, &gm->bs_data.psy_cfg);
	if (IS_ERR(gm->bs_data.psy)) {
		bm_err("[BAT_probe] power_supply_register Battery Fail !!\n");
		ret = PTR_ERR(gm->bs_data.psy);
		return ret;
	}
	bm_err("[BAT_probe] power_supply_register Battery Success !!\n");
	return 0;
}

void fg_check_bootmode(struct device *dev,
	struct mtk_battery *gm)
{
	struct device_node *boot_node = NULL;
	struct tag_bootmode *tag = NULL;

	boot_node = of_parse_phandle(dev->of_node, "bootmode", 0);
	if (!boot_node)
		bm_err("%s: failed to get boot mode phandle\n", __func__);
	else {
		tag = (struct tag_bootmode *)of_get_property(boot_node,
							"atag,boot", NULL);
		if (!tag)
			bm_err("%s: failed to get atag,boot\n", __func__);
		else {
			bm_err("%s: size:0x%x tag:0x%x bootmode:0x%x boottype:0x%x\n",
				__func__, tag->size, tag->tag,
				tag->bootmode, tag->boottype);
			gm->bootmode = tag->bootmode;
			gm->boottype = tag->boottype;
		}
	}
}

void fg_check_lk_swocv(struct device *dev,
	struct mtk_battery *gm)
{
	struct device_node *boot_node = NULL;
	int len = 0;
	char temp[12];
	int *prop;

	boot_node = of_parse_phandle(dev->of_node, "bootmode", 0);
	if (!boot_node)
		bm_err("%s: failed to get boot mode phandle\n", __func__);
	else {
		prop = (void *)of_get_property(
			boot_node, "atag,fg_swocv_v", &len);

		if (prop == NULL) {
			bm_err("fg_swocv_v prop == NULL, len=%d\n", len);
		} else {
			snprintf(temp, (len + 1), "%s", prop);
			kstrtoint(temp, 10, &gm->ptim_lk_v);
			bm_err("temp %s gm->ptim_lk_v=%d\n",
				temp, gm->ptim_lk_v);
		}

		prop = (void *)of_get_property(
			boot_node, "atag,fg_swocv_i", &len);

		if (prop == NULL) {
			bm_err("fg_swocv_i prop == NULL, len=%d\n", len);
		} else {
			snprintf(temp, (len + 1), "%s", prop);
			kstrtoint(temp, 10, &gm->ptim_lk_i);
			bm_err("temp %s gm->ptim_lk_i=%d\n",
				temp, gm->ptim_lk_i);
		}
		prop = (void *)of_get_property(
			boot_node, "atag,shutdown_time", &len);

		if (prop == NULL) {
			bm_err("shutdown_time prop == NULL, len=%d\n", len);
		} else {
			snprintf(temp, (len + 1), "%s", prop);
			kstrtoint(temp, 10, &gm->pl_shutdown_time);
			bm_err("temp %s gm->pl_shutdown_time=%d\n",
				temp, gm->pl_shutdown_time);
		}
	}

	bm_err("swocv_v:%d swocv_i:%d shutdown_time:%d\n",
		gm->ptim_lk_v, gm->ptim_lk_i, gm->pl_shutdown_time);
}

/* begin add by bing-zhang for getting ocv from preloader on 20210827 */
int fg_check_pl_boot_voltage(struct device *dev,
       struct mtk_battery *gm)
{
	struct device_node *boot_voltage_node = NULL;
	void *prop;
	char boot_voltage_tmp[10] = { 0 };
	int ret;
	unsigned long size = 0;

	boot_voltage_node = of_parse_phandle(dev->of_node, "bootvoltage", 0);
	if (!boot_voltage_node)
		bm_err("%s: failed to get boot voltage mode phandle\n", __func__);
	else {

		prop = (void *)of_get_property(boot_voltage_node, "atag,boot_voltage", (int *)&size);
		if (!prop)
			return -1;
		if (size >= sizeof(boot_voltage_tmp)) {
			pr_info("%s: error to get lcmname size=%ld\n", __func__, size);
			return -1;
		}
		memset((void *)boot_voltage_tmp, 0, sizeof(boot_voltage_tmp));
		strncpy((char *)boot_voltage_tmp, prop, sizeof(boot_voltage_tmp));
		boot_voltage_tmp[size] = '\0';
		ret = kstrtoint(boot_voltage_tmp, 10, &(gm->pl_bat_vol));
	}
	return 0;
}
/* end add by bing-zhang for getting ocv from preloader on 20210827 */

int battery_init(struct platform_device *pdev)
{
	int ret = 0;
	bool b_recovery_mode = 0;
	struct mtk_battery *gm;
	struct mtk_gauge *gauge;
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
	struct device_node *np = pdev->dev.of_node; //Added by tangshan.bai for LEVIN-6148
#endif

	gauge = dev_get_drvdata(&pdev->dev);
	gm = gauge->gm;
	gm->fixed_bat_tmp = 0xffff;
	gm->tmp_table = Fg_Temperature_Table;
	gm->log_level = BMLOG_ERROR_LEVEL;
	gm->sw_iavg_gap = 3000;

	init_waitqueue_head(&gm->wait_que);

	fg_check_bootmode(&pdev->dev, gm);
	fg_check_lk_swocv(&pdev->dev, gm);
	fg_custom_init_from_header(gm);
	fg_custom_init_from_dts(pdev, gm);

	gauge_coulomb_service_init(gm);
	gm->coulomb_plus.callback = fg_coulomb_int_h_handler;
	gauge_coulomb_consumer_init(&gm->coulomb_plus, &pdev->dev, "car+1%");
	gm->coulomb_minus.callback = fg_coulomb_int_l_handler;
	gauge_coulomb_consumer_init(&gm->coulomb_minus, &pdev->dev, "car-1%");

	gauge_coulomb_consumer_init(&gm->uisoc_plus, &pdev->dev, "uisoc+1%");
	gm->uisoc_plus.callback = fg_bat_int2_h_handler;
	gauge_coulomb_consumer_init(&gm->uisoc_minus, &pdev->dev, "uisoc-1%");
	gm->uisoc_minus.callback = fg_bat_int2_l_handler;



	alarm_init(&gm->tracking_timer, ALARM_BOOTTIME,
		tracking_timer_callback);
	INIT_WORK(&gm->tracking_timer_work, tracking_timer_work_handler);
	alarm_init(&gm->one_percent_timer, ALARM_BOOTTIME,
		one_percent_timer_callback);
	INIT_WORK(&gm->one_percent_timer_work, one_percent_timer_work_handler);

	alarm_init(&gm->sw_uisoc_timer, ALARM_BOOTTIME,
		sw_uisoc_timer_callback);
	INIT_WORK(&gm->sw_uisoc_timer_work, sw_uisoc_timer_work_handler);


	kthread_run(battery_update_routine, gm, "battery_thread");
	fg_drv_thread_hrtimer_init(gm);
	battery_sysfs_create_group(gm->bs_data.psy);
	gm->is_probe_done = true;

	/* for gauge hal hw ocv */
	gm->bs_data.bat_batt_temp = force_get_tbat(gm, true);
	mtk_power_misc_init(gm);

	ret = mtk_battery_daemon_init(pdev);
	b_recovery_mode = is_recovery_mode();

	/* begin add by bing-zhang for getting ocv from preloader on 20210827 */
	ret = fg_check_pl_boot_voltage(&pdev->dev, gm);
	battery_ocv_to_soc(gm);
	/* end add by bing-zhang for getting ocv from preloader on 20210827 */
	if (ret == 0 && b_recovery_mode == 0)
		bm_err("[%s]: daemon mode DONE\n", __func__);
	else {
		gm->algo.active = true;
		battery_algo_init(gm);
		bm_err("[%s]: kernel mode DONE\n", __func__);
	}

/* Begin added by dapeng.qiao for task 11038299 on 2021-05-1 */
#ifdef TCT_BMS_SW_SUPPORT
	if (bms_sw_support)
		print_fg_log_init();
#endif
/* End added by dapeng.qiao for task 11038299 on 2021-05-1 */

//Begin Added by tangshan.bai for LEVIN-6148
#if defined(CONFIG_TCT_FEATURE_PEAK_MANAGMENT)
	gm->BAT_peak_level = 100;
	//sprintf(gm->batt_resistance, "%d:%d", force_get_tbat_internal(gm, true), get_rac()); /*temp:R*/
	sprintf(gm->batt_resistance, "%d:%d", force_get_tbat_internal(gm, true), 110); /*temp:R*/
	if (of_property_read_string(np, "CHARGE_CYCLE_TABLE",
				    &gm->charge_cycle_table) < 0) {
		gm->charge_cycle_table = "00,0;00,0;00,0;00,0;00,0";
		pr_info("%s: cannot get charge_cycle_table\n", __func__);
	}
/* Begin added by dapeng.qiao for task SOCAOSP13-9123 on 2022-09-5 */
    memset(&life_array, 0, sizeof(life_array));
    life_array_avg = 0;
    life_cnt = 0;
    pr_err("%s: record PEAK_MANAGMENT g_begin_fg_coulomb init!\n", __func__);
    gm->BatteryVerify = 0x01;       //default not verify state and hide 0% life
    gm->g_begin_fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
    fg_max_monotone = gm->g_begin_fg_coulomb;
    gm->g_begin_v_soc = gm->fg_cust_data.v_soc;
    gm->g_begin_c_soc = gm->fg_cust_data.c_soc;
    gm->g_stop_fg_coulomb = 0x00;
    gm->g_stop_v_soc = 0x00;
    gm->g_stop_c_soc = 0x00;
    gm->g_begin_log_coulomb = bserach_begin_log_coulomb(BEGIN_V_SOC);
    gm->g_stop_log_coulomb = gm->g_begin_log_coulomb;
    if(gm->gauge->hw_status.is_bat_plugout){
        gm->BatteryVerify |= 0x0080;
    }else{
        gm->BatteryVerify &= ~0x0080;
    }
    gm->display_soc = -1;
    gm->peak_enforce_full = 0;
    gm->daemon_uisoc = -1;
    gm->bat_state = BAT_DISCHARGING;
	INIT_DELAYED_WORK(&gm->tracking_to_zero_work, uisoc_tracking_to_zero_work);

    pr_info("%s: record boot if battery plugout? gm->gauge->hw_status.is_bat_plugout:%d,gm->BatteryVerify:0x%x\n",\
        __func__,gm->gauge->hw_status.is_bat_plugout, gm->BatteryVerify);
/* End added by dapeng.qiao for task SOCAOSP13-9123 on 2022-09-5 */
#endif
//End Added by tangshan.bai for LEVIN-6148

	return 0;
}

