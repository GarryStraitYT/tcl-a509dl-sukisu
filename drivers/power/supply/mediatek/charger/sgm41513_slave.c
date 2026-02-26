// SPDX-License-Identifier: GPL-2.0


#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
#include <mt-plat/mtk_boot.h>
#include <mt-plat/upmu_common.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <mt-plat/v1/charger_class.h>
#include <linux/power_supply.h>
//#include <linux/regulator/driver.h>
//Begin add by bin.song.hz for task:9988113 on 2020.9.22
//#include <linux/regulator/machine.h>
//End add by bin.song.hz for task:9988113 on 2020.9.22

#include <linux/mutex.h>
#include <linux/errno.h>
/* Begin added by bitao.xiong for ENCORETF-5643 on 2022-07-22 */
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
/* End added by bitao.xiong for ENCORETF-5643 on 2022-07-22 */
#include "sgm41513.h"

#define SGM41513_DRV_VERSION	"V1.0_TCL"

/*sgm41513 REG06 VREG[5:0]*/
static const u32 vbat_arr[] = {
	3856000, 3888000, 3920000, 3952000,
	3984000, 4016000, 4048000, 4080000,
	4112000, 4144000, 4176000, 4208000,
	4240000, 4272000, 4304000, 4336000,
	4368000, 4400000, 4432000, 4464000,
	4496000, 4528000, 4560000, 4592000,
	4624000
};

/*sgm41513 REG02 ICHG[5:0], uA */
static const u32 ibatt_arr[] = {
	0, 5000, 10000, 15000, 20000, 25000, 30000, 35000, 40000,
	50000, 60000, 70000, 80000, 90000, 100000, 110000, 130000,
	150000, 170000, 190000, 210000, 230000, 250000, 270000,
	300000, 330000, 360000, 390000, 420000, 450000, 480000, 510000,
	540000, 600000, 660000, 720000, 780000, 840000, 900000, 960000,
	1020000, 1080000, 1140000, 1200000, 1260000, 1320000, 1380000, 1440000,
	1500000, 1620000, 1740000, 1860000, 1980000, 2100000, 2220000, 2340000,
	2460000, 2580000, 2700000, 2820000, 2940000, 3000000
};

/*sgm41513 REG00 IINLIM[5:0]*/
static const u32 ibus_arr[] = {
	100000, 200000, 300000, 400000,
	500000, 600000, 700000, 800000,
	900000, 1000000, 1100000, 1200000,
	1300000, 1400000, 1500000, 1600000,
	1700000, 1800000, 1900000, 2000000,
	2100000, 2200000, 2300000, 2400000,
	2500000, 2600000, 2700000, 2800000,
	2900000, 3000000, 3100000, 3200000
};

static const u32 __maybe_unused vindpm_arr[] = {
	3900, 4000, 4100, 4200, 4300, 4400,
	4500, 4600, 4700, 4800, 4900, 5000,
	5100, 5200, 5300, 5400,
};

/* sgm41513 REG02 BOOST_LIM[7], mA */
static const u32 __maybe_unused BOOST_CURRENT_LIMIT[] = {
	500, 1200
};

struct sgm41513_info {
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	int irq;

	//struct regulator_dev *otg_rdev;

	int intr_gpio;
	int en_gpio;

	u8 chip_rev;
	atomic_t is_chip_en;
	struct mutex i2c_access_lock;
	struct mutex adc_access_lock;
	struct mutex gpio_access_lock;
	struct mutex irq_access_lock;

	int chip_pn;
};

static DEFINE_MUTEX(g_input_current_mutex);
static struct i2c_client *new_client;
static const struct i2c_device_id sgm41513_i2c_id[] = { {"sgm41513", 0}, {} };

static int sgm41513_driver_probe(struct i2c_client *client,
				const struct i2c_device_id *id);

#if 0
static u32 charging_value_to_parameter(const u32 *parameter,
		const u32 array_size, const u32 val)
{
	if (val < array_size)
		return parameter[val];

	pr_info("Can't find the parameter\n");
	return parameter[0];
}
#endif

static u8 charging_parameter_to_value(const u32
		*parameter, const u32 array_size,
		const u32 val)
{
	u8 i;

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	pr_info("NO register value match\n");
	return 0;
}

static u32 bmt_find_closest_level(const u32 *pList,
		u32 number,	u32 level)
{
	u32 i;
	u32 max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = 1;
	else
		max_value_in_last_element = 0;

	if (max_value_in_last_element == 1) {
		for (i = (number - 1); i != 0; i--) {
			/* max value in the last element */
			if (pList[i] <= level) {
				pr_debug_ratelimited("%d<=%d, i=%d\n",
					pList[i], level, i);
				return pList[i];
			}
		}

		pr_info("Can't find closest level\n");
		return pList[0];
	} else {
		/* max value in the first element */
		for (i = 0; i < number; i++) {
			if (pList[i] <= level)
				return pList[i];
		}

		pr_info("Can't find closest level\n");
		return pList[number - 1];
	}
}


u8 sgm41513_reg[SGM41513_REG_NUM] = { 0 };

static DEFINE_MUTEX(sgm41513_i2c_access);
static DEFINE_MUTEX(sgm41513_access_lock);

static int g_sgm41513_hw_exist = 0; // change by TCT-cuiping.shi


static int sgm41513_read_byte(u8 cmd, u8 *returnData)
{
	u8 xfers = 2;
	int ret, retries = 1;

	mutex_lock(&sgm41513_i2c_access);

	do {
		struct i2c_msg msgs[2] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1,
				.buf = &cmd,
			},
			{

				.addr = new_client->addr,
				.flags = I2C_M_RD,
				.len = 1,
				.buf = returnData,
			}
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			pr_info("skipping non-existent adapter %s\n",
				new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&sgm41513_i2c_access);

	return ret == xfers ? 1 : -1;
}

static int sgm41513_write_byte(u8 cmd, u8 writeData)
{
	u8 xfers = 1;
	int ret, retries = 1;
	u8 buf[8];

	mutex_lock(&sgm41513_i2c_access);

	buf[0] = cmd;
	memcpy(&buf[1], &writeData, 1);

	do {
		struct i2c_msg msgs[1] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1 + 1,
				.buf = buf,
			},
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			pr_info("skipping non-existent adapter %s\n",
				new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&sgm41513_i2c_access);

	return ret == xfers ? 1 : -1;
}

static int sgm41513_read_interface(u8 RegNum,
				    u8 *val, u8 MASK, u8 SHIFT)
{
	u8 reg_val = 0;
	int ret = 0;

	ret = sgm41513_read_byte(RegNum, &reg_val);

	pr_debug_ratelimited("[%s] Reg[%x]=0x%x\n", __func__,
			     RegNum, reg_val);

	reg_val &= (MASK << SHIFT);
	*val = (reg_val >> SHIFT);

	pr_debug_ratelimited("[%s] val=0x%x\n", __func__, *val);

	return ret;
}

static int sgm41513_config_interface(u8 RegNum,
				      u8 val, u8 MASK, u8 SHIFT)
{
	u8 reg_val = 0;
	u8 reg_val_ori = 0;
	int ret = 0;

	mutex_lock(&sgm41513_access_lock);
	ret = sgm41513_read_byte(RegNum, &reg_val);

	reg_val_ori = reg_val;
	reg_val &= ~(MASK << SHIFT);
	reg_val |= (val << SHIFT);

	ret = sgm41513_write_byte(RegNum, reg_val);
	mutex_unlock(&sgm41513_access_lock);
	pr_debug_ratelimited("[%s] write Reg[%x]=0x%x from 0x%x\n", __func__,
			     RegNum,
			     reg_val, reg_val_ori);

	return ret;
}

static inline bool __sgm41513_is_chip_en(struct sgm41513_info *info);


/* CON0---------------------------------------------------- */
static inline void sgm41513_set_en_hiz(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON0),
				       val,
				       (u8)(CON0_EN_HIZ_MASK),
				       (u8)(CON0_EN_HIZ_SHIFT));
}

static inline void sgm41513_set_iinlim(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON0),
				       val,
				       (u8)(CON0_IINLIM_MASK),
				       (u8)(CON0_IINLIM_SHIFT));
}

#if 0
static inline void sgm41513_set_stat_ctrl(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON0),
				   val,
				   (u8)(CON0_STAT_IMON_CTRL_MASK),
				   (u8)(CON0_STAT_IMON_CTRL_SHIFT));
}

/* CON1---------------------------------------------------- */
static inline void sgm41513_set_reg_rst(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON11),
				       val,
				       (u8)(CON11_REG_RST_MASK),
				       (u8)(CON11_REG_RST_SHIFT));
}
#endif

static inline void sgm41513_set_pfm(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON1),
				       val,
				       (u8)(CON1_PFM_MASK),
				       (u8)(CON1_PFM_SHIFT));
}

static inline void sgm41513_set_wdt_rst(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON1),
				       val,
				       (u8)(CON1_WDT_RST_MASK),
				       (u8)(CON1_WDT_RST_SHIFT));
}

static inline void sgm41513_set_otg_config(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON1),
				       val,
				       (u8)(CON1_OTG_CONFIG_MASK),
				       (u8)(CON1_OTG_CONFIG_SHIFT));
}

static u32 __maybe_unused sgm41513_get_otg_config(void)
{
	u8 val = 0;

	sgm41513_read_interface((u8)(SGM41513_CON1),
				     (&val),
				     (u8)(CON1_OTG_CONFIG_MASK),
				     (u8)(CON1_OTG_CONFIG_SHIFT));
	return val;
}

static inline void sgm41513_set_chg_config(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON1),
				       val,
				       (u8)(CON1_CHG_CONFIG_MASK),
				       (u8)(CON1_CHG_CONFIG_SHIFT));
}

static inline void sgm41513_set_sys_min(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON1),
				       val,
				       (u8)(CON1_SYS_MIN_MASK),
				       (u8)(CON1_SYS_MIN_SHIFT));
}

static inline void sgm41513_set_batlowv(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON1),
				       val,
				       (u8)(CON1_MIN_VBAT_SEL_MASK),
				       (u8)(CON1_MIN_VBAT_SEL_SHIFT));
}

/* CON2---------------------------------------------------- */
static inline void sgm41513_set_rdson(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON2),
				       val,
				       (u8)(CON2_Q1_FULLON_MASK),
				       (u8)(CON2_Q1_FULLON_SHIFT));
}

#if 0
static inline void sgm41513_set_boost_lim(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON2),
				       val,
				       (u8)(CON2_BOOST_LIM_MASK),
				       (u8)(CON2_BOOST_LIM_SHIFT));
}
#endif

static inline void sgm41513_set_ichg(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON2),
				       val,
				       (u8)(CON2_ICHG_MASK),
				       (u8)(CON2_ICHG_SHIFT));
}

/* CON3---------------------------------------------------- */
static inline void sgm41513_set_iprechg(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON3),
				       val,
				       (u8)(CON3_IPRECHG_MASK),
				       (u8)(CON3_IPRECHG_SHIFT));
}

static inline void sgm41513_set_iterm(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON3),
				       val,
				       (u8)(CON3_ITERM_MASK),
				       (u8)(CON3_ITERM_SHIFT));
}

/* CON4---------------------------------------------------- */
static inline void sgm41513_set_vreg(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON4),
				       val,
				       (u8)(CON4_VREG_MASK),
				       (u8)(CON4_VREG_SHIFT));
}

#if 0
static inline void sgm41513_set_topoff_timer(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON4),
				       val,
				       (u8)(CON4_TOPOFF_TIMER_MASK),
				       (u8)(CON4_TOPOFF_TIMER_SHIFT));
}
#endif

static inline void sgm41513_set_vrechg(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON4),
				       val,
				       (u8)(CON4_VRECHG_MASK),
				       (u8)(CON4_VRECHG_SHIFT));
}

/* CON5---------------------------------------------------- */
static inline void sgm41513_set_en_term(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON5),
				       val,
				       (u8)(CON5_EN_TERM_MASK),
				       (u8)(CON5_EN_TERM_SHIFT));
}

static inline void sgm41513_set_watchdog(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON5),
				       val,
				       (u8)(CON5_WATCHDOG_MASK),
				       (u8)(CON5_WATCHDOG_SHIFT));
}

static inline void sgm41513_set_en_timer(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON5),
				       val,
				       (u8)(CON5_EN_TIMER_MASK),
				       (u8)(CON5_EN_TIMER_SHIFT));
}

#if 0
static inline void sgm41513_set_chg_timer(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON5),
				       val,
				       (u8)(CON5_CHG_TIMER_MASK),
				       (u8)(CON5_CHG_TIMER_SHIFT));
}

/* CON6---------------------------------------------------- */
static inline void sgm41513_set_treg(u8 val)
{
#ifdef FIXME
	sgm41513_config_interface((u8)(SGM41513_CON6),
				       val,
				       (u8)(CON6_BOOSTV_MASK),
				       (u8)(CON6_BOOSTV_SHIFT));
#endif
}
#endif

static inline void sgm41513_set_vindpm(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON6),
				       val,
				       (u8)(CON6_VINDPM_MASK),
				       (u8)(CON6_VINDPM_SHIFT));
}

static inline void sgm41513_set_ovp(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON6),
				       val,
				       (u8)(CON6_OVP_MASK),
				       (u8)(CON6_OVP_SHIFT));
}

#if 0
static inline void sgm41513_set_boostv(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON6),
				       val,
				       (u8)(CON6_BOOSTV_MASK),
				       (u8)(CON6_BOOSTV_SHIFT));
}
#endif

/* CON7---------------------------------------------------- */
static inline void sgm41513_set_tmr2x_en(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON7),
					val,
					(u8)(CON7_TMR2X_EN_MASK),
					(u8)(CON7_TMR2X_EN_SHIFT));
}

static inline void sgm41513_set_batfet_disable(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON7),
				val,
				(u8)(CON7_BATFET_Disable_MASK),
				(u8)(CON7_BATFET_Disable_SHIFT));
}

#if 0
static inline void sgm41513_set_batfet_delay(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON7),
				       val,
				       (u8)(CON7_BATFET_DLY_MASK),
				       (u8)(CON7_BATFET_DLY_SHIFT));
}

static inline void sgm41513_set_batfet_reset_enable(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON7),
				val,
				(u8)(CON7_BATFET_RST_EN_MASK),
				(u8)(CON7_BATFET_RST_EN_SHIFT));
}

/* CON8---------------------------------------------------- */
static u32 sgm41513_get_system_status(void)
{
	u8 val = 0;

	sgm41513_read_interface((u8)(SGM41513_CON8),
				     (&val), (u8)(0xFF),
				     (u8)(0x0));
	return val;
}

static u32 sgm41513_get_vbus_stat(void)
{
	u8 val = 0;

	sgm41513_read_interface((u8)(SGM41513_CON8),
				     (&val),
				     (u8)(CON8_VBUS_STAT_MASK),
				     (u8)(CON8_VBUS_STAT_SHIFT));
	return val;
}
#endif

static u32 sgm41513_get_chrg_stat(void)
{
	u8 val = 0;

	sgm41513_read_interface((u8)(SGM41513_CON8),
				     (&val),
				     (u8)(CON8_CHRG_STAT_MASK),
				     (u8)(CON8_CHRG_STAT_SHIFT));
	return val;
}

#if 0
static u32 sgm41513_get_vsys_stat(void)
{
	u8 val = 0;

	sgm41513_read_interface((u8)(SGM41513_CON8),
				     (&val),
				     (u8)(CON8_VSYS_STAT_MASK),
				     (u8)(CON8_VSYS_STAT_SHIFT));
	return val;
}

static u32 sgm41513_get_pg_stat(void)
{
	u8 val = 0;

	sgm41513_read_interface((u8)(SGM41513_CON8),
				     (&val),
				     (u8)(CON8_PG_STAT_MASK),
				     (u8)(CON8_PG_STAT_SHIFT) );
	return val;
}
#endif

/*CON10----------------------------------------------------------*/
static inline void sgm41513_set_int_mask(u8 val)
{
	sgm41513_config_interface((u8)(SGM41513_CON10),
				       val,
				       (u8)(CON10_INT_MASK_MASK),
				       (u8)(CON10_INT_MASK_SHIFT));
}

static int sgm41513_dump_register(struct charger_device *chg_dev)
{
	u8 i = 0;
	int ret = 0;

	pr_info("[sgm41513]\n");
	for (i = 0; i < SGM41513_REG_NUM; i++) {
		ret = sgm41513_read_byte(i, &sgm41513_reg[i]);
		if (ret < 0) {
			pr_info("[sgm41513] i2c transfor error:%d\n", ret);
			return 0;
		}
		pr_info("[0x%x]=0x%x\n", i, sgm41513_reg[i]);
	}
	return 0;
}


static void sgm41513_hw_component_detect(struct sgm41513_info *info)
{
	int ret = 0;
	u8 val = 0;

	ret = sgm41513_read_interface(0x0B, &val, 0xFF, 0x0);
	if (ret < 0) {
		pr_err("%s: sgm41513 i2c read err: %d\n", __func__, ret);
		return;
	}

	info->chip_pn = (val & 0x78) >> 3;

	/* 0: SGM41513, 1: SGM41513A or SGM41513D */
	if (info->chip_pn >= 0)
		g_sgm41513_hw_exist = 1;
	else
		g_sgm41513_hw_exist = 0;

	pr_err("[%s] %d, %d, 0x%x, %d\n", __func__,
			g_sgm41513_hw_exist, info->chip_pn, val, ret);
}

/* Begin added by weijun for termination on 2021-08-16 */
static int sgm41513_enable_term(struct charger_device *chg_dev,
				   bool en)
{
	pr_err("[%s] enable:%d\n", __func__, en);
	if (en) {
		sgm41513_set_en_term(0x1);
	} else {
		sgm41513_set_en_term(0x0);
	}

	return 0;
}
/* End added by weijun for termination on 2021-08-16 */

static int sgm41513_enable_charging(struct charger_device *chg_dev,
				   bool en)
{
	pr_err("[%s] enable state : %d\n", __func__, en);
	if (en) {
		/* enable charging */
		sgm41513_set_en_hiz(0);
		sgm41513_set_otg_config(0);
		sgm41513_set_chg_config(1);
		sgm41513_set_batfet_disable(0);	/* enable Q4 turn on */
	} else {
		/* disable charging */
		sgm41513_set_chg_config(0);
		sgm41513_set_batfet_disable(1);	/* Disable Q4 turn off default */
		sgm41513_set_otg_config(0);
		/*sgm41513_set_en_hiz(0x1);*/
	}

	return 0;
}

/* Begin added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
static bool __sgm41513_is_chip_en(struct sgm41513_info *info)
{
	int en = -1;
    bool is_enabled = 0;

	en = gpio_get_value(info->en_gpio);

    if(0 == en) {
        is_enabled = 1;
    } else {
        is_enabled = 0;
    }

	return is_enabled;
}

static int __sgm41513_enable_chip(struct sgm41513_info *info, bool en)
{
	bool is_chip_en = false;

	mutex_lock(&info->gpio_access_lock);
	is_chip_en = __sgm41513_is_chip_en(info);
	if (en && !is_chip_en) {
		gpio_set_value(info->en_gpio, 0);
		dev_info(info->dev, "%s: set gpio low\n", __func__);
	} else if (!en && is_chip_en) {
		gpio_set_value(info->en_gpio, 1);
		dev_info(info->dev, "%s: set gpio high\n", __func__);
	}

	/* Wait for chip's enable/disable */
	mdelay(1);
	atomic_set(&info->is_chip_en, en);
	mutex_unlock(&info->gpio_access_lock);
	return 0;
}

static int sgm41513_hw_init(void);

static int sgm41513_enable_chip(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct sgm41513_info *info = dev_get_drvdata(&chg_dev->dev);

	/* Do the following flow for enabling chip */
    pr_err("%s enable = %d\n", __func__, en);

	sgm41513_hw_init();

	ret = __sgm41513_enable_chip(info, en);
	if (ret < 0)
		return ret;

	sgm41513_set_otg_config(0);
	return ret;
}

static int sgm41513_is_chip_enabled(struct charger_device *chg_dev, bool *en)
{
	struct sgm41513_info *info = dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&info->gpio_access_lock);
	*en = __sgm41513_is_chip_en(info);
	mutex_unlock(&info->gpio_access_lock);

    pr_err("%s: %d\n", __func__, *en);
	return 0;
}

static int sgm41513_is_charging_enabled(struct charger_device *chg_dev, bool *en)
{
	u8 val = 0;

	sgm41513_read_interface(SGM41513_CON1, &val, CON1_CHG_CONFIG_MASK, CON1_CHG_CONFIG_SHIFT);
	*en = val;
	pr_err("%s: %d\n", __func__, *en);
	return 0;
}

static int sgm41513_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = ibatt_arr[0];
    pr_err("%s: %d\n", __func__, *uA);
	return 0;
}

static int sgm41513_get_current(struct charger_device *chg_dev, u32 *uA)
{
	u8 ret_val = 0;
	u8 arr_size = ARRAY_SIZE(ibatt_arr);

	if (!arr_size) {
		*uA = 0;
		return -EINVAL;
	}

	sgm41513_read_interface(SGM41513_CON2, &ret_val, CON2_ICHG_MASK,
					CON2_ICHG_SHIFT);
	if (ret_val < arr_size) {
		*uA = ibatt_arr[ret_val];
	} else {
		*uA = ibatt_arr[(arr_size-1)];
	}

	pr_info("%s: %d\n", __func__, *uA);
	return 0;
}

static int sgm41513_set_current(struct charger_device *chg_dev,
			       u32 uA)
{
	u32 set_chr_current;
	u32 array_size;
	u8 reg_val;

	pr_info("%s: ibatt=%d\n", __func__, uA);

	array_size = ARRAY_SIZE(ibatt_arr);
	set_chr_current = bmt_find_closest_level(ibatt_arr, array_size,
			  uA);
	reg_val = charging_parameter_to_value(ibatt_arr, array_size,
			 set_chr_current);
	pr_info("%s: reg_val=0x%x\n", __func__, reg_val);

	sgm41513_set_ichg(reg_val);
	return 0;
}

static int sgm41513_get_input_current(struct charger_device *chg_dev,
				     u32 *aicr)
{
	int ret = 0;

#ifdef FIXME
	u8 val = 0;

	sgm41513_read_interface(SGM41513_CON0, &val, CON0_IINLIM_MASK,
			       CON0_IINLIM_SHIFT);
	ret = (int)val;
	*aicr = ibus_arr[val];
#endif

	return ret;
}


static int sgm41513_set_input_current(struct charger_device *chg_dev,
				     u32 current_value)
{
	u32 set_chr_current;
	u32 array_size;
	u8 reg_val;

	pr_info("%s: ibus=%d\n", __func__, current_value);

	array_size = ARRAY_SIZE(ibus_arr);
	set_chr_current = bmt_find_closest_level(ibus_arr, array_size,
			  current_value);
	reg_val = charging_parameter_to_value(ibus_arr, array_size,
			 set_chr_current);
	pr_info("%s: reg_val=0x%x\n", __func__, reg_val);

	sgm41513_set_iinlim(reg_val);
	return 0;
}

static int sgm41513_set_cv_voltage(struct charger_device *chg_dev,
				  u32 cv)
{
	u32 array_size;
	u32 set_cv_voltage;
	u8 reg_val;

	array_size = ARRAY_SIZE(vbat_arr);
	set_cv_voltage = bmt_find_closest_level(vbat_arr, array_size, cv);
	reg_val = charging_parameter_to_value(vbat_arr, array_size,
			 set_cv_voltage);
	pr_info("%s: reg_val=0x%x\n", __func__, reg_val);

	sgm41513_set_vreg(reg_val);
	return 0;
}

static int sgm41513_reset_watch_dog_timer(struct charger_device
		*chg_dev)
{
	pr_info("charging_reset_watch_dog_timer\n");

	sgm41513_set_wdt_rst(0x1);	/* Kick watchdog */
	sgm41513_set_watchdog(0x3);	/* WDT 160s */
	return 0;
}

static int sgm41513_set_vindpm_voltage(struct charger_device *chg_dev,
				      u32 vindpm)
{
	return 0;
}

static int sgm41513_is_charging_done(struct charger_device *chg_dev,
				       bool *is_done)
{
	u32 ret_val;

	ret_val = sgm41513_get_chrg_stat();

	if (ret_val == 0x3)
		*is_done = true;
	else
		*is_done = false;

	return 0;
}

#if 0
static int sgm41513_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;

	pr_info("%s en = %d\n", __func__, en);
	if (en) {
		sgm41513_set_chg_config(0);
		sgm41513_set_otg_config(1);
		sgm41513_set_watchdog(0x3);	/* WDT 160s */
	} else {
		sgm41513_set_otg_config(0);
		sgm41513_set_chg_config(1);
	}
	return ret;
}

static int sgm41513_set_boost_current_limit(struct charger_device
		*chg_dev, u32 uA)
{
	int ret = 0;
	u32 array_size = 0;
	u32 boost_ilimit = 0;
	u8 boost_reg = 0;

	uA /= 1000;
	array_size = ARRAY_SIZE(BOOST_CURRENT_LIMIT);
	boost_ilimit = bmt_find_closest_level(BOOST_CURRENT_LIMIT, array_size,
					      uA);
	boost_reg = charging_parameter_to_value(BOOST_CURRENT_LIMIT,
						array_size, boost_ilimit);
	sgm41513_set_boost_lim(boost_reg);

	return ret;
}
#endif

static int sgm41513_enable_safetytimer(struct charger_device *chg_dev,
				      bool en)
{
	if (en)
		sgm41513_set_en_timer(0x1);
	else
		sgm41513_set_en_timer(0x0);

	return 0;
}

static int sgm41513_get_is_safetytimer_enable(struct charger_device
		*chg_dev, bool *en)
{
	u8 val = 0;

	sgm41513_read_interface(SGM41513_CON5, &val, CON5_EN_TIMER_MASK,
			       CON5_EN_TIMER_SHIFT);
	*en = (bool)val;
	return val;
}

/* Begin added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
static int sgm41513_hw_init(void)
{
	sgm41513_set_en_hiz(0x0);
	sgm41513_set_iinlim(0x0);	/* Iinlim 100mA */
	sgm41513_set_ichg(0x1e);	/* ICHG 480mA */
	sgm41513_set_vindpm(0x7);	/* SGM41511/3, BQ25601 6 VIN DPM check 4.6V */
	sgm41513_set_wdt_rst(0x1);	/* Kick watchdog */
	sgm41513_set_sys_min(0x5);	/* Minimum system voltage 3.5V */
	sgm41513_set_iprechg(0xf);	/* Precharge current 240mA */
	sgm41513_set_iterm(0xf);	/* Termination current 240mA */
	sgm41513_set_vreg(0x11);	/* VREG 4.4V */
    sgm41513_set_ovp(0x02);	/* ovp 10.5V for 9v input */
	sgm41513_set_pfm(0x0);		/* enable pfm */
	sgm41513_set_rdson(0x0);	/*close rdson*/
	sgm41513_set_batlowv(0x1);	/* BATLOWV 3.0V */
	sgm41513_set_vrechg(0x0);	/* VRECHG 0.1V (4.108V) */
	sgm41513_set_en_term(0x1);	/* Enable termination */
	sgm41513_set_watchdog(0x0);	/* WDT disable */
	sgm41513_set_tmr2x_en(0x0);	/* Disable timer X2 */
	sgm41513_set_en_timer(0x1);	/* Enable charge timer */
	sgm41513_set_int_mask(0x0);	/* Disable fault interrupt */
    sgm41513_set_chg_config(0);	/* Disable charge default */
    sgm41513_set_batfet_disable(1);	/* Disable Q4 turn off default */

	sgm41513_set_otg_config(0);
	pr_info("%s: hw_init down!\n", __func__);
	return 0;
}

/* Begin added by bitao.xiong for ENCORETF-5643 on 2022-07-22 */
static int board_id_get_voltage(struct device *dev)
{
	struct iio_channel *channel;
	int voltage = 0;
	int ret = 0;

	channel = iio_channel_get(dev, "board_id-channel");
	if (IS_ERR(channel)) {
		ret = PTR_ERR(channel);
		pr_err("[%s] iio channel not found %d\n",
		__func__, ret);
		return ret;
	}

	if (channel)
		ret = iio_read_channel_processed(channel, &voltage);

	if (ret <= 0) {
		pr_err("[%s] iio_read_channel_processed failed\n", __func__);
		return ret;
	}

	pr_err("%s:voltage=%d\n", __func__, voltage);
	return voltage;
}
/* End added by bitao.xiong for ENCORETF-5643 on 2022-07-22 */

static int sgm41513_parse_dt(struct sgm41513_info *info,
			    struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret = 0;
	int board_id_voltage = 0; /* Added by bitao.xiong for ENCORETF-5643 on 2022-07-22 */

	pr_info("%s\n", __func__);
	if (!np) {
		pr_err("%s: no of node\n", __func__);
		return -ENODEV;
	}

	if (of_property_read_string(np, "charger_name",
				    &info->chg_dev_name) < 0) {
		info->chg_dev_name = "secondary_chg";
		pr_err("%s: no charger name\n", __func__);
	} else {
        pr_err("%s: charger name=%s\n", __func__, info->chg_dev_name);
    }

	if (of_property_read_string(np, "alias_name",
				    &(info->chg_props.alias_name)) < 0) {
		info->chg_props.alias_name = "sgm41513";
		pr_err("%s: no alias name\n", __func__);
	} else {
        pr_err("%s: alias name=%s\n", __func__,info->chg_props.alias_name);
    }

	/* Begin modified by bitao.xiong for ENCORETF-5643 on 2022-07-22 */
	board_id_voltage = board_id_get_voltage(dev);
	if (board_id_voltage <= 0 || board_id_voltage >= 180) {
		info->en_gpio = of_get_named_gpio(np, "gpio_sgm41513_en", 0);
		if(info->en_gpio <= 0){
			pr_err("%s: no sgm41513_en_pin\n", __func__);
			return -ENODATA;
		} else {
			pr_err("%s: en_pin=%d\n", __func__, info->en_gpio);
		}
	} else {
		info->en_gpio = of_get_named_gpio(np, "gpio_sgm41513_en_v0", 0);
		if(info->en_gpio <= 0){
			pr_err("%s: no sgm41513_en_pin\n", __func__);
			return -ENODATA;
		} else {
			pr_err("%s: en_pin=%d\n", __func__, info->en_gpio);
		}
	}
	/* End modified by bitao.xiong for ENCORETF-5643 on 2022-07-22 */

    ret = devm_gpio_request(dev, info->en_gpio, "gpio_sgm41513_en_num");
	if (ret) {
		pr_err("%s: request sgm41513_en_pin err\n", __func__);
		return -EINVAL;
	}

    if (of_property_read_string(np, "eint_name", &info->eint_name) < 0) {
        info->eint_name = "chr_stat";
        pr_err("%s: no eint name\n", __func__);
    } else {
        pr_err("%s: eint_name=%s\n", __func__,info->eint_name);
    }

	return 0;
}
/* End added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */


/* Add-start by baiwei.peng for hiz enable on 2022/11/16 */
static int sgm41513_enable_hz(struct charger_device *chg_dev, bool en)
{
	pr_err("[%s] hiz en : %d\n", __func__, en);

        if (en) {
		sgm41513_set_en_hiz(1);
        } else {
		sgm41513_set_en_hiz(0);
        }

        return 0;
}
/* Add-end by baiwei.peng for hiz enable on 2022/11/16 */

static struct charger_ops sgm41513_chg_ops = {
//#ifdef FIXME
	.enable_hz = sgm41513_enable_hz,
//#endif

	/* Normal charging */
	.dump_registers = sgm41513_dump_register,
	.enable = sgm41513_enable_charging,
	.enable_chip = sgm41513_enable_chip,
	.is_enabled = sgm41513_is_charging_enabled,
	.is_chip_enabled = sgm41513_is_chip_enabled,
	.get_min_charging_current = sgm41513_get_min_ichg,
	.get_charging_current = sgm41513_get_current,
	.set_charging_current = sgm41513_set_current,
	.get_input_current = sgm41513_get_input_current,
	.set_input_current = sgm41513_set_input_current,
	/*.get_constant_voltage = sgm41513_get_battery_voreg,*/
	.set_constant_voltage = sgm41513_set_cv_voltage,
	.kick_wdt = sgm41513_reset_watch_dog_timer,
	.set_mivr = sgm41513_set_vindpm_voltage,
	.is_charging_done = sgm41513_is_charging_done,
	.enable_termination = sgm41513_enable_term, // weijun ++

	/* Safety timer */
	.enable_safety_timer = sgm41513_enable_safetytimer,
	.is_safety_timer_enabled = sgm41513_get_is_safetytimer_enable,


	/* Power path */
	/*.enable_powerpath = sgm41513_enable_power_path, */
	/*.is_powerpath_enabled = sgm41513_get_is_power_path_enable, */


	/* OTG */
	/*
	.enable_otg = sgm41513_enable_otg,
	.set_boost_current_limit = sgm41513_set_boost_current_limit,
	.event = sgm41513_do_event,
	*/
};

/* Begin added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
static int sgm41513_driver_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	struct sgm41513_info *info = NULL;

	pr_info("%s (%s)\n", __func__, SGM41513_DRV_VERSION);

	info = devm_kzalloc(&client->dev, sizeof(struct sgm41513_info),
		    GFP_KERNEL);
	if (!info){
		pr_err("%s: sgm41513 kzalloc fail\n", __func__);
		return -ENOMEM;
	}

	new_client = client;
	info->dev = &client->dev;
	info->chip_pn = -1;

	sgm41513_hw_component_detect(info);
	if (!g_sgm41513_hw_exist){
		pr_err("%s: g_sgm41513_hw_exist =0 return error!!!\n", __func__);
		return -ENODEV;
	}

	ret = sgm41513_parse_dt(info, &client->dev);
	if (ret < 0){
		pr_err("%s: sgm41513_parse_dt fail\n", __func__);
		return ret;
	}

	mutex_init(&info->i2c_access_lock);
	mutex_init(&info->gpio_access_lock);

	gpio_direction_output(info->en_gpio, 1);
	sgm41513_hw_init();

	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
						&client->dev, info,
						&sgm41513_chg_ops,
						&info->chg_props);
	if (IS_ERR_OR_NULL(info->chg_dev)) {
		pr_err("%s: register info->chg_dev  failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}

	/* otg regulator */
	//config.dev = info->dev;
	//config.driver_data = info;
	//Begin add by bin.song.hz for task:9988113 on 2020.9.22
	//config.init_data = &sgm41513_vbus_init_data;
	//End add by bin.song.hz for task:9988113 on 2020.9.22
	/*
	info->otg_rdev = devm_regulator_register(info->dev,
					&sgm41513_otg_rdesc, &config);
	if (IS_ERR(info->otg_rdev)) {
		ret = PTR_ERR(info->otg_rdev);
		pr_info("%s: register otg regulator failed (%d)\n", __func__, ret);
		return ret;
	} else {
		pr_info("%s: register otg regulator successed (%d)\n", __func__, ret);
	}
	*/

	sgm41513_dump_register(info->chg_dev);

	return 0;
}
/* End added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */

static u8 g_reg_value;
static ssize_t sgm41513_access_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	pr_info("[%s] 0x%x\n", __func__, g_reg_value);
	return sprintf(buf, "%u\n", g_reg_value);
}

static ssize_t sgm41513_access_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	u32 reg_value = 0;
	u32 reg_address = 0;

	pr_info("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		pr_info("[%s] buf is %s and size is %zu\n", __func__, buf,
			size);

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16,
				(u32 *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16,
				(u32 *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (u32 *)&reg_value);
			pr_info("[%s] write sgm41513 reg 0x%x with value 0x%x !\n",
					__func__, (u32)reg_address, reg_value);

			ret = sgm41513_config_interface(reg_address,
				reg_value, 0xFF, 0x0);
		} else {
			ret = sgm41513_read_interface(reg_address,
					     &g_reg_value, 0xFF, 0x0);
			pr_info("[%s] read sgm41513 reg 0x%x with value 0x%x !\n",
					__func__, (u32)reg_address, g_reg_value);
			pr_info("[%s] use \"cat sgm41513_access\" to get value\n",
					__func__);
		}
	}
	return size;
}

static DEVICE_ATTR_RW(sgm41513_access);

static int sgm41513_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	if (!g_sgm41513_hw_exist){
		pr_err("%s: g_sgm41513_hw_exist =0 return !!!\n", __func__);
		return -ENODEV;
	}

	pr_info("******** %s!! ********\n", __func__);

	ret_device_file = device_create_file(&(dev->dev),
					     &dev_attr_sgm41513_access);

	return 0;
}

static struct platform_device sgm41513_user_space_device = {
	.name = "sgm41513-user",
	.id = -1,
};

static struct platform_driver sgm41513_user_space_driver = {
	.probe = sgm41513_user_space_probe,
	.driver = {
		.name = "sgm41513-user",
	},
};

#ifdef CONFIG_OF
static const struct of_device_id sgm41513_of_match[] = {
	{.compatible = "mediatek,sgm41513"},
	{},
};
#endif

static struct i2c_driver sgm41513_driver = {
	.driver = {
		.name = "sgm41513",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sgm41513_of_match,
#endif
	},
	.probe = sgm41513_driver_probe,
	.id_table = sgm41513_i2c_id,
};

static int __init sgm41513_init(void)
{
	int ret = 0;

	/* i2c registration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
	pr_info("[%s] init start with i2c DTS", __func__);
#else
	pr_info("[%s] init start. ch=%d\n", __func__, SGM41513_BUSNUM);
	i2c_register_board_info(SGM41513_BUSNUM, &i2c_sgm41513, 1);
#endif

	if (i2c_add_driver(&sgm41513_driver) != 0) {
		pr_info("[%s] failed to register sgm41513 driver.\n",
				__func__);
		return -ENODEV;
	} else {
		pr_info("[%s] Success to register sgm41513 driver.\n",
				__func__);
	}

	/* sgm41513 user space access interface */
	ret = platform_device_register(&sgm41513_user_space_device);
	if (ret) {
		pr_info("****[%s] Unable to device register(%d)\n", __func__,
			ret);
		return ret;
	}
	ret = platform_driver_register(&sgm41513_user_space_driver);
	if (ret) {
		pr_info("****[%s] Unable to register driver (%d)\n", __func__,
			ret);
		return ret;
	}

	return 0;
}

static void __exit sgm41513_exit(void)
{
	i2c_del_driver(&sgm41513_driver);
}
module_init(sgm41513_init);
module_exit(sgm41513_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C sgm41513 slave Driver");
MODULE_AUTHOR("will cai <will.cai@mediatek.com>");
MODULE_VERSION(SGM41513_DRV_VERSION);
