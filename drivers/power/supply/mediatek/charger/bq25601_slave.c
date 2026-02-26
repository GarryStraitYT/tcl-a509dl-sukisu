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
#include <mt-plat/v1/charger_class.h>	//Modify by qiuguangliang for ODIN5G-5840 on 2022.02.18
#include <linux/power_supply.h>
//#include <linux/regulator/driver.h>
//Begin add by bin.song.hz for task:9988113 on 2020.9.22
//#include <linux/regulator/machine.h>
//End add by bin.song.hz for task:9988113 on 2020.9.22
/* Begin added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
#include <linux/mutex.h>
#include <linux/errno.h>
#include "bq25601.h"

#define BQ25601_DRV_VERSION	"V2.0_TCL"
/* End added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */


#define GETARRAYNUM(array) (ARRAY_SIZE(array))

/*bq25601 REG06 VREG[5:0]*/
static const unsigned int VBAT_CV_VTH[] = {
	3856000, 3888000, 3920000, 3952000,
	3984000, 4016000, 4048000, 4080000,
	4112000, 4144000, 4176000, 4208000,
	4240000, 4272000, 4304000, 4336000,
	4368000, 4400000, 4432000, 4464000,
	4496000, 4528000, 4560000, 4592000,
	4624000
};

/*BQ25601 REG04 ICHG[6:0]*/
static const unsigned int CS_VTH[] = {
	0, 60000, 120000, 180000, 240000,
	300000, 360000, 420000, 480000, 540000,
	600000, 660000, 720000, 780000, 840000,
	900000, 960000, 1020000, 1080000, 1140000,
	1200000, 1260000, 1320000, 1380000, 1440000,
	1500000, 1560000, 1620000, 1680000, 1740000,
	1800000, 1860000, 1920000, 1980000, 2040000,
	2100000, 2160000, 2220000, 2280000, 2340000,
	2400000, 2460000, 2520000, 2580000, 2640000,
	2700000, 2760000, 2820000, 2880000, 2940000,
	3000000
};

/*BQ25601 REG00 IINLIM[5:0]*/
static const unsigned int INPUT_CS_VTH[] = {
	100000, 200000, 300000, 400000,
	500000, 600000, 700000, 800000,
	900000, 1000000, 1100000, 1200000,
	1300000, 1400000, 1500000, 1600000,
	1700000, 1800000, 1900000, 2000000,
	2100000, 2200000, 2300000, 2400000,
	2500000, 2600000, 2700000, 2800000,
	2900000, 3000000, 3100000, 3200000
};

/* Begin mod by jin.wang for jira 291 on 2022-3-14 */
static const unsigned int __maybe_unused VINDPM_REG[] = {
	3900, 4000, 4100, 4200, 4300, 4400,
	4500, 4600, 4700, 4800, 4900, 5000,
	5100, 5200, 5300, 5400,
};
/* End mod by jin.wang */

/* BQ25601 REG02 BOOST_LIM[7], mA */
static const unsigned int __maybe_unused BOOST_CURRENT_LIMIT[] = {
	500, 1200
};

struct bq25601_info {
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	int irq;
	//struct regulator_dev *otg_rdev;

/* Begin added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
	int intr_gpio;
	int en_gpio;
	u8 chip_rev;
	atomic_t is_chip_en;
	struct mutex i2c_access_lock;
	struct mutex adc_access_lock;
	struct mutex gpio_access_lock;
	struct mutex irq_access_lock;
/* End added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
};

DEFINE_MUTEX(g_input_current_mutex);
static struct i2c_client *new_client;
static const struct i2c_device_id bq25601_i2c_id[] = { {"bq25601", 0}, {} };

static int bq25601_driver_probe(struct i2c_client *client,
				const struct i2c_device_id *id);

#if 0
static unsigned int charging_value_to_parameter(const unsigned int
		*parameter, const unsigned int array_size,
		const unsigned int val)
{
	if (val < array_size)
		return parameter[val];

	pr_info("Can't find the parameter\n");
	return parameter[0];

}
#endif

static unsigned int charging_parameter_to_value(const unsigned int
		*parameter, const unsigned int array_size,
		const unsigned int val)
{
	unsigned int i;

	pr_debug_ratelimited("array_size = %d\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	pr_info("NO register value match\n");
	/* TODO: ASSERT(0);    // not find the value */
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList,
		unsigned int number,
		unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

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


unsigned char bq25601_reg[bq25601_REG_NUM] = { 0 };

static DEFINE_MUTEX(bq25601_i2c_access);
static DEFINE_MUTEX(bq25601_access_lock);

static int g_bq25601_hw_exist; // change by TCT-cuiping.shi

#ifdef CONFIG_MTK_I2C_EXTENSION
unsigned int bq25601_read_byte(unsigned char cmd,
			       unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&bq25601_i2c_access);

	/* new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) |
	 * I2C_WR_FLAG;
	 */
	new_client->ext_flag =
		((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG |
		I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
		new_client->ext_flag = 0;
		mutex_unlock(&bq25601_i2c_access);

		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
	new_client->ext_flag = 0;
	mutex_unlock(&bq25601_i2c_access);

	return 1;
}

unsigned int bq25601_write_byte(unsigned char cmd,
				unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&bq25601_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) |
			       I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		new_client->ext_flag = 0;
		mutex_unlock(&bq25601_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&bq25601_i2c_access);
	return 1;
}
#else
static int bq25601_read_byte(unsigned char cmd,
			       unsigned char *returnData)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&bq25601_i2c_access);

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

	mutex_unlock(&bq25601_i2c_access);

	return ret == xfers ? 1 : -1;
}

static int bq25601_write_byte(unsigned char cmd,
				unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&bq25601_i2c_access);

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

	mutex_unlock(&bq25601_i2c_access);

	return ret == xfers ? 1 : -1;
}
#endif
static int bq25601_read_interface(unsigned char RegNum,
				    unsigned char *val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char bq25601_reg = 0;
	int ret = 0;

	ret = bq25601_read_byte(RegNum, &bq25601_reg);

	pr_debug_ratelimited("[%s] Reg[%x]=0x%x\n", __func__,
			     RegNum, bq25601_reg);

	bq25601_reg &= (MASK << SHIFT);
	*val = (bq25601_reg >> SHIFT);

	pr_debug_ratelimited("[%s] val=0x%x\n", __func__, *val);

	return ret;
}

static int bq25601_config_interface(unsigned char RegNum,
				      unsigned char val, unsigned char MASK,
				      unsigned char SHIFT)
{
	unsigned char bq25601_reg = 0;
	unsigned char bq25601_reg_ori = 0;
	int ret = 0;

	mutex_lock(&bq25601_access_lock);
	ret = bq25601_read_byte(RegNum, &bq25601_reg);

	bq25601_reg_ori = bq25601_reg;
	bq25601_reg &= ~(MASK << SHIFT);
	bq25601_reg |= (val << SHIFT);

	ret = bq25601_write_byte(RegNum, bq25601_reg);
	mutex_unlock(&bq25601_access_lock);
	pr_debug_ratelimited("[%s] write Reg[%x]=0x%x from 0x%x\n", __func__,
			     RegNum,
			     bq25601_reg, bq25601_reg_ori);

	return ret;
}

/* Begin added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
static inline bool __bq25601_is_chip_en(struct bq25601_info *info);
#if 0
static int bq25601_i2c_test_bit(struct bq25601_info *info, u8 cmd, u8 shift,
	bool *is_one)
{
	int ret = 0;
	u8 data = 0;
    unsigned char bq25601_reg = 0;

//	ret = bq25601_i2c_read_byte(info, cmd);
	ret = bq25601_read_byte(cmd, &bq25601_reg);
	if (ret < 0) {
		*is_one = false;
		return ret;
	}

	data = bq25601_reg & (1 << shift);
	*is_one = (data == 0 ? false : true);

	return ret;
}
#endif

static int bq25601_i2c_update_bits(struct bq25601_info *info, u8 cmd, u8 data,
	u8 mask)
{
	int ret = 0;
	u8 reg_data = 0;

	mutex_lock(&info->i2c_access_lock);
	mutex_lock(&info->gpio_access_lock);
	if (__bq25601_is_chip_en(info)) {
		ret = bq25601_read_byte(cmd, &reg_data);
		if (ret < 0)
			goto out;

		reg_data = ret & 0xFF;
		reg_data &= ~mask;
		reg_data |= (data & mask);

		ret = bq25601_write_byte(cmd, reg_data);
	} else
		ret = -EINVAL;

out:
	mutex_unlock(&info->gpio_access_lock);
	mutex_unlock(&info->i2c_access_lock);
	return ret;
}

static inline int bq25601_set_bit(struct bq25601_info *info, u8 reg, u8 mask)
{
	return bq25601_i2c_update_bits(info, reg, mask, mask);
}

static inline int bq25601_clr_bit(struct bq25601_info *info, u8 reg, u8 mask)
{
	return bq25601_i2c_update_bits(info, reg, 0x00, mask);
}
/* End added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */


/* CON0---------------------------------------------------- */
static inline void bq25601_set_en_hiz(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT));
}

static inline void bq25601_set_iinlim(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT));
}

static inline void bq25601_set_stat_ctrl(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON0),
				   (unsigned char) (val),
				   (unsigned char) (CON0_STAT_IMON_CTRL_MASK),
				   (unsigned char) (CON0_STAT_IMON_CTRL_SHIFT));
}

/* CON1---------------------------------------------------- */

static inline void bq25601_set_reg_rst(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON11),
				       (unsigned char) (val),
				       (unsigned char) (CON11_REG_RST_MASK),
				       (unsigned char) (CON11_REG_RST_SHIFT));
}

static inline void bq25601_set_pfm(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_PFM_MASK),
				       (unsigned char) (CON1_PFM_SHIFT));
}

static inline void bq25601_set_wdt_rst(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_WDT_RST_MASK),
				       (unsigned char) (CON1_WDT_RST_SHIFT));
}

static inline void bq25601_set_otg_config(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OTG_CONFIG_MASK),
				       (unsigned char) (CON1_OTG_CONFIG_SHIFT));
}

static unsigned int __maybe_unused bq25601_get_otg_config(void)
{
	unsigned char val = 0;

	bq25601_read_interface((unsigned char) (bq25601_CON1),
				     (&val),
				     (unsigned char) (CON1_OTG_CONFIG_MASK),
				     (unsigned char) (CON1_OTG_CONFIG_SHIFT));
	return val;
}

static inline void bq25601_set_chg_config(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CHG_CONFIG_MASK),
				       (unsigned char) (CON1_CHG_CONFIG_SHIFT));
}

static inline void bq25601_set_sys_min(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_SYS_MIN_MASK),
				       (unsigned char) (CON1_SYS_MIN_SHIFT));
}

static inline void bq25601_set_batlowv(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_MIN_VBAT_SEL_MASK),
				       (unsigned char) (CON1_MIN_VBAT_SEL_SHIFT));
}

/* CON2---------------------------------------------------- */
static inline void bq25601_set_rdson(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_Q1_FULLON_MASK),
				       (unsigned char) (CON2_Q1_FULLON_SHIFT));
}

#if 0
static inline void bq25601_set_boost_lim(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_BOOST_LIM_MASK),
				       (unsigned char) (CON2_BOOST_LIM_SHIFT));
}
#endif

static inline void bq25601_set_ichg(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_ICHG_MASK),
				       (unsigned char) (CON2_ICHG_SHIFT));
}

/* CON3---------------------------------------------------- */
static inline void bq25601_set_iprechg(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_IPRECHG_MASK),
				       (unsigned char) (CON3_IPRECHG_SHIFT));
}

static inline void bq25601_set_iterm(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_ITERM_MASK),
				       (unsigned char) (CON3_ITERM_SHIFT));
}

/* CON4---------------------------------------------------- */
static inline void bq25601_set_vreg(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VREG_MASK),
				       (unsigned char) (CON4_VREG_SHIFT));
}

#if 0
static inline void bq25601_set_topoff_timer(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_TOPOFF_TIMER_MASK),
				       (unsigned char) (CON4_TOPOFF_TIMER_SHIFT));
}
#endif

static inline void bq25601_set_vrechg(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VRECHG_MASK),
				       (unsigned char) (CON4_VRECHG_SHIFT));
}

/* CON5---------------------------------------------------- */
static inline void bq25601_set_en_term(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TERM_MASK),
				       (unsigned char) (CON5_EN_TERM_SHIFT));
}

static inline void bq25601_set_watchdog(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_WATCHDOG_MASK),
				       (unsigned char) (CON5_WATCHDOG_SHIFT));
}

static inline void bq25601_set_en_timer(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TIMER_MASK),
				       (unsigned char) (CON5_EN_TIMER_SHIFT));
}

#if 0
static inline void bq25601_set_chg_timer(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_CHG_TIMER_MASK),
				       (unsigned char) (CON5_CHG_TIMER_SHIFT));
}

/* CON6---------------------------------------------------- */
static inline void bq25601_set_treg(unsigned int val)
{
#ifdef FIXME
	bq25601_config_interface((unsigned char) (bq25601_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BOOSTV_MASK),
				       (unsigned char) (CON6_BOOSTV_SHIFT));
#endif
}
#endif

static inline void bq25601_set_vindpm(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VINDPM_MASK),
				       (unsigned char) (CON6_VINDPM_SHIFT));
}

static inline void bq25601_set_ovp(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_OVP_MASK),
				       (unsigned char) (CON6_OVP_SHIFT));
}

#if 0
static inline void bq25601_set_boostv(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BOOSTV_MASK),
				       (unsigned char) (CON6_BOOSTV_SHIFT));
}

/* CON7---------------------------------------------------- */
static inline void bq25601_set_tmr2x_en(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON7),
					(unsigned char) (val),
					(unsigned char) (CON7_TMR2X_EN_MASK),
					(unsigned char) (CON7_TMR2X_EN_SHIFT));
}
#endif

static inline void bq25601_set_batfet_disable(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON7),
				(unsigned char) (val),
				(unsigned char) (CON7_BATFET_Disable_MASK),
				(unsigned char) (CON7_BATFET_Disable_SHIFT));
}

#if 0
static inline void bq25601_set_batfet_delay(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_DLY_MASK),
				       (unsigned char) (CON7_BATFET_DLY_SHIFT));
}

static inline void bq25601_set_batfet_reset_enable(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON7),
				(unsigned char) (val),
				(unsigned char) (CON7_BATFET_RST_EN_MASK),
				(unsigned char) (CON7_BATFET_RST_EN_SHIFT));
}

/* CON8---------------------------------------------------- */
static unsigned int __maybe_unused bq25601_get_system_status(void)
{
	unsigned char val = 0;

	bq25601_read_interface((unsigned char) (bq25601_CON8),
				     (&val), (unsigned char) (0xFF),
				     (unsigned char) (0x0));
	return val;
}

static unsigned int __maybe_unused bq25601_get_vbus_stat(void)
{
	unsigned char val = 0;

	bq25601_read_interface((unsigned char) (bq25601_CON8),
				     (&val),
				     (unsigned char) (CON8_VBUS_STAT_MASK),
				     (unsigned char) (CON8_VBUS_STAT_SHIFT));
	return val;
}
#endif

static unsigned int bq25601_get_chrg_stat(void)
{
	unsigned char val = 0;

	bq25601_read_interface((unsigned char) (bq25601_CON8),
				     (&val),
				     (unsigned char) (CON8_CHRG_STAT_MASK),
				     (unsigned char) (CON8_CHRG_STAT_SHIFT));
	return val;
}

#if 0
static unsigned int __maybe_unused bq25601_get_vsys_stat(void)
{
	unsigned char val = 0;

	bq25601_read_interface((unsigned char) (bq25601_CON8),
				     (&val),
				     (unsigned char) (CON8_VSYS_STAT_MASK),
				     (unsigned char) (CON8_VSYS_STAT_SHIFT));
	return val;
}

static unsigned int __maybe_unused bq25601_get_pg_stat(void)
{
	unsigned char val = 0;

	bq25601_read_interface((unsigned char) (bq25601_CON8),
				     (&val),
				     (unsigned char) (CON8_PG_STAT_MASK),
				     (unsigned char) (CON8_PG_STAT_SHIFT));
	return val;
}
#endif

/*CON10----------------------------------------------------------*/
static inline void bq25601_set_int_mask(unsigned int val)
{
	bq25601_config_interface((unsigned char) (bq25601_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_INT_MASK_MASK),
				       (unsigned char) (CON10_INT_MASK_SHIFT));
}

#if defined(CONFIG_TCT_NB_CHG_5G_PATCH)
static int bq25601_dump_register(struct charger_device *chg_dev)
{
	unsigned char i = 0;
	int ret = 0;

	pr_info("[bq25601]\n");
	for (i = 0; i < bq25601_REG_NUM; i++) {
		ret = bq25601_read_byte(i, &bq25601_reg[i]);
		if (ret < 0) {
			pr_info("[bq25601] i2c transfor error\n");
			return 0;
		}
		pr_info("[0x%x]=0x%x\n", i, bq25601_reg[i]);
	}
	return 0;
}
#else
static int bq25601_dump_register(struct charger_device *chg_dev)
{
	unsigned int ret = 0;
	unsigned int i = 0;
	unsigned int ichg_set = 0;
	unsigned int iinlim = 0;
	unsigned int chg_state = 0;
	unsigned int chg_en = 0;
	unsigned int fault = 0;
	unsigned int vindpm =0;
	unsigned int cv_set=0;
	unsigned int vindpm_vbat =0;
	unsigned int ovp_set =0;
	unsigned int vbus_status =0;
	unsigned int ovp_cts[]={5500,6500,10500,14000};

	for (i = 0; i < bq25601_REG_NUM; i++) {
		ret = bq25601_read_byte(i, &bq25601_reg[i]);
		if (ret < 0) {
			pr_info("[bq25601] i2c transfor error\n");
			return 1;
		}
	}

	iinlim = (bq25601_reg[bq25601_CON0] & (CON0_IINLIM_MASK << CON0_IINLIM_SHIFT))>>CON0_IINLIM_SHIFT;
	iinlim = 100 + iinlim * 100;
	chg_state = (bq25601_reg[bq25601_CON8] & (CON8_CHRG_STAT_MASK << CON8_CHRG_STAT_SHIFT))>>CON8_CHRG_STAT_SHIFT;
	chg_en = (bq25601_reg[bq25601_CON1] & (CON1_CHG_CONFIG_MASK << CON1_CHG_CONFIG_SHIFT))>>CON1_CHG_CONFIG_SHIFT;
	ichg_set = (bq25601_reg[bq25601_CON2] & (CON2_ICHG_MASK << CON2_ICHG_SHIFT))>>CON2_ICHG_SHIFT;
	ichg_set = ichg_set * 60;
	fault = (bq25601_reg[bq25601_CON9] & (CON9_CHRG_FAULT_MASK << CON9_CHRG_FAULT_SHIFT))>>CON9_CHRG_FAULT_SHIFT;
	vindpm = (bq25601_reg[bq25601_CON6] & (CON6_VINDPM_MASK << CON6_VINDPM_SHIFT))>>CON6_VINDPM_SHIFT;
	vindpm = vindpm * 100 + 3900;
	vindpm_vbat = (bq25601_reg[bq25601_CON7] & (CON7_VDPM_BAT_TRACK_MASK << CON7_VDPM_BAT_TRACK_SHIFT))>>CON7_VDPM_BAT_TRACK_SHIFT;
	vindpm_vbat = (vindpm_vbat == 0)? 0: (vindpm_vbat* 50 + 150);
	cv_set = (bq25601_reg[bq25601_CON4] & (CON4_VREG_MASK << CON4_VREG_SHIFT))>>CON4_VREG_SHIFT;
	cv_set = 3856 + cv_set * 32;  // mod by jin.wang for jira-34 at 2022-1-6
	ovp_set = (bq25601_reg[bq25601_CON6] & (CON6_OVP_MASK << CON6_OVP_SHIFT))>>CON6_OVP_SHIFT;
	vbus_status = (bq25601_reg[bq25601_CON8] & (CON8_VBUS_STAT_MASK << CON8_VBUS_STAT_SHIFT))>>CON8_VBUS_STAT_SHIFT;

	printk("[%s %d] reg(0)=0x%02x, reg(1)=0x%02x, reg(2)=0x%02x, reg(3)=0x%02x, reg(4)=0x%02x, reg(5)=0x%02x, \
reg(6)=0x%02x, reg(7)=0x%02x, reg(8)=0x%02x, reg(9)=0x%02x, reg(a)=0x%02x, reg(b)=0x%02x \n", __func__, __LINE__,
		bq25601_reg[bq25601_CON0], bq25601_reg[bq25601_CON1],bq25601_reg[bq25601_CON2],
		bq25601_reg[bq25601_CON3],bq25601_reg[bq25601_CON4],bq25601_reg[bq25601_CON5],
		bq25601_reg[bq25601_CON6],bq25601_reg[bq25601_CON7],bq25601_reg[bq25601_CON8],
		bq25601_reg[bq25601_CON9],bq25601_reg[bq25601_CON10],bq25601_reg[bq25601_CON11]);

	printk("[%s %d] fault=%d, chg_en=%d, ichg_set=%d, ilim_set=%d, cv_set=%d, ovp=%d, vindpm=%d, vbus_status=%d \n", __func__, __LINE__,
		fault, chg_en, ichg_set, iinlim, cv_set, ovp_cts[ovp_set], vindpm, vbus_status);

	return 0;
}
#endif

static void bq25601_hw_component_detect(void)
{
	int ret = 0;
	unsigned char val = 0;

	ret = bq25601_read_interface(0x0B, &val, 0xFF, 0x0);
	if (ret < 0) {
		pr_err("%s: bq25601 i2c read err: %d\n", __func__, ret);
		return;
	}

	if (val == 0)
		g_bq25601_hw_exist = 0;
	else
		g_bq25601_hw_exist = 1;

	pr_err("[%s] exist=%d, Reg[0x0B]=0x%x, ret=%d\n", __func__,
			g_bq25601_hw_exist, val, ret);
}

/* Begin added by weijun for termination on 2021-08-16 */
static int bq25601_enable_term(struct charger_device *chg_dev,
				   bool en)
{
	pr_err("[%s] enable:%d\n", __func__, en);
	if (en) {
		bq25601_set_en_term(0x1);
	} else {
		bq25601_set_en_term(0x0);
	}

	return 0;
}
/* End added by weijun for termination on 2021-08-16 */

static int bq25601_enable_charging(struct charger_device *chg_dev,
				   bool en)
{
	pr_err("[%s] enable state : %d\n", __func__, en);
	if (en) {
		/* bq25601_config_interface(bq25601_CON3, 0x1, 0x1, 4); */
		/* enable charging */
		bq25601_set_en_hiz(0x0);

/* Begin add by jin.wang for jira 9612 on 2022-4-6 */
#if defined(CONFIG_TCT_NB_CHG_5G_PATCH)
		bq25601_set_otg_config(0);
#endif
/* End add by jin.wang */

		bq25601_set_chg_config(en);
		bq25601_set_batfet_disable(0);	/* enable Q4 turn on */
	} else {
		/* bq25601_config_interface(bq25601_CON3, 0x0, 0x1, 4); */
		/* enable charging */
		bq25601_set_chg_config(en);
		//pr_info("[charging_enable] under test mode: disable charging\n");
		bq25601_set_batfet_disable(1);	/* Disable Q4 turn off default */

/* Begin add by jin.wang for jira 9612 on 2022-4-6 */
#if defined(CONFIG_TCT_NB_CHG_5G_PATCH)
		bq25601_set_otg_config(0);
#endif
/* End add by jin.wang */

		/*bq25601_set_en_hiz(0x1);*/
	}

	return 0;
}

/* Begin added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
static inline bool __bq25601_is_chip_en(struct bq25601_info *info)
{
	int en = -1;
    bool is_enabled = 0;

	en = gpio_get_value(info->en_gpio);

    if(0 == en) {
        is_enabled = 1;
    } else {
        is_enabled = 0;
	}

/* Begin del by jin.wang for jira 291 on 2022-3-14 */
#if !defined(CONFIG_TCT_NB_CHG_5G_PATCH)
    pr_err("%s info->en_gpio = %d, is_enabled=%d\n", __func__, en,is_enabled);
	if ((is_enabled && atomic_read(&info->is_chip_en)) ||
		(!is_enabled && !atomic_read(&info->is_chip_en)))
		dev_notice(info->dev, "%s: en not sync(%d, %d)\n", __func__, is_enabled,
			atomic_read(&info->is_chip_en));
#endif
/* End del by jin.wang */

	return is_enabled;
}

static int __bq25601_enable_chip(struct bq25601_info *info, bool en)
{
	bool is_chip_en = false;

/* Begin del by jin.wang for jira 291 on 2022-3-14 */
#if !defined(CONFIG_TCT_NB_CHG_5G_PATCH)
	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	pr_err("%s logic enable = %d\n", __func__, en);
#endif
/* End del by jin.wang */

	mutex_lock(&info->gpio_access_lock);
	is_chip_en = __bq25601_is_chip_en(info);
	if (en && !is_chip_en) {
		gpio_set_value(info->en_gpio, 0);
		dev_info(info->dev, "%s: set gpio low\n", __func__);
//        bq25601_set_batfet_disable(0);	/* enable Q4 turn on */
	} else if (!en && is_chip_en) {
		gpio_set_value(info->en_gpio, 1);
		dev_info(info->dev, "%s: set gpio high\n", __func__);
//        bq25601_set_batfet_disable(1);	/* Disable Q4 turn off default */
	}

	/* Wait for chip's enable/disable */
	mdelay(1);
	atomic_set(&info->is_chip_en, en);
	mutex_unlock(&info->gpio_access_lock);
	return 0;
}

static int bq25601_hw_init(void);

static int bq25601_enable_chip(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct bq25601_info *info = dev_get_drvdata(&chg_dev->dev);

	/* Do the following flow for enabling chip */
    pr_err("%s enable = %d\n", __func__, en);
	if (!g_bq25601_hw_exist) {
		dev_info(info->dev, "%s: no bq25601 exists\n", __func__);
		return -ENODEV;
	}

	bq25601_hw_init();

	ret = __bq25601_enable_chip(info, en);
	if (ret < 0)
		return ret;

/* Begin add by jin.wang for jira 9612 on 2022-4-6 */
#if defined(CONFIG_TCT_NB_CHG_5G_PATCH)
	bq25601_set_otg_config(0);
#endif
/* End add by jin.wang */

/* Begin del by jin.wang for jira 291 on 2022-4-26 */
#if !defined(CONFIG_TCT_NB_CHG_5G_PATCH)
	if (!en)
		return 0;

	bq25601_dump_register(info->chg_dev);
#endif
/* End del by jin.wang */

	return ret;
}

static int bq25601_is_chip_enabled(struct charger_device *chg_dev, bool *en)
{
	struct bq25601_info *info = dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&info->gpio_access_lock);
	*en = __bq25601_is_chip_en(info);
	mutex_unlock(&info->gpio_access_lock);
    pr_err("%s after state = %d\n", __func__, *en);

	return 0;
}

static int bq25601_is_charging_enabled(struct charger_device *chg_dev, bool *en)
{
	unsigned char ret_val = 0;

	bq25601_read_interface(bq25601_CON1, &ret_val, CON1_CHG_CONFIG_MASK, CON1_CHG_CONFIG_SHIFT);
	*en = ret_val;
	pr_err("%s = %d\n", __func__, *en);
	return 0;
}

static int bq25601_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	unsigned int array_size;

	array_size = GETARRAYNUM(CS_VTH);
	*uA = bmt_find_closest_level(CS_VTH, array_size, 0);
    pr_err("%s enable = %d\n", __func__, *uA);

	return 0;
}

/* End added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */

#if 0
static int bq25601_get_current(struct charger_device *chg_dev,
			       u32 *ichg)
{
	unsigned int ret_val = 0;

#ifdef FIXME
	unsigned char ret_force_20pct = 0;

	/* Get current level */
	bq25601_read_interface(bq25601_CON2, &ret_val, CON2_ICHG_MASK,
					CON2_ICHG_SHIFT);

	/* Get Force 20% option */
	bq25601_read_interface(bq25601_CON2, &ret_force_20pct,
					CON2_FORCE_20PCT_MASK,
					CON2_FORCE_20PCT_SHIFT);

	/* Parsing */
	ret_val = (ret_val * 64) + 512;
#endif
	return ret_val;
}
#else
static int bq25601_get_current(struct charger_device *chg_dev, u32 *ichg)
{
	unsigned char ret_val = 0;

	bq25601_read_interface(bq25601_CON2, &ret_val, CON2_ICHG_MASK,
					CON2_ICHG_SHIFT);
	*ichg = ret_val * 60 * 1000; //uA

	printk("[%s %d] ichg=%d\n", __func__, __LINE__, *ichg);
	return 0;
}
#endif

static int bq25601_set_current(struct charger_device *chg_dev,
			       u32 current_value)
{
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	pr_info("[%s] charge_current_value = %ld\n", __func__, current_value);
	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size,
			  current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size,
			 set_chr_current);
	pr_info("[%s] register_value = %d\n", __func__, register_value);
	bq25601_set_ichg(register_value);

	return 0;
}

static int bq25601_get_input_current(struct charger_device *chg_dev,
				     u32 *aicr)
{
	int ret = 0;
#ifdef FIXME
	unsigned char val = 0;

	bq25601_read_interface(bq25601_CON0, &val, CON0_IINLIM_MASK,
			       CON0_IINLIM_SHIFT);
	ret = (int)val;
	*aicr = INPUT_CS_VTH[val];
#endif
	return ret;
}

static int bq25601_set_input_current(struct charger_device *chg_dev,
				     u32 current_value)
{
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	pr_info("&&&& current_value = %d\n", current_value);
	array_size = GETARRAYNUM(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size,
			  current_value);
	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size,
			 set_chr_current);
	pr_info("&&&& %s register_value = %d\n", __func__,
		register_value);
	bq25601_set_iinlim(register_value);

	return 0;
}

static int bq25601_set_cv_voltage(struct charger_device *chg_dev,
				  u32 cv)
{
	unsigned int array_size;
	unsigned int set_cv_voltage;
	unsigned short register_value;

	array_size = GETARRAYNUM(VBAT_CV_VTH);
	set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv);
	register_value = charging_parameter_to_value(VBAT_CV_VTH, array_size,
			 set_cv_voltage);
	bq25601_set_vreg(register_value);
	pr_info("&&&& cv reg value = %d\n", register_value);

	return 0;
}

static int bq25601_reset_watch_dog_timer(struct charger_device
		*chg_dev)
{
	pr_info("charging_reset_watch_dog_timer\n");

	bq25601_set_wdt_rst(0x1);	/* Kick watchdog */
	bq25601_set_watchdog(0x3);	/* WDT 160s */
	return 0;
}

static int bq25601_set_vindpm_voltage(struct charger_device *chg_dev,
				      u32 vindpm)
{
/* Begin mod by jin.wang for jira 291 on 2022-3-14 */
#if !defined(CONFIG_TCT_NB_CHG_5G_PATCH)
	int status = 0;
	unsigned int array_size;

	vindpm /= 1000;
	array_size = ARRAY_SIZE(VINDPM_REG);
	vindpm = bmt_find_closest_level(VINDPM_REG, array_size, vindpm);
	vindpm = charging_parameter_to_value(VINDPM_REG, array_size, vindpm);

	pr_info("%s vindpm =%d\r\n", __func__, vindpm);

	//	charging_set_vindpm(vindpm);
	/*bq25601_set_en_hiz(en);*/

	return status;
#else
	return 0;
#endif
/* End mod by jin.wang */
}

static int bq25601_get_charging_status(struct charger_device *chg_dev,
				       bool *is_done)
{
	unsigned int ret_val;

	ret_val = bq25601_get_chrg_stat();

	if (ret_val == 0x3)
		*is_done = true;
	else
		*is_done = false;

	return 0;
}

#if 0
static int bq25601_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;

	pr_info("%s en = %d\n", __func__, en);
	if (en) {
		bq25601_set_chg_config(0);
		bq25601_set_otg_config(1);
		bq25601_set_watchdog(0x3);	/* WDT 160s */
	} else {
		bq25601_set_otg_config(0);
		bq25601_set_chg_config(1);
	}
	return ret;
}

static int bq25601_set_boost_current_limit(struct charger_device
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
	bq25601_set_boost_lim(boost_reg);

	return ret;
}
#endif

static int bq25601_enable_safetytimer(struct charger_device *chg_dev,
				      bool en)
{
	if (en)
		bq25601_set_en_timer(0x1);
	else
		bq25601_set_en_timer(0x0);

	return 0;
}

static int bq25601_get_is_safetytimer_enable(struct charger_device
		*chg_dev, bool *en)
{
	unsigned char val = 0;

	bq25601_read_interface(bq25601_CON5, &val, CON5_EN_TIMER_MASK,
			       CON5_EN_TIMER_SHIFT);
	*en = (bool)val;
	return val;
}

/* Begin added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
static int bq25601_hw_init(void)
{
	bq25601_set_en_hiz(0x0);
	bq25601_set_iinlim(0x0);	/* Iinlim 100mA */
	bq25601_set_ichg(0x08);	/* ICHG 480mA */
	bq25601_set_vindpm(0x7);	/* SGM41511 7 BQ25601 6 VIN DPM check 4.6V */
	bq25601_set_wdt_rst(0x1);	/* Kick watchdog */
	bq25601_set_sys_min(0x5);	/* Minimum system voltage 3.5V */
	bq25601_set_iprechg(0x8);	/* Precharge current 540mA */
	bq25601_set_iterm(0x4);	    /* Termination current 300mA */
	bq25601_set_vreg(0x11);	    /* VREG 4.4V */
    bq25601_set_ovp(0x02);	    /* ovp 10.5V for 9v input */
	bq25601_set_pfm(0);       // enable pfm //modify by qiuguangliang for ODIN5G-6579 on 20220415
	bq25601_set_rdson(0x0);     /*close rdson*/
	bq25601_set_batlowv(0x1);	/* BATLOWV 3.0V */
	bq25601_set_vrechg(0x0);	/* VRECHG 0.1V (4.108V) */
	bq25601_set_en_term(0x1);	/* Enable termination */
	bq25601_set_watchdog(0x0);	/* WDT disable */

/* Begin mod by jin.wang for jira 291 on 2022-3-14 */
#if defined(CONFIG_TCT_NB_CHG_5G_PATCH)
	bq25601_set_en_timer(0x1);	/* Enable charge timer */
#else
	bq25601_set_en_timer(0x0);	/* Enable charge timer */
#endif
/* End mod by jin.wang */

	bq25601_set_int_mask(0x0);	/* Disable fault interrupt */
	bq25601_set_chg_config(0);	/* Disable charge default */
    bq25601_set_batfet_disable(1);	/* Disable Q4 turn off default */

/* Begin add by jin.wang for jira 9612 on 2022-4-6 */
#if defined(CONFIG_TCT_NB_CHG_5G_PATCH)
	bq25601_set_otg_config(0);
#endif
/* End add by jin.wang */

	pr_info("%s: hw_init down!\n", __func__);
	return 0;
}

/* Begin mod by jin.wang for jira 291 on 2022-4-21 */
static int bq25601_parse_dt(struct bq25601_info *info,
			    struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

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
		info->chg_props.alias_name = "bq25601";
		pr_err("%s: no alias name\n", __func__);
	} else {
        pr_err("%s: alias name=%s\n", __func__,info->chg_props.alias_name);
    }

    info->en_gpio = of_get_named_gpio(np,"gpio_bq25601_en",0);
    if(info->en_gpio <= 0){
        pr_err("%s: no bq25601_en_pin\n", __func__);
        return -ENODATA;
    } else {
        pr_err("%s: en_pin=%d\n", __func__, info->en_gpio);
    }

    ret = devm_gpio_request(dev, info->en_gpio, "gpio_bq25601_en_num");
	if (ret) {
		pr_err("%s: request bq25601_en_pin err\n", __func__);
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
/* End mod by jin.wang */
/* End added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */


/* Add-start by baiwei.peng for hiz enable on 2022/11/16 */
static int bq25601_enable_hz(struct charger_device *chg_dev, bool en)
{
	pr_err("[%s] hiz en : %d\n", __func__, en);

        if (en) {
		bq25601_set_en_hiz(0x1);
	} else {
		bq25601_set_en_hiz(0x0);
	}

	return 0;
}
/* Add-end by baiwei.peng for hiz enable on 2022/11/16 */

static struct charger_ops bq25601_chg_ops = {
//#ifdef FIXME
	.enable_hz = bq25601_enable_hz,
//#endif

	/* Normal charging */
	.dump_registers = bq25601_dump_register,
	.enable = bq25601_enable_charging,
/* Begin added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
	/*.enable_chip is necessary for parallel charging*/
    .enable_chip = bq25601_enable_chip,
    .is_enabled = bq25601_is_charging_enabled,
    .is_chip_enabled = bq25601_is_chip_enabled,
 	.get_min_charging_current = bq25601_get_min_ichg,
/* End added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
	.get_charging_current = bq25601_get_current,
	.set_charging_current = bq25601_set_current,
	.get_input_current = bq25601_get_input_current,
	.set_input_current = bq25601_set_input_current,
	/*.get_constant_voltage = bq25601_get_battery_voreg,*/
	.set_constant_voltage = bq25601_set_cv_voltage,
	.kick_wdt = bq25601_reset_watch_dog_timer,
	.set_mivr = bq25601_set_vindpm_voltage,
	.is_charging_done = bq25601_get_charging_status,
	.enable_termination = bq25601_enable_term, // weijun ++

	/* Safety timer */
	.enable_safety_timer = bq25601_enable_safetytimer,
	.is_safety_timer_enabled = bq25601_get_is_safetytimer_enable,


	/* Power path */
	/*.enable_powerpath = bq25601_enable_power_path, */
	/*.is_powerpath_enabled = bq25601_get_is_power_path_enable, */


	/* OTG */
	/*
	.enable_otg = bq25601_enable_otg,
	.set_boost_current_limit = bq25601_set_boost_current_limit,
	.event = bq25601_do_event,
	*/
};

/* Begin added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
static int bq25601_driver_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	struct bq25601_info *info = NULL;
	//struct regulator_config config = { };

    pr_info("%s (%s)\n", __func__, BQ25601_DRV_VERSION);

	info = devm_kzalloc(&client->dev, sizeof(struct bq25601_info),
			    GFP_KERNEL);
	if (!info){
        pr_err("%s: bq25601 kzalloc fail\n", __func__);
		return -ENOMEM;
    }

	new_client = client;
	info->dev = &client->dev;

/* Begin mod by jin.wang for jira 291 on 2022-4-22 */
	bq25601_hw_component_detect();
	if (!g_bq25601_hw_exist){
		pr_err("%s: g_bq25601_hw_exist =0 return error!!!\n", __func__);
		return -ENODEV;
	}

	ret = bq25601_parse_dt(info, &client->dev);
	if (ret < 0){
        pr_err("%s: bq25601_parse_dt fail\n", __func__);
		return ret;
    }
/* End mod by jin.wang */

	//add begin by weijun for lock init
	mutex_init(&info->i2c_access_lock);
	mutex_init(&info->gpio_access_lock);
	//add end by weijun for lock init

/* Begin mod by jin.wang for jira 291 on 2022-4-22 */
	gpio_direction_output(info->en_gpio, 1);
	bq25601_hw_init();
/* End mod by jin.wang */

	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
						&client->dev, info,
						&bq25601_chg_ops,
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
	//config.init_data = &bq25601_vbus_init_data;
	//End add by bin.song.hz for task:9988113 on 2020.9.22
	/*
	info->otg_rdev = devm_regulator_register(info->dev,
						&bq25601_otg_rdesc, &config);
	if (IS_ERR(info->otg_rdev)) {
		ret = PTR_ERR(info->otg_rdev);
		pr_info("%s: register otg regulator failed (%d)\n", __func__, ret);
		return ret;
	}else {
		pr_info("%s: register otg regulator successed (%d)\n", __func__, ret);
    }
    */

	bq25601_dump_register(info->chg_dev);

	return 0;
}
/* End added by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */

static unsigned char g_reg_value_bq25601;
static ssize_t bq25601_access_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	pr_info("[%s] 0x%x\n", __func__, g_reg_value_bq25601);
	return sprintf(buf, "%u\n", g_reg_value_bq25601);
}

static ssize_t bq25601_access_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	pr_info("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		pr_info("[%s] buf is %s and size is %zu\n", __func__, buf,
			size);

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16,
				(unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16,
				(unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);
			pr_info(
			"[%s] write bq25601 reg 0x%x with value 0x%x !\n",
			__func__,
			(unsigned int) reg_address, reg_value);
			ret = bq25601_config_interface(reg_address,
				reg_value, 0xFF, 0x0);
		} else {
			ret = bq25601_read_interface(reg_address,
					     &g_reg_value_bq25601, 0xFF, 0x0);
			pr_info("[%s] read bq25601 reg 0x%x with value 0x%x !\n",
					__func__, (unsigned int) reg_address,
					g_reg_value_bq25601);
			pr_info("[%s] use \"cat bq25601_access\" to get value\n",
					__func__);
		}
	}
	return size;
}

static DEVICE_ATTR_RW(bq25601_access);

static int bq25601_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	if (!g_bq25601_hw_exist){
		pr_err("%s: g_bq25601_hw_exist =0 return !!!\n", __func__);
		return -ENODEV;
	}

	pr_info("******** %s!! ********\n", __func__);

	ret_device_file = device_create_file(&(dev->dev),
					     &dev_attr_bq25601_access);

	return 0;
}

static struct platform_device bq25601_user_space_device = {
	.name = "bq25601-user",
	.id = -1,
};

static struct platform_driver bq25601_user_space_driver = {
	.probe = bq25601_user_space_probe,
	.driver = {
		.name = "bq25601-user",
	},
};

#ifdef CONFIG_OF
static const struct of_device_id bq25601_of_match[] = {
	{.compatible = "mediatek,bq25601"},
	{},
};
#endif

static struct i2c_driver bq25601_driver = {
	.driver = {
		.name = "bq25601",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = bq25601_of_match,
#endif
	},
	.probe = bq25601_driver_probe,
	.id_table = bq25601_i2c_id,
};

static int __init bq25601_init(void)
{
	int ret = 0;

	/* i2c registration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
	pr_info("[%s] init start with i2c DTS", __func__);
#else
	pr_info("[%s] init start. ch=%d\n", __func__, bq25601_BUSNUM);
	i2c_register_board_info(bq25601_BUSNUM, &i2c_bq25601, 1);
#endif

	if (i2c_add_driver(&bq25601_driver) != 0) {
		pr_info("[%s] failed to register bq25601 driver.\n",
				__func__);
		return -ENODEV;
	} else {
		pr_info("[%s] Success to register bq25601 driver.\n",
				__func__);
	}

	/* bq25601 user space access interface */
	ret = platform_device_register(&bq25601_user_space_device);
	if (ret) {
		pr_info("****[%s] Unable to device register(%d)\n", __func__,
			ret);
		return ret;
	}
	ret = platform_driver_register(&bq25601_user_space_driver);
	if (ret) {
		pr_info("****[%s] Unable to register driver (%d)\n", __func__,
			ret);
		return ret;
	}

	return 0;
}

static void __exit bq25601_exit(void)
{
	i2c_del_driver(&bq25601_driver);
}
module_init(bq25601_init);
module_exit(bq25601_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq25601 slave Driver");
MODULE_AUTHOR("will cai <will.cai@mediatek.com>");
MODULE_VERSION(BQ25601_DRV_VERSION);
