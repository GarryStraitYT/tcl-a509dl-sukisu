#define pr_fmt(fmt)	"[eta6963]:%s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>

#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/alarmtimer.h>

#include "charger_class.h"


/* Register 00h */
#define ETA6963_REG_00      		0x00
#define REG00_ENHIZ_MASK		    0x80
#define REG00_ENHIZ_SHIFT		    7
#define	REG00_HIZ_ENABLE			1
#define	REG00_HIZ_DISABLE			0

#define	REG00_STAT_CTRL_MASK		0x60
#define REG00_STAT_CTRL_SHIFT		5
#define	REG00_STAT_CTRL_STAT		0
#define	REG00_STAT_CTRL_ICHG		1
#define	REG00_STAT_CTRL_IINDPM		2
#define	REG00_STAT_CTRL_DISABLE		3

#define REG00_IINLIM_MASK		    0x1F
#define REG00_IINLIM_SHIFT			0
#define	REG00_IINLIM_LSB			100
#define	REG00_IINLIM_BASE			100

/* Register 01h */
#define ETA6963_REG_01		    	0x01
#define REG01_PFM_DIS_MASK	      	0x80
#define	REG01_PFM_DIS_SHIFT			7
#define	REG01_PFM_ENABLE			0
#define	REG01_PFM_DISABLE			1

#define REG01_WDT_RESET_MASK		0x40
#define REG01_WDT_RESET_SHIFT		6
#define REG01_WDT_RESET				1

#define	REG01_OTG_CONFIG_MASK		0x20
#define	REG01_OTG_CONFIG_SHIFT		5
#define	REG01_OTG_ENABLE			1
#define	REG01_OTG_DISABLE			0

#define REG01_CHG_CONFIG_MASK     	0x10
#define REG01_CHG_CONFIG_SHIFT    	4
#define REG01_CHG_DISABLE        	0
#define REG01_CHG_ENABLE         	1

#define REG01_SYS_MINV_MASK       	0x0E
#define REG01_SYS_MINV_SHIFT      	1

#define	REG01_MIN_VBAT_SEL_MASK		0x01
#define	REG01_MIN_VBAT_SEL_SHIFT	0
#define	REG01_MIN_VBAT_2P8V			0
#define	REG01_MIN_VBAT_2P5V			1


/* Register 0x02*/
#define ETA6963_REG_02              0x02
#define	REG02_BOOST_LIM_MASK		0x80
#define	REG02_BOOST_LIM_SHIFT		7
#define	REG02_BOOST_LIM_0P5A		0
#define	REG02_BOOST_LIM_1P2A		1

#define	REG02_Q1_FULLON_MASK		0x40
#define	REG02_Q1_FULLON_SHIFT		6
#define	REG02_Q1_FULLON_ENABLE		1
#define	REG02_Q1_FULLON_DISABLE		0

#define REG02_ICHG_MASK           	0x3F
#define REG02_ICHG_SHIFT          	0
#define REG02_ICHG_BASE           	0
#define REG02_ICHG_LSB            	60

/* Register 0x03*/
#define ETA6963_REG_03              	0x03
#define REG03_IPRECHG_MASK        	0xF0
#define REG03_IPRECHG_SHIFT       	4
#define REG03_IPRECHG_BASE        	60
#define REG03_IPRECHG_LSB         	60

#define REG03_ITERM_MASK          	0x0F
#define REG03_ITERM_SHIFT         	0
#define REG03_ITERM_BASE          	60
#define REG03_ITERM_LSB           	60


/* Register 0x04*/
#define ETA6963_REG_04              0x04
#define REG04_VREG_MASK           	0xF8
#define REG04_VREG_SHIFT          	3
#define REG04_VREG_BASE           	3856
#define REG04_VREG_LSB            	32

#define	REG04_TOPOFF_TIMER_MASK		0x06
#define	REG04_TOPOFF_TIMER_SHIFT	1
#define	REG04_TOPOFF_TIMER_DISABLE	0
#define	REG04_TOPOFF_TIMER_15M		1
#define	REG04_TOPOFF_TIMER_30M		2
#define	REG04_TOPOFF_TIMER_45M		3


#define REG04_VRECHG_MASK         	0x01
#define REG04_VRECHG_SHIFT        	0
#define REG04_VRECHG_100MV        	0
#define REG04_VRECHG_200MV        	1

/* Register 0x05*/
#define ETA6963_REG_05             	0x05
#define REG05_EN_TERM_MASK        	0x80
#define REG05_EN_TERM_SHIFT       	7
#define REG05_TERM_ENABLE         	1
#define REG05_TERM_DISABLE        	0

#define REG05_WDT_MASK            	0x30
#define REG05_WDT_SHIFT           	4
#define REG05_WDT_DISABLE         	0
#define REG05_WDT_40S             	1
#define REG05_WDT_80S             	2
#define REG05_WDT_160S            	3
#define REG05_WDT_BASE            	0
#define REG05_WDT_LSB             	40

#define REG05_EN_TIMER_MASK       	0x08
#define REG05_EN_TIMER_SHIFT      	3
#define REG05_CHG_TIMER_ENABLE    	1
#define REG05_CHG_TIMER_DISABLE   	0

#define REG05_CHG_TIMER_MASK      	0x04
#define REG05_CHG_TIMER_SHIFT     	2
#define REG05_CHG_TIMER_5HOURS    	0
#define REG05_CHG_TIMER_10HOURS   	1

#define	REG05_TREG_MASK				0x02
#define	REG05_TREG_SHIFT			1
#define	REG05_TREG_90C				0
#define	REG05_TREG_110C				1

#define REG05_JEITA_ISET_MASK     	0x01
#define REG05_JEITA_ISET_SHIFT    	0
#define REG05_JEITA_ISET_50PCT    	0
#define REG05_JEITA_ISET_20PCT    	1


/* Register 0x06*/
#define ETA6963_REG_06              0x06
#define	REG06_OVP_MASK				0xC0
#define	REG06_OVP_SHIFT				0x6
#define	REG06_OVP_5P5V				0
#define	REG06_OVP_6P5V				1
#define	REG06_OVP_10P5V				2
#define	REG06_OVP_14P0V				3

#define	REG06_BOOSTV_MASK			0x30
#define	REG06_BOOSTV_SHIFT			4
#define	REG06_BOOSTV_4P85V			0
#define	REG06_BOOSTV_5V				1
#define	REG06_BOOSTV_5P15V			2
#define	REG06_BOOSTV_5P3V			3

#define	REG06_VINDPM_MASK			0x0F
#define	REG06_VINDPM_SHIFT			0
#define	REG06_VINDPM_BASE			3900
#define	REG06_VINDPM_LSB			100

/* Register 0x07*/
#define ETA6963_REG_07              0x07
#define REG07_FORCE_DPDM_MASK     	0x80
#define REG07_FORCE_DPDM_SHIFT    	7
#define REG07_FORCE_DPDM          	1

#define REG07_TMR2X_EN_MASK       	0x40
#define REG07_TMR2X_EN_SHIFT      	6
#define REG07_TMR2X_ENABLE        	1
#define REG07_TMR2X_DISABLE       	0

#define REG07_BATFET_DIS_MASK     	0x20
#define REG07_BATFET_DIS_SHIFT    	5
#define REG07_BATFET_OFF          	1
#define REG07_BATFET_ON          	0

#define REG07_JEITA_VSET_MASK     	0x10
#define REG07_JEITA_VSET_SHIFT    	4
#define REG07_JEITA_VSET_4100     	0
#define REG07_JEITA_VSET_VREG     	1

#define	REG07_BATFET_DLY_MASK		0x08
#define	REG07_BATFET_DLY_SHIFT		3
#define	REG07_BATFET_DLY_0S			0
#define	REG07_BATFET_DLY_10S		1

#define	REG07_BATFET_RST_EN_MASK	0x04
#define	REG07_BATFET_RST_EN_SHIFT	2
#define	REG07_BATFET_RST_DISABLE	0
#define	REG07_BATFET_RST_ENABLE		1

#define	REG07_VDPM_BAT_TRACK_MASK	0x03
#define	REG07_VDPM_BAT_TRACK_SHIFT 	0
#define	REG07_VDPM_BAT_TRACK_DISABLE	0
#define	REG07_VDPM_BAT_TRACK_200MV	1
#define	REG07_VDPM_BAT_TRACK_250MV	2
#define	REG07_VDPM_BAT_TRACK_300MV	3

/* Register 0x08*/
#define ETA6963_REG_08              0x08
#define REG08_VBUS_STAT_MASK      0xE0
#define REG08_VBUS_STAT_SHIFT     5
#define REG08_VBUS_TYPE_NONE	  0
#define REG08_VBUS_TYPE_SDP	0x01
#define REG08_VBUS_TYPE_CDP	0x02
#define REG08_VBUS_TYPE_DCP	0x03
#define REG08_VBUS_TYPE_UNKNOWN	0x05
#define REG08_VBUS_TYPE_NON_STD	0x06

#define REG08_VBUS_TYPE_USB       1
#define REG08_VBUS_TYPE_ADAPTER   3
#define REG08_VBUS_TYPE_OTG       7

#define REG08_CHRG_STAT_MASK      0x18
#define REG08_CHRG_STAT_SHIFT     3
#define REG08_CHRG_STAT_IDLE      0
#define REG08_CHRG_STAT_PRECHG    1
#define REG08_CHRG_STAT_FASTCHG   2
#define REG08_CHRG_STAT_CHGDONE   3

#define REG08_PG_STAT_MASK        0x04
#define REG08_PG_STAT_SHIFT       2
#define REG08_POWER_GOOD          1

#define REG08_THERM_STAT_MASK     0x02
#define REG08_THERM_STAT_SHIFT    1

#define REG08_VSYS_STAT_MASK      0x01
#define REG08_VSYS_STAT_SHIFT     0
#define REG08_IN_VSYS_STAT        1


/* Register 0x09*/
#define ETA6963_REG_09              0x09
#define REG09_FAULT_WDT_MASK      0x80
#define REG09_FAULT_WDT_SHIFT     7
#define REG09_FAULT_WDT           1

#define REG09_FAULT_BOOST_MASK    0x40
#define REG09_FAULT_BOOST_SHIFT   6

#define REG09_FAULT_CHRG_MASK     0x30
#define REG09_FAULT_CHRG_SHIFT    4
#define REG09_FAULT_CHRG_NORMAL   0
#define REG09_FAULT_CHRG_INPUT    1
#define REG09_FAULT_CHRG_THERMAL  2
#define REG09_FAULT_CHRG_TIMER    3

#define REG09_FAULT_BAT_MASK      0x08
#define REG09_FAULT_BAT_SHIFT     3
#define	REG09_FAULT_BAT_OVP		1

#define REG09_FAULT_NTC_MASK      0x07
#define REG09_FAULT_NTC_SHIFT     0
#define	REG09_FAULT_NTC_NORMAL		0
#define REG09_FAULT_NTC_WARM      2
#define REG09_FAULT_NTC_COOL      3
#define REG09_FAULT_NTC_COLD      5
#define REG09_FAULT_NTC_HOT       6


/* Register 0x0A */
#define ETA6963_REG_0A              0x0A
#define	REG0A_VBUS_GD_MASK			0x80
#define	REG0A_VBUS_GD_SHIFT			7
#define	REG0A_VBUS_GD				1

#define	REG0A_VINDPM_STAT_MASK		0x40
#define	REG0A_VINDPM_STAT_SHIFT		6
#define	REG0A_VINDPM_ACTIVE			1

#define	REG0A_IINDPM_STAT_MASK		0x20
#define	REG0A_IINDPM_STAT_SHIFT		5
#define	REG0A_IINDPM_ACTIVE			1

#define	REG0A_TOPOFF_ACTIVE_MASK	0x08
#define	REG0A_TOPOFF_ACTIVE_SHIFT	3
#define	REG0A_TOPOFF_ACTIVE			1

#define	REG0A_ACOV_STAT_MASK		0x04
#define	REG0A_ACOV_STAT_SHIFT		2
#define	REG0A_ACOV_ACTIVE			1

#define	REG0A_VINDPM_INT_MASK		0x02
#define	REG0A_VINDPM_INT_SHIFT		1
#define	REG0A_VINDPM_INT_ENABLE		0
#define	REG0A_VINDPM_INT_DISABLE	1

#define	REG0A_IINDPM_INT_MASK		0x01
#define	REG0A_IINDPM_INT_SHIFT		0
#define	REG0A_IINDPM_INT_ENABLE		0
#define	REG0A_IINDPM_INT_DISABLE	1

#define	REG0A_INT_MASK_MASK			0x03
#define	REG0A_INT_MASK_SHIFT		0


#define	ETA6963_REG_0B				0x0B
#define	REG0B_REG_RESET_MASK		0x80
#define	REG0B_REG_RESET_SHIFT		7
#define	REG0B_REG_RESET				1

#define REG0B_PN_MASK             	0x78
#define REG0B_PN_SHIFT            	3

#define REG0B_ETA_ID_MASK			0x04
#define REG0B_ETA_ID_SHIFT			2

#define REG0B_DEV_REV_MASK        	0x03
#define REG0B_DEV_REV_SHIFT       	0

struct eta6963_charge_param {
	int vlim;
	int ilim;
	int ichg;
	int vreg;
};

enum stat_ctrl {
	STAT_CTRL_STAT,
	STAT_CTRL_ICHG,
	STAT_CTRL_INDPM,
	STAT_CTRL_DISABLE,
};

enum vboost {
	BOOSTV_4850 = 4850,
	BOOSTV_5000 = 5000,
	BOOSTV_5150 = 5150,
	BOOSTV_5300 = 5300,
};

enum iboost {
	BOOSTI_500 = 500,
	BOOSTI_1200 = 1200,
};

enum vac_ovp {
	VAC_OVP_5500 = 5500,
	VAC_OVP_6500 = 6500,
	VAC_OVP_10500 = 10500,
	VAC_OVP_14000 = 14000,
};


struct eta6963_platform_data {
	struct eta6963_charge_param usb;
	int iprechg;
	int iterm;
	enum stat_ctrl statctrl;
	enum vboost boostv;	// options are 4850,
	enum iboost boosti; // options are 500mA, 1200mA
	enum vac_ovp vac_ovp;
};

enum {
	PN_BQ25601,
	PN_ETA6963,
};

enum eta6963_part_no {
	BQ25601 = 0x02,
	ETA6963 = 0x07,
};

static int pn_data[] = {
	[PN_BQ25601] = 0x02,
	[PN_ETA6963] = 0x07,
};

static char *pn_str[] = {
	[PN_BQ25601] = "bq25601",
	[PN_ETA6963] = "eta6963",
};

enum charge_chip {
	CHARGE_UNKNOWN,
	CHARGE_BQ25601,
	CHARGE_ETA6963,
};

static char *charge_chip_name[] = {
	"unknown",
	"bq25601",
	"eta6963",
};

struct eta6963_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct eta6963_info {
	struct device *dev;
	struct i2c_client *client;

	enum charge_chip chip;
	enum eta6963_part_no part_no;
	int revision;

	const char *chg_dev_name;
	const char *eint_name;

	int status;
	int irq;

	struct mutex i2c_rw_lock;
	struct mutex gpio_access_lock;

	bool power_good;
	atomic_t is_chip_en;

	struct eta6963_platform_data *platform_data;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;

	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;

	bool  slave_mode;
	int enable_pin;

	struct alarm wdt_timer;
	struct workqueue_struct *wdt_workq;
	struct work_struct kick_work;
	struct delayed_work chg_stat_work;
	unsigned int polling_interval;
	bool polling_enabled;
	struct mutex			wdt_lock;
	struct mutex			irq_complete;
	bool				resume_completed;
	bool				irq_waiting;
	bool				irq_disabled;

	struct eta6963_regulator	otg_vreg;
};

static int __eta6963_read_reg(struct eta6963_info *info, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(info->client, reg);
	if (ret < 0) {
		dev_err(info->dev, "i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __eta6963_write_reg(struct eta6963_info *info, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(info->client, reg, val);
	if (ret < 0) {
		dev_err(info->dev, "i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	return 0;
}

static inline bool __eta6963_is_chip_en(struct eta6963_info *info)
{
	int en = 0;
	bool is_enabled = 0;

	en = gpio_get_value(info->enable_pin);
	if(0 == en)
		is_enabled = 1;
	else
		is_enabled = 0;
    pr_err("%s info->enable_pin = %d, is_enabled=%d\n", __func__, en,is_enabled);
	if ((is_enabled && !atomic_read(&info->is_chip_en)) ||
			(!is_enabled && atomic_read(&info->is_chip_en)))
			dev_notice(info->dev, "%s: en not sync(%d, %d)\n", __func__, is_enabled,
			atomic_read(&info->is_chip_en));
	return is_enabled;
}

static int eta6963_read_byte(struct eta6963_info *info, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&info->i2c_rw_lock);
	mutex_lock(&info->gpio_access_lock);
	ret = __eta6963_read_reg(info, reg, data);
	mutex_unlock(&info->gpio_access_lock);
	mutex_unlock(&info->i2c_rw_lock);

	return ret;
}

static int eta6963_write_byte(struct eta6963_info *info, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&info->i2c_rw_lock);
	mutex_lock(&info->gpio_access_lock);
	if (__eta6963_is_chip_en(info))
		ret = __eta6963_write_reg(info, reg, data);
	else
		ret = -EINVAL;
	mutex_unlock(&info->gpio_access_lock);
	mutex_unlock(&info->i2c_rw_lock);

	if (ret)
		dev_err(info->dev, "Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}

static int eta6963_update_bits(struct eta6963_info *info, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&info->i2c_rw_lock);
	mutex_lock(&info->gpio_access_lock);
	ret = __eta6963_read_reg(info, reg, &tmp);
	if (ret) {
		dev_err(info->dev, "Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __eta6963_write_reg(info, reg, tmp);
	if (ret)
		dev_err(info->dev, "Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&info->gpio_access_lock);
	mutex_unlock(&info->i2c_rw_lock);
	return ret;
}



static int eta6963_enable_otg(struct eta6963_info *info)
{
	u8 val = REG01_OTG_ENABLE << REG01_OTG_CONFIG_SHIFT;

	return eta6963_update_bits(info, ETA6963_REG_01, REG01_OTG_CONFIG_MASK,
				   val);

}

static int eta6963_disable_otg(struct eta6963_info *info)
{
	u8 val = REG01_OTG_DISABLE << REG01_OTG_CONFIG_SHIFT;

	return eta6963_update_bits(info, ETA6963_REG_01, REG01_OTG_CONFIG_MASK,
				   val);

}

static int eta6963_enable_charger(struct eta6963_info *info)
{
	int ret;
	u8 val = REG01_CHG_ENABLE << REG01_CHG_CONFIG_SHIFT;

	ret =
	    eta6963_update_bits(info, ETA6963_REG_01, REG01_CHG_CONFIG_MASK, val);

	return ret;
}

static int eta6963_disable_charger(struct eta6963_info *info)
{
	int ret;
	u8 val = REG01_CHG_DISABLE << REG01_CHG_CONFIG_SHIFT;

	ret =
	    eta6963_update_bits(info, ETA6963_REG_01, REG01_CHG_CONFIG_MASK, val);
	return ret;
}

static int eta6963_set_chargecurrent(struct eta6963_info *info, int curr)
{
	u8 ichg;

	if (curr < REG02_ICHG_BASE)
		curr = REG02_ICHG_BASE;

	ichg = (curr - REG02_ICHG_BASE) / REG02_ICHG_LSB;
	return eta6963_update_bits(info, ETA6963_REG_02, REG02_ICHG_MASK,
				   ichg << REG02_ICHG_SHIFT);

}

static int eta6963_set_term_current(struct eta6963_info *info, int curr)
{
	u8 iterm;

	if (curr < REG03_ITERM_BASE)
		curr = REG03_ITERM_BASE;

	iterm = (curr - REG03_ITERM_BASE) / REG03_ITERM_LSB;

	return eta6963_update_bits(info, ETA6963_REG_03, REG03_ITERM_MASK,
				   iterm << REG03_ITERM_SHIFT);
}

static int eta6963_set_prechg_current(struct eta6963_info *info, int curr)
{
	u8 iprechg;

	if (curr < REG03_IPRECHG_BASE)
		curr = REG03_IPRECHG_BASE;

	iprechg = (curr - REG03_IPRECHG_BASE) / REG03_IPRECHG_LSB;

	return eta6963_update_bits(info, ETA6963_REG_03, REG03_IPRECHG_MASK,
				   iprechg << REG03_IPRECHG_SHIFT);
}

static int eta6963_set_chargevolt(struct eta6963_info *info, int volt)
{
	u8 val;

	if (volt < REG04_VREG_BASE)
		volt = REG04_VREG_BASE;

	val = (volt - REG04_VREG_BASE) / REG04_VREG_LSB;
	return eta6963_update_bits(info, ETA6963_REG_04, REG04_VREG_MASK,
				   val << REG04_VREG_SHIFT);
}

static int eta6963_set_input_volt_limit(struct eta6963_info *info, int volt)
{
	u8 val;

	if (volt < REG06_VINDPM_BASE)
		volt = REG06_VINDPM_BASE;

	val = (volt - REG06_VINDPM_BASE) / REG06_VINDPM_LSB;
	return eta6963_update_bits(info, ETA6963_REG_06, REG06_VINDPM_MASK,
				   val << REG06_VINDPM_SHIFT);
}

static int eta6963_set_input_current_limit(struct eta6963_info *info, int curr)
{
	u8 val;

	if (curr < REG00_IINLIM_BASE)
		curr = REG00_IINLIM_BASE;

	val = (curr - REG00_IINLIM_BASE) / REG00_IINLIM_LSB;
	return eta6963_update_bits(info, ETA6963_REG_00, REG00_IINLIM_MASK,
				   val << REG00_IINLIM_SHIFT);
}

static int eta6963_set_watchdog_timer(struct eta6963_info *info, u8 timeout)
{
	u8 temp;

	temp = (u8) (((timeout -
		       REG05_WDT_BASE) / REG05_WDT_LSB) << REG05_WDT_SHIFT);

	return eta6963_update_bits(info, ETA6963_REG_05, REG05_WDT_MASK, temp);
}

static int eta6963_disable_watchdog_timer(struct eta6963_info *info)
{
	u8 val = REG05_WDT_DISABLE << REG05_WDT_SHIFT;

	return eta6963_update_bits(info, ETA6963_REG_05, REG05_WDT_MASK, val);
}

static int eta6963_reset_watchdog_timer(struct eta6963_info *info)
{
	u8 val = REG01_WDT_RESET << REG01_WDT_RESET_SHIFT;

	return eta6963_update_bits(info, ETA6963_REG_01, REG01_WDT_RESET_MASK,
				   val);
}

static int __maybe_unused eta6963_reset_chip(struct eta6963_info *info)
{
	int ret;
	u8 val = REG0B_REG_RESET << REG0B_REG_RESET_SHIFT;

	ret =
	    eta6963_update_bits(info, ETA6963_REG_0B, REG0B_REG_RESET_MASK, val);
	return ret;
}

static int eta6963_enter_hiz_mode(struct eta6963_info *info)
{
	u8 val = REG00_HIZ_ENABLE << REG00_ENHIZ_SHIFT;

	return eta6963_update_bits(info, ETA6963_REG_00, REG00_ENHIZ_MASK, val);

}

static int eta6963_exit_hiz_mode(struct eta6963_info *info)
{

	u8 val = REG00_HIZ_DISABLE << REG00_ENHIZ_SHIFT;

	return eta6963_update_bits(info, ETA6963_REG_00, REG00_ENHIZ_MASK, val);

}

static int eta6963_get_hiz_mode(struct eta6963_info *info, u8 *state)
{
	u8 val;
	int ret;

	ret = eta6963_read_byte(info, ETA6963_REG_00, &val);
	if (ret)
		return ret;
	*state = (val & REG00_ENHIZ_MASK) >> REG00_ENHIZ_SHIFT;

	return 0;
}

static int eta6963_enable_term(struct eta6963_info *info, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = REG05_TERM_ENABLE << REG05_EN_TERM_SHIFT;
	else
		val = REG05_TERM_DISABLE << REG05_EN_TERM_SHIFT;

	ret = eta6963_update_bits(info, ETA6963_REG_05, REG05_EN_TERM_MASK, val);

	return ret;
}

static int eta6963_set_boost_current(struct eta6963_info *info, int curr)
{
	u8 val;

	val = REG02_BOOST_LIM_0P5A;
	if (curr == BOOSTI_1200)
		val = REG02_BOOST_LIM_1P2A;

	return eta6963_update_bits(info, ETA6963_REG_02, REG02_BOOST_LIM_MASK,
				   val << REG02_BOOST_LIM_SHIFT);
}

static int eta6963_set_boost_voltage(struct eta6963_info *info, int volt)
{
	u8 val;

	if (volt == BOOSTV_4850)
		val = REG06_BOOSTV_4P85V;
	else if (volt == BOOSTV_5150)
		val = REG06_BOOSTV_5P15V;
	else if (volt == BOOSTV_5300)
		val = REG06_BOOSTV_5P3V;
	else
		val = REG06_BOOSTV_5V;

	return eta6963_update_bits(info, ETA6963_REG_06, REG06_BOOSTV_MASK,
				   val << REG06_BOOSTV_SHIFT);
}

static int eta6963_set_acovp_threshold(struct eta6963_info *info, int volt)
{
	u8 val;

	if (volt == VAC_OVP_14000)
		val = REG06_OVP_14P0V;
	else if (volt == VAC_OVP_10500)
		val = REG06_OVP_10P5V;
	else if (volt == VAC_OVP_6500)
		val = REG06_OVP_6P5V;
	else
		val = REG06_OVP_5P5V;

	return eta6963_update_bits(info, ETA6963_REG_06, REG06_OVP_MASK,
				   val << REG06_OVP_SHIFT);
}

static int eta6963_set_stat_ctrl(struct eta6963_info *info, int ctrl)
{
	u8 val;

	val = ctrl;

	return eta6963_update_bits(info, ETA6963_REG_00, REG00_STAT_CTRL_MASK,
				   val << REG00_STAT_CTRL_SHIFT);
}

static int eta6963_set_int_mask(struct eta6963_info *info, int mask)
{
	u8 val;

	val = mask;

	return eta6963_update_bits(info, ETA6963_REG_0A, REG0A_INT_MASK_MASK,
				   val << REG0A_INT_MASK_SHIFT);
}

static int __maybe_unused eta6963_enable_batfet(struct eta6963_info *info)
{
	const u8 val = REG07_BATFET_ON << REG07_BATFET_DIS_SHIFT;

	return eta6963_update_bits(info, ETA6963_REG_07, REG07_BATFET_DIS_MASK,
				   val);
}

static int __maybe_unused eta6963_disable_batfet(struct eta6963_info *info)
{
	const u8 val = REG07_BATFET_OFF << REG07_BATFET_DIS_SHIFT;

	return eta6963_update_bits(info, ETA6963_REG_07, REG07_BATFET_DIS_MASK,
				   val);
}

static int __maybe_unused eta6963_set_batfet_delay(struct eta6963_info *info, uint8_t delay)
{
	u8 val;

	if (delay == 0)
		val = REG07_BATFET_DLY_0S;
	else
		val = REG07_BATFET_DLY_10S;

	val <<= REG07_BATFET_DLY_SHIFT;

	return eta6963_update_bits(info, ETA6963_REG_07, REG07_BATFET_DLY_MASK,
				   val);
}

static int eta6963_enable_safety_timer(struct eta6963_info *info)
{
	const u8 val = REG05_CHG_TIMER_ENABLE << REG05_EN_TIMER_SHIFT;

	return eta6963_update_bits(info, ETA6963_REG_05, REG05_EN_TIMER_MASK,
				   val);
}

static int eta6963_disable_safety_timer(struct eta6963_info *info)
{
	const u8 val = REG05_CHG_TIMER_DISABLE << REG05_EN_TIMER_SHIFT;

	return eta6963_update_bits(info, ETA6963_REG_05, REG05_EN_TIMER_MASK,
				   val);
}

static void enable_wdt_polling(struct eta6963_info *info, bool poll_en)
{
	struct timespec time, time_now, end_time;
	ktime_t ktime;
	int ret = 0;

	if (info) {
		mutex_lock(&info->wdt_lock);
		if (info->polling_enabled != poll_en) {
			if (poll_en) {
				get_monotonic_boottime(&time_now);
				time.tv_sec = info->polling_interval;
				time.tv_nsec = 0;
				end_time = timespec_add(time_now, time);
				ktime = ktime_set(end_time.tv_sec, end_time.tv_nsec);
				alarm_start(&info->wdt_timer, ktime);
				info->polling_enabled = true;
				ret = eta6963_reset_watchdog_timer(info);
				ret |= eta6963_set_watchdog_timer(info, 40);
				if (ret) {
					//gpio_set_value(info->enable_pin, 1);
					dev_err(info->dev, "%s:reset/set watchdog failed,ret=%d\n", __func__, ret);
				}
			} else {
				ret = eta6963_disable_watchdog_timer(info);
				if (ret)
					dev_err(info->dev, "Failed to disable charging,ret=%d\n", ret);
				info->polling_enabled = false;
				alarm_cancel(&info->wdt_timer);
			}
		}
		mutex_unlock(&info->wdt_lock);
	}
}

static void wdt_kick_work(struct work_struct *work)
{
	int ret;
	ktime_t ktime;
	struct timespec time, time_now, end_time;
	struct eta6963_info *info =
		container_of(work, struct eta6963_info, kick_work);

	msleep(100); /*avoid i2c errors(i2c-5: err, access at suspend no irq stage) */
	ret = eta6963_reset_watchdog_timer(info);
	if (ret) {
		//gpio_set_value(info->enable_pin, 1);
		dev_err(info->dev, "%s:kick watchdog failed,disalbe charge via enable_pin\n", __func__);
		return ;
	}

	if (info->polling_enabled == true) {
		get_monotonic_boottime(&time_now);
		time.tv_sec = info->polling_interval;
		time.tv_nsec = 0;
		end_time = timespec_add(time_now, time);
		ktime = ktime_set(end_time.tv_sec, end_time.tv_nsec);
		alarm_start(&info->wdt_timer, ktime);
	}
	pm_relax(info->dev);
}

static enum alarmtimer_restart wdt_timer_func(struct alarm *alarm,
						 ktime_t now)
{
	struct eta6963_info *info =
		container_of(alarm, struct eta6963_info, wdt_timer);

	pm_stay_awake(info->dev);
	queue_work(info->wdt_workq, &info->kick_work);

	return ALARMTIMER_NORESTART;
}

static struct eta6963_platform_data *eta6963_parse_dt(struct device_node *np,
						      struct eta6963_info *info)
{
	int ret;
	struct eta6963_platform_data *pdata;

	pdata = devm_kzalloc(info->dev, sizeof(struct eta6963_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return NULL;

	info->enable_pin =
		of_get_named_gpio(info->dev->of_node, "eta6963_en_gpio", 0);
	if(info->enable_pin < 0){
		dev_err(info->dev, "%s: no gpio_eta6963_en\n", __func__);
		info->enable_pin = -1;
	}

	if (of_property_read_string(np, "charger_name", &info->chg_dev_name) < 0) {
		info->chg_dev_name = "primary_chg";
		pr_warn("no charger name\n");
	}

	if (info->chg_dev_name
		&& !strncmp(info->chg_dev_name,
				"secondary_chg", strlen("secondary_chg")))
		info->slave_mode = true;
	else
		info->slave_mode = false;

	if (of_property_read_string(np, "alias_name", &(info->chg_props.alias_name)) < 0) {
		if (info->slave_mode)
			info->chg_props.alias_name = "eta6963-slave";
		else
			info->chg_props.alias_name = "eta6963-master";
		pr_warn("no alias name\n");
	}
	if (of_property_read_string(np, "eint_name", &info->eint_name) < 0) {
		info->eint_name = "chr_stat";
		pr_warn("no eint name\n");
	}

	ret = of_property_read_u32(np, "usb-vlim", &pdata->usb.vlim);
	if (ret) {
		pdata->usb.vlim = 4500;
		dev_err(info->dev, "Failed to read node of usb-vlim\n");
	}

	ret = of_property_read_u32(np, "usb-ilim", &pdata->usb.ilim);
	if (ret) {
		pdata->usb.ilim = 2000;
		dev_err(info->dev, "Failed to read node of usb-ilim\n");
	}

	ret = of_property_read_u32(np, "usb-vreg", &pdata->usb.vreg);
	if (ret) {
		pdata->usb.vreg = 4200;
		dev_err(info->dev, "Failed to read node of usb-vreg\n");
	}

	ret = of_property_read_u32(np, "usb-ichg", &pdata->usb.ichg);
	if (ret) {
		pdata->usb.ichg = 2000;
		dev_err(info->dev, "Failed to read node of usb-ichg\n");
	}

	ret = of_property_read_u32(np, "stat-pin-ctrl",
				   &pdata->statctrl);
	if (ret) {
		pdata->statctrl = 0;
		dev_err(info->dev, "Failed to read node of stat-pin-ctrl\n");
	}

	ret = of_property_read_u32(np, "precharge-current",
				   &pdata->iprechg);
	if (ret) {
		pdata->iprechg = 180;
		dev_err(info->dev, "Failed to read node of precharge-current\n");
	}

	ret = of_property_read_u32(np, "termination-current",
				   &pdata->iterm);
	if (ret) {
		pdata->iterm = 180;
		pr_err
		    ("Failed to read node of termination-current\n");
	}

	ret = of_property_read_u32(np, "boost-voltage",
				 &pdata->boostv);
	if (ret) {
		pdata->boostv = 5000;
		dev_err(info->dev, "Failed to read node of boost-voltage\n");
	}

	ret =
	    of_property_read_u32(np, "boost-current",
				 &pdata->boosti);
	if (ret) {
		pdata->boosti = 1200;
		dev_err(info->dev, "Failed to read node of boost-current\n");
	}

	ret = of_property_read_u32(np, "vac-ovp-threshold",
				   &pdata->vac_ovp);
	if (ret) {
		pdata->vac_ovp = 6500;
		dev_err(info->dev, "Failed to read node of vac-ovp-threshold\n");
	}

	return pdata;
}

static void chg_stat_work_handler(struct work_struct *data)
{
	struct eta6963_info *info = NULL;
	static u8 last_stat = 0xFF;
	u8 curr_stat = 0;
	int ret = 0;

	pr_info("%s\n", __func__);
	info = container_of(to_delayed_work(data),
					struct eta6963_info, chg_stat_work);
	if (IS_ERR_OR_NULL(info)) {
		pr_err("[%s]: NULL pointer\n", __func__);
		return;
	};

	ret = eta6963_read_byte(info, ETA6963_REG_08, &curr_stat);
	if (!ret) {
		curr_stat = curr_stat & REG08_CHRG_STAT_MASK;
		curr_stat = curr_stat >> REG08_CHRG_STAT_SHIFT;

		if (curr_stat != last_stat) {
			dev_info(info->dev, "[%s]: %d -> %d\n", __func__,
					last_stat, curr_stat);
			last_stat = curr_stat;
			power_supply_changed(info->psy);
		}
	}
}

static irqreturn_t eta6963_irq_handler(int irq, void *data)
{
	int ret;
	u8 reg_val;
	bool prev_pg;
	struct eta6963_info *info = (struct eta6963_info *)data;

	pm_stay_awake(info->dev);
	mutex_lock(&info->irq_complete);

	if (info->slave_mode)
		goto handled;

	info->irq_waiting = true;
	if (!info->resume_completed) {
		dev_dbg(info->dev, "IRQ triggered before device-resume\n");
		if (!info->irq_disabled) {
			disable_irq_nosync(irq);
			info->irq_disabled = true;
		}
		goto handled;
	}
	info->irq_waiting = false;

	ret = eta6963_read_byte(info, ETA6963_REG_08, &reg_val);
	if (ret)
		goto handled;

	prev_pg = info->power_good;

	info->power_good = !!(reg_val & REG08_PG_STAT_MASK);

	if (!prev_pg && info->power_good) {
		pr_notice("adapter/usb inserted\n");
		schedule_delayed_work(&info->chg_stat_work,
				msecs_to_jiffies(500));
	} else if (prev_pg && !info->power_good) {
		pr_notice("adapter/usb removed\n");
		schedule_delayed_work(&info->chg_stat_work,
				msecs_to_jiffies(200));
	}

handled:
	mutex_unlock(&info->irq_complete);
	pm_relax(info->dev);
	return IRQ_HANDLED;
}

static int eta6963_register_interrupt(struct eta6963_info *info)
{
	int ret = 0;
	/*struct device_node *np;

	np = of_find_node_by_name(NULL, info->eint_name);
	if (np) {
		info->irq = irq_of_parse_and_map(np, 0);
	} else {
		dev_err(info->dev, "couldn't get irq node\n");
		return -ENODEV;
	}
	dev_info(info->dev, "irq = %d\n", info->irq);*/

	if (!info->client->irq) {
		dev_info(info->dev, "info->client->irq is NULL\n");//remember to config dws
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(info->dev, info->client->irq, NULL,
					eta6963_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"eta_irq", info);
	if (ret < 0) {
		dev_err(info->dev, "request thread irq failed:%d\n", ret);
		return ret;
	}

	enable_irq_wake(info->irq);

	return 0;
}

static int __eta6963_enable_chip(struct eta6963_info *info, bool en)
{
	bool is_chip_en = false;

	dev_info(info->dev, "%s: en = %d\n", __func__, en);
	mutex_lock(&info->gpio_access_lock);
	is_chip_en = __eta6963_is_chip_en(info);
	if (en && !is_chip_en) {
		if (info->enable_pin > 0) {
			gpio_set_value(info->enable_pin, 0);
			dev_info(info->dev, "%s: set gpio low\n", __func__);
		} else {
			dev_info(info->dev, "%s: info->enable_pin is invalid\n", __func__);
		}
	} else if (!en && is_chip_en) {
		if (info->enable_pin > 0) {
			gpio_set_value(info->enable_pin, 1);
			dev_info(info->dev, "%s: set gpio high\n", __func__);
		} else {
			dev_info(info->dev, "%s: info->enable_pin is invalid\n", __func__);
		}
	}

	/* Wait for chip's enable/disable */
	mdelay(1);
	atomic_set(&info->is_chip_en, en);
	mutex_unlock(&info->gpio_access_lock);

	return 0;
}

static int eta6963_init_device(struct eta6963_info *info)
{
	int ret;

	eta6963_reset_watchdog_timer(info);
	if (info->slave_mode) {
		eta6963_disable_watchdog_timer(info); /*default disable wdt for slave charge if not plugin charger*/
		eta6963_disable_charger(info); /* sw control disable chg */
		//eta6963_enter_hiz_mode(info);  /* enable Hiz mode */
	}

	eta6963_enable_term(info, true);

	if (info->enable_pin > 0)
		gpio_set_value(info->enable_pin, 0); /* hw control enable chg */

	ret = eta6963_set_input_volt_limit(info, info->platform_data->usb.vlim);
	if (ret)
		dev_err(info->dev, "Failed to set usb input voltage limit, ret = %d\n", ret);

	ret = eta6963_set_input_current_limit(info, info->platform_data->usb.ilim);
	if (ret)
		dev_err(info->dev, "Failed to set usb input current limit, ret = %d\n", ret);

	ret = eta6963_set_chargevolt(info, info->platform_data->usb.vreg);
	if (ret)
		dev_err(info->dev, "Failed to set battery cv, ret = %d\n", ret);

	ret = eta6963_set_chargecurrent(info, info->platform_data->usb.ichg);
	if (ret)
		dev_err(info->dev, "Failed to set charge current, ret = %d\n", ret);

	ret = eta6963_set_stat_ctrl(info, info->platform_data->statctrl);
	if (ret)
		dev_err(info->dev, "Failed to set stat pin control mode, ret = %d\n", ret);

	ret = eta6963_set_prechg_current(info, info->platform_data->iprechg);
	if (ret)
		dev_err(info->dev, "Failed to set prechg current, ret = %d\n", ret);

	ret = eta6963_set_term_current(info, info->platform_data->iterm);
	if (ret)
		dev_err(info->dev, "Failed to set termination current, ret = %d\n", ret);

	ret = eta6963_set_boost_voltage(info, info->platform_data->boostv);
	if (ret)
		dev_err(info->dev, "Failed to set boost voltage, ret = %d\n", ret);

	ret = eta6963_set_boost_current(info, info->platform_data->boosti);
	if (ret)
		dev_err(info->dev, "Failed to set boost current, ret = %d\n", ret);

	ret = eta6963_set_acovp_threshold(info, info->platform_data->vac_ovp);
	if (ret)
		dev_err(info->dev, "Failed to set acovp threshold, ret = %d\n", ret);

	ret = eta6963_set_int_mask(info,
				   REG0A_IINDPM_INT_MASK |
				   REG0A_VINDPM_INT_MASK);
	if (ret)
		dev_err(info->dev, "Failed to set vindpm and iindpm int mask\n");

	/* [BSP]Begin added by bitao.xiong for BORANAOM-1968 on 2022/11/23 */
	#if defined(TARGET_BUILD_MMITEST)
	ret = eta6963_disable_safety_timer(info);
	if (ret)
		dev_err(info->dev, "Failed to disable both fastcharge and precharge timer\n");
	#else
	ret = eta6963_enable_safety_timer(info);
	if (ret)
		dev_err(info->dev, "Failed to enable both fastcharge and precharge timer\n");
	#endif
	ret = eta6963_disable_watchdog_timer(info);
	if (ret)
		dev_err(info->dev, "Failed to disable watchdog timer\n");
	/* [BSP]End added by bitao.xiong for BORANAOM-1968 on 2022/11/23 */
	return ret;
}

static void determine_initial_status(struct eta6963_info *info)
{
	eta6963_irq_handler(info->irq, (void *) info);
}

static int eta6963_detect_device(struct eta6963_info *info)
{
	int ret;
	u8 data;

	ret = eta6963_read_byte(info, ETA6963_REG_0B, &data);
	if (ret) {
		info->chip = CHARGE_UNKNOWN;
	} else {
		info->part_no = (data & REG0B_PN_MASK) >> REG0B_PN_SHIFT;
		info->revision =
		    (data & REG0B_DEV_REV_MASK) >> REG0B_DEV_REV_SHIFT;

		switch (info->part_no) {
		case ETA6963:
			if ((data & REG0B_ETA_ID_MASK) >> REG0B_ETA_ID_SHIFT)
				info->chip = CHARGE_ETA6963;
			else
				info->chip = CHARGE_UNKNOWN;
			break;
		case BQ25601:
			info->chip = CHARGE_BQ25601;
			break;
		default:
			info->chip = CHARGE_UNKNOWN;
		}
	}

	return ret;
}

static void eta6963_dump_regs(struct eta6963_info *info)
{
	int addr;
	u8 val;
	int ret;

	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = eta6963_read_byte(info, addr, &val);
		if (ret == 0)
			dev_err(info->dev, "Reg[%.2x] = 0x%.2x\n", addr, val);
	}
}

static ssize_t
eta6963_show_registers(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct eta6963_info *info = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "eta6963 Reg");
	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = eta6963_read_byte(info, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
				       "Reg[%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t
eta6963_store_registers(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct eta6963_info *info = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg < 0x0B) {
		eta6963_write_byte(info, (unsigned char) reg,
				   (unsigned char) val);
	}

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, eta6963_show_registers,
		   eta6963_store_registers);

static struct attribute *eta6963_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group eta6963_attr_group = {
	.attrs = eta6963_attributes,
};

static int eta6963_charging(struct charger_device *chg_dev, bool enable)
{

	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 val;

	/*Begin: modify for SNTBBH-5726 by wanglin.chen on 2023.04.07*/
	if (info->slave_mode) {
		if (enable) {
			eta6963_exit_hiz_mode(info);
		} else {
			eta6963_enter_hiz_mode(info);
		}
	}
	/*End: modify for SNTBBH-5726 by wanglin.chen on 2023.04.07*/
	if (enable) {
		ret = eta6963_enable_charger(info);
        } else {
		ret = eta6963_disable_charger(info);
        }
	dev_err(info->dev, "%s charger %s\n", enable ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	return ret;
}

static int eta6963_enable_chip(struct charger_device *chg_dev, bool en)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);

	__eta6963_enable_chip(info, en);
	enable_wdt_polling(info, en);
	dev_err(info->dev, "%s: %s eta6963 chip\n", __func__, en ? "enable" : "disable");

	return 0;
}

static int eta6963_is_chip_enabled(struct charger_device *chg_dev, bool *en)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	mutex_lock(&info->gpio_access_lock);
	*en = __eta6963_is_chip_en(info);
	mutex_unlock(&info->gpio_access_lock);
	return 0;
}

static int eta6963_plug_in(struct charger_device *chg_dev)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);

	enable_wdt_polling(info, true);
	return 0;
}

static int eta6963_plug_out(struct charger_device *chg_dev)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);

	enable_wdt_polling(info, false);
	return 0;
}

static int eta6963_dump_register(struct charger_device *chg_dev)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);

	eta6963_dump_regs(info);

	return 0;
}

static int eta6963_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	u8 val;
	int ret = 0;

	ret = eta6963_read_byte(info, ETA6963_REG_01, &val);
	if (!ret)
		*en = (val & REG01_CHG_CONFIG_MASK) >> REG01_CHG_CONFIG_SHIFT;

	return ret;
}

static int eta6963_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 val;

	ret = eta6963_read_byte(info, ETA6963_REG_08, &val);
	if (!ret) {
		val = val & REG08_CHRG_STAT_MASK;
		val = val >> REG08_CHRG_STAT_SHIFT;
		*done = (val == REG08_CHRG_STAT_CHGDONE);
	}

	return ret;
}

static int eta6963_set_ichg(struct charger_device *chg_dev, u32 curr)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);

	dev_err(info->dev, "charge curr = %d\n", curr);

	return eta6963_set_chargecurrent(info, curr / 1000);
}

static int eta6963_get_ichg(struct charger_device *chg_dev, u32 *curr)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int ichg;
	int ret;

	ret = eta6963_read_byte(info, ETA6963_REG_02, &reg_val);
	if (!ret) {
		ichg = (reg_val & REG02_ICHG_MASK) >> REG02_ICHG_SHIFT;
		ichg = ichg * REG02_ICHG_LSB + REG02_ICHG_BASE;
		*curr = ichg * 1000;
	}

	return ret;
}

static int eta6963_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	*curr = 60 * 1000;

	return 0;
}

static int eta6963_set_vchg(struct charger_device *chg_dev, u32 volt)
{
	struct eta6963_info*info = dev_get_drvdata(&chg_dev->dev);

	dev_err(info->dev, "charge volt = %d\n", volt);

	return eta6963_set_chargevolt(info, volt / 1000);
}

static int eta6963_get_vchg(struct charger_device *chg_dev, u32 *volt)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int vchg;
	int ret;

	ret = eta6963_read_byte(info, ETA6963_REG_04, &reg_val);
	if (!ret) {
		vchg = (reg_val & REG04_VREG_MASK) >> REG04_VREG_SHIFT;
		vchg = vchg * REG04_VREG_LSB + REG04_VREG_BASE;
		*volt = vchg * 1000;
	}

	return ret;
}

static int eta6963_set_ivl(struct charger_device *chg_dev, u32 volt)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);

	dev_err(info->dev, "vindpm volt = %d\n", volt);

	return eta6963_set_input_volt_limit(info, volt / 1000);

}

static int eta6963_set_icl(struct charger_device *chg_dev, u32 curr)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);

	dev_err(info->dev, "indpm curr = %d\n", curr);

	return eta6963_set_input_current_limit(info, curr / 1000);
}

static int eta6963_get_icl(struct charger_device *chg_dev, u32 *curr)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int icl;
	int ret;

	ret = eta6963_read_byte(info, ETA6963_REG_00, &reg_val);
	if (!ret) {
		icl = (reg_val & REG00_IINLIM_MASK) >> REG00_IINLIM_SHIFT;
		icl = icl * REG00_IINLIM_LSB + REG00_IINLIM_BASE;
		*curr = icl * 1000;
	}

	return ret;

}

static int eta6963_kick_wdt(struct charger_device *chg_dev)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);

	return eta6963_reset_watchdog_timer(info);
}

static int eta6963_set_otg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);

	if (en) {
		ret = eta6963_enable_otg(info);
	} else {
		ret = eta6963_disable_otg(info);
	}

	if (!ret)
		enable_wdt_polling(info, en);

	dev_err(info->dev, "%s OTG %s\n", en ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	return ret;
}

static int eta6963_set_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	int ret;

	if (en)
		ret = eta6963_enable_safety_timer(info);
	else
		ret = eta6963_disable_safety_timer(info);

	return ret;
}

static int eta6963_is_safety_timer_enabled(struct charger_device *chg_dev,
					   bool *en)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;

	ret = eta6963_read_byte(info, ETA6963_REG_05, &reg_val);

	if (!ret)
		*en = !!(reg_val & REG05_EN_TIMER_MASK);

	return ret;
}

static int eta6963_set_boost_ilmt(struct charger_device *chg_dev, u32 curr)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	int ret;

	dev_err(info->dev, "otg curr = %d\n", curr);

	ret = eta6963_set_boost_current(info, curr / 1000);

	return ret;
}

static int eta6963_set_ta_current_pattern(struct charger_device *chg_dev,
                                          bool is_increase)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	bool increase = is_increase;

	if (increase == true) {
		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_increase() on 1");
		msleep(85);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_increase() off 1");
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_increase() on 2");
		msleep(85);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_increase() off 2");
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_increase() on 3");
		msleep(281);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_increase() off 3");
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_increase() on 4");
		msleep(281);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_increase() off 4");
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_increase() on 5");
		msleep(281);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_increase() off 5");
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_increase() on 6");
		msleep(485);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_increase() off 6");
		msleep(50);

		dev_info(info->dev, "mtk_ta_increase() end\n");

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		msleep(200);
	} else {
		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_decrease() on 1");
		msleep(281);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_decrease() off 1");
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_decrease() on 2");
		msleep(281);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_decrease() off 2");
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_decrease() on 3");
		msleep(281);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_decrease() off 3");
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_decrease() on 4");
		msleep(85);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_decrease() off 4");
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_decrease() on 5");
		msleep(85);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_decrease() off 5");
		msleep(85);

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
		dev_dbg(info->dev, "mtk_ta_decrease() on 6");
		msleep(485);

		eta6963_set_ichg(chg_dev, 60000);  /* 60mA */
		dev_dbg(info->dev, "mtk_ta_decrease() off 6");
		msleep(50);

		dev_info(info->dev, "mtk_ta_decrease() end\n");

		eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
	}

	return 0;
}

static int eta6963_set_pep20_efficiency_table(struct charger_device *chg_dev)
{
#if 0
    struct charger_manager *chg_mgr = NULL;

    chg_mgr = charger_dev_get_drvdata(chg_dev);
    if (!chg_mgr)
        return -EINVAL;

    chg_mgr->pe2.profile[0].vchr = 6500000;
    chg_mgr->pe2.profile[1].vchr = 6500000;
    chg_mgr->pe2.profile[2].vchr = 6500000;
    chg_mgr->pe2.profile[3].vchr = 7000000;
    chg_mgr->pe2.profile[4].vchr = 7000000;
    chg_mgr->pe2.profile[5].vchr = 7000000;
    chg_mgr->pe2.profile[6].vchr = 7000000;
    chg_mgr->pe2.profile[7].vchr = 7000000;
    chg_mgr->pe2.profile[8].vchr = 7500000;
    chg_mgr->pe2.profile[9].vchr = 8000000;
#endif
	return 0;
}

static int eta6963_set_ta_reset(struct charger_device *chg_dev)
{
	eta6963_set_ivl(chg_dev, 4200000); /* set vindpm as 4.2V */
	eta6963_set_ichg(chg_dev, 480000);  /* 480mA */

	eta6963_set_icl(chg_dev, 100000); /* 100mA */
	msleep(250);
	eta6963_set_icl(chg_dev, 700000); /* 700mA */

    return 0;
}

static struct timespec ptime[13];

static int cptime[13][2];

static int dtime(int i)
{
    struct timespec time;

    time = timespec_sub(ptime[i], ptime[i - 1]);
    return time.tv_nsec / 1000000;
}

#define PEOFFTIME 40
#define PEONTIME 90

static int eta6963_set_ta20_current_pattern(struct charger_device *chg_dev,
                                            u32 chr_vol)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);
	int value;
	int i, j = 0;
	int flag;

	eta6963_set_ivl(chg_dev, 4200000); //set vindpm as 4.2V
	eta6963_set_ichg(chg_dev, 480000);  /* 480mA */
	eta6963_charging(chg_dev, true);

	usleep_range(1000, 1200);
	value = (chr_vol - 5500000) / 500000;

	eta6963_set_icl(chg_dev, 100000); /* 100mA */
	msleep(70);

	get_monotonic_boottime(&ptime[j++]);
	for (i = 4; i >= 0; i--) {
		flag = value & (1 << i);

		if (flag == 0) {
			eta6963_set_icl(chg_dev, 700000); /* 700mA */
			msleep(PEOFFTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEOFFTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 30 || cptime[j][1] > 65) {
				dev_info(info->dev, "%s fail1: idx:%d target:%d actual:%d\n",
						__func__, i, PEOFFTIME, cptime[j][1]);
			}
			j++;
			eta6963_set_icl(chg_dev, 100000); /* 100mA */
			msleep(PEONTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEONTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 90 || cptime[j][1] > 115) {
				dev_info(info->dev, "%s fail2: idx:%d target:%d actual:%d\n",
						__func__, i, PEOFFTIME, cptime[j][1]);
			}
			j++;

		} else {
			eta6963_set_icl(chg_dev, 700000); /* 700mA */
			msleep(PEONTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEONTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 90 || cptime[j][1] > 115) {
				dev_info(info->dev, "%s fail3: idx:%d target:%d actual:%d\n",
						__func__, i, PEOFFTIME, cptime[j][1]);
			}
			j++;
			eta6963_set_icl(chg_dev, 100000); /* 100mA */
			msleep(PEOFFTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEOFFTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 30 || cptime[j][1] > 65) {
				dev_info(info->dev, "%s fail4: idx:%d target:%d actual:%d\n",
				__func__, i, PEOFFTIME, cptime[j][1]);
			}
			j++;
		}
	}

	eta6963_set_icl(chg_dev, 700000); /* 700mA */
	msleep(160);
	get_monotonic_boottime(&ptime[j]);
	cptime[j][0] = 160;
	cptime[j][1] = dtime(j);
	if (cptime[j][1] < 150 || cptime[j][1] > 240) {
		dev_info(info->dev, "%s fail5: idx:%d target:%d actual:%d\n",
				__func__, i, PEOFFTIME, cptime[j][1]);
	}
	j++;

	eta6963_set_icl(chg_dev, 100000); /* 100mA */
	msleep(30);
	eta6963_set_icl(chg_dev, 700000); /* 700mA */

	dev_info(info->dev, "%d %d: %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
	        chr_vol, value, cptime[1][0], cptime[2][0], cptime[3][0],
	        cptime[4][0], cptime[5][0], cptime[6][0], cptime[7][0],
	        cptime[8][0], cptime[9][0], cptime[10][0], cptime[11][0]);
	dev_info(info->dev, "%3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
	        cptime[1][1], cptime[2][1], cptime[3][1], cptime[4][1],
	        cptime[5][1], cptime[6][1], cptime[7][1], cptime[8][1],
	        cptime[9][1], cptime[10][1], cptime[11][1]);

	return 0;
}

static int eta6963_enable_cable_drop_comp(struct charger_device *chg_dev, bool en)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);

	dev_err(info->dev, "%s: en = %d\n", __func__, en);
	return eta6963_set_ta20_current_pattern(chg_dev, 21000000);
}

static int eta6963_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct eta6963_info *info = dev_get_drvdata(&chg_dev->dev);

	if (info == NULL)
		return -EINVAL;
	if (chg_dev == NULL)
		return -EINVAL;

	printk("%s: event = %d\n", __func__, event);
	if (!info->slave_mode)
		schedule_delayed_work(&info->chg_stat_work,
					msecs_to_jiffies(200));
	return 0;
}

static struct charger_ops eta6963_chg_ops = {
	/* Normal charging */
	/* cable plug in/out for primary charger */
	.plug_in = eta6963_plug_in,
	.plug_out = eta6963_plug_out,
	.dump_registers = eta6963_dump_register,
	.enable = eta6963_charging,
	.is_enabled = eta6963_is_charging_enable,
	/* enable/disable chip for secondary charger */
	.enable_chip = eta6963_enable_chip,
	.is_chip_enabled = eta6963_is_chip_enabled,

	.get_charging_current = eta6963_get_ichg,
	.set_charging_current = eta6963_set_ichg,
	.get_input_current = eta6963_get_icl,
	.set_input_current = eta6963_set_icl,
	.get_constant_voltage = eta6963_get_vchg,
	.set_constant_voltage = eta6963_set_vchg,
	.kick_wdt = eta6963_kick_wdt,
	.set_mivr = eta6963_set_ivl,
	.is_charging_done = eta6963_is_charging_done,
	.get_min_charging_current = eta6963_get_min_ichg,

	/* Safety timer */
	.enable_safety_timer = eta6963_set_safety_timer,
	.is_safety_timer_enabled = eta6963_is_safety_timer_enabled,

	/* enable/disable powerpath for primary charger */
	.enable_powerpath = NULL,
	.is_powerpath_enabled = NULL,

	/* OTG */
	.enable_otg = eta6963_set_otg,
	.set_boost_current_limit = eta6963_set_boost_ilmt,
	.enable_discharge = NULL,

	/* PE+/PE+20 */
	.send_ta_current_pattern = eta6963_set_ta_current_pattern,
	.set_pe20_efficiency_table = eta6963_set_pep20_efficiency_table,
	.send_ta20_current_pattern = eta6963_set_ta20_current_pattern,
	.reset_ta = eta6963_set_ta_reset,
	.enable_cable_drop_comp = eta6963_enable_cable_drop_comp,

	/* ADC */
	.get_tchg_adc = NULL,
	/* Event */
	.event = eta6963_do_event,
};

static enum power_supply_usb_type eta6963_charger_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static enum power_supply_property eta6963_charge_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_USB_TYPE,
};

#define VBUS_THR 4000
static int eta6963_get_online(struct eta6963_info *info, union power_supply_propval *val)
{
	static struct power_supply *chg_psy;
	union power_supply_propval prop;

	if (chg_psy == NULL) {
		chg_psy = power_supply_get_by_name("mtk_charger_type");
	}

	if (IS_ERR_OR_NULL(chg_psy)) {
		dev_notice(info->dev, "%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	} else {
		power_supply_get_property(chg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		val->intval = prop.intval > VBUS_THR ? 1 : 0;
	}
	return 0;
}

static int eta6963_get_charger_type(struct eta6963_info *info, union power_supply_propval *val)
{
	static struct power_supply *chg_psy;

	if (chg_psy == NULL) {
			chg_psy = power_supply_get_by_name("mtk_charger_type");
	}

	if (IS_ERR_OR_NULL(chg_psy)) {
		dev_notice(info->dev, "%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	} else {
		power_supply_get_property(chg_psy,
				POWER_SUPPLY_PROP_USB_TYPE, val);
		switch (val->intval) {
		case POWER_SUPPLY_USB_TYPE_UNKNOWN:
			info->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
			break;
		case POWER_SUPPLY_USB_TYPE_SDP:
			info->psy_desc.type = POWER_SUPPLY_TYPE_USB;
			break;
		case POWER_SUPPLY_USB_TYPE_CDP:
			info->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
			break;
		case POWER_SUPPLY_USB_TYPE_DCP:
			info->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		}
	}
	return 0;
}

static int eta6963_get_charger_status(struct eta6963_info *info, union power_supply_propval *val)
{
	int ret, status;
	u8 reg_val;

	ret = eta6963_read_byte(info, ETA6963_REG_08, &reg_val);
	if (!ret) {
		reg_val = reg_val & REG08_CHRG_STAT_MASK;
		reg_val = reg_val >> REG08_CHRG_STAT_SHIFT;
		switch (reg_val) {
		case REG08_CHRG_STAT_IDLE:
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case REG08_CHRG_STAT_PRECHG ... REG08_CHRG_STAT_FASTCHG:
			status = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case REG08_CHRG_STAT_CHGDONE:
			status = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			ret = -EIO;
		}
		val->intval = status;
	} else {
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
	}

	return ret;
}

static int eta6963_charge_get_props(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct eta6963_info *info = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = eta6963_get_online(info, val);
		if (ret)
			return -ENODATA;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = eta6963_get_charger_status(info, val);
		if (ret)
			return -ENODATA;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		ret = eta6963_get_charger_type(info, val);
		if (ret)
			return -ENODATA;
		break;
	default:
		return 0;
	}

	return ret;
}

static char *eta6963_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

static int eta6963_psy_register(struct eta6963_info *info)
{
	int ret;

	info->psy_cfg.drv_data = info;
	info->psy_cfg.of_node = info->dev->of_node;

	info->psy_cfg.supplied_to = eta6963_charger_supplied_to;
	info->psy_cfg.num_supplicants = ARRAY_SIZE(eta6963_charger_supplied_to);

	if (info->slave_mode)
		info->psy_desc.name = "eta6963-slave";
	else
		info->psy_desc.name = "eta6963-master";

	info->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
	info->psy_desc.properties = eta6963_charge_props;
	info->psy_desc.num_properties = ARRAY_SIZE(eta6963_charge_props);
	info->psy_desc.get_property = eta6963_charge_get_props;
	info->psy_desc.usb_types = eta6963_charger_usb_type;
	info->psy_desc.num_usb_types = ARRAY_SIZE(eta6963_charger_usb_type);


	info->psy = devm_power_supply_register(info->dev,
			&info->psy_desc, &info->psy_cfg);
	if (IS_ERR(info->psy)) {
		dev_info(info->dev, "failed to register psy:%d\n", ret);
		return PTR_ERR(info->psy);
	}

	dev_info(info->dev, "%s power supply register successfully\n", info->psy_desc.name);

	return 0;
}

static int eta6963_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret = 0;
	struct eta6963_info *info = rdev_get_drvdata(rdev);

	ret = eta6963_enable_otg(info);
	if (ret)
		dev_err(info->dev, "Couldn't enable  OTG mode ret=%d\n", ret);
	else 
		enable_wdt_polling(info, true);

	return ret;
}

static int eta6963_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret = 0;
	struct eta6963_info *info = rdev_get_drvdata(rdev);

	ret = eta6963_disable_otg(info);

	if (ret)
		dev_err(info->dev, "Couldn't disable OTG mode rc=%d\n", ret);
	else 
		enable_wdt_polling(info, false);

	return ret;
}

static int eta6963_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int ret = 0;
	u8 val = 0;
	struct eta6963_info *info = rdev_get_drvdata(rdev);

	ret = eta6963_read_byte(info, ETA6963_REG_01, &val);
	if (ret)
		dev_err(info->dev, "Couldn't read OTG status ret=%d\n", ret);

	return (val & REG01_OTG_CONFIG_MASK) ? 1 : 0;
}

static struct regulator_ops eta6963_chg_otg_reg_ops = {
	.enable		= eta6963_otg_regulator_enable,
	.disable	= eta6963_otg_regulator_disable,
	.is_enabled	= eta6963_otg_regulator_is_enable,
};

static int eta6963_regulator_init(struct eta6963_info *info)
{
	int rc = 0;
	struct regulator_config cfg = {};
	struct regulator_init_data *init_data;

	info->otg_vreg.rdesc.owner = THIS_MODULE;
	info->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
	info->otg_vreg.rdesc.ops = &eta6963_chg_otg_reg_ops;
	info->otg_vreg.rdesc.name =
		info->dev->of_node->name;
	info->otg_vreg.rdesc.of_match =
		info->dev->of_node->name;

	init_data = of_get_regulator_init_data(info->dev, info->dev->of_node,
					&info->otg_vreg.rdesc);
	if (!init_data) {
		dev_err(info->dev, "regulator init data is missing\n");
		return -EINVAL;
	}

	cfg.dev = info->dev;
	cfg.driver_data = info;
	cfg.init_data = init_data;
	cfg.of_node = info->dev->of_node;

	info->otg_vreg.rdev = devm_regulator_register(info->dev,
					&info->otg_vreg.rdesc, &cfg);
	if (IS_ERR(info->otg_vreg.rdev)) {
		rc = PTR_ERR(info->otg_vreg.rdev);
		info->otg_vreg.rdev = NULL;
		if (rc != -EPROBE_DEFER)
			dev_err(info->dev, "OTG reg failed, rc=%d\n", rc);
	}
	return rc;
}

static struct of_device_id eta6963_charger_match_table[] = {
	{
	 .compatible = "ti,bq25601",
	 .data = &pn_data[PN_BQ25601],
	 },
	{
	 .compatible = "eta,eta6963",
	 .data = &pn_data[PN_ETA6963],
	 },
	{},
};
MODULE_DEVICE_TABLE(of, eta6963_charger_match_table);


static int eta6963_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct eta6963_info *info;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;

	int ret = 0;

	info = devm_kzalloc(&client->dev, sizeof(struct eta6963_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &client->dev;
	info->client = client;

	i2c_set_clientdata(client, info);

	mutex_init(&info->i2c_rw_lock);
	mutex_init(&info->gpio_access_lock);

	mutex_init(&info->irq_complete);
	mutex_init(&info->wdt_lock);
	atomic_set(&info->is_chip_en, 0);

	info->resume_completed = true;
	info->irq_waiting = false;

	match = of_match_node(eta6963_charger_match_table, node);
	if (match == NULL) {
		dev_err(info->dev, "device tree match not found\n");
		return -EINVAL;
	}

	info->platform_data = eta6963_parse_dt(node, info);

	if (!info->platform_data) {
		dev_err(info->dev, "No platform data provided.\n");
		return -EINVAL;
	}

	ret = eta6963_detect_device(info);
	if (info->chip == CHARGE_UNKNOWN) {
		ret = devm_gpio_request_one(info->dev, info->enable_pin, GPIOF_OUT_INIT_HIGH, "eta6963_en_gpio");
		if (ret) {
			dev_err(info->dev, "%s ce_gpio request fail\n", __func__);
			info->enable_pin = -1;
		}
		dev_err(info->dev, "No eta6963 device found!\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(info->dev, info->enable_pin, info->slave_mode ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW, "eta6963_en_gpio");
		if (ret) {
			dev_err(info->dev, "%s ce_gpio request fail\n", __func__);
			info->enable_pin = -1;
		}
		dev_info(info->dev, "%s: slave mode=%d\n", __func__, info->slave_mode);
	}

	if (info->part_no != *(int *)match->data)
		dev_info(info->dev, "part no mismatch, hw:%s, devicetree:%s\n",
			pn_str[info->part_no], pn_str[*(int *) match->data]);

	ret = eta6963_init_device(info);
	if (ret) {
		dev_err(info->dev, "Failed to init device\n");
		return ret;
	}
	if (!info->slave_mode) {
		ret = eta6963_regulator_init(info);
		if (ret) {
			dev_err(info->dev, "Couldn't initialize eta6963 ragulator ret=%d\n", ret);
			return ret;
		}
	}

	ret = eta6963_psy_register(info);
	eta6963_register_interrupt(info);

	info->chg_dev = charger_device_register(info->chg_dev_name,
					      &client->dev, info,
					      &eta6963_chg_ops,
					      &info->chg_props);
	if (IS_ERR_OR_NULL(info->chg_dev)) {
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}

	ret = sysfs_create_group(&info->dev->kobj, &eta6963_attr_group);
	if (ret)
		dev_err(info->dev, "failed to register sysfs. err: %d\n", ret);

	info->wdt_workq =
			create_singlethread_workqueue("wdt_workq");
	INIT_WORK(&info->kick_work, wdt_kick_work);
	INIT_DELAYED_WORK(&info->chg_stat_work, chg_stat_work_handler);
	info->polling_interval = 20;
	device_init_wakeup(info->dev, true);
	alarm_init(&info->wdt_timer, ALARM_BOOTTIME,
			wdt_timer_func);
	determine_initial_status(info);

	dev_err(info->dev, "eta6963 probe successfully, Part Num:%d, Revision:%d\n!",
	       info->part_no, info->revision);

	return 0;
}

static int eta6963_charger_remove(struct i2c_client *client)
{
	struct eta6963_info *info = i2c_get_clientdata(client);

	enable_wdt_polling(info, false);
	mutex_destroy(&info->i2c_rw_lock);
	mutex_destroy(&info->gpio_access_lock);
	mutex_destroy(&info->irq_complete);
	mutex_destroy(&info->wdt_lock);

	sysfs_remove_group(&info->dev->kobj, &eta6963_attr_group);

	return 0;
}

static void eta6963_charger_shutdown(struct i2c_client *client)
{

}

static int eta6963_charger_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct eta6963_info *info = i2c_get_clientdata(client);

	mutex_lock(&info->irq_complete);
	info->resume_completed = false;
	mutex_unlock(&info->irq_complete);
	dev_info(info->dev, "Suspend successfully!");

	return 0;
}

static int eta6963_charger_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct eta6963_info *info = i2c_get_clientdata(client);

	if (info->irq_waiting) {
		pr_info_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int eta6963_charger_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct eta6963_info *info = i2c_get_clientdata(client);


	mutex_lock(&info->irq_complete);
	info->resume_completed = true;
	if (info->irq_waiting) {
		info->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&info->irq_complete);
		eta6963_irq_handler(client->irq, info);
	} else {
		mutex_unlock(&info->irq_complete);
	}

	dev_info(info->dev, "Resume successfully!");

	return 0;
}

static const struct dev_pm_ops eta6963_charger_pm_ops = {
	.resume			= eta6963_charger_resume,
	.suspend_noirq 	= eta6963_charger_suspend_noirq,
	.suspend		= eta6963_charger_suspend,
};

static struct i2c_driver eta6963_charger_driver = {
	.driver = {
		   .name = "eta6963-charger",
		   .owner = THIS_MODULE,
		   .of_match_table = eta6963_charger_match_table,
		   .pm	= &eta6963_charger_pm_ops,
	},

	.probe = eta6963_charger_probe,
	.remove = eta6963_charger_remove,
	.shutdown = eta6963_charger_shutdown,

};

module_i2c_driver(eta6963_charger_driver);

MODULE_DESCRIPTION("ETA ETA6963 Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
