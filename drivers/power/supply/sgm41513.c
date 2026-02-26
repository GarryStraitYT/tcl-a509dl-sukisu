
#include <linux/types.h>
#include <linux/init.h>     /* For init/exit macros */
#include <linux/module.h>   /* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/alarmtimer.h>
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

#include "charger_class.h"

#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#define SGM41513_REG_NUM            0x10

/* Reg0x00 */
#define SGM41513_CON0               0x00
#define CON0_EN_HIZ_MASK			0x01
#define CON0_EN_HIZ_SHIFT			7

#define	CON0_STAT_IMON_CTRL_MASK	0x03
#define	CON0_STAT_IMON_CTRL_SHIFT 5

#define CON0_IINLIM_MASK   0x1F
#define CON0_IINLIM_SHIFT  0

/* Reg0x01 */
#define SGM41513_CON1               0x01
#define CON1_PFM_MASK     0x01
#define CON1_PFM_SHIFT    7

#define CON1_WDT_RST_MASK     0x01
#define CON1_WDT_RST_SHIFT    6

#define CON1_OTG_CONFIG_MASK	0x01
#define CON1_OTG_CONFIG_SHIFT	5

#define CON1_CHG_CONFIG_MASK        0x01
#define CON1_CHG_CONFIG_SHIFT       4

#define CON1_SYS_MIN_MASK        0x07
#define CON1_SYS_MIN_SHIFT       1

#define	CON1_MIN_VBAT_SEL_MASK	0x01
#define	CON1_MIN_VBAT_SEL_SHIFT	0

/* Reg0x02 */
#define SGM41513_CON2               0x02
#define CON2_BOOST_LIM_MASK   0x01
#define CON2_BOOST_LIM_SHIFT  7

#define CON2_Q1_FULLON_MASK   0x01
#define CON2_Q1_FULLON_SHIFT  6

#define CON2_ICHG_MASK    0x3F
#define CON2_ICHG_SHIFT   0

/* Reg0x03 */
#define SGM41513_CON3               0x03
#define CON3_IPRECHG_MASK   0x0F
#define CON3_IPRECHG_SHIFT  4

#define CON3_ITERM_MASK           0x0F
#define CON3_ITERM_SHIFT          0

/* Reg0x04 */
#define SGM41513_CON4               0x04
#define CON4_VREG_MASK     0x1F
#define CON4_VREG_SHIFT    3

#define	CON4_TOPOFF_TIMER_MASK 0x03
#define	CON4_TOPOFF_TIMER_SHIFT 1

#define CON4_VRECHG_MASK    0x01
#define CON4_VRECHG_SHIFT   0

/* Reg0x05 */
#define SGM41513_CON5               0x05
#define CON5_EN_TERM_MASK      0x01
#define CON5_EN_TERM_SHIFT     7

#define CON5_WATCHDOG_MASK     0x03
#define CON5_WATCHDOG_SHIFT    4

#define CON5_EN_TIMER_MASK      0x01
#define CON5_EN_TIMER_SHIFT     3

#define CON5_CHG_TIMER_MASK           0x01
#define CON5_CHG_TIMER_SHIFT          2

#define CON5_TREG_MASK     0x01
#define CON5_TREG_SHIFT    1

#define CON5_JEITA_ISET_L_MASK     0x01
#define CON5_JEITA_ISET_L_SHIFT    0

/* Reg0x06 */
#define SGM41513_CON6               0x06
#define	CON6_OVP_MASK		0x03
#define	CON6_OVP_SHIFT		6

#define	CON6_BOOSTV_MASK	0x3
#define	CON6_BOOSTV_SHIFT	4

#define	CON6_VINDPM_MASK	0x0F
#define	CON6_VINDPM_SHIFT	0

/* Reg0x07, different */
#define SGM41513_CON7               0x07
#define	CON7_IINDET_EN_MASK	0x01
#define	CON7_IINDET_EN_SHIFT	7

#define CON7_TMR2X_EN_MASK      0x01
#define CON7_TMR2X_EN_SHIFT     6

#define CON7_BATFET_Disable_MASK      0x01
#define CON7_BATFET_Disable_SHIFT     5

#define	CON7_JEITA_VSET_H_MASK		0x01
#define	CON7_JEITA_VSET_H_SHIFT		4

#define	CON7_BATFET_DLY_MASK		0x01
#define	CON7_BATFET_DLY_SHIFT		3

#define	CON7_BATFET_RST_EN_MASK		0x01
#define	CON7_BATFET_RST_EN_SHIFT	2

#define	CON7_VDPM_BAT_TRACK_MASK	0x03
#define	CON7_VDPM_BAT_TRACK_SHIFT	0

/* Reg0x08 */
#define SGM41513_CON8               0x08
#define CON8_VBUS_STAT_MASK      0x07
#define CON8_VBUS_STAT_SHIFT     5

#define CON8_CHRG_STAT_MASK           0x03
#define CON8_CHRG_STAT_SHIFT          3

#define CON8_PG_STAT_MASK           0x01
#define CON8_PG_STAT_SHIFT          2

#define CON8_THERM_STAT_MASK           0x01
#define CON8_THERM_STAT_SHIFT          1

#define CON8_VSYS_STAT_MASK           0x01
#define CON8_VSYS_STAT_SHIFT          0

/* Reg0x09 */
#define SGM41513_CON9               0x09
#define CON9_WATCHDOG_FAULT_MASK      0x01
#define CON9_WATCHDOG_FAULT_SHIFT     7

#define CON9_OTG_FAULT_MASK           0x01
#define CON9_OTG_FAULT_SHIFT          6

#define CON9_CHRG_FAULT_MASK           0x03
#define CON9_CHRG_FAULT_SHIFT          4

#define CON9_BAT_FAULT_MASK           0x01
#define CON9_BAT_FAULT_SHIFT          3

#define CON9_NTC_FAULT_MASK           0x07
#define CON9_NTC_FAULT_SHIFT          0

/* Reg0x0A */
#define SGM41513_CONA               0x0A
#define	CONA_VBUS_GD_MASK				0x01
#define	CONA_VBUS_GD_SHIFT				7

#define	CONA_VINDPM_STAT_MASK			0x01
#define	CONA_VINDPM_STAT_SHIFT			6

#define	CONA_IINDPM_STAT_MASK			0x01
#define	CONA_IINDPM_STAT_SHIFT			5

#define	CONA_TOPOFF_ACTIVE_MASK		0x01
#define	CONA_TOPOFF_ACTIVE_SHIFT		3

#define	CONA_ACOV_STAT_MASK			0x01
#define	CONA_ACOV_STAT_SHIFT			2

#define	CONA_VINDPM_INT_MASK			0x01
#define	CONA_VINDPM_INT_SHIFT			1

#define	CONA_IINDPM_INT_MASK			0x01
#define	CONA_IINDPM_INT_SHIFT			0

/* Reg0x0B */
#define SGM41513_CONB               0x0B
#define CONB_REG_RST_MASK     0x01
#define CONB_REG_RST_SHIFT    7

#define CONB_PN_MASK		0x0F
#define CONB_PN_SHIFT		3

#define CONB_SGMPART_MASK		0x01
#define CONB_SGMPART_SHIFT		2
#define SGM41513_PN_41513_ID		0x00
#define SGM41513_PN_41513A_ID		0x01

#define CONB_Rev_MASK           0x03
#define CONB_Rev_SHIFT          0

/* Reg0x0C */
#define SGM41513_CONC               0x0C

/* Reg0x0D */
#define SGM41513_COND               0x0D
#define COND_EN_PUMPX_MASK   0x1
#define COND_EN_PUMPX_SHIFT  7

#define COND_PUMPX_UP_MASK         0x1
#define COND_PUMPX_UP_SHIFT        6

#define COND_PUMPX_DN_MASK         0x1
#define COND_PUMPX_DN_SHIFT        5

#define COND_JEITA_EN_MASK         0x1
#define COND_JEITA_EN_SHIFT        0

/* Reg0x0E */
#define SGM41513_CONE               0x0E

/* Reg0x0F */
#define SGM41513_CONF               0x0F
#define CONF_VINDPM_OS_MASK         0x3
#define CONF_VINDPM_OS_SHIFT        0


/*SGM41513 REG04 VREG[7:3]*/
static const unsigned int VBAT_CV_VTH[] = {
    3856000, 3888000, 3920000, 3952000,
	3984000, 4016000, 4048000, 4080000,
	4112000, 4144000, 4176000, 4208000,
	4240000, 4272000, 4304000, 4352000,
	4368000, 4400000, 4432000, 4464000,
	4496000, 4528000, 4560000, 4592000,
	4624000
};

/*SGM41513 REG02 ICHG[5:0]*/
static const unsigned int CS_VTH[] = {
    0, 5000, 10000, 15000, 20000,
	25000, 30000, 35000, 40000, 50000,
	60000, 70000, 80000, 90000, 100000,
	110000, 130000, 150000, 170000, 190000,
	210000, 230000, 250000, 270000, 300000,
	330000, 360000, 390000, 420000, 450000,
	480000, 510000, 540000, 600000, 660000,
	720000, 780000, 840000, 900000, 960000,
	1020000, 1080000, 1140000, 1200000, 1260000,
	1320000, 1380000, 1440000, 1500000, 1620000,
	1740000, 1860000, 1980000, 2100000, 2220000,
	2340000, 2460000, 2580000, 2700000, 2820000,
	2940000, 3000000, 3000000, 3000000
};

/*SGM41513 REG00 IINLIM[4:0]*/
static const unsigned int INPUT_CS_VTH[] = {
	100000, 200000, 300000, 400000,
	500000, 600000, 700000, 800000,
	900000, 1000000, 1100000, 1200000,
	1300000, 1400000, 1500000, 1600000,
	1700000, 1800000, 1900000, 2000000,
	2100000, 2200000, 2300000, 2500000,
	2600000, 2700000, 2800000, 2900000,
	3000000, 3100000, 3200000
};

/* SGM41513 REG06 VINDPM[3:0] */
static const unsigned int VINDPM_REG[4][16] =
{
	{3900, 4000, 4100, 4200, 4300, 4400,
	 4500, 4600, 4700, 4800, 4900, 5000,
	 5100, 5200, 5300, 5400},
	{5900, 6000, 6100, 6200, 6300, 6400,
	 6500, 6600, 6700, 6800, 6900, 7000,
	 7100, 7200, 7300, 7400},
	{7500, 7600, 7700, 7800, 7900, 8000,
	 8100, 8200, 8300, 8400, 8500, 8600,
	 8700, 8800, 8900, 9000},
	{10500, 10600, 10700, 10800, 10900, 11000,
	 11100, 11200, 11300, 11400, 11500, 11600,
	 11700, 11800, 11900, 12000},
};

#define chr_err(fmt, args...)		\
do {								\
		pr_notice(fmt, ##args);		\
} while (0)

#define chr_debug(fmt, args...)		\
do {								\
		pr_notice(fmt, ##args);		\
} while (0)

struct sgm41513_info {
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct charger_properties chg_props;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	int irq;
	struct power_supply *chg_psy;
	struct power_supply_desc psy_desc;

/* Begin add by jin.wang for jira 5550 on 2022-1-7 */
	struct delayed_work chg_stat_work;
/* End add by jin.wang */
	struct alarm wdt_timer;
	struct workqueue_struct *wdt_workq;
	struct work_struct kick_work;
	unsigned int polling_interval;
	bool polling_enabled;
	struct mutex			irq_complete;
	bool				resume_completed;
	bool				irq_waiting;
	bool				irq_disabled;
	bool slave_mode;
	int enable_pin;
	int intr_gpio;
};

static DEFINE_MUTEX (g_input_current_mutex);
static struct i2c_client *new_client;

#if defined(CONFIG_TCT_CHG_PASSAT)
static unsigned int max_cv = 4450000;
static unsigned int now_cv = 4450000;
#else
static unsigned int now_cv = 4400000;
static unsigned int max_cv = 4400000;
#endif

static unsigned int charging_parameter_to_value(const unsigned int *parameter,
                                         const unsigned int array_size,
                                         const unsigned int val)
{
	unsigned int i;
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
		for (i = (number - 1); i != 0; i--) { /* max value in the last element */
			if (pList[i] <= level) {
				return pList[i];
			}
		}
		pr_info("Can't find closest level- %d\n", level);
		return pList[0];
	}
	else {
		for (i = 0; i < number; i++) { /* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}

		pr_info("Can't find closest level+ %d\n", level);
		return pList[number - 1];
	}
}

static unsigned char sgm41513_reg[SGM41513_REG_NUM] = {
    0
};

static DEFINE_MUTEX (sgm41513_i2c_access);
static DEFINE_MUTEX (sgm41513_access_lock);

static bool g_sgm41513_hw_exist = false;

static unsigned int sgm41513_read_byte(unsigned char cmd, unsigned char *returnData)
{
	unsigned char xfers = 2;
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
	}
	while (ret != xfers && --retries);

	mutex_unlock(&sgm41513_i2c_access);

	return ret == xfers ? 1 : -1;
}

static unsigned int sgm41513_write_byte(unsigned char cmd, unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

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
	}
	while (ret != xfers && --retries);

	mutex_unlock(&sgm41513_i2c_access);

	return ret == xfers ? 1 : -1;
}

static unsigned int sgm41513_read_interface(unsigned char RegNum,
                                    unsigned char *val,
                                    unsigned char MASK,
                                    unsigned char SHIFT)
{
	unsigned char sgm41513_reg = 0;
	unsigned int ret = 0;

	ret = sgm41513_read_byte(RegNum, &sgm41513_reg);

	chr_debug("[sgm41513_read_interface] Reg[%x]=0x%x\n", RegNum, sgm41513_reg);

	sgm41513_reg &= (MASK << SHIFT);
	*val = (sgm41513_reg >> SHIFT);

	chr_debug("[sgm41513_read_interface] val=0x%x\n", *val);

	return ret;
}

static unsigned int sgm41513_config_interface(unsigned char RegNum,
                                      unsigned char val,
                                      unsigned char MASK,
                                      unsigned char SHIFT)
{
	unsigned char sgm41513_reg = 0;
	unsigned char sgm41513_reg_ori = 0;
	unsigned int ret = 0;

	mutex_lock(&sgm41513_access_lock);
	ret = sgm41513_read_byte(RegNum, &sgm41513_reg);

	sgm41513_reg_ori = sgm41513_reg;
	sgm41513_reg &= ~(MASK << SHIFT);
	sgm41513_reg |= (val << SHIFT);

	ret = sgm41513_write_byte(RegNum, sgm41513_reg);
	mutex_unlock(&sgm41513_access_lock);

	chr_debug("[sgm41513_config_interface] write Reg[%x]=0x%x from 0x%x\n",
	          RegNum,
	          sgm41513_reg,
	          sgm41513_reg_ori);
	return ret;
}

/* CON0---------------------------------------------------- */
static void sgm41513_set_en_hiz(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON0),
                               (unsigned char) (val),
                               (unsigned char) (CON0_EN_HIZ_MASK),
                               (unsigned char) (CON0_EN_HIZ_SHIFT));
}

static int sgm41513_is_hz_enable(bool *en)
{
	unsigned char val = 0;
	sgm41513_read_interface((unsigned char) (SGM41513_CON0),
                             (&val),
                             (unsigned char) (CON0_EN_HIZ_MASK),
                             (unsigned char) (CON0_EN_HIZ_SHIFT));
	*en = (val == 0 ? false : true);
	return 0;
}

static void sgm41513_set_stat_pin(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON0),
                               (unsigned char) (val),
                               (unsigned char) (CON0_STAT_IMON_CTRL_MASK),
                               (unsigned char) (CON0_STAT_IMON_CTRL_SHIFT));
}

static void sgm41513_set_iinlim(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON0),
                               (val),
                               (unsigned char) (CON0_IINLIM_MASK),
                               (unsigned char) (CON0_IINLIM_SHIFT));
}

/* CON1---------------------------------------------------- */
static void sgm41513_wd_reset(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON1),
                               (val),
                               (unsigned char) (CON1_WDT_RST_MASK),
                               (unsigned char) (CON1_WDT_RST_SHIFT));
}

static void sgm41513_otg_en(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON1),
			                   (val),
			                   (unsigned char) (CON1_OTG_CONFIG_MASK),
			                   (unsigned char) (CON1_OTG_CONFIG_SHIFT));
}

static int sgm41513_is_otg_en(bool *en)
{
	unsigned char val = 0;
	sgm41513_read_interface((unsigned char) (SGM41513_CON1),
                             (&val),
                             (unsigned char) (CON1_OTG_CONFIG_MASK),
                             (unsigned char) (CON1_OTG_CONFIG_SHIFT));
	*en = (val == 0 ? false : true);
	return 0;
}

static void sgm41513_chg_en(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON1),
                               (val),
                               (unsigned char) (CON1_CHG_CONFIG_MASK),
                               (unsigned char) (CON1_CHG_CONFIG_SHIFT));
}

/* CON2--------------------------------------------------- */
static void sgm41513_set_boost_lim(unsigned int val)
{
    sgm41513_config_interface((unsigned char) (SGM41513_CON2),
								(unsigned char) (val),
								(unsigned char) (CON2_BOOST_LIM_MASK),
								(unsigned char) (CON2_BOOST_LIM_SHIFT));
}

static void sgm41513_set_ichg(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON2),
                               (unsigned char) (val),
                               (unsigned char) (CON2_ICHG_MASK),
                               (unsigned char) (CON2_ICHG_SHIFT));
}

/* CON3---------------------------------------------------- */
static void sgm41513_set_iprechg(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON3),
                               (val),
                               (unsigned char) (CON3_IPRECHG_MASK),
                               (unsigned char) (CON3_IPRECHG_SHIFT));
}

static void sgm41513_set_iterml(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON3),
                               (val),
                               (unsigned char) (CON3_ITERM_MASK),
                               (unsigned char) (CON3_ITERM_SHIFT));
}

/* CON4---------------------------------------------------- */
static void sgm41513_set_vreg(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON4),
                               (unsigned char) (val),
                               (unsigned char) (CON4_VREG_MASK),
                               (unsigned char) (CON4_VREG_SHIFT));
}

static unsigned int sgm41513_get_vreg(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = sgm41513_read_interface((unsigned char) (SGM41513_CON4),
                                 (&val),
                                 (unsigned char) (CON4_VREG_MASK),
                                 (unsigned char) (CON4_VREG_SHIFT));
    return val;
}

/* CON5---------------------------------------------------- */
static void sgm41513_set_wd_timer(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON5),
                               (unsigned char) (val),
                               (unsigned char) (CON5_WATCHDOG_MASK),
                               (unsigned char) (CON5_WATCHDOG_SHIFT));
}

static void sgm41513_set_chg_timer(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON5),
                               (unsigned char) (val),
                               (unsigned char) (CON5_CHG_TIMER_MASK),
                               (unsigned char) (CON5_CHG_TIMER_SHIFT));
}

static void sgm41513_en_chg_timer(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON5),
                               (unsigned char) (val),
                               (unsigned char) (CON5_EN_TIMER_MASK),
                               (unsigned char) (CON5_EN_TIMER_SHIFT));
}

static unsigned int sgm41513_get_chg_timer_enable(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = sgm41513_read_interface((unsigned char) (SGM41513_CON5),
                                 &val,
                                 (unsigned char) (CON5_EN_TIMER_MASK),
                                 (unsigned char) (CON5_EN_TIMER_SHIFT));

    return val;
}

/* CON6--------------------------------------------------- */
static void sgm41513_set_ovp(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON6),
							(unsigned char) (val),
							(unsigned char) (CON6_OVP_MASK),
							(unsigned char) (CON6_OVP_SHIFT)
							);

}

static unsigned int sgm41513_get_vindpm(void)
{
    unsigned char val = 0;
	sgm41513_read_interface((unsigned char) (SGM41513_CON6),
	                         (&val),
	                         (unsigned char) (CON6_VINDPM_MASK),
	                         (unsigned char) (CON6_VINDPM_SHIFT));
    return val;
}

static void sgm41513_set_vindpm(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON6),
                               (unsigned char) (val),
                               (unsigned char) (CON6_VINDPM_MASK),
                               (unsigned char) (CON6_VINDPM_SHIFT));
}

static void sgm41513_set_boostv(unsigned int val)
{
    sgm41513_config_interface((unsigned char) (SGM41513_CON6),
								(unsigned char) (val),
								(unsigned char) (CON6_BOOSTV_MASK),
								(unsigned char) (CON6_BOOSTV_SHIFT));
}

/* CON7---------------------------------------------------- */
static void sgm41513_batfet_rst_en(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CON7),
	                         (unsigned char) (val),
	                         (unsigned char) (CON7_BATFET_RST_EN_MASK),
	                         (unsigned char) (CON7_BATFET_RST_EN_SHIFT));
}

/* CON8--------------------------------------------------- */
static unsigned int sgm41513_get_chrg_state(void)
{
	unsigned char val = 0;
	sgm41513_read_interface((unsigned char) (SGM41513_CON8),
                             (&val),
                             (unsigned char) (CON8_CHRG_STAT_MASK),
                             (unsigned char) (CON8_CHRG_STAT_SHIFT));
	return val;
}

/* CON9--------------------------------------------------- */
static unsigned int sgm41513_get_wdt_state(void)
{
	unsigned char val = 0;
	sgm41513_read_interface((unsigned char) (SGM41513_CON9),
                             (&val),
                             (unsigned char) (CON9_WATCHDOG_FAULT_MASK),
                             (unsigned char) (CON9_WATCHDOG_FAULT_SHIFT));
	return val;
}

/* CONA---------------------------------------------------- */
static void sgm41513_set_vindpm_irq_mask(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CONA),
                               (unsigned char) (val),
                               (unsigned char) (CONA_VINDPM_INT_MASK),
                               (unsigned char) (CONA_VINDPM_INT_SHIFT));
}

static void sgm41513_set_iindpm_irq_mask(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CONA),
                               (unsigned char) (val),
                               (unsigned char) (CONA_IINDPM_INT_MASK),
                               (unsigned char) (CONA_IINDPM_INT_SHIFT));
}

/* CONB---------------------------------------------------- */

/* CONC---------------------------------------------------- */

/* COND---------------------------------------------------- */
static void sgm41513_set_jeita_en(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_COND),
                               (unsigned char) (val),
                               (unsigned char) (COND_JEITA_EN_MASK),
                               (unsigned char) (COND_JEITA_EN_SHIFT));
}

static void sgm41513_en_pumpx(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_COND),
                               (unsigned char) (val),
                               (unsigned char) (COND_EN_PUMPX_MASK),
                               (unsigned char) (COND_EN_PUMPX_SHIFT));
}

/* CONF---------------------------------------------------- */
static unsigned int sgm41513_get_vindpm_os(void)
{
    unsigned char val = 0;
	sgm41513_read_interface((unsigned char) (SGM41513_CONF),
	                         (&val),
	                         (unsigned char) (CONF_VINDPM_OS_MASK),
	                         (unsigned char) (CONF_VINDPM_OS_SHIFT));
    return val;
}

static void sgm41513_set_vindpm_os(unsigned int val)
{
	sgm41513_config_interface((unsigned char) (SGM41513_CONF),
                               (unsigned char) (val),
                               (unsigned char) (CONF_VINDPM_OS_MASK),
                               (unsigned char) (CONF_VINDPM_OS_SHIFT));
}

static void sgm41513_pumpx_up(unsigned int val)
{
	unsigned int ret = 0;

	/* Input current limit = 500 mA, changes after PE+ detection */
	sgm41513_set_iinlim(4);

	/* CC mode current = 2040 mA */
	sgm41513_set_ichg(0x22);
	sgm41513_chg_en(1);
	sgm41513_en_pumpx(1);

	if (val == 1) {
		ret = sgm41513_config_interface((unsigned char) (SGM41513_COND),
		                               (unsigned char) (1),
		                               (unsigned char) (COND_PUMPX_UP_MASK),
		                               (unsigned char) (COND_PUMPX_UP_SHIFT));
	}
	else {
		ret = sgm41513_config_interface((unsigned char) (SGM41513_CON9),
		                               (unsigned char) (1),
		                               (unsigned char) (COND_PUMPX_DN_MASK),
		                               (unsigned char) (COND_PUMPX_DN_SHIFT));
	}

	/* Input current limit = 500 mA, changes after PE+ detection */
	sgm41513_set_iinlim(4);

	/* CC mode current = 2040 mA */
	sgm41513_set_ichg(0x22);
	msleep(3000);
}

static void enable_wdt_polling(struct sgm41513_info *info, bool poll_en);
static void sgm41513_hw_component_detect(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

	ret = sgm41513_read_interface(SGM41513_CONB,
					&val, CONB_SGMPART_MASK, CONB_SGMPART_SHIFT);

	if (ret == 1) {
		ret = sgm41513_read_interface(SGM41513_CONB,
					&val, CONB_PN_MASK, CONB_PN_SHIFT);
		switch (val) {
			case SGM41513_PN_41513_ID:
			case SGM41513_PN_41513A_ID:
				g_sgm41513_hw_exist = true;
				pr_err("[%s] charger IC is SGM41513(%x)\n", __func__, val);
				break;
			default:
				pr_err("[%s] charger IC is Unknown\n", __func__);
				break;
		}
	}
}

static int sgm41513_enable_charging(struct charger_device *chg_dev, bool en)
{
	struct sgm41513_info *info = dev_get_drvdata(&chg_dev->dev);
	int status = 0;

	if (info->slave_mode) {
		//sgm41513_set_en_hiz(!en);
		gpio_set_value(info->enable_pin, !en);
		usleep_range(1000, 1200);
	}

	if (en) {
		sgm41513_set_en_hiz(0x0);
		sgm41513_chg_en(en);
	}
	else {
		sgm41513_chg_en(en);
	}

	return status;
}

static int sgm41513_get_current(struct charger_device *chg_dev, u32 *ichg)
{
    unsigned char val = 0;
	unsigned char array_size = ARRAY_SIZE(CS_VTH);

	/* Get current level */
	sgm41513_read_interface((unsigned char) (SGM41513_CON2),
                             (&val),
                             (unsigned char) (CON2_ICHG_MASK),
                             (unsigned char) (CON2_ICHG_SHIFT));

	if (val < array_size) {
		*ichg = CS_VTH[val];  // uA
		return 0;
	} else
		return -EINVAL;
}

static int sgm41513_set_current(struct charger_device *chg_dev,
                               u32 current_value)
{
    unsigned int status = true;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	pr_info("#### FCC = %d\n", current_value);

	array_size = ARRAY_SIZE(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size,
			  current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size,
			 set_chr_current);
	pr_info("#### %s register_value = %d\n", __func__,
			register_value);

	sgm41513_set_ichg(register_value);
	return status;
}

static int sgm41513_get_input_current(struct charger_device *chg_dev, u32 *aicr)
{
	*aicr = 0;
	return -EINVAL;
}

static int sgm41513_set_input_current(struct charger_device *chg_dev,
                                     u32 current_value)
{
	int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	mutex_lock(&g_input_current_mutex);
	pr_info("### ICL = %d\n", current_value);

	array_size = ARRAY_SIZE(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH,
	                                         array_size,
	                                         current_value);
	register_value = charging_parameter_to_value(INPUT_CS_VTH,
	                                             array_size,
	                                             set_chr_current);
	sgm41513_set_iinlim(register_value);
	mutex_unlock(&g_input_current_mutex);

	pr_info("### %s register_value = %d\n", __func__,
			register_value);

    /*For USB_IF compliance test only when USB is in suspend(Ibus < 2.5mA) or unconfigured(Ibus < 70mA) states*/
#ifdef CONFIG_USBIF_COMPLIANCE
	if (current_value < 10000) {
		sgm41513_set_vindpm_os(3);  // set vindpm as 10.5V
		sgm41513_set_vindpm(0);
	} else {
		sgm41513_set_vindpm_os(0);  // set vindpm as 4.2V
		sgm41513_set_vindpm(3);
	}
#endif

    return status;
}

static int sgm41513_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
{
	int status = 0;
	unsigned short int array_size;
	unsigned int set_cv_voltage;
	unsigned short int register_value;

	now_cv = cv;
	if (max_cv < now_cv) {
		max_cv = now_cv;
	}
	pr_info("%s: max_cv=%d now_cv:%d\n", __func__, max_cv, now_cv);

	array_size = ARRAY_SIZE(VBAT_CV_VTH);
	set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv);
	register_value = charging_parameter_to_value(VBAT_CV_VTH,
	                                         ARRAY_SIZE(VBAT_CV_VTH),
	                                         set_cv_voltage);
	sgm41513_set_vreg(register_value);
	return status;
}

static int sgm41513_get_battery_vreg(struct charger_device *chg_dev, u32 *cv)
{
	u8 reg_cv = 0;
	reg_cv = sgm41513_get_vreg();

	if (reg_cv > 0) {
		*cv = (reg_cv > 0x18) ? 4624000 : VBAT_CV_VTH[reg_cv];
		return 0;
	}
	else {
		*cv = 0;
		return -1;
	}
}

static int sgm41513_reset_watch_dog_timer(struct charger_device *chg_dev)
{
	sgm41513_wd_reset(1);
	return 0;
}

static int sgm41513_is_power_path_enable(struct charger_device *chg_dev,
                                            bool *en)
{
	u32 vindpm_os = 0, vindpm_val = 0;

	vindpm_os = sgm41513_get_vindpm_os();
	vindpm_val = sgm41513_get_vindpm();

	if (vindpm_os == 3 && vindpm_val == 0) {  // 10.5V
		*en = false;
	}
	return 0;
}

static int sgm41513_set_vindpm_voltage(struct charger_device *chg_dev,
                                      u32 vindpm)
{
	u32 array_size, vindpm_os = 0;
	bool is_power_path_enable = true;
	int ret = 0;

	/* Since sgm41513 uses vindpm to turn off power path
	    If power path is disabled, do not adjust mivr	*/
	ret = sgm41513_is_power_path_enable(chg_dev, &is_power_path_enable);
	if (ret == 0 && !is_power_path_enable) {
		pr_info("%s: power path is disable, skip setting vindpm = %d\n",
		        __func__, vindpm);
		return 0;
	}

	if (true) {  // set vindpm to 4.2V always
		vindpm_os = 0;
		vindpm = 3;
	} else {
		vindpm /= 1000;
		if (vindpm >= VINDPM_REG[3][0] && vindpm <= VINDPM_REG[3][15]) {
			vindpm_os = 3;  // 10500 ~ 12000 mV
		} else if ((vindpm >= VINDPM_REG[2][0] && vindpm <= VINDPM_REG[2][15])
			|| (vindpm > VINDPM_REG[2][15] && vindpm <= VINDPM_REG[3][0])) {
			vindpm_os = 2;  // 7500 ~ 9000 mV ~ 10500 mV
		} else if ((vindpm >= VINDPM_REG[1][0] && vindpm <= VINDPM_REG[1][15])
			|| (vindpm > VINDPM_REG[1][15] && vindpm < VINDPM_REG[2][0])) {
			vindpm_os = 1;  // 5900 ~ 7400 mV ~ 7500 mV
		} else {
			vindpm_os = 0;  // 3900 ~ 5400 mV
		}
		array_size = ARRAY_SIZE(VINDPM_REG[vindpm_os]);
		vindpm = bmt_find_closest_level(VINDPM_REG[vindpm_os], array_size, vindpm);
		vindpm = charging_parameter_to_value(VINDPM_REG[vindpm_os], array_size, vindpm);

		pr_info("%s: final vindpm_os=%d, vindpm=%d\n",
		        __func__, vindpm_os, vindpm);
	}

	sgm41513_set_vindpm_os(vindpm_os);
	sgm41513_set_vindpm(vindpm);
	return 0;
}

static int sgm41513_is_charging_done(struct charger_device *chg_dev,
                                       bool *is_done)
{
	int status = 0;
	unsigned char val = 0;

	sgm41513_read_interface(SGM41513_CON8,
							&val,
							CON8_CHRG_STAT_MASK,
							CON8_CHRG_STAT_SHIFT);
	if (now_cv < max_cv) {
		*is_done = false;
		printk("now_cv(%d) < max_cv(%d) never full\n", now_cv, max_cv);
		return 0;
	}

	if (val == 0x3) /* check if chrg done */
		*is_done = true;
	else
		*is_done = false;

	return status;
}

static int sgm41513_enable_otg(struct charger_device *chg_dev, bool en)
{
	struct sgm41513_info *info = charger_get_data(chg_dev);
	sgm41513_otg_en(en);
	enable_wdt_polling(info, en);
	return 0;
}

static int sgm41513_enable_safetytimer(struct charger_device *chg_dev, bool en)
{
	sgm41513_en_chg_timer(en);
	return 0;
}

static int sgm41513_get_is_safetytimer_enable(struct charger_device *chg_dev,
                                             bool *en)
{
	u32 reg_safetytimer = 0;
	reg_safetytimer = sgm41513_get_chg_timer_enable();
	*en = (reg_safetytimer) ? true : false;
	return 0;
}

static int sgm41513_set_ta_current_pattern(struct charger_device *chg_dev,
                                          bool is_increase)
{
	sgm41513_pumpx_up(is_increase);
	pr_err("%s: Pumping up adaptor...", __func__);
	return 0;
}

static int sgm41513_set_pep20_efficiency_table(struct charger_device *chg_dev)
{
	return 0;
}

static int sgm41513_set_ta_reset(struct charger_device *chg_dev)
{
	sgm41513_set_vindpm_os(0);  // set vindpm as 4.2V
	sgm41513_set_vindpm(3);

	sgm41513_set_ichg(8);  // Ibatt = 480 mA
	sgm41513_set_iinlim(0);  // ibus = 100mA
	msleep(250);
	sgm41513_set_iinlim(6); // ibus = 700mA
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

static int sgm41513_set_ta20_current_pattern(struct charger_device *chg_dev,
                                            u32 chr_vol)
{
	int value;
	int i, j = 0;
	int flag;

	sgm41513_set_vindpm_os(0);  // set vindpm as 4.2V
	sgm41513_set_vindpm(3);
	sgm41513_set_ichg(8);  // Ibatt = 480 mA

	usleep_range(1000, 1200);

	value = (chr_vol - 5500000) / 500000;
	sgm41513_set_iinlim(0x0);  // set ibus 100mA
	msleep(70);

	get_monotonic_boottime(&ptime[j++]);
	for (i = 4; i >= 0; i--) {
			flag = value & (1 << i);
			if (flag == 0) {
			sgm41513_set_iinlim(6); // ibus = 700mA

			msleep(PEOFFTIME);

			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEOFFTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 30 || cptime[j][1] > 65) {
			    pr_info("charging_set_ta20_current_pattern fail1: idx:%d target:%d actual:%d\n",
			            i,
			            PEOFFTIME,
			            cptime[j][1]);
			}
			j++;
			sgm41513_set_iinlim(0x0);
			msleep(PEONTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEONTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 90 || cptime[j][1] > 115) {
			    pr_info("charging_set_ta20_current_pattern fail2: idx:%d target:%d actual:%d\n",
			            i,
			            PEOFFTIME,
			            cptime[j][1]);
			}
			j++;
	    }
	    else {
			sgm41513_set_iinlim(6); // ibus = 700mA
			msleep(PEONTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEONTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 90 || cptime[j][1] > 115) {
			    pr_info("charging_set_ta20_current_pattern fail3: idx:%d target:%d actual:%d\n",
			            i,
			            PEOFFTIME,
			            cptime[j][1]);
			}
			j++;
			sgm41513_set_iinlim(0x0);
			msleep(PEOFFTIME);
			get_monotonic_boottime(&ptime[j]);
			cptime[j][0] = PEOFFTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 30 || cptime[j][1] > 65) {
			    pr_info("charging_set_ta20_current_pattern fail4: idx:%d target:%d actual:%d\n",
			            i,
			            PEOFFTIME,
			            cptime[j][1]);
			}
			j++;
		}
	}

	sgm41513_set_iinlim(6); // ibus = 700mA
	msleep(160);
	get_monotonic_boottime(&ptime[j]);
	cptime[j][0] = 160;
	cptime[j][1] = dtime(j);
	if (cptime[j][1] < 150 || cptime[j][1] > 240) {
		pr_info("charging_set_ta20_current_pattern fail5: idx:%d target:%d actual:%d\n",
		        i,
		        PEOFFTIME,
		        cptime[j][1]);
	}
	j++;
	sgm41513_set_iinlim(0x0);
	msleep(30);
	sgm41513_set_iinlim(6); // ibus = 700mA

	pr_info("%d %d: %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
	        chr_vol, value, cptime[1][0], cptime[2][0], cptime[3][0],
	        cptime[4][0], cptime[5][0], cptime[6][0], cptime[7][0],
	        cptime[8][0], cptime[9][0], cptime[10][0], cptime[11][0]);
	pr_info("%3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
	        cptime[1][1], cptime[2][1], cptime[3][1], cptime[4][1],
	        cptime[5][1], cptime[6][1], cptime[7][1], cptime[8][1],
	        cptime[9][1], cptime[10][1], cptime[11][1]);

	return 0;
}

static int sgm41513_dump_register(struct charger_device *chg_dev)
{
	unsigned char i = 0;

	for (i = 0; i < SGM41513_REG_NUM; i++) {
		sgm41513_read_byte(i, &sgm41513_reg[i]);
		chr_debug("[sgm41513 reg@][0x%x]=0x%x ", i, sgm41513_reg[i]);
	}
	return 0;
}

/* Begin add by jin.wang for jira 5550 on 2022-1-7 */
static void chg_stat_work_handler(struct work_struct *data)
{
	struct sgm41513_info *info = NULL;
	static unsigned char last_stat = 0xFF;
	unsigned char curr_stat = 0;
	int ret = 0;

	pr_info("%s\n", __func__);
	info = container_of(to_delayed_work(data),
					struct sgm41513_info, chg_stat_work);
	if (IS_ERR_OR_NULL(info)) {
		pr_err("[%s]: NULL pointer\n", __func__);
		return;
	};

	ret = sgm41513_read_interface(SGM41513_CON8,
							&curr_stat,
							CON8_CHRG_STAT_MASK,
							CON8_CHRG_STAT_SHIFT);
	if ((ret == 1) && curr_stat != last_stat) {
		pr_info("[%s]: %d -> %d\n", __func__,
				last_stat, curr_stat);
		last_stat = curr_stat;
		power_supply_changed(info->chg_psy);
	}
}
/* End add by jin.wang */

static irqreturn_t sgm41513_irq_handler(int irq, void *data)
{
/* Begin add by jin.wang for jira 5550 on 2022-1-7 */
	struct sgm41513_info *info = (struct sgm41513_info *)data;
/* End add by jin.wang */
	bool en = false;
	pr_info("%s\n", __func__);
	mutex_lock(&info->irq_complete);
	info->irq_waiting = true;
	if (!info->resume_completed) {
		dev_dbg(info->dev, "IRQ triggered before device-resume\n");
		if (!info->irq_disabled) {
			disable_irq_nosync(irq);
			info->irq_disabled = true;
		}
		mutex_unlock(&info->irq_complete);
		return IRQ_HANDLED;
	}
	info->irq_waiting = false;

	/* Skip irq if in OTG mode */
	sgm41513_is_otg_en(&en);
	if (en) {
		mutex_unlock(&info->irq_complete);
		return IRQ_HANDLED;
	}

/* Begin add by jin.wang for jira 5550 on 2022-1-7 */
	schedule_delayed_work(&info->chg_stat_work,
				msecs_to_jiffies(100));
/* End add by jin.wang */
	mutex_unlock(&info->irq_complete);
	return IRQ_HANDLED;
}

static int sgm41513_register_irq(struct sgm41513_info *info)
{
	int ret = 0;

	struct device_node *np;

	if (info->intr_gpio < 0) {
		/* Parse irq number from dts */
		np = of_find_node_by_name(NULL, info->eint_name);
		if (np)
			info->irq = irq_of_parse_and_map(np, 0);
		else {
			pr_err("%s: cannot get node\n", __func__);
			ret = -ENODEV;
			return ret;
		}
	} else {
		ret = devm_gpio_request_one(info->dev, info->intr_gpio, GPIOF_DIR_IN,
				devm_kasprintf(info->dev, GFP_KERNEL,
				"sgm41513_intr_gpio.%s", dev_name(info->dev)));
		if (ret < 0) {
			dev_notice(info->dev, "%s gpio request fail(%d)\n",
						__func__, ret);
			return ret;
		}
		info->irq = gpio_to_irq(info->intr_gpio);
		if (info->irq < 0) {
			dev_notice(info->dev, "%s gpio2irq fail(%d)\n",
						__func__, info->irq);
			return info->irq;
		}
	}

	dev_info(info->dev, "%s irq = %d\n", __func__, info->irq);

	/* Request threaded IRQ */
	ret = devm_request_threaded_irq(info->dev,
                                info->irq,
                                NULL,
                                sgm41513_irq_handler,
                                IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                info->eint_name,
                                info);
	if (ret < 0) {
		pr_err("%s: request thread irq failed\n", __func__);
		return ret;
	}

	enable_irq_wake(info->irq);
	return 0;
}

static int sgm41513_parse_dt(struct sgm41513_info *info, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	pr_info("%s\n", __func__);

	if (!np) {
		pr_err("%s: no of node\n", __func__);
		return -ENODEV;
	}

	if (of_property_read_string(np, "charger_name", &info->chg_dev_name) < 0) {
		info->chg_dev_name = "primary_chg";
		pr_warn("%s: no charger name\n", __func__);
	}

	if (of_property_read_string(np, "alias_name", &(info->chg_props.alias_name))
	                < 0) {
		info->chg_props.alias_name = "sgm41513";
		pr_warn("%s: no alias name\n", __func__);
	}

	info->intr_gpio = of_get_named_gpio(np, "gpio_sgm41513_intr", 0);
	if (info->intr_gpio < 0) {
		dev_notice(info->dev, "%s no gpio_sgm41513_intr\n",
				      __func__);
		info->intr_gpio = -1;
	}

	info->enable_pin =
		of_get_named_gpio(info->dev->of_node, "gpio_sgm41513_en", 0);
	if(info->enable_pin < 0){
		pr_err("%s: no gpio_sgm41513_en\n", __func__);
		info->enable_pin = -1;
	}
	pr_info("%s: enable_pin=%d\n", __func__, info->enable_pin);

	if (info->chg_dev_name
		&& !strncmp(info->chg_dev_name, "secondary_chg", strlen("secondary_chg"))) {
		info->slave_mode = 1;
		pr_info("%s: slave mode\n", __func__);

		ret = devm_gpio_request_one(info->dev, info->enable_pin, GPIOF_OUT_INIT_HIGH, "sgm41513_en_pin");
		if (ret) {
			pr_err("%s ce_gpio request fail\n", __func__);
			info->enable_pin = -1;
		}
	}

	if (!info->slave_mode) {
		if (of_property_read_string(np, "eint_name", &info->eint_name) < 0) {
			info->eint_name = "chr_stat";
			pr_warn("%s: no eint name\n", __func__);
		}
	}

	return 0;
}

static int sgm41513_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct sgm41513_info *info = dev_get_drvdata(&chg_dev->dev);

	if (info == NULL)
		return -EINVAL;
	if (chg_dev == NULL)
		return -EINVAL;

	printk("%s: event = %d\n", __func__, event);
	power_supply_changed(info->chg_psy);
	return 0;
}

static int sgm41513_charger_plug_in(struct charger_device *chg_dev)
{
	struct sgm41513_info *info = dev_get_drvdata(&chg_dev->dev);
	if (0x1 == sgm41513_get_wdt_state()) {
		chr_err("%s wdt timeout,reset some reg\n", __func__);
		sgm41513_wd_reset(1);
		sgm41513_set_stat_pin(3);
		if (info->slave_mode)
			sgm41513_set_ovp(2);  // set OVP to 10.5V
		else
			sgm41513_set_ovp(1);  // set OVP to 6.5V
		sgm41513_batfet_rst_en(0); // disable batfet reset
		sgm41513_set_iterml(0xD); //180 mA
		sgm41513_set_iprechg(0xF); //240mA
		sgm41513_set_vindpm_os(0);  // set vindpm as 4.2V
		sgm41513_set_vindpm(3);
		sgm41513_set_vindpm_irq_mask(1);  // mask vindpm int
		sgm41513_set_iindpm_irq_mask(1);  // mask iindpm int
		sgm41513_set_jeita_en(0);  // disable jeita
	}
	enable_wdt_polling(info, true);
	return 0;
}

static int sgm41513_charger_plug_out(struct charger_device *chg_dev)
{
	struct sgm41513_info *info = dev_get_drvdata(&chg_dev->dev);

	enable_wdt_polling(info, false);
	return 0;
}

static int sgm41513_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = 0;
	return 0;
}

static int sgm41513_get_min_aicr(struct charger_device *chg_dev, u32 *uA)
{
	*uA = 100000;
	return 0;
}

static int sgm41513_enable_chip(struct charger_device *chg_dev, bool en)
{
	struct sgm41513_info *info = dev_get_drvdata(&chg_dev->dev);
	sgm41513_set_en_hiz(en ? 0x00 : 0x01);
	enable_wdt_polling(info, en);
	pr_err("%s: %s sgm41513 chip\n", __func__,
			en ? "enable" : "disable");
	return 0;
}

static int sgm41513_is_chip_enabled(struct charger_device *chg_dev, bool *en)
{
	sgm41513_is_hz_enable(en);
	return 0;
}

static struct charger_ops sgm41513_chg_ops = {
#if 0
	.enable_hz = sgm41513_enable_hz,
#endif
	.enable_chip = sgm41513_enable_chip,
	.is_chip_enabled = sgm41513_is_chip_enabled,
	/* Normal charging */
	.plug_in = sgm41513_charger_plug_in,
	.plug_out = sgm41513_charger_plug_out,
	.dump_registers = sgm41513_dump_register,
	.enable = sgm41513_enable_charging,
	.get_charging_current = sgm41513_get_current,
	.set_charging_current = sgm41513_set_current,
	.get_input_current = sgm41513_get_input_current,
	.set_input_current = sgm41513_set_input_current,
	.get_constant_voltage = sgm41513_get_battery_vreg,
	.set_constant_voltage = sgm41513_set_cv_voltage,
	.kick_wdt = sgm41513_reset_watch_dog_timer,
	.set_mivr = sgm41513_set_vindpm_voltage,

	.is_charging_done = sgm41513_is_charging_done,

	.get_min_charging_current = sgm41513_get_min_ichg,
	.get_min_input_current = sgm41513_get_min_aicr,

	/* Safety timer */
	.enable_safety_timer = sgm41513_enable_safetytimer,
	.is_safety_timer_enabled = sgm41513_get_is_safetytimer_enable,

	/* OTG */
	.enable_otg = sgm41513_enable_otg,

	/* PE+/PE+20 */
	.send_ta_current_pattern = sgm41513_set_ta_current_pattern,
	.set_pe20_efficiency_table = sgm41513_set_pep20_efficiency_table,
	.send_ta20_current_pattern = sgm41513_set_ta20_current_pattern,
	.reset_ta = sgm41513_set_ta_reset,

	/* Event */
	.event = sgm41513_do_event,
};

static void enable_wdt_polling(struct sgm41513_info *info, bool poll_en)
{
	struct timespec time, time_now, end_time;
	ktime_t ktime;

	if (info) {
		if (poll_en) {
			get_monotonic_boottime(&time_now);
			time.tv_sec = info->polling_interval;
			time.tv_nsec = 0;
			end_time = timespec_add(time_now, time);
			ktime = ktime_set(end_time.tv_sec, end_time.tv_nsec);
			alarm_start(&info->wdt_timer, ktime);
			sgm41513_set_wd_timer(0x1); // set wdg timer 40s
			info->polling_enabled = true;
		} else {
			sgm41513_set_wd_timer(0x0); //disable wd timer
			info->polling_enabled = false;
			alarm_cancel(&info->wdt_timer);
		}
	}
}

static void wdt_kick_work(struct work_struct *work)
{
	ktime_t ktime;
	struct timespec time, time_now, end_time;
	struct sgm41513_info *info =
		container_of(work, struct sgm41513_info, kick_work);

	msleep(100); /*avoid i2c errors(i2c-5: err, access at suspend no irq stage) */
	sgm41513_reset_watch_dog_timer(info->chg_dev);
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
	struct sgm41513_info *info =
		container_of(alarm, struct sgm41513_info, wdt_timer);

	pm_stay_awake(info->dev);
	queue_work(info->wdt_workq, &info->kick_work);

	return ALARMTIMER_NORESTART;
}

/* TODO: before enable otg function, make slave charger hiz mode or forbid chg? */
static int sgm41513_boost_enable(struct regulator_dev *rdev)
{
	struct sgm41513_info *info = rdev_get_drvdata(rdev);

	pr_err("sgm41513 otg regulator enable\n");
	sgm41513_chg_en(0);
	sgm41513_otg_en(1);
	enable_wdt_polling(info, true);
	return 0;
}

static int sgm41513_boost_disable(struct regulator_dev *rdev)
{
	struct sgm41513_info *info = rdev_get_drvdata(rdev);

	pr_err("sgm41513 otg regulator disable\n");
	sgm41513_otg_en(0);
	sgm41513_chg_en(1);
	enable_wdt_polling(info, false);
	return 0;
}

static int sgm41513_boost_is_enabled(struct regulator_dev *rdev)
{
	bool en;
	sgm41513_is_otg_en(&en);
	return en;
}

static const struct regulator_ops sgm41513_chg_otg_ops = {
	.enable = sgm41513_boost_enable,
	.disable = sgm41513_boost_disable,
	.is_enabled = sgm41513_boost_is_enabled,
};

static const struct regulator_desc sgm41513_otg_rdesc = {
	.name = "usb-otg-vbus",
	.ops = &sgm41513_chg_otg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static const struct regulator_init_data sgm41513_vbus_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static enum power_supply_usb_type sgm41513_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static enum power_supply_property sgm41513_charge_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_USB_TYPE,
};

#define VBUS_THR 4000
static int sgm41513_get_online(struct sgm41513_info *info, union power_supply_propval *val)
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

static int sgm41513_get_charger_type(struct sgm41513_info *info, union power_supply_propval *val)
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

static int sgm41513_charge_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct sgm41513_info *info = power_supply_get_drvdata(psy);
	int ret = 0;
	unsigned int chg_stat;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = sgm41513_get_online(info, val);
		if (ret)
			return -ENODATA;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		chg_stat = sgm41513_get_chrg_state();
		switch (chg_stat) {
			case 0:
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
			case 1:
			case 2:
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
				break;
			case 3:
				val->intval = POWER_SUPPLY_STATUS_FULL;
				break;
			default:
				ret = -ENODATA;
				break;
			}
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		ret = sgm41513_get_charger_type(info, val);
		if (ret)
			return -ENODATA;
		break;
	default:
		return 0;
	}

	return ret;
}

static char *sgm41513_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

static const struct power_supply_desc sgm41513_charge_desc = {
	.name = "sgm41513",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sgm41513_charge_props,
	.num_properties = ARRAY_SIZE(sgm41513_charge_props),
	.get_property = sgm41513_charge_get_property,
	.usb_types = sgm41513_usb_type,
	.num_usb_types = ARRAY_SIZE(sgm41513_usb_type),

};

#ifdef CONFIG_TCT_DEVICEINFO
extern char charger_module_name[256];
#endif

static void sgm41513_hw_init(struct sgm41513_info *info)
{
	if (info->slave_mode) {  /* Slave mode */
		sgm41513_set_ovp(2);  // set OVP to 10.5V
		sgm41513_otg_en(0);	/* disable OTG function */
		sgm41513_chg_en(0);  /* sw control disable chg */
		sgm41513_set_en_hiz(0x1);  /* enable Hiz mode */
	} else {
		sgm41513_set_en_hiz(0x0);
		sgm41513_set_ovp(0x1);  // set OVP to 6.5V
	}

	if (info->enable_pin > 0)
		gpio_set_value(info->enable_pin, 0); /* hw control enable chg */

	sgm41513_set_vindpm_os(0);  // set vindpm as 4.2V
	sgm41513_set_vindpm(3);
	sgm41513_set_vindpm_irq_mask(1);  // mask vindpm int
	sgm41513_set_iindpm_irq_mask(1);  // mask iindpm int
	sgm41513_set_stat_pin(3); // disable stat pin
	sgm41513_write_byte(SGM41513_CON3, 0xFD);  // set ipre/iterm as 240mA/180mA
	sgm41513_batfet_rst_en(0);  // disable batfet reset
	sgm41513_set_chg_timer(1);  // set chg safety timer as 10 hrs
	sgm41513_set_wd_timer(0x0);  // disable wdg timer
	sgm41513_set_jeita_en(0);  // disable jeita
	sgm41513_set_boostv(1); // set otg vbus as 5V
	sgm41513_set_boost_lim(0); // set otg max current as 1.2A

#if defined(TARGET_BUILD_MMITEST)
	sgm41513_chg_en(0);  // turnoff charging in mmitest when boot up
	sgm41513_en_chg_timer(0); //Disable both fastcharge and precharge timer
#else
	sgm41513_en_chg_timer(1); //Disable both fastcharge and precharge timer
#endif
}

static int sgm41513_driver_probe(struct i2c_client *client,
                                const struct i2c_device_id *id)
{
	int ret = 0;
	struct sgm41513_info *info = NULL;
	struct power_supply_config psy_cfg = {};
	struct regulator_config config = { };
	struct regulator_dev *rdev;

	pr_info("[sgm41513_driver_probe]\n");

	info = devm_kzalloc(&client->dev, sizeof(struct sgm41513_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	new_client = client;
	info->dev = &client->dev;
	ret = sgm41513_parse_dt(info, &client->dev);
	if (ret < 0)
		return ret;

	sgm41513_hw_component_detect();
	if (!g_sgm41513_hw_exist) {
		ret = devm_gpio_request_one(info->dev, info->enable_pin, GPIOF_OUT_INIT_HIGH, "sgm41513_en_pin");
		if (ret) {
			pr_err("%s ce_gpio request fail\n", __func__);
			info->enable_pin = -1;
		}
		return -ENODEV;
	}
	i2c_set_clientdata(client, info);
	mutex_init(&info->irq_complete);

	info->resume_completed = true;
	info->irq_waiting = false;
	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
	                                        &client->dev,
	                                        info,
	                                        &sgm41513_chg_ops,
	                                        &info->chg_props);
	if (IS_ERR_OR_NULL(info->chg_dev)) {
		pr_err("%s: register charger device failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}

	info->psy = power_supply_get_by_name("charger");
	if (!info->psy) {
		pr_err("%s: get power supply failed\n", __func__);
	}

	sgm41513_hw_init(info);

	if (!info->slave_mode) {
		/* otg regulator */
		config.dev = &client->dev;
		config.driver_data = info;
		config.init_data = &sgm41513_vbus_init_data;
		rdev = devm_regulator_register(&client->dev, &sgm41513_otg_rdesc, &config);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			pr_err("otg regulator register err: %d\n", ret);
		}
		alarm_init(&info->wdt_timer, ALARM_BOOTTIME,
			wdt_timer_func);

		info->wdt_workq =
				create_singlethread_workqueue("wdt_workq");
		INIT_WORK(&info->kick_work, wdt_kick_work);
		info->polling_interval = 20;
	}
	device_init_wakeup(info->dev, true);
	memcpy(&info->psy_desc, &sgm41513_charge_desc, sizeof(info->psy_desc));
	psy_cfg.drv_data = info;
	psy_cfg.of_node = client->dev.of_node;
	psy_cfg.supplied_to = sgm41513_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sgm41513_charger_supplied_to);
	info->chg_psy = devm_power_supply_register(&client->dev, &info->psy_desc, &psy_cfg);
	if (IS_ERR(info->chg_psy)) {
		printk("\033[44m %s %d info->chg_psy error\033[0m \n", __func__, __LINE__);
		return -EINVAL;
	}

	if (!info->slave_mode) {
		INIT_DELAYED_WORK(&info->chg_stat_work, chg_stat_work_handler);
		sgm41513_register_irq(info);
	}

	sgm41513_dump_register(info->chg_dev);

#ifdef CONFIG_TCT_DEVICEINFO
	if (g_sgm41513_hw_exist)
	    sprintf(charger_module_name, "SGM41513:SGM:0x%02x:XXXXXXXX",
	       (sgm41513_reg[SGM41513_CONB] & (CONB_PN_MASK << CONB_PN_SHIFT)) >> CONB_PN_SHIFT);
#endif

    return 0;
}

static int sgm41513_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sgm41513_info *info = i2c_get_clientdata(client);

	mutex_lock(&info->irq_complete);
	info->resume_completed = false;
	mutex_unlock(&info->irq_complete);
	pr_info("Suspend successfully!");

	return 0;
}

static int sgm41513_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sgm41513_info *info = i2c_get_clientdata(client);

	if (info->irq_waiting) {
		pr_info_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int sgm41513_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sgm41513_info *info = i2c_get_clientdata(client);


	mutex_lock(&info->irq_complete);
	info->resume_completed = true;
	if (info->irq_waiting) {
		info->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&info->irq_complete);
		sgm41513_irq_handler(client->irq, info);
	} else {
		mutex_unlock(&info->irq_complete);
	}

	pr_info("Resume successfully!");

	return 0;
}


static unsigned char g_reg_value_sgm41513;
static ssize_t sgm41513_access_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
	pr_info("[show_sgm41513_access] 0x%x\n", g_reg_value_sgm41513);
	return sprintf(buf, "%u\n", g_reg_value_sgm41513);
}

static ssize_t sgm41513_access_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	pr_info("[store_sgm41513_access]\n");

	if (buf != NULL && size != 0) {
		pr_info("[store_sgm41513_access] buf is %s and size is %zu\n",
		        buf,
		        size);

		pvalue = (char*) buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16, (unsigned int*) &reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int*) &reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int*) &reg_value);
			pr_info("[store_sgm41513_access] write sgm41513 reg 0x%x with value 0x%x !\n",
			        (unsigned int) reg_address,
			        reg_value);
			ret = sgm41513_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = sgm41513_read_interface(reg_address,
			                             &g_reg_value_sgm41513,
			                             0xFF,
			                             0x0);
			pr_info("[store_sgm41513_access] read sgm41513 reg 0x%x with value 0x%x !\n",
			        (unsigned int) reg_address,
			        g_reg_value_sgm41513);
			pr_info("[store_sgm41513_access] Please use \"cat sgm41513_access\" to get value\n");
		}
	}
	return size;
}

static DEVICE_ATTR_RW(sgm41513_access); /* 664 */

static int sgm41513_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;
	pr_info("******** sgm41513_user_space_probe!! ********\n");
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_sgm41513_access);
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

static const struct i2c_device_id sgm41513_i2c_id[] = {
	{ "sgm41513", 0 },
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id sgm41513_of_match[] = {
	{ .compatible = "sgm,sgm41513", },
	{ },
};
#endif

static const struct dev_pm_ops sgm41513_pm_ops = {
	.resume			= sgm41513_resume,
	.suspend_noirq 	= sgm41513_suspend_noirq,
	.suspend		= sgm41513_suspend,
};

static struct i2c_driver sgm41513_driver = {
	.driver = {
		.name = "sgm41513",
	#ifdef CONFIG_OF
		.of_match_table = sgm41513_of_match,
	#endif
		.pm	= &sgm41513_pm_ops,
	},
	.probe = sgm41513_driver_probe,
	.id_table = sgm41513_i2c_id,
};

static int __init sgm41513_init(void)
{
	int ret = 0;

	pr_info("[sgm41513_init] init start with i2c DTS");

	if (i2c_add_driver(&sgm41513_driver) != 0) {
		pr_info("[sgm41513_init] failed to register sgm41513 i2c driver.\n");
	} else {
		pr_info("[sgm41513_init] Success to register sgm41513 i2c driver.\n");
	}

	/* sgm41513 user space access interface */
	ret = platform_device_register(&sgm41513_user_space_device);
	if (ret) {
		pr_info("****[sgm41513_init] Unable to device register(%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&sgm41513_user_space_driver);
	if (ret) {
		pr_info("****[sgm41513_init] Unable to register driver (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit sgm41513_exit(void)
{
	i2c_del_driver(&sgm41513_driver);
}
module_init (sgm41513_init);
module_exit (sgm41513_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C sgm41513 Driver");
MODULE_AUTHOR("will cai <will.cai@mediatek.com>");
