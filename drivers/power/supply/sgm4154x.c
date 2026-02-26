
#include <linux/types.h>
#include <linux/init.h>     /* For init/exit macros */
#include <linux/module.h>   /* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
#include <mt-plat/mtk_boot.h>
#include <mt-plat/upmu_common.h>

#include "charger_class.h"

#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define SGM4154X_REG_NUM            0x10

/* Reg0x00 */
#define SGM4154X_CON0               0x00
#define CON0_EN_HIZ_MASK			0x01
#define CON0_EN_HIZ_SHIFT			7

#define	CON0_STAT_IMON_CTRL_MASK	0x03
#define	CON0_STAT_IMON_CTRL_SHIFT 5

#define CON0_IINLIM_MASK   0x1F
#define CON0_IINLIM_SHIFT  0

/* Reg0x01 */
#define SGM4154X_CON1               0x01
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
#define SGM4154X_CON2               0x02
#define CON2_BOOST_LIM_MASK   0x01
#define CON2_BOOST_LIM_SHIFT  7

#define CON2_ICHG_MASK    0x3F
#define CON2_ICHG_SHIFT   0

/* Reg0x03 */
#define SGM4154X_CON3               0x03
#define CON3_IPRECHG_MASK   0x0F
#define CON3_IPRECHG_SHIFT  4

#define CON3_ITERM_MASK           0x0F
#define CON3_ITERM_SHIFT          0

/* Reg0x04 */
#define SGM4154X_CON4               0x04
#define CON4_VREG_MASK     0x1F
#define CON4_VREG_SHIFT    3

#define	CON4_TOPOFF_TIMER_MASK 0x03
#define	CON4_TOPOFF_TIMER_SHIFT 1

#define CON4_VRECHG_MASK    0x01
#define CON4_VRECHG_SHIFT   0

/* Reg0x05 */
#define SGM4154X_CON5               0x05
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

/* Reg0x06 */
#define SGM4154X_CON6               0x06
#define	CON6_OVP_MASK		0x03
#define	CON6_OVP_SHIFT		6

#define	CON6_BOOSTV_MASK	0x3
#define	CON6_BOOSTV_SHIFT	4

#define	CON6_VINDPM_MASK	0x0F
#define	CON6_VINDPM_SHIFT	0

/* Reg0x07, different */
#define SGM4154X_CON7               0x07
#define	CON7_IINDET_EN_MASK	0x01
#define	CON7_IINDET_EN_SHIFT	7

#define CON7_TMR2X_EN_MASK      0x01
#define CON7_TMR2X_EN_SHIFT     6

#define CON7_BATFET_Disable_MASK      0x01
#define CON7_BATFET_Disable_SHIFT     5

#define	CON7_BATFET_DLY_MASK		0x01
#define	CON7_BATFET_DLY_SHIFT		3

#define	CON7_BATFET_RST_EN_MASK		0x01
#define	CON7_BATFET_RST_EN_SHIFT	2

#define	CON7_VDPM_BAT_TRACK_MASK	0x03
#define	CON7_VDPM_BAT_TRACK_SHIFT	0

/* Reg0x08 */
#define SGM4154X_CON8               0x08
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
#define SGM4154X_CON9               0x09
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
#define SGM4154X_CONA               0x0A
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
#define SGM4154X_CONB               0x0B
#define CONB_REG_RST_MASK     0x01
#define CONB_REG_RST_SHIFT    7

#define CONB_PN_MASK		0x0F
#define CONB_PN_SHIFT		3
#define SGM4154X_PN_41541_ID		0x0C
#define SGM4154X_PN_41542_ID		0x0D

#define CONB_Rev_MASK           0x03
#define CONB_Rev_SHIFT          0

/* Reg0x0C */
#define SGM4154X_CONC               0x0C

/* Reg0x0D */
#define SGM4154X_COND               0x0D
#define COND_EN_PUMPX_MASK   0x1
#define COND_EN_PUMPX_SHIFT  7

#define COND_PUMPX_UP_MASK         0x1
#define COND_PUMPX_UP_SHIFT        6

#define COND_PUMPX_DN_MASK         0x1
#define COND_PUMPX_DN_SHIFT        5

#define COND_JEITA_EN_MASK         0x1
#define COND_JEITA_EN_SHIFT        0

/* Reg0x0E */
#define SGM4154X_CONE               0x0E

/* Reg0x0F */
#define SGM4154X_CONF               0x0F
#define CONF_VINDPM_OS_MASK         0x3
#define CONF_VINDPM_OS_SHIFT        0


/*SGM4154X REG04 VREG[7:3]*/
static const unsigned int VBAT_CV_VTH[] = {
    3856000, 3888000, 3920000, 3952000,
	3984000, 4016000, 4048000, 4080000,
	4112000, 4144000, 4176000, 4208000,
	4240000, 4272000, 4304000, 4336000,
	4368000, 4400000, 4432000, 4464000,
	4496000, 4528000, 4560000, 4592000,
	4624000
};

/*SGM4154X REG02 ICHG[5:0]*/
static const unsigned int CS_VTH[] = {
    0, 6000, 12000, 18000, 24000,
	30000, 36000, 42000, 48000, 54000,
	60000, 66000, 72000, 78000, 84000,
	90000, 96000, 102000, 108000, 114000,
	120000, 126000, 132000, 138000, 144000,
	150000, 156000, 162000, 168000, 174000,
	180000, 186000, 192000, 198000, 204000,
	210000, 216000, 222000, 228000, 234000,
	240000, 246000, 252000, 258000, 264000,
	270000, 276000, 282000, 288000, 294000,
	300000, 306000, 312000, 318000, 324000,
	330000, 336000, 342000, 348000, 354000,
	360000, 366000, 372000, 378000
};

/*SGM4154X REG00 IINLIM[4:0]*/
static const unsigned int INPUT_CS_VTH[] = {
	10000, 20000, 30000, 40000,
	50000, 60000, 70000, 80000,
	90000, 100000, 110000, 120000,
	130000, 140000, 150000, 160000,
	170000, 180000, 190000, 200000,
	210000, 220000, 230000, 250000,
	260000, 270000, 280000, 290000,
	300000, 310000, 320000
};

/* SGM4154X REG06 VINDPM[3:0] */
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

struct sgm4154x_info {
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct charger_properties chg_props;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	int irq;
	int intr_gpio;
	int ceb_gpio;
	struct power_supply *chg_psy;
	struct power_supply_desc psy_desc;
	struct delayed_work chg_stat_work;
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

static unsigned char sgm4154x_reg[SGM4154X_REG_NUM] = {
    0
};

static DEFINE_MUTEX (sgm4154x_i2c_access);
static DEFINE_MUTEX (sgm4154x_access_lock);

static bool g_sgm4154x_hw_exist = false;

static unsigned int sgm4154x_read_byte(unsigned char cmd, unsigned char *returnData)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&sgm4154x_i2c_access);

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

	mutex_unlock(&sgm4154x_i2c_access);

	return ret == xfers ? 1 : -1;
}

static unsigned int sgm4154x_write_byte(unsigned char cmd, unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&sgm4154x_i2c_access);

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

	mutex_unlock(&sgm4154x_i2c_access);

	return ret == xfers ? 1 : -1;
}

static unsigned int sgm4154x_read_interface(unsigned char RegNum,
                                    unsigned char *val,
                                    unsigned char MASK,
                                    unsigned char SHIFT)
{
	unsigned char sgm4154x_reg = 0;
	unsigned int ret = 0;

	ret = sgm4154x_read_byte(RegNum, &sgm4154x_reg);

	chr_debug("[sgm4154x_read_interface] Reg[%x]=0x%x\n", RegNum, sgm4154x_reg);

	sgm4154x_reg &= (MASK << SHIFT);
	*val = (sgm4154x_reg >> SHIFT);

	chr_debug("[sgm4154x_read_interface] val=0x%x\n", *val);

	return ret;
}

static unsigned int sgm4154x_config_interface(unsigned char RegNum,
                                      unsigned char val,
                                      unsigned char MASK,
                                      unsigned char SHIFT)
{
	unsigned char sgm4154x_reg = 0;
	unsigned char sgm4154x_reg_ori = 0;
	unsigned int ret = 0;

	mutex_lock(&sgm4154x_access_lock);
	ret = sgm4154x_read_byte(RegNum, &sgm4154x_reg);

	sgm4154x_reg_ori = sgm4154x_reg;
	sgm4154x_reg &= ~(MASK << SHIFT);
	sgm4154x_reg |= (val << SHIFT);

	ret = sgm4154x_write_byte(RegNum, sgm4154x_reg);
	mutex_unlock(&sgm4154x_access_lock);

	chr_debug("[sgm4154x_config_interface] write Reg[%x]=0x%x from 0x%x\n",
	          RegNum,
	          sgm4154x_reg,
	          sgm4154x_reg_ori);
	return ret;
}

/* CON0---------------------------------------------------- */
static void sgm4154x_set_en_hiz(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON0),
                               (unsigned char) (val),
                               (unsigned char) (CON0_EN_HIZ_MASK),
                               (unsigned char) (CON0_EN_HIZ_SHIFT));
}

static void sgm4154x_set_stat_pin(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON0),
                               (unsigned char) (val),
                               (unsigned char) (CON0_STAT_IMON_CTRL_MASK),
                               (unsigned char) (CON0_STAT_IMON_CTRL_SHIFT));
}

static void sgm4154x_set_iinlim(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON0),
                               (val),
                               (unsigned char) (CON0_IINLIM_MASK),
                               (unsigned char) (CON0_IINLIM_SHIFT));
}

/* CON1---------------------------------------------------- */
static void sgm4154x_wd_reset(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON1),
                               (val),
                               (unsigned char) (CON1_WDT_RST_MASK),
                               (unsigned char) (CON1_WDT_RST_SHIFT));
}

static void sgm4154x_otg_en(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON1),
			                   (val),
			                   (unsigned char) (CON1_OTG_CONFIG_MASK),
			                   (unsigned char) (CON1_OTG_CONFIG_SHIFT));
}

static int sgm4154x_is_otg_en(bool *en)
{
	unsigned char val = 0;
	sgm4154x_read_interface((unsigned char) (SGM4154X_CON1),
                             (&val),
                             (unsigned char) (CON1_OTG_CONFIG_MASK),
                             (unsigned char) (CON1_OTG_CONFIG_SHIFT));
	*en = (val == 0 ? false : true);
	return 0;
}

static void sgm4154x_chg_en(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON1),
                               (val),
                               (unsigned char) (CON1_CHG_CONFIG_MASK),
                               (unsigned char) (CON1_CHG_CONFIG_SHIFT));
}

/* CON2--------------------------------------------------- */
static void sgm4154x_set_boost_lim(unsigned int val)
{
    sgm4154x_config_interface((unsigned char) (SGM4154X_CON2),
								(unsigned char) (val),
								(unsigned char) (CON2_BOOST_LIM_MASK),
								(unsigned char) (CON2_BOOST_LIM_SHIFT));
}

static void sgm4154x_set_ichg(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON2),
                               (unsigned char) (val),
                               (unsigned char) (CON2_ICHG_MASK),
                               (unsigned char) (CON2_ICHG_SHIFT));
}

/* CON3---------------------------------------------------- */
static void sgm4154x_set_iprechg(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON3),
                               (val),
                               (unsigned char) (CON3_IPRECHG_MASK),
                               (unsigned char) (CON3_IPRECHG_SHIFT));
}

static void sgm4154x_set_iterml(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON3),
                               (val),
                               (unsigned char) (CON3_ITERM_MASK),
                               (unsigned char) (CON3_ITERM_SHIFT));
}

/* CON4---------------------------------------------------- */
static void sgm4154x_set_vreg(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON4),
                               (unsigned char) (val),
                               (unsigned char) (CON4_VREG_MASK),
                               (unsigned char) (CON4_VREG_SHIFT));
}

static unsigned int sgm4154x_get_vreg(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = sgm4154x_read_interface((unsigned char) (SGM4154X_CON4),
                                 (&val),
                                 (unsigned char) (CON4_VREG_MASK),
                                 (unsigned char) (CON4_VREG_SHIFT));
    return val;
}

/* CON5---------------------------------------------------- */
static void sgm4154x_set_wd_timer(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON5),
                               (unsigned char) (val),
                               (unsigned char) (CON5_WATCHDOG_MASK),
                               (unsigned char) (CON5_WATCHDOG_SHIFT));
}

static void sgm4154x_set_chg_timer(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON5),
                               (unsigned char) (val),
                               (unsigned char) (CON5_CHG_TIMER_MASK),
                               (unsigned char) (CON5_CHG_TIMER_SHIFT));
}

static void sgm4154x_en_chg_timer(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON5),
                               (unsigned char) (val),
                               (unsigned char) (CON5_EN_TIMER_MASK),
                               (unsigned char) (CON5_EN_TIMER_SHIFT));
}

static unsigned int sgm4154x_get_chg_timer_enable(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = sgm4154x_read_interface((unsigned char) (SGM4154X_CON5),
                                 &val,
                                 (unsigned char) (CON5_EN_TIMER_MASK),
                                 (unsigned char) (CON5_EN_TIMER_SHIFT));

    return val;
}

/* CON6--------------------------------------------------- */
static void sgm4154x_set_ovp(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON6),
							(unsigned char) (val),
							(unsigned char) (CON6_OVP_MASK),
							(unsigned char) (CON6_OVP_SHIFT)
							);

}

static unsigned int sgm4154x_get_vindpm(void)
{
    unsigned char val = 0;
	sgm4154x_read_interface((unsigned char) (SGM4154X_CON6),
	                         (&val),
	                         (unsigned char) (CON6_VINDPM_MASK),
	                         (unsigned char) (CON6_VINDPM_SHIFT));
    return val;
}

static void sgm4154x_set_vindpm(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON6),
                               (unsigned char) (val),
                               (unsigned char) (CON6_VINDPM_MASK),
                               (unsigned char) (CON6_VINDPM_SHIFT));
}

static void sgm4154x_set_boostv(unsigned int val)
{
    sgm4154x_config_interface((unsigned char) (SGM4154X_CON6),
								(unsigned char) (val),
								(unsigned char) (CON6_BOOSTV_MASK),
								(unsigned char) (CON6_BOOSTV_SHIFT));
}

/* CON7---------------------------------------------------- */
static void sgm4154x_batfet_rst_en(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CON7),
	                         (unsigned char) (val),
	                         (unsigned char) (CON7_BATFET_RST_EN_MASK),
	                         (unsigned char) (CON7_BATFET_RST_EN_SHIFT));
}

/* CON8--------------------------------------------------- */
static unsigned int sgm4154x_get_chrg_state(void)
{
	unsigned char val = 0;
	sgm4154x_read_interface((unsigned char) (SGM4154X_CON8),
                             (&val),
                             (unsigned char) (CON8_CHRG_STAT_MASK),
                             (unsigned char) (CON8_CHRG_STAT_SHIFT));
	return val;
}

/* CON9--------------------------------------------------- */
static unsigned int sgm4154x_get_wdt_state(void)
{
	unsigned char val = 0;
	sgm4154x_read_interface((unsigned char) (SGM4154X_CON9),
                             (&val),
                             (unsigned char) (CON9_WATCHDOG_FAULT_MASK),
                             (unsigned char) (CON9_WATCHDOG_FAULT_SHIFT));
	return val;
}

/* CONA---------------------------------------------------- */
static void sgm4154x_set_vindpm_irq_mask(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CONA),
                               (unsigned char) (val),
                               (unsigned char) (CONA_VINDPM_INT_MASK),
                               (unsigned char) (CONA_VINDPM_INT_SHIFT));
}

static void sgm4154x_set_iindpm_irq_mask(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CONA),
                               (unsigned char) (val),
                               (unsigned char) (CONA_IINDPM_INT_MASK),
                               (unsigned char) (CONA_IINDPM_INT_SHIFT));
}

/* CONB---------------------------------------------------- */

/* CONC---------------------------------------------------- */

/* COND---------------------------------------------------- */
static void sgm4154x_set_jeita_en(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_COND),
                               (unsigned char) (val),
                               (unsigned char) (COND_JEITA_EN_MASK),
                               (unsigned char) (COND_JEITA_EN_SHIFT));
}

static void sgm4154x_en_pumpx(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_COND),
                               (unsigned char) (val),
                               (unsigned char) (COND_EN_PUMPX_MASK),
                               (unsigned char) (COND_EN_PUMPX_SHIFT));
}

/* CONF---------------------------------------------------- */
static unsigned int sgm4154x_get_vindpm_os(void)
{
    unsigned char val = 0;
	sgm4154x_read_interface((unsigned char) (SGM4154X_CONF),
	                         (&val),
	                         (unsigned char) (CONF_VINDPM_OS_MASK),
	                         (unsigned char) (CONF_VINDPM_OS_SHIFT));
    return val;
}

static void sgm4154x_set_vindpm_os(unsigned int val)
{
	sgm4154x_config_interface((unsigned char) (SGM4154X_CONF),
                               (unsigned char) (val),
                               (unsigned char) (CONF_VINDPM_OS_MASK),
                               (unsigned char) (CONF_VINDPM_OS_SHIFT));
}

static void sgm4154x_pumpx_up(unsigned int val)
{
	unsigned int ret = 0;

	/* Input current limit = 500 mA, changes after PE+ detection */
	sgm4154x_set_iinlim(4);

	/* CC mode current = 2040 mA */
	sgm4154x_set_ichg(0x22);
	sgm4154x_chg_en(1);
	sgm4154x_en_pumpx(1);

	if (val == 1) {
		ret = sgm4154x_config_interface((unsigned char) (SGM4154X_COND),
		                               (unsigned char) (1),
		                               (unsigned char) (COND_PUMPX_UP_MASK),
		                               (unsigned char) (COND_PUMPX_UP_SHIFT));
	}
	else {
		ret = sgm4154x_config_interface((unsigned char) (SGM4154X_CON9),
		                               (unsigned char) (1),
		                               (unsigned char) (COND_PUMPX_DN_MASK),
		                               (unsigned char) (COND_PUMPX_DN_SHIFT));
	}

	/* Input current limit = 500 mA, changes after PE+ detection */
	sgm4154x_set_iinlim(4);

	/* CC mode current = 2040 mA */
	sgm4154x_set_ichg(0x22);
	msleep(3000);
}

static void sgm4154x_hw_component_detect(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

	ret = sgm4154x_read_interface(SGM4154X_CONB,
					&val, CONB_PN_MASK, CONB_PN_SHIFT);
	switch (val) {
		case SGM4154X_PN_41541_ID:
		case SGM4154X_PN_41542_ID:
			g_sgm4154x_hw_exist = true;
			pr_err("[%s] charger IC is SGM4154X(%x)\n", __func__, val);
			break;
		default:
			pr_err("[%s] charger IC is Unknown\n", __func__);
			break;
	}
}

static int sgm4154x_enable_charging(struct charger_device *chg_dev, bool en)
{
	int status = 0;

	if (en) {
		sgm4154x_set_en_hiz(0x0);
		sgm4154x_chg_en(en);
	}
	else {
		sgm4154x_chg_en(en);
	}

	return status;
}

static int sgm4154x_get_current(struct charger_device *chg_dev, u32 *ichg)
{
    unsigned char val = 0;
	unsigned char array_size = ARRAY_SIZE(CS_VTH);

	/* Get current level */
	sgm4154x_read_interface((unsigned char) (SGM4154X_CON2),
                             (&val),
                             (unsigned char) (CON2_ICHG_MASK),
                             (unsigned char) (CON2_ICHG_SHIFT));

	if (val < array_size) {
		*ichg = CS_VTH[val] * 10;  // uA
		return 0;
	} else
		return -EINVAL;
}

static int sgm4154x_set_current(struct charger_device *chg_dev,
                               u32 current_value)
{
    unsigned int status = true;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	pr_info("#### FCC = %d\n", current_value);

	current_value /= 10;
	array_size = ARRAY_SIZE(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size,
			  current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size,
			 set_chr_current);
	pr_info("#### %s register_value = %d\n", __func__,
			register_value);

	sgm4154x_set_ichg(register_value);
	return status;
}

static int sgm4154x_get_input_current(struct charger_device *chg_dev, u32 *aicr)
{
	*aicr = 0;
	return -EINVAL;
}

static int sgm4154x_set_input_current(struct charger_device *chg_dev,
                                     u32 current_value)
{
	int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	mutex_lock(&g_input_current_mutex);
	current_value /= 10;
	pr_info("### ICL = %d\n", current_value);

	array_size = ARRAY_SIZE(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH,
	                                         array_size,
	                                         current_value);
	register_value = charging_parameter_to_value(INPUT_CS_VTH,
	                                             array_size,
	                                             set_chr_current);
	sgm4154x_set_iinlim(register_value);
	mutex_unlock(&g_input_current_mutex);

	pr_info("### %s register_value = %d\n", __func__,
			register_value);

    /*For USB_IF compliance test only when USB is in suspend(Ibus < 2.5mA) or unconfigured(Ibus < 70mA) states*/
#ifdef CONFIG_USBIF_COMPLIANCE
	if (current_value < 10000) {
		sgm4154x_set_vindpm_os(3);  // set vindpm as 10.5V
		sgm4154x_set_vindpm(0);
	} else {
		sgm4154x_set_vindpm_os(0);  // set vindpm as 4.2V
		sgm4154x_set_vindpm(3);
	}
#endif

    return status;
}

static int sgm4154x_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
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
	sgm4154x_set_vreg(register_value);
	return status;
}

static int sgm4154x_get_battery_vreg(struct charger_device *chg_dev, u32 *cv)
{
	u8 reg_cv = 0;
	reg_cv = sgm4154x_get_vreg();

	if (reg_cv > 0) {
		*cv = (reg_cv > 0x18) ? 4624000 : VBAT_CV_VTH[reg_cv];
		return 0;
	}
	else {
		*cv = 0;
		return -1;
	}
}

static int sgm4154x_reset_watch_dog_timer(struct charger_device *chg_dev)
{
	sgm4154x_wd_reset(1);
	return 0;
}

static int sgm4154x_is_power_path_enable(struct charger_device *chg_dev,
                                            bool *en)
{
	u32 vindpm_os = 0, vindpm_val = 0;

	vindpm_os = sgm4154x_get_vindpm_os();
	vindpm_val = sgm4154x_get_vindpm();

	if (vindpm_os == 3 && vindpm_val == 0) {  // 10.5V
		*en = false;
	}
	return 0;
}

static int sgm4154x_set_vindpm_voltage(struct charger_device *chg_dev,
                                      u32 vindpm)
{
	u32 array_size, vindpm_os = 0;
	bool is_power_path_enable = true;
	int ret = 0;

	/* Since sgm4154x uses vindpm to turn off power path
	    If power path is disabled, do not adjust mivr	*/
	ret = sgm4154x_is_power_path_enable(chg_dev, &is_power_path_enable);
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

	sgm4154x_set_vindpm_os(vindpm_os);
	sgm4154x_set_vindpm(vindpm);
	return 0;
}

static int sgm4154x_get_charging_status(struct charger_device *chg_dev,
                                       int *status)
{
	unsigned char val = 0;
	*status = POWER_SUPPLY_STATUS_UNKNOWN;

	sgm4154x_read_interface(SGM4154X_CON8,
							&val,
							CON8_CHRG_STAT_MASK,
							CON8_CHRG_STAT_SHIFT);
	switch (val) {
	case 0x00:
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case 0x01:
	case 0x02:
		*status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case 0x03:
		if (now_cv < max_cv) {  // sync with sgm4154x_is_charging_done
			pr_err("now_cv(%d) < max_cv(%d), report charging\n",
					now_cv, max_cv);
			*status = POWER_SUPPLY_STATUS_CHARGING;
		} else {
			*status = POWER_SUPPLY_STATUS_FULL;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int sgm4154x_is_charging_done(struct charger_device *chg_dev,
                                       bool *is_done)
{
	int status = 0;
	unsigned char val = 0;

	sgm4154x_read_interface(SGM4154X_CON8,
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

static int sgm4154x_enable_otg(struct charger_device *chg_dev, bool en)
{
	sgm4154x_otg_en(en);
	return 0;
}

static int sgm4154x_enable_safetytimer(struct charger_device *chg_dev, bool en)
{
	sgm4154x_en_chg_timer(en);
	return 0;
}

static int sgm4154x_get_is_safetytimer_enable(struct charger_device *chg_dev,
                                             bool *en)
{
	u32 reg_safetytimer = 0;
	reg_safetytimer = sgm4154x_get_chg_timer_enable();
	*en = (reg_safetytimer) ? true : false;
	return 0;
}

static int sgm4154x_set_ta_current_pattern(struct charger_device *chg_dev,
                                          bool is_increase)
{
	sgm4154x_pumpx_up(is_increase);
	pr_err("%s: Pumping up adaptor...", __func__);
	return 0;
}

static int sgm4154x_set_pep20_efficiency_table(struct charger_device *chg_dev)
{
	return 0;
}

static int sgm4154x_set_ta_reset(struct charger_device *chg_dev)
{
	sgm4154x_set_vindpm_os(0);  // set vindpm as 4.2V
	sgm4154x_set_vindpm(3);

	sgm4154x_set_ichg(8);  // Ibatt = 480 mA
	sgm4154x_set_iinlim(0);  // ibus = 100mA
	msleep(250);
	sgm4154x_set_iinlim(6); // ibus = 700mA
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

static int sgm4154x_set_ta20_current_pattern(struct charger_device *chg_dev,
                                            u32 chr_vol)
{
	int value;
	int i, j = 0;
	int flag;

	sgm4154x_set_vindpm_os(0);  // set vindpm as 4.2V
	sgm4154x_set_vindpm(3);
	sgm4154x_set_ichg(8);  // Ibatt = 480 mA

	usleep_range(1000, 1200);

	value = (chr_vol - 5500000) / 500000;
	sgm4154x_set_iinlim(0x0);  // set ibus 100mA
	msleep(70);

	get_monotonic_boottime(&ptime[j++]);
	for (i = 4; i >= 0; i--) {
			flag = value & (1 << i);
			if (flag == 0) {
			sgm4154x_set_iinlim(6); // ibus = 700mA

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
			sgm4154x_set_iinlim(0x0);
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
			sgm4154x_set_iinlim(6); // ibus = 700mA
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
			sgm4154x_set_iinlim(0x0);
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

	sgm4154x_set_iinlim(6); // ibus = 700mA
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
	sgm4154x_set_iinlim(0x0);
	msleep(30);
	sgm4154x_set_iinlim(6); // ibus = 700mA

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

static int sgm4154x_dump_register(struct charger_device *chg_dev)
{
	unsigned char i = 0;

	for (i = 0; i < SGM4154X_REG_NUM; i++) {
		sgm4154x_read_byte(i, &sgm4154x_reg[i]);
		chr_debug("[sgm4154x reg@][0x%x]=0x%x ", i, sgm4154x_reg[i]);
	}
	return 0;
}

static void chg_stat_work_handler(struct work_struct *data)
{
	struct sgm4154x_info *info = NULL;
	static unsigned char last_stat = 0xFF;
	unsigned char curr_stat = 0;
	int ret = 0;

	pr_info("%s\n", __func__);
	info = container_of(to_delayed_work(data),
					struct sgm4154x_info, chg_stat_work);
	if (IS_ERR_OR_NULL(info)) {
		pr_err("[%s]: NULL pointer\n", __func__);
		return;
	};

	ret = sgm4154x_read_interface(SGM4154X_CON8,
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

static irqreturn_t sgm4154x_irq_handler(int irq, void *data)
{
	struct sgm4154x_info *info = (struct sgm4154x_info *)data;
	bool en = false;
	pr_info("%s\n", __func__);

	/* Skip irq if in OTG mode */
	sgm4154x_is_otg_en(&en);
	if (en)
		return IRQ_HANDLED;

	schedule_delayed_work(&info->chg_stat_work,
				msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

static int sgm4154x_register_irq(struct sgm4154x_info *info)
{
	int ret = 0;

	if (info->intr_gpio <= 0) {
		dev_err(info->dev, "%s intr_gpio invalid(%d)\n",
				__func__, info->intr_gpio);
		return -EINVAL;
	}

	ret = devm_gpio_request_one(info->dev, info->intr_gpio, GPIOF_DIR_IN,
			devm_kasprintf(info->dev, GFP_KERNEL,
			"sgm4154x_intr_gpio.%s", dev_name(info->dev)));
	if (ret < 0) {
		dev_err(info->dev, "%s gpio request fail(%d)\n",
				      __func__, ret);
		return ret;
	}

	info->irq = gpio_to_irq(info->intr_gpio);
	if (info->irq < 0) {
		dev_err(info->dev, "%s gpio2irq fail(%d)\n",
				      __func__, info->irq);
		return -EINVAL;
	}
	pr_info("%s: irq = %d\n", __func__, info->irq);

	/* Request threaded IRQ */
	ret = devm_request_threaded_irq(info->dev,
                                info->irq,
                                NULL,
                                sgm4154x_irq_handler,
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

static int sgm4154x_parse_dt(struct sgm4154x_info *info, struct device *dev)
{
	int ret = 0;
	struct device_node *np = dev->of_node;

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
		info->chg_props.alias_name = "sgm4154x";
		pr_warn("%s: no alias name\n", __func__);
	}

	if (of_property_read_string(np, "eint_name", &info->eint_name) < 0) {
		info->eint_name = "chr_stat";
		pr_warn("%s: no eint name\n", __func__);
	}

	ret = of_get_named_gpio(np, "sgm4154x,intr_gpio", 0);
	if (ret < 0) {
		dev_err(info->dev, "%s no sgm4154x,intr_gpio(%d)\n",
					  __func__, ret);
		info->intr_gpio = -1;
	} else {
		info->intr_gpio = ret;
	}

	ret = of_get_named_gpio(np, "sgm4154x,en_gpio", 0);
	if (ret < 0) {
		dev_err(info->dev, "%s no sgm4154x,en_gpio(%d)\n",
					__func__, ret);
		info->ceb_gpio = -1;
	} else {
		info->ceb_gpio = ret;
	}

	if (info->ceb_gpio > 0) {
		gpio_direction_output(info->ceb_gpio, 0); /* hw control enable chg */
	}

	return 0;
}

static int sgm4154x_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct sgm4154x_info *info = NULL;

	if (IS_ERR_OR_NULL(chg_dev))
		return -EINVAL;

	info = dev_get_drvdata(&chg_dev->dev);
	if (IS_ERR_OR_NULL(info) || IS_ERR_OR_NULL(info->chg_psy))
		return -EINVAL;

	printk("%s: event = %d\n", __func__, event);
	power_supply_changed(info->chg_psy);
	return 0;
}

static int sgm4154x_charger_plug_in(struct charger_device *chg_dev)
{
	if (0x1 == sgm4154x_get_wdt_state()) {
		chr_err("%s wdt timeout,reset some reg\n", __func__);
		sgm4154x_wd_reset(1);
		sgm4154x_set_stat_pin(3);
		sgm4154x_set_ovp(2);  // set OVP to 10.5V
		sgm4154x_batfet_rst_en(0); // disable batfet reset
		sgm4154x_set_iterml(2); //180 mA
		sgm4154x_set_iprechg(2); //180mA
		sgm4154x_set_vindpm_os(0);  // set vindpm as 4.2V
		sgm4154x_set_vindpm(3);
		sgm4154x_set_vindpm_irq_mask(1);  // mask vindpm int
		sgm4154x_set_iindpm_irq_mask(1);  // mask iindpm int
		sgm4154x_set_jeita_en(0);  // disable jeita
	}
	return 0;
}

static int sgm4154x_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = 0;
	return 0;
}

static int sgm4154x_get_min_aicr(struct charger_device *chg_dev, u32 *uA)
{
	*uA = 100000;
	return 0;
}

static struct charger_ops sgm4154x_chg_ops = {
#if 0
	.enable_hz = sgm4154x_enable_hz,
#endif

	/* Normal charging */
	.plug_in = sgm4154x_charger_plug_in,
	.dump_registers = sgm4154x_dump_register,
	.enable = sgm4154x_enable_charging,
	.get_charging_current = sgm4154x_get_current,
	.set_charging_current = sgm4154x_set_current,
	.get_input_current = sgm4154x_get_input_current,
	.set_input_current = sgm4154x_set_input_current,
	.get_constant_voltage = sgm4154x_get_battery_vreg,
	.set_constant_voltage = sgm4154x_set_cv_voltage,
	.kick_wdt = sgm4154x_reset_watch_dog_timer,
	.set_mivr = sgm4154x_set_vindpm_voltage,

	.is_charging_done = sgm4154x_is_charging_done,
	.get_chg_status = sgm4154x_get_charging_status,

	.get_min_charging_current = sgm4154x_get_min_ichg,
	.get_min_input_current = sgm4154x_get_min_aicr,

	/* Safety timer */
	.enable_safety_timer = sgm4154x_enable_safetytimer,
	.is_safety_timer_enabled = sgm4154x_get_is_safetytimer_enable,

	/* OTG */
	.enable_otg = sgm4154x_enable_otg,

	/* PE+/PE+20 */
	.send_ta_current_pattern = sgm4154x_set_ta_current_pattern,
	.set_pe20_efficiency_table = sgm4154x_set_pep20_efficiency_table,
	.send_ta20_current_pattern = sgm4154x_set_ta20_current_pattern,
	.reset_ta = sgm4154x_set_ta_reset,

	/* Event */
	.event = sgm4154x_do_event,
};

/* TODO: before enable otg function, make slave charger hiz mode or forbid chg? */
static int sgm4154x_boost_enable(struct regulator_dev *rdev)
{
	pr_err("sgm4154x otg regulator enable\n");
	sgm4154x_chg_en(0);
	sgm4154x_otg_en(1);
	return 0;
}

static int sgm4154x_boost_disable(struct regulator_dev *rdev)
{
	pr_err("sgm4154x otg regulator disable\n");
	sgm4154x_otg_en(0);
	sgm4154x_chg_en(1);
	return 0;
}

static int sgm4154x_boost_is_enabled(struct regulator_dev *rdev)
{
	bool en;
	sgm4154x_is_otg_en(&en);
	return en;
}

static const struct regulator_ops sgm4154x_chg_otg_ops = {
	.enable = sgm4154x_boost_enable,
	.disable = sgm4154x_boost_disable,
	.is_enabled = sgm4154x_boost_is_enabled,
};

static const struct regulator_desc sgm4154x_otg_rdesc = {
	.name = "usb-otg-vbus",
	.ops = &sgm4154x_chg_otg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static const struct regulator_init_data sgm4154x_vbus_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static enum power_supply_usb_type sgm4154x_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static enum power_supply_property sgm4154x_charge_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_USB_TYPE,
};

#define VBUS_THR 4000
static int sgm4154x_get_online(struct sgm4154x_info *bq, union power_supply_propval *val)
{
	static struct power_supply *chg_psy;
	union power_supply_propval prop;

	if (chg_psy == NULL) {
		chg_psy = power_supply_get_by_name("mtk_charger_type");
	}

	if (IS_ERR_OR_NULL(chg_psy)) {
		dev_notice(bq->dev, "%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	} else {
		power_supply_get_property(chg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		val->intval = prop.intval > VBUS_THR ? 1 : 0;
	}
	return 0;
}
static int sgm4154x_get_charger_type(struct sgm4154x_info *bq, union power_supply_propval *val)
{
	static struct power_supply *chg_psy;

	if (chg_psy == NULL) {
			chg_psy = power_supply_get_by_name("mtk_charger_type");
	}

	if (IS_ERR_OR_NULL(chg_psy)) {
		dev_notice(bq->dev, "%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	} else {
		power_supply_get_property(chg_psy,
				POWER_SUPPLY_PROP_USB_TYPE, val);
		switch (val->intval) {
		case POWER_SUPPLY_USB_TYPE_UNKNOWN:
			bq->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
			break;
		case POWER_SUPPLY_USB_TYPE_SDP:
			bq->psy_desc.type = POWER_SUPPLY_TYPE_USB;
			break;
		case POWER_SUPPLY_USB_TYPE_CDP:
			bq->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
			break;
		case POWER_SUPPLY_USB_TYPE_DCP:
			bq->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		}
	}
	return 0;
}

static int sgm4154x_charge_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct sgm4154x_info *bq = power_supply_get_drvdata(psy);
	int ret = 0;
	unsigned int chg_stat;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = sgm4154x_get_online(bq, val);
		if (ret)
			return -ENODATA;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		chg_stat = sgm4154x_get_chrg_state();
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
		ret = sgm4154x_get_charger_type(bq, val);
		if (ret)
			return -ENODATA;
		break;
	default:
		return 0;
	}

	return ret;
}

static char *sgm4154x_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

static const struct power_supply_desc sgm4154x_charge_desc = {
	.name = "sgm4154x",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sgm4154x_charge_props,
	.num_properties = ARRAY_SIZE(sgm4154x_charge_props),
	.get_property = sgm4154x_charge_get_property,
	.usb_types = sgm4154x_usb_type,
	.num_usb_types = ARRAY_SIZE(sgm4154x_usb_type),

};

#ifdef CONFIG_TCT_DEVICEINFO
extern char charger_module_name[256];
#endif

static void sgm4154x_hw_init(void)
{
	sgm4154x_set_vindpm_os(0);  // set vindpm as 4.2V
	sgm4154x_set_vindpm(3);
	sgm4154x_set_vindpm_irq_mask(1);  // mask vindpm int
	sgm4154x_set_iindpm_irq_mask(1);  // mask iindpm int
	sgm4154x_set_ovp(2);  // set OVP to 10.5V
	sgm4154x_set_stat_pin(3); // disable stat pin
	sgm4154x_write_byte(SGM4154X_CON3, 0x22);  // set ipre/iterm as 180mA/180mA
	sgm4154x_batfet_rst_en(0);  // disable batfet reset
	sgm4154x_set_chg_timer(1);  // set chg safety timer as 10 hrs
	sgm4154x_set_wd_timer(0);  // disable wdg timer
	sgm4154x_set_jeita_en(0);  // disable jeita
	sgm4154x_set_boostv(1); // set otg vbus as 5V
	sgm4154x_set_boost_lim(0); // set otg max current as 1.2A

/* Begin add by jin.wang jira 5195 on 2021.12.30 */
#if defined(TARGET_BUILD_MMITEST)
	sgm4154x_chg_en(0);  // turnoff charging in mmitest when boot up
#endif
/* End add by jin.wang */
}

static int sgm4154x_driver_probe(struct i2c_client *client,
                                const struct i2c_device_id *id)
{
	int ret = 0;
	struct sgm4154x_info *info = NULL;
	struct power_supply_config psy_cfg = {};
	struct regulator_config config = { };
	struct regulator_dev *rdev;

	pr_info("[sgm4154x_driver_probe]\n");

	info = devm_kzalloc(&client->dev, sizeof(struct sgm4154x_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	new_client = client;
	info->dev = &client->dev;

	sgm4154x_hw_component_detect();
	if (!g_sgm4154x_hw_exist)
		return -ENODEV;

	ret = sgm4154x_parse_dt(info, &client->dev);
	if (ret < 0)
		return ret;

	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
	                                        &client->dev,
	                                        info,
	                                        &sgm4154x_chg_ops,
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

	sgm4154x_hw_init();

	/* otg regulator */
	config.dev = &client->dev;
	config.driver_data = info;
	config.init_data = &sgm4154x_vbus_init_data;
	rdev = devm_regulator_register(&client->dev, &sgm4154x_otg_rdesc, &config);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		pr_err("otg regulator register err: %d\n", ret);
	}

	memcpy(&info->psy_desc, &sgm4154x_charge_desc, sizeof(info->psy_desc));
	psy_cfg.drv_data = info;
	psy_cfg.of_node = client->dev.of_node;
	psy_cfg.supplied_to = sgm4154x_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sgm4154x_charger_supplied_to);
	info->chg_psy = devm_power_supply_register(&client->dev, &info->psy_desc, &psy_cfg);
	if (IS_ERR(info->chg_psy)) {
		printk("\033[44m %s %d info->chg_psy error\033[0m \n", __func__, __LINE__);
		return -EINVAL;
	}

	INIT_DELAYED_WORK(&info->chg_stat_work, chg_stat_work_handler);
	sgm4154x_register_irq(info);

	sgm4154x_dump_register(info->chg_dev);

#ifdef CONFIG_TCT_DEVICEINFO
	if (g_sgm4154x_hw_exist)
	    sprintf(charger_module_name, "SGM4154X:SGM:0x%02x:XXXXXXXX",
	       (sgm4154x_reg[SGM4154X_CONB] & (CONB_PN_MASK << CONB_PN_SHIFT)) >> CONB_PN_SHIFT);
#endif

    return 0;
}

static unsigned char g_reg_value_sgm4154x;
static ssize_t sgm4154x_access_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
	pr_info("[show_sgm4154x_access] 0x%x\n", g_reg_value_sgm4154x);
	return sprintf(buf, "%u\n", g_reg_value_sgm4154x);
}

static ssize_t sgm4154x_access_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	pr_info("[store_sgm4154x_access]\n");

	if (buf != NULL && size != 0) {
		pr_info("[store_sgm4154x_access] buf is %s and size is %zu\n",
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
			pr_info("[store_sgm4154x_access] write sgm4154x reg 0x%x with value 0x%x !\n",
			        (unsigned int) reg_address,
			        reg_value);
			ret = sgm4154x_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = sgm4154x_read_interface(reg_address,
			                             &g_reg_value_sgm4154x,
			                             0xFF,
			                             0x0);
			pr_info("[store_sgm4154x_access] read sgm4154x reg 0x%x with value 0x%x !\n",
			        (unsigned int) reg_address,
			        g_reg_value_sgm4154x);
			pr_info("[store_sgm4154x_access] Please use \"cat sgm4154x_access\" to get value\n");
		}
	}
	return size;
}

static DEVICE_ATTR_RW(sgm4154x_access); /* 664 */

static int sgm4154x_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;
	pr_info("******** sgm4154x_user_space_probe!! ********\n");
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_sgm4154x_access);
	return 0;
}

static struct platform_device sgm4154x_user_space_device = {
	.name = "sgm4154x-user",
	.id = -1,
};

static struct platform_driver sgm4154x_user_space_driver = {
	.probe = sgm4154x_user_space_probe,
	.driver = {
		.name = "sgm4154x-user",
	},
};

static const struct i2c_device_id sgm4154x_i2c_id[] = {
	{ "sgm41541", 0 },
	{ "sgm41542", 0 },
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id sgm4154x_of_match[] = {
	{ .compatible = "sgm,sgm41541", },
	{ .compatible = "sgm,sgm41542", },
	{ },
};
#endif

static struct i2c_driver sgm4154x_driver = {
	.driver = {
	.name = "sgm4154x",
#ifdef CONFIG_OF
	.of_match_table = sgm4154x_of_match,
#endif
	},
	.probe = sgm4154x_driver_probe,
	.id_table = sgm4154x_i2c_id,
};

static int __init sgm4154x_init(void)
{
	int ret = 0;

	pr_info("[sgm4154x_init] init start with i2c DTS");

	if (i2c_add_driver(&sgm4154x_driver) != 0) {
		pr_info("[sgm4154x_init] failed to register sgm4154x i2c driver.\n");
	} else {
		pr_info("[sgm4154x_init] Success to register sgm4154x i2c driver.\n");
	}

	/* sgm4154x user space access interface */
	ret = platform_device_register(&sgm4154x_user_space_device);
	if (ret) {
		pr_info("****[sgm4154x_init] Unable to device register(%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&sgm4154x_user_space_driver);
	if (ret) {
		pr_info("****[sgm4154x_init] Unable to register driver (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit sgm4154x_exit(void)
{
	i2c_del_driver(&sgm4154x_driver);
}
module_init (sgm4154x_init);
module_exit (sgm4154x_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C sgm4154x Driver");
MODULE_AUTHOR("will cai <will.cai@mediatek.com>");
