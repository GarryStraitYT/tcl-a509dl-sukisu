
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
#include <linux/of_gpio.h>
#endif
#include <mt-plat/mtk_boot.h>
#include <mt-plat/upmu_common.h>
//#include <mt-plat/charger_type.h>
#include "charger_class.h"

#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#if IS_ENABLED(CONFIG_TCT_CHARGER)
#include "mtk_charger.h"
#endif

/* History 1.0: modified by hailong.chen for task 9168065 on 2020-04-14 */
/* History 2.0: modified by hailong.chen for task 9208823 on 2020-04-16 */
/* History 3.0: modified by hailong.chen for task 9338024 on 2020-04-29 */
/* History 4.0: modified by hailong.chen for task 9338024 on 2020-05-21 */
/* History 5.0: modified by hailong.chen for task 9497181 on 2020-06-06 */
/* History 6.0: modified by hailong.chen for task 9494299 on 2020-06-11 */
/* History 7.0: modified by bin.song.hz  for task 9772991 on 2020-08-07 */

#define BQ25890_CON0               0x00
#define BQ25890_CON1               0x01
#define BQ25890_CON2               0x02
#define BQ25890_CON3               0x03
#define BQ25890_CON4               0x04
#define BQ25890_CON5               0x05
#define BQ25890_CON6               0x06
#define BQ25890_CON7               0x07
#define BQ25890_CON8               0x08
#define BQ25890_CON9               0x09
#define BQ25890_CONA               0x0A
#define BQ25890_CONB               0x0B
#define BQ25890_CONC               0x0C
#define BQ25890_COND               0x0D
#define BQ25890_CONE               0x0E
#define BQ25890_CONF               0x0F
#define BQ25890_CON10              0x10
#define BQ25890_CON11              0x11
#define BQ25890_CON12              0x12
#define BQ25890_CON13              0x13
#define BQ25890_CON14              0x14

#define BQ25890_REG_NUM            0x15

//CON0
#define CON0_EN_HIZ_MASK           0x1
#define CON0_EN_HIZ_SHIFT          7

#define CON0_EN_ILIM_MASK          0x1
#define CON0_EN_ILIM_SHIFT         6

#define CON0_IINLIM_MASK           0x3F
#define CON0_IINLIM_SHIFT          0

//CON1
#define CON1_DP_DAC_MASK           0x7
#define CON1_DP_DAC_SHIFT          5

#define CON1_DM_DAC_MASK           0x7
#define CON1_DM_DAC_SHIFT          2

#define CON1_VINDPM_OS_MASK        0x1
#define CON1_VINDPM_OS_SHIFT       0

#define CON1_VINDPM_OS_SY6970QCC_MASK        0x1F
#define CON1_VINDPM_OS_SY6970QCC_SHIFT       0

//CON2
#define CON2_CONV_START_MASK       0x1
#define CON2_CONV_START_SHIFT      7

#define CON2_CONV_RATE_MASK        0x1
#define CON2_CONV_RATE_SHIFT       6

#define CON2_BOOST_FREQ_MASK       0x1
#define CON2_BOOST_FREQ_SHIFT      5

#define CON2_ICO_EN_MASK           0x1
#define CON2_ICO_EN_RATE_SHIFT     4

#define CON2_HVDCP_EN_MASK         0x1
#define CON2_HVDCP_EN_SHIFT        3

#define CON2_MAX_EN_MASK           0x1
#define CON2_MAX_EN_SHIFT          2

#define CON2_FORCE_DPDM_MASK       0x1
#define CON2_FORCE_DPDM_SHIFT      1

#define CON2_AUTO_DPDM_EN_MASK     0x1
#define CON2_AUTO_DPDM_EN_SHIFT    0

//CON3
#define CON3_FORCE_DSEL_MASK       0x1
#define CON3_FORCE_DSEL_SHIFT      7

#define CON3_WD_MASK               0x1
#define CON3_WD_SHIFT              6

#define CON3_OTG_CONFIG_MASK       0x1
#define CON3_OTG_CONFIG_SHIFT      5

#define CON3_CHG_CONFIG_MASK       0x1
#define CON3_CHG_CONFIG_SHIFT      4

#define CON3_SYS_V_LIMIT_MASK      0x7
#define CON3_SYS_V_LIMIT_SHIFT     1

//CON4
#define CON4_EN_PUMPX_MASK         0x1
#define CON4_EN_PUMPX_SHIFT        7

#define CON4_ICHG_MASK             0x7F
#define CON4_ICHG_SHIFT            0

//CON5
#define CON5_IPRECHG_MASK          0xF
#define CON5_IPRECHG_SHIFT         4

#define CON5_ITERM_MASK            0xF
#define CON5_ITERM_SHIFT           0

//CON6
#define CON6_VREG_MASK             0x3F
#define CON6_VREG_SHIFT            2

#define CON6_BATLOWV_MASK          0x1
#define CON6_BATLOWV_SHIFT         1

#define CON6_VRECHG_MASK           0x1
#define CON6_VRECHG_SHIFT          0

//CON7
#define CON7_EN_TERM_CHG_MASK      0x1
#define CON7_EN_TERM_CHG_SHIFT     7

#define CON7_STAT_DIS_MASK         0x1
#define CON7_STAT_DIS_SHIFT        6

#define CON7_WTG_TIM_SET_MASK      0x3
#define CON7_WTG_TIM_SET_SHIFT     4

#define CON7_EN_TIMER_MASK         0x1
#define CON7_EN_TIMER_SHIFT        3

#define CON7_SET_CHG_TIM_MASK      0x3
#define CON7_SET_CHG_TIM_SHIFT     1

#define CON7_JEITA_ISET_MASK       0x1
#define CON7_JEITA_ISET_SHIFT      0

//CON8
#define CON8_BAT_COMP_MASK         0x7
#define CON8_BAT_COMP_SHIFT        5

#define CON8_VCLAMP_MASK           0x7
#define CON8_VCLAMP_SHIFT          2

#define CON8_TREG_MASK             0x3
#define CON8_TREG_SHIFT            0

//CON9
#define CON9_FORCE_ICO_MASK        0x1
#define CON9_FORCE_ICO_SHIFT       7

#define CON9_TMR2X_EN_MASK         0x01
#define CON9_TMR2X_EN_SHIFT        6

#define CON9_BATFET_DIS_MASK       0x01
#define CON9_BATFET_DIS_SHIFT      5

#define CON9_JEITA_VSET_MASK       0x01
#define CON9_JEITA_VSET_SHIFT      4

#define CON9_BATFET_DLY_MASK       0x01
#define CON9_BATFET_DLY_SHIFT      3

#define CON9_BATFET_RST_EN_MASK    0x01
#define CON9_BATFET_RST_EN_SHIFT   2

#define CON9_PUMPX_UP_MASK         0x1
#define CON9_PUMPX_UP_SHIFT        1

#define CON9_PUMPX_DN_MASK         0x1
#define CON9_PUMPX_DN_SHIFT        0

//CONA
#define CONA_BOOST_VLIM_MASK       0xF
#define CONA_BOOST_VLIM_SHIFT      4

#define CONA_PFM_OTG_DIS_MASK      0x01
#define CONA_PFM_OTG_DIS_SHIFT     3

#define CONA_BOOST_ILIM_MASK       0x07
#define CONA_BOOST_ILIM_SHIFT      0

//CONB
#define CONB_VBUS_STAT_MASK        0x7
#define CONB_VBUS_STAT_SHIFT       5

#define CONB_CHRG_STAT_MASK        0x3
#define CONB_CHRG_STAT_SHIFT       3

#define CONB_PG_STAT_MASK          0x1
#define CONB_PG_STAT_SHIFT         2

#define CONB_SDP_STAT_MASK         0x1
#define CONB_SDP_STAT_SHIFT        1

#define CONB_VSYS_STAT_MASK        0x1
#define CONB_VSYS_STAT_SHIFT       0

//CONC
#define CONC_WATG_STAT_MASK        0x1
#define CONC_WATG_STAT_SHIFT       7

#define CONC_BOOST_STAT_MASK       0x1
#define CONC_BOOST_STAT_SHIFT      6

#define CONC_CHRG_FAULT_MASK       0x3
#define CONC_CHRG_FAULT_SHIFT      4

#define CONC_BAT_STAT_MASK         0x1
#define CONC_BAT_STAT_SHIFT        3

#define CONC_NTC_FAULT_MASK        0x7
#define CONC_NTC_FAULT_SHIFT       0

//COND
#define COND_FORCE_VINDPM_MASK     0x01
#define COND_FORCE_VINDPM_SHIFT    7

#define COND_VINDPM_MASK           0x7F
#define COND_VINDPM_SHIFT          0

//CONE
#define CONE_THERM_STAT_MASK       0x01
#define CONE_THERM_STAT_SHIFT      7

#define CONE_VBAT_MASK             0x7F
#define CONE_VBAT_SHIFT            0

//CONF
#define CONF_VSYS_MASK             0x7F
#define CONF_VSYS_SHIFT            0

//CON10
#define CON10_TSPCT_MASK           0x7F
#define CON10_TSPCT_SHIFT          0

/* CON11 */
#define CON11_VBUS_GD_MASK         0x01
#define CON11_VBUS_GD_SHIFT        7

#define CON11_VBUS_MASK            0x7F
#define CON11_VBUS_SHIFT           0

//CON12
#define CON12_ICHG_STAT_MASK       0x7F
#define CON12_ICHG_STAT_SHIFT      0

//CON13
#define CON13_VDPM_STAT_MASK       0x1
#define CON13_VDPM_STAT_SHIFT      7

#define CON13_IDPM_STAT_MASK       0x1
#define CON13_IDPM_STAT_SHIFT      6

#define CON13_IINLIM_MASK          0x3F
#define CON13_IINLIM_SHIFT         0

//CON14
#define CON14_REG_RST_MASK         0x01
#define CON14_REG_RST_SHIFT        7

#define CON14_ICO_STAT_MASK        0x01
#define CON14_ICO_STAT_SHIFT       6

#define CON14_PN_MASK              0x7
#define CON14_PN_SHIFT             3

#define CON14_TS_PROFILE_MASK      0x1
#define CON14_TS_PROFILE_SHIFT     2

#define CON14_DEV_REV_MASK         0x3
#define CON14_DEV_REV_SHIFT        0


/*BQ25890 REG06 VREG[5:0]*/
static const unsigned int VBAT_CV_VTH[] = {
    3840000, 3856000, 3872000, 3888000,
    3904000, 3920000, 3936000, 3952000,
    3968000, 3984000, 4000000, 4016000,
    4032000, 4048000, 4064000, 4080000,
    4096000, 4112000, 4128000, 4144000,
    4160000, 4176000, 4192000, 4208000,
    4224000, 4240000, 4256000, 4272000,
    4288000, 4304000, 4320000, 4336000,
    4352000, 4368000, 4384000, 4400000,
    4416000, 4432000, 4448000, 4464000,
    4480000, 4496000, 4512000, 4528000,
    4544000, 4560000, 4576000, 4592000,
    4608000
};

/*BQ25890 REG04 ICHG[6:0]*/
static const unsigned int CS_VTH[] = {
    0, 6400, 12800, 19200,
    25600, 32000, 38400, 44800,
    50000, 57600, 64000, 70400,//modified by hailong.chen for usb charging current
    76800, 83200, 89600, 96000,
    102400, 108800, 115200, 121600,
    128000, 134400, 140800, 147200,
    153600, 160000, 166400, 172800,
    179200, 185600, 192000, 198400,
    204800, 211200, 217600, 224000,
    230400, 236800, 243200, 249600,
    256000, 262400, 268800, 275200,
    281600, 288000, 294400, 300000,//modified by hailong.chen for PE2.0 charging current
    307200, 313600, 320000, 326400,
    332800, 339200, 345600, 352000,
    358400, 364800, 371200, 377600,
    384000, 390400, 396800, 403200,
    409600, 416000, 422400, 428800,
    435200, 441600, 448000, 454400,
    460800, 467200, 473600, 480000,
    486400, 492800, 499200, 505600
};

/*BQ25890 REG00 IINLIM[5:0]*/
static const unsigned int INPUT_CS_VTH[] = {
    10000, 15000, 20000, 25000,
    30000, 35000, 40000, 45000,
    50000, 55000, 60000, 65000,
    70000, 75000, 80000, 85000,
    90000, 95000, 100000, 105000,
    110000, 115000, 120000, 125000,
    130000, 135000, 140000, 145000,
    150000, 155000, 160000, 165000,
    170000, 175000, 180000, 185000,
    190000, 195000, 200000, 200500,
    210000, 215000, 220000, 225000,
    230000, 235000, 240000, 245000,
    250000, 255000, 260000, 265000,
    270000, 275000, 280000, 285000,
    290000, 295000, 300000, 305000,
    310000, 315000, 320000, 325000
};

static const unsigned int VINDPM_REG[] = {
    2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900, 4000,
    4100, 4200, 4300, 4400, 4500, 4600, 4700, 4800, 4900, 5000, 5100, 5200, 5300, 5400, 5500,
    5600, 5700, 5800, 5900, 6000, 6100, 6200, 6300, 6400, 6500, 6600, 6700, 6800, 6900, 7000,
    7100, 7200, 7300, 7400, 7500, 7600, 7700, 7800, 7900, 8000, 8100, 8200, 8300, 8400, 8500,
    8600, 8700, 8800, 8900, 9000, 9100, 9200, 9300, 9400, 9500, 9600, 9700, 9800, 9900, 10000,
    10100, 10200, 10300, 10400, 10500, 10600, 10700, 10800, 10900, 11000, 11100, 11200, 11300,
    11400, 11500, 11600, 11700, 11800, 11900, 12000, 12100, 12200, 12300, 12400, 12500, 12600,
    12700, 12800, 12900, 13000, 13100, 13200, 13300, 13400, 13500, 13600, 13700, 13800, 13900,
    14000, 14100, 14200, 14300, 14400, 14500, 14600, 14700, 14800, 14900, 15000, 15100, 15200,
    15300
};

/* BQ25890 REG0A BOOST_LIM[2:0], mA */
static const unsigned int BOOST_CURRENT_LIMIT[] = {
    500, 750, 1200, 1400, 1650, 1875, 2150,
};


/* BQ25890 REG08 VCLAMP, mV */
static const unsigned int IRCMP_VOLT_CLAMP[] = {
    0, 32, 64, 128, 160, 192, 224,
};

/* BQ25890 REG08 BAT_COMP, mohm */
static const unsigned int IRCMP_RESISTOR[] = {
    0, 20, 40, 60, 80, 100, 120, 140,
};

//#define BQ25890_CHARGER_TYPE_DETECT
//#define BQ25890_LOG_ON

#define bq_err(fmt, args...)		\
do {								\
		pr_notice(fmt, ##args);		\
} while (0)

#define bq_debug(fmt, args...)		\
do {								\
		pr_notice(fmt, ##args);		\
} while (0)

#ifdef CONFIG_OF
#else

#define bq25890_SLAVE_ADDR_WRITE   0xD4
#define bq25890_SLAVE_ADDR_Read    0xD5

#ifdef I2C_SWITHING_CHARGER_CHANNEL
#define bq25890_BUSNUM I2C_SWITHING_CHARGER_CHANNEL
#else
#define bq25890_BUSNUM 0
#endif

#endif

struct bq25890_info {
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct charger_properties chg_props;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	//enum charger_type chg_type;
	int irq;

/* Begin mod by jin.wang for proper type on 2022-4-6 */
	int intr_gpio;
	int ceb_gpio;
/* End mod by jin.wang */

#if 0
	struct gtimer otg_kthread_gtimer;
	struct workqueue_struct *otg_boost_workq;
	struct work_struct kick_work;
	unsigned int polling_interval;
	bool polling_enabled;
#endif
/* Begin Modified by qiuguangliang for 11297832 on 2021-06-30*/
	struct power_supply *chg_psy;
	struct power_supply_desc psy_desc;
/* End Modified by qiuguangliang for 11297832 on 2021-06-30 */

/* Begin add by jin.wang for jira 5550 on 2022-1-7 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	struct delayed_work chg_stat_work;
#endif
/* End add by jin.wang */
};

static unsigned int g_input_current;
static DEFINE_MUTEX (g_input_current_mutex);
static unsigned int charging_error;
static struct i2c_client *new_client;
static const struct i2c_device_id bq25890_i2c_id[] = {
    {"bq25890", 0},
    {}
};

//Begin Modified by qiuguangliang && jin.wang for 11297832 on 2021-12-2
#if IS_ENABLED(CONFIG_TCT_CHG_PASSAT)
static unsigned int max_cv = 4450000;
static unsigned int now_cv = 4450000;
#else
static unsigned int now_cv = 4400000;
static unsigned int max_cv = 4400000;
#endif
//End Modified by qiuguangliang && jin.wang

/* [BSP]Begin added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */
static bool force_disable_charge = false;
/* [BSP]End added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */

static int bq25890_driver_probe(struct i2c_client *client,
                                const struct i2c_device_id *id);

static unsigned int charging_value_to_parameter(const unsigned int *parameter,
                                         const unsigned int array_size,
                                         const unsigned int val)
{
    if (val < array_size)
        return parameter[val];

    pr_info("Can't find the parameter\n");
    return parameter[0];

}

static unsigned int charging_parameter_to_value(const unsigned int *parameter,
                                         const unsigned int array_size,
                                         const unsigned int val)
{
    unsigned int i;
    #ifdef BQ25890_LOG_ON
    pr_debug_ratelimited("array_size = %d\n", array_size);
    #endif
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
                #ifdef BQ25890_LOG_ON
                pr_debug_ratelimited("zzf_%d<=%d, i=%d\n", pList[i], level, i);
                #endif
                return pList[i];
            }
        }

        pr_info("Can't find closest level\n");
        return pList[0];
        /* return CHARGE_CURRENT_0_00_MA; */
    }
    else {
        for (i = 0; i < number; i++) { /* max value in the first element */
            if (pList[i] <= level)
                return pList[i];
        }

        pr_info("Can't find closest level\n");
        return pList[number - 1];
        /* return CHARGE_CURRENT_0_00_MA; */
    }
}

static unsigned char bq25890_reg[BQ25890_REG_NUM] = {
    0
};

static DEFINE_MUTEX (bq25890_i2c_access);
static DEFINE_MUTEX (bq25890_access_lock);

static bool g_bq25890_hw_exist = false;
static bool g_sy6970qcc_hw_exist = false;
#define BQ25890H_PN            0x03
#define SY6970QCC_PN           0x01

#ifdef CONFIG_MTK_I2C_EXTENSION
static unsigned int bq25890_read_byte(unsigned char cmd, unsigned char *returnData)
{
    char cmd_buf[1] = { 0x00 };
    char readData = 0;
    int ret = 0;

    mutex_lock(&bq25890_i2c_access);

    /* new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG; */
    new_client->ext_flag =
        ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
    ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
    if (ret < 0) {
        /* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
        new_client->ext_flag = 0;
        mutex_unlock(&bq25890_i2c_access);

        return 0;
    }

    readData = cmd_buf[0];
    *returnData = readData;

    /* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
    new_client->ext_flag = 0;
    mutex_unlock(&bq25890_i2c_access);

    return 1;
}

static unsigned int bq25890_write_byte(unsigned char cmd, unsigned char writeData)
{
    char write_data[2] = { 0 };
    int ret = 0;

    mutex_lock(&bq25890_i2c_access);

    write_data[0] = cmd;
    write_data[1] = writeData;

    new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) {
        new_client->ext_flag = 0;
        mutex_unlock(&bq25890_i2c_access);
        return 0;
    }

    new_client->ext_flag = 0;
    mutex_unlock(&bq25890_i2c_access);
    return 1;
}
#else
static unsigned int bq25890_read_byte(unsigned char cmd, unsigned char *returnData)
{
    unsigned char xfers = 2;
    int ret, retries = 1;

    mutex_lock(&bq25890_i2c_access);

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

    mutex_unlock(&bq25890_i2c_access);

    return ret == xfers ? 1 : -1;
}

static unsigned int bq25890_write_byte(unsigned char cmd, unsigned char writeData)
{
    unsigned char xfers = 1;
    int ret, retries = 1;
    unsigned char buf[8];

    mutex_lock(&bq25890_i2c_access);

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

    mutex_unlock(&bq25890_i2c_access);

    return ret == xfers ? 1 : -1;
}
#endif
static unsigned int bq25890_read_interface(unsigned char RegNum,
                                    unsigned char *val,
                                    unsigned char MASK,
                                    unsigned char SHIFT)
{
    unsigned char bq25890_reg = 0;
    unsigned int ret = 0;

    ret = bq25890_read_byte(RegNum, &bq25890_reg);

    bq_debug("[bq25890_read_interface] Reg[%x]=0x%x\n", RegNum, bq25890_reg);

    bq25890_reg &= (MASK << SHIFT);
    *val = (bq25890_reg >> SHIFT);

    bq_debug("[bq25890_read_interface] val=0x%x\n", *val);

    return ret;
}

static unsigned int bq25890_config_interface(unsigned char RegNum,
                                      unsigned char val,
                                      unsigned char MASK,
                                      unsigned char SHIFT)
{
    unsigned char bq25890_reg = 0;
    unsigned char bq25890_reg_ori = 0;
    unsigned int ret = 0;

    mutex_lock(&bq25890_access_lock);
    ret = bq25890_read_byte(RegNum, &bq25890_reg);

    bq25890_reg_ori = bq25890_reg;
    bq25890_reg &= ~(MASK << SHIFT);
    bq25890_reg |= (val << SHIFT);

    ret = bq25890_write_byte(RegNum, bq25890_reg);
    mutex_unlock(&bq25890_access_lock);
    bq_debug("[bq25890_config_interface] write Reg[%x]=0x%x from 0x%x\n",
              RegNum,
              bq25890_reg,
              bq25890_reg_ori);

    /* Check */
    /* bq25890_read_byte(RegNum, &bq25890_reg); */
    /* printk("[bq25890_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq25890_reg); */

    return ret;
}

/* write one register directly */
static unsigned int __maybe_unused bq25890_reg_config_interface(unsigned char RegNum,
                                          unsigned char val)
{
    unsigned int ret = 0;

    ret = bq25890_write_byte(RegNum, val);

    return ret;
}

/* CON0---------------------------------------------------- */

static void bq25890_set_en_hiz(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON0),
                                   (unsigned char) (val),
                                   (unsigned char) (CON0_EN_HIZ_MASK),
                                   (unsigned char) (CON0_EN_HIZ_SHIFT));
}

static void bq25890_set_en_ilim(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON0),
                                   (unsigned char) (val),
                                   (unsigned char) (CON0_EN_ILIM_MASK),
                                   (unsigned char) (CON0_EN_ILIM_SHIFT));
}

static void bq25890_set_iinlim(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON0),
                                   (val),
                                   (unsigned char) (CON0_IINLIM_MASK),
                                   (unsigned char) (CON0_IINLIM_SHIFT));
}

static unsigned int __maybe_unused bq25890_get_iinlim(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CON0),
                                 (&val),
                                 (unsigned char) (CON0_IINLIM_MASK),
                                 (unsigned char) (CON0_IINLIM_SHIFT));
    return val;
}

/* CON1---------------------------------------------------- */

static void __maybe_unused bq25890_set_vindpm_os(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON1),
                                   (unsigned char) (val),
                                   (unsigned char) (CON1_VINDPM_OS_MASK),
                                   (unsigned char) (CON1_VINDPM_OS_SHIFT));
}

static void __maybe_unused sy6970qcc_set_vindpm_os(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON1),
                                   (unsigned char) (val),
                                   (unsigned char) (CON1_VINDPM_OS_SY6970QCC_MASK),
                                   (unsigned char) (CON1_VINDPM_OS_SY6970QCC_SHIFT));
}

/* CON2---------------------------------------------------- */

static void bq25890_ADC_start(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON2),
                                   (unsigned char) (val),
                                   (unsigned char) (CON2_CONV_START_MASK),
                                   (unsigned char) (CON2_CONV_START_SHIFT));
}

static void __maybe_unused bq25890_set_ADC_rate(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON2),
                                   (unsigned char) (val),
                                   (unsigned char) (CON2_CONV_RATE_MASK),
                                   (unsigned char) (CON2_CONV_RATE_SHIFT));
}

static void bq25890_set_ico_en_start(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON2),
                                   (unsigned char) (val),
                                   (unsigned char) (CON2_ICO_EN_MASK),
                                   (unsigned char) (CON2_ICO_EN_RATE_SHIFT));
}

static void __maybe_unused bq25890_set_force_dpdm(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON2),
                                   (unsigned char) (val),
                                   (unsigned char) (CON2_FORCE_DPDM_MASK),
                                   (unsigned char) (CON2_FORCE_DPDM_SHIFT));
}

#ifdef BQ25890_CHARGER_TYPE_DETECT
static int bq25890_get_force_dpdm(bool *force)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CON2),
                     &val,
                     (unsigned char) (CON2_FORCE_DPDM_MASK),
                     (unsigned char) (CON2_FORCE_DPDM_SHIFT)
    );

    *force = (val == 0 ? false : true);
    return ret;
}

static void bq25890_set_auto_dpdm(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON2),
                       (unsigned char) (val),
                       (unsigned char) (CON2_AUTO_DPDM_EN_MASK),
                       (unsigned char) (CON2_AUTO_DPDM_EN_SHIFT)
        );
}
#endif

/* CON3---------------------------------------------------- */

static void bq25890_wd_reset(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON3),
                                   (val),
                                   (unsigned char) (CON3_WD_MASK),
                                   (unsigned char) (CON3_WD_SHIFT));
}

static void bq25890_otg_en(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON3),
                                   (val),
                                   (unsigned char) (CON3_OTG_CONFIG_MASK),
                                   (unsigned char) (CON3_OTG_CONFIG_SHIFT));
}

static int bq25890_is_otg_en(bool *en)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CON3),
                                 (&val),
                                 (unsigned char) (CON3_OTG_CONFIG_MASK),
                                 (unsigned char) (CON3_OTG_CONFIG_SHIFT));

    *en = (val == 0 ? false : true);

    return ret;
}

static void bq25890_chg_en(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON3),
                                   (val),
                                   (unsigned char) (CON3_CHG_CONFIG_MASK),
                                   (unsigned char) (CON3_CHG_CONFIG_SHIFT));
}

static unsigned int __maybe_unused bq25890_get_chg_en(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CON3),
                                 (&val),
                                 (unsigned char) (CON3_CHG_CONFIG_MASK),
                                 (unsigned char) (CON3_CHG_CONFIG_SHIFT));
    return val;
}

static void __maybe_unused bq25890_set_sys_min(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON3),
                                   (val),
                                   (unsigned char) (CON3_SYS_V_LIMIT_MASK),
                                   (unsigned char) (CON3_SYS_V_LIMIT_SHIFT));
}

/* CON4---------------------------------------------------- */

static void bq25890_en_pumpx(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON4),
                                   (unsigned char) (val),
                                   (unsigned char) (CON4_EN_PUMPX_MASK),
                                   (unsigned char) (CON4_EN_PUMPX_SHIFT));
}

static void bq25890_set_ichg(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON4),
                                   (unsigned char) (val),
                                   (unsigned char) (CON4_ICHG_MASK),
                                   (unsigned char) (CON4_ICHG_SHIFT));
}

static unsigned int bq25890_get_reg_ichg(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CON4),
                                 (&val),
                                 (unsigned char) (CON4_ICHG_MASK),
                                 (unsigned char) (CON4_ICHG_SHIFT));
    return val;
}

/* CON5---------------------------------------------------- */

static void bq25890_set_iprechg(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON5),
                                   (val),
                                   (unsigned char) (CON5_IPRECHG_MASK),
                                   (unsigned char) (CON5_IPRECHG_SHIFT));
}

static void bq25890_set_iterml(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON5),
                                   (val),
                                   (unsigned char) (CON5_ITERM_MASK),
                                   (unsigned char) (CON5_ITERM_SHIFT));
}

/* CON6---------------------------------------------------- */

static void bq25890_set_vreg(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON6),
                                   (unsigned char) (val),
                                   (unsigned char) (CON6_VREG_MASK),
                                   (unsigned char) (CON6_VREG_SHIFT));
}

static unsigned int bq25890_get_vreg(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CON6),
                                 (&val),
                                 (unsigned char) (CON6_VREG_MASK),
                                 (unsigned char) (CON6_VREG_SHIFT));
    return val;
}

static void bq25890_set_batlowv(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON6),
                                   (unsigned char) (val),
                                   (unsigned char) (CON6_BATLOWV_MASK),
                                   (unsigned char) (CON6_BATLOWV_SHIFT));
}

static void __maybe_unused bq25890_set_vrechg(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON6),
                                   (unsigned char) (val),
                                   (unsigned char) (CON6_VRECHG_MASK),
                                   (unsigned char) (CON6_VRECHG_SHIFT));
}

/* CON7---------------------------------------------------- */

static void __maybe_unused bq25890_en_term_chg(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON7),
                                   (unsigned char) (val),
                                   (unsigned char) (CON7_EN_TERM_CHG_MASK),
                                   (unsigned char) (CON7_EN_TERM_CHG_SHIFT));
}

static void __maybe_unused bq25890_en_state_dis(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON7),
                                   (unsigned char) (val),
                                   (unsigned char) (CON7_STAT_DIS_MASK),
                                   (unsigned char) (CON7_STAT_DIS_SHIFT));
}

static void bq25890_set_wd_timer(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON7),
                                   (unsigned char) (val),
                                   (unsigned char) (CON7_WTG_TIM_SET_MASK),
                                   (unsigned char) (CON7_WTG_TIM_SET_SHIFT));
}

static void bq25890_en_chg_timer(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON7),
                                   (unsigned char) (val),
                                   (unsigned char) (CON7_EN_TIMER_MASK),
                                   (unsigned char) (CON7_EN_TIMER_SHIFT));
}

static unsigned int bq25890_get_chg_timer_enable(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CON7),
                                 &val,
                                 (unsigned char) (CON7_EN_TIMER_MASK),
                                 (unsigned char) (CON7_EN_TIMER_SHIFT));

    return val;
}

static void __maybe_unused bq25890_set_chg_timer(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON7),
                                   (unsigned char) (val),
                                   (unsigned char) (CON7_SET_CHG_TIM_MASK),
                                   (unsigned char) (CON7_SET_CHG_TIM_SHIFT));
}

/* CON8--------------------------------------------------- */
static void __maybe_unused bq25890_set_thermal_regulation(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON8),
                                   (unsigned char) (val),
                                   (unsigned char) (CON8_TREG_MASK),
                                   (unsigned char) (CON8_TREG_SHIFT));
}

static void bq25890_set_VBAT_clamp(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON8),
                                   (unsigned char) (val),
                                   (unsigned char) (CON8_VCLAMP_MASK),
                                   (unsigned char) (CON8_VCLAMP_SHIFT));
}

static void bq25890_set_VBAT_IR_compensation(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON8),
                                   (unsigned char) (val),
                                   (unsigned char) (CON8_BAT_COMP_MASK),
                                   (unsigned char) (CON8_BAT_COMP_SHIFT));
}

/* CON9---------------------------------------------------- */
static void bq25890_pumpx_up(unsigned int val)
{
    unsigned int ret = 0;

    /* Input current limit = 500 mA, changes after PE+ detection */
    bq25890_set_iinlim(0x08);

    /* CC mode current = 2048 mA */
    bq25890_set_ichg(0x20);

    bq25890_chg_en(1);

    bq25890_en_pumpx(1);
    if (val == 1) {
        ret = bq25890_config_interface((unsigned char) (BQ25890_CON9),
                                       (unsigned char) (1),
                                       (unsigned char) (CON9_PUMPX_UP_MASK),
                                       (unsigned char) (CON9_PUMPX_UP_SHIFT));
    }
    else {
        ret = bq25890_config_interface((unsigned char) (BQ25890_CON9),
                                       (unsigned char) (1),
                                       (unsigned char) (CON9_PUMPX_DN_MASK),
                                       (unsigned char) (CON9_PUMPX_DN_SHIFT));
    }

    /* Input current limit = 500 mA, changes after PE+ detection */
    bq25890_set_iinlim(0x08);

    /* CC mode current = 2048 mA */
    bq25890_set_ichg(0x20);

    msleep(3000);
}

static void __maybe_unused bq25890_set_force_ico(void)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CON9),
                                   (unsigned char) (1),
                                   (unsigned char) (CON9_FORCE_ICO_MASK),
                                   (unsigned char) (CON9_FORCE_ICO_MASK));
}

static void bq25890_batfet_rst_en(unsigned int val)
{
    bq25890_config_interface((unsigned char) (BQ25890_CON9),
                             (unsigned char) (val),
                             (unsigned char) (CON9_BATFET_RST_EN_MASK),
                             (unsigned char) (CON9_BATFET_RST_EN_SHIFT));
}

/* CONA---------------------------------------------------- */
static void bq25890_set_boost_ilim(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CONA),
                                   (unsigned char) (val),
                                   (unsigned char) (CONA_BOOST_ILIM_MASK),
                                   (unsigned char) (CONA_BOOST_ILIM_SHIFT));
}

static void __maybe_unused bq25890_set_boost_vlim(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_CONA),
                                   (unsigned char) (val),
                                   (unsigned char) (CONA_BOOST_VLIM_MASK),
                                   (unsigned char) (CONA_BOOST_VLIM_SHIFT));
}

/* CONB---------------------------------------------------- */

static unsigned int __maybe_unused bq25890_get_vbus_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CONB),
                                 (&val),
                                 (unsigned char) (CONB_VBUS_STAT_MASK),
                                 (unsigned char) (CONB_VBUS_STAT_SHIFT));
    return val;
}

static unsigned int bq25890_get_chrg_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CONB),
                                 (&val),
                                 (unsigned char) (CONB_CHRG_STAT_MASK),
                                 (unsigned char) (CONB_CHRG_STAT_SHIFT));
    return val;
}

static unsigned int __maybe_unused bq25890_get_pg_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CONB),
                                 (&val),
                                 (unsigned char) (CONB_PG_STAT_MASK),
                                 (unsigned char) (CONB_PG_STAT_SHIFT));
    return val;
}

static unsigned int __maybe_unused bq25890_get_sdp_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CONB),
                                 (&val),
                                 (unsigned char) (CONB_SDP_STAT_MASK),
                                 (unsigned char) (CONB_SDP_STAT_SHIFT));
    return val;
}

static unsigned int __maybe_unused bq25890_get_vsys_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CONB),
                                 (&val),
                                 (unsigned char) (CONB_VSYS_STAT_MASK),
                                 (unsigned char) (CONB_VSYS_STAT_SHIFT));
    return val;
}

/* CON0C---------------------------------------------------- */
static unsigned int bq25890_get_wdt_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CONC),
                                 (&val),
                                 (unsigned char) (CONC_WATG_STAT_MASK),
                                 (unsigned char) (CONC_WATG_STAT_SHIFT));
    return val;
}

static unsigned int __maybe_unused bq25890_get_boost_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CONC),
                                 (&val),
                                 (unsigned char) (CONC_BOOST_STAT_MASK),
                                 (unsigned char) (CONC_BOOST_STAT_SHIFT));
    return val;
}

static unsigned int __maybe_unused bq25890_get_chrg_fault_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CONC),
                                 (&val),
                                 (unsigned char) (CONC_CHRG_FAULT_MASK),
                                 (unsigned char) (CONC_CHRG_FAULT_SHIFT));
    return val;
}

static unsigned int __maybe_unused bq25890_get_bat_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CONC),
                                 (&val),
                                 (unsigned char) (CONC_BAT_STAT_MASK),
                                 (unsigned char) (CONC_BAT_STAT_SHIFT));
    return val;
}

/* COND */
static void bq25890_set_force_vindpm(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25890_config_interface((unsigned char) (BQ25890_COND),
                                   (unsigned char) (val),
                                   (unsigned char) (COND_FORCE_VINDPM_MASK),
                                   (unsigned char) (COND_FORCE_VINDPM_SHIFT));
}

static void bq25890_set_vindpm(unsigned int val)
{
    unsigned int ret = 0;

/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	val = 0x14;  // set vindpm to 4.5V always
#endif
/* End add by jin.wang */

    ret = bq25890_config_interface((unsigned char) (BQ25890_COND),
                                   (unsigned char) (val),
                                   (unsigned char) (COND_VINDPM_MASK),
                                   (unsigned char) (COND_VINDPM_SHIFT));
}

static unsigned int bq25890_get_vindpm(void)
{
    int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_COND),
                                 (&val),
                                 (unsigned char) (COND_VINDPM_MASK),
                                 (unsigned char) (COND_VINDPM_SHIFT));
    return val;
}

/* CONDE */
static unsigned int __maybe_unused bq25890_get_vbat(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CONE),
                                 (&val),
                                 (unsigned char) (CONE_VBAT_MASK),
                                 (unsigned char) (CONE_VBAT_SHIFT));
    return val;
}

/* CON11 */
static unsigned int __maybe_unused bq25890_get_vbus(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CON11),
                                 (&val),
                                 (unsigned char) (CON11_VBUS_MASK),
                                 (unsigned char) (CON11_VBUS_SHIFT));
    return val;
}

/* CON12 */
static unsigned int __maybe_unused bq25890_get_ichg(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CON12),
                                 (&val),
                                 (unsigned char) (CON12_ICHG_STAT_MASK),
                                 (unsigned char) (CON12_ICHG_STAT_SHIFT));
    return val;
}

/* CON13 /// */

static unsigned int __maybe_unused bq25890_get_idpm_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CON13),
                                 (&val),
                                 (unsigned char) (CON13_IDPM_STAT_MASK),
                                 (unsigned char) (CON13_IDPM_STAT_SHIFT));
    return val;
}

static unsigned int bq25890_get_vdpm_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface((unsigned char) (BQ25890_CON13),
                                 (&val),
                                 (unsigned char) (CON13_VDPM_STAT_MASK),
                                 (unsigned char) (CON13_VDPM_STAT_SHIFT));
    return val;
}

/* Begin add by jin.wang jira 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static unsigned char bq25890_get_ico_ichg(void)
{
    unsigned char val = 0;
    bq25890_read_interface((unsigned char) (BQ25890_CON13),
                           (&val),
                           (unsigned char) (CON13_IINLIM_MASK),
                           (unsigned char) (CON13_IINLIM_SHIFT));
    return val;
}
#endif
/* End add by jin.wang */

#ifdef BQ25890_CHARGER_TYPE_DETECT
static int bq25890_get_charger_type(struct bq25890_info *info)
{
    unsigned int vbus_stat = 0;
    unsigned int pg_stat = 0;

    enum charger_type CHR_Type_num = CHARGER_UNKNOWN;


    pg_stat = bq25890_get_pg_state();
    vbus_stat = bq25890_get_vbus_state();
    pr_notice("vbus_stat: 0x%x, pg_stat:0x%x\n", vbus_stat, pg_stat);

    switch (vbus_stat) {
    case 0: /* No input */
        CHR_Type_num = CHARGER_UNKNOWN;
        break;
    case 1: /* SDP */
        CHR_Type_num = STANDARD_HOST;
        break;
    case 2: /* CDP */
        CHR_Type_num = CHARGING_HOST;
        break;
    case 3: /* DCP */
        CHR_Type_num = STANDARD_CHARGER;
        break;
    case 5: /* Unknown adapter */
        CHR_Type_num = NONSTANDARD_CHARGER;
        break;
    case 6: /* Non-standard adapter */
        CHR_Type_num = NONSTANDARD_CHARGER;
        break;
    default:
        CHR_Type_num = NONSTANDARD_CHARGER;
        break;
    }

    return CHR_Type_num;
}

static int bq25890_set_charger_type(struct bq25890_info *info)
{
    int ret = 0;
    union power_supply_propval propval;

#if defined(CONFIG_PROJECT_PHY) || defined(CONFIG_PHY_MTK_SSUSB)
    if (info->chg_type == STANDARD_HOST ||
        info->chg_type == CHARGING_HOST)
        Charger_Detect_Release();
#endif

    if (info->chg_type != CHARGER_UNKNOWN)
        propval.intval = 1;
    else
        propval.intval = 0;
    ret = power_supply_set_property(info->psy,
        POWER_SUPPLY_PROP_ONLINE, &propval);
    if (ret < 0)
        pr_notice("%s: inform power supply online failed, ret = %d\n",
            __func__, ret);

    propval.intval = info->chg_type;
    ret = power_supply_set_property(info->psy,
        POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
    if (ret < 0)
        pr_notice("%s: inform power supply type failed, ret = %d\n",
            __func__, ret);

    return ret;
}
#endif

static void bq25890_hw_component_detect(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25890_read_interface(BQ25890_CON14, &val, CON14_PN_MASK, CON14_PN_SHIFT);
    switch (val) {
        case BQ25890H_PN:
            g_bq25890_hw_exist = true;
            pr_err("[%s] charger IC is BQ25890H\n", __func__);
            break;
        case SY6970QCC_PN:
            g_sy6970qcc_hw_exist = true;
            pr_err("[%s] charger IC is SY6970QCC\n", __func__);
            break;
        default:
            pr_err("[%s] charger IC is Unknown\n", __func__);
            break;
    }
}

static unsigned int charging_get_error_state(void)
{
    return charging_error;
}

static int bq25890_enable_charging(struct charger_device *chg_dev, bool en)
{
    int status = 0;

    /* [BSP]Begin added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */
    if (force_disable_charge)
        return status;
    /* [BSP]End added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */

    if (en) {
        /* bq25890_config_interface(BQ25890_CON3, 0x1, 0x1, 4); //enable charging */
        bq25890_set_en_hiz(0x0);
        bq25890_chg_en(en);
    }
    else {
        /* bq25890_config_interface(BQ25890_CON3, 0x0, 0x1, 4); //enable charging */
        bq25890_chg_en(en);
        if (charging_get_error_state())
            pr_info("[charging_enable] under test mode: disable charging\n");

        /*bq25890_set_en_hiz(0x1);*/
    }

    return status;
}

static int bq25890_get_current(struct charger_device *chg_dev, u32 *ichg)
{
    int status = 0;
    unsigned int array_size;
    /*unsigned char reg_value; */
    unsigned int val;

    /*Get current level */
    array_size = ARRAY_SIZE(CS_VTH);
    val = bq25890_get_reg_ichg();
    *ichg = charging_value_to_parameter(CS_VTH, array_size, val) * 10;

    return status;
}

static int bq25890_set_current(struct charger_device *chg_dev,
                               u32 current_value)
{
    int status = 0;
    unsigned int set_chr_current;
    unsigned int array_size;
    unsigned int register_value;

    current_value /= 10;
    array_size = ARRAY_SIZE(CS_VTH);
    set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
    register_value = charging_parameter_to_value(CS_VTH,
                                                 array_size,
                                                 set_chr_current);
    /* bq25890_config_interface(BQ25890_CON4, register_value, 0x7F, 0); */
    bq25890_set_ichg(register_value);

    return status;
}

static int bq25890_get_input_current(struct charger_device *chg_dev, u32 *aicr)
{
/* Begin mod by jin.wang jira 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
    unsigned char val;
    val = bq25890_get_ico_ichg();

	if (val < ARRAY_SIZE(INPUT_CS_VTH))
		*aicr = INPUT_CS_VTH[val] * 10;
	else
		*aicr = 0;

    return 0;
#else
	int ret = 0;
    *aicr = g_input_current;
    return ret;
#endif
/* End mod by jin.wang */
}

static int bq25890_set_input_current(struct charger_device *chg_dev,
                                     u32 current_value)
{
    int status = 0;
    unsigned int set_chr_current;
    unsigned int array_size;
    unsigned int register_value;

    mutex_lock(&g_input_current_mutex);
    current_value /= 10;
    array_size = ARRAY_SIZE(INPUT_CS_VTH);
    set_chr_current = bmt_find_closest_level(INPUT_CS_VTH,
                                             array_size,
                                             current_value);
    g_input_current = set_chr_current;
    register_value = charging_parameter_to_value(INPUT_CS_VTH,
                                                 array_size,
                                                 set_chr_current);
    bq25890_set_iinlim(register_value);
    mutex_unlock(&g_input_current_mutex);

/* Begin mod by jin.wang for task 5195 at 2021-12-27 */
    /*For USB_IF compliance test only when USB is in suspend(Ibus < 2.5mA) or unconfigured(Ibus < 70mA) states*/
#ifdef CONFIG_USBIF_COMPLIANCE
	if (current_value < 10000)
	    register_value = 0xFF;
	else
	    register_value = 0x13;

	bq25890_config_interface((unsigned char) (BQ25890_COND),
	                       (unsigned char) (register_value),
	                       (unsigned char) (COND_VINDPM_MASK),
	                       (unsigned char) (COND_VINDPM_SHIFT));
#endif
/* End mod by jin.wang */

    return status;
}

/* [BSP]Begin added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */
static int bq2890h_get_battery_voltage(void)
{
    union power_supply_propval prop;
    static struct power_supply *bat_psy = NULL;
    int ret;

    if (IS_ERR_OR_NULL(bat_psy))
        bat_psy = power_supply_get_by_name("battery");

    if (IS_ERR_OR_NULL(bat_psy)) {
        pr_err("%s Couldn't get bat_psy\n", __func__);
        return -EINVAL;
    } else {
        ret = power_supply_get_property(bat_psy,
        POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
        ret = prop.intval;
    }
    return ret;
}
/* [BSP]Begin added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */

static int bq25890_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
{
    int status = 0;
    unsigned short int array_size;
    unsigned int set_cv_voltage;
    unsigned short int register_value;
    unsigned int vbat = 0; /* [BSP]Added by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */
    /*static kal_int16 pre_register_value; */

//Begin Modified by qiuguangliang for 11297832 on 2021-06-30
	now_cv = cv;
	if (max_cv < now_cv) {
		max_cv = now_cv;
	}
	pr_info("charging_set_cv_voltage max_cv=%d now_cv:%d\n", max_cv, now_cv);
//End Modified by qiuguangliang for 11297832 on 2021-06-30

    array_size = ARRAY_SIZE(VBAT_CV_VTH);
    /*pre_register_value = -1; */
    set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv);
    register_value = charging_parameter_to_value(VBAT_CV_VTH,
                                                 ARRAY_SIZE(VBAT_CV_VTH),
                                                 set_cv_voltage);
    #ifdef BQ25890_LOG_ON
    pr_info("charging_set_cv_voltage register_value=0x%x %d %d\n",
            register_value, cv, set_cv_voltage);
    #endif
    /* [BSP]Begin modified by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */
    vbat = bq2890h_get_battery_voltage();
    if (g_sy6970qcc_hw_exist && vbat > set_cv_voltage * 102 / 100) {
        pr_info("%s:sy6970qcc can't set the cv(%d),vbat(%d)is too hight,it will be triggele the threshold(%d) of the ovp of battery,forced stop  charging.\n",
            __func__, set_cv_voltage, vbat, set_cv_voltage * 102 / 100);
        bq25890_enable_charging(chg_dev, false);
        usleep_range(1000, 1200);
        force_disable_charge = true;
    } else {
        bq25890_set_vreg(register_value);
        usleep_range(1000, 1200);
        force_disable_charge = false;
    }
    /* [BSP]End modified by bitao.xiong for SOCAOSP13-9015 on 2022/11/02 */

    return status;
}

static int bq25890_get_battery_voreg(struct charger_device *chg_dev, u32 *cv)
{
    u8 reg_cv = 0;
    reg_cv = bq25890_get_vreg();

    if (reg_cv > 0) {
        *cv = (reg_cv > 0x30) ? 4608000 : VBAT_CV_VTH[reg_cv];
        return 0;
    }
    else {
        *cv = 0;
        return -1;
    }
}

static int bq25890_reset_watch_dog_timer(struct charger_device *chg_dev)
{
    int status = 0;

    bq25890_wd_reset(1);

    return status;
}

static int charging_set_vindpm(u32 v)
{
    int status = 0;

    bq25890_set_vindpm(v);

    return status;
}

static int bq25890_get_is_power_path_enable(struct charger_device *chg_dev,
                                            bool *en)
{
    int ret = 0;
    u32 reg_vindpm = 0;

    reg_vindpm = bq25890_get_vindpm();
    *en = (reg_vindpm == 0x7F) ? false : true;

    return ret;
}

static int bq25890_set_vindpm_voltage(struct charger_device *chg_dev,
                                      u32 vindpm)
{
    int status = 0;
    unsigned int array_size;
    bool is_power_path_enable = true;
    int ret = 0;

    vindpm /= 1000;
    array_size = ARRAY_SIZE(VINDPM_REG);
    vindpm = bmt_find_closest_level(VINDPM_REG, array_size, vindpm);
    vindpm = charging_parameter_to_value(VINDPM_REG, array_size, vindpm);

    /*
     * Since BQ25890 uses vindpm to turn off power path
     * If power path is disabled, do not adjust mivr
     */
    ret = bq25890_get_is_power_path_enable(chg_dev, &is_power_path_enable);
    if (ret == 0 && !is_power_path_enable) {
        pr_info("%s: power path is disable, skip setting vindpm = %d\n",
                __func__,
                vindpm);
        return 0;
    }
    bq25890_set_force_vindpm(1);
    charging_set_vindpm(vindpm);
    /*bq25890_set_en_hiz(en);*/

    return status;
}

/* Begin add by jin.wang for jira 2064 on 2021-11-30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static int bq25890_get_charging_status(struct charger_device *chg_dev,
                                       int *status)
{
	unsigned char reg_value = 0;
	*status = POWER_SUPPLY_STATUS_UNKNOWN;

	bq25890_read_interface(BQ25890_CONB, &reg_value, 0x3, 3);
	pr_err("%s: reg_value:0x%x\n", __func__, reg_value);

	switch (reg_value) {
	case 0x00:
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case 0x01:
	case 0x02:
		*status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case 0x03:
		if (now_cv < max_cv) {  // sync with bq25890_is_charging_done
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

static int bq25890_is_charging_done(struct charger_device *chg_dev,
                                       bool *is_done)
#else
static int bq25890_get_charging_status(struct charger_device *chg_dev,
                                       bool *is_done)
#endif
/* End mod by jin.wang */
{
    int status = 0;
/* Begin modified by jin.wang for PR 11476064 on 2021-9-8 */
    unsigned char reg_value = 0;
/* End modified by jin.wang for PR 11476064 on 2021-9-8 */

    bq25890_read_interface(BQ25890_CONB, &reg_value, 0x3, 3); /* ICHG to BAT */
	//Begin Modified by qiuguangliang for 11297832 on 2021-06-30
	printk("\033[44m  %s reg_value:%d \033[0m \n", __func__, reg_value);
	if (now_cv < max_cv) {
		*is_done = false;
		printk("\033[44m  now_cv(%d) < max_cv(%d) never full  \033[0m \n", now_cv, max_cv);
		return 0;
	}
	//End Modified by qiuguangliang for 11297832 on 2021-06-30
    if (reg_value == 0x3) /* check if chrg done */
        *is_done = true;
    else
        *is_done = false;

    return status;
}

static int bq25890_enable_power_path(struct charger_device *chg_dev, bool en)
{
    int ret = 0;

    if (en)
        bq25890_set_en_hiz(0x0);
    else
        bq25890_set_en_hiz(0x1);

    return ret;
}
// add begin by TCT-cuiping.shi for otg kick timer
#if 0
static void enable_boost_polling(struct charger_device *chg_dev,bool poll_en)
{

    struct bq25890_info *g_info = dev_get_drvdata(&chg_dev->dev);
    if (g_info) {
        if (poll_en) {
            bq25890_wd_reset(1);
            gtimer_start(&g_info->otg_kthread_gtimer,
                g_info->polling_interval);

            g_info->polling_enabled = true;
        } else {
            g_info->polling_enabled = false;
            gtimer_stop(&g_info->otg_kthread_gtimer);
        }
    }
}

static void usbotg_boost_kick_work(struct work_struct *work)
{
    struct bq25890_info *g_info =
        container_of(work, struct bq25890_info, kick_work);

    pr_info("usbotg_boost_kick_work\n");

    bq25890_wd_reset(1);

    if (g_info->polling_enabled == true) {
        gtimer_start(&g_info->otg_kthread_gtimer,
            g_info->polling_interval);
    }
}

static int usbotg_gtimer_func(struct gtimer *data)
{
    struct bq25890_info *g_info =
        container_of(data, struct bq25890_info, otg_kthread_gtimer);

    queue_work(g_info->otg_boost_workq,
        &g_info->kick_work);

    return 0;
}
#endif
// add end by TCT-cuiping.shi for otg kick timer

static int bq25890_enable_otg(struct charger_device *chg_dev, bool en)
{
    int ret = 0;

    /* If OTG is enabled, BC1.2 should not work */
#if 0//ndef JRD_PROJECT_MORGAN4GTMO // change begin for otg set
    bq25890_set_auto_dpdm(!en);
#endif // change end for otg set
    //Begin add by bing-zhang for 11693754 on 2021/12/28
    bq25890_enable_power_path(chg_dev, true);
    //End add by bing-zhang for 11693754 on 2021/12/28
    bq25890_otg_en(en);
    // add begin by TCT-cuiping.shi for otg kick timer
#if 0
    enable_boost_polling(chg_dev,en);
#endif
    // add end by TCT-cuiping.shi for otg kick timer

    return ret;
}

static int bq25890_set_boost_current_limit(struct charger_device *chg_dev,
                                           u32 uA)
{
    int ret = 0;
    u32 array_size = 0;
    u32 boost_ilimit = 0;
    u8 boost_reg = 0;

    uA /= 1000;
    array_size = ARRAY_SIZE(BOOST_CURRENT_LIMIT);
    boost_ilimit = bmt_find_closest_level(BOOST_CURRENT_LIMIT, array_size, uA);
    boost_reg = charging_parameter_to_value(BOOST_CURRENT_LIMIT,
                                            array_size,
                                            boost_ilimit);
    bq25890_set_boost_ilim(boost_reg);
    pr_err("%s, boot current=%dmA\n", __func__, uA);

    return ret;
}

static int bq25890_enable_safetytimer(struct charger_device *chg_dev, bool en)
{
    int status = 0;

    bq25890_en_chg_timer(en);

    return status;
}

static int bq25890_get_is_safetytimer_enable(struct charger_device *chg_dev,
                                             bool *en)
{
    int ret = 0;
    u32 reg_safetytimer = 0;

    reg_safetytimer = bq25890_get_chg_timer_enable();
    *en = (reg_safetytimer) ? true : false;

    return ret;
}

static int bq25890_set_ta_current_pattern(struct charger_device *chg_dev,
                                          bool is_increase)
{
    /*unsigned int charging_status = KAL_FALSE; */
#if 1
    bq25890_pumpx_up(is_increase);
    #ifdef BQ25890_LOG_ON
    pr_debug("Pumping up adaptor...");
    #else
    pr_err("%s: Pumping up adaptor...", __func__);
    #endif

#else
    if (increase == KAL_TRUE) {
        bq25890_set_ichg(0x0);  /* 64mA */
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_increase() on 1");
        msleep(85);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_increase() off 1");
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_increase() on 2");
        msleep(85);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_increase() off 2");
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_increase() on 3");
        msleep(281);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_increase() off 3");
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_increase() on 4");
        msleep(281);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_increase() off 4");
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_increase() on 5");
        msleep(281);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_increase() off 5");
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_increase() on 6");
        msleep(485);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_increase() off 6");
        msleep(50);

        pr_info("mtk_ta_increase() end\n");

        bq25890_set_ichg(0x8);  /* 512mA */
        msleep(200);
    } else {
        bq25890_set_ichg(0x0);  /* 64mA */
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_decrease() on 1");
        msleep(281);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_decrease() off 1");
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_decrease() on 2");
        msleep(281);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_decrease() off 2");
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_decrease() on 3");
        msleep(281);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_decrease() off 3");
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_decrease() on 4");
        msleep(85);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_decrease() off 4");
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_decrease() on 5");
        msleep(85);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_decrease() off 5");
        msleep(85);

        bq25890_set_ichg(0x8);  /* 512mA */
        pr_debug("mtk_ta_decrease() on 6");
        msleep(485);

        bq25890_set_ichg(0x0);  /* 64mA */
        pr_debug("mtk_ta_decrease() off 6");
        msleep(50);

        pr_info("mtk_ta_decrease() end\n");

        bq25890_set_ichg(0x8);  /* 512mA */
    }
#endif
    return 0;
}

static int bq25890_set_pep20_efficiency_table(struct charger_device *chg_dev)
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

static int bq25890_set_ta_reset(struct charger_device *chg_dev)
{
#if 0 //change begin by TCT-cuiping.shi
    bq25890_set_vindpm(0x13);
    #else
    bq25890_write_byte(BQ25890_COND, 0x93);
#endif //change end by TCT-cuiping.shi
    bq25890_set_ichg(8);

    bq25890_set_ico_en_start(0);
    bq25890_set_iinlim(0x0);
    msleep(250);
    bq25890_set_iinlim(0xc);
#if 0//ndef JRD_PROJECT_MORGAN4GTMO // add by TCT-cuiping.shi for charge
    bq25890_set_ico_en_start(1);
    #endif // add by TCT-cuiping.shi for charge
    return 0;
}

/* Begin mod by jin.wang for task 5195 at 2021-12-27 */
static struct timespec ptime[13];
/* End mod by jin.wang */

static int cptime[13][2];

static int dtime(int i)
{
    struct timespec time;

    time = timespec_sub(ptime[i], ptime[i - 1]);
    return time.tv_nsec / 1000000;
}

#define PEOFFTIME 40
#define PEONTIME 90

static int bq25890_set_ta20_current_pattern(struct charger_device *chg_dev,
                                            u32 chr_vol)
{
    int value;
    int i, j = 0;
    int flag;

#if 0 //change begin by TCT-cuiping.shi
    bq25890_set_vindpm(0x13);
#else
    bq25890_write_byte(BQ25890_COND, 0x93);
#endif //change end by TCT-cuiping.shi

/* Begin mod by jin.wang for androidT on 2022.4.6 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
    bq25890_set_ichg(0x0D);  // 832mA
#else
    bq25890_set_iinlim(0x3f);
#endif
/* End mod by jin.wang */

    bq25890_set_ico_en_start(0);

/* Begin del by jin.wang for androidT on 2022.4.6 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
    bq25890_enable_charging(chg_dev, true);
#endif
/* End del by jin.wang */

    usleep_range(1000, 1200);
    value = (chr_vol - 5500000) / 500000;

    bq25890_set_iinlim(0x0);
    msleep(70);

    get_monotonic_boottime(&ptime[j++]);
    for (i = 4; i >= 0; i--) {
        flag = value & (1 << i);

        if (flag == 0) {
            bq25890_set_iinlim(0xc);
            msleep(PEOFFTIME);
            get_monotonic_boottime(&ptime[j]);
            cptime[j][0] = PEOFFTIME;
            cptime[j][1] = dtime(j);
            if (cptime[j][1] < 30 || cptime[j][1] > 65) {
                pr_info("charging_set_ta20_current_pattern fail1: idx:%d target:%d actual:%d\n",
                        i,
                        PEOFFTIME,
                        cptime[j][1]);
                //return -EIO; //modify by qiuguangliang for 11301112 on 20210705
            }
            j++;
            bq25890_set_iinlim(0x0);
            msleep(PEONTIME);
            get_monotonic_boottime(&ptime[j]);
            cptime[j][0] = PEONTIME;
            cptime[j][1] = dtime(j);
            if (cptime[j][1] < 90 || cptime[j][1] > 115) {
                pr_info("charging_set_ta20_current_pattern fail2: idx:%d target:%d actual:%d\n",
                        i,
                        PEOFFTIME,
                        cptime[j][1]);
                //return -EIO; //modify by qiuguangliang for 11301112 on 20210705
            }
            j++;

        }
        else {
            bq25890_set_iinlim(0xc);
            msleep(PEONTIME);
            get_monotonic_boottime(&ptime[j]);
            cptime[j][0] = PEONTIME;
            cptime[j][1] = dtime(j);
            if (cptime[j][1] < 90 || cptime[j][1] > 115) {
                pr_info("charging_set_ta20_current_pattern fail3: idx:%d target:%d actual:%d\n",
                        i,
                        PEOFFTIME,
                        cptime[j][1]);
                //return -EIO; //modify by qiuguangliang for 11301112 on 20210705
            }
            j++;
            bq25890_set_iinlim(0x0);
            msleep(PEOFFTIME);
            get_monotonic_boottime(&ptime[j]);
            cptime[j][0] = PEOFFTIME;
            cptime[j][1] = dtime(j);
            if (cptime[j][1] < 30 || cptime[j][1] > 65) {
                pr_info("charging_set_ta20_current_pattern fail4: idx:%d target:%d actual:%d\n",
                        i,
                        PEOFFTIME,
                        cptime[j][1]);
                //return -EIO; //modify by qiuguangliang for 11301112 on 20210705
            }
            j++;
        }
    }

    bq25890_set_iinlim(0xc);
    msleep(160);
    get_monotonic_boottime(&ptime[j]);
    cptime[j][0] = 160;
    cptime[j][1] = dtime(j);
    if (cptime[j][1] < 150 || cptime[j][1] > 240) {
        pr_info("charging_set_ta20_current_pattern fail5: idx:%d target:%d actual:%d\n",
                i,
                PEOFFTIME,
                cptime[j][1]);
                //return -EIO; //modify by qiuguangliang for 11301112 on 20210705
    }
    j++;

    bq25890_set_iinlim(0x0);
    msleep(30);
    bq25890_set_iinlim(0xc);

/* Begin mod by jin.wang task 2064 on 2021.11.1 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pr_info("%d %d: %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
            chr_vol, value, cptime[1][0], cptime[2][0], cptime[3][0],
            cptime[4][0], cptime[5][0], cptime[6][0], cptime[7][0],
            cptime[8][0], cptime[9][0], cptime[10][0], cptime[11][0]);
	pr_info("%3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
            cptime[1][1], cptime[2][1], cptime[3][1], cptime[4][1],
            cptime[5][1], cptime[6][1], cptime[7][1], cptime[8][1],
            cptime[9][1], cptime[10][1], cptime[11][1]);
#else
    pr_info("[charging_set_ta20_current_pattern]:chr_vol:%d bit:%d time:%3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
            chr_vol,
            value,
            cptime[1][0],
            cptime[2][0],
            cptime[3][0],
            cptime[4][0],
            cptime[5][0],
            cptime[6][0],
            cptime[7][0],
            cptime[8][0],
            cptime[9][0],
            cptime[10][0],
            cptime[11][0]);

    pr_info("[charging_set_ta20_current_pattern2]:chr_vol:%d bit:%d time:%3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d!!\n",
            chr_vol,
            value,
            cptime[1][1],
            cptime[2][1],
            cptime[3][1],
            cptime[4][1],
            cptime[5][1],
            cptime[6][1],
            cptime[7][1],
            cptime[8][1],
            cptime[9][1],
            cptime[10][1],
            cptime[11][1]);
#endif
/* End mod by jin.wang */

#if 0//ndef JRD_PROJECT_MORGAN4GTMO // add by TCT-cuiping.shi for charge
    bq25890_set_ico_en_start(1);
#endif // add by TCT-cuiping.shi for charge

/* Begin del by jin.wang task 2064 on 2021.10.26 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
    bq25890_set_iinlim(0x3f);
#endif
/* End del by jin.wang */

    return 0;
}

static int bq25890_enable_cable_drop_comp(struct charger_device *chg_dev, bool en)
{
    pr_err("%s: en = %d\n", __func__, en);
    return bq25890_set_ta20_current_pattern(chg_dev, 21000000);
}

static int bq25890_dump_register(struct charger_device *chg_dev)
{
    unsigned char i = 0;
    unsigned char ichg = 0;
    unsigned char ichg_reg = 0;
    unsigned char iinlim = 0;
    unsigned char vbat = 0;
    unsigned char chrg_state = 0;
    unsigned char chr_en = 0;
    unsigned char vbus = 0;
    unsigned char vdpm = 0;
    unsigned char fault = 0;
    // add begin by TCT-cuiping.shi for log
    unsigned char vindpm = 0;
    unsigned char vindpm_os = 0;
    unsigned char vindpm_os_sy6970qcc = 0;
    unsigned char iinlim_eff = 0;
    unsigned char cv_set = 0;
    // add end by TCT-cuiping.shi for log

    bq25890_ADC_start(1);
    for (i = 0; i < BQ25890_REG_NUM; i++) {
        bq25890_read_byte(i, &bq25890_reg[i]);
        bq_debug("[bq25890 reg@][0x%x]=0x%x ", i, bq25890_reg[i]);
    }

    iinlim = (bq25890_reg[BQ25890_CON0]
                    & (CON0_IINLIM_MASK << CON0_IINLIM_SHIFT))
                    >> CON0_IINLIM_SHIFT;
    chrg_state = (bq25890_reg[BQ25890_CONB]
                    & (CONB_CHRG_STAT_MASK << CONB_CHRG_STAT_SHIFT))
                    >> CONB_CHRG_STAT_SHIFT;
    chr_en = (bq25890_reg[BQ25890_CON3]
                    & (CON3_CHG_CONFIG_MASK << CON3_CHG_CONFIG_SHIFT))
                    >> CON3_CHG_CONFIG_SHIFT;
    ichg_reg = (bq25890_reg[BQ25890_CON4] & (CON4_ICHG_MASK << CON4_ICHG_SHIFT))
                    >> CON4_ICHG_SHIFT;
    ichg = (bq25890_reg[BQ25890_CON12]
                    & (CON12_ICHG_STAT_MASK << CON12_ICHG_STAT_SHIFT))
                    >> CON12_ICHG_STAT_SHIFT;
    vbat = (bq25890_reg[BQ25890_CONE] & (CONE_VBAT_MASK << CONE_VBAT_SHIFT))
                    >> CONE_VBAT_SHIFT;
    vbus = (bq25890_reg[BQ25890_CON11] & (CON11_VBUS_MASK << CON11_VBUS_SHIFT))
                    >> CON11_VBUS_SHIFT;
    vdpm = (bq25890_reg[BQ25890_CON13]
                    & (CON13_VDPM_STAT_MASK << CON13_VDPM_STAT_SHIFT))
                    >> CON13_VDPM_STAT_SHIFT;
    fault = (bq25890_reg[BQ25890_CONC]
                    & (CONC_CHRG_FAULT_MASK << CONC_CHRG_FAULT_SHIFT))
                    >> CONC_CHRG_FAULT_SHIFT;
    vindpm = (bq25890_reg[BQ25890_COND]
                    & (COND_VINDPM_MASK << COND_VINDPM_SHIFT))
                    >> COND_VINDPM_SHIFT;
    vindpm_os = (bq25890_reg[BQ25890_CON1]
                    & (CON1_VINDPM_OS_MASK << CON1_VINDPM_OS_SHIFT))
                    >> CON1_VINDPM_OS_SHIFT;
    vindpm_os_sy6970qcc = (bq25890_reg[BQ25890_CON1]
                    & (CON1_VINDPM_OS_SY6970QCC_MASK << CON1_VINDPM_OS_SY6970QCC_SHIFT))
                    >> CON1_VINDPM_OS_SY6970QCC_SHIFT;
    iinlim_eff = (bq25890_reg[BQ25890_CON13]
                    & (CON13_IINLIM_MASK << CON13_IINLIM_SHIFT))
                    >> CON13_IINLIM_SHIFT;
    cv_set = (bq25890_reg[BQ25890_CON6] & (CON6_VREG_MASK << CON6_VREG_SHIFT))
                    >> CON6_VREG_SHIFT;
    pr_err("[bq25890]0=0x%02x,1=0x%02x,2=0x%02x,3=0x%02x,4=0x%02x,\
5=0x%02x,6=0x%02x,7=0x%02x,8=0x%02x,9=0x%02x,A=0x%02x,\
B=0x%02x,C=0x%02x,D=0x%02x,E=0x%02x,F=0x%02x,0x10=0x%02x,\
0x11=0x%02x,0x12=0x%02x,0x13=0x%02x,0x14=0x%02x,\
Ichg_set=%d,Ilim_set=%d,cv_set=%d,Vbus_adc=%d,err=%d,Ibat_adc=%d,\
Vbat_adc=%d,ChrStat=%d,CHGEN=%d,VDPM=%d,%s=%d,iinlim_eff=%d\n",
            bq25890_reg[BQ25890_CON0],
            bq25890_reg[BQ25890_CON1],
            bq25890_reg[BQ25890_CON2],
            bq25890_reg[BQ25890_CON3],
            bq25890_reg[BQ25890_CON4],
            bq25890_reg[BQ25890_CON5],
            bq25890_reg[BQ25890_CON6],
            bq25890_reg[BQ25890_CON7],
            bq25890_reg[BQ25890_CON8],
            bq25890_reg[BQ25890_CON9],
            bq25890_reg[BQ25890_CONA],
            bq25890_reg[BQ25890_CONB],
            bq25890_reg[BQ25890_CONC],
            bq25890_reg[BQ25890_COND],
            bq25890_reg[BQ25890_CONE],
            bq25890_reg[BQ25890_CONF],
            bq25890_reg[BQ25890_CON10],
            bq25890_reg[BQ25890_CON11],
            bq25890_reg[BQ25890_CON12],
            bq25890_reg[BQ25890_CON13],
            bq25890_reg[BQ25890_CON14],
            ichg_reg * 64,
            iinlim * 50 + 100,
            (cv_set > 0x30) ? 4608 : (VBAT_CV_VTH[cv_set] / 1000),
            vbus * 100 + 2600,
            fault,
            ichg * 50,
            vbat * 20 + 2304,
            chrg_state,
            chr_en,
            vdpm,
            ((bq25890_reg[BQ25890_COND] & 0x80) == 0x80) ? "vindpm" : "vindpm_os",
            ((bq25890_reg[BQ25890_COND] & 0x80) == 0x80) ? (vindpm * 100 + 2600) :
            ((g_sy6970qcc_hw_exist) ? (vindpm_os * 100) : (vindpm_os ? 600 : 400)),
            iinlim_eff * 50 + 100);

    return 0;
}

/* Begin add by jin.wang for jira 5550 on 2022-1-7 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
static void chg_stat_work_handler(struct work_struct *data)
{
	struct bq25890_info *info = NULL;
	static unsigned char last_stat = 0xFF;
	unsigned char curr_stat = 0;
	static unsigned char last_pg_stat = 0xFF;
	unsigned char curr_pg_stat = 0;

	int ret = 0;

	pr_info("%s\n", __func__);
	info = container_of(to_delayed_work(data),
					struct bq25890_info, chg_stat_work);
	if (IS_ERR_OR_NULL(info)) {
		pr_err("[%s]: NULL pointer\n", __func__);
		return;
	};

	ret = bq25890_read_interface(BQ25890_CONB,
							&curr_stat,
							CONB_CHRG_STAT_MASK,
							CONB_CHRG_STAT_SHIFT);
	curr_pg_stat = bq25890_get_pg_state();
	if ((ret == 1) && curr_stat != last_stat && last_pg_stat != curr_pg_stat) {
		pr_info("[%s]: %d -> %d, %d->%d\n", __func__,
				last_stat, curr_stat, last_pg_stat, curr_pg_stat);
		last_stat = curr_stat;
        last_pg_stat = curr_pg_stat;
		power_supply_changed(info->chg_psy);
	}
}
#endif
/* End add by jin.wang */

static irqreturn_t bq25890_irq_handler(int irq, void *data)
{
#ifdef BQ25890_CHARGER_TYPE_DETECT
    u8 pg_stat = 0;
    enum charger_type org_chg_type;
    struct bq25890_info *info = (struct bq25890_info *)data;
#endif
    struct bq25890_info *info = (struct bq25890_info *)data;
    bool en = false;
    pr_info("%s\n", __func__);

    /* Skip irq if in OTG mode */
    bq25890_is_otg_en(&en);
    if (en)
        return IRQ_HANDLED;

/* Begin add by jin.wang for jira 5550 on 2022-1-7 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	schedule_delayed_work(&info->chg_stat_work,
				msecs_to_jiffies(200));
#endif
/* End add by jin.wang */

#ifndef BQ25890_CHARGER_TYPE_DETECT
    return IRQ_HANDLED;
#else
    /* Set vindpm to 4.5V */
    bq25890_set_force_vindpm(1);
    bq25890_set_vindpm(0x13);

    pg_stat = bq25890_get_pg_state();

    org_chg_type = info->chg_type;
    if (pg_stat) {
        info->chg_type = bq25890_get_charger_type(info);
    } else {
#if defined(CONFIG_PROJECT_PHY) || defined(CONFIG_PHY_MTK_SSUSB)
        Charger_Detect_Init();
#endif
        info->chg_type = CHARGER_UNKNOWN;
        pr_info("%s: plugout\n", __func__);
    }
    if (info->chg_type != org_chg_type)
        bq25890_set_charger_type(info);

    return IRQ_HANDLED;
#endif
}

static int bq25890_register_irq(struct bq25890_info *info)
{
    int ret = 0;
#if 0
    struct device_node *np;

    /* Parse irq number from dts */
    np = of_find_node_by_name(NULL, info->eint_name);
    if (np)
        info->irq = irq_of_parse_and_map(np, 0);
    else {
        pr_err("%s: cannot get node\n", __func__);
        ret = -ENODEV;
        goto err_nodev;
    }
#else
/* Begin mod by jin.wang for androidT on 2022-4-8 */
	if (info->intr_gpio <= 0) {
		dev_err(info->dev, "%s intr_gpio invalid(%d)\n",
				__func__, info->intr_gpio);
		return -EINVAL;
	}

	ret = devm_gpio_request_one(info->dev, info->intr_gpio, GPIOF_DIR_IN,
			devm_kasprintf(info->dev, GFP_KERNEL,
			"bq2589x_intr_gpio.%s", dev_name(info->dev)));
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
/* End mod by jin.wang */
#endif

    pr_info("%s: irq = %d\n", __func__, info->irq);

    /* Request threaded IRQ */
    ret = devm_request_threaded_irq(info->dev,
                                    info->irq,
                                    NULL,
                                    bq25890_irq_handler,
                                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                    info->eint_name,
                                    info);
    if (ret < 0) {
        pr_err("%s: request thread irq failed\n", __func__);
        goto err_request_irq;
    }

    enable_irq_wake(info->irq);
    return 0;

//err_nodev:
err_request_irq: return ret;
}

static int bq25890_parse_dt(struct bq25890_info *info, struct device *dev)
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
        info->chg_props.alias_name = "bq25890";
        pr_warn("%s: no alias name\n", __func__);
    }

    if (of_property_read_string(np, "eint_name", &info->eint_name) < 0) {
        info->eint_name = "chr_stat";
        pr_warn("%s: no eint name\n", __func__);
    }

/* Begin mod by jin.wang for androidT on 2022-4-18 */
	ret = of_get_named_gpio(np, "bq2589x,intr_gpio", 0);
	if (ret < 0) {
		dev_err(info->dev, "%s no bq2589x,intr_gpio(%d)\n",
				      __func__, ret);
		info->intr_gpio = -1;
	} else {
		info->intr_gpio = ret;
	}

	ret = of_get_named_gpio(np, "bq2589x,en_gpio", 0);
	if (ret < 0) {
		dev_err(info->dev, "%s no bq2589x,en_gpio(%d)\n",
				    __func__, ret);
		info->ceb_gpio = -1;
	} else {
		info->ceb_gpio = ret;
	}
/* End mod by jin.wang */

    return 0;
}

#if IS_ENABLED(CONFIG_TCT_CHARGER)
static int bq25890_do_event(struct charger_device *chg_dev, u32 event,
                                u32 args)
{
    struct bq25890_info *info = charger_get_data(chg_dev);
    pr_info("%s\n", __func__);
    if (IS_ERR_OR_NULL(info->chg_psy)) {
        pr_notice("%s: cannot get psy\n", __func__);
        return -ENODEV;
    }

    switch (event) {
    case EVENT_FULL:
    case EVENT_RECHARGE:
    case EVENT_DISCHARGE:
        msleep(100); //avoid to can't get charge state
        pr_info("%s, chr_state=0x%x\n", __func__, bq25890_get_chrg_state());
        power_supply_changed(info->chg_psy);
        break;
    default:
        break;
    }

    return 0;
}
#else
static int bq25890_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{

//Begin Modified by qiuguangliang for 11297832 on 2021-06-30
	struct bq25890_info *info = dev_get_drvdata(&chg_dev->dev);

	if (info == NULL)
		return -EINVAL;
	if (chg_dev == NULL)
		return -EINVAL;

	printk("%s: event = %d\n", __func__, event);
	power_supply_changed(info->chg_psy);
//End Modified by qiuguangliang for 11297832 on 2021-06-30

#if 0
    pr_info("%s: event = %d\n", __func__, event);
    switch (event) {
    case EVENT_EOC:
        charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
        break;
    case EVENT_RECHARGE:
        charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
        break;
    default:
        break;
    }
#endif
    return 0;
}
#endif

static int bq25890_charger_plug_in(struct charger_device *chg_dev)
{
    if (0x1 == bq25890_get_wdt_state()) {
        bq_err("%s wdt timeout,reset some reg\n", __func__);
        bq25890_wd_reset(1);
        bq25890_set_en_ilim(0);
        bq25890_batfet_rst_en(0);

/* Begin modified by bitao.xiong for defect-11676271 on 2021-11-24 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
        bq25890_set_VBAT_clamp(0x0);
        bq25890_set_VBAT_IR_compensation(0x0);
#else
        bq25890_set_VBAT_clamp(0x4);
        bq25890_set_VBAT_IR_compensation(0x3);
#endif
/* End modified by bitao.xiong for defect-11676271 on 2021-11-24 */

        bq25890_set_iterml(0x2);//192mA //Modified by qiuguangliang for 11297832 on 2021-07-24
        bq25890_set_batlowv(0x1);//3.0V
        bq25890_set_iprechg(0x1);//128mA
        bq25890_set_force_vindpm(1);
        bq25890_set_vindpm(0x13); //4.5V
    }

	// TODO: this code maybe conflict with mmitest design
    bq25890_enable_charging(chg_dev, true);
    return 0;
}

static int bq25890_charger_plug_out(struct charger_device *chg_dev)
{
	// TODO: this code maybe conflict with mmitest design
    bq25890_enable_charging(chg_dev, false);
    return 0;
}

static int bq25890_get_mivr_state(struct charger_device *chg_dev, bool *in_loop)
{
    *in_loop = bq25890_get_vdpm_state();
    return 0;
}

static int bq25890_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
    *uA = 0;
    return 0;
}

static int bq25890_get_min_aicr(struct charger_device *chg_dev, u32 *uA)
{
    *uA = 100000;
    return 0;
}

static struct charger_ops bq25890_chg_ops = {
#if 0
    .enable_hz = bq25890_enable_hz,
#endif

    /* Normal charging */
    .plug_in = bq25890_charger_plug_in,
    .plug_out = bq25890_charger_plug_out,
    .dump_registers = bq25890_dump_register,
    .enable = bq25890_enable_charging,
    .get_charging_current = bq25890_get_current,
    .set_charging_current = bq25890_set_current,
    .get_input_current = bq25890_get_input_current,
    .set_input_current = bq25890_set_input_current,
    .get_constant_voltage = bq25890_get_battery_voreg,
    .set_constant_voltage = bq25890_set_cv_voltage,
    .kick_wdt = bq25890_reset_watch_dog_timer,
    .set_mivr = bq25890_set_vindpm_voltage,
    .get_mivr_state = bq25890_get_mivr_state,

/* Begin mod by jin.wang for jira 2064 on 2021-11-30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	.is_charging_done = bq25890_is_charging_done,
	.get_chg_status = bq25890_get_charging_status,
#else
    .is_charging_done = bq25890_get_charging_status,
#endif
/* End mod by jin.wang */

    .get_min_charging_current = bq25890_get_min_ichg,
    .get_min_input_current = bq25890_get_min_aicr,

    /* Safety timer */
    .enable_safety_timer = bq25890_enable_safetytimer,
    .is_safety_timer_enabled = bq25890_get_is_safetytimer_enable,

    /* Power path */
    .enable_powerpath = bq25890_enable_power_path,
    .is_powerpath_enabled = bq25890_get_is_power_path_enable,

#ifdef BQ25890_CHARGER_TYPE_DETECT
    /* Charger type detection */
    .enable_chg_type_det = bq25890_enable_chg_type_det,
#endif

    /* OTG */
    .enable_otg = bq25890_enable_otg,
    .set_boost_current_limit = bq25890_set_boost_current_limit,

    /* PE+/PE+20 */
    .send_ta_current_pattern = bq25890_set_ta_current_pattern,
    .set_pe20_efficiency_table = bq25890_set_pep20_efficiency_table,
    .send_ta20_current_pattern = bq25890_set_ta20_current_pattern,
    .reset_ta = bq25890_set_ta_reset,
    .enable_cable_drop_comp = bq25890_enable_cable_drop_comp,

    /* Event */
    .event = bq25890_do_event,
};


static int bq25890_boost_enable(struct regulator_dev *rdev)
{
	pr_err("bq25890 otg regulator enable\n");

/* Begin add by jin.wang jira 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	bq25890_chg_en(0);
#endif
/* End add by jin.wang */

	bq25890_otg_en(1);
	return 0;
}

static int bq25890_boost_disable(struct regulator_dev *rdev)
{
	pr_err("bq25890 otg regulator disable\n");
	bq25890_otg_en(0);

/* Begin add by jin.wang jira 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	bq25890_chg_en(1);
#endif
/* End add by jin.wang */

	return 0;
}

static int bq25890_boost_is_enabled(struct regulator_dev *rdev)
{
    bool en;

    bq25890_is_otg_en(&en);
    
    return en;
}

static const struct regulator_ops bq25890_chg_otg_ops = {
	.enable = bq25890_boost_enable,
	.disable = bq25890_boost_disable,
	.is_enabled = bq25890_boost_is_enabled,
};

static const struct regulator_desc bq25890_otg_rdesc = {
	.name = "usb-otg-vbus",
	.ops = &bq25890_chg_otg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static const struct regulator_init_data bq25890_vbus_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};
/* Begin Modified by qiuguangliang for 11297832 on 2021-06-30 */
enum bq25890_charging_status {
	BQ2560X_CHG_STATUS_NOT_CHARGING = 0,
	BQ2560X_CHG_STATUS_PRECHARGE,
	BQ2560X_CHG_STATUS_FAST_CHARGING,
	BQ2560X_CHG_STATUS_DONE,
	BQ2560X_CHG_STATUS_MAX,
};
static enum power_supply_usb_type bq25890_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};
static enum power_supply_property bq25890_charge_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_USB_TYPE,
};
#define VBUS_THR 4000
static int bq25890_get_online(struct bq25890_info *bq, union power_supply_propval *val)
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
static int bq25890_get_charger_type(struct bq25890_info *bq, union power_supply_propval *val)
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

static int bq25890_charge_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct bq25890_info *bq = power_supply_get_drvdata(psy);
	int ret = 0;
	unsigned int chg_stat;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = bq25890_get_online(bq, val);
		if (ret)
			return -ENODATA;
		break;
	case POWER_SUPPLY_PROP_STATUS:
			chg_stat = bq25890_get_chrg_state();

		switch (chg_stat) {
			case BQ2560X_CHG_STATUS_NOT_CHARGING:
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
			case BQ2560X_CHG_STATUS_PRECHARGE:
			case BQ2560X_CHG_STATUS_FAST_CHARGING:
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
				break;
			case BQ2560X_CHG_STATUS_DONE:
				val->intval = POWER_SUPPLY_STATUS_FULL;
				break;
			default:
				ret = -ENODATA;
				break;
			}
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		ret = bq25890_get_charger_type(bq, val);
		if (ret)
			return -ENODATA;
		break;
	default:
		return 0;
	}

	return ret;
}

static char *bq25890_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

static const struct power_supply_desc bq25890_charge_desc = {
	.name = "bq25890",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = bq25890_charge_props,
	.num_properties = ARRAY_SIZE(bq25890_charge_props),
	.get_property = bq25890_charge_get_property,
	.usb_types = bq25890_usb_type,
	.num_usb_types = ARRAY_SIZE(bq25890_usb_type),

};
/* End Modified by qiuguangliang for 11297832 on 2021-06-30 */

#if IS_ENABLED(CONFIG_TCT_DEVICEINFO)
extern char charger_module_name[256];
#endif

static int bq25890_driver_probe(struct i2c_client *client,
                                const struct i2c_device_id *id)
{
#ifdef BQ25890_CHARGER_TYPE_DETECT
    int ret = 0, i = 0;
    struct bq25890_info *info = NULL;
    bool force_dpdm = false;
    unsigned int pg_stat = 0;
#else
    int ret = 0;
    struct bq25890_info *info = NULL;
#endif

/* Begin Modified by qiuguangliang for 11297832 on 2021-06-30 */
    struct power_supply_config psy_cfg = {};
/* End Modified by qiuguangliang for 11297832 on 2021-06-30 */
    struct regulator_config config = { };
    struct regulator_dev *rdev;

    pr_info("[bq25890_driver_probe]\n");

    info = devm_kzalloc(&client->dev, sizeof(struct bq25890_info), GFP_KERNEL);
    if (!info)
        return -ENOMEM;

    new_client = client;
    info->dev = &client->dev;

/* Begin modified by jin.wang for PR 11476064 on 2021-9-8 */
    bq25890_hw_component_detect();
    if ((!g_bq25890_hw_exist) && (!g_sy6970qcc_hw_exist))
        return -ENODEV;

    ret = bq25890_parse_dt(info, &client->dev);
    if (ret < 0)
        return ret;

    if (g_sy6970qcc_hw_exist)
        info->chg_props.alias_name = "sy6970qcc";
/* End modified by jin.wang for PR 11476064 on 2021-9-8 */

    /* Register charger device */
    info->chg_dev = charger_device_register(info->chg_dev_name,
                                            &client->dev,
                                            info,
                                            &bq25890_chg_ops,
                                            &info->chg_props);
    if (IS_ERR_OR_NULL(info->chg_dev)) {
        pr_err("%s: register charger device failed\n", __func__);
        ret = PTR_ERR(info->chg_dev);
        return ret;
    }

    /* bq25890_hw_init(); //move to charging_hw_xxx.c */
    bq25890_set_en_ilim(0);

    info->psy = power_supply_get_by_name("charger");
    if (!info->psy) {
        pr_err("%s: get power supply failed\n", __func__);
        //return -EINVAL;
    }

#ifdef BQ25890_CHARGER_TYPE_DETECT
    /* Force charger type detection */
#if defined(CONFIG_PROJECT_PHY) || defined(CONFIG_PHY_MTK_SSUSB)
    Charger_Detect_Init();
#endif
    msleep(50);
    pg_stat = bq25890_get_pg_state();
    if (pg_stat) {
        pr_info("%s: force charger type detection\n", __func__);
        /* Force dpdm will become 0 after detecting is finished */
        bq25890_set_force_dpdm(1);
        for (i = 0; i < 10; i++) {
            msleep(500);
            bq25890_get_force_dpdm(&force_dpdm);
            if (!force_dpdm)
                break;
        }
        info->chg_type = bq25890_get_charger_type(info);
        bq25890_set_charger_type(info);
    }

    bq25890_set_auto_dpdm(1);
#else
    ret = bq25890_config_interface(BQ25890_CON2, 0x0, 0x07, 0x0);
    bq25890_set_ico_en_start(0);
    ret = bq25890_write_byte(BQ25890_COND, 0x93);
    ret = bq25890_write_byte(BQ25890_CON5, 0x12); //Modified by qiuguangliang for 11297832 on 2021-07-24

/* Begin modified by bitao.xiong for defect-11676271 on 2021-11-24 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
    bq25890_set_VBAT_clamp(0x0);
    bq25890_set_VBAT_IR_compensation(0x0);
#else
    bq25890_set_VBAT_clamp(0x4);
    bq25890_set_VBAT_IR_compensation(0x3);
#endif
/* End modified by bitao.xiong for defect-11676271 on 2021-11-24 */

    bq25890_batfet_rst_en(0);

/* Begin add by jin.wang for jira 2064 at 2021-10-26 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
    bq25890_set_boost_vlim(0x8); // set otg vbus as 5V
    bq25890_set_boost_ilim(0x4);  // set otg max current as 1.65A
    bq25890_set_chg_timer(2);  // set chg safety timer as 24 hrs
#endif
/* End add by jin.wang */

#endif

/* Begin add by jin.wang for androidT on 2022.4.6 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
#if IS_ENABLED(TARGET_BUILD_MMITEST)
    bq25890_chg_en(0);  // turnoff charging in mmitest when boot up
#endif
#endif
/* End add by jin.wang */

/* Begin del by jin.wang for jira 5550 on 2022-1-7 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
    bq25890_register_irq(info);
#endif
/* End del by jin.wang */

#if 0
    gtimer_init(&info->otg_kthread_gtimer, info->dev, "otg_boost");
    info->otg_kthread_gtimer.callback = usbotg_gtimer_func;
    info->polling_interval = 20;
    info->otg_boost_workq = create_singlethread_workqueue("otg_boost_workq");
    INIT_WORK(&info->kick_work, usbotg_boost_kick_work);
#endif

    /* otg regulator */
	config.dev = &client->dev;
	config.driver_data = info;
	config.init_data = &bq25890_vbus_init_data;
	rdev = devm_regulator_register(&client->dev, &bq25890_otg_rdesc, &config);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		pr_err("otg regulator register err: %d\n", ret);
	}
    
    bq25890_set_wd_timer(0);

/* Begin Modified by qiuguangliang for 11297832 on 2021-06-30 */
	memcpy(&info->psy_desc, &bq25890_charge_desc, sizeof(info->psy_desc));

	psy_cfg.drv_data = info;
	psy_cfg.of_node = client->dev.of_node;
	psy_cfg.supplied_to = bq25890_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(bq25890_charger_supplied_to);
	info->chg_psy = devm_power_supply_register(&client->dev, &info->psy_desc, &psy_cfg);
	if (IS_ERR(info->chg_psy)) {
		printk("\033[44m %s %d info->chg_psy error\033[0m \n", __func__, __LINE__);
		return -EINVAL;
	}
/* End Modified by qiuguangliang for 11297832 on 2021-06-30 */

/* Begin add by jin.wang for jira 5550 on 2022-1-7 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	INIT_DELAYED_WORK(&info->chg_stat_work, chg_stat_work_handler);
	bq25890_register_irq(info);
#endif
/* End add by jin.wang */

    bq25890_dump_register(info->chg_dev);

#if IS_ENABLED(CONFIG_TCT_DEVICEINFO)
    if (g_bq25890_hw_exist)
        sprintf(charger_module_name, "BQ25890H:TI:0x%x:AMY0001192C1",
                (bq25890_reg[BQ25890_CON14] & (CON14_DEV_REV_MASK << CON14_DEV_REV_SHIFT)) >> CON14_DEV_REV_SHIFT);
    else if (g_sy6970qcc_hw_exist)
        sprintf(charger_module_name, "SY6970QCC:SILERGY:0x%x:AMY0001779C1",
                (bq25890_reg[BQ25890_CON14] & (CON14_DEV_REV_MASK << CON14_DEV_REV_SHIFT)) >> CON14_DEV_REV_SHIFT);
#endif

    return 0;
}

static unsigned char g_reg_value_bq25890;
static ssize_t bq25890_access_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    pr_info("[show_bq25890_access] 0x%x\n", g_reg_value_bq25890);
    return sprintf(buf, "%u\n", g_reg_value_bq25890);
}

static ssize_t bq25890_access_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t size)
{
    int ret = 0;
    char *pvalue = NULL, *addr, *val;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;

    pr_info("[store_bq25890_access]\n");

    if (buf != NULL && size != 0) {
        pr_info("[store_bq25890_access] buf is %s and size is %zu\n",
                buf,
                size);

        pvalue = (char*) buf;
        if (size > 3) {
            addr = strsep(&pvalue, " ");
            ret = kstrtou32(addr, 16, (unsigned int*) &reg_address);
        }
        else
            ret = kstrtou32(pvalue, 16, (unsigned int*) &reg_address);

        if (size > 3) {
            val = strsep(&pvalue, " ");
            ret = kstrtou32(val, 16, (unsigned int*) &reg_value);
            pr_info("[store_bq25890_access] write bq25890 reg 0x%x with value 0x%x !\n",
                    (unsigned int) reg_address,
                    reg_value);
            ret = bq25890_config_interface(reg_address, reg_value, 0xFF, 0x0);
        }
        else {
            ret = bq25890_read_interface(reg_address,
                                         &g_reg_value_bq25890,
                                         0xFF,
                                         0x0);
            pr_info("[store_bq25890_access] read bq25890 reg 0x%x with value 0x%x !\n",
                    (unsigned int) reg_address,
                    g_reg_value_bq25890);
            pr_info("[store_bq25890_access] Please use \"cat bq25890_access\" to get value\n");
        }
    }
    return size;
}

static DEVICE_ATTR_RW(bq25890_access); /* 664 */

static int bq25890_user_space_probe(struct platform_device *dev)
{
    int ret_device_file = 0;

    pr_info("******** bq25890_user_space_probe!! ********\n");

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25890_access);

    return 0;
}

static struct platform_device bq25890_user_space_device = {
    .name = "bq25890-user",
    .id = -1,
};

static struct platform_driver bq25890_user_space_driver = {
    .probe = bq25890_user_space_probe,
    .driver = {
        .name = "bq25890-user",
    },
};

#ifdef CONFIG_OF
static const struct of_device_id bq25890_of_match[] = {
    {.compatible = "mediatek,sw_charger"},
    {.compatible = "mediatek,bq25890h"},
    {},
};
#else
static struct i2c_board_info i2c_bq25890 __initdata = {
    I2C_BOARD_INFO("bq25890", (bq25890_SLAVE_ADDR_WRITE >> 1))
};
#endif

static struct i2c_driver bq25890_driver = {
    .driver = {
        .name = "bq25890",
#ifdef CONFIG_OF
        .of_match_table = bq25890_of_match,
#endif
                    },
    .probe = bq25890_driver_probe,
    .id_table = bq25890_i2c_id,
};

static int __init bq25890_init(void)
{
    int ret = 0;

    /* i2c registeration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
    pr_info("[bq25890_init] init start with i2c DTS");
#else
    pr_info("[bq25890_init] init start. ch=%d\n", bq25890_BUSNUM);
    i2c_register_board_info(bq25890_BUSNUM, &i2c_bq25890, 1);
#endif
    if (i2c_add_driver(&bq25890_driver) != 0) {
        pr_info("[bq25890_init] failed to register bq25890 i2c driver.\n");
    }
    else {
        pr_info("[bq25890_init] Success to register bq25890 i2c driver.\n");
    }

    /* bq25890 user space access interface */
    ret = platform_device_register(&bq25890_user_space_device);
    if (ret) {
        pr_info("****[bq25890_init] Unable to device register(%d)\n", ret);
        return ret;
    }
    ret = platform_driver_register(&bq25890_user_space_driver);
    if (ret) {
        pr_info("****[bq25890_init] Unable to register driver (%d)\n", ret);
        return ret;
    }

    return 0;
}

static void __exit bq25890_exit(void)
{
    i2c_del_driver(&bq25890_driver);
}
module_init (bq25890_init);
module_exit (bq25890_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq25890 Driver");
MODULE_AUTHOR("will cai <will.cai@mediatek.com>");
