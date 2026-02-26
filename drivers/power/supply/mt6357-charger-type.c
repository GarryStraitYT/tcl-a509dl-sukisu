// SPDX-License-Identifier: GPL-2.0

#include <linux/device.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mfd/mt6397/core.h>/* PMIC MFD core header */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>
#include <mtk_musb.h>
#include <linux/reboot.h>

/* ============================================================ */
/* pmic control start*/
/* ============================================================ */
#define PMIC_RG_BC11_VREF_VTH_ADDR                         0xb98
#define PMIC_RG_BC11_VREF_VTH_MASK                         0x3
#define PMIC_RG_BC11_VREF_VTH_SHIFT                        0

#define PMIC_RG_BC11_CMP_EN_ADDR                           0xb98
#define PMIC_RG_BC11_CMP_EN_MASK                           0x3
#define PMIC_RG_BC11_CMP_EN_SHIFT                          2

#define PMIC_RG_BC11_IPD_EN_ADDR                           0xb98
#define PMIC_RG_BC11_IPD_EN_MASK                           0x3
#define PMIC_RG_BC11_IPD_EN_SHIFT                          4

#define PMIC_RG_BC11_IPU_EN_ADDR                           0xb98
#define PMIC_RG_BC11_IPU_EN_MASK                           0x3
#define PMIC_RG_BC11_IPU_EN_SHIFT                          6

#define PMIC_RG_BC11_BIAS_EN_ADDR                          0xb98
#define PMIC_RG_BC11_BIAS_EN_MASK                          0x1
#define PMIC_RG_BC11_BIAS_EN_SHIFT                         8

#define PMIC_RG_BC11_BB_CTRL_ADDR                          0xb98
#define PMIC_RG_BC11_BB_CTRL_MASK                          0x1
#define PMIC_RG_BC11_BB_CTRL_SHIFT                         9

#define PMIC_RG_BC11_RST_ADDR                              0xb98
#define PMIC_RG_BC11_RST_MASK                              0x1
#define PMIC_RG_BC11_RST_SHIFT                             10

#define PMIC_RG_BC11_VSRC_EN_ADDR                          0xb98
#define PMIC_RG_BC11_VSRC_EN_MASK                          0x3
#define PMIC_RG_BC11_VSRC_EN_SHIFT                         11

#define PMIC_RG_BC11_DCD_EN_ADDR                           0xb98
#define PMIC_RG_BC11_DCD_EN_MASK                           0x1
#define PMIC_RG_BC11_DCD_EN_SHIFT                          13

#define PMIC_RGS_BC11_CMP_OUT_ADDR                         0xb98
#define PMIC_RGS_BC11_CMP_OUT_MASK                         0x1
#define PMIC_RGS_BC11_CMP_OUT_SHIFT                        14

#define PMIC_RGS_CHRDET_ADDR                               0xa88
#define PMIC_RGS_CHRDET_MASK                               0x1
#define PMIC_RGS_CHRDET_SHIFT                              4

#define R_CHARGER_1	330
#define R_CHARGER_2	39

struct mtk_charger_type {
	struct mt6397_chip *chip;
	struct regmap *regmap;
	struct platform_device *pdev;
	struct mutex ops_lock;

	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;
	struct power_supply *psy;
	struct power_supply_desc ac_desc;
	struct power_supply_config ac_cfg;
	struct power_supply *ac_psy;
	struct power_supply_desc usb_desc;
	struct power_supply_config usb_cfg;
	struct power_supply *usb_psy;

	struct iio_channel *chan_vbus;
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	struct delayed_work chr_work;  /* Modified by bitao.xiong for defect-10053764 on 2020-10-22 */
#else
	struct work_struct chr_work;
#endif

	enum power_supply_usb_type type;

/* Begin mod by jin.wang for androidT on 2022.4.8 */
//#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	int redet_cnt;
	enum power_supply_type real_charger_type; /* Added by bitao.xiong for defect-10053764 on 2020-10-22 */
	enum power_supply_type charge_type;
#endif
//#endif
/* End mod by jin.wang */

	int first_connect;
	int bc12_active;
	u32 bootmode;
	u32 boottype;
};

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

/* Begin added by bin.song.hz for task 10431118 on 2020-12-07 */
struct mtk_charger_type *chr_info;
/* End added by bin.song.hz for task 10431118 on 2020-12-07 */

static enum power_supply_property chr_type_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
/* Begin added by hailong.chen for defect 10130350 on 2020-11-23 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	POWER_SUPPLY_PROP_REAL_TYPE,
#endif
/* End added by hailong.chen for defect 10130350 on 2020-11-23 */
};

static enum power_supply_property mt_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
/* Begin added by hailong.chen for task 9517048 on 2020-06-19 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_HV_FLAG,  // add by jin.wang for jira 2064
#endif
/* End added by hailong.chen for task 9517048 on 2020-06-19 */
};

static enum power_supply_property mt_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

void bc11_set_register_value(struct regmap *map,
	unsigned int addr,
	unsigned int mask,
	unsigned int shift,
	unsigned int val)
{
	regmap_update_bits(map,
		addr,
		mask << shift,
		val << shift);
}

unsigned int bc11_get_register_value(struct regmap *map,
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

static void hw_bc11_init(struct mtk_charger_type *info)
{
#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
	int timeout = 300;
#endif
	msleep(200);
	if (info->first_connect == true) {
#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
		/* add make sure USB Ready */
		if (is_usb_rdy() == false) {
			pr_info("CDP, block\n");
			while (is_usb_rdy() == false && timeout > 0) {
				msleep(100);
				timeout--;
			}
			if (timeout == 0)
				pr_info("CDP, timeout\n");
			else
				pr_info("CDP, free\n");
		} else
			pr_info("CDP, PASS\n");
#endif
		info->first_connect = false;
	}

	/* RG_bc11_BIAS_EN=1 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_BIAS_EN_ADDR,
		PMIC_RG_BC11_BIAS_EN_MASK,
		PMIC_RG_BC11_BIAS_EN_SHIFT,
		1);
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPU_EN_ADDR,
		PMIC_RG_BC11_IPU_EN_MASK,
		PMIC_RG_BC11_IPU_EN_SHIFT,
		0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0);
	/* bc11_RST=1 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_RST_ADDR,
		PMIC_RG_BC11_RST_MASK,
		PMIC_RG_BC11_RST_SHIFT,
		1);
	/* bc11_BB_CTRL=1 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_BB_CTRL_ADDR,
		PMIC_RG_BC11_BB_CTRL_MASK,
		PMIC_RG_BC11_BB_CTRL_SHIFT,
		1);
	/* add pull down to prevent PMIC leakage */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x1);
#if IS_ENABLED(CONFIG_TCT_CHARGER)
    msleep(55);
#else
    msleep(50);
#endif

	pr_debug("%s\n",__func__);
#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
	Charger_Detect_Init();
#endif
}

/* Begin added by hailong.chen for defect 10130350 on 2020-11-23 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
static unsigned int hw_bc11_stepA1(struct mtk_charger_type *info)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x1);
#if IS_ENABLED(CONFIG_TCT_CHARGER)
    msleep(100);
#else
    msleep(80);
#endif
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(info->regmap,
					PMIC_RGS_BC11_CMP_OUT_ADDR,
					PMIC_RGS_BC11_CMP_OUT_MASK,
					PMIC_RGS_BC11_CMP_OUT_SHIFT);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x0);
	pr_err("%s: wChargerAvail=0x%x\n",__func__, wChargerAvail);
	return wChargerAvail;
}
#endif
/* End added by hailong.chen for defect 10130350 on 2020-11-23 */


/* Begin modified by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */
//in order to support qcom charger
#if (IS_ENABLED(CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT) \
	&& IS_ENABLED(CONFIG_CHARGER_TYPE_MT6357))
unsigned int disable_vsrc_for_pe2(void)
{
    unsigned int vsrc_value = 0;

/* Begin add by jin.wang for task 2064 on 2022-4-1 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	bc11_set_register_value(chr_info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x2);
#endif
/* End add by jin.wang */

    vsrc_value = bc11_get_register_value(chr_info->regmap,
			PMIC_RG_BC11_VSRC_EN_ADDR,
		    PMIC_RG_BC11_VSRC_EN_MASK,
		    PMIC_RG_BC11_VSRC_EN_SHIFT);
    if(vsrc_value > 0){
       	/* RG_bc11_VSRC_EN[1:0]=00 */
	    bc11_set_register_value(chr_info->regmap,
		    PMIC_RG_BC11_VSRC_EN_ADDR,
		    PMIC_RG_BC11_VSRC_EN_MASK,
		    PMIC_RG_BC11_VSRC_EN_SHIFT,
		    0);
		//Charger_Detect_Release_Pulldown(); //see mtk-case:ALPS06213981
        msleep(50);
    }
    return vsrc_value;
}
#endif
/* End modified by dapeng.qiao task 10672631 porting for prague on 2021.1.13 */

static unsigned int hw_bc11_DCD(struct mtk_charger_type *info)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_IPU_EN[1.0] = 10 */
// Begin modified by zhangkun for MODEL3-3674 on 2022-11-10
#ifdef CONFIG_TCT_PROJECT_MODEL_3
        msleep(50);
#endif
// End modified by zhangkun for MODEL3-3674 on 2022-11-10
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPU_EN_ADDR,
		PMIC_RG_BC11_IPU_EN_MASK,
		PMIC_RG_BC11_IPU_EN_SHIFT,
		0x2);
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x1);
	/* RG_bc11_VREF_VTH = [1:0]=01 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0x1);
	/* RG_bc11_CMP_EN[1.0] = 10 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x2);
#if IS_ENABLED(CONFIG_TCT_CHARGER)
    msleep(95);
#else
    msleep(80);
#endif
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(info->regmap,
		PMIC_RGS_BC11_CMP_OUT_ADDR,
		PMIC_RGS_BC11_CMP_OUT_MASK,
		PMIC_RGS_BC11_CMP_OUT_SHIFT);

	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPU_EN_ADDR,
		PMIC_RG_BC11_IPU_EN_MASK,
		PMIC_RG_BC11_IPU_EN_SHIFT,
		0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0x0);
/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pr_err("%s: wChargerAvail=0x%x\n",__func__, wChargerAvail);
#endif
/* End add by jin.wang */
	return wChargerAvail;
}

static unsigned int hw_bc11_stepA2(struct mtk_charger_type *info)
{
	unsigned int wChargerAvail = 0;
// Begin modified by zhangkun for MODEL3-3674 on 2022-11-10
#ifdef CONFIG_TCT_PROJECT_MODEL_3
        msleep(80);
#endif
// End modified by zhangkun for MODEL3-3674 on 2022-11-10
	/* RG_bc11_VSRC_EN[1.0] = 10 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0x2);
	/* RG_bc11_IPD_EN[1:0] = 01 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x1);
#if IS_ENABLED(CONFIG_TCT_CHARGER)
    msleep(90);
#else
    msleep(80);
#endif
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(info->regmap,
					PMIC_RGS_BC11_CMP_OUT_ADDR,
					PMIC_RGS_BC11_CMP_OUT_MASK,
					PMIC_RGS_BC11_CMP_OUT_SHIFT);

	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x0);
/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pr_err("%s: wChargerAvail=0x%x\n",__func__, wChargerAvail);
#endif
/* End add by jin.wang */
	return wChargerAvail;
}

static unsigned int hw_bc11_stepB2(struct mtk_charger_type *info)
{
	unsigned int wChargerAvail = 0;
// Begin modified by zhangkun for MODEL3-3674 on 2022-11-10
#ifdef CONFIG_TCT_PROJECT_MODEL_3
        msleep(80);
#endif
// End modified by zhangkun for MODEL3-3674 on 2022-11-10
	/*enable the voltage source to DM*/
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0x1);
	/* enable the pull-down current to DP */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x2);
	/* VREF threshold voltage for comparator  =0.325V */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0x0);
	/* enable the comparator to DP */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x2);
#if IS_ENABLED(CONFIG_TCT_CHARGER)
    msleep(90);
#else
    msleep(80);
#endif
	wChargerAvail = bc11_get_register_value(info->regmap,
		PMIC_RGS_BC11_CMP_OUT_ADDR,
		PMIC_RGS_BC11_CMP_OUT_MASK,
		PMIC_RGS_BC11_CMP_OUT_SHIFT);
	/*reset to default value*/
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0x0);
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x0);
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x0);
	if (wChargerAvail == 1) {
		bc11_set_register_value(info->regmap,
			PMIC_RG_BC11_VSRC_EN_ADDR,
			PMIC_RG_BC11_VSRC_EN_MASK,
			PMIC_RG_BC11_VSRC_EN_SHIFT,
			0x2);
		pr_info("charger type: DCP, keep DM voltage source in stepB2\n");
	}
/* Begin add by jin.wang task 2064 on 2021.11.2 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pr_err("%s: wChargerAvail=0x%x\n",__func__, wChargerAvail);
#endif
/* End add by jin.wang */
	return wChargerAvail;

}

static void hw_bc11_done(struct mtk_charger_type *info)
{
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VSRC_EN_ADDR,
		PMIC_RG_BC11_VSRC_EN_MASK,
		PMIC_RG_BC11_VSRC_EN_SHIFT,
		0x0);
	/* RG_bc11_VREF_VTH = [1:0]=0 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_VREF_VTH_ADDR,
		PMIC_RG_BC11_VREF_VTH_MASK,
		PMIC_RG_BC11_VREF_VTH_SHIFT,
		0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_CMP_EN_ADDR,
		PMIC_RG_BC11_CMP_EN_MASK,
		PMIC_RG_BC11_CMP_EN_SHIFT,
		0x0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPU_EN_ADDR,
		PMIC_RG_BC11_IPU_EN_MASK,
		PMIC_RG_BC11_IPU_EN_SHIFT,
		0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_IPD_EN_ADDR,
		PMIC_RG_BC11_IPD_EN_MASK,
		PMIC_RG_BC11_IPD_EN_SHIFT,
		0x0);
	/* RG_bc11_BIAS_EN=0 */
	bc11_set_register_value(info->regmap,
		PMIC_RG_BC11_BIAS_EN_ADDR,
		PMIC_RG_BC11_BIAS_EN_MASK,
		PMIC_RG_BC11_BIAS_EN_SHIFT,
		0x0);

#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
	Charger_Detect_Release();
#endif
}

static void dump_charger_name(int type)
{
	switch (type) {
	case POWER_SUPPLY_TYPE_UNKNOWN:
		pr_info("charger type: %d, CHARGER_UNKNOWN\n", type);
		break;
	case POWER_SUPPLY_TYPE_USB:
		pr_info("charger type: %d, Standard USB Host\n", type);
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		pr_info("charger type: %d, Charging USB Host\n", type);
		break;
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		pr_info("charger type: %d, Non-standard Charger\n", type);
		break;
#else
#ifdef FIXME
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		pr_info("charger type: %d, Non-standard Charger\n", type);
		break;
#endif
#endif
	case POWER_SUPPLY_TYPE_USB_DCP:
		pr_info("charger type: %d, Standard Charger\n", type);
		break;
	default:
		pr_info("charger type: %d, Not Defined!!!\n", type);
		break;
	}
}

static int get_charger_type(struct mtk_charger_type *info)
{
	enum power_supply_usb_type type;
// Begin modified by zhangkun for MODEL3-2523 on 2022-10-14
/* [BSP]Begin modified by bitao.xiong for SNTTF-635 on 2022/10/29 */
#if IS_ENABLED(CONFIG_TCT_PROJECT_MODEL_3) || IS_ENABLED(CONFIG_TCT_CHARGER)
    if(mutex_is_locked(&info->ops_lock) == 1){
        pr_notice("%s: get_charger_type ongoing\n\n", __func__);
    }
    mutex_lock(&info->ops_lock);
#endif
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if ((info->charge_type != POWER_SUPPLY_TYPE_UNKNOWN)
		&& (info->charge_type != POWER_SUPPLY_TYPE_USB_FLOAT)) {
        mutex_unlock(&info->ops_lock);
        pr_notice("%s:get_charger_type ignore the second detection(charge_type=%d)\n", __func__,info->charge_type);
        return info->charge_type;
    }
#endif
/* [BSP]End modified by bitao.xiong for SNTTF-635 on 2022/10/29 */
// End modified by zhangkun for MODEL3-2523 on 2022-10-14
	hw_bc11_init(info);
	if (hw_bc11_DCD(info)) {
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		/* Begin modified by bitao.xiong for task-10075354 on 2020-10-21 */
		if (hw_bc11_stepA1(info)) {
			info->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			type = POWER_SUPPLY_USB_TYPE_DCP;
/* Begin mod by jin.wang for androidT on 2022.4.18 */
//#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
			info->charge_type = POWER_SUPPLY_TYPE_USB_DCP;
//#endif
/* End mod by jin.wang */
		} else {
#if IS_ENABLED(CONFIG_TCT_PROJECT_SONATA)
			if (hw_bc11_stepA1(info)) {
				info->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
				type = POWER_SUPPLY_USB_TYPE_DCP;
				info->charge_type = POWER_SUPPLY_TYPE_USB_DCP;
			}else{
			    info->psy_desc.type = POWER_SUPPLY_TYPE_USB_FLOAT;
			    type = POWER_SUPPLY_USB_TYPE_DCP;
			    info->charge_type = POWER_SUPPLY_TYPE_USB_FLOAT;
			}
#else
/* Begin mod by jin.wang for androidT on 2022.4.18 */
			info->psy_desc.type = POWER_SUPPLY_TYPE_USB_FLOAT;
/* End mod by jin.wang */
			type = POWER_SUPPLY_USB_TYPE_DCP;
/* Begin mod by jin.wang for androidT on 2022.4.8 */
//#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
			info->charge_type = POWER_SUPPLY_TYPE_USB_FLOAT;
//#endif
/* End mod by jin.wang */
#endif//CONFIG_TCT_PROJECT_SONATA
		}
		/* End modified by bitao.xiong for task-10075354 on 2020-10-21 */
#else
		info->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		type = POWER_SUPPLY_USB_TYPE_DCP;
#endif
	} else {
		if (hw_bc11_stepA2(info)) {
			if (hw_bc11_stepB2(info)) {
				info->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
				type = POWER_SUPPLY_USB_TYPE_DCP;

/* Begin mod by jin.wang for androidT on 2022.4.8 */
//#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
#if IS_ENABLED(CONFIG_TCT_CHARGER)
				info->charge_type = POWER_SUPPLY_TYPE_USB_DCP;
#endif
//#endif
/* End mod by jin.wang */
			} else {
				info->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
				type = POWER_SUPPLY_USB_TYPE_CDP;

/* Begin mod by jin.wang for androidT on 2022.4.8 */
//#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
#if IS_ENABLED(CONFIG_TCT_CHARGER)
				info->charge_type = POWER_SUPPLY_TYPE_USB_CDP;
#endif
//#endif
/* End mod by jin.wang */
			}
		} else {
			info->psy_desc.type = POWER_SUPPLY_TYPE_USB;
			type = POWER_SUPPLY_USB_TYPE_SDP;

/* Begin mod by jin.wang for androidT on 2022.4.8 */
//#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
#if IS_ENABLED(CONFIG_TCT_CHARGER)
			info->charge_type = POWER_SUPPLY_TYPE_USB;
#endif
//#endif
/* End mod by jin.wang */
		}
	}

	if (type != POWER_SUPPLY_USB_TYPE_DCP)
		hw_bc11_done(info);
	else
		pr_info("charger type: skip bc11 release for BC12 DCP SPEC\n");

	pr_info("mt6357_get_charge_type: charge_type=%d\n", info->charge_type);
	//dump_charger_name(info->psy_desc.type);
// Begin modified by zhangkun for MODEL3-2523 on 2022-10-14
/* [BSP]Begin modified by bitao.xiong for SNTTF-635 on 2022/10/29 */
#if IS_ENABLED(CONFIG_TCT_PROJECT_MODEL_3) || IS_ENABLED(CONFIG_TCT_CHARGER)
	mutex_unlock(&info->ops_lock);
#endif
/* [BSP]End modified by bitao.xiong for SNTTF-635 on 2022/10/29 */
// End modified by zhangkun for MODEL3-2523 on 2022-10-14
	return type;
}

static int get_vbus_voltage(struct mtk_charger_type *info,
	int *val)
{
	int ret;

	if (!IS_ERR(info->chan_vbus)) {
		ret = iio_read_channel_processed(info->chan_vbus, val);
		if (ret < 0)
			pr_notice("[%s]read fail,ret=%d\n", __func__, ret);
	} else {
		pr_notice("[%s]chan error %d\n", __func__, info->chan_vbus);
		ret = -ENOTSUPP;
	}

	*val = (((R_CHARGER_1 +
			R_CHARGER_2) * 100 * *val) /
			R_CHARGER_2) / 100;

	return ret;
}


void do_charger_detect(struct mtk_charger_type *info, bool en)
{
	union power_supply_propval prop_online, prop_type, prop_usb_type;
	int ret = 0;

#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if (!mt_usb_is_device()) {
		pr_info("charger type: UNKNOWN, Now is usb host mode. Skip detection\n");
		return;
	}
#else
#ifndef CONFIG_TCPC_CLASS
	if (!mt_usb_is_device()) {
		pr_info("charger type: UNKNOWN, Now is usb host mode. Skip detection\n");
		return;
	}
#endif
#endif

	prop_online.intval = en;
	if (en) {
		ret = power_supply_set_property(info->psy,
				POWER_SUPPLY_PROP_ONLINE, &prop_online);
		ret = power_supply_get_property(info->psy,
				POWER_SUPPLY_PROP_TYPE, &prop_type);
		ret = power_supply_get_property(info->psy,
				POWER_SUPPLY_PROP_USB_TYPE, &prop_usb_type);
		pr_notice("type:%d usb_type:%d\n", prop_type.intval, prop_usb_type.intval);
	} else {
		info->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		info->type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		pr_notice("%s type:0 usb_type:0\n", __func__);

/* Begin mod by jin.wang for androidT on 2022.4.8 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
		info->charge_type = POWER_SUPPLY_TYPE_UNKNOWN;
		info->real_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		info->redet_cnt = 0;
#endif
/* End mod by jin.wang */
	}

#if IS_ENABLED(CONFIG_TCT_CHARGER)
/*Begin: add by wanling.chen for LUNA84GVZW-5839 on 2023.04.16*/
	pr_notice("%s en=%d redet_cnt=%d, charge_type=%d\n", __func__, en, info->redet_cnt, info->charge_type);
	if (info->charge_type == POWER_SUPPLY_TYPE_USB_FLOAT
	&& info->redet_cnt == 0) {
		info->redet_cnt++;
		info->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		info->charge_type = POWER_SUPPLY_TYPE_UNKNOWN;
		/*Begin: modify by wanglin.chen for RIOTMO-4351 on 2023.06.21*/
		cancel_delayed_work(&chr_info->chr_work);
		/*End: modify by wanglin.chen for RIOTMO-4351 on 2023.06.21*/
		schedule_delayed_work(&info->chr_work, msecs_to_jiffies(1000));
	}
	if (info->charge_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		info->real_charger_type = info->charge_type;
	}
/*End: add by wanling.chen for LUNA84GVZW-5839 on 2023.04.16*/
#endif

	power_supply_changed(info->psy);
}

static void do_charger_detection_work(struct work_struct *data)
{
#if IS_ENABLED(CONFIG_TCT_CHARGER)
/* Begin modified by bitao.xiong for defect-10053764 on 2020-10-22 */
	struct mtk_charger_type *info = (struct mtk_charger_type *)container_of(
				     to_delayed_work(data), struct mtk_charger_type, chr_work);
/* End modified by bitao.xiong for defect-10053764 on 2020-10-22 */
#else
	struct mtk_charger_type *info = (struct mtk_charger_type *)container_of(
					data, struct mtk_charger_type, chr_work);

#endif
	unsigned int chrdet = 0;

	chrdet = bc11_get_register_value(info->regmap,
		PMIC_RGS_CHRDET_ADDR,
		PMIC_RGS_CHRDET_MASK,
		PMIC_RGS_CHRDET_SHIFT);

	pr_notice("%s: chrdet:%d\n", __func__, chrdet);
	//if (chrdet) /* Deleted by bin.song.hz for task 10431118 on 2020-12-07 */
		do_charger_detect(info, chrdet);
	if (!chrdet) {
		hw_bc11_done(info);
		/* 8 = KERNEL_POWER_OFF_CHARGING_BOOT */
		/* 9 = LOW_POWER_OFF_CHARGING_BOOT */
		if (info->bootmode == 8 || info->bootmode == 9) {
			pr_info("%s: Unplug Charger/USB\n", __func__);

#ifndef CONFIG_TCPC_CLASS
			pr_info("%s: system_state=%d\n", __func__,
				system_state);
			if (system_state != SYSTEM_POWER_OFF)
				kernel_power_off();
#endif
		}
	}
}

/* Begin mod by jin.wang task 2064 on 2021.11.30 */
#if IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
void tct_start_det_charger(int delay_ms)
{
	pr_notice("%s: start chg type redet %d.\n", __func__, delay_ms);
#if 0
	cancel_delayed_work_sync(&chr_info->chr_work);
#endif
	schedule_delayed_work(&chr_info->chr_work,
			msecs_to_jiffies(delay_ms));
}

void tct_stop_det_charger(void)
{
	pr_notice("%s: cancel chg type redet.\n", __func__);
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	if(chr_info->redet_cnt == 0)
#endif
    {
		cancel_delayed_work(&chr_info->chr_work);
		//cancel_delayed_work_sync(&chr_info->chr_work);
	}
}
#else
/* Begin added by bin.song.hz for task 10431118 on 2020-12-07 */
void tct_detect_charger(void)
{
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	schedule_delayed_work(&chr_info->chr_work, 0);
#else
	schedule_work(&chr_info->chr_work);
#endif
}
/* End added by bin.song.hz for task 10431118 on 2020-12-07 */
#endif
/* End mod by jin.wang */

irqreturn_t chrdet_int_handler(int irq, void *data)
{
	struct mtk_charger_type *info = data;
	unsigned int chrdet = 0;

	chrdet = bc11_get_register_value(info->regmap,
		PMIC_RGS_CHRDET_ADDR,
		PMIC_RGS_CHRDET_MASK,
		PMIC_RGS_CHRDET_SHIFT);
	if (!chrdet) {
		hw_bc11_done(info);
		/* 8 = KERNEL_POWER_OFF_CHARGING_BOOT */
		/* 9 = LOW_POWER_OFF_CHARGING_BOOT */
		if (info->bootmode == 8 || info->bootmode == 9) {
			pr_info("%s: Unplug Charger/USB\n", __func__);

#ifndef CONFIG_TCPC_CLASS
			pr_info("%s: system_state=%d\n", __func__,
				system_state);
			if (system_state != SYSTEM_POWER_OFF)
				kernel_power_off();
#endif
		}
	}
	pr_notice("%s: chrdet:%d\n", __func__, chrdet);
	do_charger_detect(info, chrdet);

	return IRQ_HANDLED;
}


static int psy_chr_type_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mtk_charger_type *info;
	int vbus = 0;

/* Begin del by jin.wang for jira 2064 on 2021-11-1 */
#if !IS_ENABLED(CONFIG_TCT_NB_CHG_PATCH)
	pr_notice("%s: prop:%d\n", __func__, psp);
#endif
/* End del by jin.wang */

	info = (struct mtk_charger_type *)power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (info->type == POWER_SUPPLY_USB_TYPE_UNKNOWN)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = info->psy_desc.type;
		break;
/* Begin added by hailong.chen for defect 10130350 on 2020-11-23 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = info->real_charger_type;
		break;
#endif
/* End added by hailong.chen for defect 10130350 on 2020-11-23 */
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = info->type;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		get_vbus_voltage(info, &vbus);
		val->intval = vbus;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int psy_chr_type_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	struct mtk_charger_type *info;

	pr_notice("%s: prop:%d %d\n", __func__, psp, val->intval);

	info = (struct mtk_charger_type *)power_supply_get_drvdata(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		info->type = get_charger_type(info);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Begin mod by jin.wang for androidT on 2022-4-18 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
bool flag_support_fast_charging; //added by dapeng.qiao for task 11028765 on 2021-04-19
#endif
/* End mod by jin.wang */

static int mt_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mtk_charger_type *info;

	info = (struct mtk_charger_type *)power_supply_get_drvdata(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		/* Force to 1 in all charger type */
		if (info->type != POWER_SUPPLY_USB_TYPE_UNKNOWN)
			val->intval = 1;
		/* Reset to 0 if charger type is USB */
		if ((info->type == POWER_SUPPLY_USB_TYPE_SDP) ||
			(info->type == POWER_SUPPLY_USB_TYPE_CDP))
			val->intval = 0;
		break;

/* Begin added by dapeng.qiao & jin.wang for task 11028765 on 2021-08-30 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (info->psy_desc.type == POWER_SUPPLY_TYPE_USB_DCP) {
			if (flag_support_fast_charging)
				val->intval = 2000000;  // popup "Fast charging"
			else
				val->intval = 1200000;  // popup "charging"
		} else {
			val->intval = 500000;  // popup "slow charging"
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (flag_support_fast_charging &&
			(info->psy_desc.type == POWER_SUPPLY_TYPE_USB_DCP))
			val->intval = 9000000;
		else
			val->intval = 5000000;
		break;
	case POWER_SUPPLY_PROP_HV_FLAG:
		val->intval = flag_support_fast_charging;
		break;
#endif
/* End added by dapeng.qiao & jin.wang for task 11028765 on 2021-08-30 */

	default:
		return -EINVAL;
	}

	return 0;
}

/* Begin added by jin.wang for jira 2064 on 2021-10-25 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
static int mt_ac_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	pr_notice("%s: prop:%d %d\n", __func__, psp, val->intval);
	switch (psp) {
	case POWER_SUPPLY_PROP_HV_FLAG:
		flag_support_fast_charging = (bool)(val->intval);
		/* [BSP]Begin added by bitao.xiong for SNTBBH-4690 on 2023/01/10 */
		power_supply_changed(psy);
		/* [BSP]End added by bitao.xiong for SNTBBH-4690 on 2023/01/10 */
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mt_ac_property_is_writeable(struct power_supply *psy,
					       enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_HV_FLAG:
		return 1;
	default:
		return 0;
	}
}
#endif
/* End added by jin.wang */

static int mt_usb_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mtk_charger_type *info;

	info = (struct mtk_charger_type *)power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if ((info->type == POWER_SUPPLY_USB_TYPE_SDP) ||
			(info->type == POWER_SUPPLY_USB_TYPE_CDP))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
/* Begin modified by dapeng.qiao & jin.wang for task 11028765 on 2021-08-30 */
		if (info->type == POWER_SUPPLY_USB_TYPE_CDP)
			val->intval = 1200000;  // popup "charging"
		else
			val->intval = 500000;  // popup "slow charging"
/* End modified by dapeng.qiao & jin.wang for task 11028765 on 2021-08-30 */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = 5000000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int psy_charger_type_property_is_writeable(struct power_supply *psy,
					       enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_usb_type mt6357_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
#if IS_ENABLED(CONFIG_TCT_CHARGER)
/* Begin added by bitao.xiong for task-10075354 on 2020-10-21 */
	POWER_SUPPLY_USB_TYPE_FLOAT,
/* End added by bitao.xiong for task-10075354 on 2020-10-21 */
#endif
};

static char *mt6357_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

static int check_boot_mode(struct mtk_charger_type *info, struct device *dev)
{
	struct device_node *boot_node = NULL;
	struct tag_bootmode *tag = NULL;

	boot_node = of_parse_phandle(dev->of_node, "bootmode", 0);
	if (!boot_node)
		pr_notice("%s: failed to get boot mode phandle\n", __func__);
	else {
		tag = (struct tag_bootmode *)of_get_property(boot_node,
							"atag,boot", NULL);
		if (!tag)
			pr_notice("%s: failed to get atag,boot\n", __func__);
		else {
			pr_notice("%s: size:0x%x tag:0x%x bootmode:0x%x boottype:0x%x\n",
				__func__, tag->size, tag->tag,
				tag->bootmode, tag->boottype);
			info->bootmode = tag->bootmode;
			info->boottype = tag->boottype;
		}
	}
	return 0;
}

static int mt6357_charger_type_probe(struct platform_device *pdev)
{
	struct mtk_charger_type *info;
	struct iio_channel *chan_vbus;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret = 0;

	pr_notice("%s: starts\n", __func__);

	chan_vbus = devm_iio_channel_get(
		&pdev->dev, "pmic_vbus");
	if (IS_ERR(chan_vbus)) {
		pr_notice("mt6357 charger type requests probe deferral ret:%d\n",
			chan_vbus);
		return -EPROBE_DEFER;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info),
		GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->chip = (struct mt6397_chip *)dev_get_drvdata(
		pdev->dev.parent);
	info->regmap = info->chip->regmap;

	dev_set_drvdata(&pdev->dev, info);
	info->pdev = pdev;
	mutex_init(&info->ops_lock);

	check_boot_mode(info, &pdev->dev);

	info->psy_desc.name = "mtk_charger_type";
	info->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_desc.properties = chr_type_properties;
	info->psy_desc.num_properties = ARRAY_SIZE(chr_type_properties);
	info->psy_desc.get_property = psy_chr_type_get_property;
	info->psy_desc.set_property = psy_chr_type_set_property;
	info->psy_desc.property_is_writeable =
			psy_charger_type_property_is_writeable;
	info->psy_desc.usb_types = mt6357_charger_usb_types,
	info->psy_desc.num_usb_types = ARRAY_SIZE(mt6357_charger_usb_types),
	info->psy_cfg.drv_data = info;

	info->psy_cfg.of_node = np;
	info->psy_cfg.supplied_to = mt6357_charger_supplied_to;
	info->psy_cfg.num_supplicants = ARRAY_SIZE(mt6357_charger_supplied_to);

	info->ac_desc.name = "ac";
	info->ac_desc.type = POWER_SUPPLY_TYPE_MAINS;
	info->ac_desc.properties = mt_ac_properties;
	info->ac_desc.num_properties = ARRAY_SIZE(mt_ac_properties);
	info->ac_desc.get_property = mt_ac_get_property;
/* Begin add by jin.wang for jira 2064 on 2021-10-25 */
#if IS_ENABLED(CONFIG_TCT_CHARGER)
	info->ac_desc.set_property = mt_ac_set_property;
	info->ac_desc.property_is_writeable = mt_ac_property_is_writeable;
#endif
/* End add by jin.wang */
	info->ac_cfg.drv_data = info;

	info->usb_desc.name = "usb";
	info->usb_desc.type = POWER_SUPPLY_TYPE_USB;
	info->usb_desc.properties = mt_usb_properties;
	info->usb_desc.num_properties = ARRAY_SIZE(mt_usb_properties);
	info->usb_desc.get_property = mt_usb_get_property;
	info->usb_cfg.drv_data = info;

	info->psy = power_supply_register(&pdev->dev, &info->psy_desc,
			&info->psy_cfg);

	if (IS_ERR(info->psy)) {
		pr_notice("%s Failed to register power supply: %ld\n",
			__func__, PTR_ERR(info->psy));
		return PTR_ERR(info->psy);
	}
	pr_notice("%s register psy success\n", __func__);

	info->chan_vbus = devm_iio_channel_get(
		&pdev->dev, "pmic_vbus");
	if (IS_ERR(info->chan_vbus)) {
		pr_notice("chan_vbus auxadc get fail, ret=%d\n",
			PTR_ERR(info->chan_vbus));
	}

	if (of_property_read_u32(np, "bc12_active", &info->bc12_active) < 0)
		pr_notice("%s: no bc12_active\n", __func__);

	pr_notice("%s: bc12_active:%d\n", __func__, info->bc12_active);

	if (info->bc12_active) {
		info->ac_psy = power_supply_register(&pdev->dev,
				&info->ac_desc, &info->ac_cfg);

		if (IS_ERR(info->ac_psy)) {
			pr_notice("%s Failed to register power supply: %ld\n",
				__func__, PTR_ERR(info->ac_psy));
			return PTR_ERR(info->ac_psy);
		}

		info->usb_psy = power_supply_register(&pdev->dev,
				&info->usb_desc, &info->usb_cfg);

		if (IS_ERR(info->usb_psy)) {
			pr_notice("%s Failed to register power supply: %ld\n",
				__func__, PTR_ERR(info->usb_psy));
			return PTR_ERR(info->usb_psy);
		}

#if IS_ENABLED(CONFIG_TCT_CHARGER)
/* Begin modified by bitao.xiong for defect-10053764 on 2020-10-22 */
		INIT_DELAYED_WORK(&info->chr_work, do_charger_detection_work);
		schedule_delayed_work(&info->chr_work, 0);
/* End modified by bitao.xiong for defect-10053764 on 2020-10-22 */
#else
		INIT_WORK(&info->chr_work, do_charger_detection_work);
		schedule_work(&info->chr_work);
#endif
// Begin modified by zhangkun for MODEL3-1888 on 2022-09-19
// Begin modified by zhangkun for MODEL3-2453 on 2022-10-18
//#ifndef CONFIG_TCT_PROJECT_MODEL_3
// Begin modified by chengjie.mi for CIVICS-2419 on 2022-11-3
#if (!defined(CONFIG_TCT_PROJECT_CIVIC_S))
		ret = devm_request_threaded_irq(&pdev->dev,
			platform_get_irq_byname(pdev, "chrdet"), NULL,
			chrdet_int_handler, IRQF_TRIGGER_HIGH, "chrdet", info);
		if (ret < 0)
			pr_notice("%s request chrdet irq fail\n", __func__);
#endif
//#endif
// End modified by chengjie.mi for CIVICS-2419 on 2022-11-3
// End modified by zhangkun for MODEL3-2453 on 2022-10-18
// End modified by zhangkun for MODEL3-1888 on 2022-09-19
	}

	info->first_connect = true;

    /* Begin added by bin.song.hz for task 10431118 on 2020-12-07 */
    chr_info = info;
    /* End added by bin.song.hz for task 10431118 on 2020-12-07 */

	pr_notice("%s: done\n", __func__);

	return 0;
}

static const struct of_device_id mt6357_charger_type_of_match[] = {
	{.compatible = "mediatek,mt6357-charger-type",},
	{},
};

static int mt6357_charger_type_remove(struct platform_device *pdev)
{
	struct mtk_charger_type *info = platform_get_drvdata(pdev);

#if IS_ENABLED(CONFIG_TCT_CHARGER)
/* Begin modified by bitao.xiong for defect-10053764 on 2020-10-22 */
	if (info) {
		cancel_delayed_work_sync(&info->chr_work);
		devm_kfree(&pdev->dev, info);
	}
/* End modified by bitao.xiong for defect-10053764 on 2020-10-22 */
#else
	if (info)
		devm_kfree(&pdev->dev, info);
#endif
	return 0;
}

MODULE_DEVICE_TABLE(of, mt6357_charger_type_of_match);

static struct platform_driver mt6357_charger_type_driver = {
	.probe = mt6357_charger_type_probe,
	.remove = mt6357_charger_type_remove,
	//.shutdown = mt6357_charger_type_shutdown,
	.driver = {
		.name = "mt6357-charger-type-detection",
		.of_match_table = mt6357_charger_type_of_match,
		},
};

static int __init mt6357_charger_type_init(void)
{
	return platform_driver_register(&mt6357_charger_type_driver);
}
module_init(mt6357_charger_type_init);

static void __exit mt6357_charger_type_exit(void)
{
	platform_driver_unregister(&mt6357_charger_type_driver);
}
module_exit(mt6357_charger_type_exit);

MODULE_AUTHOR("wy.chuang <wy.chuang@mediatek.com>");
MODULE_DESCRIPTION("MTK Charger Type Detection Driver");
MODULE_LICENSE("GPL");

