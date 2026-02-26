// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
// begin modified by tct-hq/yzheng4 in 2022-12-09
#if defined(CONFIG_MTK_LUNA84GVZW_CAMERA)
    {LUNA84GVZW_GC05A2_SENSOR_ID, 0xA0, gc05a2_read_region},
/*Begin jiantaohuang for LUNA84GVZW-3892, otp porting for lunavzw hi846 on 20221215*/
	{LUNA84GVZW_HI846_SENSOR_ID, 0x20, hi846_new_read_region},
/*End jiantaohuang for LUNA84GVZW-3892, otp porting for lunavzw hi846 on 20221215*/
#endif
// end modified by tct-hq/yzheng4 in 2022-12-09
#if defined(TCT_CAMERA_PROJECT_CIVIC)
	{GDIR220061_HI1336_SENSOR_ID, 0xA0, Common_read_region, Common_write_region_gt24p64b},
        {SWLU7166_GC13A0_SENSOR_ID, 0xA0, Common_read_region, Common_write_region_p24c64f},
        {LHVM15798_GC05A2_SENSOR_ID, 0xA0, gc05a2_front_read_region},
#elif defined(TCT_CAMERA_PROJECT_CIVIC_PLUS)
        {GKQR220088_HI5021Q_SENSOR_ID, 0xA0, Common_read_region, Common_write_region_p24c128e, 0x4000},
        {GKQR220088_HI5021QMAC_SENSOR_ID, 0xA0, Common_read_region, NULL, 0x3000},
	{CTXWM18799_GC08A3_SENSOR_ID, 0xA0, ctxwm18799_gc08a3_read_region},
	{LHMBGH6651_GC08A3_SENSOR_ID, 0xA0, ctxwm18799_gc08a3_read_region},
#else
//End modified by chengyixuan for CIVICPL-3060 on 2022-07-16
	{TSPPHCF1318_HI1336_SENSOR_ID, 0xA0, Common_read_region, Common_write_region_p24c128e},
	/*Begin yaqin.zhang for sonatatf otp porting 2022/6/29*/
	{TSPPHCP2120_HI1336_SENSOR_ID, 0xA0, Common_read_region, Common_write_region_p24c128e},
	{SHNCND42B_S5K3L6_SENSOR_ID, 0xA0, Common_read_region, Common_write_region_gt24p64b},
	/*End yaqin.zhang for sonatatf otp porting 2022/6/29*/
	/*Begin ersen.shang for [Task][ 11425156][cruze/cruze pro camera bring up] 202108*/
	{SHNBA815M_GC08A3_SENSOR_ID, 0xA0, gc08a3_read_region},
	{TSPPSNP1082_S5KJN1_SENSOR_ID, 0xA0, Common_read_region, Common_write_region_p24c128e},
	{TSPPSNP1269_S5KJN1_SENSOR_ID, 0xA0, Common_read_region, Common_write_region_p24c128e},
	{TSPPHCP1088_HI1336_SENSOR_ID, 0xA0, Common_read_region, Common_write_region_gt24p64b},
	/*End   ersen.shang for [Task][ 11425156][cruze/cruze pro camera bring up] 202108*/
	/*Begin zihao.li for [Task][11621559] AustinTF add s5kjn1 otp config & dual camera calibration on 20211027*/
	{TSPPSNP1269_S5KJN1_SENSOR_ID, 0xA0, Common_read_region, Common_write_region_p24c128e},
	/*End   zihao.li for [Task][11621559] AustinTF add s5kjn1 otp config & dual camera calibration on 20211027*/
	/*Begin majinrui for [Task: 11653505][austintf camera otp config] 20211028*/
	{TSPSCF0002_S5K3L6_SENSOR_ID , 0xA2, Common_read_region},
	{SHNBMD28B_OV13B10_SENSOR_ID , 0xA2, Common_read_region},
	/*End   majinrui for [Task: 11653505][austintf camera otp config] 20211028*/
	{TSPPH8A1408_HI846_SENSOR_ID, 0xA0, hi846_read_region},
	{TCL0003BA_GC08A3_SENSOR_ID, 0xA0, gc08a3_read_region},
	{IMX230_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX338_SENSOR_ID, 0xA0, Common_read_region},
	{S5K4E6_SENSOR_ID, 0xA8, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3M3_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX318_SENSOR_ID, 0xA0, Common_read_region},
	{OV8858_SENSOR_ID, 0xA8, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	/*B+B*/
	{S5K2P7_SENSOR_ID, 0xA0, Common_read_region},
	{OV8856_SENSOR_ID, 0xA0, Common_read_region},
	/*61*/
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3L8_SENSOR_ID, 0xA0, Common_read_region},
	{S5K5E8YX_SENSOR_ID, 0xA2, Common_read_region},
	/*99*/
	{IMX258_SENSOR_ID, 0xA0, Common_read_region},
	{IMX258_MONO_SENSOR_ID, 0xA0, Common_read_region},
	/*97*/
	{OV23850_SENSOR_ID, 0xA0, Common_read_region},
	{OV23850_SENSOR_ID, 0xA8, Common_read_region},
	{S5K3M2_SENSOR_ID, 0xA0, Common_read_region},
	/*55*/
	{S5K2P8_SENSOR_ID, 0xA2, Common_read_region},
	{S5K2P8_SENSOR_ID, 0xA0, Common_read_region},
	{OV8858_SENSOR_ID, 0xA2, Common_read_region},
	/* Others */
	{S5K2X8_SENSOR_ID, 0xA0, Common_read_region},
	{IMX377_SENSOR_ID, 0xA0, Common_read_region},
	{IMX214_SENSOR_ID, 0xA0, Common_read_region},
	{IMX214_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{IMX486_SENSOR_ID, 0xA8, Common_read_region},
	{OV12A10_SENSOR_ID, 0xA8, Common_read_region},
	{OV13855_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3L8_SENSOR_ID, 0xA0, Common_read_region},
	{HI556_SENSOR_ID, 0x51, Common_read_region},
	{S5K5E8YX_SENSOR_ID, 0x5a, Common_read_region},
	{S5K5E8YXREAR2_SENSOR_ID, 0x5a, Common_read_region},
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
//Begin modified by chengyixuan for CIVICPL-3060 on 2022-07-16
#endif
//End modified by chengyixuan for CIVICPL-3060 on 2022-07-16
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


