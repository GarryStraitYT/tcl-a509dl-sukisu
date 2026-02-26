// SPDX-License-Identifier: GPL-2.0

#include "kd_imgsensor.h"

#include "regulator/regulator.h"
#include "gpio/gpio.h"
/*#include "mt6306/mt6306.h"*/
#include "mclk/mclk.h"



#include "imgsensor_cfg_table.h"

enum IMGSENSOR_RETURN
	(*hw_open[IMGSENSOR_HW_ID_MAX_NUM])(struct IMGSENSOR_HW_DEVICE **) = {
	imgsensor_hw_regulator_open,
	imgsensor_hw_gpio_open,
	/*imgsensor_hw_mt6306_open,*/
	imgsensor_hw_mclk_open
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config[] = {
	{
		IMGSENSOR_SENSOR_IDX_MAIN,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_ID_MCLK, IMGSENSOR_HW_PIN_MCLK},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_AVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DOVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_PDN},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_RST},
			{IMGSENSOR_HW_ID_NONE, IMGSENSOR_HW_PIN_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB,
		IMGSENSOR_I2C_DEV_1,
		{
			{IMGSENSOR_HW_ID_MCLK, IMGSENSOR_HW_PIN_MCLK},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_AVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DOVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_PDN},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_RST},
			{IMGSENSOR_HW_ID_NONE, IMGSENSOR_HW_PIN_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN2,
		IMGSENSOR_I2C_DEV_2,
		{
			{IMGSENSOR_HW_ID_MCLK, IMGSENSOR_HW_PIN_MCLK},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_AVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DOVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_DVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_PDN},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_RST},
			{IMGSENSOR_HW_ID_NONE, IMGSENSOR_HW_PIN_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB2,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_ID_MCLK, IMGSENSOR_HW_PIN_MCLK},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_AVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DOVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_DVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_PDN},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_RST},
			{IMGSENSOR_HW_ID_NONE, IMGSENSOR_HW_PIN_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN3,
		IMGSENSOR_I2C_DEV_2,
		{
			{IMGSENSOR_HW_ID_MCLK, IMGSENSOR_HW_PIN_MCLK},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_AVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DOVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_PDN},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_RST},
			{IMGSENSOR_HW_ID_NONE, IMGSENSOR_HW_PIN_NONE},
		},
	},

	{IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ platform_power_sequence[] = {
#ifdef MIPI_SWITCH
	{
		PLATFORM_POWER_SEQ_NAME,
		{
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_EN,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
				0
			},
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
				IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0
			},
		},
		IMGSENSOR_SENSOR_IDX_SUB,
	},
	{
		PLATFORM_POWER_SEQ_NAME,
		{
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_EN,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
				0
			},
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0
			},
		},
		IMGSENSOR_SENSOR_IDX_MAIN2,
	},
#endif

	{NULL}
};

/* Legacy design */
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence[] = {
/*Begin 20220112 junyiliu add for rapidtf camera porting*/
#if defined(CXTVM15502_GC5035_MIPI_RAW)
	{
		SENSOR_DRVNAME_CXTVM15502_GC5035_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{AFVDD, Vol_2800, 1},
			{RST, Vol_High, 2},
			{PDN, Vol_High, 1},

		},
	},
#endif
#if defined(CXTVC12301_GC02M1_MIPI_RAW)
	{
		SENSOR_DRVNAME_CXTVC12301_GC02M1_MIPI_RAW,
		{{SensorMCLK, Vol_High, 0},
		{PDN, Vol_Low, 5},
		{RST, Vol_Low, 5},
		{DOVDD, Vol_1800, 5},
		{DVDD, Vol_1800, 5},
		{AVDD, Vol_2800, 5},
		{PDN, Vol_High, 5},
		{RST, Vol_High, 5}},
	},

#endif
/*End 20220112 junyiliu add for rapidtf camera porting*/
/*Begin 20211102 junyiliu add for Jetta_VZW camera porting*/
#if defined(SHNBF821B_GC08A3_MIPI_RAW)
	{
		SENSOR_DRVNAME_SHNBF821B_GC08A3_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 1},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1200, 0},
			{AVDD, Vol_2800, 1},
			//{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 1},
			{RST, Vol_High, 1},
		}
	},
#endif
#if defined(SHNBAD17B_OV13B10_MIPI_RAW)
	{
		SENSOR_DRVNAME_SHNBAD17B_OV13B10_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 0},
			{RST, Vol_High, 5},
		}
	},
#endif
/*End 20211102 junyiliu add for Jetta_VZW camera porting*/

/*Begin 20210805 liuhui add for cruze camera*/
#if defined(TSPPHCP1088_HI1336_MIPI_RAW)
	{
		SENSOR_DRVNAME_TSPPHCP1088_HI1336_MIPI_RAW,
		{
			{PDN,   Vol_Low,  0},
			{RST,   Vol_Low,  1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD,  Vol_1100, 2},
			{SensorMCLK, Vol_High, 2},
			{PDN,   Vol_High, 2},
			{RST,   Vol_High, 5},
		},
	},
#endif
#if defined(SHN9F511B_GC5035_MIPI_RAW)
	{
		SENSOR_DRVNAME_SHN9F511B_GC5035_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 2},
			{PDN, Vol_High, 2},
		},
	},
#endif
#if defined(EWYPC5319_HI556_MIPI_RAW)
		{
			SENSOR_DRVNAME_EWYPC5319_HI556_MIPI_RAW,
			{
				{PDN, Vol_Low, 1},
				{RST, Vol_Low, 1},
				{DOVDD, Vol_1800, 1},
				{AVDD, Vol_2800, 1},
				{DVDD, Vol_1200, 1},
				{SensorMCLK, Vol_High, 2},
				{PDN, Vol_High, 2},
				{RST, Vol_High, 2},
			},
		},
#endif
#if defined(SHNBA815M_GC08A3_MIPI_RAW)
	{
		SENSOR_DRVNAME_SHNBA815M_GC08A3_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 2},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(TSPPH8A1327_HI846_MIPI_RAW)
	{
		SENSOR_DRVNAME_TSPPH8A1327_HI846_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 2},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(SHNBF204C_GC02M1_MIPI_RAW)
	{
		SENSOR_DRVNAME_SHNBF204C_GC02M1_MIPI_RAW,
		{
			{PDN  , Vol_Low , 0},
			{SensorMCLK, Vol_High, 1},
			{DOVDD, Vol_1800, 2},
			{DVDD , Vol_1800, 1},
			{AVDD , Vol_2800, 3},
			{PDN  , Vol_High, 1},
		},
	},
#endif
#if defined(TSPPSNP1082_S5KJN1_MIPI_RAW)
	{
		SENSOR_DRVNAME_TSPPSNP1082_S5KJN1_MIPI_RAW,
		{
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1200, 0},
			{AVDD, Vol_2800, 0},
			{RST, Vol_High, 1},
			{SensorMCLK, Vol_High, 1},
			{AFVDD, Vol_2800, 2},
		},
	},
#endif
#if defined(TSPS8F9074_S5K4H7_MIPI_RAW)
	{
		SENSOR_DRVNAME_TSPS8F9074_S5K4H7_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(CXTVC12511_GC2905_MIPI_RAW)
	{
		SENSOR_DRVNAME_CXTVC12511_GC2905_MIPI_RAW,
		{
			{PDN,Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 3},
			{PDN,Vol_High, 1},
		},
	},
#endif
#if defined(CXTVC12499_GC02M1B_MIPI_RAW)
	{
		SENSOR_DRVNAME_CXTVC12499_GC02M1B_MIPI_RAW,
		{
			{PDN  , Vol_Low , 0},
			{SensorMCLK, Vol_High, 1},
			{DOVDD, Vol_1800, 2},
			{DVDD , Vol_1800, 1},
			{AVDD , Vol_2800, 3},
			{PDN  , Vol_High, 1},
		},
	},
#endif

/*End 20210805 liuhui add for cruze camera*/
// add by lihao.zhu for applo84gboost_refresh
#if defined(R_CXT_GC5035_MIPI_RAW)
    {
        SENSOR_DRVNAME_R_CXT_GC5035_MIPI_RAW,
        {
                {PDN, Vol_Low, 1, Vol_Low, 1},
                {RST, Vol_Low, 1, Vol_Low, 1},
                {DOVDD, Vol_1800, 1, Vol_Low, 1},
                {DVDD, Vol_1200, 1, Vol_Low, 1},
                {AVDD, Vol_2800, 1, Vol_Low, 1},
                {RST, Vol_High, 1, Vol_Low, 1},
                {PDN, Vol_High, 1, Vol_Low, 1},
                {SensorMCLK, Vol_High, 1, Vol_Low, 1},
        },
    },
#endif
#if defined(F_SHINETECH_GC5035_MIPI_RAW)
	{
		SENSOR_DRVNAME_F_SHINETECH_GC5035_MIPI_RAW,
		{
			{PDN, Vol_Low, 5, Vol_Low, 1},
			{RST, Vol_Low, 5, Vol_Low, 1},
			{DOVDD, Vol_1800, 5, Vol_Low, 1},
			{DVDD, Vol_1200, 5, Vol_Low, 1},
			{AVDD, Vol_2800, 5, Vol_Low, 1},
			{RST, Vol_High, 5, Vol_Low, 1},
			{PDN, Vol_High, 5, Vol_Low, 1},
			{SensorMCLK, Vol_High, 5, Vol_Low, 1},
		},
	},
#endif
#if defined(R_SHINETECH_GC5035_MIPI_RAW)
	{
		SENSOR_DRVNAME_R_SHINETECH_GC5035_MIPI_RAW,
		{
			{PDN, Vol_Low, 5, Vol_Low, 1},
			{RST, Vol_Low, 5, Vol_Low, 1},
			{DOVDD, Vol_1800, 5, Vol_Low, 1},
			{DVDD, Vol_1200, 5, Vol_Low, 1},
			{AVDD, Vol_2800, 5, Vol_Low, 1},
			{AFVDD, Vol_Low, 5, Vol_Low, 1},
			{RST, Vol_High, 5, Vol_Low, 1},
			{PDN, Vol_High, 5, Vol_Low, 1},
			{SensorMCLK, Vol_High, 5, Vol_Low, 1},
		},
	},
#endif
#if defined(F_EWELLY_HI556_MIPI_RAW)
	{
		SENSOR_DRVNAME_F_EWELLY_HI556_MIPI_RAW,
		{
			{PDN, Vol_Low, 5, Vol_Low, 1},
			{RST, Vol_Low, 5, Vol_Low, 1},
			{DOVDD, Vol_1800, 5, Vol_Low, 1},
			{AVDD, Vol_2800, 5, Vol_Low, 1},
			{DVDD, Vol_1200, 5, Vol_Low, 1},
			{SensorMCLK, Vol_High, 5, Vol_Low, 1},
			{PDN, Vol_High, 5, Vol_Low, 1},
			{RST, Vol_High, 5, Vol_Low, 1},
		},
	},
#endif
#if defined(R_EWELLY_HI556_MIPI_RAW)
	{
		SENSOR_DRVNAME_R_EWELLY_HI556_MIPI_RAW,
		{
			{PDN, Vol_Low, 5, Vol_Low, 1},
			{RST, Vol_Low, 5, Vol_Low, 1},
			{DOVDD, Vol_1800, 5, Vol_Low, 1},
			{AVDD, Vol_2800, 5, Vol_Low, 1},
			{DVDD, Vol_1200, 5, Vol_Low, 1},
			{AFVDD, Vol_Low, 5, Vol_Low, 1},
			{SensorMCLK, Vol_High, 5, Vol_Low, 1},
			{PDN, Vol_High, 5, Vol_Low, 1},
			{RST, Vol_High, 5, Vol_Low, 1},
		},
	},
#endif
#if defined(GC5025HMIPI_RAW)
	{
                SENSOR_DRVNAME_GC5025HMIPI_RAW,
                {
                        {RST, Vol_Low, 1, Vol_Low, 1},
                        {PDN, Vol_Low, 1, Vol_Low, 1},
                        {DOVDD, Vol_1800, 1, Vol_Low, 1},
                        {DVDD, Vol_1200, 1, Vol_Low, 1},
                        {AVDD, Vol_2800, 1, Vol_Low, 1},
                        {SensorMCLK, Vol_High, 2, Vol_Low, 1},
                        {PDN, Vol_High, 1, Vol_Low, 1},
                        {RST, Vol_High, 1, Vol_Low, 1},
                        {PDN, Vol_High, 1, Vol_Low, 1},
                },
	},
#endif
#if defined(F_SHINETECH_OV5670_MIPI_RAW)
    {
                SENSOR_DRVNAME_F_SHINETECH_OV5670_MIPI_RAW,
                {
                        {SensorMCLK, Vol_High, 0},
                        {PDN, Vol_Low, 1},
                        {RST, Vol_Low, 2},
                        {DOVDD, Vol_1800, 1},
                        {AVDD, Vol_2800, 1},
                        {DVDD, Vol_1200, 2},
                        {PDN, Vol_High, 1},
                        {RST, Vol_High, 5},
                },
    },
#endif
// add by lihao.zhu for applo84gboost_refresh
#if defined(GC5035_MIPI_RAW)
	{
		SENSOR_DRVNAME_TSPPHCP1088_HI1336_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST,   Vol_Low,  1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD,  Vol_1200, 2},
			{SensorMCLK, Vol_High, 2},
			{PDN,   Vol_High, 2},
			{RST,   Vol_High, 5},
		},
	},
#endif
#if defined(SHN9F511B_GC5035_MIPI_RAW)
	{
		SENSOR_DRVNAME_SHN9F511B_GC5035_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 2},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(EWYPC5319_HI556_MIPI_RAW)
	{
		SENSOR_DRVNAME_EWYPC5319_HI556_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 2},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(SHNBA815M_GC08A3_MIPI_RAW)
	{
		SENSOR_DRVNAME_SHNBA815M_GC08A3_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 2},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(SHNBF204C_GC02M1_MIPI_RAW)
	{
		SENSOR_DRVNAME_SHNBF204C_GC02M1_MIPI_RAW,
		{
			{PDN  , Vol_Low , 0},
			{SensorMCLK, Vol_High, 1},
			{DOVDD, Vol_1800, 2},
			{DVDD , Vol_1800, 1},
			{AVDD , Vol_2800, 3},
			{PDN  , Vol_High, 1},
		},
	},
#endif
#if defined(TSPPSNP1082_S5KJN1_MIPI_RAW)
	{
		SENSOR_DRVNAME_TSPPSNP1082_S5KJN1_MIPI_RAW,
		{
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1200, 0},
			{AVDD, Vol_2800, 0},
			{RST, Vol_High, 1},
			{SensorMCLK, Vol_High, 1},
			{AFVDD, Vol_2800, 2},
		},
	},
#endif
#if defined(TSPS8F9074_S5K4H7_MIPI_RAW)
	{
		SENSOR_DRVNAME_TSPS8F9074_S5K4H7_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(CXTVC12511_GC2905_MIPI_RAW)
	{
		SENSOR_DRVNAME_CXTVC12511_GC2905_MIPI_RAW,
		{
			{PDN,Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 3},
			{PDN,Vol_High, 1},
		},
	},
#endif
/*End 20210805 liuhui add for cruze camera*/
#if defined(IMX398_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX398_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1100, 0},
			{AFVDD, Vol_2800, 0},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 1},
		},
	},
#endif
#if defined(OV23850_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV23850_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 2},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(IMX386_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX386_MIPI_RAW,
		{
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(IMX386_MIPI_MONO)
	{
		SENSOR_DRVNAME_IMX386_MIPI_MONO,
		{
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 5},
		},
	},
#endif

#if defined(IMX338_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX338_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2500, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1100, 0},
			{AFVDD, Vol_2800, 0},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(S5K4E6_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K4E6_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2900, 0},
			{DVDD, Vol_1200, 2},
			{AFVDD, Vol_2800, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K3P8SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P8SP_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(S5K2T7SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2T7SP_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(S5K2T7SP_MIPI_RAW_5M)
	{
		SENSOR_DRVNAME_S5K2T7SP_MIPI_RAW_5M,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(IMX230_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX230_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2500, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1100, 0},
			{AFVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 10}
		},
	},
#endif
#if defined(S5K3M2_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3M2_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K3P3SX_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P3SX_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K5E2YA_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K4ECGX_MIPI_YUV)
	{
		SENSOR_DRVNAME_S5K4ECGX_MIPI_YUV,
		{
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(OV16880_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV16880_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(S5K2P7_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2P7_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1000, 1},
			{DOVDD, Vol_1800, 1},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_Low, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(S5K2P8_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2P8_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(IMX258_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX258_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(IMX258_MIPI_MONO)
	{
		SENSOR_DRVNAME_IMX258_MIPI_MONO,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(IMX377_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX377_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(OV8858_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8858_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV8856_MIPI_RAW)
	{SENSOR_DRVNAME_OV8856_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 2},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(S5K2X8_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2X8_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(IMX214_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX214_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX214_MIPI_MONO)
	{
		SENSOR_DRVNAME_IMX214_MIPI_MONO,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX230_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX230_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 10},
			{DOVDD, Vol_1800, 10},
			{DVDD, Vol_1200, 10},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K3L8_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3L8_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K5E8YX_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K5E8YX_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2}
		},
	},
#endif

#if defined(IMX362_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX362_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 10},
			{DOVDD, Vol_1800, 10},
			{DVDD, Vol_1200, 10},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K2L7_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2L7_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 3},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX318_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX318_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 10},
			{DOVDD, Vol_1800, 10},
			{DVDD, Vol_1200, 10},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(OV8865_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8865_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 5},
			{RST, Vol_Low, 5},
			{DOVDD, Vol_1800, 5},
			{AVDD, Vol_2800, 5},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 5},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX219_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX219_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 10},
			{DOVDD, Vol_1800, 10},
			{DVDD, Vol_1000, 10},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K3M3_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3M3_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV5670_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV5670_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 5},
			{RST, Vol_Low, 5},
			{DOVDD, Vol_1800, 5},
			{AVDD, Vol_2800, 5},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 5},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(OV5670_MIPI_RAW_2)
	{
		SENSOR_DRVNAME_OV5670_MIPI_RAW_2,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 5},
			{RST, Vol_Low, 5},
			{DOVDD, Vol_1800, 5},
			{AVDD, Vol_2800, 5},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 5},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(OV20880_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV20880_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1100, 1},
			{RST, Vol_High, 5}
		},
	},
#endif
	/* add new sensor before this line */
	{NULL,},
};
int platform_power_sequence_size=sizeof(platform_power_sequence)/sizeof(platform_power_sequence[0]);
int sensor_power_sequence_size=sizeof(sensor_power_sequence)/sizeof(sensor_power_sequence[0]);
