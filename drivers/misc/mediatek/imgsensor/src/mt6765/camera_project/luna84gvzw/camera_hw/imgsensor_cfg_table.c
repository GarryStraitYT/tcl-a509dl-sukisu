// SPDX-License-Identifier: GPL-2.0

#include "kd_imgsensor.h"

#include "regulator/regulator.h"
#include "gpio/gpio.h"
/*#include "mt6306/mt6306.h"*/
#include "mclk/mclk.h"

#include "imgsensor_cfg_table.h"

enum IMGSENSOR_RETURN (*hw_open[IMGSENSOR_HW_ID_MAX_NUM])(struct IMGSENSOR_HW_DEVICE **) = {
    imgsensor_hw_regulator_open,
    imgsensor_hw_gpio_open,
    /*imgsensor_hw_mt6306_open,*/
    imgsensor_hw_mclk_open};

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
            {IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_AFVDD},
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
    {IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ platform_power_sequence[] = {
#ifdef MIPI_SWITCH
    {
        PLATFORM_POWER_SEQ_NAME,
        {
            {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN,
             IMGSENSOR_HW_PIN_STATE_LEVEL_0,
             0,
             IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
             0},
            {IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
             IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
             0,
             IMGSENSOR_HW_PIN_STATE_LEVEL_0,
             0},
        },
        IMGSENSOR_SENSOR_IDX_SUB,
    },
    {
        PLATFORM_POWER_SEQ_NAME,
        {
            {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN,
             IMGSENSOR_HW_PIN_STATE_LEVEL_0,
             0,
             IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
             0},
            {IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
             IMGSENSOR_HW_PIN_STATE_LEVEL_0,
             0,
             IMGSENSOR_HW_PIN_STATE_LEVEL_0,
             0},
        },
        IMGSENSOR_SENSOR_IDX_MAIN2,
    },
#endif

    {NULL}};

/* Legacy design */
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence[] = {
// begin modified by tct-hq/yzheng4 in 2022-11-25
#if defined (LUNA84GVZW_HI846_MIPI_RAW)
    {
        SENSOR_DRVNAME_LUNA84GVZW_HI846_MIPI_RAW,
        {
            {AFVDD, Vol_2800, 1},
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1200, 1},
            {AVDD, Vol_2800, 2},
            {SensorMCLK, Vol_High, 2},
            {RST, Vol_High, 2},
        },
    },
#endif
#if defined (LUNA84GVZW_GC05A2_MIPI_RAW)
    {
        SENSOR_DRVNAME_LUNA84GVZW_GC05A2_MIPI_RAW,
        {
            {PDN, Vol_Low, 1},
            {RST, Vol_Low, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1200, 1},
            {AVDD, Vol_2800, 1},
            {PDN, Vol_High, 2},
            {RST, Vol_High, 2},
            {SensorMCLK, Vol_High, 2},
            {VDD_None, Vol_High, 2},
        },
    },
#endif
//jiantaohuang
#if defined (LUNA84GVZW_GC08A3_SENSOR_ID)
    {
        SENSOR_DRVNAME_LUNA84GVZW_GC08A3_MIPI_RAW,
        {
            {AFVDD, Vol_2800, 1},
            {PDN, Vol_Low, 0},
            {RST, Vol_Low, 1},
            {SensorMCLK, Vol_High, 1},
            {DOVDD, Vol_1800, 1},
            {DVDD, Vol_1200, 1},
            {AVDD, Vol_2800, 1},
            {PDN, Vol_High, 1},
            {RST, Vol_High, 2},
        },
    },
#endif

#if defined (LUNA84GVZW_HI556_SENSOR_ID)
    {
        SENSOR_DRVNAME_LUNA84GVZW_HI556_MIPI_RAW,
        {
            {PDN, Vol_Low, 0},
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

//jiantaohuang

// end modified by tct-hq/yzheng4 in 2022-11-25
    /* add new sensor before this line */
    {
        NULL,
    },
};
int platform_power_sequence_size = sizeof(platform_power_sequence) / sizeof(platform_power_sequence[0]);
int sensor_power_sequence_size = sizeof(sensor_power_sequence) / sizeof(sensor_power_sequence[0]);
