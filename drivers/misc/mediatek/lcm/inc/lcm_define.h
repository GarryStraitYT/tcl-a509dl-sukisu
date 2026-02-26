/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _LCM_DEFINE_H
#define _LCM_DEFINE_H

#if defined(MTK_LCM_DEVICE_TREE_SUPPORT)
/* LCM_FUNC */
#define LCM_FUNC_GPIO	1
#define LCM_FUNC_I2C	2
#define LCM_FUNC_UTIL	3
#define LCM_FUNC_CMD	4
/*begin add by zhiquan.wen.hz for xr11451676 on 20210903*/
#define LCM_FUNC_TP	5
/*end add by zhiquan.wen.hz for xr11451676 on 20210903*/

/* LCM_GPIO_TYPE */
#ifdef CONFIG_TCT_FEATURE_PARAM_SEPARATION
typedef enum {
    LCM_GPIO_MODE = 1,
    LCM_GPIO_DIR,
    LCM_GPIO_OUT,
    LCM_GPIO_VDD3V3_EN_MODE,
    LCM_GPIO_VDD3V3_EN_DIR,
    LCM_GPIO_VDD3V3_EN_OUT,
    LCM_GPIO_BIAS_ENN_MODE,
    LCM_GPIO_BIAS_ENN_DIR,
    LCM_GPIO_BIAS_ENN_OUT,
    LCM_GPIO_BIAS_ENP_MODE,
    LCM_GPIO_BIAS_ENP_DIR,
    LCM_GPIO_BIAS_ENP_OUT,
    TP_GPIO_RESET_MODE,
    TP_GPIO_RESET_DIR,
    TP_GPIO_RESET_OUT,
    TP_GPIO_READ_COMPARE,
	LCM_GPIO_MAX
} LCM_GPIO_TYPE;
#else
#define LCM_GPIO_MODE   1
#define LCM_GPIO_DIR    2
#define LCM_GPIO_OUT    3
#endif

/* LCM_GPIO_MODE_DATA */
#define LCM_GPIO_MODE_00	0
#define LCM_GPIO_MODE_01	1
#define LCM_GPIO_MODE_02	2
#define LCM_GPIO_MODE_03	3
#define LCM_GPIO_MODE_04	4
#define LCM_GPIO_MODE_05	5
#define LCM_GPIO_MODE_06	6
#define LCM_GPIO_MODE_07	7
#define MAX_LCM_GPIO_MODE	8

/* LCM_GPIO_DIR_DATA */
#define LCM_GPIO_DIR_IN	0
#define LCM_GPIO_DIR_OUT	1

/* LCM_GPIO_OUT_DATA */
#define LCM_GPIO_OUT_ZERO	0
#define LCM_GPIO_OUT_ONE	1

/* LCM_I2C_TYPE */
#define LCM_I2C_WRITE	1
/*begin add by zhiquan.wen.hz for xr11451676 on 20211108*/
#define LCM_I2C_SET_CONFIG   2
/*end add by zhiquan.wen.hz for xr11451676 on 20211108*/

/* LCM_UTIL_TYPE */
#define LCM_UTIL_RESET	1
#define LCM_UTIL_MDELAY	2
#define LCM_UTIL_UDELAY	3
#define LCM_UTIL_WRITE_CMD_V1	4
#define LCM_UTIL_WRITE_CMD_V2	5
#define LCM_UTIL_READ_CMD_V1	6
#define LCM_UTIL_READ_CMD_V2	7
#define LCM_UTIL_WRITE_CMD_V21	8
#define LCM_UTIL_WRITE_CMD_V22	9
#define LCM_UTIL_WRITE_CMD_V23	10
#define LCM_UTIL_RAR	11
/*begin add by zhiquan.wen.hz for xr11451676 on 20210903*/
#define LCM_UTIL_SET_CMDQ	12
/*end add by zhiquan.wen.hz for xr11451676 on 20210903*/

/* LCM_UTIL_RESET_DATA */
#define LCM_UTIL_RESET_LOW	0
#define LCM_UTIL_RESET_HIGH	1

/* LCM_UTIL_WRITE_CMD_V2_DATA */
#define LCM_UTIL_WRITE_CMD_V2_NULL	0xF9
/*begin add by zhiquan.wen.hz for xr11451676 on 20210903*/
/* LCM_TP_TYPE*/
#define LCM_TP_FUNC_SUSPEND		1
#define LCM_TP_FUNC_RESUME		2
/*end add by zhiquan.wen.hz for xr11451676 on 20210903*/
#endif

#endif				/* _LCM_DEFINE_H */
