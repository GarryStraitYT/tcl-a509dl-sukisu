/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __LCM_I2C_H__
#define __LCM_I2C_H__

#include "lcm_drv.h"
#include "lcm_common.h"


#if defined(MTK_LCM_DEVICE_TREE_SUPPORT)
enum LCM_STATUS lcm_i2c_set_data(char type, const struct LCM_DATA_T2 *t2);
//begin add by zhiquan.wen.hz for xr11451676 on 20211109
#ifdef CONFIG_TCT_FEATURE_PARAM_SEPARATION
enum LCM_STATUS lcm_i2c_set_config_data(char type, const struct LCM_DATA_T2 *t2);
#endif
//end add by zhiquan.wen.hz for xr11451676 on 20211109

#endif
extern struct i2c_client *_lcm_i2c_client;
#endif

