/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __LCM_COMMON_H__
#define __LCM_COMMON_H__

#include "lcm_drv.h"

#if defined(MTK_LCM_DEVICE_TREE_SUPPORT)
enum LCM_STATUS {
	LCM_STATUS_OK = 0,
	LCM_STATUS_ERROR,
};


void lcm_common_parse_dts(const struct LCM_DTS *DTS,
	unsigned char force_update);
void lcm_common_set_util_funcs(const struct LCM_UTIL_FUNCS *util);
/*begin add by zhiquan.wen.hz for xr11451676 on 20210903*/
void lcm_common_get_params(struct LCM_PARAMS *params);
/*end add by zhiquan.wen.hz for xr11451676 on 20210903*/
void lcm_common_init(void);
void lcm_common_suspend(void);
void lcm_common_resume(void);

#if defined(CONFIG_TCT_FEATURE_PARAM_SEPARATION)
void lcm_common_pre_suspend(void);
void lcm_common_post_suspend(void);
void lcm_common_pre_resume(void);
void lcm_common_post_resume(void);
#endif

void lcm_common_update(unsigned int x, unsigned int y, unsigned int width,
	unsigned int height);
void lcm_common_setbacklight(unsigned int level);
void lcm_common_setbacklight_cmdq(void *handle, unsigned int level);
unsigned int lcm_common_compare_id(void);
unsigned int lcm_common_ata_check(unsigned char *buffer);
#endif

#endif
