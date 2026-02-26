/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _MTK_DRM_ARR_H_
#define _MTK_DRM_ARR_H_

enum LFR_MODE {
	LFR_MODE_DISABLE = 0,
	LFR_MODE_STATIC_MODE,
	LFR_MODE_DYNAMIC_MODE,
	LFR_MODE_BOTH_MODE,
	LFR_MODE_NUM
};

enum LFR_TYPE {
	LFR_TYPE_LP_MODE = 0,
	LFR_TYPE_VSYNC_ONLY,
	LFR_TYPE_HSYNC_ONLY,
	LFR_TYPE_BOTH_MODE,
	LFR_TYPE_NUM
};

struct mtk_dsi_lfr_con {
	unsigned int lfr_mode;
	unsigned int lfr_type;
	unsigned int lfr_enable;
	unsigned int lfr_update;
	unsigned int lfr_vse_dis;
	unsigned int lfr_skip_num;
	unsigned int lfr_mask;
};
struct mtk_dsi_lfr_sta {
	unsigned int lfr_skip_count;
	unsigned int lfr_skip_sta;
};

 /*interface with fpsgo*/
typedef void (*FPS_CHG_CALLBACK)(unsigned int new_fps);
int drm_register_fps_chg_callback(
	FPS_CHG_CALLBACK fps_chg_cb);
int drm_unregister_fps_chg_callback(
	FPS_CHG_CALLBACK fps_chg_cb);
/*interface with primary_display*/
void drm_invoke_fps_chg_callbacks(unsigned int new_fps);

#endif
