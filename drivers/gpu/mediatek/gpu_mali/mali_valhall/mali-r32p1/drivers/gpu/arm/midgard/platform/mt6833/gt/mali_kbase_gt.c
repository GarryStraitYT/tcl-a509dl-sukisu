// SPDX-License-Identifier: GPL-2.0
/********************************************************************************
** Copyright (C), 2014-2021, TCL TV Comm Corp., Ltd
** VENDOR_EDIT, All rights reserved.
**
** File: - mali_kbase_config_devfreq.c
** Description:
**     GPU Turbo
**
** ------------------------------- Revision History: ---------------------------
** <author>            <date>       <version>      <desc>
** -----------------------------------------------------------------------------
** shu5.zhang@tcl.com    2021-06-25       1.0       add init version.
*******************************************************************************/
#include <mali_kbase.h>
#include "mali_kbase_gt.h"
#include "tcl/gputurbo.h"
//#include "mali_scaling.h"
#include <device/mali_kbase_device.h>
#include "mtk_gpu_dvfs.h"
#include "mtk_gpufreq.h"
#include "ged_dvfs.h"
#include <linux/kernel.h>
#include <linux/device.h>
#include "platform/mtk_platform_common.h"

int gt_thread_tgid;

unsigned int kbase_get_frametime(struct GpuRunParameter *data)
{
	struct list_head *entry = NULL;
	const struct list_head *kbdev_list = NULL;

	gt_thread_tgid = data->tgid;

	kbdev_list = kbase_device_get_list();
	list_for_each(entry, kbdev_list) {
		struct kbase_device *kbdev = NULL;
		struct kbase_context *kctx = NULL;
		struct kbase_context *tmp = NULL;

		kbdev = list_entry(entry, struct kbase_device, entry);

		mutex_lock(&kbdev->kctx_list_lock);
		list_for_each_entry_safe(kctx, tmp, &kbdev->kctx_list, kctx_list_link) {
			// pr_err("%s %d kctx->pid=%d",__FUNCTION__ ,__LINE__,kctx->pid);
			if(kctx->tgid == data->tgid) {
				// pr_err("%s %d pid=%d",__FUNCTION__ ,__LINE__,data->pid);
				data->gpuActiveTime = kctx->gpu_active_time;
				data->gpuFullTime = kctx->frame_time;
			}
		}
		mutex_unlock(&kbdev->kctx_list_lock);
	}

	kbase_device_put_list(kbdev_list);
	return 0;
}

unsigned int kbase_gpu_utilisation(void)
{
	struct list_head *entry = NULL;
	const struct list_head *kbdev_list = NULL;
	int temp;

	kbdev_list = kbase_device_get_list();
	list_for_each(entry, kbdev_list) {
		struct kbase_device *kbdev = NULL;

		// pr_err("%d %s",__LINE__,__FUNCTION__);
		kbdev = list_entry(entry, struct kbase_device, entry);
		temp = kbdev->pm.backend.metrics.utilisation;
	}

	kbase_device_put_list(kbdev_list);
	return temp;
}

unsigned int kbase_gpu_curfreq(void)
{
	return mt_gpufreq_get_cur_freq() * 1000;
}

unsigned int kbase_get_devfreq(struct devfreq **devfreq)
{
	struct list_head *entry = NULL;
	const struct list_head *kbdev_list = NULL;

	kbdev_list = kbase_device_get_list();
	list_for_each(entry, kbdev_list) {
		struct kbase_device *kbdev = NULL;

		pr_err("%d %s",__LINE__,__FUNCTION__);
		kbdev = list_entry(entry, struct kbase_device, entry);
		*devfreq = kbdev->devfreq;
	}
	kbase_device_put_list(kbdev_list);
	return 0;
}
