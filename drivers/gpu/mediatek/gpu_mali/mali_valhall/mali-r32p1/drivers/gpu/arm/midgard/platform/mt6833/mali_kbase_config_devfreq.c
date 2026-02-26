// SPDX-License-Identifier: GPL-2.0
/********************************************************************************
** Copyright (C), 2014-2021, TCL TV Comm Corp., Ltd
** VENDOR_EDIT, All rights reserved.
**
** File: - mali_kbase_config_devfreq.c
** Description:
**     GPU dynamic frequency governor
**
** ------------------------------- Revision History: ---------------------------
** <author>            <date>       <version>      <desc>
** -----------------------------------------------------------------------------
** shu5.zhang@tcl.com    2021-06-25       1.0       add init version.
*******************************************************************************/

#include <linux/ioport.h>
#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config.h>
#include "mtk_gpu_dvfs.h"
#include "mtk_gpufreq.h"
#include "ged_dvfs.h"
#include "platform/mtk_platform_common.h"

//#include "mali_scaling.h"
//#include "mali_clock.h"
//#include "meson_main2.h"

#ifdef CONFIG_DEVFREQ_THERMAL
#include <linux/devfreq_cooling.h>
#endif

#include <trace/events/power.h>
#if (KERNEL_VERSION(3, 13, 0) <= LINUX_VERSION_CODE)
#include <linux/pm_opp.h>
#else
#include <linux/opp.h>
#endif

#include "mali_kbase_devfreq_callback.h"

#ifdef CONFIG_PM_DEVFREQ
#include <tcl/gpu_devfreq.h>
#endif

#include <linux/pm_runtime.h>
#include <backend/gpu/mali_kbase_pm_internal.h>
//#include <backend/gpu/mali_kbase_device_internal.h>
#include <device/mali_kbase_device_internal.h>
#include "mali_kbase_config_platform.h"

// #ifdef VENDOR_EDIT
// shu5.zhang@tcl.com 2021/08/23 add for gmc
#ifdef CONFIG_GPU_GMC_GENERIC
#include "gmc/mali_kbase_gmc.h"
#endif
// #endif /* VENDOR_EDIT */
#include "gt/mali_kbase_gt.h"

#define KHZ 1000 // KHz
#define DEFAULT_POLLING_MS 8

extern void mali_post_init(void);
struct kbase_device;

unsigned long mt6833_freq_table[] = {
	390000000, //415000000, 441000000,
	467000000, //493000000, 519000000,
	545000000, //570000000, 596000000,
	622000000, //648000000, 674000000,
	700000000, //711000000, 722000000,
	733000000, //745000000, 756000000,
	767000000, //778000000, 790000000,
	801000000, //812000000, 823000000,
	835000000, //846000000, 857000000,
	868000000, //880000000, 895000000,
	910000000, //925000000, 940000000,
	955000000
};

#ifdef CONFIG_PM_DEVFREQ
static struct gpu_devfreq_data gpu_devfreq_priv_data = {
	.vsync_hit = 0,
	.cl_boost = 0,
};

static inline void gpu_devfreq_rcu_read_lock(void)
{
#if (KERNEL_VERSION(4, 14, 0) > LINUX_VERSION_CODE)
	rcu_read_lock();
#endif
}

static inline void gpu_devfreq_rcu_read_unlock(void)
{
#if (KERNEL_VERSION(4, 14, 0) > LINUX_VERSION_CODE)
	rcu_read_unlock();
#endif
}

static inline void gpu_devfreq_opp_put(struct dev_pm_opp *opp)
{
#if (KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE)
	dev_pm_opp_put(opp);
#endif
}

static int mali_kbase_devfreq_target(struct device *dev, unsigned long *_freq,
	u32 flags)
{
	struct kbase_device *kbdev = NULL;
	unsigned long old_freq;
	//struct dev_pm_opp *opp = NULL;
	unsigned long freq;
	int freqid;
	int i32MaxLevel;
	int i;

	KBASE_DEBUG_ASSERT(dev != NULL);
	kbdev = (struct kbase_device *)dev->platform_data;
	KBASE_DEBUG_ASSERT(kbdev != NULL);
	old_freq = kbdev->devfreq->previous_freq;
	/*gpu_devfreq_rcu_read_lock();
	opp = devfreq_recommended_opp(dev, _freq, flags);
	if (IS_ERR(opp)) {
		pr_err("[mali]Failed to get Operating Performance Point\n");
		gpu_devfreq_rcu_read_unlock();
		return PTR_ERR(opp);
	}
	freq = dev_pm_opp_get_freq(opp);
	gpu_devfreq_opp_put(opp);
	gpu_devfreq_rcu_read_unlock();
	pr_debug("%d freq=%ld\n",__LINE__,freq);
*/

	i32MaxLevel = (int)(mt_gpufreq_get_dvfs_table_num() - 1);
	freqid = i32MaxLevel;
	for (i = 0; i <= i32MaxLevel; i++) {
		unsigned long gpu_freq;

		gpu_freq = mt_gpufreq_get_freq_by_idx(i) * 1000;

		if (*_freq > gpu_freq) {
			if (i == 0)
				freqid = 0;
			else
				freqid = i-1;
			break;
		}
	}
	freq = mt_gpufreq_get_freq_by_idx(freqid) * 1000;

	if (old_freq == freq)
		goto update_target;

	// pr_err("[mali] set gpu freqency, [%lu->%lu]\n",old_freq, freq);
	mtk_common_gpufreq_commit(freqid);
update_target:
	// pr_err("[mali] same freq\n");
	*_freq = freq;

	return 0;
}

#ifdef CONFIG_MALI_BOUND_REPORT
static bool mali_kbase_bound_report(struct kbase_device *kbdev,
	bool bound_event)
{
	bool out_data = false;
	bool bound_it = false;

	if (kbdev == NULL)
		return false;

	if (bound_event)
		kbdev->bound_report_info.bound_times_in_fifo++;
	if (kfifo_is_full(&kbdev->bound_report_info.bound_fifo)) {
		kfifo_out(&kbdev->bound_report_info.bound_fifo, &out_data, 1);
		if (out_data)
			kbdev->bound_report_info.bound_times_in_fifo--;
	}
	kfifo_in(&kbdev->bound_report_info.bound_fifo, &bound_event, 1);

	/* bound threshold 3 times */
	if (kbdev->bound_report_info.bound_times_in_fifo > 3)
		bound_it = true; // bound detected!!!

	--kbdev->bound_report_info.duration_times;
	/* new bound state is changed */
	if (bound_it != kbdev->bound_report_info.report_bound_flag) {
		/* check the period, make sure the previous flag state last at
		 * least for a while(BOUND_DURATION_TIMES*20 ms)
		 */
		if (kbdev->bound_report_info.duration_times <= 0) {
			kbdev->bound_report_info.report_bound_flag = bound_it;
			kbdev->bound_report_info.duration_times =
				BOUND_DURATION_TIMES;
		}
	}

	return kbdev->bound_report_info.report_bound_flag;
}
#endif

#ifdef CONFIG_DEVFREQ_THERMAL
void mali_kbase_devfreq_detect_bound_worker(struct work_struct *work)
{
//	int err = 0;
	struct mali_kbase_device_data *temp_data = NULL;
	struct kbase_device *kbdev = NULL;
	//bool bound_event = false;
	struct thermal_cooling_device *cdev = NULL;

	KBASE_DEBUG_ASSERT(work != NULL);
		temp_data = container_of(work,
		struct mali_kbase_device_data, bound_detect_work);
	KBASE_DEBUG_ASSERT(temp_data != NULL);
	kbdev = container_of(temp_data,
		struct kbase_device, mali_dev_data);
	KBASE_DEBUG_ASSERT(kbdev != NULL);

	cdev = kbdev->devfreq_cooling;
#if defined(CONFIG_MALI_MIDGARD_DVFS)
/*	bound_event = kbase_ipa_dynamic_bound_detect(
		kbdev->mali_dev_data.ipa_ctx, &err,
		kbdev->mali_dev_data.bound_detect_freq,
		kbdev->mali_dev_data.bound_detect_btime,
		cdev->ipa_enabled);*/
#endif

	//cdev->ipa_enabled = false;
	//cdev->bound_event = bound_event;

#ifdef CONFIG_MALI_BOUND_REPORT
	kbdev->bound_report_info.report_bound_flag =
		mali_kbase_bound_report(kbdev, bound_event);
#endif
}

static void mali_kbase_devfreq_detect_bound(struct kbase_device *kbdev,
	unsigned long cur_freq, unsigned long btime)
{
	kbdev->mali_dev_data.bound_detect_freq = cur_freq;
	kbdev->mali_dev_data.bound_detect_btime = btime;
	queue_work(system_unbound_wq,
		&kbdev->mali_dev_data.bound_detect_work);
}
#endif

static int mali_kbase_get_dev_status(struct device *dev,
	struct devfreq_dev_status *stat)
{
	struct gpu_devfreq_data *priv_data = &gpu_devfreq_priv_data;
	struct kbase_device *kbdev = NULL;

	KBASE_DEBUG_ASSERT(dev != NULL);
	KBASE_DEBUG_ASSERT(stat != NULL);
	kbdev = (struct kbase_device *)dev->platform_data;
	KBASE_DEBUG_ASSERT(kbdev != NULL);
	if (kbdev->pm.backend.metrics.kbdev != kbdev) {
		pr_err("%s pm backend metrics not initialized\n", __func__);
		return -EINVAL;
	}
	// pr_err("%s %d",__FUNCTION__ ,__LINE__);
	(void)kbase_pm_get_dvfs_action(kbdev);
	stat->busy_time = kbdev->pm.backend.metrics.utilisation;
	stat->total_time = 100; /* base time 100ns */

	stat->current_frequency = mt_gpufreq_get_cur_freq() * 1000;

	// stat->current_frequency = devfreq_get_clk();
	priv_data->vsync_hit = kbdev->pm.backend.metrics.vsync_hit;
	priv_data->cl_boost = kbdev->pm.backend.metrics.cl_boost;
	stat->private_data = (void *)priv_data;

#ifdef CONFIG_DEVFREQ_THERMAL
	/* Avoid sending HWC dump cmd to GPU when GPU is power-off */
	if (kbdev->pm.backend.gpu_powered)
		(void)mali_kbase_devfreq_detect_bound(kbdev,
			stat->current_frequency, stat->busy_time);

#if KERNEL_VERSION(4, 1, 15) <= LINUX_VERSION_CODE
	memcpy(&kbdev->devfreq->last_status, stat, sizeof(*stat)); /* unsafe_function_ignore: memcpy */
#else
	memcpy(&kbdev->devfreq_cooling->last_status, stat, sizeof(*stat)); /* unsafe_function_ignore: memcpy */
#endif
#endif

	return 0;
}

static struct devfreq_dev_profile mali_kbase_devfreq_profile = {
	/* it would be abnormal to enable devfreq monitor
	 * during initialization.
	 */
	.polling_ms = DEFAULT_POLLING_MS, // STOP_POLLING,
	.target = mali_kbase_devfreq_target,
	.get_dev_status = mali_kbase_get_dev_status,
	.freq_table = mt6833_freq_table,
	.max_state = 12,
};
#endif

#ifdef CONFIG_PM_DEVFREQ
void gpu_devfreq_initial_freq(const struct kbase_device *kbdev)
{
    mali_kbase_devfreq_profile.initial_freq = mt_gpufreq_get_cur_freq() * 1000;
	printk(" %s %d initial_freq = %lu\n", __FUNCTION__, __LINE__,
        mali_kbase_devfreq_profile.initial_freq);
}

void gpu_devfreq_init(struct kbase_device *kbdev)
{
	struct device *dev = kbdev->dev;
	// int opp_count;
/*
	opp_count = dev_pm_opp_get_opp_count(dev);
	if (opp_count <= 0)
		return;
*/
	gpu_devfreq_initial_freq(kbdev);

	// dev_set_name(dev, "gpufreq");
	kbdev->devfreq = devfreq_add_device(dev,
		&mali_kbase_devfreq_profile,
		GPU_DEFAULT_GOVERNOR,
		NULL);
    printk("%s %d\n", __FUNCTION__, __LINE__);
}
#endif

// #ifdef VENDOR_EDIT
// shu5.zhang@tcl.com 2021/08/23 add for gmc
#ifdef CONFIG_GPU_GMC_GENERIC
struct gmc_ops kbase_gmc_ops = {
	.compress_kctx = kbase_gmc_compress,
	.decompress_kctx = kbase_gmc_decompress,
	.meminfo_open = kbase_gmc_meminfo_open,
};
#endif
// #endif /* VENDOR_EDIT */
#ifdef CONFIG_GRAPHICS_GT
struct gt_ops kbase_gt_ops = {
	.get_frametime = kbase_get_frametime,
	.get_gpu_utilisation = kbase_gpu_utilisation,
	.get_gpu_curfreq = kbase_gpu_curfreq,
	.get_gpufreq_device = kbase_get_devfreq,
};
#endif

int kbase_platform_backend_init(struct kbase_device *kbdev)
{
	kbdev->mali_dev_data.callbacks =
		(struct mali_kbase_callbacks *)gpu_get_callbacks();

// #ifdef VENDOR_EDIT
// shu5.zhang@tcl.com 2021/08/23 add for gmc
#ifdef CONFIG_GPU_GMC_GENERIC
	if (gmc_register_device(&kbase_gmc_ops, &kbdev->mali_dev_data.kbase_gmc_device)) {
		/* Failed to initialize GMC. */
		dev_warn(kbdev->dev, "GMC initialization failed.\n");
	}
#endif
// #endif /* VENDOR_EDIT */

	gpu_devfreq_init(kbdev);

	pr_err("%d %s",__LINE__,__FUNCTION__);
#ifdef CONFIG_GRAPHICS_GT
	register_gt_callback(&kbase_gt_ops);
#endif
	return 0;
}