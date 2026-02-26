#ifndef CONFIG_DEVFREQ_H
#define CONFIG_DEVFREQ_H

#include <mali_kbase.h>
// #ifdef VENDOR_EDIT
// shu5.zhang@tcl.com 2021/08/23 add for gmc
#ifdef CONFIG_GPU_GMC_GENERIC
#include "tcl/gmc.h"
#endif
// #endif /* VENDOR_EDIT */

#ifdef CONFIG_PM_DEVFREQ
void gpu_devfreq_initial_freq(const struct kbase_device *kbdev);
void gpu_devfreq_init(struct kbase_device *kbdev);
int kbase_platform_backend_init(struct kbase_device *kbdev);
#endif

struct mali_kbase_device_data {
    struct mali_kbase_callbacks *callbacks;

    /* Add other device data here */
    /* Data about dynamic IPA */
    struct work_struct bound_detect_work;
    unsigned long bound_detect_freq;
    unsigned long bound_detect_btime;
// #ifdef VENDOR_EDIT
// shu5.zhang@tcl.com 2021/08/23 add for gm
#ifdef CONFIG_GPU_GMC_GENERIC
	struct gmc_device kbase_gmc_device;
#endif
// #endif /* VENDOR_EDIT */
};

// #ifdef VENDOR_EDIT
// shu5.zhang@tcl.com 2021/08/23 add for gmc
#ifdef CONFIG_GPU_GMC_GENERIC
/**
 * struct kbase_mali_ctx_data - all platform data in context level.
 */
struct kbase_mali_ctx_data {
	/* Add other context data here */
	bool set_pt_flag;
};
#endif
// #endif /* VENDOR_EDIT */

#endif